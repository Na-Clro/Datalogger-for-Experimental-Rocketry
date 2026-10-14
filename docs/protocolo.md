# Protocolo de comunicación

Los tres programas comparten estos formatos. Si se modifica uno, hay que
actualizar los otros: `RocketPacket` está definido en ambos firmwares y en
`pc/protocolo.py`.

## Paquete de telemetría — `RocketPacket` (27 bytes, little-endian, packed)

| Offset | Campo | Tipo | Descripción |
|---|---|---|---|
| 0 | `ms` | uint32 | `millis()` del cohete |
| 4 | `acc[3]` | int16 ×3 | aceleración X/Y/Z en cuentas (±16 g → 2048 LSB/g) |
| 10 | `gyro[3]` | int16 ×3 | velocidad angular X/Y/Z sin offset (±2000 °/s → 16,4 LSB/(°/s)) |
| 16 | `altitude` | float32 | altitud relativa a la rampa, filtrada (m) |
| 20 | `temp_c_x100` | int16 | temperatura × 100 (°C) |
| 22 | `count` | uint32 | número de muestra (20 Hz) |
| 26 | `status` | uint8 | bits de estado (tabla siguiente) |

En Python: `struct` con formato `<I3h3hfhIB`.

### Bits de `status`

| Bit | Máscara | Significado |
|---|---|---|
| 0 | `0x01` | continuidad del e-match OK |
| 1 | `0x02` | grabando en EEPROM |
| 2 | `0x04` | armado |
| 3 | `0x08` | paracaídas desplegado |
| 4 | `0x10` | en vuelo (despegue detectado) |
| 5 | `0x20` | error de lectura I2C de sensores |
| 6 | `0x40` | temperatura de respaldo en uso (falla del canal de temperatura del BMP280) |
| 7 | `0x80` | EEPROM llena, se dejó de grabar |

## Enlace LoRa

433 MHz, SF9, BW 125 kHz, CRC activado, sync word `0xA5`.

- **Cohete → tierra:** un `RocketPacket` cada 500 ms (~226 ms en el aire).
- **Tierra → cohete:** comando de 4 bytes:

  ```
  [0xA5][CMD][VALOR][CHECKSUM = (0xA5 + CMD + VALOR) & 0xFF]
  ```

  | CMD | Nombre | VALOR |
  |---|---|---|
  | `0x01` | armar | 1 arma, 0 desarma |
  | `0x02` | grabar | 1 inicia, 0 detiene |
  | `0x03` | ping | reservado |

  La estación terrena reenvía el comando cada 370 ms (hasta 6 s o 20
  intentos) hasta que el bit correspondiente de `status` lo confirma.

## Estación terrena → PC (USB)

```
[0xAA][0xBB][LEN=33][RocketPacket (27) | rssi int16 (2) | snr float32 (4)][XOR de los 33 bytes]
```

El cohete usa el mismo frame (con rssi y snr en 0) en la telemetría por
USB del modo de pruebas (`T`, opción `9`).

## PC → estación terrena (USB)

Dos caracteres ASCII: `A1` / `A0` (armar / desarmar), `R1` / `R0`
(iniciar / detener la grabación).

## PC → cohete (USB directo)

| Comando | Acción |
|---|---|
| `D` | descarga la EEPROM (formato abajo) |
| `S` | imprime el estado |
| `E` + `Y` | borra la EEPROM (pide `Y` dentro de 5 s) |
| `G` | recalibra el offset del giróscopo (cohete quieto) |
| `T` | entra al banco de pruebas (no disponible armado o en vuelo) |

Formato de la descarga:

```
INICIO_DESCARGA registros=<N>
D:<54 caracteres hex = 27 bytes de un RocketPacket>
...
FIN_DESCARGA
```

## Mapa de la EEPROM (24LC256, 32 KiB)

| Dirección | Contenido |
|---|---|
| 0–5 | header: `0xBE 0xEF` + cantidad de registros (uint32 big-endian) |
| 6 … | registros `RocketPacket` consecutivos de 27 bytes |
| última página (64 B) | reservada para el test de EEPROM del banco de pruebas |

El header se actualiza cada 20 registros (~1 s), así que tras un reset se
pierde como máximo el último segundo de datos.
