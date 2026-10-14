# Aviónica para cohetería experimental

Sistema completo de adquisición de datos para cohetes modelo: un **datalogger
a bordo**, una **estación terrena** con enlace LoRa y un **software de PC**
para telemetría en vivo, control y análisis post-vuelo.

```
 ┌──────────────────────┐   LoRa 433 MHz   ┌──────────────────────┐   USB    ┌──────────────────────┐
 │  COHETE              │ ───────────────► │  ESTACIÓN TERRENA    │ ───────► │  PC                  │
 │  ESP32-C3 Supermini  │   telemetría 2Hz │  STM32 Blue Pill     │  frames  │  Python + PySide6    │
 │  MPU6050 · BMP280    │ ◄─────────────── │  Ra-01 · TFT redondo │ ◄─────── │  vivo · CSV · análisis│
 │  24LC256 · e-match   │   comandos       │  botón               │ comandos │                      │
 └──────────────────────┘                  └──────────────────────┘          └──────────────────────┘
          ▲  USB directo: descarga de la EEPROM, estado, banco de pruebas
          └───────────────────────────────────────────────────────────────────────────── PC
```

## Qué hace

**Cohete** (`firmware/rocket_datalogger`)
- Muestrea acelerómetro y giróscopo (MPU6050, ±16 g / ±2000 °/s) y altitud
  barométrica (BMP280) a 20 Hz, y graba en EEPROM a esa misma tasa.
- Transmite telemetría por LoRa a 2 Hz y recibe comandos de armado y grabado.
- Lógica de despliegue del paracaídas **a bordo**, independiente del enlace:
  detección de despegue, disparo por descenso desde el apogeo y timer de
  respaldo. Armar exige continuidad del e-match.
- Calibra el offset del giróscopo al arrancar y tiene temperatura redundante
  para la compensación del barómetro.
- Banco de pruebas por USB (comando `T`): EEPROM, sensores, continuidad y
  disparo del e-match, LoRa TX/RX, test global y telemetría por USB.

**Estación terrena** (`firmware/ground_station`)
- Recibe la telemetría y la reenvía a la PC con RSSI y SNR.
- Pantalla redonda con estado del enlace, armado, altitud, grabación y vuelo.
- Un botón: toque corto arma/desarma, mantenido 3 s inicia/detiene la grabación.
- Reintenta cada comando hasta ver la confirmación en la telemetría.

**PC** (`pc/`)
- **En Vivo:** valores actuales y máximos, gráficos acoplables, vector de
  aceleración 3D, botones de armado/grabado y registro a CSV por vuelo.
- **Descarga del cohete:** descarga de la EEPROM por USB, consulta de estado,
  recalibración del giróscopo, borrado protegido y consola del banco de pruebas.
- **Análisis:** resumen de vuelo (apogeo, velocidad, aceleración, Mach,
  presión dinámica, inclinación en rampa) y gráficos por variable.
- Altitud y velocidad estimadas con un **filtro de Kalman** (barómetro +
  acelerómetro), con suavizador RTS en el análisis.

## Inicio rápido

1. **Hardware:** conexiones en [`docs/hardware.md`](docs/hardware.md).
2. **Firmware:** abrir cada carpeta de `firmware/` en el Arduino IDE y
   seguir la configuración de [`docs/compilacion.md`](docs/compilacion.md).
3. **PC:**
   ```bash
   cd pc
   pip install -r requirements.txt
   python main.py
   ```
   Requiere Python 3.9 o superior. Detalles en [`pc/README.md`](pc/README.md).

Antes de volar, revisá los parámetros de vuelo al principio de
`rocket_datalogger.ino` (`UMBRAL_DESCENSO_M`, `BACKUP_TIMER_MS`, rango del
acelerómetro) y el protocolo en [`docs/protocolo.md`](docs/protocolo.md).

## Seguridad

Este proyecto controla un circuito de ignición. Usalo bajo tu propia
responsabilidad y respetando la normativa local de cohetería y de uso del
espectro radioeléctrico.

- Probá la lógica de disparo con una carga de prueba (LED + resistencia),
  nunca con un igniter real en banco.
- El e-match solo se puede armar con continuidad, y el modo de pruebas no
  se puede abrir con el sistema armado o en vuelo.
- La potencia LoRa está en 20 dBm: verificá el límite legal de tu banda.

## Estructura

```
firmware/
  rocket_datalogger/    firmware del cohete (ESP32-C3)
  ground_station/       firmware de la estación terrena (STM32F103C8)
pc/                     software de PC (Python)
docs/                   hardware, compilación y protocolo
```

## Licencia

MIT — ver [`LICENSE`](LICENSE). © 2026 Daniel Reula Sanabria.
