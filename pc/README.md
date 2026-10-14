# Estación de Control (PC)

Aplicación de escritorio en Python + PySide6 para operar el sistema:
telemetría en vivo desde la estación terrena, armado y grabación, registro
a CSV, descarga directa de la EEPROM del cohete y análisis post-vuelo.

## Instalación

```bash
pip install -r requirements.txt
python main.py
```

Requiere Python 3.9 o superior.

## Pestañas

### En Vivo
Conectarse al puerto de la **estación terrena** (no al cohete).

- **Panel de telemetría:** altitud filtrada y cruda, velocidad vertical,
  aceleración, giro por eje (sin offset), temperatura, RSSI/SNR, paquetes
  recibidos y descartados, ruido de altitud estimado, y los indicadores de
  ARMADO/SEGURO, grabación y estado de vuelo.
- **Comandos:** ARMAR/DESARMAR y GRABAR/DETENER. La estación terrena
  reintenta hasta confirmar; lo que se muestra siempre es el estado real
  que reporta el cohete.
- **Sesión de vuelo:**
  - *Finalizar vuelo y guardar* cierra el CSV, ofrece detener la grabación
    del cohete y abrir el vuelo en Análisis.
  - *Nuevo vuelo* reinicia gráficos, máximos y filtros y empieza un archivo
    `vuelo_AAAAMMDD_HHMMSS.csv` en la misma carpeta.
- **Gráficos acoplables:** altitud (cruda y filtrada), velocidad,
  aceleración, giróscopo, temperatura y vector de aceleración 3D. Se pueden
  reordenar, apilar, desacoplar u ocultar desde "Paneles ▾".

### Descarga del cohete
Conexión **USB directa al cohete**.

- **Descargar EEPROM**, guardar como CSV y abrir en Análisis.
- **Consultar estado** (comando `S`).
- **Recalibrar giróscopo** (comando `G`, con el cohete quieto).
- **Borrar memoria** (`E` + `Y`); si todavía no descargaste en esta sesión
  pide una confirmación adicional.
- **Debug / Telemetría:** consola del banco de pruebas del firmware (`T`)
  con botones para cada test, y telemetría en vivo por USB (opción `9`).
  El test de disparo pide dos confirmaciones, la segunda escribiendo
  `DISPARAR`.

### Análisis
Abre un CSV (en vivo o descargado) y muestra el resumen de vuelo: apogeo,
velocidades máximas de ascenso y descenso, aceleración máxima, velocidad
angular máxima, Mach, presión dinámica, inclinación en la rampa,
despegue/despliegue y paquetes perdidos. Sub-pestañas: Resumen,
Aceleración, Giróscopo, Altitud y Temperatura, Inercial y Derivados.

## Filtrado de datos

- **El CSV guarda siempre los datos crudos.** Los filtros se aplican a lo
  que se muestra y se recalculan al abrir un archivo.
- **Altitud y velocidad vertical:** filtro de Kalman de dos estados
  (altitud, velocidad) que usa la altitud barométrica como medición y la
  aceleración vertical como entrada. La aceleración vertical sale de
  proyectar el acelerómetro sobre la dirección de la gravedad medida en la
  rampa, así que no importa cómo esté montado el MPU6050. Después del
  despliegue del paracaídas el acelerómetro deja de usarse.
- **Ruido del barómetro:** se estima en la rampa y el filtro se ajusta solo.
- **Saltos espurios** de altitud se descartan cuando la dinámica es tranquila.
- **Análisis:** además del filtro hacia adelante se aplica un suavizador
  RTS hacia atrás.
- **Giróscopo:** el firmware resta el offset calibrado al arrancar; la PC
  corrige el remanente con los datos en reposo.
- **Eje de tiempo:** se descartan frames corruptos y se toleran reinicios
  del cohete, así el eje no salta.

## Módulos

| Archivo | Contenido |
|---|---|
| `protocolo.py` | formatos binarios, bits de estado, parser de frames, comandos |
| `filtros.py` | Kalman + RTS, offset del giróscopo, validación de tiempo |
| `analisis.py` | conversiones, resumen de vuelo, Mach, presión dinámica, inercial |
| `registro_csv.py` | escritura y lectura de CSV |
| `enlace_serial.py` | hilo de comunicación con la estación terrena |
| `descarga_directa.py` | hilos de USB directo al cohete (descarga, comandos, consola) |
| `ui_principal.py` | interfaz gráfica |
| `main.py` | punto de entrada |

## Trabajo futuro

Hay botones deshabilitados ("Operaciones remotas por LoRa") reservados para:

1. Descarga de la EEPROM por LoRa a través de la estación terrena.
2. Consulta de estado por LoRa (el comando `0x03` ya está reservado).
3. Borrado remoto, habilitado solo después de una descarga física.

Los tres necesitan cambios en ambos firmwares.
