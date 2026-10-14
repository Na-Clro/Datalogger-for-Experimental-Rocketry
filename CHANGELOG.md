# Cambios

## 1.0.0 — 2026-10-14

Primera versión publicada.

**Cohete**
- Muestreo y grabado a 20 Hz, telemetría LoRa a 2 Hz con comandos de armado y grabado.
- Despliegue a bordo: despegue, descenso desde el apogeo y timer de respaldo.
- EEPROM con header persistente y escritura por página.
- BMP280 en modo de alta resolución con configuración verificada.
- Temperatura redundante (BMP280 → MPU6050 → valor congelado) sin saltos de altitud.
- Calibración del offset del giróscopo al arrancar y comando `G`.
- Banco de pruebas por USB (`T`) con telemetría directa.
- Bit de estado `0x80` para EEPROM llena.

**Estación terrena**
- Reenvío a la PC con RSSI/SNR, reintento de comandos hasta confirmación.
- Interfaz para pantalla redonda; aviso de EEPROM llena.

**PC**
- Telemetría en vivo, comandos, sesiones de vuelo con un CSV por vuelo.
- Descarga de EEPROM, estado, recalibración del giróscopo, borrado protegido, consola de pruebas.
- Filtro de Kalman barómetro + acelerómetro, suavizador RTS, corrección del giróscopo.
- Eje de tiempo robusto ante frames corruptos y reinicios.
