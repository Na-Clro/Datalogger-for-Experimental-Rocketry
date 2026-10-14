# Hardware y conexiones

## Cohete — ESP32-C3 Supermini

| Componente | Señal | Pin ESP32-C3 |
|---|---|---|
| Bus I2C (MPU6050, BMP280, 24LC256) | SDA | GPIO8 |
| | SCL | GPIO9 |
| LoRa Ra-01 (SX1278) | SCK | GPIO4 |
| | MISO | GPIO5 |
| | MOSI | GPIO6 |
| | NSS (CS) | GPIO7 |
| | RST | GPIO10 |
| | DIO0 | GPIO2 |
| E-match | Gate del MOSFET | GPIO3 |
| | Sensado de continuidad | GPIO1 |

Direcciones I2C: MPU6050 `0x68`, BMP280 `0x76` (SDO a GND), EEPROM `0x50`.

Notas:
- **Todo a 3,3 V.** Los módulos BMP280 de 6 pines ("GY-BME/P280") no traen
  regulador: alimentarlos con 5 V los puede dañar.
- En el módulo BMP280, **CSB debe ir a 3,3 V** (si cae a bajo el chip queda
  en modo SPI) y **SDO a GND** (si queda flotante la dirección es indefinida).
- El bus I2C necesita pull-ups (se usaron 5,6 kΩ). El firmware trabaja a
  400 kHz; con cables largos o pull-ups débiles bajar a 100 kHz en `setup()`.
- GPIO2, GPIO8 y GPIO9 son pines de arranque del ESP32-C3: no forzarlos a
  un nivel distinto del de reposo durante el encendido.
- Las SPI por defecto del core ESP32 para el C3 son SCK 4 / MISO 5 / MOSI 6.

### Probar el BMP280 al recibirlo

Hay módulos falsos o de descarte. Con el modo de pruebas (`T`, opción `3`):
el Chip ID debe ser `0x58` y la temperatura cercana a la del ambiente.
Además, al arrancar (comando `S`) la fuente de temperatura debe ser
`BMP280` y no un respaldo.

## Estación terrena — STM32F103C8T6 (Blue Pill)

| Componente | Señal | Pin |
|---|---|---|
| LoRa Ra-01 (SPI1 hardware) | SCK | PA5 |
| | MISO | PA6 |
| | MOSI | PA7 |
| | NSS (CS) | PA4 |
| | RST | PB0 |
| | DIO0 | PB1 |
| TFT GC9A01A 1,28" redondo (SPI por software) | CS | PB9 |
| | DC | PB10 |
| | RST | PA8 |
| | MOSI (SDA) | PB15 |
| | SCK (SCL) | PB13 |
| Botón (a GND, pull-up interno) | | PA0 |

La PC se conecta al **USB de la Blue Pill** (puerto serie CDC).
