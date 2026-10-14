# Compilación y carga del firmware

Ambos firmwares se compilan con el **Arduino IDE 2.x**. Cada carpeta de
`firmware/` es un sketch: abrir el `.ino` desde su propia carpeta.

## Librerías (Gestor de librerías del IDE)

| Librería | Autor | Usada en |
|---|---|---|
| LoRa | Sandeep Mistry | cohete y estación terrena |
| Adafruit GFX Library | Adafruit | estación terrena |
| Adafruit GC9A01A | Adafruit | estación terrena |

## Cohete — ESP32-C3 Supermini

1. En *Preferencias → URLs adicionales de gestor de placas* agregar:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. *Gestor de placas* → instalar **esp32** (Espressif Systems).
3. *Herramientas*:
   - Placa: **ESP32C3 Dev Module**
   - **USB CDC On Boot: Enabled** (sin esto no hay puerto serie por USB:
     no funcionan la descarga, el estado ni el modo de pruebas)
4. Compilar y cargar. Monitor serie a 115200 baudios.

## Estación terrena — Blue Pill (STM32F103C8)

1. Agregar la URL del paquete STM32duino:
   `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
2. *Gestor de placas* → instalar **STM32 MCU based boards**.
3. *Herramientas*:
   - Placa: **Generic STM32F1 series**, Board part number: **BluePill F103C8**
   - USB support: **CDC (generic 'Serial' supersede U(S)ART)**, para que la
     PC vea la estación por el USB de la placa
   - Upload method: según tu programador (por ejemplo STM32CubeProgrammer SWD
     con un ST-Link)

### Error `wiring_private.h: No such file or directory`

Algunas versiones de la librería Adafruit GC9A01A incluyen
`wiring_private.h`, que no existe en el core STM32. Editar
`Adafruit_GC9A01A.cpp` en la carpeta de librerías y comentar esa línea:

```cpp
// #include "wiring_private.h"
```
