/*
 * =====================================================================
 *  ESTACION TERRENA - Blue Pill STM32F103C8T6
 *  Version: 1.0.0
 *  LoRa Ra-01 (SPI1 hardware) + TFT GC9A01A 1.28" redondo (SPI software)
 *  Boton unico: toque corto = ARMAR/DESARMAR | mantenido 3s = GRABAR/PARAR
 *
 *  IMPORTANTE: este protocolo debe coincidir EXACTO con el firmware
 *  del cohete (rocket_datalogger.ino). Si cambias el struct RocketPacket
 *  o el protocolo de comandos de un lado, hay que replicarlo del otro.
 * =====================================================================
 */

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// --- PINES LORA (SPI1 hardware: SCK=PA5, MISO=PA6, MOSI=PA7) ---
#define LORA_CS    PA4
#define LORA_RST   PB0
#define LORA_DIO0  PB1

// --- PINES TFT (SPI software) ---
#define TFT_CS     PB9
#define TFT_DC     PB10
#define TFT_RST    PA8
#define TFT_MOSI   PB15
#define TFT_SCK    PB13

#define BTN_PIN    PA0   // Boton a GND, usamos pull-up interno

Adafruit_GC9A01A tft = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

// --- LORA ---
#define LORA_FREQ       433E6
#define LORA_SYNC_WORD  0xA5   // DEBE coincidir con el del cohete

// --- PROTOCOLO DE COMANDOS (igual que en el cohete) ---
#define CMD_MAGIC       0xA5
#define CMD_SET_ARMED   0x01
#define CMD_SET_RECORD  0x02
#define CMD_PING        0x03

// --- TIEMPOS DE BOTON ---
#define TAP_MAX_MS      1000UL   // toque corto: hasta 1s -> ARMAR/DESARMAR
#define HOLD_MS         3000UL   // mantenido: 3s -> GRABAR/PARAR
#define LINK_TIMEOUT_MS 3000UL   // sin paquetes en este tiempo -> "SIN SENAL"

// --- REINTENTO DE COMANDOS HASTA CONFIRMACION ---
// Un solo envio tiene pocas chances de caer en la ventana de escucha del
// cohete (que transmite ~226ms de cada 400ms de ciclo). Reintentamos con
// un periodo DESFASADO respecto al ciclo del cohete para barrer distintas
// fases, hasta ver confirmado el bit correspondiente en la telemetria.
#define CMD_REENVIO_MS    370UL
#define CMD_TIMEOUT_MS    6000UL
#define CMD_MAX_INTENTOS  20

// --- ESTRUCTURA DE TELEMETRIA (debe ser IDENTICA a la del cohete, 27 bytes) ---
struct __attribute__((__packed__)) RocketPacket {
    uint32_t ms;
    int16_t  acc[3];
    int16_t  gyro[3];
    float    altitude;
    int16_t  temp_c_x100;
    uint32_t count;
    uint8_t  status;
} incoming;

// --- ESTADO ---
unsigned long t_press_start = 0;
bool boton_estado_anterior = false; // false = suelto
bool accion_larga_disparada = false;
unsigned long last_packet_time = 0;
unsigned long msg_temporal_hasta = 0;
String msg_temporal = "";

// --- Comando pendiente de confirmacion ---
bool cmd_pendiente = false;
uint8_t cmd_pendiente_tipo = 0;
uint8_t cmd_pendiente_valor = 0;
unsigned long cmd_pendiente_inicio = 0;
unsigned long cmd_pendiente_ultimo_envio = 0;
uint8_t cmd_pendiente_intentos = 0;

bool lora_activo = false; // reflejo real del estado configurado en setup()

// --- PROTOTIPOS ---
void enviarComando(uint8_t cmd, uint8_t val);
void solicitarComando(uint8_t tipo, uint8_t valor);
void procesarComandoPendiente();
void procesarBoton();
void dibujarUIBase();
void actualizarUI(bool hayPaqueteNuevo);
void mostrarMensajeTemporal(const char* txt, uint16_t color);

void setup() {
    Serial.begin(115200);
    pinMode(BTN_PIN, INPUT_PULLUP); // ajustar a INPUT_PULLDOWN si tu cableado es a VCC

    tft.begin();
    tft.fillScreen(GC9A01A_BLACK);
    dibujarUIBase();

    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
    lora_activo = LoRa.begin(LORA_FREQ);
    if (lora_activo) {
        LoRa.setSpreadingFactor(9);
        LoRa.setTxPower(20); // verificar limite legal de tu banda/region
        LoRa.setSyncWord(LORA_SYNC_WORD);
        LoRa.enableCrc();
        LoRa.receive();
    } else {
        mostrarMensajeTemporal("ERROR LORA - REINTENTANDO", GC9A01A_RED);
    }
}

void loop() {
    // Reintentar LoRa SOLO si de verdad fallo en setup() (no lo reiniciamos
    // si ya esta andando, porque LoRa.begin() resetea sync word/CRC/SF).
    static unsigned long last_retry = 0;
    if (!lora_activo && millis() - last_retry > 3000) {
        last_retry = millis();
        if (LoRa.begin(LORA_FREQ)) {
            LoRa.setSpreadingFactor(9);
            LoRa.setTxPower(20);
            LoRa.setSyncWord(LORA_SYNC_WORD);
            LoRa.enableCrc();
            LoRa.receive();
            lora_activo = true;
        }
    }

    procesarBoton();
    procesarComandoPendiente();

    // --- COMANDOS DESDE LA PC (software ground_station_pc) ---
    // Formato por USB: 'A'+val , 'R'+val  (ej: "A1" arma, "R0" detiene el grabado)
    if (Serial.available() >= 2) {
        char c = Serial.read();
        char v = Serial.read();
        if (c == 'A') solicitarComando(CMD_SET_ARMED, v == '1' ? 1 : 0);
        else if (c == 'R') solicitarComando(CMD_SET_RECORD, v == '1' ? 1 : 0);
    }

    // --- RECEPCION DE TELEMETRIA ---
    int packetSize = LoRa.parsePacket();
    bool paqueteNuevo = false;
    if (packetSize == sizeof(RocketPacket)) {
        LoRa.readBytes((uint8_t*)&incoming, sizeof(RocketPacket));
        last_packet_time = millis();
        paqueteNuevo = true;

        // --- Chequear si este paquete confirma el comando pendiente ---
        if (cmd_pendiente) {
            uint8_t bit = (cmd_pendiente_tipo == CMD_SET_ARMED) ? 0x04 : 0x02;
            bool estado_actual  = (incoming.status & bit) != 0;
            bool estado_deseado = (cmd_pendiente_valor == 1);
            if (estado_actual == estado_deseado) {
                cmd_pendiente = false;
                const char* etiqueta = (cmd_pendiente_tipo == CMD_SET_ARMED)
                    ? (estado_deseado ? "ARMADO OK" : "SEGURO OK")
                    : (estado_deseado ? "REC ON OK" : "REC OFF OK");
                mostrarMensajeTemporal(etiqueta, GC9A01A_GREEN);
            }
        }

        // --- Enviar a la PC con checksum y calidad de señal ---
        int16_t rssi = (int16_t)LoRa.packetRssi();
        float snr = LoRa.packetSnr();
        uint8_t payload[sizeof(RocketPacket) + sizeof(rssi) + sizeof(snr)];
        memcpy(payload, &incoming, sizeof(RocketPacket));
        memcpy(payload + sizeof(RocketPacket), &rssi, sizeof(rssi));
        memcpy(payload + sizeof(RocketPacket) + sizeof(rssi), &snr, sizeof(snr));

        uint8_t checksum = 0;
        for (uint8_t b : payload) checksum ^= b;

        Serial.write(0xAA);
        Serial.write(0xBB);
        Serial.write((uint8_t)sizeof(payload));
        Serial.write(payload, sizeof(payload));
        Serial.write(checksum);
    }

    actualizarUI(paqueteNuevo);
}

// =====================================================================
void enviarComando(uint8_t cmd, uint8_t val) {
    uint8_t checksum = (CMD_MAGIC + cmd + val) & 0xFF;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write(CMD_MAGIC);
    LoRa.write(cmd);
    LoRa.write(val);
    LoRa.write(checksum);
    LoRa.endPacket();
    LoRa.receive();
}

// Registra un comando como "pendiente" y dispara el primer envio ya mismo.
// procesarComandoPendiente() se encarga de reintentar hasta confirmarlo.
void solicitarComando(uint8_t tipo, uint8_t valor) {
    cmd_pendiente = true;
    cmd_pendiente_tipo = tipo;
    cmd_pendiente_valor = valor;
    cmd_pendiente_inicio = millis();
    cmd_pendiente_intentos = 0;
    cmd_pendiente_ultimo_envio = 0; // fuerza el primer envio inmediato
}

// Reintenta el comando pendiente cada CMD_REENVIO_MS hasta que la telemetria
// confirme el bit deseado (chequeo en loop(), al recibir cada paquete), o
// hasta agotar el tiempo/intentos maximos.
void procesarComandoPendiente() {
    if (!cmd_pendiente) return;
    unsigned long ahora = millis();

    if ((ahora - cmd_pendiente_inicio > CMD_TIMEOUT_MS) || (cmd_pendiente_intentos >= CMD_MAX_INTENTOS)) {
        cmd_pendiente = false;
        mostrarMensajeTemporal("SIN CONFIRMACION", GC9A01A_RED);
        return;
    }

    if (ahora - cmd_pendiente_ultimo_envio >= CMD_REENVIO_MS) {
        enviarComando(cmd_pendiente_tipo, cmd_pendiente_valor);
        cmd_pendiente_ultimo_envio = ahora;
        cmd_pendiente_intentos++;
    }
}

// =====================================================================
//  BOTON: toque corto = armar/desarmar | mantenido 3s = grabar/parar
// =====================================================================
void procesarBoton() {
    bool presionado = (digitalRead(BTN_PIN) == LOW);

    if (presionado && !boton_estado_anterior) {
        // flanco de bajada: inicio de pulsacion
        t_press_start = millis();
        accion_larga_disparada = false;
    }

    if (presionado && !accion_larga_disparada) {
        unsigned long transcurrido = millis() - t_press_start;
        if (transcurrido >= HOLD_MS) {
            bool grabando_actual = (incoming.status & 0x02);
            solicitarComando(CMD_SET_RECORD, grabando_actual ? 0 : 1);
            accion_larga_disparada = true;
        }
    }

    if (!presionado && boton_estado_anterior) {
        // flanco de subida: solto el boton
        unsigned long transcurrido = millis() - t_press_start;
        if (!accion_larga_disparada && transcurrido < TAP_MAX_MS) {
            bool armado_actual = (incoming.status & 0x04);
            solicitarComando(CMD_SET_ARMED, armado_actual ? 0 : 1);
        }
        // Si solto entre TAP_MAX_MS y HOLD_MS: zona muerta, no hace nada
        // (evita toques largos accidentales que no llegan a confirmar grabado)
    }

    boton_estado_anterior = presionado;
}

// =====================================================================
//  INTERFAZ - layout pensado para pantalla circular 240x240
// =====================================================================
// 6 filas distribuidas entre y=40 y y=210 (banda central del circulo,
// donde el ancho de cuerda es mayor). Cada fila se limpia antes de
// escribir, pero SOLO cuando su contenido realmente cambio respecto al
// frame anterior (evita bit-banguear el SPI de la pantalla sin necesidad
// y deja el loop() mas libre para escuchar LoRa y reintentar comandos).
#define FILA_LINK_Y     40
#define FILA_ARMADO_Y   68
#define FILA_ALT_Y      100
#define FILA_REC_Y      142
#define FILA_VUELO_Y    164
#define FILA_INFO_Y     192

// Formatea un float con 1 decimal SIN depender de snprintf("%f"), que en
// STM32 (newlib-nano) suele no imprimir nada si el core no tiene habilitado
// el soporte de floats en printf.
void formatear1decimal(float valor, char* out, size_t outSize) {
    bool negativo = valor < 0;
    float av = negativo ? -valor : valor;
    int entero = (int)av;
    int decimal = (int)((av - entero) * 10.0f + 0.5f);
    if (decimal >= 10) { decimal = 0; entero++; }
    snprintf(out, outSize, "%s%d.%d", negativo ? "-" : "", entero, decimal);
}

void limpiarFila(int16_t y, int16_t alto) {
    tft.fillRect(0, y, 240, alto, GC9A01A_BLACK);
}

// Dibuja texto centrado horizontalmente (x=120) en la fila indicada.
void centrarTexto(const char* txt, int16_t y, uint8_t size, uint16_t color) {
    tft.setTextSize(size);
    tft.setTextColor(color);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(120 - (int16_t)(w / 2), y);
    tft.print(txt);
}

void dibujarUIBase() {
    tft.fillScreen(GC9A01A_BLACK);
}

void actualizarUI(bool hayPaqueteNuevo) {
    // --- Valores previos, para redibujar SOLO lo que cambio ---
    static bool primera = true;
    static bool prev_con_senal, prev_armado, prev_grabando, prev_en_vuelo;
    static bool prev_desplegado, prev_err_i2c, prev_continuidad, prev_mem_llena;
    static float prev_altitude = -999999.0f;

    bool con_senal = (millis() - last_packet_time) < LINK_TIMEOUT_MS && last_packet_time > 0;
    bool armado      = (incoming.status & 0x04);
    bool grabando    = (incoming.status & 0x02);
    bool en_vuelo    = (incoming.status & 0x10);
    bool desplegado  = (incoming.status & 0x08);
    bool err_i2c     = (incoming.status & 0x20);
    bool continuidad = (incoming.status & 0x01);
    bool mem_llena   = (incoming.status & 0x80);
    char buf[28];

    // --- Fila 1: enlace ---
    if (primera || con_senal != prev_con_senal) {
        limpiarFila(FILA_LINK_Y, 18);
        centrarTexto(con_senal ? "SENAL OK" : "SIN SENAL", FILA_LINK_Y, 1,
                     con_senal ? GC9A01A_GREEN : GC9A01A_RED);
        prev_con_senal = con_senal;
    }

    // --- Fila 2: ARMADO/SEGURO (el dato mas critico) ---
    if (primera || armado != prev_armado) {
        limpiarFila(FILA_ARMADO_Y, 24);
        centrarTexto(armado ? "ARMADO" : "SEGURO", FILA_ARMADO_Y, 2,
                     armado ? GC9A01A_RED : GC9A01A_GREEN);
        prev_armado = armado;
    }

    // --- Fila 3: altitud (dato de vuelo principal, cambia casi siempre) ---
    if (primera || fabs(incoming.altitude - prev_altitude) > 0.05f) {
        limpiarFila(FILA_ALT_Y, 34);
        char numbuf[12];
        formatear1decimal(incoming.altitude, numbuf, sizeof(numbuf));
        snprintf(buf, sizeof(buf), "%s m", numbuf);
        centrarTexto(buf, FILA_ALT_Y, 3, GC9A01A_WHITE);
        prev_altitude = incoming.altitude;
    }

    // --- Fila 4: continuidad + grabando (+ error I2C) ---
    if (primera || grabando != prev_grabando || continuidad != prev_continuidad ||
        err_i2c != prev_err_i2c || mem_llena != prev_mem_llena) {
        limpiarFila(FILA_REC_Y, 16);
        snprintf(buf, sizeof(buf), "%s  %s%s%s",
                 continuidad ? "IGN:OK" : "IGN:ERR",
                 grabando ? "REC:ON" : "REC:OFF",
                 mem_llena ? " MEM!" : "",
                 err_i2c ? " I2C!" : "");
        centrarTexto(buf, FILA_REC_Y, 1, (continuidad && !mem_llena) ? GC9A01A_WHITE : GC9A01A_RED);
        prev_grabando = grabando; prev_continuidad = continuidad;
        prev_err_i2c = err_i2c; prev_mem_llena = mem_llena;
    }

    // --- Fila 5: estado de vuelo ---
    if (primera || en_vuelo != prev_en_vuelo || desplegado != prev_desplegado) {
        limpiarFila(FILA_VUELO_Y, 16);
        centrarTexto(desplegado ? "PARACAIDAS OK" : (en_vuelo ? "EN VUELO" : "EN RAMPA"),
                     FILA_VUELO_Y, 1,
                     desplegado ? GC9A01A_MAGENTA : (en_vuelo ? GC9A01A_CYAN : GC9A01A_WHITE));
        prev_en_vuelo = en_vuelo; prev_desplegado = desplegado;
    }

    // --- Fila 6: PKT/RSSI, comando en curso, o mensaje de confirmacion.
    // Esta fila cambia seguido (cuenta de paquetes, reintentos), asi que
    // se redibuja directamente cuando corresponde, sin cache. ---
    if (cmd_pendiente) {
        limpiarFila(FILA_INFO_Y, 16);
        snprintf(buf, sizeof(buf), "ENVIANDO... (%d)", cmd_pendiente_intentos);
        centrarTexto(buf, FILA_INFO_Y, 1, GC9A01A_YELLOW);
    } else if (millis() < msg_temporal_hasta) {
        limpiarFila(FILA_INFO_Y, 16);
        centrarTexto(msg_temporal.c_str(), FILA_INFO_Y, 1, GC9A01A_YELLOW);
    } else if (hayPaqueteNuevo || primera) {
        limpiarFila(FILA_INFO_Y, 16);
        snprintf(buf, sizeof(buf), "PKT:%lu %ddBm", (unsigned long)incoming.count, (int)LoRa.packetRssi());
        centrarTexto(buf, FILA_INFO_Y, 1, GC9A01A_WHITE);
    }

    primera = false;
}

void mostrarMensajeTemporal(const char* txt, uint16_t color) {
    msg_temporal = String(txt);
    msg_temporal_hasta = millis() + 1500;
}
