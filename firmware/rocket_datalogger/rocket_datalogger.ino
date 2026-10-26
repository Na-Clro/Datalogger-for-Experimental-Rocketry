/*
 * =====================================================================
 *  DATALOGGER DE COHETE - ESP32-C3 Supermini
 *  Version: 1.0.1
 *
 *  Sensores: MPU6050 (acelerometro/giroscopo) + BMP280 (presion/temp), I2C
 *  Memoria:  EEPROM 24LC256 por I2C
 *  Radio:    LoRa Ra-01 (SX1278) por SPI
 *  Salida:   MOSFET para e-match (paracaidas)
 *
 *  Funcionamiento:
 *   - Muestreo y logica de vuelo a 20 Hz; grabado en EEPROM a 20 Hz y
 *     telemetria LoRa a 2 Hz.
 *   - El armado del e-match y el grabado en EEPROM requieren un comando
 *     explicito desde tierra (magic byte + checksum).
 *   - La logica de despliegue corre a bordo y no depende del enlace LoRa:
 *     deteccion de despegue, apogeo por descenso barometrico y timer de
 *     respaldo.
 *   - EEPROM con header persistente (sobrevive un reset) y escritura
 *     respetando paginas de 64 bytes.
 *   - Calibracion del offset del giroscopo al arrancar.
 *   - Temperatura redundante para la compensacion del barometro.
 *   - Comandos por USB: D (descargar), S (estado), E (borrar), G
 *     (recalibrar giroscopio), T (banco de pruebas).
 *
 *  AJUSTAR ANTES DE VOLAR (ver PARAMETROS DE VUELO):
 *   - UMBRAL_DESCENSO_M: caida desde el apogeo que dispara el e-match.
 *   - BACKUP_TIMER_MS: tiempo maximo desde el despegue para forzar el
 *     disparo (dale margen, ej. 1.5x el tiempo al apogeo esperado).
 *   - Rango del acelerometro (+/-16 g por defecto) segun el motor.
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

// ----------------------- PINES -----------------------
#define LORA_CS     7
#define LORA_RST    10
#define LORA_DIO0   2
#define EMATCH_PIN  3
#define SENSE_PIN   1   // Continuidad del e-match

// ----------------------- I2C --------------------------
#define I2C_SDA       8
#define I2C_SCL       9
#define MPU_ADDR      0x68
#define BMP_ADDR      0x76
#define EEPROM_ADDR   0x50

// ----------------------- EEPROM ------------------------
// 24LC256 = 32768 bytes. Si usas 24LC512, cambiar a 65536.
#define EEPROM_SIZE           32768UL
#define EEPROM_PAGE_SIZE      64
#define EEPROM_HEADER_ADDR    0
#define EEPROM_HEADER_SIZE    6      // 2 bytes magic + 4 bytes contador
#define EEPROM_MAGIC_HI       0xBE
#define EEPROM_MAGIC_LO       0xEF
#define HEADER_UPDATE_EVERY   20     // actualizar header cada N registros (~1s a 20Hz)
// Se reserva la ULTIMA pagina de la EEPROM exclusivamente para el test de
// autodiagnostico (modo debug), para que nunca se solape con datos de
// vuelo reales. Los registros de vuelo solo pueden ocupar hasta
// EEPROM_USABLE_SIZE, nunca la pagina de prueba.
#define EEPROM_TEST_ADDR      (EEPROM_SIZE - EEPROM_PAGE_SIZE)
#define EEPROM_USABLE_SIZE    (EEPROM_SIZE - EEPROM_PAGE_SIZE)

// ----------------------- LORA --------------------------
#define LORA_FREQ         433E6
#define LORA_SYNC_WORD    0xA5   // cambiar si hay otros equipos LoRa cerca

// ----------------------- SENSIBILIDADES (para el modo debug) ------------
// Deben coincidir con la configuracion real del MPU6050 hecha en setup()
// (ACCEL_CONFIG=0x18 -> +/-16g, GYRO_CONFIG=0x18 -> +/-2000dps). Si
// cambias esos registros, actualiza tambien estos valores.
#define ACC_LSB_POR_G      2048.0f
#define GYRO_LSB_POR_DPS   16.4f

// ----------------------- PARAMETROS DE VUELO ------------
// Muestreo/grabado en EEPROM a 20Hz (buena resolucion para el analisis
// posterior), pero TRANSMISION por LoRa mucho mas lenta (2Hz): a SF9 un
// paquete de 27 bytes tarda ~226ms en el aire, asi que transmitir mas
// seguido que eso deja al cohete casi sin ventana para RECIBIR comandos.
// A 2Hz (500ms) queda un margen amplio (~270ms libres por ciclo).
#define SAMPLE_INTERVAL_MS      50     // 20 Hz: sensores + logica de vuelo + EEPROM
#define TX_INTERVAL_MS          500    // 2 Hz: enlace LoRa (telemetria + comandos)
#define ALT_FILTER_ALPHA       0.25f   // filtro exponencial altitud
#define LIFTOFF_THRESHOLD_M    10.0f   // sobre altitud de plataforma
#define UMBRAL_DESCENSO_M      8.0f    // caida desde el apogeo para disparar (5-10 m)
#define BACKUP_TIMER_MS        20000UL // failsafe: forzar disparo si no
                                        // se detecto apogeo en este tiempo
                                        // desde el despegue (AJUSTAR)
#define EMATCH_FIRE_DURATION_MS 2000UL

// ----------------------- PROTOCOLO DE COMANDOS (uplink) -----
// Paquete de 4 bytes: [MAGIC][CMD][VALOR][CHECKSUM]
#define CMD_MAGIC        0xA5
#define CMD_SET_ARMED    0x01
#define CMD_SET_RECORD   0x02
#define CMD_PING         0x03

// ----------------------- ESTRUCTURA DE TELEMETRIA (27 bytes) --
struct __attribute__((__packed__)) RocketPacket {
    uint32_t ms;
    int16_t  acc[3];
    int16_t  gyro[3];
    float    altitude;      // metros, relativa a la plataforma de lanzamiento
    int16_t  temp_c_x100;   // temperatura x100 (ej: 2345 = 23.45 C)
    uint32_t count;
    uint8_t  status;
    // Bit0: continuidad OK | Bit1: grabando | Bit2: armado
    // Bit3: desplegado      | Bit4: en vuelo | Bit5: error I2C sensores
    // Bit6: temperatura de respaldo en uso (ver TEMPERATURA REDUNDANTE)
    // Bit7: EEPROM llena (el grabado se detuvo)
} data;

// ----------------------- VARIABLES GLOBALES --------------------
uint32_t ee_write_addr = EEPROM_HEADER_SIZE;
uint32_t ee_record_count = 0;
uint32_t ee_records_since_header_update = 0;

unsigned long last_tx = 0;

// ---- Transmision LoRa NO bloqueante ----
// endPacket() bloqueante frena el loop todo el tiempo en el aire (~230 ms a
// SF9), y eso dejaba un hueco de 250 ms sin muestras cada 500 ms. Ahora se
// transmite en modo asincrono y el fin de la transmision se detecta leyendo
// el pin DIO0 (mapeado a TxDone), sin interrupciones.
bool lora_ok = false;
bool lora_tx_en_curso = false;
unsigned long t_tx_inicio = 0;
#define LORA_TX_TIMEOUT_MS   600UL   // margen sobre el tiempo en el aire
#define LORA_TX_MIN_MS        30UL   // no mirar DIO0 antes de esto
void loraTxDoneNoUsado() {}          // solo para que la libreria mapee DIO0 a TxDone
unsigned long last_sample = 0;
bool armado = false;
bool recording_eeprom = false;
bool en_vuelo = false;
bool paracaidas_desplegado = false;
unsigned long t_despegue = 0;
unsigned long t_despliegue = 0;

float altitud_plataforma = 0.0f;   // referencia tomada en tierra al bootear
float altitud_filtrada = 0.0f;
float altitud_max = -9999.0f;
bool  altitud_inicializada = false;

// ---- Coeficientes de calibracion BMP280 ----
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t  t_fine;

// ----------------------- PROTOTIPOS -----------------------
void leerCalibracionBMP280();
bool leerMPU6050();
bool leerBMP280Raw(int32_t &adc_T, int32_t &adc_P);
bool configurarBMP280(bool verbose);
double compensarTemperatura(int32_t adc_T);
double compensarPresion(int32_t adc_P);
void procesarComandoLoRa();
void chequearDespliegue(float alt_actual);
void eepromLeerHeader();
void eepromEscribirHeader();
bool eepromWriteChunk(uint32_t addr, uint8_t* buf, uint8_t len);
bool eepromWrite(uint32_t addr, uint8_t* buf, uint8_t len);
bool eepromWaitReady();
void guardarEnEEPROM();
void descargarMemoriaEEPROM();
void borrarMemoriaEEPROM();
void imprimirEstado();

// ----- Modo debug / banco de pruebas (ver seccion al final del archivo) -----
void modoDebug();
bool testEEPROM();
bool testMPU6050(bool continuo);
bool testBMP280(bool continuo);
bool testContinuidadEMatch();
void testDisparoEMatch();
bool testLoRaTransmitir();
bool testLoRaEscuchar();
void testGlobal();
void testDebugEnVivo();

// =====================================================================
//  TEMPERATURA REDUNDANTE (BMP280 / MPU6050 / valor congelado)
// =====================================================================
// La compensacion de presion del BMP280 necesita la temperatura DEL CHIP
// del BMP (t_fine), no la del aire. Es muy sensible: con coeficientes
// tipicos, 1 C de error en t_fine son ~12 m de altitud. Si el canal de
// temperatura del BMP falla, la mejor estimacion de la temperatura de su
// chip es la del chip del MPU6050 (misma placa, mismo ambiente) corregida
// por un offset. Fuentes, en orden de prioridad:
//   1) BMP280   : normal.
//   2) MPU6050  : temp del MPU filtrada + offset BMP-MPU. El offset se
//                 APRENDE solo mientras ambos sensores andan bien; si el BMP
//                 ya arranco roto, se usa OFFSET_MPU_A_BMP_DEFECTO_C.
//   3) CONGELADA: ultima temperatura valida conocida (si fallan ambos).
// El cambio de fuente es de una sola via durante la sesion (no vuelve al
// BMP aunque se "recupere") para no meter escalones en pleno vuelo, y en
// el momento del cambio se re-ajusta la referencia de altitud para que la
// altitud relativa no salte.
enum FuenteTemp : uint8_t { FUENTE_BMP = 0, FUENTE_MPU = 1, FUENTE_FIJA = 2 };

// ---- Parametros de ajuste ----
const float OFFSET_MPU_A_BMP_DEFECTO_C = 0.0f;  // T_bmp - T_mpu si no se pudo aprender (ver comando S)
const float TEMP_VALIDA_MIN_C  = -40.0f;        // rango operativo del BMP280
const float TEMP_VALIDA_MAX_C  =  85.0f;
const float DIF_MAX_BMP_MPU_C  =  20.0f;        // diferencia maxima creible entre ambos chips
const uint8_t LECTURAS_MALAS_PARA_CAMBIAR = 5;  // consecutivas (a 20 Hz = 0.25 s)
const float ALPHA_TEMP_MPU     = 0.05f;         // filtro de la temp del MPU (tau ~1 s a 20 Hz)
const float ALPHA_OFFSET       = 0.01f;         // aprendizaje lento del offset (tau ~5 s)

// ---- Estado ----
float   mpu_temp_c        = 25.0f;   // ultima lectura cruda del MPU
float   mpu_temp_filtrada = 25.0f;
bool    mpu_temp_valida   = false;
bool    mpu_temp_inicializada = false;
float   offset_mpu_a_bmp  = OFFSET_MPU_A_BMP_DEFECTO_C;
bool    offset_aprendido  = false;
float   ultima_temp_valida_c = 25.0f;
FuenteTemp fuente_temp    = FUENTE_BMP;
uint8_t lecturas_bmp_malas = 0;
// Para el re-ajuste sin escalon al cambiar de fuente:
bool    ajuste_fuente_pendiente = false;
int32_t t_fine_anterior = 0;
const char* nombreFuenteTemp();

// =====================================================================
//  CALIBRACION DEL OFFSET (BIAS) DEL GIROSCOPO
// =====================================================================
// El MPU6050 tiene un offset de cero de fabrica (hasta +/-20 deg/s segun
// datasheet; tipicamente unos pocos deg/s). En reposo deberia leer 0 y
// lee ese offset constante. Al arrancar se promedian muestras con el
// cohete quieto y se resta ese promedio a cada lectura. Si se detecta
// movimiento durante la calibracion se reintenta; si no se logra, queda
// en 0 (sin correccion) y se avisa. Se puede repetir con el comando 'G'.
const int     GYRO_CAL_MUESTRAS      = 200;    // ~1 s (5 ms entre muestras)
const int16_t GYRO_CAL_RANGO_MAX     = 80;     // max-min permitido por eje (~5 deg/s)
int16_t gyro_bias[3] = {0, 0, 0};
bool    gyro_calibrado = false;
bool calibrarGiroscopo(bool verbose);

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);   // I2C fast mode (bajar a 100000 si hay cables largos o pull-ups debiles)

    pinMode(EMATCH_PIN, OUTPUT);
    digitalWrite(EMATCH_PIN, LOW);   // E-match apagado por seguridad
    pinMode(SENSE_PIN, INPUT);

    // ---- LoRa ----
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(LORA_FREQ)) {
        // No colgamos el sistema: seguimos pudiendo grabar en EEPROM
        // aunque no haya radio.
        Serial.println("ERROR: LoRa no responde. Continuo sin radio.");
    } else {
        LoRa.setSpreadingFactor(9);
        LoRa.setTxPower(20);
        LoRa.setSyncWord(LORA_SYNC_WORD);
        LoRa.enableCrc();
        // Registrar un callback de TxDone hace que endPacket(true) mapee DIO0
        // a "TxDone". Se desengancha la interrupcion enseguida: el fin de la
        // transmision se lee por polling del pin, sin codigo SPI en ISR (y
        // sin que una ISR limpie los flags de RxDone antes de parsePacket()).
        LoRa.onTxDone(loraTxDoneNoUsado);
        detachInterrupt(digitalPinToInterrupt(LORA_DIO0));
        LoRa.receive();
        lora_ok = true;
    }

    // ---- MPU6050 ----
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);       // Wake up
    Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A); Wire.write(0x03);       // DLPF ~44Hz, reduce ruido vibracion
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); Wire.write(0x18);       // Gyro +/-2000 dps (rockets con giro)
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); Wire.write(0x18);       // Accel +/-16g (evitar saturar en el burn)
    Wire.endTransmission();
    delay(100);                               // dejar estabilizar el MPU tras configurarlo
    calibrarGiroscopo(true);

    // ---- BMP280 ----
    // Calibracion de fabrica + configuracion verificada (ver
    // configurarBMP280: soft reset, oversampling x16/x2, IIR x8, modo normal).
    leerCalibracionBMP280();
    if (!configurarBMP280(false)) {
        Serial.println("ERROR: no se pudo configurar el BMP280 (revisar conexion I2C).");
    }

    // ---- Pre-calentamiento: leer MPU (temp de respaldo) y BMP varias
    // veces ANTES de tomar la referencia. Si el canal de temperatura del
    // BMP esta roto, el cambio de fuente ocurre aca y la referencia de
    // plataforma ya se toma con la fuente definitiva.
    for (int i = 0; i < 10; i++) {
        leerMPU6050();
        int32_t aT, aP;
        if (leerBMP280Raw(aT, aP)) { compensarTemperatura(aT); compensarPresion(aP); }
        delay(20);
    }
    Serial.print("Fuente de temperatura: "); Serial.println(nombreFuenteTemp());

    // ---- Referencia de altitud en plataforma ----
    // Promediamos varias muestras estando quieto en la rampa.
    float suma = 0;
    int muestras_ok = 0;
    for (int i = 0; i < 20; i++) {
        int32_t adc_T, adc_P;
        leerMPU6050();                             // mantiene viva la temp de respaldo
        if (leerBMP280Raw(adc_T, adc_P)) {
            compensarTemperatura(adc_T);           // fija t_fine
            double p = compensarPresion(adc_P);    // Pa
            if (p > 0) {
                suma += 44330.0f * (1.0f - pow(p / 101325.0, 1.0 / 5.255));
                muestras_ok++;
            }
        }
        delay(20);
    }
    altitud_plataforma = (muestras_ok > 0) ? (suma / muestras_ok) : 0.0f;
    altitud_filtrada = altitud_plataforma;
    altitud_max = 0.0f;

    // ---- EEPROM: recuperar puntero persistente ----
    eepromLeerHeader();

    Serial.println("Setup completo.");
    imprimirEstado();
}

// =====================================================================
void loop() {
    // 1. COMANDOS DESDE TIERRA (armado / grabado). No se escucha mientras
    // se transmite: parsePacket() pasaria el radio a recepcion y cortaria la TX.
    if (lora_ok && !lora_tx_en_curso) procesarComandoLoRa();

    // 2. COMANDOS POR USB (bench / descarga)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'D') {
            descargarMemoriaEEPROM();
        } else if (c == 'S') {
            imprimirEstado();
        } else if (c == 'E') {
            Serial.println("Confirmar borrado de EEPROM: enviar 'Y' en 5s...");
            unsigned long t0 = millis();
            bool confirmado = false;
            while (millis() - t0 < 5000) {
                if (Serial.available() && Serial.read() == 'Y') { confirmado = true; break; }
            }
            if (confirmado) borrarMemoriaEEPROM();
            else Serial.println("Borrado cancelado.");
        } else if (c == 'T') {
            modoDebug();
        } else if (c == 'G') {
            if (en_vuelo || armado) {
                Serial.println("No se recalibra el giroscopio con el sistema armado o en vuelo.");
            } else {
                calibrarGiroscopo(true);
            }
        }
    }

    unsigned long now = millis();

    // 3. ADQUISICION + LOGICA DE VUELO + GRABADO EN EEPROM (20 Hz)
    if (now - last_sample >= SAMPLE_INTERVAL_MS) {
        last_sample = now;
        data.ms = now;
        data.status = 0;

        bool i2c_error = false;

        // --- MPU6050 ---
        if (!leerMPU6050()) i2c_error = true;

        // --- BMP280 -> altitud ---
        // La temperatura se registra siempre que la lectura I2C funciono,
        // aunque la presion compensada resulte invalida.
        int32_t adc_T, adc_P;
        float altitud_actual = altitud_filtrada; // valor previo si falla lectura
        if (leerBMP280Raw(adc_T, adc_P)) {
            double T = compensarTemperatura(adc_T);
            data.temp_c_x100 = (int16_t)(T * 100.0);

            double P = compensarPresion(adc_P);
            if (P > 0) {
                altitud_actual = 44330.0f * (1.0f - pow(P / 101325.0, 1.0 / 5.255));
            } else {
                i2c_error = true;
            }
        } else {
            i2c_error = true;
        }

        // Filtro exponencial para no disparar por ruido barometrico
        altitud_filtrada = ALT_FILTER_ALPHA * altitud_actual + (1.0f - ALT_FILTER_ALPHA) * altitud_filtrada;
        data.altitude = altitud_filtrada - altitud_plataforma;

        // --- Deteccion de despegue / apogeo / descenso ---
        chequearDespliegue(altitud_filtrada);

        // --- Apagar el e-match automaticamente tras el tiempo de disparo ---
        if (paracaidas_desplegado && (millis() - t_despliegue > EMATCH_FIRE_DURATION_MS)) {
            digitalWrite(EMATCH_PIN, LOW);
        }

        // --- Flags de estado ---
        if (digitalRead(SENSE_PIN) == HIGH) data.status |= 0x01;
        if (recording_eeprom)               data.status |= 0x02;
        if (armado)                         data.status |= 0x04;
        if (paracaidas_desplegado)          data.status |= 0x08;
        if (en_vuelo)                       data.status |= 0x10;
        if (i2c_error)                      data.status |= 0x20;
        if (fuente_temp != FUENTE_BMP)      data.status |= 0x40;  // temp de respaldo activa

        data.count++;

        // --- Grabar en EEPROM si esta autorizado (a la misma tasa de muestreo) ---
        if (recording_eeprom) guardarEnEEPROM();
    }

    // 4. TRANSMISION LORA (2 Hz, no bloqueante) - manda la ultima muestra
    // disponible. Mientras esta en el aire el muestreo de 20 Hz sigue.
    if (lora_ok && !lora_tx_en_curso && now - last_tx >= TX_INTERVAL_MS) {
        last_tx = now;
        LoRa.idle();
        if (LoRa.beginPacket()) {
            LoRa.write((uint8_t*)&data, sizeof(data));
            LoRa.endPacket(true);            // asincrono: vuelve enseguida
            lora_tx_en_curso = true;
            t_tx_inicio = now;
        } else {
            LoRa.receive();
        }
    }
    if (lora_tx_en_curso) {
        unsigned long en_aire = millis() - t_tx_inicio;
        bool termino = (en_aire >= LORA_TX_MIN_MS && digitalRead(LORA_DIO0) == HIGH);
        if (termino || en_aire > LORA_TX_TIMEOUT_MS) {
            lora_tx_en_curso = false;
            LoRa.receive();                  // volver a escuchar comandos
        }
    }
}

// =====================================================================
//  LECTURA DE SENSORES
// =====================================================================
bool leerMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU_ADDR, 14) != 14) { mpu_temp_valida = false; return false; }
    for (int i = 0; i < 3; i++) data.acc[i]  = (Wire.read() << 8) | Wire.read();
    { // temp interna del MPU: respaldo si el canal de temperatura del BMP280 esta muerto
        int16_t t_raw = (Wire.read() << 8) | Wire.read();
        mpu_temp_c = t_raw / 340.0f + 36.53f;
        mpu_temp_valida = (mpu_temp_c > TEMP_VALIDA_MIN_C && mpu_temp_c < TEMP_VALIDA_MAX_C);
        if (mpu_temp_valida) {
            if (!mpu_temp_inicializada) { mpu_temp_filtrada = mpu_temp_c; mpu_temp_inicializada = true; }
            else mpu_temp_filtrada += ALPHA_TEMP_MPU * (mpu_temp_c - mpu_temp_filtrada);
        }
    }
    for (int i = 0; i < 3; i++) {
        int16_t crudo = (Wire.read() << 8) | Wire.read();
        int32_t corregido = (int32_t)crudo - gyro_bias[i];
        if (corregido >  32767) corregido =  32767;   // saturar, no desbordar
        if (corregido < -32768) corregido = -32768;
        data.gyro[i] = (int16_t)corregido;
    }
    return true;
}

// Lee SOLO el giroscopio en crudo (sin restar bias), para calibrar.
static bool leerGiroCrudo(int16_t g[3]) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU_ADDR, 6) != 6) return false;
    for (int i = 0; i < 3; i++) g[i] = (Wire.read() << 8) | Wire.read();
    return true;
}

bool calibrarGiroscopo(bool verbose) {
    for (int intento = 1; intento <= 3; intento++) {
        int32_t suma[3] = {0, 0, 0};
        int16_t minv[3] = { 32767,  32767,  32767};
        int16_t maxv[3] = {-32768, -32768, -32768};
        int ok = 0;
        for (int n = 0; n < GYRO_CAL_MUESTRAS; n++) {
            int16_t g[3];
            if (leerGiroCrudo(g)) {
                for (int i = 0; i < 3; i++) {
                    suma[i] += g[i];
                    if (g[i] < minv[i]) minv[i] = g[i];
                    if (g[i] > maxv[i]) maxv[i] = g[i];
                }
                ok++;
            }
            delay(5);
        }
        bool quieto = ok > GYRO_CAL_MUESTRAS / 2;
        for (int i = 0; i < 3 && quieto; i++) {
            if (maxv[i] - minv[i] > GYRO_CAL_RANGO_MAX) quieto = false;
        }
        if (quieto) {
            for (int i = 0; i < 3; i++) gyro_bias[i] = (int16_t)(suma[i] / ok);
            gyro_calibrado = true;
            if (verbose) {
                Serial.print("Giroscopio calibrado. Bias (deg/s) X/Y/Z = ");
                for (int i = 0; i < 3; i++) {
                    Serial.print(gyro_bias[i] / GYRO_LSB_POR_DPS, 2);
                    Serial.print(i < 2 ? " / " : "\n");
                }
            }
            return true;
        }
        if (verbose) {
            Serial.print("Calibracion de giroscopio: movimiento detectado (intento ");
            Serial.print(intento); Serial.println("/3). Dejar el cohete QUIETO...");
        }
        delay(500);
    }
    if (verbose) {
        Serial.println("AVISO: no se pudo calibrar el giroscopio (se movio o fallo I2C).");
        Serial.println("Se mantiene el bias anterior. Repetir con 'G' con el cohete quieto.");
    }
    return false;
}

bool leerBMP280Raw(int32_t &adc_T, int32_t &adc_P) {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF7);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(BMP_ADDR, 6) != 6) return false;
    uint8_t b[6];
    for (int i = 0; i < 6; i++) b[i] = Wire.read();
    adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
    return true;
}

// Lee un registro del BMP280. Devuelve true si la lectura fue exitosa.
static bool bmpLeerReg(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)BMP_ADDR, 1) != 1) return false;
    val = Wire.read();
    return true;
}

static bool bmpEscribirReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// Configura el BMP280 y VERIFICA que la configuracion haya quedado
// (relee ctrl_meas/config). Secuencia: soft reset (0xB6 -> 0xE0), config
// (0xF5) y por ultimo ctrl_meas (0xF4), que es el que arranca la medicion.
// Reintenta hasta 3 veces.
bool configurarBMP280(bool verbose) {
    const uint8_t CONFIG_VAL = 0x0C;  // standby 0.5 ms, filtro IIR x8
    // osrs_t x2, osrs_p x16, modo normal (11) = 010 101 11 = 0x57.
    // Modo "ultra high resolution" del datasheet: conversion max ~43 ms
    // (~23 Hz), alcanza para el muestreo de 20 Hz. El IIR queda en x8 para
    // no meter demasiado retardo en la deteccion de apogeo.
    const uint8_t CTRL_VAL   = 0x57;
    bool ok = false;

    for (int intento = 1; intento <= 3 && !ok; intento++) {
        bmpEscribirReg(0xE0, 0xB6);
        delay(10);  // el datasheet pide ~2 ms tras el reset
        bool ack_cfg  = bmpEscribirReg(0xF5, CONFIG_VAL);
        bool ack_ctrl = bmpEscribirReg(0xF4, CTRL_VAL);
        delay(100);

        uint8_t rb_ctrl = 0xFF, rb_cfg = 0xFF;
        bool l1 = bmpLeerReg(0xF4, rb_ctrl);
        bool l2 = bmpLeerReg(0xF5, rb_cfg);
        ok = l1 && l2 && ack_ctrl && ack_cfg &&
             (rb_ctrl == CTRL_VAL) && ((rb_cfg & 0xFC) == CONFIG_VAL);
    }

    if (verbose) {
        Serial.println(ok ? "BMP280 configurado." : "BMP280: no se pudo verificar la configuracion.");
    }
    return ok;
}

void leerCalibracionBMP280() {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0x88);
    uint8_t err = Wire.endTransmission(false);
    uint8_t recibidos = Wire.requestFrom(BMP_ADDR, 24);

    if (err != 0 || recibidos != 24) {
        Serial.println("ERROR CRITICO: fallo la lectura de calibracion del BMP280");
        Serial.println("(revisar conexion I2C / direccion 0x76 vs 0x77). La altitud");
        Serial.println("y la temperatura NO van a ser confiables hasta resolver esto.");
        // Igual intentamos leer lo que haya en el buffer para no bloquear el
        // resto del arranque, pero el chequeo de dig_P1 de abajo va a avisar
        // de nuevo si los valores resultantes son invalidos.
    }

    uint8_t c[24];
    for (int i = 0; i < 24; i++) c[i] = Wire.available() ? Wire.read() : 0;
    dig_T1 = (c[1] << 8) | c[0];
    dig_T2 = (c[3] << 8) | c[2];
    dig_T3 = (c[5] << 8) | c[4];
    dig_P1 = (c[7] << 8) | c[6];
    dig_P2 = (c[9] << 8) | c[8];
    dig_P3 = (c[11] << 8) | c[10];
    dig_P4 = (c[13] << 8) | c[12];
    dig_P5 = (c[15] << 8) | c[14];
    dig_P6 = (c[17] << 8) | c[16];
    dig_P7 = (c[19] << 8) | c[18];
    dig_P8 = (c[21] << 8) | c[20];
    dig_P9 = (c[23] << 8) | c[22];

    // dig_P1 nunca deberia ser 0 en un sensor real (ver nota en
    // compensarPresion: si da 0, la presion compensada siempre sale 0).
    // Si pasa, es la señal mas clara de que la calibracion no se leyo bien.
    if (dig_P1 == 0) {
        Serial.println("ERROR CRITICO: dig_P1 = 0 -- la calibracion del BMP280 es");
        Serial.println("invalida. La presion/altitud va a quedar siempre en un valor");
        Serial.println("invalido. Revisar conexion I2C del BMP280 y reiniciar.");
    }
}

// Formulas de compensacion estandar del datasheet Bosch BMP280.
// Compensacion "pura": devuelve T del BMP y su t_fine.
static double tempBMPDatasheet(int32_t adc_T, int32_t &tf) {
    double var1, var2;
    var1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
    var2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) *
            (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
    tf = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0;
}

const char* nombreFuenteTemp() {
    switch (fuente_temp) {
        case FUENTE_BMP: return "BMP280";
        case FUENTE_MPU: return "MPU6050 (respaldo)";
        default:         return "CONGELADA (respaldo)";
    }
}

// Pasa a una fuente de respaldo (solo hacia "abajo" en prioridad) y deja
// pedido el re-ajuste de referencia de altitud para compensarPresion().
static void cambiarFuenteTemp(FuenteTemp nueva) {
    if (nueva <= fuente_temp) return;
    t_fine_anterior = t_fine;
    ajuste_fuente_pendiente = true;
    fuente_temp = nueva;
    Serial.print("AVISO: fuente de temperatura -> "); Serial.print(nombreFuenteTemp());
    Serial.print("  (offset MPU->BMP = "); Serial.print(offset_mpu_a_bmp, 2);
    Serial.println(offset_aprendido ? " C, aprendido)" : " C, por defecto)");
}

// Devuelve la temperatura que se usa (y registra), y fija t_fine para
// compensarPresion() segun la fuente activa.
double compensarTemperatura(int32_t adc_T) {
    int32_t tf_bmp;
    double t_bmp = tempBMPDatasheet(adc_T, tf_bmp);
    bool mpu_ok = mpu_temp_valida && mpu_temp_inicializada;

    // ---- Salud del canal de temperatura del BMP ----
    // adc_T == 0 o 0x80000 (valor de reset) nunca son mediciones reales.
    bool bmp_ok = (adc_T != 0) && (adc_T != 0x80000) &&
                  (t_bmp > TEMP_VALIDA_MIN_C) && (t_bmp < TEMP_VALIDA_MAX_C);
    // Discrepancia grande con el MPU: solo se usa como criterio cuando el
    // offset entre chips ya fue aprendido (si no, no hay referencia justa).
    if (bmp_ok && mpu_ok && offset_aprendido &&
        fabs(t_bmp - (mpu_temp_filtrada + offset_mpu_a_bmp)) > DIF_MAX_BMP_MPU_C) {
        bmp_ok = false;
    }

    if (fuente_temp == FUENTE_BMP) {
        if (bmp_ok) {
            lecturas_bmp_malas = 0;
            t_fine = tf_bmp;
            ultima_temp_valida_c = (float)t_bmp;
            // Aprender el offset entre chips mientras ambos andan bien
            if (mpu_ok) {
                float dif = (float)t_bmp - mpu_temp_filtrada;
                if (!offset_aprendido) { offset_mpu_a_bmp = dif; offset_aprendido = true; }
                else offset_mpu_a_bmp += ALPHA_OFFSET * (dif - offset_mpu_a_bmp);
            }
            return t_bmp;
        }
        // Lectura mala: mientras no se confirme la falla, se mantiene el
        // ultimo t_fine bueno (no se toca).
        if (++lecturas_bmp_malas < LECTURAS_MALAS_PARA_CAMBIAR) {
            return ultima_temp_valida_c;
        }
        cambiarFuenteTemp(mpu_ok ? FUENTE_MPU : FUENTE_FIJA);
    }

    if (fuente_temp == FUENTE_MPU && !mpu_ok) {
        cambiarFuenteTemp(FUENTE_FIJA);
    }

    float t_usada;
    if (fuente_temp == FUENTE_MPU) {
        t_usada = mpu_temp_filtrada + offset_mpu_a_bmp;
        ultima_temp_valida_c = t_usada;
    } else {
        t_usada = ultima_temp_valida_c;   // congelada
    }
    t_fine = (int32_t)(t_usada * 5120.0f);
    return t_usada;
}

// Presion compensada segun el datasheet, con el t_fine indicado.
static double presionConTfine(int32_t adc_P, int32_t tf) {
    double var1, var2, p;
    var1 = ((double)tf / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
    if (var1 == 0.0) return 0; // evita division por cero
    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
    return p; // Pa
}

double compensarPresion(int32_t adc_P) {
    double p = presionConTfine(adc_P, t_fine);
    // Si recien se cambio de fuente de temperatura, el nuevo t_fine mueve
    // la presion absoluta. Se corre la referencia (plataforma y filtro) en
    // la misma cantidad para que la altitud RELATIVA siga continua.
    if (ajuste_fuente_pendiente) {
        ajuste_fuente_pendiente = false;
        double p_ant = presionConTfine(adc_P, t_fine_anterior);
        if (p > 0 && p_ant > 0) {
            float salto = 44330.0f * (1.0f - pow(p / 101325.0, 1.0 / 5.255))
                        - 44330.0f * (1.0f - pow(p_ant / 101325.0, 1.0 / 5.255));
            altitud_plataforma += salto;
            altitud_filtrada   += salto;
            Serial.print("Referencia de altitud re-ajustada en "); Serial.print(salto, 2);
            Serial.println(" m por cambio de fuente de temperatura.");
        }
    }
    return p;
}

// =====================================================================
//  LOGICA DE VUELO: despegue, apogeo, descenso, disparo
// =====================================================================
void chequearDespliegue(float alt_filtrada_abs) {
    // Detectar despegue (evita que ruido en tierra dispare la logica)
    if (!en_vuelo && (alt_filtrada_abs - altitud_plataforma) > LIFTOFF_THRESHOLD_M) {
        en_vuelo = true;
        t_despegue = millis();
    }

    if (!en_vuelo) return;

    // Trackear altitud maxima (apogeo) durante el ascenso
    if (alt_filtrada_abs > altitud_max) altitud_max = alt_filtrada_abs;

    if (paracaidas_desplegado) return; // ya disparo, no repetir

    bool disparar = false;

    // Condicion principal: bajo N metros desde el maximo alcanzado
    if (armado && (altitud_max - alt_filtrada_abs) >= UMBRAL_DESCENSO_M) {
        disparar = true;
    }

    // Failsafe: si pasa demasiado tiempo desde el despegue sin disparar
    // (barometro tapado, obstruido, etc.), disparar de todos modos.
    if (armado && (millis() - t_despegue) > BACKUP_TIMER_MS) {
        disparar = true;
    }

    if (disparar) {
        digitalWrite(EMATCH_PIN, HIGH);
        t_despliegue = millis();
        paracaidas_desplegado = true;
        armado = false; // auto-desarmar tras el disparo, seguridad extra
    }
}

// =====================================================================
//  COMANDOS DESDE TIERRA
// =====================================================================
void procesarComandoLoRa() {
    int packetSize = LoRa.parsePacket();
    if (packetSize != 4) {
        // Si llego basura de otro tamano, la descartamos sin procesar.
        return;
    }
    uint8_t buf[4];
    for (int i = 0; i < 4; i++) buf[i] = LoRa.read();

    uint8_t checksum = (buf[0] + buf[1] + buf[2]) & 0xFF;
    if (buf[0] != CMD_MAGIC || buf[3] != checksum) return; // paquete invalido

    uint8_t cmd = buf[1];
    uint8_t val = buf[2];

    Serial.print("CMD recibido: 0x");
    Serial.print(cmd, HEX);
    Serial.print(" val=");
    Serial.println(val);

    if (cmd == CMD_SET_ARMED) {
        if (val == 1) {
            // Interlock de seguridad: no armar sin continuidad del e-match
            if (digitalRead(SENSE_PIN) == HIGH && !paracaidas_desplegado) {
                armado = true;
            } else {
                Serial.println("ARMADO RECHAZADO (sin continuidad o ya desplegado)");
            }
        } else {
            armado = false;
        }
    } else if (cmd == CMD_SET_RECORD) {
        recording_eeprom = (val == 1);
    } else if (cmd == CMD_PING) {
        // Reservado: se puede usar para forzar un paquete de estado extra.
    }
}

// =====================================================================
//  EEPROM: header persistente + escritura segura por pagina
// =====================================================================
void eepromLeerHeader() {
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(EEPROM_HEADER_ADDR >> 8));
    Wire.write((uint8_t)(EEPROM_HEADER_ADDR & 0xFF));
    if (Wire.endTransmission(false) != 0) { ee_write_addr = EEPROM_HEADER_SIZE; return; }
    if (Wire.requestFrom(EEPROM_ADDR, EEPROM_HEADER_SIZE) != EEPROM_HEADER_SIZE) {
        ee_write_addr = EEPROM_HEADER_SIZE;
        return;
    }
    uint8_t h[EEPROM_HEADER_SIZE];
    for (int i = 0; i < EEPROM_HEADER_SIZE; i++) h[i] = Wire.read();

    if (h[0] == EEPROM_MAGIC_HI && h[1] == EEPROM_MAGIC_LO) {
        ee_record_count = ((uint32_t)h[2] << 24) | ((uint32_t)h[3] << 16) |
                          ((uint32_t)h[4] << 8)  |  (uint32_t)h[5];
        ee_write_addr = EEPROM_HEADER_SIZE + ee_record_count * sizeof(RocketPacket);
        if (ee_write_addr > EEPROM_USABLE_SIZE) ee_write_addr = EEPROM_HEADER_SIZE; // corrupto
    } else {
        // Primera vez que se usa esta EEPROM: inicializar header.
        ee_record_count = 0;
        ee_write_addr = EEPROM_HEADER_SIZE;
        eepromEscribirHeader();
    }
}

void eepromEscribirHeader() {
    uint8_t h[EEPROM_HEADER_SIZE];
    h[0] = EEPROM_MAGIC_HI; h[1] = EEPROM_MAGIC_LO;
    h[2] = (ee_record_count >> 24) & 0xFF;
    h[3] = (ee_record_count >> 16) & 0xFF;
    h[4] = (ee_record_count >> 8)  & 0xFF;
    h[5] =  ee_record_count        & 0xFF;
    eepromWrite(EEPROM_HEADER_ADDR, h, EEPROM_HEADER_SIZE);
}

bool eepromWaitReady() {
    unsigned long t0 = millis();
    while (millis() - t0 < 10) {
        Wire.beginTransmission(EEPROM_ADDR);
        if (Wire.endTransmission() == 0) return true;
    }
    return false;
}

bool eepromWriteChunk(uint32_t addr, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    for (uint8_t i = 0; i < len; i++) Wire.write(buf[i]);
    if (Wire.endTransmission() != 0) return false;
    return eepromWaitReady();
}

// Escritura que respeta el limite de pagina (64 bytes) de las 24LCxxx.
bool eepromWrite(uint32_t addr, uint8_t* buf, uint8_t len) {
    uint16_t espacio_pagina = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
    if (len <= espacio_pagina) {
        return eepromWriteChunk(addr, buf, len);
    }
    // Cruza el limite de pagina: partir en dos escrituras.
    if (!eepromWriteChunk(addr, buf, espacio_pagina)) return false;
    return eepromWriteChunk(addr + espacio_pagina, buf + espacio_pagina, len - espacio_pagina);
}

void guardarEnEEPROM() {
    if (ee_write_addr + sizeof(RocketPacket) > EEPROM_USABLE_SIZE) {
        data.status |= 0x80; // memoria llena
        return;
    }
    if (eepromWrite(ee_write_addr, (uint8_t*)&data, sizeof(RocketPacket))) {
        ee_write_addr += sizeof(RocketPacket);
        ee_record_count++;
        ee_records_since_header_update++;
        if (ee_records_since_header_update >= HEADER_UPDATE_EVERY) {
            eepromEscribirHeader();
            ee_records_since_header_update = 0;
        }
    }
}

void descargarMemoriaEEPROM() {
    Serial.print("INICIO_DESCARGA registros=");
    Serial.println(ee_record_count);
    for (uint32_t i = 0; i < ee_record_count; i++) {
        uint32_t addr = EEPROM_HEADER_SIZE + i * sizeof(RocketPacket);
        Wire.beginTransmission(EEPROM_ADDR);
        Wire.write((uint8_t)(addr >> 8));
        Wire.write((uint8_t)(addr & 0xFF));
        if (Wire.endTransmission(false) != 0) continue;
        if (Wire.requestFrom(EEPROM_ADDR, (int)sizeof(RocketPacket)) != (int)sizeof(RocketPacket)) continue;

        Serial.print("D:");
        for (int b = 0; b < (int)sizeof(RocketPacket); b++) {
            uint8_t v = Wire.read();
            if (v < 0x10) Serial.print("0");
            Serial.print(v, HEX);
        }
        Serial.println();
    }
    Serial.println("FIN_DESCARGA");
}

void borrarMemoriaEEPROM() {
    ee_record_count = 0;
    ee_records_since_header_update = 0;
    ee_write_addr = EEPROM_HEADER_SIZE;
    eepromEscribirHeader();
    Serial.println("EEPROM logicamente borrada (header reseteado).");
}

void imprimirEstado() {
    Serial.println("---- ESTADO ----");
    Serial.print("Armado: "); Serial.println(armado);
    Serial.print("Grabando: "); Serial.println(recording_eeprom);
    Serial.print("En vuelo: "); Serial.println(en_vuelo);
    Serial.print("Desplegado: "); Serial.println(paracaidas_desplegado);
    Serial.print("Registros en EEPROM: "); Serial.println(ee_record_count);
    Serial.print("Altitud plataforma (ref): "); Serial.println(altitud_plataforma);
    Serial.print("Fuente de temperatura: "); Serial.println(nombreFuenteTemp());
    Serial.print("Temp MPU (filtrada): "); Serial.print(mpu_temp_filtrada, 2);
    Serial.print(" C  offset MPU->BMP: "); Serial.print(offset_mpu_a_bmp, 2);
    Serial.println(offset_aprendido ? " C (aprendido)" : " C (por defecto)");
    Serial.print("Bias giroscopio (deg/s) X/Y/Z: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(gyro_bias[i] / GYRO_LSB_POR_DPS, 2);
        Serial.print(i < 2 ? " / " : "");
    }
    Serial.println(gyro_calibrado ? "  (calibrado)" : "  (SIN calibrar)");
    Serial.println("Comandos: D=descargar  S=estado  E=borrar  T=modo debug  G=recalibrar giroscopio");
    Serial.println("-----------------");
}

// =====================================================================
//  MODO DEBUG / BANCO DE PRUEBAS
// =====================================================================
// Se entra con el comando 'T' por USB directo. Mientras el cohete esta
// en este modo, se queda en un bucle bloqueante leyendo un menu por
// Serial: la adquisicion normal, el registro en EEPROM y la telemetria
// LoRa quedan pausados hasta salir ('Q'). Por eso no se permite entrar
// si el sistema esta armado o en vuelo (ver chequeo al inicio).
// =====================================================================

void modoDebug() {
    if (armado || en_vuelo) {
        Serial.println("No se puede entrar en modo debug: el sistema esta armado o en vuelo.");
        Serial.println("Desarma primero (o reinicia el cohete) antes de usar el modo debug.");
        return;
    }

    Serial.println();
    Serial.println("===================================");
    Serial.println(" MODO DEBUG / TEST - Banco de pruebas");
    Serial.println("===================================");
    Serial.println("La adquisicion normal, el registro en EEPROM y la");
    Serial.println("telemetria LoRa quedan PAUSADOS mientras esten aca.");

    bool salir = false;
    while (!salir) {
        Serial.println();
        Serial.println("--- Menu ---");
        Serial.println("1 - Test EEPROM (escritura/lectura de verificacion)");
        Serial.println("2 - Test MPU6050 (acelerometro + giroscopio en vivo, ~10s)");
        Serial.println("3 - Test BMP280 (presion/temperatura en vivo, ~10s)");
        Serial.println("4 - Test continuidad E-Match (seguro, no dispara nada)");
        Serial.println("5 - Test de DISPARO E-Match (PELIGROSO, pide confirmacion)");
        Serial.println("6 - Test de transmision LoRa");
        Serial.println("7 - Test de recepcion LoRa (escucha 8s)");
        Serial.println("8 - TEST GLOBAL (corre 1,2,3,4,6 automaticamente)");
        Serial.println("9 - Telemetria en vivo por USB (para el software de PC)");
        Serial.println("Q - Salir del modo debug");
        Serial.print("> ");

        char opcion = 0;
        while (!opcion) {
            if (Serial.available()) opcion = Serial.read();
        }
        Serial.println(opcion);

        switch (opcion) {
            case '1': testEEPROM(); break;
            case '2': testMPU6050(true); break;
            case '3': testBMP280(true); break;
            case '4': testContinuidadEMatch(); break;
            case '5': testDisparoEMatch(); break;
            case '6': testLoRaTransmitir(); break;
            case '7': testLoRaEscuchar(); break;
            case '8': testGlobal(); break;
            case '9': testDebugEnVivo(); break;
            case 'Q': case 'q':
                salir = true;
                Serial.println("Saliendo del modo debug. Retomando operacion normal.");
                break;
            case '\n': case '\r':
                break; // ignorar Enter sobrante
            default:
                Serial.println("Opcion invalida.");
                break;
        }
    }
}

// --- Test de EEPROM: usa EXCLUSIVAMENTE la ultima pagina (EEPROM_TEST_ADDR),
// reservada para esto y nunca usada por el registro de vuelo, asi que esta
// prueba nunca puede pisar datos de un vuelo ya grabado. ---
bool testEEPROM() {
    Serial.println("--- Test EEPROM ---");

    Wire.beginTransmission(EEPROM_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("FALLO: la EEPROM no responde en el bus I2C (direccion 0x50).");
        return false;
    }
    Serial.println("OK: EEPROM detectada en el bus I2C.");

    uint8_t patron[EEPROM_PAGE_SIZE];
    for (int i = 0; i < EEPROM_PAGE_SIZE; i++) patron[i] = (uint8_t)(i ^ 0xA5);

    unsigned long t0 = millis();
    if (!eepromWrite(EEPROM_TEST_ADDR, patron, EEPROM_PAGE_SIZE)) {
        Serial.println("FALLO: error al escribir la pagina de prueba.");
        return false;
    }
    unsigned long t_escritura = millis() - t0;

    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(EEPROM_TEST_ADDR >> 8));
    Wire.write((uint8_t)(EEPROM_TEST_ADDR & 0xFF));
    if (Wire.endTransmission(false) != 0) {
        Serial.println("FALLO: error de I2C al posicionar el puntero de lectura.");
        return false;
    }
    if (Wire.requestFrom(EEPROM_ADDR, (int)EEPROM_PAGE_SIZE) != (int)EEPROM_PAGE_SIZE) {
        Serial.println("FALLO: no se pudieron leer los 64 bytes de verificacion.");
        return false;
    }

    bool coincide = true;
    for (int i = 0; i < EEPROM_PAGE_SIZE; i++) {
        uint8_t leido = Wire.read();
        if (leido != patron[i]) coincide = false;
    }

    if (!coincide) {
        Serial.println("FALLO: los datos leidos no coinciden con lo escrito.");
        return false;
    }

    Serial.print("OK: escritura/lectura de 64 bytes verificada (");
    Serial.print(t_escritura);
    Serial.println(" ms).");
    Serial.print("Registros de vuelo almacenados: "); Serial.println(ee_record_count);
    Serial.print("Espacio usado: "); Serial.print(ee_write_addr);
    Serial.print(" / "); Serial.print(EEPROM_USABLE_SIZE); Serial.println(" bytes utiles");
    return true;
}

// --- Test de MPU6050: verifica WHO_AM_I y transmite muestras en vivo.
// La magnitud del vector de aceleracion en reposo deberia ser ~1g sin
// importar la orientacion del sensor -- es un buen chequeo rapido. ---
bool testMPU6050(bool continuo) {
    Serial.println("--- Test MPU6050 ---");

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75); // registro WHO_AM_I
    if (Wire.endTransmission(false) != 0) {
        Serial.println("FALLO: el MPU6050 no responde en el bus I2C.");
        return false;
    }
    if (Wire.requestFrom(MPU_ADDR, 1) != 1) {
        Serial.println("FALLO: no se pudo leer WHO_AM_I.");
        return false;
    }
    uint8_t whoami = Wire.read();
    Serial.print("WHO_AM_I = 0x"); Serial.println(whoami, HEX);
    if (whoami != 0x68) {
        Serial.println("ADVERTENCIA: WHO_AM_I distinto de 0x68 (revisar si es un clon/variante).");
    }

    int n = continuo ? 100 : 15;
    int muestras_ok = 0;
    float suma_mag = 0;
    Serial.println("    AX(g)   AY(g)   AZ(g)  |  GX(dps) GY(dps) GZ(dps)   [Q + Enter para cortar]");
    for (int i = 0; i < n; i++) {
        if (Serial.available() && Serial.read() == 'Q') {
            Serial.println("Interrumpido por el usuario.");
            break;
        }
        if (leerMPU6050()) {
            muestras_ok++;
            float ax = data.acc[0] / ACC_LSB_POR_G;
            float ay = data.acc[1] / ACC_LSB_POR_G;
            float az = data.acc[2] / ACC_LSB_POR_G;
            float gx = data.gyro[0] / GYRO_LSB_POR_DPS;
            float gy = data.gyro[1] / GYRO_LSB_POR_DPS;
            float gz = data.gyro[2] / GYRO_LSB_POR_DPS;
            float mag = sqrt(ax * ax + ay * ay + az * az);
            suma_mag += mag;
            Serial.print("  "); Serial.print(ax, 2); Serial.print("   "); Serial.print(ay, 2);
            Serial.print("   "); Serial.print(az, 2);
            Serial.print("   |   "); Serial.print(gx, 1); Serial.print("   "); Serial.print(gy, 1);
            Serial.print("   "); Serial.println(gz, 1);
        } else {
            Serial.println("Error de lectura I2C en esta muestra.");
        }
        delay(100);
    }

    if (muestras_ok == 0) {
        Serial.println("FALLO: no se pudo leer ninguna muestra valida.");
        return false;
    }

    float mag_prom = suma_mag / muestras_ok;
    Serial.print("Magnitud promedio del vector de aceleracion: "); Serial.print(mag_prom, 2); Serial.println(" g");
    bool mag_ok = (mag_prom > 0.7f && mag_prom < 1.3f);
    if (!mag_ok) {
        Serial.println("ADVERTENCIA: se esperaba ~1g en reposo. Revisar montaje/calibracion.");
    }
    bool resultado = (whoami == 0x68) && mag_ok;
    Serial.println(resultado ? "RESULTADO: PASS" : "RESULTADO: FALLO/ADVERTENCIA");
    return resultado;
}

// --- Test de BMP280: verifica el registro de ID del chip y transmite
// presion/temperatura/altitud en vivo. ---
bool testBMP280(bool continuo) {
    Serial.println("--- Test BMP280 ---");

    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xD0); // registro de ID
    if (Wire.endTransmission(false) != 0) {
        Serial.println("FALLO: el BMP280 no responde en el bus I2C.");
        return false;
    }
    if (Wire.requestFrom(BMP_ADDR, 1) != 1) {
        Serial.println("FALLO: no se pudo leer el registro de ID.");
        return false;
    }
    uint8_t chip_id = Wire.read();
    Serial.print("Chip ID = 0x"); Serial.println(chip_id, HEX);
    if (chip_id != 0x58) {
        Serial.println("ADVERTENCIA: ID distinto de 0x58 (esperado para BMP280).");
    }

    int n = continuo ? 100 : 15;
    int muestras_ok = 0;
    float temp_prom = 0;
    Serial.println("  Temp(C)   Altitud_rel(m)   [Q + Enter para cortar]");
    for (int i = 0; i < n; i++) {
        if (Serial.available() && Serial.read() == 'Q') {
            Serial.println("Interrumpido por el usuario.");
            break;
        }
        int32_t adc_T, adc_P;
        if (leerBMP280Raw(adc_T, adc_P)) {
            double T = compensarTemperatura(adc_T);
            double P = compensarPresion(adc_P);
            if (P > 0) {
                float alt_rel = 44330.0f * (1.0f - pow(P / 101325.0, 1.0 / 5.255)) - altitud_plataforma;
                muestras_ok++;
                temp_prom += T;
                Serial.print("   "); Serial.print(T, 2); Serial.print("      "); Serial.println(alt_rel, 2);
            } else {
                Serial.println("Error de compensacion de presion en esta muestra.");
            }
        } else {
            Serial.println("Error de lectura I2C en esta muestra.");
        }
        delay(100);
    }

    if (muestras_ok == 0) {
        Serial.println("FALLO: no se pudo leer ninguna muestra valida.");
        return false;
    }

    temp_prom /= muestras_ok;
    Serial.print("Temperatura promedio: "); Serial.print(temp_prom, 1); Serial.println(" C");
    bool temp_ok = (temp_prom > -20.0f && temp_prom < 60.0f);
    if (!temp_ok) Serial.println("ADVERTENCIA: temperatura fuera de rango plausible.");
    bool resultado = (chip_id == 0x58) && temp_ok;
    Serial.println(resultado ? "RESULTADO: PASS" : "RESULTADO: FALLO/ADVERTENCIA");
    return resultado;
}

// --- Test de continuidad del e-match: SOLO lee el pin de sense, no
// energiza nada. Seguro para correr siempre, incluso con un igniter
// real conectado. ---
bool testContinuidadEMatch() {
    Serial.println("--- Test de continuidad E-Match ---");
    bool ok = (digitalRead(SENSE_PIN) == HIGH);
    Serial.println(ok ? "OK: continuidad detectada." : "FALLO: sin continuidad (circuito abierto o e-match no conectado).");
    return ok;
}

// --- Test de DISPARO del e-match: energiza la salida real por un pulso
// corto. PELIGROSO si hay un igniter real conectado -- lo va a disparar.
// Requiere escribir la palabra exacta "DISPARAR" como confirmacion. ---
void testDisparoEMatch() {
    Serial.println("--- Prueba de DISPARO del E-Match ---");
    Serial.println("ADVERTENCIA: esto energiza la salida del MOSFET por 150 ms.");
    Serial.println("Si hay un igniter real conectado, SE VA A DISPARAR.");
    Serial.println("Para probar solo la salida electronica sin gastar un igniter,");
    Serial.println("desconecta el igniter real y usa una carga de prueba (LED+resistencia).");
    Serial.println();
    Serial.println("Para continuar escribi exactamente: DISPARAR  (y Enter)");
    Serial.println("Tenes 10 segundos. Cualquier otra cosa cancela.");

    String entrada = "";
    unsigned long t0 = millis();
    while (millis() - t0 < 10000) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') break;
            entrada += c;
        }
    }

    if (entrada != "DISPARAR") {
        Serial.println("Cancelado (no se recibio la confirmacion exacta).");
        return;
    }

    Serial.print("Continuidad antes del disparo: ");
    Serial.println(digitalRead(SENSE_PIN) == HIGH ? "OK" : "SIN CONTINUIDAD");
    Serial.println("Disparando por 150 ms...");
    digitalWrite(EMATCH_PIN, HIGH);
    delay(150);
    digitalWrite(EMATCH_PIN, LOW);
    Serial.println("Pulso finalizado, pin en LOW.");
}

// --- Test de transmision LoRa: manda un paquete de prueba con la
// configuracion actual. Para confirmar que se recibe, hay que verlo
// desde la estacion terrena o un sniffer aparte. ---
bool testLoRaTransmitir() {
    Serial.println("--- Test de transmision LoRa ---");
    Serial.print("Frecuencia: "); Serial.print(LORA_FREQ / 1E6); Serial.println(" MHz");
    Serial.print("Sync word: 0x"); Serial.println(LORA_SYNC_WORD, HEX);

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.print("TEST_COHETE");
    int resultado = LoRa.endPacket();
    LoRa.receive();

    if (resultado) {
        Serial.println("OK: paquete de prueba transmitido (verificar recepcion en tierra/sniffer).");
        return true;
    }
    Serial.println("FALLO: LoRa.endPacket() no confirmo la transmision.");
    return false;
}

// --- Test de recepcion LoRa: escucha 8 segundos e informa lo que llegue,
// con RSSI/SNR de cada paquete. Util para verificar el enlace con la
// estacion terrena o un sniffer transmitiendo desde el otro lado. ---
bool testLoRaEscuchar() {
    Serial.println("--- Test de recepcion LoRa (escucha 8s) ---");
    Serial.println("Transmiti algo desde la estacion terrena o el sniffer ahora...");
    LoRa.receive();
    unsigned long t0 = millis();
    int recibidos = 0;
    while (millis() - t0 < 8000) {
        int packetSize = LoRa.parsePacket();
        if (packetSize > 0) {
            recibidos++;
            Serial.print("Paquete recibido: "); Serial.print(packetSize); Serial.print(" bytes, RSSI=");
            Serial.print(LoRa.packetRssi()); Serial.print(" dBm, SNR="); Serial.println(LoRa.packetSnr());
            while (LoRa.available()) LoRa.read();
        }
    }
    Serial.print("Total de paquetes recibidos: "); Serial.println(recibidos);
    return recibidos > 0;
}

// --- Test global: corre las pruebas no destructivas en secuencia y
// muestra un resumen PASS/FALLO. A PROPOSITO no incluye el disparo del
// e-match (opcion 5): esa prueba queda siempre como accion manual
// separada, nunca dentro de una corrida automatica. ---
void testGlobal() {
    Serial.println();
    Serial.println("===== TEST GLOBAL =====");
    Serial.println("(no incluye el disparo del e-match: esa prueba es manual, opcion 5)");
    Serial.println();

    bool r_eeprom = testEEPROM();
    Serial.println();
    bool r_mpu = testMPU6050(false);
    Serial.println();
    bool r_bmp = testBMP280(false);
    Serial.println();
    bool r_ematch = testContinuidadEMatch();
    Serial.println();
    bool r_lora = testLoRaTransmitir();

    Serial.println();
    Serial.println("===== RESUMEN =====");
    Serial.print("EEPROM:              "); Serial.println(r_eeprom ? "PASS" : "FALLO");
    Serial.print("MPU6050:             "); Serial.println(r_mpu ? "PASS" : "FALLO");
    Serial.print("BMP280:              "); Serial.println(r_bmp ? "PASS" : "FALLO");
    Serial.print("Continuidad e-match: "); Serial.println(r_ematch ? "PASS" : "FALLO (revisar conexion)");
    Serial.print("LoRa (TX):           "); Serial.println(r_lora ? "PASS" : "FALLO");
    Serial.println("====================");
}

// --- Telemetria en vivo por USB directo, para el software de PC.
// Usa EXACTAMENTE el mismo formato de trama que ya manda la estacion
// terrena hacia la PC (0xAA 0xBB + LEN + RocketPacket + rssi + snr +
// checksum), con rssi/snr en 0 porque no hay radio de por medio -- asi
// el software de PC puede conectarse a este puerto con el mismo codigo
// que ya usa para la estacion terrena, sin parsers nuevos.
//
// IMPORTANTE (seguridad): esta funcion NO ejecuta chequearDespliegue()
// ni ninguna logica de disparo, y NO graba en EEPROM. Solo lee sensores
// y transmite. Esto es a proposito: no se puede armar desde el modo
// debug (ver el chequeo al entrar a modoDebug), pero si ademas corriera
// la logica de vuelo real aca, un bench test (por ejemplo agitando el
// sensor a mano) podria dejar las variables globales en_vuelo /
// paracaidas_desplegado / altitud_max en un estado incorrecto para un
// vuelo real posterior si no se reinicia el cohete antes de volar.
void testDebugEnVivo() {
    Serial.println("--- Telemetria en vivo por USB (formato estacion terrena->PC) ---");
    Serial.println("Conecta el software de PC a este puerto COM (pestaña de descarga,");
    Serial.println("telemetria en vivo por USB directo). No arma ni graba en EEPROM.");
    Serial.println("Enviar 'Q' para salir.");
    Serial.println();
    delay(500); // dar tiempo a que el usuario lea antes de que empiece el binario

    RocketPacket muestra;
    uint32_t contador_local = 0;
    unsigned long ultimo = 0;

    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'Q' || c == 'q') {
                Serial.println();
                Serial.println("Saliendo de la telemetria en vivo.");
                break;
            }
        }

        unsigned long ahora = millis();
        if (ahora - ultimo < SAMPLE_INTERVAL_MS) continue;
        ultimo = ahora;

        muestra.ms = ahora;
        muestra.status = 0;
        bool i2c_error = false;

        if (leerMPU6050()) {
            muestra.acc[0] = data.acc[0]; muestra.acc[1] = data.acc[1]; muestra.acc[2] = data.acc[2];
            muestra.gyro[0] = data.gyro[0]; muestra.gyro[1] = data.gyro[1]; muestra.gyro[2] = data.gyro[2];
        } else {
            i2c_error = true;
        }

        int32_t adc_T, adc_P;
        float alt_rel = 0.0f;
        int16_t temp_x100 = muestra.temp_c_x100; // conservar el ultimo valor bueno si esta lectura falla
        if (leerBMP280Raw(adc_T, adc_P)) {
            double T = compensarTemperatura(adc_T);
            temp_x100 = (int16_t)(T * 100.0); // independiente de si la presion da bien o mal

            double P = compensarPresion(adc_P);
            if (P > 0) {
                alt_rel = 44330.0f * (1.0f - pow(P / 101325.0, 1.0 / 5.255)) - altitud_plataforma;
            } else {
                i2c_error = true;
            }
        } else {
            i2c_error = true;
        }
        muestra.altitude = alt_rel;
        muestra.temp_c_x100 = temp_x100;

        if (digitalRead(SENSE_PIN) == HIGH) muestra.status |= 0x01;
        // armado/grabando/en_vuelo/desplegado NO se recalculan aca (ver nota
        // de seguridad arriba); se reportan tal cual estan, que en este modo
        // siempre deberian ser false salvo que ya hayan quedado en true por
        // un vuelo real previo sin reiniciar el cohete.
        if (recording_eeprom)      muestra.status |= 0x02;
        if (armado)                muestra.status |= 0x04;
        if (paracaidas_desplegado) muestra.status |= 0x08;
        if (en_vuelo)              muestra.status |= 0x10;
        if (i2c_error)             muestra.status |= 0x20;
        if (fuente_temp != FUENTE_BMP) muestra.status |= 0x40;

        muestra.count = contador_local++;

        int16_t rssi_dummy = 0;
        float snr_dummy = 0.0f;
        uint8_t payload[sizeof(RocketPacket) + sizeof(rssi_dummy) + sizeof(snr_dummy)];
        memcpy(payload, &muestra, sizeof(RocketPacket));
        memcpy(payload + sizeof(RocketPacket), &rssi_dummy, sizeof(rssi_dummy));
        memcpy(payload + sizeof(RocketPacket) + sizeof(rssi_dummy), &snr_dummy, sizeof(snr_dummy));

        uint8_t checksum = 0;
        for (uint8_t b : payload) checksum ^= b;

        Serial.write(0xAA);
        Serial.write(0xBB);
        Serial.write((uint8_t)sizeof(payload));
        Serial.write(payload, sizeof(payload));
        Serial.write(checksum);
    }
}

