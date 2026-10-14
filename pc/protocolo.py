"""
protocolo.py
Definiciones del protocolo de comunicacion, IDENTICAS a las de los firmwares
(rocket_datalogger.ino y ground_station.ino). Si en algun momento cambia el
struct RocketPacket o el protocolo de comandos de un lado, hay que
replicarlo aca tambien o se rompe el parseo.

Este modulo no depende de PySide6/pyserial a proposito, para poder
testearlo de forma aislada y reutilizarlo tanto en el enlace en vivo como
en la descarga directa por USB.
"""
import struct
from dataclasses import dataclass
from typing import Optional, List

# --- Formato del RocketPacket (27 bytes), igual al struct __attribute__((packed)) ---
# ms(u32), acc[3](i16), gyro[3](i16), altitude(f32), temp_c_x100(i16), count(u32), status(u8)
FORMATO_ROCKET_PACKET = '<I3h3hfhIB'
TAM_ROCKET_PACKET = struct.calcsize(FORMATO_ROCKET_PACKET)
assert TAM_ROCKET_PACKET == 27, f"El formato no matchea el struct del firmware ({TAM_ROCKET_PACKET} != 27)"

# --- Payload que manda la estacion terrena por USB en cada telemetria en vivo ---
# RocketPacket (27) + rssi:int16 (2) + snr:float32 (4) = 33 bytes
FORMATO_PAYLOAD_LIVE = FORMATO_ROCKET_PACKET + 'hf'
TAM_PAYLOAD_LIVE = struct.calcsize(FORMATO_PAYLOAD_LIVE)
assert TAM_PAYLOAD_LIVE == 33

SYNC_1 = 0xAA
SYNC_2 = 0xBB

# --- Bits de status, igual que en el firmware ---
BIT_CONTINUIDAD = 0x01
BIT_GRABANDO    = 0x02
BIT_ARMADO      = 0x04
BIT_DESPLEGADO  = 0x08
BIT_EN_VUELO    = 0x10
BIT_ERROR_I2C   = 0x20
BIT_TEMP_RESPALDO = 0x40  # temperatura de respaldo (MPU6050 o congelada) en uso
BIT_MEMORIA_LLENA = 0x80  # EEPROM llena: el cohete dejo de grabar


@dataclass
class Muestra:
    """Una muestra de telemetria ya decodificada. rssi/snr quedan en None
    cuando el dato viene de una descarga de EEPROM (ahi no existen)."""
    ms: int
    acc_x: int; acc_y: int; acc_z: int
    gyro_x: int; gyro_y: int; gyro_z: int
    altitude: float
    temp_c: float
    count: int
    status: int
    rssi: Optional[int] = None
    snr: Optional[float] = None
    t_pc: float = 0.0  # timestamp de la PC al recibir el dato (epoch, segundos)

    @property
    def continuidad(self) -> bool: return bool(self.status & BIT_CONTINUIDAD)
    @property
    def grabando(self) -> bool: return bool(self.status & BIT_GRABANDO)
    @property
    def armado(self) -> bool: return bool(self.status & BIT_ARMADO)
    @property
    def desplegado(self) -> bool: return bool(self.status & BIT_DESPLEGADO)
    @property
    def en_vuelo(self) -> bool: return bool(self.status & BIT_EN_VUELO)
    @property
    def error_i2c(self) -> bool: return bool(self.status & BIT_ERROR_I2C)
    @property
    def temp_respaldo(self) -> bool: return bool(self.status & BIT_TEMP_RESPALDO)
    @property
    def memoria_llena(self) -> bool: return bool(self.status & BIT_MEMORIA_LLENA)
    @property
    def acc_mag(self) -> float:
        return (self.acc_x ** 2 + self.acc_y ** 2 + self.acc_z ** 2) ** 0.5


def decodificar_payload_live(payload: bytes, t_pc: float) -> Muestra:
    """Decodifica los 33 bytes de un frame de telemetria en vivo (recibido
    de la estacion terrena por USB)."""
    if len(payload) != TAM_PAYLOAD_LIVE:
        raise ValueError(f"Payload de {len(payload)} bytes, se esperaban {TAM_PAYLOAD_LIVE}")
    (ms, ax, ay, az, gx, gy, gz, alt, tempx100, count, status, rssi, snr) = \
        struct.unpack(FORMATO_PAYLOAD_LIVE, payload)
    return Muestra(ms=ms, acc_x=ax, acc_y=ay, acc_z=az, gyro_x=gx, gyro_y=gy, gyro_z=gz,
                    altitude=alt, temp_c=tempx100 / 100.0, count=count, status=status,
                    rssi=rssi, snr=snr, t_pc=t_pc)


def decodificar_registro_eeprom(datos: bytes, t_pc: float) -> Muestra:
    """Decodifica los 27 bytes de un registro descargado de la EEPROM del
    cohete (por USB directo, comando 'D'). No trae rssi/snr."""
    if len(datos) != TAM_ROCKET_PACKET:
        raise ValueError(f"Registro de {len(datos)} bytes, se esperaban {TAM_ROCKET_PACKET}")
    (ms, ax, ay, az, gx, gy, gz, alt, tempx100, count, status) = \
        struct.unpack(FORMATO_ROCKET_PACKET, datos)
    return Muestra(ms=ms, acc_x=ax, acc_y=ay, acc_z=az, gyro_x=gx, gyro_y=gy, gyro_z=gz,
                    altitude=alt, temp_c=tempx100 / 100.0, count=count, status=status,
                    rssi=None, snr=None, t_pc=t_pc)


class ExtractorDeFrames:
    """
    Parser de framing con buffer para la telemetria en vivo que llega por
    el puerto serie de la estacion terrena:

        [0xAA][0xBB][LEN][payload de LEN bytes][checksum XOR]

    Se le van pasando los bytes crudos que llegan (append_bytes) y se le
    pide que extraiga las Muestras completas y validas que encuentre en el
    buffer acumulado. Tolera basura y frames partidos entre lecturas.
    """
    def __init__(self):
        self._buf = bytearray()

    def append_bytes(self, data: bytes):
        self._buf.extend(data)

    def extraer_muestras(self, t_pc_actual: float) -> List[Muestra]:
        muestras: List[Muestra] = []
        while True:
            idx = self._buf.find(bytes([SYNC_1, SYNC_2]))
            if idx == -1:
                # sin sync en el buffer: nos quedamos solo con el ultimo
                # byte (por si es el inicio de un 0xAA cortado a la mitad)
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break

            if idx > 0:
                del self._buf[:idx]  # descartar basura antes del sync

            if len(self._buf) < 3:
                break  # todavia no llego el byte de LEN

            largo = self._buf[2]
            total = 3 + largo + 1  # sync(2) + len(1) + payload + checksum(1)
            if len(self._buf) < total:
                break  # el frame todavia no esta completo, esperar mas datos

            payload = bytes(self._buf[3:3 + largo])
            checksum_recibido = self._buf[3 + largo]

            del self._buf[:total]  # consumir el frame (venga bien o mal)

            checksum_calc = 0
            for b in payload:
                checksum_calc ^= b
            if checksum_calc != checksum_recibido:
                continue  # frame corrupto, seguimos buscando el proximo

            if largo == TAM_PAYLOAD_LIVE:
                try:
                    muestras.append(decodificar_payload_live(payload, t_pc_actual))
                except (struct.error, ValueError):
                    continue
            # si algun dia el firmware agrega otro tipo de frame con otro
            # LEN, cae aca y se ignora sin romper el parser

        return muestras


# --- Comandos hacia la estacion terrena (formato ASCII de 2 bytes) ---
def comando_armado(valor: bool) -> bytes:
    return b'A1' if valor else b'A0'


def comando_grabado(valor: bool) -> bytes:
    return b'R1' if valor else b'R0'


# =====================================================================
# PIN (pendiente): comandos remotos vía LoRa, relayados por la estación
# terrena hacia el cohete. NINGUNO de estos comandos esta implementado
# todavia en ground_station.ino ni en rocket_datalogger.ino -- hoy la
# estacion terrena solo reconoce 'A'/'R' (ver enviar_comando en
# ui_principal.py). Se dejan definidos aca como referencia de protocolo
# para cuando se implemente esta funcionalidad en ambos firmwares:
#   - Descarga de la EEPROM completa a traves del enlace LoRa (hoy solo
#     existe via USB directo al cohete, ver descarga_directa.py).
#   - Consulta de estado remota (reutilizaria CMD_PING=0x03, ya reservado
#     en el protocolo de comandos del cohete, pero sin uso todavia).
#   - Borrado remoto de la EEPROM. Por diseño de seguridad, este comando
#     NO deberia habilitarse en la interfaz hasta que se haya completado
#     una descarga por conexion fisica en la sesion actual (ver el gateo
#     correspondiente en PestanaDescarga, ui_principal.py).
# =====================================================================
def comando_descarga_remota_lora() -> bytes:
    """PIN: sin soporte de firmware todavia."""
    return b'D1'


def comando_estado_remoto_lora() -> bytes:
    """PIN: sin soporte de firmware todavia."""
    return b'S1'


def comando_borrado_remoto_lora() -> bytes:
    """PIN: sin soporte de firmware todavia. Ver nota de seguridad arriba."""
    return b'X1'
