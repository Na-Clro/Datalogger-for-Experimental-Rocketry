"""
registro_csv.py
Escritura de CSV en vivo (una fila por muestra recibida, con flush
inmediato para no perder datos si se cuelga algo) y lectura de CSV viejos
para el modo de analisis. Un mismo esquema de columnas sirve tanto para
telemetria en vivo (con rssi/snr) como para datos descargados de la
EEPROM (sin rssi/snr, quedan vacios).
"""
import csv
import os
from typing import List, Optional
from protocolo import Muestra

COLUMNAS = [
    't_pc', 'ms', 'count',
    'altitude_m', 'temp_c',
    'acc_x', 'acc_y', 'acc_z',
    'gyro_x', 'gyro_y', 'gyro_z',
    'status_hex',
    'continuidad', 'grabando', 'armado', 'desplegado', 'en_vuelo', 'error_i2c', 'temp_respaldo', 'memoria_llena',
    'rssi_dbm', 'snr_db',
]


def _muestra_a_fila(m: Muestra) -> dict:
    return {
        't_pc': f'{m.t_pc:.3f}',
        'ms': m.ms,
        'count': m.count,
        'altitude_m': f'{m.altitude:.3f}',
        'temp_c': f'{m.temp_c:.2f}',
        'acc_x': m.acc_x, 'acc_y': m.acc_y, 'acc_z': m.acc_z,
        'gyro_x': m.gyro_x, 'gyro_y': m.gyro_y, 'gyro_z': m.gyro_z,
        'status_hex': f'0x{m.status:02X}',
        'continuidad': int(m.continuidad), 'grabando': int(m.grabando),
        'armado': int(m.armado), 'desplegado': int(m.desplegado),
        'en_vuelo': int(m.en_vuelo), 'error_i2c': int(m.error_i2c),
        'temp_respaldo': int(m.temp_respaldo),
        'memoria_llena': int(m.memoria_llena),
        'rssi_dbm': m.rssi if m.rssi is not None else '',
        'snr_db': f'{m.snr:.1f}' if m.snr is not None else '',
    }


class EscritorCSV:
    """Escribe un CSV en vivo, fila por fila, con flush inmediato."""

    def __init__(self, path: str):
        nuevo = not os.path.exists(path)
        self._f = open(path, 'a', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._f, fieldnames=COLUMNAS)
        if nuevo:
            self._writer.writeheader()
            self._f.flush()

    def escribir(self, m: Muestra):
        self._writer.writerow(_muestra_a_fila(m))
        self._f.flush()

    def cerrar(self):
        try:
            self._f.close()
        except Exception:
            pass


def guardar_csv(path: str, muestras: List[Muestra]):
    """Guarda una lista completa de muestras de una sola vez (para exportar
    una descarga de EEPROM ya terminada)."""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNAS)
        writer.writeheader()
        for m in muestras:
            writer.writerow(_muestra_a_fila(m))


def leer_csv(path: str) -> List[Muestra]:
    """Lee un CSV (en vivo o de descarga) y lo reconstruye como lista de
    Muestra, para el modo de analisis post-vuelo."""
    muestras = []
    with open(path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for fila in reader:
            def _int(v, default=0):
                try:
                    return int(v)
                except (ValueError, TypeError):
                    return default

            def _float(v, default=0.0):
                try:
                    return float(v)
                except (ValueError, TypeError):
                    return default

            status_str = fila.get('status_hex', '0x0')
            try:
                status = int(status_str, 16)
            except ValueError:
                status = 0

            rssi_raw = fila.get('rssi_dbm', '')
            snr_raw = fila.get('snr_db', '')
            rssi: Optional[int] = _int(rssi_raw) if rssi_raw not in ('', None) else None
            snr: Optional[float] = _float(snr_raw) if snr_raw not in ('', None) else None

            m = Muestra(
                ms=_int(fila.get('ms')),
                acc_x=_int(fila.get('acc_x')), acc_y=_int(fila.get('acc_y')), acc_z=_int(fila.get('acc_z')),
                gyro_x=_int(fila.get('gyro_x')), gyro_y=_int(fila.get('gyro_y')), gyro_z=_int(fila.get('gyro_z')),
                altitude=_float(fila.get('altitude_m')),
                temp_c=_float(fila.get('temp_c')),
                count=_int(fila.get('count')),
                status=status,
                rssi=rssi, snr=snr,
                t_pc=_float(fila.get('t_pc')),
            )
            muestras.append(m)
    return muestras
