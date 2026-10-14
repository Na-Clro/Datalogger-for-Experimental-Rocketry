"""
analisis.py
Calculos de post-procesamiento sobre una lista de Muestra ya ordenada por
tiempo: velocidad vertical (derivada numerica de la altitud), resumen de
vuelo (apogeo, aceleracion maxima, velocidad maxima/minima, deteccion de
despegue segun el bit en_vuelo, perdida de paquetes en telemetria en vivo).
"""
from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from protocolo import Muestra

# El firmware del cohete configura el MPU6050 en +/-16g (registro ACCEL_CONFIG=0x18)
# y +/-2000 dps (registro GYRO_CONFIG=0x18). Si en algun momento cambias esos
# registros en rocket_datalogger.ino, hay que actualizar estas sensibilidades.
ACC_LSB_POR_G = 2048.0
GYRO_LSB_POR_DPS = 16.4
G_A_MS2 = 9.80665


def acc_cuentas_a_g(cuentas: float) -> float:
    return cuentas / ACC_LSB_POR_G


def acc_cuentas_a_ms2(cuentas: float) -> float:
    return acc_cuentas_a_g(cuentas) * G_A_MS2


def gyro_cuentas_a_dps(cuentas: float) -> float:
    return cuentas / GYRO_LSB_POR_DPS


def calcular_magnitud_giro_dps(muestras: List[Muestra]) -> np.ndarray:
    """Velocidad angular total (magnitud del vector giroscopio), en grados/s."""
    gx = np.array([gyro_cuentas_a_dps(m.gyro_x) for m in muestras])
    gy = np.array([gyro_cuentas_a_dps(m.gyro_y) for m in muestras])
    gz = np.array([gyro_cuentas_a_dps(m.gyro_z) for m in muestras])
    return np.sqrt(gx ** 2 + gy ** 2 + gz ** 2)


def angulo_inclinacion_inicial_deg(muestras: List[Muestra]) -> Optional[float]:
    """
    Angulo entre el eje Z (asumido alineado con el eje de empuje) y el
    vector de gravedad medido, promediado sobre las muestras previas al
    despegue. Sirve como chequeo de verticalidad en la rampa; NO es valido
    durante el vuelo (ahi el acelerometro mide fuerza especifica, no solo
    gravedad).
    """
    idx_despegue = next((i for i, m in enumerate(muestras) if m.en_vuelo), len(muestras))
    if idx_despegue < 3:
        return None
    ax = np.mean([m.acc_x for m in muestras[:idx_despegue]])
    ay = np.mean([m.acc_y for m in muestras[:idx_despegue]])
    az = np.mean([m.acc_z for m in muestras[:idx_despegue]])
    mag = (ax ** 2 + ay ** 2 + az ** 2) ** 0.5
    if mag == 0:
        return None
    return float(np.degrees(np.arccos(np.clip(az / mag, -1.0, 1.0))))


def calcular_navegacion_inercial(muestras: List[Muestra]):
    """
    Velocidad y altura estimadas por doble integracion del acelerometro
    (eje Z, asumido alineado con el eje de empuje del cohete). El sesgo se
    calibra promediando las muestras previas al despegue (deberian leer
    ~1g si el cohete esta vertical y en reposo en la rampa).

    ADVERTENCIA: esta estimacion DERIVA con el tiempo (el error se acumula
    cuadraticamente en la posicion). Se incluye como comparacion frente a
    la altitud barometrica, no como reemplazo de esta.

    Devuelve (velocidad_ms, altura_m), ambos arrays numpy.
    """
    n = len(muestras)
    if n < 2:
        return np.zeros(n), np.zeros(n)

    t = np.array([m.ms for m in muestras], dtype=np.float64) / 1000.0
    acc_z_ms2 = np.array([acc_cuentas_a_ms2(m.acc_z) for m in muestras])

    idx_despegue = next((i for i, m in enumerate(muestras) if m.en_vuelo), n)
    if idx_despegue >= 5:
        sesgo = float(np.mean(acc_z_ms2[:idx_despegue]))
    else:
        sesgo = G_A_MS2  # sin suficientes muestras en rampa: asumir 1g nominal

    acc_neta = acc_z_ms2 - sesgo

    v = np.zeros(n)
    h = np.zeros(n)
    for i in range(1, n):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 1.0:
            # Salto de tiempo grande (perdida de paquetes en vivo): no
            # integrar ese hueco para no generar un pico espurio.
            dt = 0.0
        v[i] = v[i - 1] + acc_neta[i] * dt
        h[i] = h[i - 1] + v[i] * dt
    return v, h


def calcular_mach(muestras: List[Muestra], velocidad_ms: np.ndarray) -> np.ndarray:
    """Numero de Mach usando la velocidad del sonido local (segun la
    temperatura medida por el propio cohete)."""
    temp_k = np.array([m.temp_c for m in muestras]) + 273.15
    a_sonido = 20.05 * np.sqrt(np.maximum(temp_k, 1.0))  # m/s, aire seco
    return np.abs(velocidad_ms) / a_sonido


def calcular_presion_dinamica_pa(muestras: List[Muestra], velocidad_ms: np.ndarray) -> np.ndarray:
    """Presion dinamica aproximada q = 0.5 * rho * v^2, con densidad del
    aire segun la formula barometrica estandar (valida en la troposfera,
    hasta ~11 km)."""
    alt = np.clip(np.array([m.altitude for m in muestras]), 0, 11000)
    rho = 1.225 * (1 - 2.25577e-5 * alt) ** 4.2559
    return 0.5 * rho * velocidad_ms ** 2


def calcular_velocidad_vertical(muestras: List[Muestra]) -> np.ndarray:
    """
    Devuelve un array (misma longitud que muestras) con la velocidad
    vertical estimada en m/s, por diferencia central de la altitud
    respecto al tiempo real de cada muestra (usa 'ms', no asume muestreo
    parejo: valido tanto para telemetria en vivo (2Hz) como para datos de
    EEPROM (20Hz, con gaps si hubo perdida de paquetes).
    """
    n = len(muestras)
    if n < 2:
        return np.zeros(n)

    t = np.array([m.ms for m in muestras], dtype=np.float64) / 1000.0  # segundos
    alt = np.array([m.altitude for m in muestras], dtype=np.float64)

    vel = np.zeros(n)
    # Diferencia central para los puntos interiores
    dt_central = t[2:] - t[:-2]
    dt_central[dt_central == 0] = np.nan  # evitar division por cero si hay ms repetidos
    vel[1:-1] = (alt[2:] - alt[:-2]) / dt_central

    # Bordes: diferencia hacia adelante/atras
    if t[1] != t[0]:
        vel[0] = (alt[1] - alt[0]) / (t[1] - t[0])
    if t[-1] != t[-2]:
        vel[-1] = (alt[-1] - alt[-2]) / (t[-1] - t[-2])

    return np.nan_to_num(vel, nan=0.0, posinf=0.0, neginf=0.0)


@dataclass
class ResumenVuelo:
    n_muestras: int
    duracion_s: float
    altitud_max_m: float
    t_apogeo_s: Optional[float]
    idx_apogeo: Optional[int]
    velocidad_max_ms: float       # mayor velocidad ascendente
    velocidad_min_ms: float       # mayor velocidad descendente (mas negativa)
    acc_mag_max_g: float          # magnitud maxima del vector acc, en g
    acc_mag_max_ms2: float
    t_despegue_s: Optional[float]
    t_desplegado_s: Optional[float]
    paquetes_totales: int
    paquetes_esperados: Optional[int]  # segun el rango de 'count', None si no aplica
    paquetes_perdidos_pct: Optional[float]
    veloc_angular_max_dps: float
    mach_max: float
    presion_dinamica_max_pa: float
    angulo_inclinacion_deg: Optional[float]
    altura_inercial_max_m: float


def resumen_de_vuelo(muestras: List[Muestra],
                     vel: Optional[np.ndarray] = None,
                     alt_suavizada: Optional[np.ndarray] = None,
                     giro_mag_dps: Optional[np.ndarray] = None) -> ResumenVuelo:
    """Si se pasan vel / alt_suavizada / giro_mag_dps (por ejemplo ya
    filtrados con filtros.py) se usan esos; si no, se calculan crudos."""
    if not muestras:
        return ResumenVuelo(0, 0.0, 0.0, None, None, 0.0, 0.0, 0.0, 0.0, None, None, 0, None, None,
                             0.0, 0.0, 0.0, None, 0.0)

    t0 = muestras[0].ms / 1000.0
    t = [m.ms / 1000.0 - t0 for m in muestras]
    alt = list(alt_suavizada) if alt_suavizada is not None else [m.altitude for m in muestras]
    acc_mag = [m.acc_mag for m in muestras]
    if vel is None:
        vel = calcular_velocidad_vertical(muestras)

    idx_apogeo = int(np.argmax(alt))
    idx_acc_max = int(np.argmax(acc_mag))

    t_despegue = None
    for m, ti in zip(muestras, t):
        if m.en_vuelo:
            t_despegue = ti
            break

    t_desplegado = None
    for m, ti in zip(muestras, t):
        if m.desplegado:
            t_desplegado = ti
            break

    counts = [m.count for m in muestras]
    paquetes_esperados = None
    perdidos_pct = None
    if len(counts) >= 2 and counts[-1] >= counts[0]:
        paquetes_esperados = counts[-1] - counts[0] + 1
        if paquetes_esperados > 0:
            perdidos = paquetes_esperados - len(muestras)
            perdidos_pct = max(0.0, 100.0 * perdidos / paquetes_esperados)

    giro_dps = giro_mag_dps if giro_mag_dps is not None else calcular_magnitud_giro_dps(muestras)
    mach = calcular_mach(muestras, vel)
    q_din = calcular_presion_dinamica_pa(muestras, vel)
    _, h_inercial = calcular_navegacion_inercial(muestras)

    return ResumenVuelo(
        n_muestras=len(muestras),
        duracion_s=t[-1] - t[0] if len(t) > 1 else 0.0,
        altitud_max_m=alt[idx_apogeo],
        t_apogeo_s=t[idx_apogeo],
        idx_apogeo=idx_apogeo,
        velocidad_max_ms=float(np.max(vel)) if len(vel) else 0.0,
        velocidad_min_ms=float(np.min(vel)) if len(vel) else 0.0,
        acc_mag_max_g=acc_cuentas_a_g(acc_mag[idx_acc_max]),
        acc_mag_max_ms2=acc_cuentas_a_ms2(acc_mag[idx_acc_max]),
        t_despegue_s=t_despegue,
        t_desplegado_s=t_desplegado,
        paquetes_totales=len(muestras),
        paquetes_esperados=paquetes_esperados,
        paquetes_perdidos_pct=perdidos_pct,
        veloc_angular_max_dps=float(np.max(giro_dps)) if len(giro_dps) else 0.0,
        mach_max=float(np.max(mach)) if len(mach) else 0.0,
        presion_dinamica_max_pa=float(np.max(q_din)) if len(q_din) else 0.0,
        angulo_inclinacion_deg=angulo_inclinacion_inicial_deg(muestras),
        altura_inercial_max_m=float(np.max(h_inercial)) if len(h_inercial) else 0.0,
    )
