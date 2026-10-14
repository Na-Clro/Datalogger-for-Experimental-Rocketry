"""
filtros.py
Procesamiento de señal para que lo que se muestra en la PC sea "racional":

  - ValidadorTiempo: arma un eje de tiempo continuo y descarta muestras
    corruptas (un frame basura que pasa el checksum XOR puede traer un 'ms'
    absurdo y hacer que el eje de tiempo salte a millones de segundos).
    Tambien tolera que el cohete se reinicie (millis() vuelve a 0).

  - EstimadorBiasGiro: estima el offset del giroscopio mientras el cohete
    esta quieto en la rampa y lo resta (el firmware ya calibra al
    arrancar; esto corrige lo que quede, por ejemplo deriva termica).

  - FiltroKalmanVertical: estima altitud y velocidad vertical fusionando la
    altitud barometrica (ruidosa pero sin deriva) con la aceleracion medida
    (suave pero que deriva si se integra sola). Es el esquema estandar de
    los altimetros de coheteria. Funciona en vivo (paso a paso) y tambien
    tiene un suavizador hacia atras (RTS) para el analisis post-vuelo, que
    usa datos "del futuro" y da curvas todavia mas limpias.

La aceleracion vertical se obtiene proyectando el vector del acelerometro
sobre la direccion de la gravedad medida en la rampa (asi no hace falta
saber cual eje del MPU apunta hacia la nariz). Vale mientras el cohete
mantenga la orientacion de la rampa (ascenso); despues del despliegue del
paracaidas el cohete da vueltas y la aceleracion deja de usarse.
"""
from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

import numpy as np

G0 = 9.80665
ACC_LSB_POR_G = 2048.0
GYRO_LSB_POR_DPS = 16.4


# =====================================================================
#  Tiempo continuo + descarte de muestras corruptas
# =====================================================================
class ValidadorTiempo:
    """
    Recibe muestras en orden de llegada y devuelve el tiempo continuo (s)
    de cada una, o None si la muestra debe descartarse.

    Reglas:
      - Avance normal: 0 < d_ms <= MAX_SALTO_MS y 0 < d_count <= MAX_SALTO_COUNT.
      - Muestra que no cumple: se retiene como "candidata" y se descarta.
        Si la siguiente es coherente con la candidata (y no con la ultima
        aceptada) se interpreta como reinicio del cohete: se acepta y el
        tiempo continua sumando el hueco real (segun el reloj de la PC).
        Si la siguiente es coherente con la ultima aceptada, la candidata
        era basura y se olvida.
    """
    MAX_SALTO_MS = 60_000
    MAX_SALTO_COUNT = 5_000
    HUECO_POR_DEFECTO_S = 1.0  # si no hay reloj de PC (datos de EEPROM)

    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        self._ultima = None
        self._candidata = None
        self._t = 0.0
        self.descartadas = 0
        self.reinicios = 0

    @classmethod
    def _coherente(cls, previa, nueva) -> bool:
        d_ms = nueva.ms - previa.ms
        d_count = nueva.count - previa.count
        return 0 < d_ms <= cls.MAX_SALTO_MS and 0 < d_count <= cls.MAX_SALTO_COUNT

    def procesar(self, m) -> Optional[float]:
        if self._ultima is None:
            self._ultima = m
            self._t = 0.0
            return self._t

        if self._coherente(self._ultima, m):
            self._t += (m.ms - self._ultima.ms) / 1000.0
            self._ultima = m
            self._candidata = None
            return self._t

        if self._candidata is not None and self._coherente(self._candidata, m):
            # Reinicio del cohete (o cambio de sesion): continuamos el eje
            # de tiempo con el hueco real medido por la PC.
            hueco = m.t_pc - self._ultima.t_pc
            if not (0 < hueco < 3600):
                hueco = self.HUECO_POR_DEFECTO_S
            self._t += hueco
            self._ultima = m
            self._candidata = None
            self.reinicios += 1
            return self._t

        self._candidata = m
        self.descartadas += 1
        return None


def normalizar_tiempo(muestras: list) -> Tuple[list, np.ndarray, ValidadorTiempo]:
    """Para el analisis: filtra muestras corruptas y devuelve
    (muestras_validas, t_continuo_s, validador). Las muestras devueltas
    tienen 'ms' reescrito al tiempo continuo, asi todas las funciones de
    analisis que usan m.ms quedan coherentes."""
    v = ValidadorTiempo()
    salida, tiempos = [], []
    for m in muestras:
        t = v.procesar(m)
        if t is not None:
            tiempos.append(t)
            salida.append(replace(m, ms=int(round(t * 1000.0))))
    return salida, np.array(tiempos, dtype=np.float64), v


# =====================================================================
#  Bias del giroscopio
# =====================================================================
class EstimadorBiasGiro:
    """Promedio movil del giroscopio mientras el cohete esta quieto en la
    rampa (no en vuelo y con giro chico). En vuelo el bias queda congelado."""
    UMBRAL_QUIETO_DPS = 6.0   # por encima de esto se considera que se mueve
    ALPHA = 0.05              # ~20 muestras de memoria

    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        self.bias = np.zeros(3)
        self._iniciado = False

    def procesar(self, m) -> np.ndarray:
        """Devuelve el giro corregido (deg/s) [x, y, z]."""
        g = np.array([m.gyro_x, m.gyro_y, m.gyro_z], dtype=np.float64) / GYRO_LSB_POR_DPS
        quieto = (not m.en_vuelo) and np.all(np.abs(g - self.bias) < self.UMBRAL_QUIETO_DPS)
        if quieto:
            if not self._iniciado:
                self.bias = g.copy()
                self._iniciado = True
            else:
                self.bias += self.ALPHA * (g - self.bias)
        return g - self.bias


def giro_sin_bias(muestras: list) -> Tuple[np.ndarray, np.ndarray]:
    """Para el analisis: estima el bias con las muestras previas al
    despegue (mediana, robusta a movimientos) y lo resta.
    Devuelve (giro_dps Nx3, bias_dps[3])."""
    g = np.array([[m.gyro_x, m.gyro_y, m.gyro_z] for m in muestras], dtype=np.float64) / GYRO_LSB_POR_DPS
    if len(g) == 0:
        return g.reshape(0, 3), np.zeros(3)
    idx = next((i for i, m in enumerate(muestras) if m.en_vuelo), len(muestras))
    previas = g[:idx]
    if len(previas) >= 5:
        quietas = previas[np.all(np.abs(previas - np.median(previas, axis=0)) < 6.0, axis=1)]
        base = quietas if len(quietas) >= 5 else previas
        bias = np.median(base, axis=0)
    else:
        bias = np.zeros(3)
    return g - bias, bias


# =====================================================================
#  Kalman de altitud / velocidad vertical
# =====================================================================
@dataclass
class ParametrosKalman:
    sigma_alt_m: float = 1.0         # ruido inicial de la altitud barometrica (1 sigma)
    adaptar_ruido_alt: bool = True   # re-estimar el ruido real mientras esta en la rampa
    sigma_alt_min: float = 0.2
    sigma_alt_max: float = 5.0
    sigma_acc_con_imu: float = 2.0   # incertidumbre de la aceleracion usada como entrada (m/s^2)
    sigma_acc_sin_imu: float = 6.0   # maniobra no modelada cuando no se usa el acelerometro
    gate_sigmas: float = 5.0         # rechazo de saltos de altitud (en sigmas de innovacion)
    max_rechazos_seguidos: int = 3   # despues de esto se acepta igual (cambio real)


class FiltroKalmanVertical:
    """
    Estado x = [altitud, velocidad]. Modelo:
        h' = h + v dt + 0.5 a dt^2
        v' = v + a dt
    con 'a' = aceleracion vertical medida por la IMU (si es confiable) o 0.
    Medicion: altitud barometrica.
    """
    def __init__(self, params: Optional[ParametrosKalman] = None):
        self.p = params or ParametrosKalman()
        self.reiniciar()

    def reiniciar(self):
        self.x = None
        self.P = None
        self.t = None
        self._rechazos = 0
        self._R = self.p.sigma_alt_m ** 2
        self._n_rampa = 0
        self._media_rampa = 0.0
        self._var_rampa = 0.0
        # gravedad de referencia medida en la rampa (vector en g)
        self._g_ref = None
        self._g_ref_n = 0

    # ---------- referencia de gravedad ----------
    def _actualizar_referencia(self, acc_g: np.ndarray, m):
        mag = float(np.linalg.norm(acc_g))
        if m.en_vuelo or not (0.85 < mag < 1.15):
            return
        if self._g_ref is None:
            self._g_ref = acc_g.copy()
            self._g_ref_n = 1
        else:
            alpha = max(0.02, 1.0 / (self._g_ref_n + 1))
            self._g_ref += alpha * (acc_g - self._g_ref)
            self._g_ref_n += 1

    def aceleracion_vertical(self, m) -> Optional[float]:
        """Aceleracion vertical (m/s^2, positiva hacia arriba) o None si no
        es confiable usar la IMU en este momento."""
        acc_g = np.array([m.acc_x, m.acc_y, m.acc_z], dtype=np.float64) / ACC_LSB_POR_G
        self._actualizar_referencia(acc_g, m)
        if self._g_ref is None or self._g_ref_n < 3 or m.desplegado:
            return None
        n_ref = float(np.linalg.norm(self._g_ref))
        if n_ref < 0.5:
            return None
        arriba = self._g_ref / n_ref
        # En reposo el acelerometro mide +1g "hacia arriba"; la
        # aceleracion real es lo que exceda esa referencia.
        return (float(np.dot(acc_g, arriba)) - n_ref) * G0

    # ---------- pasos del filtro ----------
    @staticmethod
    def _matrices(dt: float, sigma_a: float):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([0.5 * dt * dt, dt])
        Q = sigma_a ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                                     [dt ** 3 / 2, dt ** 2]])
        return F, B, Q

    @property
    def sigma_alt_estimado(self) -> float:
        return float(np.sqrt(self._R))

    def paso(self, t: float, alt: float, a_vert: Optional[float], en_rampa: bool = False):
        """Avanza el filtro hasta 't' con la medicion 'alt'. Devuelve
        (alt_filtrada, vel, info) donde info guarda lo necesario para el
        suavizador."""
        R = self._R
        if self.x is None:
            self.x = np.array([alt, 0.0])
            self.P = np.diag([R, 4.0])
            self.t = t
            return self.x[0], self.x[1], None

        dt = t - self.t
        if dt <= 0:
            return self.x[0], self.x[1], None
        dt = min(dt, 2.0)  # huecos largos (perdida de paquetes): no extrapolar de mas
        self.t = t

        usar_imu = a_vert is not None
        sigma_a = self.p.sigma_acc_con_imu if usar_imu else self.p.sigma_acc_sin_imu
        F, B, Q = self._matrices(dt, sigma_a)
        u = a_vert if usar_imu else 0.0

        x_pred = F @ self.x + B * u
        P_pred = F @ self.P @ F.T + Q

        innov = alt - x_pred[0]
        if en_rampa and self.p.adaptar_ruido_alt:
            # En la rampa la altitud real es constante: su varianza medida
            # ES el ruido del sensor. Media y varianza moviles (~20 muestras).
            self._n_rampa += 1
            if self._n_rampa == 1:
                self._media_rampa = alt
            else:
                d = alt - self._media_rampa
                self._media_rampa += 0.05 * d
                self._var_rampa += 0.05 * (min(d * d, 100.0) - self._var_rampa)
            if self._n_rampa >= 10:
                R = float(np.clip(self._var_rampa, self.p.sigma_alt_min ** 2, self.p.sigma_alt_max ** 2))
                self._R = R
        S = P_pred[0, 0] + R
        # El rechazo de saltos solo se aplica con dinamica tranquila (rampa,
        # planeo, paracaidas): durante el empuje el modelo puede quedar
        # atras de la realidad y rechazar mediciones buenas empeora todo.
        dinamica_tranquila = (a_vert is None) or abs(a_vert) < 2.0 * G0
        if (dinamica_tranquila and abs(innov) > self.p.gate_sigmas * np.sqrt(S)
                and self._rechazos < self.p.max_rechazos_seguidos):
            # Salto espurio de altitud: se ignora esta medicion
            self._rechazos += 1
            self.x, self.P = x_pred, P_pred
        else:
            self._rechazos = 0
            K = P_pred[:, 0] / S
            self.x = x_pred + K * innov
            self.P = P_pred - np.outer(K, P_pred[0, :])

        info = (F, x_pred, P_pred)
        return self.x[0], self.x[1], info


def suavizar_vertical(muestras: list, t: np.ndarray,
                      params: Optional[ParametrosKalman] = None) -> Tuple[np.ndarray, np.ndarray]:
    """Analisis post-vuelo: Kalman hacia adelante + suavizador RTS hacia
    atras. Devuelve (altitud_suavizada, velocidad_suavizada)."""
    n = len(muestras)
    if n == 0:
        return np.zeros(0), np.zeros(0)
    kf = FiltroKalmanVertical(params)
    xs = np.zeros((n, 2))
    Ps = np.zeros((n, 2, 2))
    infos = [None] * n
    for i, m in enumerate(muestras):
        a = kf.aceleracion_vertical(m)
        _, _, info = kf.paso(float(t[i]), float(m.altitude), a, en_rampa=not m.en_vuelo)
        xs[i] = kf.x
        Ps[i] = kf.P
        infos[i] = info

    # RTS
    xs_s = xs.copy()
    Ps_s = Ps.copy()
    for k in range(n - 2, -1, -1):
        info = infos[k + 1]
        if info is None:
            xs_s[k] = xs[k]
            continue
        F, x_pred, P_pred = info
        try:
            C = Ps[k] @ F.T @ np.linalg.inv(P_pred)
        except np.linalg.LinAlgError:
            continue
        xs_s[k] = xs[k] + C @ (xs_s[k + 1] - x_pred)
        Ps_s[k] = Ps[k] + C @ (Ps_s[k + 1] - P_pred) @ C.T
    return xs_s[:, 0], xs_s[:, 1]
