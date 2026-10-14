"""
ui_principal.py
Ventana principal de la estacion de control en PC.

Pestanas:
  - En Vivo: conexion a la estacion terrena, telemetria en vivo con
    paneles ACOPLABLES (se pueden arrastrar, apilar en pestanas, flotar o
    cerrar) para altitud, aceleracion, giroscopio, temperatura y vector 3D.
  - Descarga del cohete: conexion USB directa para bajar la EEPROM,
    consultar estado, y borrar memoria. Incluye una seccion de
    "operaciones remotas por LoRa" marcada como trabajo futuro (sin
    soporte de firmware todavia).
  - Analisis: abrir un CSV y ver el resumen de vuelo junto con
    sub-pestanas dedicadas a cada grupo de variables (aceleracion,
    giroscopio, altitud/temperatura, navegacion inercial y derivados).
"""
import os
import sys
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, List, Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QFileDialog, QGroupBox,
    QProgressBar, QMessageBox, QSplitter, QPlainTextEdit, QDockWidget,
    QToolButton, QMenu, QInputDialog, QStackedWidget
)

import pyqtgraph as pg
import serial.tools.list_ports

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from protocolo import Muestra, comando_armado, comando_grabado
from enlace_serial import HiloEnlaceVivo
from descarga_directa import HiloDescarga, HiloComandoRapido, HiloUSBDirecto
from registro_csv import EscritorCSV, guardar_csv, leer_csv
from analisis import (
    calcular_velocidad_vertical, resumen_de_vuelo, acc_cuentas_a_g,
    gyro_cuentas_a_dps, calcular_magnitud_giro_dps, calcular_navegacion_inercial,
    calcular_mach, calcular_presion_dinamica_pa,
)
__version__ = '1.0.0'

from filtros import (
    ValidadorTiempo, FiltroKalmanVertical, EstimadorBiasGiro,
    normalizar_tiempo, suavizar_vertical, giro_sin_bias,
)


@dataclass
class Estimacion:
    """Valores ya filtrados que acompañan a cada muestra en vivo."""
    t: float            # tiempo continuo (s) desde el inicio de la sesion
    alt: float          # altitud filtrada (Kalman) (m)
    vel: float          # velocidad vertical filtrada (m/s)
    giro: tuple         # giro sin bias (deg/s) x, y, z
    sigma_alt: float    # ruido de altitud estimado en la rampa (m)


class ProcesadorVivo:
    """Encadena validador de tiempo + Kalman + bias del giro, para usar
    igual en la pestaña En Vivo (LoRa) y en la telemetria por USB directo."""
    def __init__(self):
        self.validador = ValidadorTiempo()
        self.kalman = FiltroKalmanVertical()
        self.bias_giro = EstimadorBiasGiro()

    def reiniciar(self):
        self.validador.reiniciar()
        self.kalman.reiniciar()
        self.bias_giro.reiniciar()

    @property
    def descartadas(self) -> int:
        return self.validador.descartadas

    def procesar(self, m) -> Optional[Estimacion]:
        t = self.validador.procesar(m)
        if t is None:
            return None
        a_vert = self.kalman.aceleracion_vertical(m)
        alt_f, vel_f, _ = self.kalman.paso(t, m.altitude, a_vert, en_rampa=not m.en_vuelo)
        giro = self.bias_giro.procesar(m)
        return Estimacion(t=t, alt=float(alt_f), vel=float(vel_f),
                          giro=tuple(float(x) for x in giro),
                          sigma_alt=self.kalman.sigma_alt_estimado)


def listar_puertos() -> List[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


# =====================================================================
#  Panel de valores actuales / maximos ("gauges")
# =====================================================================
class PanelValores(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('Telemetria', parent)
        self._maximos = {}
        self._minimos = {}
        self._reset_maximos()

        grid = QGridLayout(self)
        etiquetas = [
            ('Altitud actual (filtrada)', 'alt_act', 'm'),
            ('Altitud cruda (barómetro)', 'alt_cruda', 'm'),
            ('Altitud maxima', 'alt_max', 'm'),
            ('Velocidad actual', 'vel_act', 'm/s'),
            ('Velocidad maxima', 'vel_max', 'm/s'),
            ('Aceleracion actual', 'acc_act', 'g'),
            ('Aceleracion maxima', 'acc_max', 'g'),
            ('Giro X / Y / Z', 'giro_xyz', 'deg/s'),
            ('Giro total (max)', 'giro_max', 'deg/s'),
            ('Temperatura actual', 'temp_act', 'C'),
            ('Temperatura min/max', 'temp_minmax', 'C'),
            ('RSSI', 'rssi', 'dBm'),
            ('SNR', 'snr', 'dB'),
            ('Paquetes recibidos', 'pkt', ''),
            ('Paquetes descartados', 'descartados', ''),
            ('Ruido de altitud (1σ)', 'ruido_alt', 'm'),
        ]
        self._valores = {}
        fila = 0
        for fila, (texto, clave, unidad) in enumerate(etiquetas):
            grid.addWidget(QLabel(texto + ':'), fila, 0)
            lbl = QLabel('--')
            lbl.setStyleSheet('font-weight: bold;')
            grid.addWidget(lbl, fila, 1)
            grid.addWidget(QLabel(unidad), fila, 2)
            self._valores[clave] = lbl

        fila += 1
        self.lbl_armado = QLabel('SEGURO')
        self.lbl_armado.setAlignment(Qt.AlignCenter)
        self._estilo_armado(False)
        grid.addWidget(self.lbl_armado, fila, 0, 1, 3)

        fila += 1
        self.lbl_grabando = QLabel('REC: OFF')
        self.lbl_grabando.setAlignment(Qt.AlignCenter)
        self.lbl_grabando.setStyleSheet('font-size: 14px; font-weight: bold; padding: 4px; color: #888888;')
        grid.addWidget(self.lbl_grabando, fila, 0, 1, 3)

        fila += 1
        self.lbl_vuelo = QLabel('EN RAMPA')
        self.lbl_vuelo.setAlignment(Qt.AlignCenter)
        grid.addWidget(self.lbl_vuelo, fila, 0, 1, 3)

        self._ultimo_ms: Optional[int] = None
        self._ultima_alt: Optional[float] = None

    def _estilo_armado(self, armado: bool):
        color = '#e74c3c' if armado else '#2ecc71'
        self.lbl_armado.setText('ARMADO' if armado else 'SEGURO')
        self.lbl_armado.setStyleSheet(
            f'font-size: 18px; font-weight: bold; background-color: {color}; color: white; padding: 6px;')

    def _reset_maximos(self):
        self._maximos = {
            'alt_max': -1e9, 'acc_max': -1e9, 'vel_max': -1e9, 'giro_max': -1e9,
            'temp_max': -1e9,
        }
        self._minimos = {'temp_min': 1e9}

    def reiniciar(self):
        self._reset_maximos()
        self._ultimo_ms = None
        self._ultima_alt = None
        for lbl in self._valores.values():
            lbl.setText('--')
        self._estilo_armado(False)
        self.lbl_grabando.setText('REC: OFF')
        self.lbl_vuelo.setText('EN RAMPA')

    def mostrar_descartados(self, n: int):
        self._valores['descartados'].setText(f'{n}')

    def actualizar(self, m: Muestra, est: Estimacion, descartados: int = 0):
        # Velocidad: estimacion Kalman (altitud baro + acelerometro), no la
        # derivada directa de la altitud (que con +/-2 m de ruido a 2 Hz da
        # saltos de mas de 10 m/s estando quieto).
        vel = est.vel

        acc_g = acc_cuentas_a_g(m.acc_mag)
        gx, gy, gz = est.giro
        giro_mag = (gx ** 2 + gy ** 2 + gz ** 2) ** 0.5

        self._maximos['alt_max'] = max(self._maximos['alt_max'], est.alt)
        self._maximos['acc_max'] = max(self._maximos['acc_max'], acc_g)
        self._maximos['vel_max'] = max(self._maximos['vel_max'], vel)
        self._maximos['giro_max'] = max(self._maximos['giro_max'], giro_mag)
        self._maximos['temp_max'] = max(self._maximos['temp_max'], m.temp_c)
        self._minimos['temp_min'] = min(self._minimos['temp_min'], m.temp_c)

        self._valores['alt_act'].setText(f'{est.alt:.1f}')
        self._valores['alt_cruda'].setText(f'{m.altitude:.1f}')
        self._valores['descartados'].setText(f'{descartados}')
        self._valores['ruido_alt'].setText(f'{est.sigma_alt:.2f}')
        self._valores['alt_max'].setText(f"{self._maximos['alt_max']:.1f}")
        self._valores['vel_act'].setText(f'{vel:.1f}')
        self._valores['vel_max'].setText(f"{self._maximos['vel_max']:.1f}")
        self._valores['acc_act'].setText(f'{acc_g:.2f}')
        self._valores['acc_max'].setText(f"{self._maximos['acc_max']:.2f}")
        self._valores['giro_xyz'].setText(f'{gx:.0f} / {gy:.0f} / {gz:.0f}')
        self._valores['giro_max'].setText(f"{self._maximos['giro_max']:.0f}")
        self._valores['temp_act'].setText(f'{m.temp_c:.1f}' + (' (resp.)' if m.temp_respaldo else ''))
        self._valores['temp_minmax'].setText(f"{self._minimos['temp_min']:.1f} / {self._maximos['temp_max']:.1f}")
        self._valores['rssi'].setText(f'{m.rssi}' if m.rssi is not None else '--')
        self._valores['snr'].setText(f'{m.snr:.1f}' if m.snr is not None else '--')
        self._valores['pkt'].setText(f'{m.count}')

        self._estilo_armado(m.armado)

        if m.memoria_llena:
            self.lbl_grabando.setText('REC: MEMORIA LLENA')
            color_rec = '#e74c3c'
        else:
            self.lbl_grabando.setText('REC: ON' if m.grabando else 'REC: OFF')
            color_rec = '#00bcd4' if m.grabando else '#888888'
        self.lbl_grabando.setStyleSheet(f'font-size: 14px; font-weight: bold; padding: 4px; color: {color_rec};')

        if m.desplegado:
            self.lbl_vuelo.setText('PARACAIDAS DESPLEGADO')
        elif m.en_vuelo:
            self.lbl_vuelo.setText('EN VUELO')
        else:
            self.lbl_vuelo.setText('EN RAMPA')


# =====================================================================
#  Botones de comando (armado / grabado)
# =====================================================================
class PanelBotones(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('Comandos', parent)
        layout = QVBoxLayout(self)

        self.btn_armar = QPushButton('ARMAR E-MATCH')
        self.btn_armar.setStyleSheet('font-weight: bold; padding: 10px; background-color: #2ecc71;')
        self.btn_grabar = QPushButton('INICIAR GRABADO')
        self.btn_grabar.setStyleSheet('font-weight: bold; padding: 10px;')

        layout.addWidget(self.btn_armar)
        layout.addWidget(self.btn_grabar)

        self._armado_actual = False
        self._grabando_actual = False

        self.btn_armar.clicked.connect(self._on_armar)
        self.btn_grabar.clicked.connect(self._on_grabar)

        self.solicitar_armado = None
        self.solicitar_grabado = None

    def _on_armar(self):
        objetivo = not self._armado_actual
        if objetivo:
            resp = QMessageBox.question(
                self, 'Confirmar armado',
                'Confirmas ARMAR el e-match?\n\nEsto habilita el disparo real del paracaidas.',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if resp != QMessageBox.Yes:
                return
        if self.solicitar_armado:
            self.solicitar_armado(objetivo)

    def _on_grabar(self):
        objetivo = not self._grabando_actual
        if self.solicitar_grabado:
            self.solicitar_grabado(objetivo)

    def actualizar_estado(self, m: Muestra):
        self._armado_actual = m.armado
        self._grabando_actual = m.grabando
        self.btn_armar.setText('DESARMAR' if m.armado else 'ARMAR E-MATCH')
        color = '#e74c3c' if m.armado else '#2ecc71'
        self.btn_armar.setStyleSheet(f'font-weight: bold; padding: 10px; background-color: {color};')
        self.btn_grabar.setText('DETENER GRABADO' if m.grabando else 'INICIAR GRABADO')


# =====================================================================
#  Sesion de vuelo: finalizar / nuevo vuelo
# =====================================================================
class PanelSesion(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('Sesión de vuelo', parent)
        layout = QVBoxLayout(self)
        self.btn_finalizar = QPushButton('FINALIZAR VUELO Y GUARDAR')
        self.btn_finalizar.setStyleSheet('font-weight: bold; padding: 8px;')
        self.btn_finalizar.setToolTip(
            'Cierra el CSV de este vuelo (queda guardado) y deja de registrar.\n'
            'Opcionalmente detiene la grabación en la EEPROM del cohete.')
        self.btn_nuevo = QPushButton('NUEVO VUELO')
        self.btn_nuevo.setStyleSheet('font-weight: bold; padding: 8px;')
        self.btn_nuevo.setToolTip(
            'Reinicia gráficos, máximos y filtros, y empieza a registrar en un\n'
            'archivo CSV nuevo con fecha y hora (no mezcla vuelos en un mismo archivo).')
        self.lbl_archivo = QLabel('Sin registro activo')
        self.lbl_archivo.setWordWrap(True)
        self.lbl_archivo.setStyleSheet('color: #555; font-size: 11px;')
        layout.addWidget(self.btn_finalizar)
        layout.addWidget(self.btn_nuevo)
        layout.addWidget(self.lbl_archivo)


# =====================================================================
#  Botones de operaciones remotas (LoRa) -- PIN: trabajo futuro
# =====================================================================
class PanelOperacionesRemotas(QGroupBox):
    """
    Placeholder para comandos que en el futuro viajarian por LoRa a
    traves de la estacion terrena: descarga remota de la EEPROM, consulta
    de estado remota, y borrado remoto. NINGUNO tiene soporte de firmware
    todavia (ver protocolo.py), por eso los botones estan deshabilitados.
    Se dejan visibles para que el operador sepa que la funcionalidad esta
    planificada, y para no tener que rediseñar la pantalla cuando se
    implemente.
    """
    def __init__(self, parent=None):
        super().__init__('Operaciones remotas por LoRa (próximamente)', parent)
        layout = QVBoxLayout(self)

        self.btn_descargar = QPushButton('Descargar EEPROM por LoRa')
        self.btn_estado = QPushButton('Consultar estado por LoRa')
        self.btn_borrar = QPushButton('Borrar memoria por LoRa')

        for b in (self.btn_descargar, self.btn_estado, self.btn_borrar):
            b.setEnabled(False)
            layout.addWidget(b)

        self.btn_descargar.setToolTip(
            'Pendiente de implementar: requiere un protocolo de descarga paginada\n'
            'por LoRa en el firmware del cohete y de la estación terrena.')
        self.btn_estado.setToolTip(
            'Pendiente de implementar: el cohete ya reserva el código de comando\n'
            'CMD_PING (0x03) para esto, pero todavía no hace nada con él.')
        self.btn_borrar.setToolTip(
            'Pendiente de implementar. Por diseño, este botón NO debería habilitarse\n'
            'hasta confirmar que los datos se descargaron por conexión física\n'
            '(ver la pestaña "Descarga del cohete").')

        nota = QLabel(
            'Estos comandos todavía no existen en el firmware -- quedan reservados '
            'acá como referencia de protocolo (ver protocolo.py) para una próxima '
            'iteración del proyecto.'
        )
        nota.setWordWrap(True)
        nota.setStyleSheet('color: #777; font-size: 11px;')
        layout.addWidget(nota)


# =====================================================================
#  Conjunto de gráficos en vivo (pyqtgraph), pensados para ir en docks
# =====================================================================
class ConjuntoGraficosVivo:
    MAX_PUNTOS = 3600  # ~30 min a 2Hz de telemetria en vivo

    def __init__(self):
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plot_alt = pg.PlotWidget(title='Altitud (m)')
        self.plot_alt.showGrid(x=True, y=True, alpha=0.3)
        self.plot_alt.addLegend()
        self.curva_alt_cruda = self.plot_alt.plot(
            pen=None, symbol='o', symbolSize=4, symbolBrush=(150, 150, 150), symbolPen=None,
            name='Barómetro (crudo)')
        self.curva_alt = self.plot_alt.plot(pen=pg.mkPen('b', width=2), name='Filtrada')

        self.plot_vel = pg.PlotWidget(title='Velocidad vertical (m/s)')
        self.plot_vel.showGrid(x=True, y=True, alpha=0.3)
        self.curva_vel = self.plot_vel.plot(pen=pg.mkPen((0, 140, 0), width=2))
        self.plot_vel.setXLink(self.plot_alt)

        self.plot_acc = pg.PlotWidget(title='Aceleración (g)')
        self.plot_acc.showGrid(x=True, y=True, alpha=0.3)
        self.curva_acc_x = self.plot_acc.plot(pen=pg.mkPen('r', width=1), name='X')
        self.curva_acc_y = self.plot_acc.plot(pen=pg.mkPen('g', width=1), name='Y')
        self.curva_acc_z = self.plot_acc.plot(pen=pg.mkPen('b', width=1), name='Z')
        self.plot_acc.addLegend()
        self.plot_acc.setXLink(self.plot_alt)

        self.plot_giro = pg.PlotWidget(title='Giroscopio, sin offset (deg/s)')
        self.plot_giro.showGrid(x=True, y=True, alpha=0.3)
        self.curva_giro_x = self.plot_giro.plot(pen=pg.mkPen('r', width=1), name='X')
        self.curva_giro_y = self.plot_giro.plot(pen=pg.mkPen('g', width=1), name='Y')
        self.curva_giro_z = self.plot_giro.plot(pen=pg.mkPen('b', width=1), name='Z')
        self.plot_giro.addLegend()
        self.plot_giro.setXLink(self.plot_alt)

        self.plot_temp = pg.PlotWidget(title='Temperatura (°C)')
        self.plot_temp.showGrid(x=True, y=True, alpha=0.3)
        self.curva_temp = self.plot_temp.plot(pen=pg.mkPen('m', width=2))
        self.plot_temp.setXLink(self.plot_alt)

        self._series = {k: deque(maxlen=self.MAX_PUNTOS) for k in (
            't', 'alt_cruda', 'alt', 'vel', 'accx', 'accy', 'accz',
            'girox', 'giroy', 'giroz', 'temp')}

    def reiniciar(self):
        for d in self._series.values():
            d.clear()
        for curva in (self.curva_alt_cruda, self.curva_alt, self.curva_vel,
                      self.curva_acc_x, self.curva_acc_y, self.curva_acc_z,
                      self.curva_giro_x, self.curva_giro_y, self.curva_giro_z, self.curva_temp):
            curva.setData([], [])

    def agregar(self, m: Muestra, est: Estimacion):
        s = self._series
        # El eje de tiempo viene del ValidadorTiempo: continuo, sin saltos
        # por frames corruptos ni por reinicios del cohete.
        s['t'].append(est.t)
        s['alt_cruda'].append(m.altitude)
        s['alt'].append(est.alt)
        s['vel'].append(est.vel)
        s['accx'].append(acc_cuentas_a_g(m.acc_x))
        s['accy'].append(acc_cuentas_a_g(m.acc_y))
        s['accz'].append(acc_cuentas_a_g(m.acc_z))
        s['girox'].append(est.giro[0])
        s['giroy'].append(est.giro[1])
        s['giroz'].append(est.giro[2])
        s['temp'].append(m.temp_c)

        t = list(s['t'])
        self.curva_alt_cruda.setData(t, list(s['alt_cruda']))
        self.curva_alt.setData(t, list(s['alt']))
        self.curva_vel.setData(t, list(s['vel']))
        self.curva_acc_x.setData(t, list(s['accx']))
        self.curva_acc_y.setData(t, list(s['accy']))
        self.curva_acc_z.setData(t, list(s['accz']))
        self.curva_giro_x.setData(t, list(s['girox']))
        self.curva_giro_y.setData(t, list(s['giroy']))
        self.curva_giro_z.setData(t, list(s['giroz']))
        self.curva_temp.setData(t, list(s['temp']))


# =====================================================================
#  Vector 3D de aceleracion (matplotlib embebido)
# =====================================================================
class VectorAcc3D(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.fig = Figure(figsize=(3, 3))
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        layout.addWidget(self.canvas)
        self._configurar_ejes()

    def _configurar_ejes(self):
        self.ax.clear()
        lim = 8
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_zlim(-lim, lim)
        self.ax.set_xlabel('X (g)')
        self.ax.set_ylabel('Y (g)')
        self.ax.set_zlabel('Z (g)')
        self.ax.set_title('Vector de aceleración')

    def actualizar(self, m: Muestra):
        ax_g = acc_cuentas_a_g(m.acc_x)
        ay_g = acc_cuentas_a_g(m.acc_y)
        az_g = acc_cuentas_a_g(m.acc_z)
        self._configurar_ejes()
        self.ax.quiver(0, 0, 0, ax_g, ay_g, az_g, color='r', linewidth=2, arrow_length_ratio=0.15)
        self.canvas.draw_idle()


# =====================================================================
#  Pestana: En Vivo
# =====================================================================
class PestanaEnVivo(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.hilo: Optional[HiloEnlaceVivo] = None
        self.escritor_csv: Optional[EscritorCSV] = None
        self._csv_path: Optional[str] = None
        self._n_csv = 0
        self._ultima_muestra: Optional[Muestra] = None
        # Callback opcional para abrir un CSV en la pestaña de Analisis
        self.al_analizar_archivo: Optional[Callable[[str], None]] = None

        # Procesamiento en vivo (ver filtros.py)
        self.proc = ProcesadorVivo()

        layout_principal = QVBoxLayout(self)

        barra = QHBoxLayout()
        self.combo_puertos = QComboBox()
        self.btn_refrescar = QPushButton('Actualizar')
        self.btn_conectar = QPushButton('Conectar')
        self.lbl_estado = QLabel('Desconectado')
        self.lbl_estado.setStyleSheet('color: red; font-weight: bold;')
        self.lbl_csv = QLabel('CSV: (no configurado)')
        self.btn_csv = QPushButton('Elegir archivo CSV...')
        self.btn_paneles = QToolButton()
        self.btn_paneles.setText('Paneles ▾')
        self.btn_paneles.setPopupMode(QToolButton.InstantPopup)

        barra.addWidget(QLabel('Puerto (estación terrena):'))
        barra.addWidget(self.combo_puertos)
        barra.addWidget(self.btn_refrescar)
        barra.addWidget(self.btn_conectar)
        barra.addWidget(self.lbl_estado)
        barra.addStretch()
        barra.addWidget(self.btn_paneles)
        barra.addWidget(self.lbl_csv)
        barra.addWidget(self.btn_csv)
        layout_principal.addLayout(barra)

        splitter = QSplitter(Qt.Horizontal)
        panel_izq = QWidget()
        layout_izq = QVBoxLayout(panel_izq)
        self.panel_valores = PanelValores()
        self.panel_botones = PanelBotones()
        self.panel_sesion = PanelSesion()
        self.panel_remoto = PanelOperacionesRemotas()
        layout_izq.addWidget(self.panel_valores)
        layout_izq.addWidget(self.panel_botones)
        layout_izq.addWidget(self.panel_sesion)
        layout_izq.addWidget(self.panel_remoto)
        layout_izq.addStretch()

        # --- Area de graficos ACOPLABLES: se puede arrastrar, apilar en
        # pestañas, flotar como ventana aparte, o cerrar cada panel, para
        # que el operador organice la pantalla como prefiera. ---
        self.graficos = ConjuntoGraficosVivo()
        self.vector3d = VectorAcc3D()

        self.ventana_docks = QMainWindow()
        self.ventana_docks.setWindowFlags(Qt.Widget)
        self.ventana_docks.setDockOptions(
            QMainWindow.AllowTabbedDocks | QMainWindow.AnimatedDocks | QMainWindow.AllowNestedDocks)

        self.dock_alt = QDockWidget('Altitud')
        self.dock_alt.setWidget(self.graficos.plot_alt)
        self.dock_vel = QDockWidget('Velocidad')
        self.dock_vel.setWidget(self.graficos.plot_vel)
        self.dock_acc = QDockWidget('Aceleración')
        self.dock_acc.setWidget(self.graficos.plot_acc)
        self.dock_giro = QDockWidget('Giroscopio')
        self.dock_giro.setWidget(self.graficos.plot_giro)
        self.dock_temp = QDockWidget('Temperatura')
        self.dock_temp.setWidget(self.graficos.plot_temp)
        self.dock_vector = QDockWidget('Vector de aceleración')
        self.dock_vector.setWidget(self.vector3d)

        self._restablecer_distribucion_paneles()

        menu_paneles = QMenu(self)
        for dock in self._docks():
            menu_paneles.addAction(dock.toggleViewAction())
        menu_paneles.addSeparator()
        accion_reset = menu_paneles.addAction('Restablecer distribución')
        accion_reset.triggered.connect(self._restablecer_distribucion_paneles)
        self.btn_paneles.setMenu(menu_paneles)

        splitter.addWidget(panel_izq)
        splitter.addWidget(self.ventana_docks)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        layout_principal.addWidget(splitter)

        self.btn_refrescar.clicked.connect(self._refrescar_puertos)
        self.btn_conectar.clicked.connect(self._toggle_conexion)
        self.btn_csv.clicked.connect(self._elegir_csv)
        self.panel_botones.solicitar_armado = self._enviar_armado
        self.panel_botones.solicitar_grabado = self._enviar_grabado
        self.panel_sesion.btn_finalizar.clicked.connect(self._finalizar_vuelo)
        self.panel_sesion.btn_nuevo.clicked.connect(self._nuevo_vuelo)

        self._refrescar_puertos()

    def _docks(self):
        return (self.dock_alt, self.dock_vel, self.dock_acc, self.dock_giro,
                self.dock_temp, self.dock_vector)

    def _restablecer_distribucion_paneles(self):
        vd = self.ventana_docks
        for dock in self._docks():
            dock.setFloating(False)
            vd.removeDockWidget(dock)
        vd.addDockWidget(Qt.LeftDockWidgetArea, self.dock_vector)
        vd.addDockWidget(Qt.TopDockWidgetArea, self.dock_alt)
        vd.splitDockWidget(self.dock_alt, self.dock_acc, Qt.Horizontal)
        vd.splitDockWidget(self.dock_alt, self.dock_vel, Qt.Vertical)
        vd.splitDockWidget(self.dock_vel, self.dock_giro, Qt.Vertical)
        vd.splitDockWidget(self.dock_acc, self.dock_temp, Qt.Vertical)
        for dock in self._docks():
            dock.setVisible(True)

    def _refrescar_puertos(self):
        actual = self.combo_puertos.currentText()
        self.combo_puertos.clear()
        puertos = listar_puertos()
        self.combo_puertos.addItems(puertos)
        if actual in puertos:
            self.combo_puertos.setCurrentText(actual)

    def _elegir_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Guardar telemetria en vivo como', 'vuelo.csv', 'CSV (*.csv)')
        if path:
            self._csv_path = path
            self.lbl_csv.setText(f'CSV: {path}')
            if self.hilo is not None:
                self._abrir_registro()

    # ---------------- registro CSV / sesiones ----------------
    def _abrir_registro(self):
        self._cerrar_registro()
        if not self._csv_path:
            return
        try:
            self.escritor_csv = EscritorCSV(self._csv_path)
            self._n_csv = 0
            self.panel_sesion.lbl_archivo.setText(f'Registrando en: {os.path.basename(self._csv_path)}')
        except Exception as e:
            QMessageBox.critical(self, 'Error CSV', f'No se pudo abrir el archivo CSV: {e}')
            self.escritor_csv = None

    def _cerrar_registro(self) -> Optional[str]:
        if self.escritor_csv is None:
            return None
        self.escritor_csv.cerrar()
        self.escritor_csv = None
        return self._csv_path

    def _reiniciar_estimadores(self):
        self.proc.reiniciar()
        self.panel_valores.reiniciar()
        self.graficos.reiniciar()

    def _finalizar_vuelo(self):
        if self.escritor_csv is None:
            QMessageBox.information(self, 'Finalizar vuelo', 'No hay ningún registro CSV activo.')
            return
        m = self._ultima_muestra
        if self.hilo is not None and m is not None and m.grabando:
            resp = QMessageBox.question(
                self, 'Grabación del cohete',
                'El cohete sigue GRABANDO en su EEPROM.\n¿Detener también la grabación del cohete?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            if resp == QMessageBox.Yes:
                self._enviar_grabado(False)
        n = self._n_csv
        path = self._cerrar_registro()
        self.panel_sesion.lbl_archivo.setText(f'Guardado: {os.path.basename(path)} ({n} muestras). '
                                              f'Usá "Nuevo vuelo" para seguir registrando.')
        resp = QMessageBox.question(
            self, 'Vuelo guardado',
            f'Vuelo guardado en:\n{path}\n({n} muestras)\n\n¿Abrirlo en la pestaña Análisis?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if resp == QMessageBox.Yes and self.al_analizar_archivo:
            self.al_analizar_archivo(path)

    def _nuevo_vuelo(self):
        if self.escritor_csv is not None:
            resp = QMessageBox.question(
                self, 'Nuevo vuelo',
                'Hay un vuelo registrándose. Se va a cerrar (queda guardado) y se empieza uno nuevo.\n¿Continuar?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            if resp != QMessageBox.Yes:
                return
            self._cerrar_registro()

        carpeta = os.path.dirname(self._csv_path) if self._csv_path else ''
        if not carpeta:
            carpeta = QFileDialog.getExistingDirectory(self, 'Carpeta donde guardar los vuelos')
            if not carpeta:
                return
        nombre = f"vuelo_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self._csv_path = os.path.join(carpeta, nombre)
        self.lbl_csv.setText(f'CSV: {self._csv_path}')
        self._reiniciar_estimadores()
        if self.hilo is not None:
            self._abrir_registro()
        else:
            self.panel_sesion.lbl_archivo.setText(f'Próximo registro: {nombre} (al conectar)')

    def _toggle_conexion(self):
        if self.hilo is None:
            puerto = self.combo_puertos.currentText()
            if not puerto:
                QMessageBox.warning(self, 'Sin puerto', 'Elegi un puerto serie primero.')
                return

            self._reiniciar_estimadores()
            self._abrir_registro()

            self.hilo = HiloEnlaceVivo(puerto)
            self.hilo.muestra_recibida.connect(self._on_muestra)
            self.hilo.error.connect(self._on_error)
            self.hilo.conectado.connect(self._on_conectado)
            self.hilo.start()
            self.btn_conectar.setText('Desconectar')
        else:
            self.hilo.detener()
            self.hilo.wait(1000)
            self.hilo = None
            if self._cerrar_registro():
                self.panel_sesion.lbl_archivo.setText(
                    f'Guardado: {os.path.basename(self._csv_path)} ({self._n_csv} muestras)')
            self.btn_conectar.setText('Conectar')
            self.lbl_estado.setText('Desconectado')
            self.lbl_estado.setStyleSheet('color: red; font-weight: bold;')

    def _on_conectado(self, ok: bool):
        if ok:
            self.lbl_estado.setText('Conectado')
            self.lbl_estado.setStyleSheet('color: green; font-weight: bold;')
        else:
            self.lbl_estado.setText('Desconectado')
            self.lbl_estado.setStyleSheet('color: red; font-weight: bold;')

    def _on_error(self, msg: str):
        QMessageBox.warning(self, 'Enlace en vivo', msg)

    def _on_muestra(self, m: Muestra):
        # Tiempo continuo + descarte de frames corruptos + filtros
        est = self.proc.procesar(m)
        if est is None:
            self.panel_valores.mostrar_descartados(self.proc.descartadas)
            return

        self._ultima_muestra = m
        self.panel_valores.actualizar(m, est, self.proc.descartadas)
        self.panel_botones.actualizar_estado(m)
        self.graficos.agregar(m, est)
        self.vector3d.actualizar(m)
        # El CSV guarda los datos CRUDOS (los filtros se recalculan en Analisis)
        if self.escritor_csv:
            self.escritor_csv.escribir(m)
            self._n_csv += 1

    def _enviar_armado(self, valor: bool):
        if self.hilo:
            self.hilo.enviar_comando(comando_armado(valor))

    def _enviar_grabado(self, valor: bool):
        if self.hilo:
            self.hilo.enviar_comando(comando_grabado(valor))

    def cerrar(self):
        if self.hilo:
            self.hilo.detener()
            self.hilo.wait(1000)
        if self.escritor_csv:
            self.escritor_csv.cerrar()


# =====================================================================
#  Pestana: Descarga del cohete (USB directo)
# =====================================================================
class SubTabDescargaEeprom(QWidget):
    """Descarga masiva de la EEPROM, consulta de estado y borrado --
    todo por USB directo al cohete (no pasa por LoRa)."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.hilo: Optional[HiloDescarga] = None
        self.hilo_cmd: Optional[HiloComandoRapido] = None
        self.muestras_descargadas: List[Muestra] = []
        self.descarga_confirmada = False  # True tras una descarga exitosa en esta sesion
        self.al_analizar = None  # callback(List[Muestra]) inyectado desde MainWindow

        layout = QVBoxLayout(self)

        barra = QHBoxLayout()
        self.combo_puertos = QComboBox()
        self.btn_refrescar = QPushButton('Actualizar')
        barra.addWidget(QLabel('Puerto (USB directo al cohete):'))
        barra.addWidget(self.combo_puertos)
        barra.addWidget(self.btn_refrescar)
        layout.addLayout(barra)

        aviso = QLabel(
            'Conecta el cable USB directo al ESP32 del cohete (no a la estación terrena). '
            'Estas operaciones NO pasan por LoRa.'
        )
        aviso.setStyleSheet('color: #555;')
        aviso.setWordWrap(True)
        layout.addWidget(aviso)

        fila_botones = QHBoxLayout()
        self.btn_descargar = QPushButton('Descargar EEPROM del cohete')
        self.btn_estado = QPushButton('Consultar Estado')
        self.btn_giro = QPushButton('Recalibrar giróscopo')
        self.btn_giro.setToolTip('Con el cohete QUIETO: vuelve a medir el offset del giróscopo (comando G).')
        self.btn_borrar = QPushButton('Borrar Memoria')
        self.btn_borrar.setStyleSheet('color: #b00020;')
        fila_botones.addWidget(self.btn_descargar)
        fila_botones.addWidget(self.btn_estado)
        fila_botones.addWidget(self.btn_giro)
        fila_botones.addWidget(self.btn_borrar)
        layout.addLayout(fila_botones)

        self.progreso = QProgressBar()
        layout.addWidget(self.progreso)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)

        botones_post = QHBoxLayout()
        self.btn_guardar = QPushButton('Guardar como CSV...')
        self.btn_guardar.setEnabled(False)
        self.btn_analizar = QPushButton('Analizar esta descarga')
        self.btn_analizar.setEnabled(False)
        botones_post.addWidget(self.btn_guardar)
        botones_post.addWidget(self.btn_analizar)
        botones_post.addStretch()
        layout.addLayout(botones_post)

        nota_debug = QLabel(
            'Para la consola de debug interactiva y la telemetría en vivo por USB '
            'directo (acelerómetro, giroscopio, altitud, etc. sin pasar por la '
            'estación terrena), ver la pestaña "Debug / Telemetría (USB directo)".'
        )
        nota_debug.setWordWrap(True)
        nota_debug.setStyleSheet('color: #777; font-size: 11px;')
        layout.addWidget(nota_debug)

        # --- Operaciones remotas (LoRa) -- PIN: trabajo futuro ---
        self.panel_remoto = PanelOperacionesRemotas()
        layout.addWidget(self.panel_remoto)

        self.btn_refrescar.clicked.connect(self._refrescar_puertos)
        self.btn_descargar.clicked.connect(self._iniciar_descarga)
        self.btn_estado.clicked.connect(self._consultar_estado)
        self.btn_giro.clicked.connect(self._recalibrar_giro)
        self.btn_borrar.clicked.connect(self._borrar_memoria)
        self.btn_guardar.clicked.connect(self._guardar_csv)
        self.btn_analizar.clicked.connect(self._analizar)

        self._refrescar_puertos()

    def _refrescar_puertos(self):
        actual = self.combo_puertos.currentText()
        self.combo_puertos.clear()
        puertos = listar_puertos()
        self.combo_puertos.addItems(puertos)
        if actual in puertos:
            self.combo_puertos.setCurrentText(actual)

    def _puerto_seleccionado(self) -> Optional[str]:
        puerto = self.combo_puertos.currentText()
        if not puerto:
            QMessageBox.warning(self, 'Sin puerto', 'Elegi un puerto serie primero.')
            return None
        return puerto

    def _bloquear_botones(self, bloquear: bool):
        for b in (self.btn_descargar, self.btn_estado, self.btn_giro, self.btn_borrar):
            b.setEnabled(not bloquear)

    def _iniciar_descarga(self):
        puerto = self._puerto_seleccionado()
        if not puerto:
            return

        self.log.clear()
        self.progreso.setValue(0)
        self.btn_guardar.setEnabled(False)
        self.btn_analizar.setEnabled(False)
        self._bloquear_botones(True)
        self.log.appendPlainText(f'Conectando a {puerto}...')

        self.hilo = HiloDescarga(puerto)
        self.hilo.progreso.connect(self._on_progreso)
        self.hilo.terminado.connect(self._on_terminado)
        self.hilo.error.connect(self._on_error_descarga)
        self.hilo.start()

    def _on_progreso(self, actual: int, total: int):
        if total > 0:
            self.progreso.setMaximum(total)
            self.progreso.setValue(actual)
        self.log.appendPlainText(f'Registros leídos: {actual}' + (f' / {total}' if total > 0 else ''))

    def _on_terminado(self, muestras: list):
        self.muestras_descargadas = muestras
        self.log.appendPlainText(f'Descarga completa: {len(muestras)} registros.')
        self.btn_guardar.setEnabled(len(muestras) > 0)
        self.btn_analizar.setEnabled(len(muestras) > 0)
        self._bloquear_botones(False)
        if len(muestras) > 0:
            self.descarga_confirmada = True

    def _on_error_descarga(self, msg: str):
        self.log.appendPlainText(f'ERROR: {msg}')
        self._bloquear_botones(False)
        QMessageBox.warning(self, 'Descarga', msg)

    def _guardar_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Guardar descarga como', 'descarga.csv', 'CSV (*.csv)')
        if path:
            guardar_csv(path, self.muestras_descargadas)
            QMessageBox.information(self, 'Guardado', f'Guardado en {path}')

    def _analizar(self):
        if self.al_analizar:
            self.al_analizar(self.muestras_descargadas)

    def _consultar_estado(self):
        puerto = self._puerto_seleccionado()
        if not puerto:
            return
        self.log.appendPlainText(f'Consultando estado en {puerto}...')
        self._bloquear_botones(True)
        self.hilo_cmd = HiloComandoRapido(puerto, modo='estado')
        self.hilo_cmd.resultado.connect(self._on_resultado_cmd)
        self.hilo_cmd.error.connect(self._on_error_cmd)
        self.hilo_cmd.start()

    def _recalibrar_giro(self):
        puerto = self._puerto_seleccionado()
        if not puerto:
            return
        self.log.appendPlainText(f'Recalibrando giróscopo en {puerto} (dejá el cohete quieto)...')
        self._bloquear_botones(True)
        self.hilo_cmd = HiloComandoRapido(puerto, modo='giroscopio')
        self.hilo_cmd.resultado.connect(self._on_resultado_cmd)
        self.hilo_cmd.error.connect(self._on_error_cmd)
        self.hilo_cmd.start()

    def _borrar_memoria(self):
        puerto = self._puerto_seleccionado()
        if not puerto:
            return

        if not self.descarga_confirmada:
            resp = QMessageBox.warning(
                self, 'Borrar memoria sin descarga previa',
                'Todavía no descargaste datos del cohete en esta sesión.\n\n'
                '¿Confirmás que ya tenés los datos respaldados de otra forma y '
                'querés borrar la EEPROM de todos modos?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if resp != QMessageBox.Yes:
                return
        else:
            resp = QMessageBox.question(
                self, 'Confirmar borrado',
                'Esto borra el contenido de la EEPROM del cohete (ya descargaste '
                'los datos en esta sesión). ¿Confirmás el borrado?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if resp != QMessageBox.Yes:
                return

        self.log.appendPlainText(f'Solicitando borrado de memoria en {puerto}...')
        self._bloquear_botones(True)
        self.hilo_cmd = HiloComandoRapido(puerto, modo='borrar')
        self.hilo_cmd.resultado.connect(self._on_resultado_cmd)
        self.hilo_cmd.error.connect(self._on_error_cmd)
        self.hilo_cmd.start()

    def _on_resultado_cmd(self, texto: str):
        self.log.appendPlainText(texto)
        self._bloquear_botones(False)
        if 'borrada' in texto.lower():
            self.descarga_confirmada = False  # ya no hay nada nuevo respaldado tras borrar

    def _on_error_cmd(self, msg: str):
        self.log.appendPlainText(f'ERROR: {msg}')
        self._bloquear_botones(False)
        QMessageBox.warning(self, 'Comando', msg)


# =====================================================================
#  Panel de telemetria (reusa PanelValores/ConjuntoGraficosVivo/VectorAcc3D
#  en un layout simple, sin docks, para la vista de USB directo)
# =====================================================================
class PanelTelemetriaDirecta(QWidget):
    """
    Igual que la pestana 'En Vivo', pero SIN los botones de armado/grabado:
    la conexion USB directa al cohete en modo debug no tiene ningun canal
    de comando (armar/grabar solo existen via LoRa desde la estacion
    terrena), asi que esta vista es de solo lectura.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)

        self.panel_valores = PanelValores()
        self.panel_valores.setMaximumWidth(340)
        layout.addWidget(self.panel_valores)

        self.graficos = ConjuntoGraficosVivo()
        self.vector3d = VectorAcc3D()
        self.vector3d.setMaximumWidth(260)

        columna_graficos = QVBoxLayout()
        fila_arriba = QHBoxLayout()
        fila_arriba.addWidget(self.graficos.plot_alt)
        fila_arriba.addWidget(self.graficos.plot_vel)
        fila_arriba.addWidget(self.graficos.plot_acc)
        fila_abajo = QHBoxLayout()
        fila_abajo.addWidget(self.graficos.plot_giro)
        fila_abajo.addWidget(self.graficos.plot_temp)
        columna_graficos.addLayout(fila_arriba)
        columna_graficos.addLayout(fila_abajo)

        contenedor_graficos = QWidget()
        contenedor_graficos.setLayout(columna_graficos)
        layout.addWidget(contenedor_graficos, stretch=1)
        layout.addWidget(self.vector3d)

        self.proc = ProcesadorVivo()

    def reiniciar(self):
        self.proc.reiniciar()
        self.panel_valores.reiniciar()
        self.graficos.reiniciar()

    def procesar_muestra(self, m: Muestra):
        est = self.proc.procesar(m)
        if est is None:
            self.panel_valores.mostrar_descartados(self.proc.descartadas)
            return
        self.panel_valores.actualizar(m, est, self.proc.descartadas)
        self.graficos.agregar(m, est)
        self.vector3d.actualizar(m)


# =====================================================================
#  Sub-pestana: consola de debug + telemetria en vivo por USB directo
# =====================================================================
class SubTabDebugUSB(QWidget):
    """
    Una sola conexion persistente al modo debug del cohete (comando 'T').
    Arranca mostrando una consola de texto con botones rapidos para cada
    opcion del menu del firmware. Al enviar '9' (telemetria en vivo), la
    vista cambia a los graficos/gauges -- mismo truco de deteccion de
    tramas que usa la pestana 'En Vivo', reaprovechado sobre esta misma
    conexion. 'Q' vuelve a la consola de texto.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.hilo: Optional[HiloUSBDirecto] = None

        layout = QVBoxLayout(self)

        barra = QHBoxLayout()
        self.combo_puertos = QComboBox()
        self.btn_refrescar = QPushButton('Actualizar')
        self.btn_conectar = QPushButton('Conectar')
        self.lbl_estado = QLabel('Desconectado')
        self.lbl_estado.setStyleSheet('color: red; font-weight: bold;')
        barra.addWidget(QLabel('Puerto (USB directo al cohete):'))
        barra.addWidget(self.combo_puertos)
        barra.addWidget(self.btn_refrescar)
        barra.addWidget(self.btn_conectar)
        barra.addWidget(self.lbl_estado)
        barra.addStretch()
        layout.addLayout(barra)

        aviso = QLabel(
            'Requiere el firmware con modo debug (comando \'T\'). No se puede entrar '
            'si el cohete esta armado o en vuelo -- eso lo decide el propio firmware.'
        )
        aviso.setWordWrap(True)
        aviso.setStyleSheet('color: #555;')
        layout.addWidget(aviso)

        # --- Fila de botones rapidos del menu del firmware ---
        fila_menu = QHBoxLayout()
        self.btn_entrar = QPushButton("Entrar ('T')")
        self.btn_eeprom = QPushButton('1: EEPROM')
        self.btn_mpu = QPushButton('2: MPU6050')
        self.btn_bmp = QPushButton('3: BMP280')
        self.btn_continuidad = QPushButton('4: Continuidad')
        self.btn_disparo = QPushButton('5: DISPARO')
        self.btn_disparo.setStyleSheet('color: #b00020; font-weight: bold;')
        self.btn_lora_tx = QPushButton('6: LoRa TX')
        self.btn_lora_rx = QPushButton('7: LoRa RX')
        self.btn_global = QPushButton('8: Test global')
        self.btn_telemetria = QPushButton('9: Telemetría en vivo')
        self.btn_salir = QPushButton("Salir ('Q')")

        for b in (self.btn_entrar, self.btn_eeprom, self.btn_mpu, self.btn_bmp,
                  self.btn_continuidad, self.btn_disparo, self.btn_lora_tx,
                  self.btn_lora_rx, self.btn_global, self.btn_telemetria, self.btn_salir):
            fila_menu.addWidget(b)
        layout.addLayout(fila_menu)

        # --- Vista apilada: consola de texto <-> telemetria en vivo ---
        self.stack = QStackedWidget()

        self.consola = QPlainTextEdit()
        self.consola.setReadOnly(True)
        self.consola.setStyleSheet('font-family: monospace; background-color: #1e1e1e; color: #d4d4d4;')
        self.stack.addWidget(self.consola)

        self.panel_telemetria = PanelTelemetriaDirecta()
        self.stack.addWidget(self.panel_telemetria)

        layout.addWidget(self.stack, stretch=1)

        # --- Entrada de texto libre (fallback generico) ---
        fila_envio = QHBoxLayout()
        self.campo_envio = QPlainTextEdit()
        self.campo_envio.setMaximumHeight(32)
        self.btn_enviar = QPushButton('Enviar')
        fila_envio.addWidget(self.campo_envio)
        fila_envio.addWidget(self.btn_enviar)
        layout.addLayout(fila_envio)

        self._habilitar_botones_menu(False)

        self.btn_refrescar.clicked.connect(self._refrescar_puertos)
        self.btn_conectar.clicked.connect(self._toggle_conexion)
        self.btn_entrar.clicked.connect(lambda: self._enviar_rapido(b'T'))
        self.btn_eeprom.clicked.connect(lambda: self._enviar_rapido(b'1'))
        self.btn_mpu.clicked.connect(lambda: self._enviar_rapido(b'2'))
        self.btn_bmp.clicked.connect(lambda: self._enviar_rapido(b'3'))
        self.btn_continuidad.clicked.connect(lambda: self._enviar_rapido(b'4'))
        self.btn_disparo.clicked.connect(self._on_disparo)
        self.btn_lora_tx.clicked.connect(lambda: self._enviar_rapido(b'6'))
        self.btn_lora_rx.clicked.connect(lambda: self._enviar_rapido(b'7'))
        self.btn_global.clicked.connect(lambda: self._enviar_rapido(b'8'))
        self.btn_telemetria.clicked.connect(self._on_telemetria)
        self.btn_salir.clicked.connect(self._on_salir)
        self.btn_enviar.clicked.connect(self._on_enviar_libre)

        self._refrescar_puertos()

    def _habilitar_botones_menu(self, habilitar: bool):
        for b in (self.btn_entrar, self.btn_eeprom, self.btn_mpu, self.btn_bmp,
                  self.btn_continuidad, self.btn_disparo, self.btn_lora_tx,
                  self.btn_lora_rx, self.btn_global, self.btn_telemetria,
                  self.btn_salir, self.btn_enviar):
            b.setEnabled(habilitar)

    def _refrescar_puertos(self):
        actual = self.combo_puertos.currentText()
        self.combo_puertos.clear()
        puertos = listar_puertos()
        self.combo_puertos.addItems(puertos)
        if actual in puertos:
            self.combo_puertos.setCurrentText(actual)

    def _toggle_conexion(self):
        if self.hilo is None:
            puerto = self.combo_puertos.currentText()
            if not puerto:
                QMessageBox.warning(self, 'Sin puerto', 'Elegi un puerto serie primero.')
                return
            self.consola.clear()
            self.panel_telemetria.reiniciar()
            self.stack.setCurrentWidget(self.consola)

            self.hilo = HiloUSBDirecto(puerto)
            self.hilo.texto_recibido.connect(self._on_texto)
            self.hilo.muestra_recibida.connect(self._on_muestra)
            self.hilo.error.connect(self._on_error)
            self.hilo.conectado.connect(self._on_conectado)
            self.hilo.start()
            self.btn_conectar.setText('Desconectar')
        else:
            self.hilo.detener()
            self.hilo.wait(1000)
            self.hilo = None
            self.btn_conectar.setText('Conectar')
            self._habilitar_botones_menu(False)
            self.lbl_estado.setText('Desconectado')
            self.lbl_estado.setStyleSheet('color: red; font-weight: bold;')

    def _on_conectado(self, ok: bool):
        if ok:
            self.lbl_estado.setText('Conectado')
            self.lbl_estado.setStyleSheet('color: green; font-weight: bold;')
            self._habilitar_botones_menu(True)
        else:
            self.lbl_estado.setText('Desconectado')
            self.lbl_estado.setStyleSheet('color: red; font-weight: bold;')
            self._habilitar_botones_menu(False)

    def _on_error(self, msg: str):
        self.consola.appendPlainText(f'\n[ERROR] {msg}\n')
        QMessageBox.warning(self, 'USB directo', msg)

    def _on_texto(self, texto: str):
        self.consola.moveCursor(QTextCursor.MoveOperation.End)
        self.consola.insertPlainText(texto)
        self.consola.moveCursor(QTextCursor.MoveOperation.End)

    def _on_muestra(self, m: Muestra):
        self.panel_telemetria.procesar_muestra(m)

    def _enviar_rapido(self, datos: bytes):
        if self.hilo:
            self.hilo.enviar(datos)

    def _on_telemetria(self):
        if not self.hilo:
            return
        self.hilo.enviar(b'9')
        self.hilo.modo = 'telemetria'
        self.panel_telemetria.reiniciar()
        self.stack.setCurrentWidget(self.panel_telemetria)

    def _on_salir(self):
        if not self.hilo:
            return
        self.hilo.enviar(b'Q')
        self.hilo.modo = 'texto'
        self.stack.setCurrentWidget(self.consola)

    def _on_disparo(self):
        resp = QMessageBox.warning(
            self, 'Prueba de disparo del E-Match',
            'ADVERTENCIA: esto energiza la salida real del MOSFET por 150 ms.\n\n'
            'Si hay un igniter real conectado, SE VA A DISPARAR.\n\n'
            'Si solo queres probar la electronica, desconecta el igniter real '
            'y usa una carga de prueba (LED + resistencia) antes de continuar.',
            QMessageBox.Ok | QMessageBox.Cancel, QMessageBox.Cancel)
        if resp != QMessageBox.Ok:
            return

        texto, ok = QInputDialog.getText(
            self, 'Confirmar disparo',
            'Para confirmar, escribi exactamente:  DISPARAR')
        if not ok or texto != 'DISPARAR':
            self.consola.appendPlainText('\n[Disparo de prueba cancelado desde la PC]\n')
            return

        if self.hilo:
            self.hilo.enviar(b'5')
            self.hilo.enviar(b'DISPARAR\n')

    def _on_enviar_libre(self):
        if self.hilo:
            texto = self.campo_envio.toPlainText()
            if texto:
                self.hilo.enviar(texto.encode('ascii', errors='ignore') + b'\n')
                self.campo_envio.clear()

    def cerrar(self):
        if self.hilo:
            self.hilo.detener()
            self.hilo.wait(1000)


# =====================================================================
#  Pestana: Descarga del cohete (envoltorio con sub-pestanas)
# =====================================================================
class PestanaDescarga(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        subtabs = QTabWidget()
        self.sub_descarga = SubTabDescargaEeprom()
        self.sub_debug = SubTabDebugUSB()
        subtabs.addTab(self.sub_descarga, 'Descarga / Estado / Borrado')
        subtabs.addTab(self.sub_debug, 'Debug / Telemetría (USB directo)')
        layout.addWidget(subtabs)

    # Proxies para que MainWindow siga usando la misma interfaz que antes
    # (self.tab_descarga.al_analizar = ...), aunque ahora vive un nivel
    # mas adentro, dentro de la sub-pestana de descarga.
    @property
    def al_analizar(self):
        return self.sub_descarga.al_analizar

    @al_analizar.setter
    def al_analizar(self, callback):
        self.sub_descarga.al_analizar = callback

    @property
    def muestras_descargadas(self):
        return self.sub_descarga.muestras_descargadas

    def cerrar(self):
        self.sub_debug.cerrar()


# =====================================================================
#  Sub-pestanas de Analisis
# =====================================================================
class SubTabFigura(QWidget):
    """Contenedor generico: una figura de matplotlib a pantalla completa
    dentro de la sub-pestaña, mas una nota opcional arriba."""
    def __init__(self, nota: Optional[str] = None, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        if nota:
            lbl = QLabel(nota)
            lbl.setWordWrap(True)
            lbl.setStyleSheet('color: #a05a00; background-color: #fff3e0; padding: 6px; border-radius: 4px;')
            layout.addWidget(lbl)
        self.fig = Figure(figsize=(8, 6))
        self.canvas = FigureCanvasQTAgg(self.fig)
        layout.addWidget(self.canvas)


class PestanaAnalisis(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.muestras: List[Muestra] = []

        layout = QVBoxLayout(self)
        barra = QHBoxLayout()
        self.btn_abrir = QPushButton('Abrir CSV...')
        barra.addWidget(self.btn_abrir)
        barra.addStretch()
        layout.addLayout(barra)

        self.resumen_label = QLabel('Abrí un archivo CSV (o bajá los datos del cohete) para ver el resumen del vuelo.')
        self.resumen_label.setStyleSheet('font-family: monospace;')
        self.resumen_label.setWordWrap(True)
        layout.addWidget(self.resumen_label)

        self.subtabs = QTabWidget()
        layout.addWidget(self.subtabs)

        self.tab_resumen = SubTabFigura()
        self.tab_acel = SubTabFigura()
        self.tab_giro = SubTabFigura()
        self.tab_alt_temp = SubTabFigura()
        self.tab_inercial = SubTabFigura(
            nota='La velocidad y altura "inerciales" se calculan integrando el acelerómetro dos '
                 'veces: acumulan error con el tiempo (derivan) y se muestran solo como '
                 'comparación frente a los valores barométricos, no como referencia principal.'
        )

        self.subtabs.addTab(self.tab_resumen, 'Resumen')
        self.subtabs.addTab(self.tab_acel, 'Aceleración')
        self.subtabs.addTab(self.tab_giro, 'Giroscopio')
        self.subtabs.addTab(self.tab_alt_temp, 'Altitud y Temperatura')
        self.subtabs.addTab(self.tab_inercial, 'Inercial y Derivados')

        self.btn_abrir.clicked.connect(self._abrir_csv)

    def cargar_muestras(self, muestras: List[Muestra]):
        self.muestras = muestras
        self._actualizar()

    def abrir_archivo(self, path: str):
        try:
            self.muestras = leer_csv(path)
        except Exception as e:
            QMessageBox.critical(self, 'Error', f'No se pudo leer el CSV: {e}')
            return
        self._actualizar()

    def _abrir_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Abrir CSV de vuelo', '', 'CSV (*.csv)')
        if not path:
            return
        self.abrir_archivo(path)

    def _actualizar(self):
        if not self.muestras:
            return

        # Tiempo continuo: descarta frames corruptos y une sesiones si el
        # cohete se reinicio (evita ejes de tiempo de "millones de segundos").
        muestras, t_arr, validador = normalizar_tiempo(self.muestras)
        if len(muestras) < 2:
            QMessageBox.warning(self, 'Análisis', 'No hay suficientes muestras válidas en el archivo.')
            return
        t = list(t_arr)

        # Altitud y velocidad: Kalman + suavizador (baro + acelerometro)
        alt_s, vel = suavizar_vertical(muestras, t_arr)
        vel_cruda = calcular_velocidad_vertical(muestras)
        # Giroscopio sin offset (estimado antes del despegue)
        giro_dps, bias_giro = giro_sin_bias(muestras)
        girox, giroy, giroz = giro_dps[:, 0], giro_dps[:, 1], giro_dps[:, 2]
        giro_mag = (girox ** 2 + giroy ** 2 + giroz ** 2) ** 0.5

        r = resumen_de_vuelo(muestras, vel=vel, alt_suavizada=alt_s, giro_mag_dps=giro_mag)
        alt = [m.altitude for m in muestras]
        temp = [m.temp_c for m in muestras]
        acc_g = [acc_cuentas_a_g(m.acc_mag) for m in muestras]
        accx = [acc_cuentas_a_g(m.acc_x) for m in muestras]
        accy = [acc_cuentas_a_g(m.acc_y) for m in muestras]
        accz = [acc_cuentas_a_g(m.acc_z) for m in muestras]
        v_in, h_in = calcular_navegacion_inercial(muestras)
        mach = calcular_mach(muestras, vel)
        q_din = calcular_presion_dinamica_pa(muestras, vel)

        lineas = [
            f"Muestras: {r.n_muestras}    Duración: {r.duracion_s:.1f} s",
            f"Apogeo (filtrado): {r.altitud_max_m:.1f} m  (t = {r.t_apogeo_s:.2f} s)    "
            f"Máximo crudo del barómetro: {max(alt):.1f} m",
            f"Velocidad máxima (ascenso): {r.velocidad_max_ms:.1f} m/s    "
            f"Velocidad mínima (descenso): {r.velocidad_min_ms:.1f} m/s",
            f"Aceleración máxima: {r.acc_mag_max_g:.1f} g  ({r.acc_mag_max_ms2:.1f} m/s²)",
            f"Velocidad angular máxima: {r.veloc_angular_max_dps:.1f} °/s",
            f"Mach máximo: {r.mach_max:.3f}    Presión dinámica máxima: {r.presion_dinamica_max_pa:.0f} Pa",
        ]
        lineas.append(f"Offset del giroscopio restado (°/s) X/Y/Z: "
                      f"{bias_giro[0]:.2f} / {bias_giro[1]:.2f} / {bias_giro[2]:.2f}")
        if validador.descartadas or validador.reinicios:
            lineas.append(f"Muestras descartadas por datos corruptos: {validador.descartadas}    "
                          f"Reinicios del cohete detectados: {validador.reinicios}")
        if r.angulo_inclinacion_deg is not None:
            lineas.append(f"Inclinación en rampa (respecto de la vertical): {r.angulo_inclinacion_deg:.1f}°")
        if r.t_despegue_s is not None:
            lineas.append(f"Despegue detectado en t = {r.t_despegue_s:.2f} s")
        else:
            lineas.append("Despegue: no detectado en estos datos")
        if r.t_desplegado_s is not None:
            lineas.append(f"Paracaídas desplegado en t = {r.t_desplegado_s:.2f} s")
        if r.paquetes_perdidos_pct is not None:
            lineas.append(
                f"Paquetes: {r.paquetes_totales} recibidos de ~{r.paquetes_esperados} esperados "
                f"({r.paquetes_perdidos_pct:.1f}% perdidos)"
            )
        self.resumen_label.setText('\n'.join(lineas))

        # --- Resumen: altitud / velocidad / aceleracion ---
        fig = self.tab_resumen.fig
        fig.clear()
        ax1 = fig.add_subplot(3, 1, 1)
        ax1.plot(t, alt, '.', color='0.6', markersize=2, label='Barómetro (crudo)')
        ax1.plot(t, alt_s, 'b-', label='Filtrada')
        ax1.set_ylabel('Altitud (m)')
        ax1.legend(loc='upper right', fontsize=8)
        ax1.grid(True, alpha=0.3)
        ax2 = fig.add_subplot(3, 1, 2, sharex=ax1)
        ax2.plot(t, vel_cruda, '-', color='0.75', linewidth=0.6, label='Derivada directa (ruidosa)')
        ax2.plot(t, vel, 'g-', label='Filtrada (baro + acelerómetro)')
        ax2.set_ylabel('Velocidad (m/s)')
        ax2.legend(loc='upper right', fontsize=8)
        ax2.grid(True, alpha=0.3)
        ax3 = fig.add_subplot(3, 1, 3, sharex=ax1)
        ax3.plot(t, acc_g, 'r-')
        ax3.set_ylabel('Aceleración (g)')
        ax3.set_xlabel('Tiempo (s)')
        ax3.grid(True, alpha=0.3)
        fig.tight_layout()
        self.tab_resumen.canvas.draw_idle()

        # --- Aceleracion: 3 ejes + magnitud ---
        fig = self.tab_acel.fig
        fig.clear()
        axa = fig.add_subplot(2, 1, 1)
        axa.plot(t, accx, 'r-', label='X', linewidth=0.8)
        axa.plot(t, accy, 'g-', label='Y', linewidth=0.8)
        axa.plot(t, accz, 'b-', label='Z', linewidth=0.8)
        axa.set_ylabel('Aceleración (g)')
        axa.legend(loc='upper right')
        axa.grid(True, alpha=0.3)
        axb = fig.add_subplot(2, 1, 2, sharex=axa)
        axb.plot(t, acc_g, 'k-')
        axb.set_ylabel('Magnitud (g)')
        axb.set_xlabel('Tiempo (s)')
        axb.grid(True, alpha=0.3)
        fig.tight_layout()
        self.tab_acel.canvas.draw_idle()

        # --- Giroscopio: 3 ejes + magnitud ---
        fig = self.tab_giro.fig
        fig.clear()
        axa = fig.add_subplot(2, 1, 1)
        axa.plot(t, girox, 'r-', label='X', linewidth=0.8)
        axa.plot(t, giroy, 'g-', label='Y', linewidth=0.8)
        axa.plot(t, giroz, 'b-', label='Z', linewidth=0.8)
        axa.set_ylabel('Velocidad angular, sin offset (°/s)')
        axa.legend(loc='upper right')
        axa.grid(True, alpha=0.3)
        axb = fig.add_subplot(2, 1, 2, sharex=axa)
        axb.plot(t, giro_mag, 'k-')
        axb.set_ylabel('Magnitud (°/s)')
        axb.set_xlabel('Tiempo (s)')
        axb.grid(True, alpha=0.3)
        fig.tight_layout()
        self.tab_giro.canvas.draw_idle()

        # --- Altitud (barometrica) y Temperatura ---
        fig = self.tab_alt_temp.fig
        fig.clear()
        axa = fig.add_subplot(2, 1, 1)
        axa.plot(t, alt, '.', color='0.6', markersize=2, label='Cruda')
        axa.plot(t, alt_s, 'b-', label='Filtrada')
        axa.legend(loc='upper right', fontsize=8)
        axa.set_ylabel('Altitud barométrica (m)')
        axa.grid(True, alpha=0.3)
        axb = fig.add_subplot(2, 1, 2, sharex=axa)
        axb.plot(t, temp, 'm-')
        axb.set_ylabel('Temperatura (°C)')
        axb.set_xlabel('Tiempo (s)')
        axb.grid(True, alpha=0.3)
        fig.tight_layout()
        self.tab_alt_temp.canvas.draw_idle()

        # --- Inercial y derivados ---
        fig = self.tab_inercial.fig
        fig.clear()
        ax1 = fig.add_subplot(4, 1, 1)
        ax1.plot(t, alt_s, 'b-', label='Barométrica (filtrada)')
        ax1.plot(t, h_in, 'r--', label='Inercial (deriva)', linewidth=0.8)
        ax1.set_ylabel('Altitud (m)')
        ax1.legend(loc='upper right', fontsize=8)
        ax1.grid(True, alpha=0.3)
        ax2 = fig.add_subplot(4, 1, 2, sharex=ax1)
        ax2.plot(t, vel, 'g-', label='Filtrada (baro + acel.)')
        ax2.plot(t, v_in, 'r--', label='Inercial (deriva)', linewidth=0.8)
        ax2.set_ylabel('Velocidad (m/s)')
        ax2.legend(loc='upper right', fontsize=8)
        ax2.grid(True, alpha=0.3)
        ax3 = fig.add_subplot(4, 1, 3, sharex=ax1)
        ax3.plot(t, mach, 'k-')
        ax3.set_ylabel('Mach')
        ax3.grid(True, alpha=0.3)
        ax4 = fig.add_subplot(4, 1, 4, sharex=ax1)
        ax4.plot(t, q_din, 'c-')
        ax4.set_ylabel('Presión dinámica (Pa)')
        ax4.set_xlabel('Tiempo (s)')
        ax4.grid(True, alpha=0.3)
        fig.tight_layout()
        self.tab_inercial.canvas.draw_idle()


# =====================================================================
#  Ventana principal
# =====================================================================
class VentanaPrincipal(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f'Estación de Control - Cohete  v{__version__}')
        self.resize(1400, 900)

        self.tabs = QTabWidget()
        self.tab_vivo = PestanaEnVivo()
        self.tab_descarga = PestanaDescarga()
        self.tab_analisis = PestanaAnalisis()

        self.tabs.addTab(self.tab_vivo, 'En Vivo')
        self.tabs.addTab(self.tab_descarga, 'Descarga del cohete')
        self.tabs.addTab(self.tab_analisis, 'Análisis')

        self.setCentralWidget(self.tabs)

        self.tab_descarga.al_analizar = self._analizar_descarga
        self.tab_vivo.al_analizar_archivo = self._analizar_archivo

    def _analizar_descarga(self, muestras: List[Muestra]):
        self.tab_analisis.cargar_muestras(muestras)
        self.tabs.setCurrentWidget(self.tab_analisis)

    def _analizar_archivo(self, path: str):
        self.tab_analisis.abrir_archivo(path)
        self.tabs.setCurrentWidget(self.tab_analisis)

    def closeEvent(self, event):
        self.tab_vivo.cerrar()
        self.tab_descarga.cerrar()
        event.accept()


def main():
    app = QApplication(sys.argv)
    ventana = VentanaPrincipal()
    ventana.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
