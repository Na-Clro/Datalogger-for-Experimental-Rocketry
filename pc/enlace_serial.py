"""
enlace_serial.py
Hilo dedicado (QThread) para el enlace en vivo con la estacion terrena por
USB. Lee bytes crudos y los pasa por el ExtractorDeFrames, emite una senal
por cada Muestra valida. El envio de comandos (armado/grabado) tambien pasa
por este mismo hilo, encolado, para no llamar a pyserial desde dos hilos
distintos al mismo tiempo.
"""
import time
import queue

import serial

from PySide6.QtCore import QThread, Signal

from protocolo import ExtractorDeFrames


class HiloEnlaceVivo(QThread):
    muestra_recibida = Signal(object)  # Muestra
    error = Signal(str)
    conectado = Signal(bool)

    def __init__(self, puerto: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self.puerto = puerto
        self.baudrate = baudrate
        self._activo = False
        self._cola_tx: "queue.Queue[bytes]" = queue.Queue()

    def enviar_comando(self, datos: bytes):
        """Encola bytes para transmitir. La escritura real la hace este
        mismo hilo (run()), nunca se llama a pyserial desde otro hilo."""
        self._cola_tx.put(datos)

    def detener(self):
        self._activo = False

    def run(self):
        extractor = ExtractorDeFrames()
        try:
            ser = serial.Serial(self.puerto, self.baudrate, timeout=0.05)
        except Exception as e:
            self.error.emit(f'No se pudo abrir {self.puerto}: {e}')
            self.conectado.emit(False)
            return

        self._activo = True
        self.conectado.emit(True)

        while self._activo:
            try:
                while not self._cola_tx.empty():
                    datos = self._cola_tx.get_nowait()
                    ser.write(datos)

                n = ser.in_waiting
                if n:
                    datos = ser.read(n)
                    extractor.append_bytes(datos)
                    for m in extractor.extraer_muestras(time.time()):
                        self.muestra_recibida.emit(m)
                else:
                    self.msleep(10)
            except serial.SerialException as e:
                self.error.emit(f'Error de puerto serie: {e}')
                break
            except Exception as e:
                self.error.emit(f'Error inesperado en el enlace: {e}')
                break

        try:
            ser.close()
        except Exception:
            pass
        self.conectado.emit(False)
