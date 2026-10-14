"""
descarga_directa.py
Hilo (QThread) para descargar la EEPROM del cohete por USB DIRECTO (no pasa
por la estacion terrena ni por LoRa). Envia el comando 'D' que ya existe en
rocket_datalogger.ino y parsea las lineas de texto que devuelve:

    INICIO_DESCARGA registros=<N>
    D:<54 caracteres hex, 27 bytes>
    D:...
    ...
    FIN_DESCARGA
"""
import time
import queue

import serial

from PySide6.QtCore import QThread, Signal

from protocolo import decodificar_registro_eeprom, ExtractorDeFrames


class HiloDescarga(QThread):
    progreso = Signal(int, int)  # (registros leidos, total esperado; -1 si aun no se sabe)
    terminado = Signal(list)     # List[Muestra]
    error = Signal(str)

    def __init__(self, puerto: str, baudrate: int = 115200, timeout_s: float = 60.0, parent=None):
        super().__init__(parent)
        self.puerto = puerto
        self.baudrate = baudrate
        self.timeout_s = timeout_s

    def run(self):
        try:
            ser = serial.Serial(self.puerto, self.baudrate, timeout=1.0)
        except Exception as e:
            self.error.emit(f'No se pudo abrir {self.puerto}: {e}')
            return

        muestras = []
        total_esperado = -1
        buf = b''
        t_inicio = time.time()

        try:
            # Algunas placas resetean al abrir el puerto (DTR/RTS); dar un
            # respiro antes de mandar el comando.
            time.sleep(0.3)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            ser.write(b'D')

            while True:
                if time.time() - t_inicio > self.timeout_s:
                    self.error.emit(
                        'Timeout esperando la descarga. Verifica que el cable USB '
                        'este conectado directo al cohete (no a la estacion terrena) '
                        'y que el sketch tenga el USB CDC habilitado.'
                    )
                    return

                chunk = ser.read(256)
                if not chunk:
                    continue
                buf += chunk

                while b'\n' in buf:
                    linea, buf = buf.split(b'\n', 1)
                    linea = linea.strip().decode('ascii', errors='ignore')
                    if not linea:
                        continue

                    if linea.startswith('INICIO_DESCARGA'):
                        try:
                            total_esperado = int(linea.split('=', 1)[1])
                        except (IndexError, ValueError):
                            total_esperado = -1
                        self.progreso.emit(0, total_esperado)

                    elif linea == 'FIN_DESCARGA':
                        self.terminado.emit(muestras)
                        return

                    elif linea.startswith('D:'):
                        try:
                            datos = bytes.fromhex(linea[2:])
                            m = decodificar_registro_eeprom(datos, t_pc=time.time())
                            muestras.append(m)
                            self.progreso.emit(len(muestras), total_esperado)
                        except Exception:
                            continue  # linea corrupta/incompleta, se ignora y se sigue
        except Exception as e:
            self.error.emit(f'Error durante la descarga: {e}')
        finally:
            try:
                ser.close()
            except Exception:
                pass


class HiloComandoRapido(QThread):
    """
    Hilo para los comandos cortos por USB directo al cohete que no
    involucran una descarga masiva: consultar estado ('S'), borrar la
    EEPROM ('E', con la confirmacion 'Y' que pide el firmware dentro de
    los 5s siguientes) y recalibrar el giroscopo ('G').
    """
    resultado = Signal(str)
    error = Signal(str)

    def __init__(self, puerto: str, modo: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self.puerto = puerto
        self.modo = modo  # 'estado', 'borrar' o 'giroscopio'
        self.baudrate = baudrate

    def _leer_lineas(self, ser, hasta_texto: str = None, timeout_s: float = 3.0):
        buf = b''
        t0 = time.time()
        lineas = []
        while time.time() - t0 < timeout_s:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                while b'\n' in buf:
                    linea, buf = buf.split(b'\n', 1)
                    linea = linea.strip().decode('ascii', errors='ignore')
                    if linea:
                        lineas.append(linea)
                        t0 = time.time()  # sigue llegando texto: extender el timeout
                        if hasta_texto and hasta_texto in linea:
                            return lineas
        return lineas

    def run(self):
        try:
            ser = serial.Serial(self.puerto, self.baudrate, timeout=0.2)
        except Exception as e:
            self.error.emit(f'No se pudo abrir {self.puerto}: {e}')
            return

        try:
            time.sleep(0.3)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            if self.modo == 'estado':
                ser.write(b'S')
                lineas = self._leer_lineas(ser, hasta_texto='----------------', timeout_s=3.0)
                self.resultado.emit('\n'.join(lineas) if lineas else '(sin respuesta del cohete)')

            elif self.modo == 'borrar':
                ser.write(b'E')
                lineas_prompt = self._leer_lineas(ser, hasta_texto='enviar', timeout_s=3.0)
                if not lineas_prompt:
                    self.error.emit('El cohete no respondio al pedido de borrado (revisa la conexion USB directa).')
                    return
                ser.write(b'Y')
                lineas_final = self._leer_lineas(ser, timeout_s=3.0)
                self.resultado.emit('\n'.join(lineas_prompt + lineas_final))
            elif self.modo == 'giroscopio':
                ser.write(b'G')
                # Hasta 3 intentos de ~1.5 s cada uno si detecta movimiento
                lineas = self._leer_lineas(ser, hasta_texto='alibrado', timeout_s=6.0)
                self.resultado.emit('\n'.join(lineas) if lineas else '(sin respuesta del cohete)')
            else:
                self.error.emit(f'Modo de comando desconocido: {self.modo}')
        except Exception as e:
            self.error.emit(f'Error de comunicacion: {e}')
        finally:
            try:
                ser.close()
            except Exception:
                pass


class HiloUSBDirecto(QThread):
    """
    Conexion persistente de 'modo debug' por USB directo al cohete
    (comando 'T' del firmware). Por defecto interpreta los bytes
    recibidos como texto (el menu interactivo del firmware). Cuando el
    usuario entra a la opcion '9' (telemetria en vivo), hay que poner
    self.modo = 'telemetria' para que en cambio se parseen como tramas
    binarias -- USA EL MISMO FORMATO que ya manda la estacion terrena
    hacia la PC (ver protocolo.py), asi que se reutiliza el mismo
    ExtractorDeFrames sin parsers nuevos.
    """
    texto_recibido = Signal(str)
    muestra_recibida = Signal(object)  # Muestra, solo llega en modo 'telemetria'
    error = Signal(str)
    conectado = Signal(bool)

    def __init__(self, puerto: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self.puerto = puerto
        self.baudrate = baudrate
        self._activo = False
        self._cola_tx: "queue.Queue[bytes]" = queue.Queue()
        self.modo = 'texto'  # o 'telemetria'

    def enviar(self, datos: bytes):
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
                    if self.modo == 'telemetria':
                        extractor.append_bytes(datos)
                        for m in extractor.extraer_muestras(time.time()):
                            self.muestra_recibida.emit(m)
                    else:
                        self.texto_recibido.emit(datos.decode('ascii', errors='replace'))
                else:
                    self.msleep(10)
            except serial.SerialException as e:
                self.error.emit(f'Error de puerto serie: {e}')
                break
            except Exception as e:
                self.error.emit(f'Error inesperado: {e}')
                break

        try:
            ser.close()
        except Exception:
            pass
        self.conectado.emit(False)
