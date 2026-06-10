"""
uart_driver.py
PC <-> MCU UART communication
"""

import time
import struct
import queue
import serial
from PyQt5.QtCore import QThread, pyqtSignal

STX = 0xAA
CMD_CONTROL       = 0x01
CMD_STATE         = 0x02
CMD_GAIN_UPDATE   = 0x03
CMD_FORCE_STATE   = 0x05   
CMD_I2C_TEST      = 0x06

CTRL_CH = 6

CH_OFF    = 0   
CH_MANUAL = 1   
CH_TEMP   = 2   
CH_FORCE  = 3   

def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def build_frame(cmd: int, payload: bytes) -> bytes:
    length   = len(payload)
    crc_data = bytes([cmd, length]) + payload
    return bytes([STX, cmd, length]) + payload + bytes([crc8(crc_data)])

class UartWorker(QThread):
    data_received  = pyqtSignal(float, list, list, list)
    force_received = pyqtSignal(float, int, float, float)   # elapsed, ch, force, displacement
    debug_message  = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port="COM3", baudrate=115200):
        super().__init__()
        self.port       = port
        self.baudrate   = baudrate
        self.running    = False
        self.ser        = None
        self.start_time = 0
        self._tx_queue        = queue.SimpleQueue()  
        self._last_state_emit = 0.0   
        self._last_force_emit = 0.0   

    def run(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.01)  
        except Exception as e:
            self.error_occurred.emit(f"Serial Init Error: {e}")
            return

        self.running    = True
        self.start_time = time.time()

        state     = 'IDLE'
        cmd       = 0
        length    = 0
        data_buf  = bytearray()
        debug_buf = bytearray()

        while self.running:
            while not self._tx_queue.empty():
                try:
                    self.ser.write(self._tx_queue.get_nowait())
                except Exception:
                    pass

            try:
                raw = self.ser.read(1)
                if not raw:
                    if debug_buf:
                        self._flush_debug(debug_buf)
                        debug_buf = bytearray()
                    continue
                byte = raw[0]
 
                if state == 'IDLE':
                    if byte == STX:
                        if debug_buf:
                            self._flush_debug(debug_buf)
                            debug_buf = bytearray()
                        state = 'WAIT_CMD'
                    else:
                        debug_buf.append(byte)
                        if byte == ord('\n'):
                            self._flush_debug(debug_buf)
                            debug_buf = bytearray()
 
                elif state == 'WAIT_CMD':
                    cmd = byte
                    state = 'WAIT_LEN'
 
                elif state == 'WAIT_LEN':
                    length = byte
                    data_buf = bytearray()
                    if length == 0:
                        state = 'WAIT_CRC'
                    elif length > 250:
                        debug_buf.extend([STX, cmd, byte])
                        state = 'IDLE'
                    else:
                        state = 'WAIT_DATA'
 
                elif state == 'WAIT_DATA':
                    data_buf.append(byte)
                    if len(data_buf) >= length:
                        state = 'WAIT_CRC'
 
                elif state == 'WAIT_CRC':
                    crc_input = bytes([cmd, length]) + bytes(data_buf)
                    if byte == crc8(crc_input):
                        self._handle_frame(cmd, bytes(data_buf))
                    else:
                        debug_buf.append(STX)
                        debug_buf.append(cmd)
                        debug_buf.append(length)
                        debug_buf.extend(data_buf)
                        debug_buf.append(byte)
                    state = 'IDLE'
 
            except Exception:
                pass

        if debug_buf:
            self._flush_debug(debug_buf)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _flush_debug(self, buf: bytearray):
        try:
            text = buf.decode('ascii', errors='replace').rstrip('\r\n')
            if text:
                self.debug_message.emit(text)
        except Exception:
            pass

    def _handle_frame(self, cmd: int, data: bytes):
        elapsed = time.time() - self.start_time
        now     = time.time()

        if cmd == CMD_STATE:
            if len(data) >= 24:
                pwm_list  = list(data[0:6])
                fan_list  = list(data[6:12])
                temp_list = [((data[12 + i*2]) | (data[13 + i*2] << 8)) / 4.0 for i in range(6)]
                if now - self._last_state_emit >= 0.05:
                    self._last_state_emit = now
                    self.data_received.emit(elapsed, temp_list, fan_list, pwm_list)

        elif cmd == CMD_FORCE_STATE:
            if len(data) >= 10:
                ch = data[1]
                force, displacement = struct.unpack_from('<ff', data, 2)
                if now - self._last_force_emit >= 0.02:
                    self._last_force_emit = now
                    self.force_received.emit(elapsed, ch, force, displacement)

    def send_control_message(self, mode, manual_pwm=0, manual_fan=False, target=0.0, display_ch=0):
        if isinstance(display_ch, (list, tuple)):
            chs = [max(0, min(CTRL_CH - 1, int(c))) for c in display_ch]
        else:
            chs = [max(0, min(CTRL_CH - 1, int(display_ch)))]

        mode_arr       = [CH_OFF] * CTRL_CH
        manual_pwm_arr = [0]      * CTRL_CH
        manual_fan_arr = [0]      * CTRL_CH
        target_raw     = [0]      * CTRL_CH

        for ch in chs:
            mode_arr[ch]       = int(mode)
            manual_pwm_arr[ch] = max(0, min(100, int(manual_pwm)))
            manual_fan_arr[ch] = 1 if manual_fan else 0
            if mode == CH_TEMP:
                target_raw[ch] = max(0, min(0xFFFF, int(round(target * 4))))   
            elif mode == CH_FORCE:
                target_raw[ch] = max(0, min(0xFFFF, int(round(target * 10))))  

        payload = bytearray()
        payload.extend(mode_arr)
        payload.extend(manual_pwm_arr)
        payload.extend(manual_fan_arr)
        for v in target_raw:
            payload.append(v & 0xFF)
            payload.append((v >> 8) & 0xFF)

        self._tx_queue.put(build_frame(CMD_CONTROL, bytes(payload)))

    def send_gain_update(self, channel, kp, ki, kd):
        payload = struct.pack('<fffB', kp, ki, kd, channel)
        self._tx_queue.put(build_frame(CMD_GAIN_UPDATE, bytes(payload)))

    def send_i2c_test(self):
        self._tx_queue.put(build_frame(CMD_I2C_TEST, b''))

    def stop(self):
        self.running = False