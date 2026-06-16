"""
can_driver.py
PC <- MCU CAN reception (SocketCAN via python-can, CAN-FD)
"""

import time
import queue
import struct
from PyQt5.QtCore import QThread, pyqtSignal

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

# ── MCU 송신 CAN ID (CAN3 standard) ──
CAN_ID_STATE       = 0x401
CAN_ID_FORCE_STATE = 0x403
CAN_ID_I2C_DEBUG   = 0x404

# ── MCU 수신 CAN ID (PC -> MCU) ──
CAN_ID_COMMAND     = 0x400   # CMD_CONTROL 페이로드 30B (zero-pad to 32B for FD DLC)
CAN_ID_GAIN_UPDATE = 0x402   # GAIN_UPDATE 페이로드 13B (zero-pad to 16B)

# ── ch_ctrl_e (uart_driver와 동일) ──
CTRL_CH   = 6
CH_OFF    = 0
CH_MANUAL = 1
CH_TEMP   = 2
CH_FORCE  = 3

class CanWorker(QThread):
    """SocketCAN(CAN-FD) 기반 RX 워커. UartWorker와 동일한 시그널을 emit."""
    data_received  = pyqtSignal(float, list, list, list)
    force_received = pyqtSignal(float, int, float, float)   # elapsed, ch, force, displacement
    debug_message  = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, interface='socketcan', channel='can0', fd=True):
        super().__init__()
        self.interface = interface
        self.channel   = channel
        self.fd        = fd
        self.running   = False
        self.bus       = None
        self.start_time = 0.0
        self._last_state_emit = 0.0
        self._last_force_emit = 0.0
        self._tx_queue = queue.SimpleQueue()

    def run(self):
        if not CAN_AVAILABLE:
            self.error_occurred.emit("python-can 미설치 — `pip install python-can` 후 재시도")
            return

        try:
            self.bus = can.Bus(interface=self.interface, channel=self.channel, fd=self.fd)
        except Exception as e:
            self.error_occurred.emit(f"CAN Init Error: {e}")
            return

        self.running    = True
        self.start_time = time.time()

        while self.running:
            while not self._tx_queue.empty():
                try:
                    self.bus.send(self._tx_queue.get_nowait())
                except Exception as e:
                    self.error_occurred.emit(f"CAN TX Error: {e}")

            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                self._handle_message(msg)
            except Exception as e:
                self.error_occurred.emit(f"CAN RX Error: {e}")
                break

        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def _handle_message(self, msg):
        elapsed = time.time() - self.start_time
        now     = time.time()
        cid     = msg.arbitration_id
        data    = bytes(msg.data)

        if cid == CAN_ID_STATE:
            if len(data) >= 24:
                pwm_list  = list(data[0:6])
                fan_list  = list(data[6:12])
                temp_list = [((data[12 + i*2]) | (data[13 + i*2] << 8)) / 4.0 for i in range(6)]
                if now - self._last_state_emit >= 0.05:
                    self._last_state_emit = now
                    self.data_received.emit(elapsed, temp_list, fan_list, pwm_list)

        elif cid == CAN_ID_FORCE_STATE:
            if len(data) >= 10:
                ch = data[1]
                force, displacement = struct.unpack_from('<ff', data, 2)
                if now - self._last_force_emit >= 0.02:
                    self._last_force_emit = now
                    self.force_received.emit(elapsed, ch, force, displacement)

        elif cid == CAN_ID_I2C_DEBUG:
            if len(data) >= 8 and data[0] == 0xEE:
                code = data[1]
                code_name = {1: "HAL_ERROR", 2: "HAL_BUSY", 3: "HAL_TIMEOUT"}.get(code, f"code={code}")
                self.debug_message.emit(f"[I2C DEBUG] {code_name}")

    def stop(self):
        self.running = False

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
        payload.extend(b'\x00' * (32 - len(payload)))

        msg = can.Message(arbitration_id=CAN_ID_COMMAND, data=bytes(payload), is_fd=self.fd, is_extended_id=False)
        self._tx_queue.put(msg)

    def send_gain_update(self, channel, kp, ki, kd):
        payload = bytearray(struct.pack('<fffB', kp, ki, kd, channel))
        payload.extend(b'\x00' * (16 - len(payload)))
        msg = can.Message(arbitration_id=CAN_ID_GAIN_UPDATE, data=bytes(payload), is_fd=self.fd, is_extended_id=False)
        self._tx_queue.put(msg)

    def send_i2c_test(self):
        pass