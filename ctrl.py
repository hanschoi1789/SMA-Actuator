"""
ctrl.py — 터미널 컨트롤러
실행: python ctrl.py

명령어:
  m        — Manual 모드 (PWM 직접 제어)
  t        — Temp PID 모드
  f        — Force 모드
  pwm N    — PWM 설정 (0~100)
  fan on/off — 팬 제어
  target N — 목표 온도(°C) 또는 힘(g) 설정
  stop     — 긴급 정지
  q        — 종료
"""

import time
import struct
import threading
import serial

PORT     = "COM6"
BAUDRATE = 115200

STX               = 0xAA
CMD_CONTROL       = 0x01
CMD_STATE         = 0x02
# CMD_FORCE_CONTROL = 0x04  (Phase 4 폐기 — CMD_CONTROL mode=CH_FORCE로 통합)
CMD_FORCE_STATE   = 0x05

CTRL_CH      = 6
DISPLAY_CH   = 0

# UI 모드 (사용자 노출)
MODE_MANUAL = 0
MODE_TEMP   = 1
MODE_FORCE  = 2

# 채널 제어 모드 (CMD 0x01 mode[6] — 펌웨어 ch_ctrl_e와 일치)
CH_OFF    = 0
CH_MANUAL = 1
CH_TEMP   = 2
CH_FORCE  = 3


def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, payload: bytes = b'') -> bytes:
    length   = len(payload)
    crc_data = bytes([cmd, length]) + payload
    return bytes([STX, cmd, length]) + payload + bytes([crc8(crc_data)])


class Controller:
    def __init__(self):
        self.ser        = None
        self.running    = False
        self.mode       = MODE_MANUAL
        self.pwm        = 0
        self.fan        = False
        self.pid_enable = False
        self.target_temp  = 0.0
        self.target_force = 0.0

        # 최신 수신값
        self.temp  = 0.0
        self.pwm_fb = 0
        self.fan_fb = False
        self.force = 0.0
        self.disp  = 0.0

    def connect(self):
        self.ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
        self.running = True
        t = threading.Thread(target=self._rx_loop, daemon=True)
        t.start()
        print(f"Connected to {PORT}\n")

    def _rx_loop(self):
        state    = 'IDLE'
        cmd      = 0
        length   = 0
        data_buf = bytearray()
        text_buf = bytearray()

        while self.running:
            try:
                raw = self.ser.read(1)
                if not raw:
                    continue
                byte = raw[0]

                if state == 'IDLE':
                    if byte == STX:
                        if text_buf:
                            text = text_buf.decode('ascii', errors='replace').strip()
                            if text:
                                print(f"\r[MCU] {text}")
                            text_buf = bytearray()
                        state = 'WAIT_CMD'
                    else:
                        text_buf.append(byte)
                        if byte == ord('\n'):
                            text = text_buf.decode('ascii', errors='replace').strip()
                            if text:
                                print(f"\r[MCU] {text}")
                            text_buf = bytearray()

                elif state == 'WAIT_CMD':
                    cmd   = byte
                    state = 'WAIT_LEN'

                elif state == 'WAIT_LEN':
                    length   = byte
                    data_buf = bytearray()
                    if length == 0:
                        state = 'WAIT_CRC'
                    elif length > 250:
                        state = 'IDLE'
                    else:
                        state = 'WAIT_DATA'

                elif state == 'WAIT_DATA':
                    data_buf.append(byte)
                    if len(data_buf) >= length:
                        state = 'WAIT_CRC'

                elif state == 'WAIT_CRC':
                    crc_ok = (byte == crc8(bytes([cmd, length]) + bytes(data_buf)))
                    if crc_ok:
                        self._handle_frame(cmd, bytes(data_buf))
                    state = 'IDLE'

            except Exception:
                pass

    def _handle_frame(self, cmd, data):
        if cmd == CMD_STATE and len(data) >= 24:
            self.pwm_fb = data[DISPLAY_CH]
            self.fan_fb = bool(data[6 + DISPLAY_CH])
            raw_t = data[12 + DISPLAY_CH*2] | (data[13 + DISPLAY_CH*2] << 8)
            self.temp = raw_t / 4.0

        elif cmd == CMD_FORCE_STATE and len(data) >= 10:
            self.force, self.disp = struct.unpack_from('<ff', data, 2)

    def _ui_mode_to_ch_mode(self):
        if self.mode == MODE_MANUAL: return CH_MANUAL
        if self.mode == MODE_TEMP:   return CH_TEMP
        if self.mode == MODE_FORCE:  return CH_FORCE
        return CH_OFF

    def send_control(self):
        """Phase 4 새 페이로드: mode[6] + manual_pwm[6] + manual_fan[6] + target[6×u16]."""
        ch_mode = self._ui_mode_to_ch_mode()

        mode_arr       = [CH_OFF] * CTRL_CH
        manual_pwm_arr = [0]      * CTRL_CH
        manual_fan_arr = [0]      * CTRL_CH
        target_raw     = [0]      * CTRL_CH

        ch = DISPLAY_CH
        mode_arr[ch]       = ch_mode
        manual_pwm_arr[ch] = max(0, min(100, self.pwm))
        manual_fan_arr[ch] = 1 if self.fan else 0
        if ch_mode == CH_TEMP:
            target_raw[ch] = max(0, min(0xFFFF, int(round(self.target_temp * 4))))    # 0.25°C/LSB
        elif ch_mode == CH_FORCE:
            target_raw[ch] = max(0, min(0xFFFF, int(round(self.target_force * 10))))  # 0.1g/LSB

        payload = bytearray()
        payload.extend(mode_arr)
        payload.extend(manual_pwm_arr)
        payload.extend(manual_fan_arr)
        for v in target_raw:
            payload.append(v & 0xFF)
            payload.append((v >> 8) & 0xFF)

        self.ser.write(build_frame(CMD_CONTROL, bytes(payload)))

    def print_status(self):
        mode_str = ['MANUAL', 'TEMP PID', 'FORCE'][self.mode]
        print(f"\n{'='*50}")
        print(f"  모드     : {mode_str}")
        print(f"  온도     : {self.temp:.1f} °C")
        print(f"  PWM      : {self.pwm_fb}%")
        print(f"  FAN      : {'ON' if self.fan_fb else 'OFF'}")
        print(f"  힘       : {self.force:.2f} g")
        print(f"  변위     : {self.disp:.2f} mm")
        if self.mode == MODE_TEMP:
            print(f"  목표온도 : {self.target_temp:.1f} °C")
        elif self.mode == MODE_FORCE:
            print(f"  목표힘   : {self.target_force:.1f} g")
        print(f"{'='*50}")

    def run(self):
        # 상태 주기적 출력 스레드
        def status_loop():
            while self.running:
                self.print_status()
                time.sleep(1.0)

        st = threading.Thread(target=status_loop, daemon=True)
        st.start()

        # heartbeat 스레드 (Phase 4: 모든 모드 동일 송신 — mode 정보가 페이로드에 포함)
        def heartbeat_loop():
            while self.running:
                self.send_control()
                time.sleep(0.1)

        ht = threading.Thread(target=heartbeat_loop, daemon=True)
        ht.start()

        print("명령어: m(수동) | t(온도PID) | f(힘제어) | pwm N | fan on/off | target N | stop | q(종료)")
        print("현재 상태는 1초마다 자동 업데이트됩니다.\n")

        while self.running:
            try:
                cmd = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd == 'q':
                break

            elif cmd == 'm':
                self.mode       = MODE_MANUAL
                self.pid_enable = False
                self.send_control()
                print(">> Manual 모드")

            elif cmd == 't':
                self.mode       = MODE_TEMP
                self.pid_enable = True
                self.send_control()
                print(f">> Temp PID 모드  목표={self.target_temp}°C")

            elif cmd == 'f':
                self.mode       = MODE_FORCE
                self.pid_enable = False
                self.pwm        = 0
                self.send_control()
                print(f">> Force 모드  목표={self.target_force}g")

            elif cmd.startswith('pwm '):
                try:
                    val = int(cmd.split()[1])
                    self.pwm = max(0, min(100, val))
                    self.send_control()
                    print(f">> PWM = {self.pwm}%")
                except:
                    print("사용법: pwm 0~100")

            elif cmd.startswith('fan '):
                val = cmd.split()[1]
                self.fan = (val == 'on')
                self.send_control()
                print(f">> FAN = {'ON' if self.fan else 'OFF'}")

            elif cmd.startswith('target '):
                try:
                    val = float(cmd.split()[1])
                    if self.mode == MODE_TEMP:
                        self.target_temp = val
                        self.send_control()
                        print(f">> 목표 온도 = {val}°C")
                    elif self.mode == MODE_FORCE:
                        self.target_force = val
                        self.send_control()
                        print(f">> 목표 힘 = {val}g")
                    else:
                        print("먼저 t 또는 f 모드 선택하세요")
                except:
                    print("사용법: target 숫자")

            elif cmd == 'stop':
                self.mode       = MODE_MANUAL
                self.pwm        = 0
                self.fan        = False
                self.pid_enable = False
                self.send_control()
                print(">> 긴급 정지!")

            else:
                print("명령어: m | t | f | pwm N | fan on/off | target N | stop | q")

        # 종료
        self.running = False
        self.mode    = MODE_MANUAL
        self.pwm     = 0
        self.fan     = False
        self.pid_enable = False
        self.send_control()
        time.sleep(0.2)
        self.ser.close()
        print("종료.")


if __name__ == "__main__":
    ctrl = Controller()
    ctrl.connect()
    ctrl.run()