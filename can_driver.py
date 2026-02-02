# can_driver.py
import os
import time
import can
from PyQt5.QtCore import QThread, pyqtSignal
import const 

class CanWorker(QThread):
    # (경과시간, 온도, 팬상태, PWM값)
    data_received = pyqtSignal(float, float, int, int)
    error_occurred = pyqtSignal(str)

    def __init__(self): 
        """init함수는 인스턴스 생성 시 무조건 실행하는 함수. 따로 호출하거나 start하지 않아도 됨."""
        super().__init__()
        self.running = False
        self.bus = None
        self.start_time = 0

    def run(self):
        """QThread내부의 시스템에 따라 start()가 호출되면 운영체제한테 새 공간(Thread)를 달라고 하고
        그 공간 안에서 run함수를 실행. 따라서 main함수에서 start()시 호출."""
        """수신 루프 (백그라운드)"""
        # 1. CAN 초기화
        try:
            # 필요시 sudo 권한 명령어 주석 해제
            os.system("sudo ip link set can0 down")
            os.system("sudo ip link set can0 up type can bitrate 1000000 dbitrate 1000000 fd on")
            self.bus = can.interface.Bus(channel="can0", interface="socketcan", fd=True)
        except Exception as e:
            self.error_occurred.emit(f"CAN Init Error: {e}")
            return

        self.running = True
        self.start_time = time.time()

        # 2. 데이터 수신
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg and msg.arbitration_id == const.RX_ID_STATE:
                    data = msg.data
                    if len(data) >= 24:
                        pwm = data[0]
                        fan = data[6]
                        raw_temp = data[12] | (data[13] << 8)
                        temp = raw_temp / 4.0
                        
                        elapsed = time.time() - self.start_time
                        self.data_received.emit(elapsed, temp, fan, pwm)
            except Exception:
                pass

        # 3. 종료 정리
        if self.bus:
            self.bus.shutdown()
        os.system("sudo ip link set can0 down")

    def send_control_message(self, pwm, fan_on, pid_enable, target_temp):
        """사용자 정의함수.이름,기능,실행시점도 작성자의 자유"""
        """명령 전송 (조용히 전송만 함)"""
        if not self.bus:
            return

        payload = [0] * 64
        
        payload[0] = max(0, min(100, int(pwm)))
        payload[6] = 1 if fan_on else 0
        payload[12] = 1 if pid_enable else 0
        
        raw_target = int(target_temp * 4)
        payload[18] = raw_target & 0xFF
        payload[19] = (raw_target >> 8) & 0xFF

        msg = can.Message(
            arbitration_id=const.TX_ID_CTRL,
            data=payload,
            is_extended_id=False,
            is_fd=True,
            bitrate_switch=True
        )
        
        try:
            self.bus.send(msg)
            # print 제거됨 -> 터미널 조용함
        except can.CanError:
            pass