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
            os.system("sudo ip link set can0 down") #can0인터페이스 비활성화
            os.system("sudo ip link set can0 up type can bitrate 1000000 dbitrate 1000000 fd on") #인터페이스 활성화, 통신타입 can으로,데이터 전송속도 설정
            self.bus = can.interface.Bus(channel="can0", interface="socketcan", fd=True) #버스 객체 생성, can0인터페이스 사용, socketcan드라이버 사용, CAN fd mode 설정
        except Exception as e:
            self.error_occurred.emit(f"CAN Init Error: {e}") #에러 메시지를 GUI로 전달
            return

        self.running = True
        self.start_time = time.time()

        # 2. 데이터 수신
        while self.running:  #while 구문은 main.py에서 close_event시 중단됨.(emergency_stop하면 연결은 끊기지만 데이터는 계속 송신)
            try:                 #에러가 발생할 수 있는 상황에서는 try구문을 씀. 발생시 곧바로 except로 넘어감.
                msg = self.bus.recv(timeout=0.1)    #canbus에서 0.1초 기다리고 메시지가 오면 msg에 저장
                if msg and msg.arbitration_id == const.RX_ID_STATE:  #msg가 존재하고 메시지 ID가 RX_ID(MCU상태)가 맞다면 실행
                    data = msg.data   #메시지 꾸러미를 풀어서 24바이트 이상인지 확인
                    if len(data) >= 24:
                        pwm = data[0]
                        fan = data[6]
                        raw_temp = data[12] | (data[13] << 8) #온도는 255를 넘어갈 수 있기 때문에 2바이트로 나눠서 보냄
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
        if not self.bus:   #통신 버스가 연결되어야만 동작
            return

        payload = [0] * 64          #Can Fd모드는 최대 64바이트까지 보낼 수 있음.
        
        payload[0] = max(0, min(100, int(pwm)))    #0보다 작으면 0으로, 100보다 크면 100으로, 소수점은 버림
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