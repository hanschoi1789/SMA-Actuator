# serial_driver.py
import serial
import time
from PyQt5.QtCore import QThread, pyqtSignal

class SerialWorker(QThread):
    # (변위, 힘) 데이터를 메인 UI로 전달하는 신호
    data_received = pyqtSignal(float, float)
    error_occurred = pyqtSignal(str)

    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None

    def run(self):
        """시리얼 데이터 수신 루프"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1) #통로를 만듬
            self.running = True
            print(f"Serial Connected: {self.port}")
        except Exception as e:
            self.error_occurred.emit(f"Serial Open Error: {e}")
            return

        while self.running:
            if self.ser.in_waiting > 0:
                try:
                    # '변위,힘\n' 형태의 데이터를 읽어옴
                    line = self.ser.readline().decode('utf-8').strip()
                    values = line.split(',')
                    if len(values) == 2:
                        disp = float(values[0])
                        force = float(values[1])
                        self.data_received.emit(disp, force)
                except Exception:
                    # 데이터가 깨져서 들어오는 경우 무시
                    pass
            else:
                time.sleep(0.001) # CPU 과점유 방지

    def stop(self):
        """쓰레드 종료 및 포트 닫기"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.wait() # 쓰레드가 완전히 종료될 때까지 대기