import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

class RealTimeGraph(QMainWindow):
    def __init__(self):
        super().__init__()

        # 1. 시리얼 설정 (포트 이름 확인 필요)
        try:
            # 타임아웃을 0에 가깝게 줄여서 블로킹 방지
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01) 
        except:
            print("포트를 찾을 수 없습니다. 연결을 확인하세요.")
            sys.exit()

        # 2. UI 설정
        self.setWindowTitle("Arduino Real-time Sensor Data (Low Latency)")
        self.resize(800, 600)
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # 3. 그래프 위젯 설정
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w') 
        self.graph_widget.addLegend()        
        self.graph_widget.showGrid(x=True, y=True)
        # OpenGL 가속 활성화 (대량 데이터 처리 시 속도 향상)
        self.graph_widget.setDownsampling(mode='peak')
        self.graph_widget.setClipToView(True)
        
        self.layout.addWidget(self.graph_widget)

        self.disp_plot = self.graph_widget.plot(pen=pg.mkPen('b', width=2), name="Displacement (mm)")
        self.force_plot = self.graph_widget.plot(pen=pg.mkPen('r', width=2), name="Force (N)")

        self.disp_data = []
        self.force_data = []
        self.max_points = 200 

        # 4. 타이머 설정
        self.timer = QTimer()
        self.timer.setInterval(10) # 10ms마다 호출 (약 100Hz)
        self.timer.timeout.connect(self.update_data)
        self.timer.start()

    def update_data(self):
        # 핵심 수정: while 루프를 사용하여 버퍼에 쌓인 데이터를 모두 처리
        # 데이터가 쌓여있다면 다 읽어서 리스트에 넣고, 그래프는 마지막에 한 번만 그립니다.
        
        data_read = False # 데이터를 실제로 읽었는지 체크

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                values = line.split(',')
                
                if len(values) == 2:
                    try:
                        disp = float(values[0])
                        force = float(values[1])

                        self.disp_data.append(disp)
                        self.force_data.append(force)
                        data_read = True # 유효한 데이터를 읽음

                    except ValueError:
                        pass # 변환 에러 시 무시하고 다음 데이터 읽기

            # while 루프가 끝나면(쌓인 데이터를 다 처리하면) 리스트 정리 및 그래프 업데이트
            if data_read:
                # 데이터 개수 유지 (슬라이딩 윈도우)
                while len(self.disp_data) > self.max_points:
                    self.disp_data.pop(0)
                    self.force_data.pop(0)

                # 그래프 그리기 (루프 밖에서 한 번만 수행하여 속도 향상)
                self.disp_plot.setData(self.disp_data)
                self.force_plot.setData(self.force_data)

        except Exception as e:
            print(f"Error: {e}")

    def closeEvent(self, event):
        if self.ser.is_open:
            self.ser.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RealTimeGraph()
    window.show()
    sys.exit(app.exec_())