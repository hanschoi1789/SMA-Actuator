import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

class RealTimeGraph(QMainWindow):
    def __init__(self):
        super().__init__()

        # 1. 시리얼 설정 
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        except:
            print("포트를 찾을 수 없습니다. 연결을 확인하세요.")
            sys.exit()

        # 2. UI 설정
        self.setWindowTitle("Arduino Real-time Sensor Data")
        self.resize(800, 600)
        
        # 메인 레이아웃
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # 3. 그래프 위젯 설정 (pyqtgraph)
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w') 
        self.graph_widget.addLegend()        
        self.graph_widget.showGrid(x=True, y=True)
        self.layout.addWidget(self.graph_widget)

        # 변위 그래프 (파란색)
        self.disp_plot = self.graph_widget.plot(pen=pg.mkPen('b', width=2), name="Displacement (mm)")
        # 힘 그래프 (빨간색)
        self.force_plot = self.graph_widget.plot(pen=pg.mkPen('r', width=2), name="Force (N)")

        # 데이터 저장 리스트
        self.disp_data = []
        self.force_data = []
        self.max_points = 200 # 화면에 표시될 최대 데이터 개수

        # 4. 타이머 설정 (주기적으로 데이터를 읽어 그래프 업데이트)
        self.timer = QTimer()
        self.timer.setInterval(13) # 10ms마다 업데이트
        self.timer.timeout.connect(self.update_data)
        self.timer.start()
        

    def update_data(self):
        if self.ser.in_waiting > 0:
            try:
                # 아두이노 데이터 읽기 ("변위,힘")
                line = self.ser.readline().decode('utf-8').strip()
                values = line.split(',')
                
                if len(values) == 2:
                    disp = float(values[0])
                    force = float(values[1])

                    # 데이터 리스트에 추가
                    self.disp_data.append(disp)
                    self.force_data.append(force)

                    # 데이터 개수 제한 (슬라이딩 윈도우)
                    if len(self.disp_data) > self.max_points:
                        self.disp_data.pop(0)
                        self.force_data.pop(0)

                    # 그래프 선 업데이트
                    self.disp_plot.setData(self.disp_data)
                    self.force_plot.setData(self.force_data)

            except Exception as e:
                print(f"Data Error: {e}")

    def closeEvent(self, event):
        self.ser.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RealTimeGraph()
    window.show()
    sys.exit(app.exec_())