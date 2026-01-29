import sys
import csv
import os
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QButtonGroup
from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg

from can_driver import CanWorker
# const.py와 can_driver.py는 같은 폴더에 있어야 합니다.

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # [UI 로드] mainwindow.ui 파일을 불러옵니다.
        # 이 시점에 UI 파일에 정의된 모든 위젯(버튼, 그래프, 라벨 등)이 self의 멤버 변수가 됩니다.
        uic.loadUi("mainwindow.ui", self)
        
        # [추가] 저장 폴더 자동 생성
        if not os.path.exists("data_logs"):
            os.makedirs("data_logs")

        # 데이터 저장소
        self.time_data = []
        self.temp_data = []
        self.target_data = [] 
        self.pwm_data = []

        # UI 표시용 캐싱 변수
        self.last_temp = 0.0
        self.last_pwm = 0
        self.last_fan = False

        # 제어 상태 변수
        self.current_pwm = 0
        self.current_fan = False
        self.current_pid_mode = False
        self.current_target = 0.0
        
        # CSV 관련 (호환성 유지)
        self.csv_file = None
        self.csv_writer = None

        # 타이머 설정
        self.tx_timer = QTimer()
        self.tx_timer.setInterval(100) # 10Hz
        self.tx_timer.timeout.connect(self.send_heartbeat)

        self.plot_timer = QTimer()
        self.plot_timer.setInterval(50)  # 20Hz
        self.plot_timer.timeout.connect(self.update_ui)

        # 워커 생성
        self.worker = CanWorker()
        self.worker.data_received.connect(self.handle_new_data)
        self.worker.error_occurred.connect(self.handle_error)

        # UI 초기화 (그래프 설정 및 시그널 연결)
        self.init_ui_logic()

    def init_ui_logic(self):
        """UI 파일 로드 후, 로직 연결 및 그래프 스타일 설정"""
        
        # 1. 그래프 설정 (UI 파일에서는 껍데기만 있으므로 스타일 지정 필요)
        # temp_plot_widget 설정
        self.temp_plot_widget.setTitle("Temperature Monitor", color="w", size="12pt")
        self.temp_plot_widget.setLabel("left", "Temperature (°C)")
        self.temp_plot_widget.showGrid(x=True, y=True)
        self.temp_plot_widget.addLegend()
        self.temp_line = self.temp_plot_widget.plot(pen=pg.mkPen('y', width=2), name="Current Temp")
        self.target_line = self.temp_plot_widget.plot(pen=pg.mkPen('r', width=2, style=Qt.DashLine), name="Target Temp")
        
        # pwm_plot_widget 설정
        self.pwm_plot_widget.setLabel("left", "PWM (%)")
        self.pwm_plot_widget.setLabel("bottom", "Time (s)")
        self.pwm_plot_widget.showGrid(x=True, y=True)
        self.pwm_plot_widget.setYRange(0, 105) 
        self.pwm_plot_widget.setXLink(self.temp_plot_widget) # X축 연동
        self.pwm_line = self.pwm_plot_widget.plot(pen=pg.mkPen('c', width=2), name="PWM Output")

        # 2. 버튼 및 입력 이벤트 연결
        self.radio_manual.toggled.connect(self.on_mode_changed)
        self.radio_pid.toggled.connect(self.on_mode_changed)
        
        self.btn_apply.clicked.connect(self.apply_settings)
        self.btn_start.clicked.connect(self.start_system)
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_save.clicked.connect(self.manual_save)

        # 3. 초기 상태 설정
        # QButtonGroup은 UI 파일에서 시각적으로 묶여도 논리적 그룹이 필요할 때 사용하지만,
        # 여기서는 단순 토글 처리를 위해 초기 상태만 확실히 잡아줍니다.
        self.on_mode_changed() # 초기 위젯 보이기/숨기기 상태 적용

    def manual_save(self):
        # 1. 파일 이름 결정
        user_input = self.edit_filename.text().strip()
        if not user_input:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename_base = f"log_{timestamp}"
        else:
            filename_base = user_input

        # 경로 설정
        csv_path = f"data_logs/{filename_base}.csv"
        img_path = f"data_logs/{filename_base}.png"

        try:
            # 2. CSV 저장
            with open(csv_path, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Time(sec)", "Temperature(C)", "PWM(%)"])
                for i in range(len(self.time_data)):
                    writer.writerow([
                        f"{self.time_data[i]:.3f}", 
                        f"{self.temp_data[i]:.2f}", 
                        self.pwm_data[i]
                    ])

            # 3. 스크린샷 저장
            pixmap = self.grab()
            pixmap.save(img_path, 'png')

            QMessageBox.information(self, "Save Success", f"Saved successfully!\nLocation: data_logs/\nFile: {filename_base}")
            
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save: {e}")

    def start_system(self):
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_apply.setEnabled(True)
        
        self.apply_settings()

        if not self.worker.isRunning():
            self.worker.start()
        self.tx_timer.start()
        self.plot_timer.start()
        
        print("System Started.")

    def apply_settings(self):
        if self.radio_manual.isChecked():
            self.current_pwm = self.spin_pwm.value()
            self.current_fan = self.chk_fan.isChecked()
            self.current_pid_mode = False
            print(f"Applied: Manual Mode, PWM={self.current_pwm}, FAN={self.current_fan}")
        else:
            self.current_target = self.spin_target.value()
            self.current_pid_mode = True
            print(f"Applied: PID Mode, Target={self.current_target}")

    def stop_all(self):
        self.worker.running = False
        self.worker.wait()
        self.tx_timer.stop()
        self.plot_timer.stop()

    def emergency_stop(self):
        print("!!! EMERGENCY STOP !!!")
        
        self.current_pwm = 0
        self.current_fan = False
        self.current_pid_mode = False
        self.current_target = 0.0

        self.send_heartbeat()

        self.tx_timer.stop()
        self.plot_timer.stop()

        try:
            self.worker.data_received.disconnect(self.handle_new_data)
        except:
            pass

        self.btn_stop.setText("STOPPED")
        self.btn_stop.setEnabled(False)
        self.btn_apply.setEnabled(False)
        self.btn_start.setEnabled(False)
        
        self.lbl_pwm.setText("PWM: 0% (STOP)")
        self.lbl_pwm.setStyleSheet("font-size: 20px; font-weight: bold; color: red;")
        
        QMessageBox.warning(self, "System Stopped", "Emergency Stop! Use 'SAVE DATA' button to save logs.")

    def closeEvent(self, event):
        self.stop_all()
        event.accept()

    def handle_new_data(self, elapsed, temp, fan, pwm):
        self.time_data.append(elapsed)
        self.temp_data.append(temp)
        self.pwm_data.append(pwm)
        
        if self.current_pid_mode:
            self.target_data.append(self.current_target)
        else:
            self.target_data.append(float('nan'))

        self.last_temp = temp
        self.last_pwm = pwm
        self.last_fan = bool(fan)

    def update_ui(self):
        if not self.time_data:
            return

        self.temp_line.setData(self.time_data, self.temp_data)
        self.target_line.setData(self.time_data, self.target_data)
        self.pwm_line.setData(self.time_data, self.pwm_data)

        self.lbl_temp.setText(f"{self.last_temp:.1f} °C")
        self.lbl_pwm.setText(f"PWM: {self.last_pwm}%")
        
        if self.last_temp > 70.0:
            self.lbl_temp.setStyleSheet("font-size: 30px; font-weight: bold; color: red;")
        else:
            self.lbl_temp.setStyleSheet("font-size: 30px; font-weight: bold; color: #2196F3;")

        if self.last_fan:
            self.lbl_fan.setText("FAN: ON")
            self.lbl_fan.setStyleSheet("font-size: 20px; font-weight: bold; color: green;")
        else:
            self.lbl_fan.setText("FAN: OFF")
            self.lbl_fan.setStyleSheet("font-size: 20px; font-weight: bold; color: gray;")

    def handle_error(self, msg):
        print(f"Error: {msg}")

    def on_mode_changed(self):
        # UI 파일에서 정의된 위젯들을 직접 제어합니다.
        is_pid = self.radio_pid.isChecked()
        self.widget_pid.setVisible(is_pid)
        self.widget_manual.setVisible(not is_pid)

    def send_heartbeat(self):
        self.worker.send_control_message(
            pwm=self.current_pwm,
            fan_on=self.current_fan,
            pid_enable=self.current_pid_mode,
            target_temp=self.current_target
        )

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
