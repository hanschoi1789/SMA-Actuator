import sys
import csv
import time
import os
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
import numpy as np

# 시리얼 및 CAN 드라이버 (기존 파일 사용)
from serial_driver import SerialWorker
from can_driver import CanWorker 

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 1. UI 파일 로드 (main_ui.ui 파일이 같은 경로에 있어야 함)
        uic.loadUi("main_ui.ui", self)
        
        # 기본 설정
        self.setWindowTitle("Thermal Control System (UI Load Ver.)")
        
        # 데이터 저장소 (기존 코드 유지)
        self.time_data = []
        self.temp_data = []
        self.target_data = [] 
        self.pwm_data = []
        self.ser_time_data = [] 
        self.disp_data = []
        self.force_data = []

        # UI 표시용 캐싱 변수
        self.last_temp = 0.0
        self.last_pwm = 0
        self.last_fan = False
        self.last_disp = None
        self.last_force = None

        # 제어 상태 변수
        self.current_pwm = 0
        self.current_fan = False
        self.current_pid_mode = False
        self.current_target = 0.0
        self.base_time = time.time()

        # [추가] 데이터 저장 폴더 확인
        if not os.path.exists("data_logs"):
            os.makedirs("data_logs")

        # 타이머 설정
        self.tx_timer = QTimer()
        self.tx_timer.setInterval(100) # 10Hz
        self.tx_timer.timeout.connect(self.send_heartbeat)

        self.plot_timer = QTimer()
        self.plot_timer.setInterval(50)  # 20Hz
        self.plot_timer.timeout.connect(self.update_ui)

        # Worker 인스턴스 설정
        self.worker = CanWorker()
        self.worker.data_received.connect(self.handle_new_data)
        self.worker.error_occurred.connect(self.handle_error)

        self.serworker = SerialWorker(port='/dev/ttyACM0', baudrate=115200)
        self.serworker.data_received.connect(self.handle_serial_data)

        # 그래프 초기화 (기존 스타일 적용)
        self.setup_plots_style()

        # UI 시그널 연결 (UI 파일 내 objectName 기준)
        self.btn_start.clicked.connect(self.start_system)
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_apply.clicked.connect(self.apply_settings)
        self.btn_save.clicked.connect(self.manual_save) # UI의 SAVE DATA 버튼

        self.radio_manual.toggled.connect(self.on_mode_changed)
        self.radio_pid.toggled.connect(self.on_mode_changed)
        
        # 초기 UI 상태
        self.on_mode_changed()

    def setup_plots_style(self):
        """기존 코드의 그래프 스타일 및 레이블 설정을 그대로 적용"""
        # Displacement Plot
        self.disp_plot_widget.setTitle("Displacement Monitor", color="w", size="12pt")
        self.disp_line = self.disp_plot_widget.plot(pen=pg.mkPen('b', width=2))
        self.disp_plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.disp_plot_widget.setLabel('left', "Displacement", units='mm')
        self.disp_plot_widget.setLabel('bottom', "Time", units='s')

        # Force Plot
        self.force_plot_widget.setTitle("Force Monitor", color="w", size="12pt")
        self.force_line = self.force_plot_widget.plot(pen=pg.mkPen('m', width=2))
        self.force_plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.force_plot_widget.setLabel('left', "Force", units='N')
        self.force_plot_widget.setLabel('bottom', "Time", units='s')

        # Temperature Plot
        self.temp_plot_widget.setTitle("Temperature Monitor", color="w", size="12pt")
        self.temp_plot_widget.setLabel("left", "Temperature (°C)")
        self.temp_plot_widget.showGrid(x=True, y=True)
        self.temp_line = self.temp_plot_widget.plot(pen=pg.mkPen('y', width=2), name="Current Temp")
        self.target_line = self.temp_plot_widget.plot(pen=pg.mkPen('r', width=2, style=Qt.DashLine), name="Target Temp")

        # PWM Plot
        self.pwm_plot_widget.setLabel("left", "PWM (%)")
        self.pwm_plot_widget.setLabel("bottom", "Time (s)")
        self.pwm_plot_widget.showGrid(x=True, y=True)
        self.pwm_plot_widget.setYRange(0, 105) 
        self.pwm_line = self.pwm_plot_widget.plot(pen=pg.mkPen('c', width=2), name="PWM Output")

        # X축 동기화
        self.disp_plot_widget.setXLink(self.temp_plot_widget)
        self.force_plot_widget.setXLink(self.temp_plot_widget)
        self.pwm_plot_widget.setXLink(self.temp_plot_widget)

    def start_system(self):
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_apply.setEnabled(True)
        
        self.apply_settings()
        self.base_time = time.time()

        try:
            if not self.serworker.isRunning():
                self.serworker.start()
        except Exception as e:
            print(f"Serial Error: {e}")

        try:
            if not self.worker.isRunning():
                self.worker.start()
        except Exception as e:
            print(f"CAN Error: {e}")

        self.tx_timer.start()
        self.plot_timer.start()
        print("System Started.")

    def apply_settings(self):
        if self.radio_manual.isChecked():
            self.current_pwm = self.spin_pwm.value()
            self.current_fan = self.chk_fan.isChecked()
            self.current_pid_mode = False
        else:
            self.current_target = self.spin_target.value()
            self.current_pid_mode = True
        print(f"Applied Settings: PID={self.current_pid_mode}, PWM={self.current_pwm}")

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
        QMessageBox.warning(self, "System Stopped", "Emergency Stop! Save logs if needed.")

    def manual_save(self):
        # 1. 데이터 존재 여부 확인 (둘 중 하나라도 있으면 진행)
        can_exists = len(self.time_data) > 0
        ser_exists = len(self.ser_time_data) > 0

        if not can_exists and not ser_exists:
            QMessageBox.warning(self, "No Data", "No Data available to save")
            return

        user_input = self.edit_filename.text().strip()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename_base = user_input if user_input else f"log_{timestamp}"

        csv_path = f"data_logs/{filename_base}.csv"
        img_path = f"data_logs/{filename_base}.png"

        try:
            # 더 긴 리스트를 기준으로 잡거나, 시리얼 데이터를 우선함
            max_len = max(len(self.time_data), len(self.ser_time_data))
            
            with open(csv_path, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Index", "Time_CAN(s)", "Temp(C)", "PWM(%)", "Time_Ser(s)", "Force(N)", "Disp(mm)"])
                
                for i in range(max_len):
                    row = [i]
                    # CAN 데이터 채우기
                    if i < len(self.time_data):
                        row.extend([f"{self.time_data[i]:.3f}", f"{self.temp_data[i]:.2f}", self.pwm_data[i]])
                    else:
                        row.extend(["", "", ""])
                    
                    # 시리얼 데이터 채우기
                    if i < len(self.ser_time_data):
                        row.extend([f"{self.ser_time_data[i]:.3f}", f"{self.force_data[i]:.2f}", f"{self.disp_data[i]:.2f}"])
                    else:
                        row.extend(["", "", ""])
                    
                    writer.writerow(row)

            self.grab().save(img_path, 'png')
            QMessageBox.information(self, "Success", "Data Saved Successfully.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save data: {e}")
    def handle_serial_data(self, disp, force):
        elapsed = time.time() - self.base_time
        self.last_disp = disp
        self.last_force = force
        self.ser_time_data.append(elapsed)
        self.disp_data.append(disp)
        self.force_data.append(force)

    def handle_new_data(self, elapsed, temp, fan, pwm):
        actual_elapsed = time.time() - self.base_time
        self.time_data.append(actual_elapsed)
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
        if not self.time_data and not self.ser_time_data:
            return
        
        if self.ser_time_data:
            self.disp_line.setData(self.ser_time_data, self.disp_data)
            self.force_line.setData(self.ser_time_data, self.force_data)
            self.disp_plot_widget.setXRange(0, self.ser_time_data[-1])

        if self.time_data:
            self.temp_line.setData(self.time_data, self.temp_data)
            self.target_line.setData(self.time_data, self.target_data)
            self.pwm_line.setData(self.time_data, self.pwm_data)

        self.lbl_temp.setText(f"{self.last_temp:.1f} °C")
        self.lbl_pwm.setText(f"PWM: {self.last_pwm}%")

        # 온도별 색상 로직
        color = "red" if self.last_temp > 70.0 else "#2196F3"
        self.lbl_temp.setStyleSheet(f"font-size: 30px; font-weight: bold; color: {color};")

        # FAN 라벨 로직
        if self.last_fan:
            self.lbl_fan.setText("FAN: ON")
            self.lbl_fan.setStyleSheet("font-size: 20px; font-weight: bold; color: green;")
        else:
            self.lbl_fan.setText("FAN: OFF")
            self.lbl_fan.setStyleSheet("font-size: 20px; font-weight: bold; color: gray;")

    def on_mode_changed(self):
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

    def handle_error(self, msg):
        print(f"Error: {msg}")

    def closeEvent(self, event):
        self.serworker.stop()
        self.worker.running = False
        self.worker.wait()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())