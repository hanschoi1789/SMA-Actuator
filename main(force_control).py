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
        uic.loadUi("main_final_ui.ui", self)
        
        # 기본 설정
        self.setWindowTitle("Thermal Control System (UI Load Ver.)")
        
        # 로깅 상태 변수 추가
        self.is_logging = False

        # 2. 독립 기록용 리스트 (START/STOP/RESET에 반응)
        self.log_buffer = {
            'time': [], 'temp': [], 'pwm': [], 'target': [],
            'ser_time': [], 'force': [], 'disp': []
        }

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
        self.base_time = time.time()

        # [제어용 상태 변수 추가]
        self.last_calc_time = None   # 직전에 계산했던 '시간' 기억
        self.last_error = 0.0        # 직전의 '오차' 기억 (미분용)
        self.error_sum = 0.0         # 오차의 '누적합' 기억 (적분용)

        # 새로운 UI 시그널 연결
        self.btn_log_start.clicked.connect(self.start_logging)
        self.btn_log_stop.clicked.connect(self.stop_logging)
        self.btn_log_reset.clicked.connect(self.reset_logging_data)
        self.btn_save.clicked.connect(self.manual_save) # 기존 저장 버튼

        # [추가] 데이터 저장 폴더 확인
        if not os.path.exists("data_logs"):
            os.makedirs("data_logs")

        # 타이머 설정
        self.tx_timer = QTimer()
        self.tx_timer.setInterval(20) # 50Hz
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
        # [추가됨] 힘 그래프 쪽에 빨간 점선(Target Force)을 추가
        self.target_line = self.force_plot_widget.plot(pen=pg.mkPen('r', width=2, style=Qt.DashLine), name="Target Force")

        # Temperature Plot
        self.temp_plot_widget.setTitle("Temperature Monitor", color="w", size="12pt")
        self.temp_plot_widget.setLabel("left", "Temperature (°C)")
        self.temp_plot_widget.showGrid(x=True, y=True)
        self.temp_line = self.temp_plot_widget.plot(pen=pg.mkPen('y', width=2), name="Current Temp")
       

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
            self.target_force = self.spin_target.value()
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
    
    def start_logging(self):
        """로깅 시작: 이제부터 들어오는 데이터를 리스트에 저장함"""
        self.is_logging = True
        self.btn_log_start.setEnabled(False)
        self.btn_log_stop.setEnabled(True)
        print("Recording Started...")

    def stop_logging(self):
        """로깅 중단: 데이터 수집을 멈춤 (저장은 SAVE 버튼으로 따로 진행)"""
        self.is_logging = False
        self.btn_log_start.setEnabled(True)
        self.btn_log_stop.setEnabled(False)
        print("Recording Stopped.")

    def reset_logging_data(self):
        """중요: 그래프는 놔두고 '저장할 데이터'만 비웁니다."""
        reply = QMessageBox.question(self, 'Reset Log Data', 
                                    "Clear only the recorded log data? (Graphs will remain)",
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            for key in self.log_buffer:
                self.log_buffer[key].clear()
            print("Log buffer cleared. You can start a new recording.")
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
            with open(csv_path, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "Temp", "PWM", "Force", "Disp"])
                
                # log_buffer 데이터를 사용하여 저장
                # 데이터 개수가 다를 수 있으므로 안전하게 처리
                max_len = max(len(self.log_buffer['time']), len(self.log_buffer['ser_time']))
                for i in range(max_len):
                    row = []
                    # CAN 데이터
                    if i < len(self.log_buffer['time']):
                        row.extend([self.log_buffer['time'][i], self.log_buffer['temp'][i], self.log_buffer['pwm'][i]])
                    else: row.extend(["", "", ""])
                    # Serial 데이터
                    if i < len(self.log_buffer['ser_time']):
                        row.extend([self.log_buffer['force'][i], self.log_buffer['disp'][i]])
                    else: row.extend(["", ""])
                    
                    writer.writerow(row)
            # --- 2. PNG 스크린샷 저장 ---
            self.grab().save(img_path, 'png')

            QMessageBox.information(self, "Success", f"Successfully saved:\nCSV: {csv_path}\nPNG: {img_path}")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))
        
    def handle_serial_data(self, disp, force):
        elapsed = time.time() - self.base_time
        self.last_disp = disp
        self.last_force = force
        self.ser_time_data.append(elapsed)
        self.disp_data.append(disp)
        self.force_data.append(force)
        # [로깅] 독립 기록
        if self.is_logging:
            self.log_buffer['ser_time'].append(elapsed)
            self.log_buffer['disp'].append(disp)
            self.log_buffer['force'].append(force)
        # --- 2. [추가] 힘 제어(Force Control) 로직 연결 ---
        # PID 모드(Auto 모드)가 켜져 있을 때만 제어기를 돌립니다.
        if self.current_pid_mode:
            
            # 받아온 disp, force 두 값 중 'force'만 제어 함수로 쏙 집어넣습니다
            target_pwm = self.Calculate_pwm(force)
            
            # 계산되어 나온 PWM 값을 현재 시스템의 PWM 변수에 업데이트합니다.
            # (그러면 tx_timer가 20ms마다 알아서 STM32로 이 값을 전송합니다)
            self.current_pwm = target_pwm

    def Calculate_pwm(self, current_force):
        # 1. 현재 시간 측정 (t)
        current_time = time.time()
        # 2. dt 계산 (최초 실행 시 방어 로직)
        if self.last_calc_time is None:
            # 처음 이 함수가 실행되었을 때는 과거 시간이 없으므로 
            # 임의의 작은 초기 dt값을 주거나 계산을 건너뜁니다.
            dt = 0.01 
        else:
            # 현재 시간 - 직전 시간 = 걸린 시간 (dt)
            dt = current_time - self.last_calc_time
            
        # 0 나누기 방지 (OS 지터 등으로 인해 시간이 0으로 측정될 수 있음)
        if dt <= 0.0:
            dt = 0.001
            
        # ----------------------------------------------------
        # 3. 오차(Error) 및 수학적 동역학 계산
        # ----------------------------------------------------
        error = self.target_force - current_force
        
        # 미분 (오차의 변화율) = (현재 오차 - 과거 오차) / 걸린 시간
        error_dot = (error - self.last_error) / dt 
        
        # 적분 (오차의 누적) = 기존 누적값 + (현재 오차 * 걸린 시간)
        self.error_sum += (error * dt) 
        
        
        # ----------------------------------------------------
        # 4. 제어 알고리즘 (SMC, PID 등) 적용
        # ----------------------------------------------------
        # 예시: 단순 PID 제어 식
        Kp = 10.0
        Kd = 0.0
        Ki = 0.1
        
        control_output = (Kp * error) + (Kd * error_dot) + (Ki * self.error_sum)
        
        # ----------------------------------------------------
        # 5. PWM 스케일링 및 포화(Saturation) 처리
        # ----------------------------------------------------
        pwm_val = int(control_output)
        if pwm_val > 100: pwm_val = 100  # CAN 통신 프로토콜에 맞춤 (0~100)
        if pwm_val < 0: pwm_val = 0
        
        # 6. 다음 루프를 위해 '현재'를 '과거'로 저장!
        self.last_calc_time = current_time
        self.last_error = error
        
        return pwm_val

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
        # [로깅] START LOGGING 상태일 때만 별도 버퍼에 기록
        if self.is_logging:
            self.log_buffer['time'].append(actual_elapsed)
            self.log_buffer['temp'].append(temp)
            self.log_buffer['pwm'].append(pwm)
            self.log_buffer['target'].append(self.current_target if self.current_pid_mode else float('nan'))

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
            target_force=self.target_force
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
