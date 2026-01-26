import sys
import csv
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QGroupBox, QGridLayout,
                             QRadioButton, QButtonGroup, QSpinBox, QDoubleSpinBox, QCheckBox, QMessageBox)
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg

from can_driver import CanWorker 
import signal

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Thermal Control System (Start/Apply Ver.)")
        self.resize(1350, 950) 
        
        # 데이터 저장소
        self.time_data = []
        self.temp_data = []
        self.target_data = [] 
        self.pwm_data = []

        # UI 표시용 캐싱 변수
        self.last_temp = 0.0
        self.last_pwm = 0
        self.last_fan = False

        # 제어 상태 변수 (Apply를 눌러야 갱신됨)
        self.current_pwm = 0
        self.current_fan = False
        self.current_pid_mode = False
        self.current_target = 0.0
        
        # CSV 및 로그 이름 관련 변수
        self.csv_file = None
        self.csv_writer = None
        self.current_log_name = None 

        # 타이머 설정 (아직 start하지 않음)
        self.tx_timer = QTimer()
        self.tx_timer.setInterval(100) # 10Hz (명령 전송)
        self.tx_timer.timeout.connect(self.send_heartbeat)

        self.plot_timer = QTimer()
        self.plot_timer.setInterval(50)  # 20Hz (화면 갱신)
        self.plot_timer.timeout.connect(self.update_ui)

        # 워커 생성 (아직 start하지 않음)
        self.worker = CanWorker()
        self.worker.data_received.connect(self.handle_new_data)
        self.worker.error_occurred.connect(self.handle_error)

        self.initUI()
        # 주의: __init__ 에서 통신이나 로깅을 바로 시작하지 않음 (Start 버튼 대기)

    def init_csv_logger(self):
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.current_log_name = f"log_{timestamp}"
            
            filename = f"{self.current_log_name}.csv"
            self.csv_file = open(filename, mode='w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Time(sec)", "Temperature(C)", "PWM(%)"])
            print(f"CSV Logging started: {filename}")
        except Exception as e:
            print(f"Failed to open CSV file: {e}")

    def save_snapshot(self):
        if self.current_log_name:
            try:
                pixmap = self.grab()
                image_filename = f"{self.current_log_name}.png"
                pixmap.save(image_filename, 'png')
                print(f"Snapshot saved: {image_filename}")
            except Exception as e:
                print(f"Failed to save snapshot: {e}")

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # 1. 그래프 영역
        self.temp_plot_widget = pg.PlotWidget()
        self.temp_plot_widget.setTitle("Temperature Monitor", color="w", size="12pt")
        self.temp_plot_widget.setLabel("left", "Temperature (°C)")
        self.temp_plot_widget.showGrid(x=True, y=True)
        self.temp_plot_widget.addLegend()
        self.temp_line = self.temp_plot_widget.plot(pen=pg.mkPen('y', width=2), name="Current Temp")
        self.target_line = self.temp_plot_widget.plot(pen=pg.mkPen('r', width=2, style=Qt.DashLine), name="Target Temp")
        
        self.pwm_plot_widget = pg.PlotWidget()
        self.pwm_plot_widget.setLabel("left", "PWM (%)")
        self.pwm_plot_widget.setLabel("bottom", "Time (s)")
        self.pwm_plot_widget.showGrid(x=True, y=True)
        self.pwm_plot_widget.setYRange(0, 105) 
        self.pwm_plot_widget.setXLink(self.temp_plot_widget)
        self.pwm_line = self.pwm_plot_widget.plot(pen=pg.mkPen('c', width=2), name="PWM Output")

        main_layout.addWidget(self.temp_plot_widget, stretch=2)
        main_layout.addWidget(self.pwm_plot_widget, stretch=1)
        
        # 2. 하단 패널
        control_panel = QHBoxLayout()
        main_layout.addLayout(control_panel, stretch=1)

        # [상태창]
        grp_status = QGroupBox("Current Status")
        layout_status = QGridLayout()
        self.lbl_temp = QLabel("0.0 °C")
        self.lbl_temp.setStyleSheet("font-size: 30px; font-weight: bold; color: #2196F3;")
        self.lbl_fan = QLabel("FAN: OFF")
        self.lbl_fan.setStyleSheet("font-size: 20px; font-weight: bold; color: gray;")
        self.lbl_pwm = QLabel("PWM: 0%")
        self.lbl_pwm.setStyleSheet("font-size: 20px;")

        layout_status.addWidget(QLabel("Temperature:"), 0, 0)
        layout_status.addWidget(self.lbl_temp, 0, 1)
        layout_status.addWidget(self.lbl_fan, 1, 0, 1, 2)
        layout_status.addWidget(self.lbl_pwm, 2, 0, 1, 2)
        grp_status.setLayout(layout_status)
        control_panel.addWidget(grp_status)

        # [제어 설정창]
        grp_control = QGroupBox("Control Settings")
        layout_control = QVBoxLayout()

        # 모드 선택
        self.radio_manual = QRadioButton("Manual Mode")
        self.radio_pid = QRadioButton("PID Mode")
        self.radio_manual.setChecked(True)
        
        self.btn_group = QButtonGroup()
        self.btn_group.addButton(self.radio_manual)
        self.btn_group.addButton(self.radio_pid)
        
        self.radio_manual.toggled.connect(self.on_mode_changed)
        self.radio_pid.toggled.connect(self.on_mode_changed)

        layout_control.addWidget(self.radio_manual)
        layout_control.addWidget(self.radio_pid)

        # 수동 제어 UI
        self.widget_manual = QWidget()
        lay_man = QHBoxLayout()
        lay_man.addWidget(QLabel("PWM (%):"))
        self.spin_pwm = QSpinBox()
        self.spin_pwm.setRange(0, 100)
        # Note: 값 변경 시 바로 업데이트하지 않음 (Apply 버튼 사용)
        lay_man.addWidget(self.spin_pwm)
        self.chk_fan = QCheckBox("FAN ON")
        lay_man.addWidget(self.chk_fan)
        self.widget_manual.setLayout(lay_man)
        
        # PID 제어 UI
        self.widget_pid = QWidget()
        lay_pid = QHBoxLayout()
        lay_pid.addWidget(QLabel("Target Temp:"))
        self.spin_target = QDoubleSpinBox()
        self.spin_target.setRange(0.0, 100.0)
        self.spin_target.setValue(25.0)
        self.spin_target.setSingleStep(0.5)
        lay_pid.addWidget(self.spin_target)
        self.widget_pid.setLayout(lay_pid)
        self.widget_pid.setVisible(False)

        layout_control.addWidget(self.widget_manual)
        layout_control.addWidget(self.widget_pid)
        
        # 구분선
        layout_control.addStretch()

        # [추가] Apply Settings 버튼
        self.btn_apply = QPushButton("Apply Settings")
        self.btn_apply.setMinimumHeight(40)
        self.btn_apply.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.btn_apply.clicked.connect(self.apply_settings)
        self.btn_apply.setEnabled(False) # Start 전에는 비활성화
        layout_control.addWidget(self.btn_apply)

        # [추가] Start 버튼
        self.btn_start = QPushButton("START SYSTEM")
        self.btn_start.setMinimumHeight(50)
        self.btn_start.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50; 
                color: white; 
                font-size: 16px; 
                font-weight: bold; 
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #e0e0e0; color: #a0a0a0; }
        """)
        self.btn_start.clicked.connect(self.start_system)
        layout_control.addWidget(self.btn_start)

        # Stop 버튼
        self.btn_stop = QPushButton("STOP (Emergency)")
        self.btn_stop.setMinimumHeight(50)
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: red; 
                color: white; 
                font-size: 16px; 
                font-weight: bold; 
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #d32f2f; }
            QPushButton:disabled { background-color: #e0e0e0; color: #a0a0a0; }
        """)
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_stop.setEnabled(False) # Start 전에는 비활성화
        layout_control.addWidget(self.btn_stop)
        
        grp_control.setLayout(layout_control)
        control_panel.addWidget(grp_control)

    # [추가] 시스템 시작 함수
    def start_system(self):
        # 1. UI 상태 변경
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_apply.setEnabled(True)
        
        # 2. 현재 입력된 값으로 초기 설정 적용
        self.apply_settings()

        # 3. CSV 로거 시작
        self.init_csv_logger()

        # 4. 워커 및 타이머 시작
        if not self.worker.isRunning():
            self.worker.start()
        self.tx_timer.start()
        self.plot_timer.start()
        
        print("System Started.")

    # [추가] 설정 적용 함수 (Apply 버튼)
    def apply_settings(self):
        # UI에 입력된 값을 내부 변수에 저장
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
        
        if self.csv_file:
            self.save_snapshot()
            self.csv_file.close()
            self.csv_file = None

    def emergency_stop(self):
        print("!!! EMERGENCY STOP !!!")
        
        self.current_pwm = 0
        self.current_fan = False
        self.current_pid_mode = False
        self.current_target = 0.0

        self.send_heartbeat() # 마지막으로 0 전송

        self.tx_timer.stop()
        self.plot_timer.stop()

        try:
            self.worker.data_received.disconnect(self.handle_new_data)
        except:
            pass

        if self.csv_file:
            self.save_snapshot()
            self.csv_file.close()
            self.csv_file = None
            print("CSV File Closed.")

        self.btn_stop.setText("STOPPED")
        self.btn_stop.setEnabled(False)
        self.btn_apply.setEnabled(False) # 정지 후 적용 불가
        self.btn_start.setEnabled(False) # 재시작 불가 (완전 종료 컨셉)
        
        self.lbl_pwm.setText("PWM: 0% (STOP)")
        self.lbl_pwm.setStyleSheet("font-size: 20px; font-weight: bold; color: red;")
        
        QMessageBox.warning(self, "System Stopped", "Current cut (PWM 0) and logging stopped.\nSnapshot Saved.")

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

        if self.csv_writer:
            try:
                self.csv_writer.writerow([f"{elapsed:.3f}", f"{temp:.2f}", pwm])
            except Exception:
                pass

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
        is_pid = self.radio_pid.isChecked()
        self.widget_pid.setVisible(is_pid)
        self.widget_manual.setVisible(not is_pid)
        # 모드를 바꿔도 바로 적용하지 않음 (Apply 눌러야 함)

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
    
    # Ctrl+C (SIGINT) 시그널 처리 함수
    def signal_handler(sig, frame):
        print("\nCtrl+C detected! Closing safely...")
        # 창을 닫으면 자동으로 closeEvent가 호출되어 스냅샷 저장 및 CSV 닫기가 수행됨
        window.close() 

    # 1. 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)

    # 2. 파이썬 인터프리터가 시그널을 주기적으로 확인할 수 있도록 더미 타이머 실행
    # (이게 없으면 PyQt 이벤트 루프 때문에 Ctrl+C가 바로 안 먹힐 수 있음)
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None) 

    sys.exit(app.exec_())