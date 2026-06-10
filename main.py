import time
import sys
import csv
import os
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg

from uart_driver import UartWorker, CH_OFF, CH_MANUAL, CH_TEMP, CH_FORCE
from imu_driver import ImuWorker
from force_mapper import ForceMapper
from posture_estimation import PostureEstimator

# ============================================================
#  설정
# ============================================================
# ── 통신 경로 설정 (CAN/UART 통합) ──
USE_UART_RX   = False    # COM 포트 없으면 자동 silent fail
USE_CAN_RX    = True     # Jetson 배포 시 True
USE_CAN_TX    = True     # Jetson 배포 시 True
CAN_INTERFACE = 'socketcan'
CAN_CHANNEL   = 'can0'

SERIAL_PORT = "/dev/ttyACM1" 
SERIAL_BAUD = 115200
DISPLAY_CH  = 0          # 표시할 채널 (0~5)

# 🌟 IMU 포트 설정
IMU_PORT = "/dev/ttyUSB1" 
IMU_BAUD = 921600

MODE_MANUAL = 0
MODE_TEMP   = 1
MODE_FORCE  = 2
MODE_CALIB  = 3

MAX_PLOT_POINTS = 500 # 50초 분량 정도

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("window.ui", self)

        if not os.path.exists("data_logs"):
            os.makedirs("data_logs")

        # ── 데이터 저장소 ──
        self.time_data   = []
        self.temp_data_ch0  = []
        self.target_data = []
        self.pwm_data    = []

        self.force_time_data = []
        self.force_data      = []
        self.disp_data       = []
        
        self.angle_data = []         
        self.target_force_data = []  
        
        self.l_thigh_data = []
        self.r_thigh_data = []
        
        self.imu_time_data = []
        self.imu_target_data = []
        self.imu_angle_data = []
        self.imu_start_time = time.time()
        
        self.imu_l_thigh_data = []
        self.imu_r_thigh_data = []
        
        self.imu_start_time = time.time()
        
        # ── 캐시 ──
        self.last_temp_ch0  = 0.0  
        self.last_pwm   = 0
        self.last_fan   = False
        self.last_force = 0.0
        self.maintain_force=0
        self.last_disp  = 0.0
        self.last_angle = 0.0
        self.last_velocity = 0.0
        self.last_thigh_mean_angle = 0.0
        self.last_thigh_velocity = 0.0
        self.last_thigh_time = time.time()                               
        self.last_l_thigh = 0.0 
        self.last_r_thigh = 0.0

        # ── 제어 상태 ──
        self.ctrl_mode            = MODE_MANUAL
        self.current_pwm          = 0
        self.current_fan          = False
        self.current_pid_mode     = False
        self.current_target       = 0.0
        self.current_force_target = 0.0
        
        self.stage1_force = 0.0
        self.stage2_force = 0.0
        self.force_alpha = 0.4
        self.filtered_target_force = 0.0 
        self.is_force_logging     = False

        # ── 타이머 ──
        self.tx_timer = QTimer()
        self.tx_timer.setInterval(10)
        self.tx_timer.timeout.connect(self.send_heartbeat)

        self.plot_timer = QTimer()
        self.plot_timer.setInterval(50)
        self.plot_timer.timeout.connect(self.update_ui)

        # ── 통신 워커 라우팅 (UART / CAN 통합) ──
        need_uart = USE_UART_RX or (not USE_CAN_TX)
        self.uart_worker = None
        if need_uart:
            self.uart_worker = UartWorker(port=SERIAL_PORT, baudrate=SERIAL_BAUD)
            self.uart_worker.error_occurred.connect(self.handle_error)
            if USE_UART_RX:
                self.uart_worker.data_received.connect(self.handle_new_data)
                self.uart_worker.force_received.connect(self.handle_force_data)
                self.uart_worker.debug_message.connect(self.handle_debug)

        self.can_worker = None
        if USE_CAN_RX or USE_CAN_TX:
            try:
                from can_driver import CanWorker
                self.can_worker = CanWorker(interface=CAN_INTERFACE, channel=CAN_CHANNEL, fd=True)
                self.can_worker.error_occurred.connect(self.handle_error)
                if USE_CAN_RX:
                    self.can_worker.data_received.connect(self.handle_new_data)
                    self.can_worker.force_received.connect(self.handle_force_data)
                    self.can_worker.debug_message.connect(self.handle_debug)
            except ImportError:
                print("[WARNING] python-can 모듈 미설치로 CAN 통신이 비활성화되었습니다.")

        self.tx_worker = self.can_worker if USE_CAN_TX else self.uart_worker
        self.worker = self.tx_worker  # 기존 코드 호환성 유지
        
        # 🌟 IMU 워커 초기화 
        self.imu_worker = ImuWorker(port=IMU_PORT, baudrate=IMU_BAUD)
        self.imu_worker.error_occurred.connect(self.handle_error)
        
        # 🌟 메인 통합 제어 루프 타이머 
        self.control_timer = QTimer(self)
        self.control_timer.setInterval(10)
        self.control_timer.timeout.connect(self.process_control_loop)
        
        # 🌟 Force Mapper + Posture Estimator
        self.force_mapper = ForceMapper(max_force=100.0, v_lift=-120.0, v_lower=120.0)
        self.posture_estimator = PostureEstimator()
        self.current_posture = "STANDING"
        self.current_lift_type = None

        self.init_plots()
        self.init_controls()
        
    def process_control_loop(self):
        if not hasattr(self, 'imu_worker') or not self.imu_worker.isRunning():
            return
            
        if not hasattr(self.imu_worker, 'current_angles') or not hasattr(self.imu_worker, 'current_vels'):
            return
            
        angles_dict = self.imu_worker.current_angles.copy()
        vels_dict = self.imu_worker.current_vels.copy()
        
        if not angles_dict or not vels_dict:
            return

        waist_angle    = angles_dict.get(0, 0.0)
        waist_velocity = vels_dict.get(0, 0.0)
        l_thigh        = angles_dict.get(1, 0.0)
        r_thigh        = angles_dict.get(2, 0.0)
        l_thigh_vel    = vels_dict.get(1, 0.0)
        r_thigh_vel    = vels_dict.get(2, 0.0)
        
        self.last_angle    = waist_angle
        self.last_l_thigh  = l_thigh  
        self.last_r_thigh  = r_thigh  
        self.last_velocity = waist_velocity
        
        thigh_mean = (l_thigh + r_thigh) / 2.0
        curr_time = time.time()
        dt = curr_time - self.last_thigh_time
        self.last_thigh_velocity = (thigh_mean - self.last_thigh_mean_angle)/dt if dt > 0 else 0.0
        self.last_thigh_mean_angle = thigh_mean
        self.last_thigh_time = curr_time
        
        angles = [waist_angle, l_thigh, r_thigh]
        vels   = [waist_velocity, l_thigh_vel, r_thigh_vel]
        posture_result = self.posture_estimator.update(angles, vels)
        
        self.current_posture   = posture_result['state']
        self.current_lift_type = posture_result['posture']
        self.current_confidence = posture_result['confidence']
        
        is_current_lifting = (self.current_posture == PostureEstimator.STATE_LIFTING)
        
        if is_current_lifting:
            raw_target_force = self.force_mapper.get_target_force(waist_angle, waist_velocity,self.current_lift_type)
        else :
            raw_target_force = 0.0 

        self.stage1_force = (self.force_alpha * raw_target_force) + (1.0 - self.force_alpha) * self.stage1_force
        self.stage2_force = (self.force_alpha * self.stage1_force) + (1.0 - self.force_alpha) * self.stage2_force

        if self.ctrl_mode == MODE_FORCE:
            self.current_force_target = self.stage2_force
            mapped_force = self.current_force_target
            
            self.spin_target_force.blockSignals(True)
            self.spin_target_force.setValue(mapped_force)
            self.spin_target_force.blockSignals(False)
            
            if self.is_force_logging:
                elapsed = time.time() - self.imu_start_time
                self.imu_time_data.append(elapsed)
                self.imu_target_data.append(mapped_force)
                self.imu_angle_data.append(waist_angle)
                self.imu_l_thigh_data.append(l_thigh) 
                self.imu_r_thigh_data.append(r_thigh) 

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_C:
            if hasattr(self, 'imu_worker') and self.imu_worker.isRunning():
                self.imu_worker.trigger_calibration()
                print("\n[캘리브레이션] 차렷 자세 0점 재설정 완료! 🎯\n")
        else:
            super().keyPressEvent(event)

    def init_plots(self):
        self.temp_plot_widget.setTitle("Temperature Monitor", color="w", size="12pt")
        self.temp_plot_widget.setLabel("left", "Temperature (°C)")
        self.temp_plot_widget.showGrid(x=True, y=True)
        self.temp_plot_widget.addLegend()
        self.temp_line_ch0 = self.temp_plot_widget.plot(pen=pg.mkPen('y', width=2), name="CH0 Temp")
        self.target_line = self.temp_plot_widget.plot(pen=pg.mkPen('r', width=2, style=Qt.DashLine), name="Target Temp")

        self.pwm_plot_widget.setTitle("PWM Output", color="w", size="12pt")
        self.pwm_plot_widget.setLabel("left", "PWM (%)")
        self.pwm_plot_widget.setLabel("bottom", "Time (s)")
        self.pwm_plot_widget.showGrid(x=True, y=True)
        self.pwm_plot_widget.setYRange(0, 105)
        self.pwm_plot_widget.setXLink(self.temp_plot_widget)
        self.pwm_line = self.pwm_plot_widget.plot(pen=pg.mkPen('c', width=2), name="PWM")

        self.force_plot_widget.setTitle("Force Monitor", color="w", size="12pt")
        self.force_plot_widget.setLabel("left", "Force (g)")
        self.force_plot_widget.showGrid(x=True, y=True)
        self.force_plot_widget.setYRange(0, 100)
        self.force_plot_widget.addLegend()
        self.force_line = self.force_plot_widget.plot(pen=pg.mkPen(color='#AA00FF', width=2), name="Force (g)")
        self.force_target_line = self.force_plot_widget.plot(pen=pg.mkPen(color='#FF4444', width=2, style=Qt.DashLine), name="Target Force")

        self.fpwm_plot_widget.setTitle("PWM Output", color="w", size="12pt")
        self.fpwm_plot_widget.setLabel("left", "PWM (%)")
        self.fpwm_plot_widget.showGrid(x=True, y=True)
        self.fpwm_plot_widget.setYRange(0, 105)
        self.fpwm_plot_widget.setXLink(self.force_plot_widget)
        self.fpwm_line = self.fpwm_plot_widget.plot(pen=pg.mkPen('c', width=2), name="PWM")

        self.ftemp_plot_widget.setTitle("Temperature", color="w", size="12pt")
        self.ftemp_plot_widget.setLabel("left", "Temperature (°C)")
        self.ftemp_plot_widget.setLabel("bottom", "Time (s)")
        self.ftemp_plot_widget.showGrid(x=True, y=True)
        self.ftemp_plot_widget.setXLink(self.force_plot_widget)
        self.ftemp_line_ch0 = self.ftemp_plot_widget.plot(pen=pg.mkPen('y', width=2), name="CH0")

    def init_controls(self):
        self.radio_manual.toggled.connect(self.on_mode_changed)
        self.radio_temp.toggled.connect(self.on_mode_changed)
        self.radio_force.toggled.connect(self.on_mode_changed)
        self.radio_calib.toggled.connect(self.on_mode_changed)

        self.btn_apply.clicked.connect(self.apply_settings)
        self.btn_start.clicked.connect(self.start_system)
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_save.clicked.connect(self.manual_save)
        self.btn_calib.clicked.connect(self.calibrate_imu)

        self.on_mode_changed()

    def on_mode_changed(self):
        manual = self.radio_manual.isChecked()
        temp   = self.radio_temp.isChecked()
        force  = self.radio_force.isChecked()
        calib  = self.radio_calib.isChecked()

        self.widget_manual.setVisible(manual)
        self.widget_pid.setVisible(temp)
        self.widget_force.setVisible(force)
        self.widget_calib.setVisible(calib)

        self.plot_stack.setCurrentIndex(1 if force else 0)

        if manual:
            self.ctrl_mode = MODE_MANUAL
        elif temp:
            self.ctrl_mode = MODE_TEMP
        elif force:
            self.ctrl_mode = MODE_FORCE
        else:
            self.ctrl_mode = MODE_CALIB

    def start_system(self):
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_apply.setEnabled(True)

        if self.uart_worker and not self.uart_worker.isRunning():
            self.uart_worker.start()
        if self.can_worker and not self.can_worker.isRunning():
            self.can_worker.start()
            
        if hasattr(self, 'imu_worker') and not self.imu_worker.isRunning():
            self.imu_worker.start()
                
        self.apply_settings()
        self.tx_timer.start()
        self.plot_timer.start()
        self.control_timer.start() 

        rx_paths = []
        if USE_UART_RX: rx_paths.append("UART")
        if USE_CAN_RX:  rx_paths.append(f"CAN/{CAN_CHANNEL}")
        tx_path = f"CAN/{CAN_CHANNEL}" if USE_CAN_TX else f"UART({SERIAL_PORT}@{SERIAL_BAUD})"
        print(f"System Started. Mode={self.ctrl_mode}  TX={tx_path}  RX=[{', '.join(rx_paths) or 'none'}]")

    def apply_settings(self):
        if self.ctrl_mode == MODE_MANUAL:
            self.current_pwm      = self.spin_pwm.value()
            self.current_fan      = self.chk_fan.isChecked()
            self.current_pid_mode = False
            print(f"Applied: Manual  PWM={self.current_pwm}  FAN={self.current_fan}")

        elif self.ctrl_mode == MODE_TEMP:
            self.current_target   = self.spin_target.value()
            self.current_pid_mode = True
            print(f"Applied: Temp PID  Target={self.current_target}°C")

        elif self.ctrl_mode == MODE_FORCE:
            self.current_force_target = self.spin_target_force.value()
            self.current_pid_mode     = False
            self.current_pwm          = 0
            print(f"Applied: Force Ctrl  Target={self.current_force_target}g  CH={DISPLAY_CH}")
            
            self.force_time_data.clear()
            self.force_data.clear()
            self.disp_data.clear()
            self.angle_data.clear()
            self.l_thigh_data.clear() 
            self.r_thigh_data.clear() 
            self.target_force_data.clear()
            
            self.imu_time_data.clear()
            self.imu_target_data.clear()
            if hasattr(self, 'imu_angle_data'):
                self.imu_angle_data.clear()
                self.imu_l_thigh_data.clear() 
                self.imu_r_thigh_data.clear() 
            
            self.imu_start_time = time.time()
            self.is_force_logging = True
            
        else:
            self.current_pid_mode     = False
            self.current_pwm          = 0
            print("Applied: Calibration Mode. Systems halted for safety.")

        self._send_current_command()

    def _send_current_command(self):
        if not self.tx_worker: return
        
        if self.ctrl_mode == MODE_MANUAL:
            self.tx_worker.send_control_message(
                mode=CH_MANUAL, manual_pwm=self.current_pwm, manual_fan=self.current_fan,
                target=0.0, display_ch=DISPLAY_CH)
        elif self.ctrl_mode == MODE_TEMP:
            self.tx_worker.send_control_message(
                mode=CH_TEMP, manual_pwm=0, manual_fan=False,
                target=self.current_target, display_ch=DISPLAY_CH)
        elif self.ctrl_mode == MODE_FORCE: 
            self.tx_worker.send_control_message(
                mode=CH_FORCE, manual_pwm=0, manual_fan=False,
                target=self.current_force_target, display_ch=DISPLAY_CH)
        else:
            self.tx_worker.send_control_message(
                mode=CH_OFF, manual_pwm=0, manual_fan=False,
                target=0.0, display_ch=DISPLAY_CH)
            
    def calibrate_imu(self):
        if hasattr(self, 'imu_worker') and self.imu_worker and self.imu_worker.isRunning():
            self.imu_worker.trigger_calibration() 
            self.statusBar().showMessage("모든 IMU 센서(허리/양측 허벅지) Zero 초기화 진행 중 (30샘플 수집)...", 3000)
            print("[GUI] 모든 IMU 센서 제로 초기화 명령 전송 완료.")
        else:
            QMessageBox.warning(self, "Warning", "IMU가 연결되어 있지 않거나 동작 중이 아닙니다.")

    def send_heartbeat(self):
        self._send_current_command()

    def stop_all(self):
        if self.uart_worker:
            self.uart_worker.running = False
            self.uart_worker.wait()
        if self.can_worker:
            self.can_worker.stop()
            self.can_worker.wait()
        
        if hasattr(self, 'imu_worker'):
            self.imu_worker.stop()
            self.imu_worker.wait()
        
        self.tx_timer.stop()
        self.plot_timer.stop()
        self.control_timer.stop() 

    def emergency_stop(self):
        print("!!! EMERGENCY STOP !!!")
        self.current_pwm      = 0
        self.current_fan      = False
        self.current_pid_mode = False
        self.current_target   = 0.0
        self.is_force_logging = False
        
        if hasattr(self, 'imu_worker'):
            self.imu_worker.stop()
            self.imu_worker.wait()
        
        if self.tx_worker:
            self.tx_worker.send_control_message(
                mode=CH_OFF, manual_pwm=0, manual_fan=False, target=0.0, display_ch=DISPLAY_CH)
            
        for w in [self.uart_worker, self.can_worker]:
            if w is not None:
                for sig, slot in [(w.data_received,  self.handle_new_data),
                                  (w.force_received, self.handle_force_data)]:
                    try:
                        sig.disconnect(slot)
                    except Exception:
                        pass
            
        self.tx_timer.stop()
        self.plot_timer.stop()
        self.control_timer.stop() 
               
        self.btn_stop.setText("STOPPED")
        self.btn_stop.setEnabled(False)
        self.btn_apply.setEnabled(False)
        self.btn_start.setEnabled(False)

        self.lbl_pwm.setText("PWM: 0% (STOP)")
        self.lbl_pwm.setStyleSheet("font-size: 18px; font-weight: bold; color: red;")

        QMessageBox.warning(self, "System Stopped", "Emergency Stop! Use 'SAVE DATA' button to save logs.")

    def closeEvent(self, event):
        self.stop_all()
        event.accept()

    def handle_new_data(self, elapsed, temp_list, fan_list, pwm_list):
        ch   = DISPLAY_CH
        temp_ch0 = temp_list[0]
        fan  = fan_list[ch]
        pwm  = pwm_list[ch]

        self.time_data.append(elapsed)
        self.temp_data_ch0.append(temp_ch0) 
        self.pwm_data.append(pwm)
        self.target_data.append(self.current_target if self.current_pid_mode else float('nan'))

        self.last_temp_ch0 = temp_ch0
        self.last_pwm  = pwm
        self.last_fan  = bool(fan)

    # 🌟 UART / CAN 모듈 업데이트에 맞춰 시그널 파라미터(ch) 대응
    def handle_force_data(self, elapsed, ch, force, displacement):
        if ch != DISPLAY_CH:
            return
            
        if self.is_force_logging:
            self.force_time_data.append(elapsed)
            self.force_data.append(force)
            self.disp_data.append(displacement)
            
            self.angle_data.append(self.last_angle)
            self.l_thigh_data.append(self.last_l_thigh) 
            self.r_thigh_data.append(self.last_r_thigh) 
            self.target_force_data.append(self.current_force_target)
        
        self.last_force = force
        self.last_disp  = displacement

    def handle_debug(self, msg):
        print(f"[MCU] {msg}")

    def handle_error(self, msg):
        print(f"[ERROR] {msg}")

    def update_ui(self):
        self.lbl_temp.setText(f"CH0: {self.last_temp_ch0:.1f}°C")
        self.lbl_pwm.setText(f"PWM: {self.last_pwm}%")
        self.lbl_force.setText(f"Force: {self.last_force:.2f} g")
        self.lbl_disp.setText(f"Disp:  {self.last_disp:.2f} mm")
        self.lbl_angle.setText(f"Torso Angle: {self.last_angle:.1f} °")
        self.lbl_angle_velocity.setText(f"Angular velocity: {self.last_velocity:.1f} °/s")
        self.lbl_angle.setText(f"Torso angle: {self.last_angle:.1f} °")
        self.lbl_angle_velocity.setText(f"Torso ang_v: {self.last_velocity:.1f} °/s")

        if hasattr(self, 'lbl_thigh_mean_angle'):
            self.lbl_thigh_mean_angle.setText(f"Thigh mean angle: {self.last_thigh_mean_angle:.1f} °")
        if hasattr(self, 'lbl_thigh_velocity'):
            self.lbl_thigh_velocity.setText(f"Thigh ang_v: {self.last_thigh_velocity:.1f} °/s")
        if hasattr(self, 'lbl_posture'):
            if self.current_posture == 'Lifting' and self.current_lift_type:
                self.lbl_posture.setText(f"Posture: {self.current_posture} ({self.current_lift_type})")
            else:
                self.lbl_posture.setText(f"Posture: {self.current_posture}")

            conf_val = getattr(self, 'current_confidence', 0.0) * 100
            display_lift = self.current_lift_type if self.current_posture == 'Lifting' else 'Waiting...'
            print(f"\r🚀 [실시간 추정] 상태: {self.current_posture:<10} | 동작: {display_lift:<15} | 신뢰도: {conf_val:5.1f}%   ", end="", flush=True)

        if self.last_temp_ch0 > 70.0:
            self.lbl_temp.setStyleSheet("font-size: 20px; font-weight: bold; color: red;")
        else:
            self.lbl_temp.setStyleSheet("font-size: 20px; font-weight: bold; color: #2196F3;")

        self.lbl_fan.setText("FAN: ON" if self.last_fan else "FAN: OFF")
        self.lbl_fan.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {'green' if self.last_fan else 'gray'};")

        if self.ctrl_mode in (MODE_MANUAL, MODE_TEMP) and self.time_data:
            plot_time = self.time_data[-MAX_PLOT_POINTS:]
            self.temp_line_ch0.setData(plot_time, self.temp_data_ch0[-MAX_PLOT_POINTS:])
            self.target_line.setData(plot_time, self.target_data[-MAX_PLOT_POINTS:])
            self.pwm_line.setData(plot_time, self.pwm_data[-MAX_PLOT_POINTS:])

        if self.ctrl_mode == MODE_FORCE:
            if self.force_time_data:
                plot_ftime = self.force_time_data[-MAX_PLOT_POINTS:]
                self.force_line.setData(plot_ftime, self.force_data[-MAX_PLOT_POINTS:])
                
            if self.imu_time_data:
                plot_itime = self.imu_time_data[-MAX_PLOT_POINTS:]
                self.force_target_line.setData(plot_itime, self.imu_target_data[-MAX_PLOT_POINTS:])
                    
            if self.time_data:
                plot_time = self.time_data[-MAX_PLOT_POINTS:]
                self.fpwm_line.setData(plot_time, self.pwm_data[-MAX_PLOT_POINTS:])
                self.ftemp_line_ch0.setData(plot_time, self.temp_data_ch0[-MAX_PLOT_POINTS:])


    def manual_save(self):
        user_input    = self.edit_filename.text().strip()
        filename_base = user_input if user_input else f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        try:
            with open(f"data_logs/{filename_base}.csv", mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Time(sec)", "Temp_CH0(C)", "PWM(%)"])
                for i in range(len(self.time_data)):
                    writer.writerow([f"{self.time_data[i]:.3f}", f"{self.temp_data_ch0[i]:.2f}", self.pwm_data[i]])

            with open(f"data_logs/{filename_base}_force.csv", mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Time(sec)", "Force(g)", "Displacement(mm)", "Waist_Angle(deg)", "L_Thigh_Angle(deg)", "R_Thigh_Angle(deg)", "Target_Force(g)"])
                
                if len(self.force_time_data) > 0:
                    length = len(self.force_time_data)
                    for i in range(length):
                        a_val = self.angle_data[i] if i < len(self.angle_data) else 0.0
                        l_val = self.l_thigh_data[i] if hasattr(self, 'l_thigh_data') and i < len(self.l_thigh_data) else 0.0 
                        r_val = self.r_thigh_data[i] if hasattr(self, 'r_thigh_data') and i < len(self.r_thigh_data) else 0.0 
                        tf_val = self.target_force_data[i] if i < len(self.target_force_data) else 0.0
                        writer.writerow([
                            f"{self.force_time_data[i]:.3f}",
                            f"{self.force_data[i]:.3f}",
                            f"{self.disp_data[i]:.3f}",
                            f"{a_val:.2f}",
                            f"{l_val:.2f}", 
                            f"{r_val:.2f}", 
                            f"{tf_val:.2f}"
                        ])
                else:
                    length = len(self.imu_time_data)
                    for i in range(length):
                        a_val = self.imu_angle_data[i] if hasattr(self, 'imu_angle_data') and i < len(self.imu_angle_data) else 0.0
                        l_val = self.imu_l_thigh_data[i] if hasattr(self, 'imu_l_thigh_data') and i < len(self.imu_l_thigh_data) else 0.0 
                        r_val = self.imu_r_thigh_data[i] if hasattr(self, 'imu_r_thigh_data') and i < len(self.imu_r_thigh_data) else 0.0 
                        tf_val = self.imu_target_data[i] if i < len(self.imu_target_data) else 0.0
                        writer.writerow([
                            f"{self.imu_time_data[i]:.3f}",
                            "0.000",
                            "0.000",
                            f"{a_val:.2f}",
                            f"{l_val:.2f}", 
                            f"{r_val:.2f}", 
                            f"{tf_val:.2f}"
                        ])

            self.grab().save(f"data_logs/{filename_base}.png", 'png')
            QMessageBox.information(self, "Save Success", f"Saved to data_logs/{filename_base}")

        except Exception as e:
            QMessageBox.critical(self, "Save Error", str(e))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())