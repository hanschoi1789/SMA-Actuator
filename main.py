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
ENABLE_UART = False
SERIAL_PORT = "COM6"
SERIAL_BAUD = 115200
DISPLAY_CH  = 0          # 표시할 채널 (0~5)


# 🌟 IMU 포트 설정 (윈도우 환경이시면 장치관리자 확인 후 'COMx'로 변경 필요)
IMU_PORT = "/dev/ttyUSB0" 
IMU_BAUD = 921600

MODE_MANUAL = 0
MODE_TEMP   = 1
MODE_FORCE  = 2
MODE_CALIB  = 3

MAX_PLOT_POINTS = 1000 # 50초 분량 정도


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("window.ui", self)

        if not os.path.exists("data_logs"):
            os.makedirs("data_logs")

        # ── 데이터 저장소 ──
        self.time_data   = []
        self.temp_data_ch_0  = []
        self.temp_data_ch_1 = []
        self.target_data = []
        self.pwm_data    = []

        self.force_time_data = []
        self.force_data      = []
        self.disp_data       = []
        
        self.angle_data = []         # 🌟 각도 데이터용 리스트
        self.target_force_data = []  # 🌟 타겟 힘 데이터용 리스트
        
        #IMUtest용 
        self.imu_time_data = []
        self.imu_target_data = []
        self.imu_angle_data = []
        self.imu_start_time = time.time()
        
        # ── 캐시 ──
        self.last_temp_ch0  = 0.0  # 🌟 CH0 최신 온도
        self.last_temp_ch1  = 0.0  # 🌟 CH1 최신 온도
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

        # ── 제어 상태 ──
        self.ctrl_mode            = MODE_MANUAL
        self.current_pwm          = 0
        self.current_fan          = False
        self.current_pid_mode     = False
        self.current_target       = 0.0
        self.current_force_target = 0.0
        
        # 타겟 힘 로우패스(EMA) 필터용 변수 추가
        self.stage1_force = 0.0
        self.stage2_force = 0.0
        
        # 튜닝 파라미터 (2번 필터링되므로 기존보다 약간 높여야 응답성이 유지됨)
        self.force_alpha = 0.3
        self.filtered_target_force = 0.0 # 이전 필터링 결과 저장용
        
        # Force 제어 로깅 활성화 플래그
        self.is_force_logging     = False

        # ── 타이머 ──
        self.tx_timer = QTimer()
        self.tx_timer.setInterval(10)
        self.tx_timer.timeout.connect(self.send_heartbeat)

        self.plot_timer = QTimer()
        self.plot_timer.setInterval(50)
        self.plot_timer.timeout.connect(self.update_ui)

        # ── UART 워커 ──
        self.worker = None

        if ENABLE_UART:
            self.worker = UartWorker(port=SERIAL_PORT, baudrate=SERIAL_BAUD)
            self.worker.data_received.connect(self.handle_new_data)
            self.worker.force_received.connect(self.handle_force_data)
            self.worker.debug_message.connect(self.handle_debug)
            self.worker.error_occurred.connect(self.handle_error)
        
        # 🌟 IMU 워커 초기화 (이벤트 큐 과부하를 막기 위해 시그널 연결 삭제)
        self.imu_worker = ImuWorker(port=IMU_PORT, baudrate=IMU_BAUD)
        self.imu_worker.error_occurred.connect(self.handle_error)
        
        # 🌟 10ms 주기로 동작하는 메인 통합 제어 루프 타이머 추가
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
        """🌟 10ms 주기로 IMU 데이터를 직접 Pull 해와서 자세 추정 및 힘 계산을 수행하는 함수"""
        if not hasattr(self, 'imu_worker') or not self.imu_worker.isRunning():
            return
            
        # IMU 워커에서 직접 최신 데이터 스냅샷을 복사해옴 (백그라운드 스레드와 분리)
        if not hasattr(self.imu_worker, 'current_angles') or not hasattr(self.imu_worker, 'current_vels'):
            return
            
        angles_dict = self.imu_worker.current_angles.copy()
        vels_dict = self.imu_worker.current_vels.copy()
        
        # 데이터가 아직 안 찼으면 무시
        if not angles_dict or not vels_dict:
            return

        # 1. 데이터 추출
        waist_angle    = angles_dict.get(0, 0.0)
        waist_velocity = vels_dict.get(0, 0.0)
        l_thigh        = angles_dict.get(1, 0.0)
        r_thigh        = angles_dict.get(2, 0.0)
        l_thigh_vel    = vels_dict.get(1, 0.0)
        r_thigh_vel    = vels_dict.get(2, 0.0)
        
        self.last_angle    = waist_angle
        self.last_velocity = waist_velocity
        
        # 2. 허벅지 평균/각속도 (UI 표시용)
        thigh_mean = (l_thigh + r_thigh) / 2.0
        curr_time = time.time()
        dt = curr_time - self.last_thigh_time
        self.last_thigh_velocity = (thigh_mean - self.last_thigh_mean_angle)/dt if dt > 0 else 0.0
        self.last_thigh_mean_angle = thigh_mean
        self.last_thigh_time = curr_time
        
        # 3. 자세 추정
        angles = [waist_angle, l_thigh, r_thigh]
        vels   = [waist_velocity, l_thigh_vel, r_thigh_vel]
        posture_result = self.posture_estimator.update(angles, vels)
        
        # 결과 저장 (UI 표시용)
        self.current_posture   = posture_result['state']
        self.current_lift_type = posture_result['posture']
        self.current_confidence = posture_result['confidence']
        
        # 4. 자세에 따른 타겟 힘 계산 분기
        is_current_lifting = (self.current_posture == PostureEstimator.STATE_LIFTING)
        
        if is_current_lifting:
            
            raw_target_force = self.force_mapper.get_target_force(waist_angle, waist_velocity,self.current_lift_type)
            
        else :
            raw_target_force = 0.0 

        # 🌟 5. Double EMA (2차 로우패스 필터) 적용
        # 1단: 목표값 변화를 1차 완화
        self.stage1_force = (self.force_alpha * raw_target_force) + (1.0 - self.force_alpha) * self.stage1_force
        
        # 2단: 1단 값을 한 번 더 추종하여 완벽한 S-Curve(2차 곡선) 생성
        self.stage2_force = (self.force_alpha * self.stage1_force) + (1.0 - self.force_alpha) * self.stage2_force

        
        # 6. Force 모드일 때 GUI 반영 + 로깅
        if self.ctrl_mode == MODE_FORCE:
            # 최종 출력은 stage2_force를 사용
            self.current_force_target = self.stage2_force
            mapped_force = self.current_force_target
            
            # SpinBox 업데이트 시 시그널 블록하여 무한 루프 방지
            self.spin_target_force.blockSignals(True)
            self.spin_target_force.setValue(mapped_force)
            self.spin_target_force.blockSignals(False)
            
            if self.is_force_logging:
                elapsed = time.time() - self.imu_start_time
                self.imu_time_data.append(elapsed)
                self.imu_target_data.append(mapped_force)
                self.imu_angle_data.append(waist_angle)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_C:
            if hasattr(self, 'imu_worker') and self.imu_worker.isRunning():
                self.imu_worker.trigger_calibration()
                print("\n[캘리브레이션] 차렷 자세 0점 재설정 완료! 🎯\n")
        else:
            super().keyPressEvent(event)

    # ──────────────────────────────────────────────────────────
    #  그래프 초기화
    # ──────────────────────────────────────────────────────────

    def init_plots(self):
        # ── Manual/Temp 페이지 ──
        self.temp_plot_widget.setTitle("Temperature Monitor", color="w", size="12pt")
        self.temp_plot_widget.setLabel("left", "Temperature (°C)")
        self.temp_plot_widget.showGrid(x=True, y=True)
        self.temp_plot_widget.addLegend()
        self.temp_line_ch0 = self.temp_plot_widget.plot(pen=pg.mkPen('y', width=2), name="CH0 Temp")
        self.temp_line_ch1 = self.temp_plot_widget.plot(pen=pg.mkPen('g', width=2), name="CH1 Temp")
        self.target_line = self.temp_plot_widget.plot(
            pen=pg.mkPen('r', width=2, style=Qt.DashLine), name="Target Temp")

        self.pwm_plot_widget.setTitle("PWM Output", color="w", size="12pt")
        self.pwm_plot_widget.setLabel("left", "PWM (%)")
        self.pwm_plot_widget.setLabel("bottom", "Time (s)")
        self.pwm_plot_widget.showGrid(x=True, y=True)
        self.pwm_plot_widget.setYRange(0, 105)
        self.pwm_plot_widget.setXLink(self.temp_plot_widget)
        self.pwm_line = self.pwm_plot_widget.plot(
            pen=pg.mkPen('c', width=2), name="PWM")

        # ── Force 페이지 ──
        self.force_plot_widget.setTitle("Force Monitor", color="w", size="12pt")
        self.force_plot_widget.setLabel("left", "Force (g)")
        self.force_plot_widget.showGrid(x=True, y=True)
        self.force_plot_widget.setYRange(0, 100)
        self.force_plot_widget.addLegend()
        self.force_line = self.force_plot_widget.plot(
            pen=pg.mkPen(color='#AA00FF', width=2), name="Force (g)")
        self.force_target_line = self.force_plot_widget.plot(
            pen=pg.mkPen(color='#FF4444', width=2, style=Qt.DashLine),
            name="Target Force")

        self.fpwm_plot_widget.setTitle("PWM Output", color="w", size="12pt")
        self.fpwm_plot_widget.setLabel("left", "PWM (%)")
        self.fpwm_plot_widget.showGrid(x=True, y=True)
        self.fpwm_plot_widget.setYRange(0, 105)
        self.fpwm_plot_widget.setXLink(self.force_plot_widget)
        self.fpwm_line = self.fpwm_plot_widget.plot(
            pen=pg.mkPen('c', width=2), name="PWM")

        self.ftemp_plot_widget.setTitle("Temperature", color="w", size="12pt")
        self.ftemp_plot_widget.setLabel("left", "Temperature (°C)")
        self.ftemp_plot_widget.setLabel("bottom", "Time (s)")
        self.ftemp_plot_widget.showGrid(x=True, y=True)
        self.ftemp_plot_widget.setXLink(self.force_plot_widget)
        self.ftemp_line_ch0 = self.ftemp_plot_widget.plot(pen=pg.mkPen('y', width=2), name="CH0")
        self.ftemp_line_ch1 = self.ftemp_plot_widget.plot(pen=pg.mkPen('g', width=2), name="CH1")

    # ──────────────────────────────────────────────────────────
    #  버튼/라디오 초기화
    # ──────────────────────────────────────────────────────────

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

    # ──────────────────────────────────────────────────────────
    #  시스템 제어
    # ──────────────────────────────────────────────────────────

    def start_system(self):
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_apply.setEnabled(True)

        if self.worker and not self.worker.isRunning():
            self.worker.start()
            
        if hasattr(self, 'imu_worker'):
            if not self.imu_worker.isRunning():
                self.imu_worker.start()
                
        self.apply_settings()
        self.tx_timer.start()
        self.plot_timer.start()
        self.control_timer.start() # 🌟 제어 루프 타이머 시작

        print(f"System Started. Mode={self.ctrl_mode}  "
              f"(UART: {SERIAL_PORT} @ {SERIAL_BAUD}baud)")

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
            self.target_force_data.clear()
            
            self.imu_time_data.clear()
            self.imu_target_data.clear()
            if hasattr(self, 'imu_angle_data'):
                self.imu_angle_data.clear()
            
            self.imu_start_time = time.time()
            self.is_force_logging = True
            
        else:
            self.current_pid_mode     = False
            self.current_pwm          = 0
            print("Applied: Calibration Mode. Systems halted for safety.")

        self._send_current_command()

    def _send_current_command(self):
        if not self.worker: return
        
        if self.ctrl_mode == MODE_MANUAL:
            self.worker.send_control_message(
                mode=CH_MANUAL,
                manual_pwm=self.current_pwm,
                manual_fan=self.current_fan,
                target=0.0,
                display_ch=DISPLAY_CH)
        elif self.ctrl_mode == MODE_TEMP:
            self.worker.send_control_message(
                mode=CH_TEMP,
                manual_pwm=0,
                manual_fan=False,
                target=self.current_target,
                display_ch=DISPLAY_CH)
        elif self.ctrl_mode == MODE_FORCE: 
            self.worker.send_control_message(
                mode=CH_FORCE,
                manual_pwm=0,
                manual_fan=False,
                target=self.current_force_target,
                display_ch=DISPLAY_CH)
        else:
            self.worker.send_control_message(
                mode=CH_OFF,
                manual_pwm=0,
                manual_fan=False,
                target=0.0,
                display_ch=DISPLAY_CH)
            
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
        if self.worker:
            self.worker.running = False
            self.worker.wait()
        
        if hasattr(self, 'imu_worker'):
            self.imu_worker.stop()
            self.imu_worker.wait()
        
        self.tx_timer.stop()
        self.plot_timer.stop()
        self.control_timer.stop() # 🌟 제어 루프 타이머 정지
        

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
        
        if self.worker:
            self.worker.send_control_message(
                mode=CH_OFF,
                manual_pwm=0,
                manual_fan=False,
                target=0.0,
                display_ch=DISPLAY_CH)
            
            for sig, slot in [(self.worker.data_received,  self.handle_new_data),
                (self.worker.force_received, self.handle_force_data)]:
                try:
                    sig.disconnect(slot)
                except Exception:
                    pass
            
        self.tx_timer.stop()
        self.plot_timer.stop()
        self.control_timer.stop() # 🌟 제어 루프 타이머 정지
               
        self.btn_stop.setText("STOPPED")
        self.btn_stop.setEnabled(False)
        self.btn_apply.setEnabled(False)
        self.btn_start.setEnabled(False)

        self.lbl_pwm.setText("PWM: 0% (STOP)")
        self.lbl_pwm.setStyleSheet("font-size: 18px; font-weight: bold; color: red;")

        QMessageBox.warning(self, "System Stopped",
            "Emergency Stop! Use 'SAVE DATA' button to save logs.")

    def closeEvent(self, event):
        self.stop_all()
        event.accept()

    # ──────────────────────────────────────────────────────────
    #  데이터 수신
    # ──────────────────────────────────────────────────────────

    def handle_new_data(self, elapsed, temp_list, fan_list, pwm_list):
        ch   = DISPLAY_CH
        temp_ch0 = temp_list[0]
        temp_ch1 = temp_list[1]
        fan  = fan_list[ch]
        pwm  = pwm_list[ch]

        self.time_data.append(elapsed)
        self.temp_data_ch0.append(temp_ch0) # 🌟 CH0 저장
        self.temp_data_ch1.append(temp_ch1) # 🌟 CH1 저장
        self.pwm_data.append(pwm)
        self.target_data.append(
            self.current_target if self.current_pid_mode else float('nan'))

        self.last_temp_ch0 = temp_ch0
        self.last_temp_ch1 = temp_ch1
        self.last_pwm  = pwm
        self.last_fan  = bool(fan)

    def handle_force_data(self, elapsed, force, displacement):
        if self.is_force_logging:
            self.force_time_data.append(elapsed)
            self.force_data.append(force)
            self.disp_data.append(displacement)
            
            self.angle_data.append(self.last_angle)
            self.target_force_data.append(self.current_force_target)
        
        self.last_force = force
        self.last_disp  = displacement

    def handle_debug(self, msg):
        print(f"[MCU] {msg}")

    def handle_error(self, msg):
        print(f"[ERROR] {msg}")

    # ──────────────────────────────────────────────────────────
    #  UI 업데이트 (20 Hz)
    # ──────────────────────────────────────────────────────────

    def update_ui(self):
        self.lbl_temp.setText(f"CH0: {self.last_temp_ch0:.1f}°C | CH1: {self.last_temp_ch1:.1f}°C")
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

        if self.last_temp_ch0 > 70.0 or self.last_temp_ch1 > 70.0:
            self.lbl_temp.setStyleSheet("font-size: 20px; font-weight: bold; color: red;")
        else:
            self.lbl_temp.setStyleSheet("font-size: 20px; font-weight: bold; color: #2196F3;")

        self.lbl_fan.setText("FAN: ON" if self.last_fan else "FAN: OFF")
        self.lbl_fan.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {'green' if self.last_fan else 'gray'};")

        if self.ctrl_mode in (MODE_MANUAL, MODE_TEMP) and self.time_data:
            plot_time = self.time_data[-MAX_PLOT_POINTS:]
            self.temp_line_ch0.setData(plot_time, self.temp_data_ch0[-MAX_PLOT_POINTS:])
            self.temp_line_ch1.setData(plot_time, self.temp_data_ch1[-MAX_PLOT_POINTS:])
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
                self.ftemp_line_ch1.setData(plot_time, self.temp_data_ch1[-MAX_PLOT_POINTS:])


    # ──────────────────────────────────────────────────────────
    #  저장
    # ──────────────────────────────────────────────────────────

    def manual_save(self):
        user_input    = self.edit_filename.text().strip()
        filename_base = user_input if user_input else \
            f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        try:
            with open(f"data_logs/{filename_base}.csv",
                      mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Time(sec)", "Temp_CH0(C)", "Temp_CH1(C)", "PWM(%)"])
                for i in range(len(self.time_data)):
                    writer.writerow([f"{self.time_data[i]:.3f}",
                                     f"{self.temp_data_ch0[i]:.2f}",
                                     f"{self.temp_data_ch1[i]:.2f}",
                                     self.pwm_data[i]])

            with open(f"data_logs/{filename_base}_force.csv",
                      mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Time(sec)", "Force(g)", "Displacement(mm)", "Angle(deg)", "Target_Force(g)"])
                
                if len(self.force_time_data) > 0:
                    length = len(self.force_time_data)
                    for i in range(length):
                        a_val = self.angle_data[i] if i < len(self.angle_data) else 0.0
                        tf_val = self.target_force_data[i] if i < len(self.target_force_data) else 0.0
                        writer.writerow([
                            f"{self.force_time_data[i]:.3f}",
                            f"{self.force_data[i]:.3f}",
                            f"{self.disp_data[i]:.3f}",
                            f"{a_val:.2f}",
                            f"{tf_val:.2f}"
                        ])
                else:
                    length = len(self.imu_time_data)
                    for i in range(length):
                        a_val = self.imu_angle_data[i] if hasattr(self, 'imu_angle_data') and i < len(self.imu_angle_data) else 0.0
                        tf_val = self.imu_target_data[i] if i < len(self.imu_target_data) else 0.0
                        writer.writerow([
                            f"{self.imu_time_data[i]:.3f}",
                            "0.000",
                            "0.000",
                            f"{a_val:.2f}",
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