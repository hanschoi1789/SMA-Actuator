# imu_driver.py 전체 수정 코드
import serial
import math
import time
from PyQt5.QtCore import QThread, pyqtSignal

class ImuWorker(QThread):
    # GUI와의 호환성을 위해 허리(0번) 데이터용 기본 시그널 유지
    angle_received = pyqtSignal(dict, dict)
    error_occurred = pyqtSignal(str)

    def __init__(self, port='/dev/ttyUSB0', baudrate=921600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None

        # 센서 ID 정의
        self.WAIST_ID = 0  
        self.L_THIGH_ID = 1
        self.R_THIGH_ID = 2
        self.sensor_ids = [self.WAIST_ID, self.L_THIGH_ID, self.R_THIGH_ID]
        
        # 각 센서별 개별 데이터 관리를 위한 딕셔너리 구조
        self.is_calibrated = {sid: False for sid in self.sensor_ids}
        self.needs_recalibration = {sid: False for sid in self.sensor_ids} 
        self.calib_samples = {sid: [] for sid in self.sensor_ids}
        
        self.q_init = {sid: [1.0, 0.0, 0.0, 0.0] for sid in self.sensor_ids}
        self.up_local_init = {sid: [0.0, 0.0, 1.0] for sid in self.sensor_ids}
        self.theta_stand = {sid: 0.0 for sid in self.sensor_ids}
        
        self.last_angle = {sid: None for sid in self.sensor_ids}
        self.last_time = {sid: 0.0 for sid in self.sensor_ids}
        self.vel_buffer = {sid: [] for sid in self.sensor_ids}

        # 터미널 모니터링용 현재값 저장소
        self.current_angles = {sid: 0.0 for sid in self.sensor_ids}
        self.current_vels = {sid: 0.0 for sid in self.sensor_ids}

    def trigger_calibration(self):
        # 🌟 GUI에서 호출 시 허리 뿐만 아니라 양쪽 허벅지도 동시에 재캘리브레이션 트리거
        print("\n[IMU Worker] 모든 센서(허리, 왼허벅지, 오른허벅지) Zero Calibration을 시작합니다...")
        for sid in self.sensor_ids:
            self.needs_recalibration[sid] = True
            self.calib_samples[sid].clear()
            self.is_calibrated[sid] = False

    def quat_inv(self, q):
        w, x, y, z = q
        norm_sq = w*w + x*x + y*y + z*z
        if norm_sq == 0: return [1, 0, 0, 0]
        return [w/norm_sq, -x/norm_sq, -y/norm_sq, -z/norm_sq]

    def rotate_vector_by_quat(self, v, q):
        qw, qx, qy, qz = q
        vx, vy, vz = v
        rx = (1 - 2*qy*qy - 2*qz*qz)*vx + (2*qx*qy - 2*qw*qz)*vy + (2*qx*qz + 2*qw*qy)*vz
        ry = (2*qx*qy + 2*qw*qz)*vx + (1 - 2*qx*qx - 2*qz*qz)*vy + (2*qy*qz - 2*qw*qx)*vz
        rz = (2*qx*qz - 2*qw*qy)*vx + (2*qy*qz + 2*qw*qx)*vy + (1 - 2*qx*qx - 2*qy*qy)*vz
        return [rx, ry, rz]

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            print(f"IMU 포트 연결 성공: {self.port}")
        except Exception as e:
            self.error_occurred.emit(f"IMU Serial Error: {e}")
            return

        self.running = True
        data_buffer = ""
        GLOBAL_UP = [0.0, 0.0, 1.0] 

        while self.running:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    raw_bytes = self.ser.read(waiting)
                    data_buffer += raw_bytes.decode('utf-8', errors='ignore')

                    if '\n' in data_buffer:
                        lines = data_buffer.split('\n')
                        data_buffer = lines.pop()

                        for line in lines:
                            line = line.strip()
                            if not line: continue
                            parts = line.split(',')
                            
                            if len(parts) >= 5:
                                identifier = parts[0].split('-')
                                if len(identifier) == 2:
                                    sid = int(identifier[1])
                                    if sid not in self.sensor_ids: continue
                                    
                                    q_z, q_y, q_x, q_w = map(float, parts[1:5])
                                    curr_q = [q_w, q_x, q_y, q_z]

                                    # 1. 캘리브레이션 처리
                                    if not self.is_calibrated[sid] or self.needs_recalibration[sid]:
                                        self.calib_samples[sid].append(curr_q)
                                        if len(self.calib_samples[sid]) < 30:
                                            continue 
                                            
                                        avg_q = [sum(x) / len(self.calib_samples[sid]) for x in zip(*self.calib_samples[sid])]
                                        norm = math.sqrt(sum(x*x for x in avg_q))
                                        self.q_init[sid] = [x/norm for x in avg_q]
                                    
                                        self.up_local_init[sid] = self.rotate_vector_by_quat(GLOBAL_UP, self.quat_inv(self.q_init[sid]))
                                        
                                        # 🌟 센서별 기준축 분기 (허리 vs 허벅지)
                                        if sid == self.WAIST_ID:
                                            self.theta_stand[sid] = math.degrees(math.atan2(self.up_local_init[sid][0], self.up_local_init[sid][2]))
                                        elif sid==self.L_THIGH_ID:
                                            self.theta_stand[sid] = math.degrees(math.atan2(self.up_local_init[sid][0], -self.up_local_init[sid][2]))
                                        elif sid==self.R_THIGH_ID:
                                            self.theta_stand[sid] = math.degrees(math.atan2(self.up_local_init[sid][0], -self.up_local_init[sid][2]))

                                        self.is_calibrated[sid] = True
                                        self.needs_recalibration[sid] = False
                                        self.calib_samples[sid].clear() 
                                        print(f"[Calibration 완료] 센서 {sid} 기준 설정됨.")

                                    # 2. 로컬 벡터 연산
                                    up_local_curr = self.rotate_vector_by_quat(GLOBAL_UP, self.quat_inv(curr_q))

                                    # 3. 360도 각도 계산 (🌟 센서별 분기)
                                    if sid == self.WAIST_ID:
                                        theta_curr = math.degrees(math.atan2(up_local_curr[0], up_local_curr[2]))
                                    elif sid==self.L_THIGH_ID:
                                        theta_curr = math.degrees(math.atan2(up_local_curr[0], -up_local_curr[2]))
                                    elif sid==self.R_THIGH_ID:
                                        theta_curr = math.degrees(math.atan2(up_local_curr[0], -up_local_curr[2]))

                                    # 4. 각도 정규화
                                    angle_diff = theta_curr - self.theta_stand[sid]
                                    if angle_diff > 180: angle_diff -= 360
                                    elif angle_diff < -180: angle_diff += 360                                        
                                    
                                    sensor_angle = angle_diff

                                    # 5. 각속도 계산
                                    curr_time = time.time()
                                    if self.last_angle[sid] is None:
                                        self.last_angle[sid] = sensor_angle
                                        self.last_time[sid] = curr_time
                                        angular_vel = 0.0
                                    else:
                                        dt = curr_time - self.last_time[sid]
                                        raw_vel = (sensor_angle - self.last_angle[sid]) / dt if dt > 0 else 0.0
                                            
                                        self.vel_buffer[sid].append(raw_vel)
                                        if len(self.vel_buffer[sid]) > 5:
                                            self.vel_buffer[sid].pop(0)
                                            
                                        angular_vel = sum(self.vel_buffer[sid]) / len(self.vel_buffer[sid])
                                        self.last_angle[sid] = sensor_angle
                                        self.last_time[sid] = curr_time

                                    # imu_driver.py의 run() 내부 루프 하단 수정

                                    # 내부 버퍼 업데이트 (수정하신 연산 반영된 부분 아래)
                                    self.current_angles[sid] = sensor_angle
                                    self.current_vels[sid] = angular_vel

                        # 🌟 기존 [if sid == self.WAIST_ID:] 부분을 제거하고, 
                        # 모든 센서가 캘리브레이션 완료되었을 때 메인 GUI로 전체 딕셔너리 전송 및 터미널 출력
                        '''if all(self.is_calibrated.values()):
                            self.angle_received.emit(self.current_angles.copy(), self.current_vels.copy())
                            
                            print(f"[IMU 각도] 허리(0): {self.current_angles[0]:6.2f}° | "
                                  f"왼허벅지(1): {self.current_angles[1]:6.2f}° | "
                                  f"오른허벅지(2): {self.current_angles[2]:6.2f}°", end="\r")'''
                                 
                time.sleep(0.001)
            except Exception:
                pass
        
        if self.ser and self.ser.is_open:
            self.ser.close()

    def stop(self):
        self.running = False