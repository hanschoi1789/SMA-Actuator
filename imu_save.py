# imu_driver.py
import serial
import math
import time
from PyQt5.QtCore import QThread, pyqtSignal

class ImuWorker(QThread):
    angle_received = pyqtSignal(float,float)
    error_occurred = pyqtSignal(str)

    def __init__(self, port='/dev/ttyUSB0', baudrate=921600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None

        self.WAIST_ID = 0  # 허리 센서 1개만 추적
        
        self.is_calibrated = False
        self.needs_recalibration = False 

        # 캘리브레이션용 변수
        self.q_init = [1.0, 0.0, 0.0, 0.0]
        self.up_local_init = [0.0, 0.0, 1.0] 
        
        # 🌟 수집용 리스트 추가
        self.calib_samples = []
        
        # 🌟 누운 상태의 절대 각도를 저장할 변수 추가
        self.theta_flat = 0.0
        
        # 🌟 각속도 계산용 변수 추가
        self.last_angle = None
        self.last_time = 0.0
        self.vel_buffer = [] # 노이즈 제거용 이동평균 버퍼

    def trigger_calibration(self):
        self.needs_recalibration = True    
        self.calib_samples.clear()  # 🌟 버튼 누를 때마다 리스트 초기화

    # --- 쿼터니언 연산 함수 ---
    def quat_inv(self, q):
        return [q[0], -q[1], -q[2], -q[3]]

    def rotate_vector_by_quat(self, v, q):
        w, x, y, z = q
        u = [x, y, z]
        
        # Cross Product 함수
        def cross(a, b):
            return [a[1]*b[2] - a[2]*b[1], 
                    a[2]*b[0] - a[0]*b[2], 
                    a[0]*b[1] - a[1]*b[0]]
        
        t = [2.0 * val for val in cross(u, v)]
        cross_u_t = cross(u, t)
        
        return [v[0] + w*t[0] + cross_u_t[0],
                v[1] + w*t[1] + cross_u_t[1],
                v[2] + w*t[2] + cross_u_t[2]]
        
        

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.error_occurred.emit(f"IMU Serial Error: {e}")
            return

        self.running = True
        data_buffer = ""

        # 절대 수직 방향 (지구의 하늘 방향 벡터, EBIMU 기본 Z-Up 가정)
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

                        # 최신 프레임 1개만 파싱
                        for line in reversed(lines):
                            line = line.strip()
                            if not line: continue
                            parts = line.split(',')
                            
                            if len(parts) >= 5:
                                identifier = parts[0].split('-')
                                if len(identifier) == 2 and int(identifier[1]) == self.WAIST_ID:
                                    q_z, q_y, q_x, q_w = map(float, parts[1:5])
                                    curr_q = [q_w, q_x, q_y, q_z]

                                   # 1. 캘리브레이션 (누운 자세 90도 저장 - 30프레임 평균)
                                    if not self.is_calibrated or self.needs_recalibration:
                                        self.calib_samples.append(curr_q)
                                        
                                        if len(self.calib_samples) < 30:
                                            self.angle_received.emit(90.0) # 수집 중에는 화면에 90도 출력
                                            break 
                                            
                                        # 30개가 모이면 평균 쿼터니언 계산
                                        avg_q = [sum(x) / len(self.calib_samples) for x in zip(*self.calib_samples)]
                                        norm = math.sqrt(sum(x*x for x in avg_q))
                                        self.q_init = [x/norm for x in avg_q]
                                    
                                        # 똑바로 선 상태의 로컬 하늘 벡터
                                        self.up_local_init = self.rotate_vector_by_quat(GLOBAL_UP, self.quat_inv(self.q_init))
                                        
                                        # X축(위아래)과 Z축(앞뒤)을 이용해 360도 회전각(atan2) 추출
                                        self.theta_flat = math.degrees(math.atan2(self.up_local_init[0], -self.up_local_init[1]))

                                        self.is_calibrated = True
                                        self.needs_recalibration = False
                                        self.calib_samples.clear() 

                                    # 2. 현재 자세에서 바라본 로컬 하늘 벡터
                                    up_local_curr = self.rotate_vector_by_quat(GLOBAL_UP, self.quat_inv(curr_q))

                                    # 3. 현재 360도 회전각 추출
                                    theta_curr = math.degrees(math.atan2(up_local_curr[0], -up_local_curr[1]))

                                    # 4. 선 상태(flat)를 0도로 기준 삼아 현재 각도 계산
                                    angle_diff = theta_curr - self.theta_flat

                                    # -180 ~ 180도 범위로 정규화 (360도 경계에서 값이 튀는 현상 원천 차단)
                                    if angle_diff > 180:
                                        angle_diff -= 360
                                    elif angle_diff < -180:
                                        angle_diff += 360                                        
                                    
                                    # 🌟 선 상태에서 0도가 되도록 설정 
                                    torso_angle = angle_diff

                                    # 🌟 각속도(Angular Velocity) 계산 로직 추가
                                    curr_time = time.time()
                                    
                                    if self.last_angle is None:
                                        self.last_angle = torso_angle
                                        self.last_time = curr_time
                                        angular_vel = 0.0
                                    else:
                                        dt = curr_time - self.last_time
                                        if dt > 0:
                                            # (현재각도 - 이전각도) / 걸린 시간 = 초당 각속도(도/초)
                                            raw_vel = (torso_angle - self.last_angle) / dt
                                        else:
                                            raw_vel = 0.0
                                            
                                        # 순간적인 튐 현상(노이즈)을 막기 위해 최근 5개 샘플의 평균을 냅니다
                                        self.vel_buffer.append(raw_vel)
                                        if len(self.vel_buffer) > 5:
                                            self.vel_buffer.pop(0)
                                            
                                        angular_vel = sum(self.vel_buffer) / len(self.vel_buffer)
                                        
                                        self.last_angle = torso_angle
                                        self.last_time = curr_time

                                    # 🌟 각도와 각속도를 함께 메인으로 전송
                                    self.angle_received.emit(torso_angle, angular_vel)
                                    break
                                 
                time.sleep(0.001)
            except Exception:
                pass
        
        if self.ser and self.ser.is_open:
            self.ser.close()

    def stop(self):
        self.running = False