import sys
import time
import pandas as pd
from collections import deque
from PyQt5.QtCore import Qt, QCoreApplication
from imu_driver import ImuWorker

class DataCollector:
    def __init__(self, window_size=20, step_size=5):
        self.window_size = window_size
        self.step_size = step_size 
        
        self.waist_q = deque(maxlen=window_size)
        self.l_thigh_q = deque(maxlen=window_size)
        self.r_thigh_q = deque(maxlen=window_size)
        
        self.waist_vel_q = deque(maxlen=window_size)
        self.l_thigh_vel_q = deque(maxlen=window_size)
        self.r_thigh_vel_q = deque(maxlen=window_size)
        
        self.dataset = [] 
        
        self.is_recording = False
        self.current_label = None
        self.step_counter = 0

    def start_recording(self, label):
        if self.is_recording and self.current_label == label:
            return

        self.current_label = label
        self.is_recording = True
        self.step_counter = 0
        
        self.waist_q.clear()
        self.l_thigh_q.clear()
        self.r_thigh_q.clear()
        self.waist_vel_q.clear()
        self.l_thigh_vel_q.clear()
        self.r_thigh_vel_q.clear()
        
        print(f"\n🔴 [{label}] 데이터 수집 중... (정지: 0, 종료: q)")

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            self.current_label = None
            print(f"\n⏸ 수집 일시 정지 (현재 누적 데이터: {len(self.dataset)}개)")

    def update_data(self, angles, vels):
        self.waist_q.append(angles[0])
        self.l_thigh_q.append(angles[1])
        self.r_thigh_q.append(angles[2])
        
        self.waist_vel_q.append(vels[0])  
        self.l_thigh_vel_q.append(vels[1])
        self.r_thigh_vel_q.append(vels[2])

        if self.is_recording and len(self.waist_q) == self.window_size:
            self.step_counter += 1
            if self.step_counter >= self.step_size:
                self._save_feature_row()
                self.step_counter = 0

    def _save_feature_row(self):
        row = {
            'label': self.current_label,
            'waist_mean': sum(self.waist_q) / self.window_size,
            'waist_max': max(self.waist_q),
            'waist_min': min(self.waist_q),
            'l_thigh_mean': sum(self.l_thigh_q) / self.window_size,
            'l_thigh_max': max(self.l_thigh_q),
            'r_thigh_mean': sum(self.r_thigh_q) / self.window_size,
            'r_thigh_max': max(self.r_thigh_q),
            'waist_omega_mean': sum(self.waist_vel_q) / self.window_size,
            'waist_omega_max': max(self.waist_vel_q),
            'waist_omega_min': min(self.waist_vel_q),
            'l_thigh_omega_mean': sum(self.l_thigh_vel_q) / self.window_size,
            'l_thigh_omega_max': max(self.l_thigh_vel_q),
            'l_thigh_omega_min': min(self.l_thigh_vel_q),
            'r_thigh_omega_mean': sum(self.r_thigh_vel_q) / self.window_size,
            'r_thigh_omega_max': max(self.r_thigh_vel_q),
            'r_thigh_omega_min': min(self.r_thigh_vel_q),
            
        }
        self.dataset.append(row)

    def export_csv(self, filename="imu_dataset.csv"):
        self.stop_recording()
        df = pd.DataFrame(self.dataset)
        df.to_csv(filename, index=False)
        print(f"\n💾 데이터셋이 {filename}로 최종 저장되었습니다. (총 {len(df)}행)")


if __name__ == '__main__':
    app = QCoreApplication(sys.argv)
    
    worker = ImuWorker()
    collector = DataCollector(window_size=20, step_size=5)
    
    # DirectConnection 덕분에 input()으로 멈춰있어도 백그라운드에서 데이터는 계속 수집됩니다.
    worker.angle_received.connect(collector.update_data, type=Qt.DirectConnection)
    worker.start()
    
    worker.trigger_calibration()

    label_map = {
        '1': 'standing',
        '2': 'stoop lifting',
        '3': 'squat lifting',
        '4': 'walking'
    }
    
    time.sleep(1) 
    
    print("\n" + "="*50)
    print(" 🎯 데이터 수집기 실행 완료")
    print(" [ 1~4 ] + Enter : 해당 동작 수집 시작")
    print(" [  0  ] + Enter : 수집 일시 정지")
    print(" [  c  ] + Enter : 센서 0점 재설정 (Calibration)")
    print(" [  q  ] + Enter : 종료 및 CSV 파일 저장")
    print("="*50 + "\n")

    try:
        while True:
            # 엔터키를 칠 때까지 대기하는 기본 방식으로 원복
            cmd = input().strip().lower() 
            
            if cmd == 'q':
                print("\n[종료] 데이터를 저장하고 프로그램을 종료합니다.")
                collector.export_csv()
                break
            elif cmd == '0':
                collector.stop_recording()
            elif cmd == 'c':
                print("\n🎯 [캘리브레이션] 차렷 자세 0점 재설정을 시작합니다...")
                worker.trigger_calibration()
            elif cmd in label_map:
                target_label = label_map[cmd]
                collector.start_recording(label=target_label)
            
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop()