import os
import joblib
import numpy as np
from collections import deque, Counter
import xgboost as xg

class PostureEstimator:
    # 상태 정의
    STATE_STANDING = "Standing"
    STATE_WALKING = "Walking"
    STATE_OBSERVING = "Observing"
    STATE_LIFTING = "Lifting"

    # 파라미터
    WINDOW_SIZE = 20
    VERIFY_STEPS = 3
    CONF_THRESHOLD = 0.35

    # 🌟 학습 데이터(imu_dataset.csv)와 정확히 동일한 컬럼 순서
    FEATURE_ORDER = [
        'waist_mean', 'waist_max', 'waist_min',
        'l_thigh_mean', 'l_thigh_max',
        'r_thigh_mean', 'r_thigh_max',
        'waist_omega_mean', 'waist_omega_max', 'waist_omega_min',
        'l_thigh_omega_mean', 'l_thigh_omega_max', 'l_thigh_omega_min',
        'r_thigh_omega_mean', 'r_thigh_omega_max', 'r_thigh_omega_min',
    ]

    def __init__(self, model_path='xgboost_model.json', encoder_path='label_encoder.pkl'):
        abs_model_path = os.path.abspath(model_path)
        abs_encoder_path = os.path.abspath(encoder_path)
        print(f"[DEBUG] CWD: {os.getcwd()}")
        
        # 1. XGBoost 모델 전용 함수로 JSON 로드
        self.model = xgb.XGBClassifier()
        self.model.load_model(abs_model_path)
        
        # 2. 라벨 인코더 로드
        self.encoder = joblib.load(abs_encoder_path)
        
        print(f"[PostureEstimator] ✅ Classes: {list(self.encoder.classes_)}")
        print(f"[PostureEstimator] ✅ Features ({len(self.FEATURE_ORDER)}): OK")

        # ===== 슬라이딩 윈도우 =====
        self.waist_q = deque(maxlen=self.WINDOW_SIZE)
        self.l_thigh_q = deque(maxlen=self.WINDOW_SIZE)
        self.r_thigh_q = deque(maxlen=self.WINDOW_SIZE)
        self.waist_vel_q = deque(maxlen=self.WINDOW_SIZE)
        self.l_thigh_vel_q = deque(maxlen=self.WINDOW_SIZE)
        self.r_thigh_vel_q = deque(maxlen=self.WINDOW_SIZE)

        # ===== 상태 머신 =====
        self.state = self.STATE_STANDING
        self.verification_buffer = deque(maxlen=self.VERIFY_STEPS)
        self.current_posture = "standing"

    def _extract_features(self):
        """data_collector.py의 _save_feature_row()와 100% 동일하게 계산"""
        N = self.WINDOW_SIZE

        feature_dict = {
            'waist_mean': sum(self.waist_q) / N,
            'waist_max': max(self.waist_q),
            'waist_min': min(self.waist_q),
            'l_thigh_mean': sum(self.l_thigh_q) / N,
            'l_thigh_max': max(self.l_thigh_q),
            'r_thigh_mean': sum(self.r_thigh_q) / N,
            'r_thigh_max': max(self.r_thigh_q),
            'waist_omega_mean': sum(self.waist_vel_q) / N,
            'waist_omega_max': max(self.waist_vel_q),
            'waist_omega_min': min(self.waist_vel_q),
            'l_thigh_omega_mean': sum(self.l_thigh_vel_q) / N,
            'l_thigh_omega_max': max(self.l_thigh_vel_q),
            'l_thigh_omega_min': min(self.l_thigh_vel_q),
            'r_thigh_omega_mean': sum(self.r_thigh_vel_q) / N,
            'r_thigh_omega_max': max(self.r_thigh_vel_q),
            'r_thigh_omega_min': min(self.r_thigh_vel_q),
        }

        # 학습 시 컬럼 순서대로 정렬
        values = [feature_dict[name] for name in self.FEATURE_ORDER]
        return np.array(values).reshape(1, -1)

    def update(self, angles, vels):
        """
        angles: [waist, l_thigh, r_thigh]
        vels:   [waist, l_thigh, r_thigh]
        """
        self.waist_q.append(angles[0])
        self.l_thigh_q.append(angles[1])
        self.r_thigh_q.append(angles[2])
        self.waist_vel_q.append(vels[0])
        self.l_thigh_vel_q.append(vels[1])
        self.r_thigh_vel_q.append(vels[2])

        # 윈도우 안 찼으면 기본값 반환
        if len(self.waist_q) < self.WINDOW_SIZE:
            return {
                'state': self.state,
                'posture': self.current_posture,
                'confidence': 0.0,
                'predicted': None,
            }

        # 예측
        features = self._extract_features()
        proba = self.model.predict_proba(features)[0]
        pred_idx = int(np.argmax(proba))
        confidence = float(proba[pred_idx])
        predicted = self.encoder.inverse_transform([pred_idx])[0]

        # 상태 머신 업데이트
        if confidence >= self.CONF_THRESHOLD:
            self.verification_buffer.append(predicted)
            if len(self.verification_buffer) >= self.VERIFY_STEPS:
                most_common = Counter(self.verification_buffer).most_common(1)[0][0]
                self.current_posture = most_common

                if most_common == "stoop lifting":
                    self.state = self.STATE_LIFTING
                elif most_common == "walking":
                    self.state = self.STATE_WALKING
                elif most_common == "standing":
                    self.state = self.STATE_STANDING
                else:  # squat lifting 등
                    self.state = self.STATE_OBSERVING

        return {
            'state': self.state,
            'posture': self.current_posture,
            'confidence': confidence,
            'predicted': predicted,
        }
