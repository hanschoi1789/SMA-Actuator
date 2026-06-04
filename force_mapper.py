# force_mapper.py
import math

class ForceMapper:
    def __init__(self, v_lift=-120.0, v_lower=120.0, lowering_ratio=0.33, max_force=100.0):
        self.v_lift = v_lift
        self.v_lower = v_lower
        self.lowering_ratio = lowering_ratio
        self.max_force = max_force
        
        self.past_lift_type='stoop lifting'
        self.filtered_target_force = 0.0
        
        # 튜닝 파라미터
        self.decay_step = 5.0    # 리프팅 종료 후 감쇠 속도 (g/10ms)

    def get_target_force(self, current_angle, current_velocity=0.0, lift_type='stoop lifting'):
        # 1. 각도 클램핑 (0도 ~ 50도 범위 제한)
        clamped_angle = max(0.0, min(50.0, current_angle))
                
        # 2. 기본 보조력 (F_lift) 계산 - Sine 곡선 매핑
        F_lift = self.max_force * math.sin(math.radians(clamped_angle)*9/5)
        final_force = 0.0
        # ==============================================================
        # 3. 자세(Posture)에 따른 타겟 보조력 계산 분기
        # ==============================================================
        #estimating일 경우 이전의 lift_type유지
        if lift_type=='estimating': 
            lift_type=self.past_lift_type
            
        if lift_type == 'squat lifting':
            # [Squat Lifting 모드]
            if current_velocity > 0:
                # 허리를 굽히는 과정 (하강) -> 게인 0.2 고정
                final_force = F_lift * 0.2
            else:
                # 허리가 펴지는 과정 (상승) -> 각속도 0 ~ -50 범위에 대해 게인 0.2 ~ 0.5 선형 매핑
                # 각속도가 음수이므로, -50을 넘어가면(-50 이하) 0.5로 고정되도록 클램핑
                vel_clamped = max(-50.0, min(0.0, current_velocity))
                
                # 선형 보간: 0일 때 0.2, -50일 때 0.5
                gain = 0.2 + (0.5 - 0.2) * (vel_clamped / -50.0)
                final_force = F_lift * gain
            self.past_lift_type='squat lifting'
                
        elif lift_type=='stoop lifting':
            # [기본: Stoop Lifting 모드]
            F_lower = F_lift * self.lowering_ratio
            
            if current_velocity <= self.v_lift:
                final_force = F_lift
            elif current_velocity >= self.v_lower:
                final_force = F_lower
            else:
                # Transition 위상 (Quadratic Interpolation)
                x = (self.v_lower - current_velocity) / (self.v_lower - self.v_lift)
                final_force = (F_lift - F_lower) * (x**2) + F_lower
            self.past_lift_type='stoop lifting'

        # ==============================================================
        # 4. 시스템 보호를 위한 최종 힘 클램핑
        # ==============================================================
        final_force = max(0.0, min(final_force, self.max_force))
                
        return final_force
    
   