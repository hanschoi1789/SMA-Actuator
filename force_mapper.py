# force_mapper.py
import math

class ForceMapper:
    def __init__(self, v_lift=-120.0, v_lower=120.0, lowering_ratio=0.33, max_force=100.0):
        """
        CSV 파일 없이 순수 수학 모델로 3가지 위상(Lifting, Transition, Lowering) 제어를 수행합니다.
        
        :param v_lift: 상승 위상 진입 각속도 임계값 (음수, 빠르게 일어날 때)
        :param v_lower: 하강 위상 진입 각속도 임계값 (양수, 빠르게 숙일 때)
        :param lowering_ratio: 하강 위상일 때 최대 힘 대비 적용할 힘의 비율 (기본 33%)
        :param max_force: 엑소수트가 낼 수 있는 최대 목표 보조력 (기본 100.0g)
        """
        
        # 🌟 제어 파라미터 변수화
        self.v_lift = v_lift
        self.v_lower = v_lower
        self.lowering_ratio = lowering_ratio
        self.max_force = max_force

    def get_target_force(self, current_angle, current_velocity=0.0):
        # 1. 각도 클램핑 (0도 ~ 50도 범위 제한)
        clamped_angle = max(0.0, min(50.0, current_angle))
                
        # 2. 상승(Lifting) 시 적용될 최대 보조력 (F_lift) 계산 - Sine 곡선 매핑
        # 각도가 50도일 때 max_force가 되도록 계산
        F_lift = self.max_force * math.sin(math.radians(clamped_angle)*9/5)
        
        # 3. 하강(Lowering) 시 적용될 최소 보조력 (F_lower) 계산
        F_lower = F_lift * self.lowering_ratio
        
        # ==============================================================
        # 4. 3가지 위상에 따른 자율 가상 임피던스 제어 (상태 기계)
        # ==============================================================
        if current_velocity <= self.v_lift:
            # [위상 1] 상승 (Lifting): 최대 보조력 발휘
            final_force = F_lift
            
        elif current_velocity >= self.v_lower:
            # [위상 2] 하강 (Lowering): 움직임 방해를 줄이기 위해 힘 감소
            final_force = F_lower
            
        else:
            # [위상 3] 전환 (Transition): 하강과 상승 사이의 2차 함수 보간 (Quadratic Interpolation)
            # 속도(v)를 0(Lowering 쪽) ~ 1(Lifting 쪽) 사이의 값 x로 정규화하강과 상승 사이의 2차 함수
            x = (self.v_lower - current_velocity) / (self.v_lower - self.v_lift)
            
            # 하강 시 미분값이 0이 되도록 부드럽게 전환하는 2차 방정식
            # F(x) = (F_lift - F_lower) * x^2 + F_lower
            final_force = (F_lift - F_lower) * (x**2) + F_lower

        # ==============================================================
        # 5. 시스템 보호를 위한 최종 힘 클램핑
        # ==============================================================
        final_force = min(final_force, self.max_force)
                
        return final_force