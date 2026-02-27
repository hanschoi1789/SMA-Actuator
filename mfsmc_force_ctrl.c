#include "main.h"
#include <math.h>

PID_Manager_typedef pid;

// =========================================================
// MFSMC 파라미터 설정
// =========================================================
// LAMBDA: 가열관성을 잡기위한 요소
// 1. 가열 시 (Target > Current):
#define MFSMC_LAMBDA_HEAT   1.0f
// 2. 냉각 시 (Target < Current): 하강 관성에 의해 히터가 켜지는 것을 방지하기 위해 매우 작게 설정
#define MFSMC_LAMBDA_COOL    0.0f

// ALPHA: 시스템 모델 추정치 (입력 민감도)
#define MFSMC_ALPHA   12.0f

// GAIN: 외란 제거 및 추종 강도
#define MFSMC_GAIN  15.0f

// CENTER: 가속이 본격적으로 시작되는 오차 지점 (변곡점)
// 예: 15.0f면 오차 15도까지는 천천히 힘을 올리다가, 15도부터 급격히 가속
#define SATURATION_CENTER    15.0f   

// WIDTH: S자 곡선의 기울기 (작을수록 급격, 클수록 완만)
// 예: 5.0~10.0 정도 추천
#define SATURATION_WIDTH     10.0f

// 강제 냉각 임계값: 현재온도가 목표온도보다 임계값 이상 높으면 출력 0고정
#define MFSMC_FORCED_COOLING_THRESHOLD  1.0f

// 최대 PWM 출력 제한 (0.0 ~ 100.0)
#define MAX_PWM_LIMIT  100.0f
// =========================================================

// MFSMC 알고리즘 구현
float Calculate_Ctrl(PID_Param_TypeDef* pid_param, float current_temp, uint8_t channel)
{
    // 시간차 (dt)
    static uint32_t last_call_time[CTRL_CH] = {0};
    uint32_t current_time = HAL_GetTick();
    if (last_call_time[channel] == 0) {
        last_call_time[channel] = current_time;
        return 0.0f;
    }
    float dt = (current_time - last_call_time[channel]) / 1000.0f; //ms 단위를 s로 변환
    last_call_time[channel] = current_time;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.1f;

    // 오차 계산 (Target - Current)
    float error = pid_param->setpoint - current_temp;

    // 강제 냉각 로직: 현재온도가 목표보다 임계값 이상 높을 때
    if (error < -MFSMC_FORCED_COOLING_THRESHOLD) {
        // last_error는 현재 오차로 갱신
        pid_param->last_error = error;
        // u_old: 모델 추정기 F_hat에서 이전 출력은 0 이었음을 반영
        pid_param->u_old = 0.0f;
        // 강제 0 출력
        return 0.0f;
    }

    // 오차 변화율 (Error Dot)
    float error_dot = (error - pid_param->last_error) / dt;

    // 상태에 따른 gain scheduling
    float alpha = MFSMC_ALPHA;
    float K_gain = MFSMC_GAIN;
    float lambda;

    if (error_dot > 0) { // [온도 하강 중]
        lambda = MFSMC_LAMBDA_COOL;
    } else { // [온도 상승 중]
        lambda = MFSMC_LAMBDA_HEAT;
    }

    // Time Delay Estimation (F_hat 추정: 현재 상태 유지에 필요한 힘)
    // 식: dot(e) = F - alpha * u
    // 이항하면: F = dot(e) + alpha * u
    float u_old = pid_param->u_old;
    float F_hat = error_dot + (alpha * u_old);

    // 슬라이딩 표면 (s) 계산
    float s = error + (lambda * error_dot);

    // [Saturation 로직]
    // shifted tanh 계산
    float abs_s = fabsf(s);
    float sigmoid_raw = (tanhf((abs_s - SATURATION_CENTER) / SATURATION_WIDTH) + 1.0f) / 2.0f;
    // 0점 보정
    float zero_bias = (tanhf((0.0f - SATURATION_CENTER) / SATURATION_WIDTH) + 1.0f) / 2.0f;
    // 정규화 및 스케일링 (bias를 뺸만큼 최대값이 작아지므로 다시 채워줌)
    float sat_val_norm = 0.0f;
    if ((1.0f - zero_bias) > 0.0001f) {
        sat_val_norm = (sigmoid_raw - zero_bias) / (1.0f - zero_bias);
    }
    //최종 sat_val (부호 복구)
    float sat_val = (s >= 0) ? sat_val_norm : -sat_val_norm;

    // MFSMC 제어 입력 계산
    float output = (1.0f / alpha) * ( F_hat + (K_gain * sat_val) );

    // 데이터 갱신
    pid_param->last_error = error;

    // 출력 제한 및 저장
    if (output > pid_param->output_max) output = pid_param->output_max;
    if (output < 0.0f) output = 0.0f;

    pid_param->u_old = output;

    return output;
}

bool Check_Temperature_Rise_Rate(uint8_t channel, float current_temp) {
    const float MIN_RISE_RATE_PER_SEC = 1.0f;
    const uint32_t MAX_LOW_RISE_TIME = 1500;
    const uint8_t MIN_PWM_FOR_MONITORING = 30;

    uint32_t current_time = HAL_GetTick();
    uint8_t current_pwm = system.state_pwm[channel];

    // 부팅중이거나 안전모드인 경우 모니터링 건너뜀
    if (pid.startup_phase || pid.params[channel].safety_mode > 0) return true;

    // PWM이 관찰수준 이상이면 모니터링 시작
    if ((current_pwm >= MIN_PWM_FOR_MONITORING) && !pid.params[channel].rise_rate_monitoring) {
        pid.params[channel].rise_rate_monitoring = true;
        pid.params[channel].last_tracked_temp = current_temp;
        pid.params[channel].last_tracked_time = current_time;
        pid.params[channel].low_rise_time = 0;
        return true;
    }

    if (pid.params[channel].rise_rate_monitoring) {
        uint32_t elapsed = current_time - pid.params[channel].last_tracked_time;
        if (elapsed >= 1000) {
            float temp_change = current_temp - pid.params[channel].last_tracked_temp;
            float rise_rate = temp_change / (elapsed / 1000.0f);

            // 온도상승률이 부족 시간이 임계값 초과 시 안전모드 진입
            if (rise_rate < MIN_RISE_RATE_PER_SEC && current_pwm >= MIN_PWM_FOR_MONITORING) {
                pid.params[channel].low_rise_time += elapsed;
                if (pid.params[channel].low_rise_time >= MAX_LOW_RISE_TIME) {
                    if (pid.params[channel].safety_mode == 0) {
                        printf("CH %u 센서 오류: 온도 상승 부족!\r\n", channel);
                        pid.params[channel].safety_mode = 3;
                        *system.pnt_pwm[channel] = 0;
                        system.state_pwm[channel] = 0;
                        FSW_on(channel);
                        Update_Fan_Status(channel);
                        pid.enable_pid[channel] = 0;
                    }
                    return false;
                }
            } else {
                pid.params[channel].low_rise_time = 0;
            }
            pid.params[channel].last_tracked_temp = current_temp;
            pid.params[channel].last_tracked_time = current_time;
        }
    }

    if (pid.params[channel].rise_rate_monitoring && current_pwm < MIN_PWM_FOR_MONITORING) {
        pid.params[channel].rise_rate_monitoring = false;
    }
    return true;
}

void Update_Fan_Status(uint8_t channel)
{
    bool fan_physical_state = (system.state_fsw[channel] > 0) ? true : false;
    switch (pid.params[channel].safety_mode) {
        case 0: system.state_fsw[channel] = fan_physical_state ? FAN_ON : FAN_OFF; break;
        case 1: system.state_fsw[channel] = FAN_SAFETY_LEVEL1; break;
        case 3: system.state_fsw[channel] = FAN_SENSOR_ERROR; break;
    }
    if (pid.params[channel].safety_mode > 0) FSW_on(channel);
}

bool Check_Temperature_Sensor(uint8_t channel, float current_temp) //센서오류 판단 함수
{
    if (pid.startup_phase) return true;
    if (current_temp > 200.0f || current_temp < -10.0f) {
        if (pid.params[channel].safety_mode != 3) {
            pid.params[channel].safety_mode = 3;
            *system.pnt_pwm[channel] = 0;
            system.state_pwm[channel] = 0;
            FSW_on(channel);
            Update_Fan_Status(channel);
            pid.enable_pid[channel] = 0;
        }
        return false;
    } else if (pid.params[channel].safety_mode == 3) {
        pid.params[channel].safety_mode = 0;
        Update_Fan_Status(channel);
    }
    return true;
}

bool Check_Safety_Temperature(uint8_t channel, float current_temp) //안전온도 판단 함수
{
    if (current_temp >= SAFETY_LIMIT_TEMP) // 안전온도 초과
    {
        if (pid.params[channel].safety_mode == 0) pid.params[channel].safety_mode = 1;
        FSW_on(channel);
        Update_Fan_Status(channel);
        if (pid.enable_pid[channel]) pid.params[channel].setpoint = SAFETY_TARGET_TEMP;
        else {
            *system.pnt_pwm[channel] = 0;
            system.state_pwm[channel] = 0;
        }
        return true;
    }
    else if (pid.params[channel].safety_mode == 1 && current_temp <= SAFETY_TARGET_TEMP)
    {
        pid.params[channel].safety_mode = 0;
        Update_Fan_Status(channel);
        return false;
    }
    return (pid.params[channel].safety_mode > 0);
}

// 인자가 target_temp에서 pc_fan_cmd(PC 명령)로 변경되었습니다.
void Control_Fan_By_Temperature(uint8_t channel, float current_temp, uint8_t pc_fan_cmd)
{
    // 이미 1단계 안전모드(80도 초과)면 팬이 풀가동 중이므로 리턴
    if (pid.params[channel].safety_mode > 0) return; 

    // 하드웨어 보호를 위한 자체 방열 시작 온도 (예: 70도)
    const float AUTO_COOLING_TEMP = 70.0f; 

    // 조건: PC에서 팬을 켜라고 명령했거나(1) OR 현재 온도가 자체 쿨링 온도를 넘었을 때
    if (pc_fan_cmd == 1 || current_temp >= AUTO_COOLING_TEMP) {
        if (system.state_fsw[channel] == FAN_OFF) {
            FSW_on(channel);
            system.state_fsw[channel] = FAN_ON;
            Update_Fan_Status(channel);
        }
    } 
    // 온도가 안전하고 PC에서도 팬을 끄라고 했을 때
    else {
        if (system.state_fsw[channel] == FAN_ON) {
            FSW_off(channel);
            system.state_fsw[channel] = FAN_OFF;
            Update_Fan_Status(channel);
        }
    }
}

void Set_PWM_Output(uint8_t channel, uint8_t duty_cycle)
{
    if (duty_cycle > 100) duty_cycle = 100;
    if (pid.params[channel].safety_mode) duty_cycle = 0;
    *system.pnt_pwm[channel] = duty_cycle;
    system.state_pwm[channel] = duty_cycle;
}

void Init_PID_Controllers(void)
{
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        pid.params[i].lambda = MFSMC_LAMBDA_HEAT;
        pid.params[i].alpha = MFSMC_ALPHA;
        pid.params[i].gain = MFSMC_GAIN;
        pid.params[i].setpoint = 50.0f;
        pid.params[i].u_old = 0.0f;
        pid.params[i].last_error = 0.0f;
        pid.params[i].output_min = 0.0f;
        pid.params[i].output_max = MAX_PWM_LIMIT;
        pid.params[i].max_temp = 80.0f;
        pid.params[i].safety_mode = 0;
        pid.params[i].rise_rate_monitoring = false;
        pid.params[i].last_tracked_temp = 0.0f;
        pid.params[i].last_tracked_time = 0;
        pid.params[i].low_rise_time = 0;
        pid.enable_pid[i] = 0;
    }
    pid.shared_data.new_temp_data = 0;
    pid.shared_data.temp_timestamp = 0;
    for (uint8_t i = 0; i < CTRL_CH; i++) pid.shared_data.temp_data[i] = 0.0f;
}
