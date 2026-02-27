#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stdbool.h>
#include "common_defs.h"

// Fan 상태값 정의
#define FAN_OFF              0   // 팬 꺼짐 (정상 모드)
#define FAN_ON               1   // 팬 켜짐 (정상 모드)
#define FAN_SAFETY_LEVEL1    17  // 1단계 안전 모드 (80°C 초과)
#define FAN_SENSOR_ERROR     49  // 센서 오류 모드 (200°C 초과 또는 -50°C 미만)
#define SAFETY_LIMIT_TEMP	 80.0f  // 1단계 안전 모드 복구를 위한 히스테리시스(°C)
#define SAFETY_TARGET_TEMP	 70.0f  // 1단계 안전 모드 복구를 위한 히스테리시스(°C)
// Fan ON/OFF 제어 임계값
#define TEMP_HIGH_THRESHOLD 8.0f
#define TEMP_LOW_THRESHOLD 5.0f

typedef struct {
    float lambda;
    float alpha;
    float gain;
    float setpoint;
    float u_old;
    float last_error;
    float output_min;
    float output_max;
    float max_temp;
    float critical_temp;
    float sensor_error_temp;
    uint8_t safety_mode;
    uint8_t recovery_needed;
    float last_tracked_temp;     // 온도 상승률 추적용 이전 온도
    uint32_t last_tracked_time;  // 온도 추적 마지막 시간 (ms)
    uint32_t low_rise_time;      // 낮은 온도 상승률 지속 시간 (ms)
    bool rise_rate_monitoring;   // 온도 상승률 모니터링 활성화 상태
} PID_Param_TypeDef;

typedef struct {
    volatile uint8_t new_temp_data;     // 새로운 온도 데이터 플래그
    volatile float temp_data[CTRL_CH];  // 온도 데이터 저장
    volatile uint32_t temp_timestamp;   // 온도 데이터 타임스탬프
} Shared_Data_TypeDef;

#define RX_BYTE_PID_TUNING        13  //
typedef struct {
    float kp;                 // 비례 게인
    float ki;                 // 적분 게인
    float kd;                 // 미분 게인
    uint8_t channel;          // 대상 채널 (0-5)
    //uint8_t save_to_flash;    // 플래시에 저장할지 여부
} Buf_FDCAN_PID_Tuning_typedef;

typedef union {
    uint8_t uint8[RX_BYTE_PID_TUNING];
    Buf_FDCAN_PID_Tuning_typedef struc;
} Buf_FDCAN_PID_Tuning_Union_typedef;

typedef struct {
	PID_Param_TypeDef params[CTRL_CH];  // 각 채널별 PID 제어기
	float target_temp[CTRL_CH];            // 목표 온도
	uint8_t enable_pid[CTRL_CH];           // PID 제어 활성화 플래그
	Shared_Data_TypeDef shared_data;
    uint32_t last_fan_toggle[CTRL_CH];
    uint32_t fan_hysteresis_ms[CTRL_CH];

    Buf_FDCAN_PID_Tuning_Union_typedef buf_fdcan_pid_tuning;
    uint8_t params_updated;   // PID 파라미터 업데이트 플래그
    uint8_t startup_phase;
}PID_Manager_typedef;

extern PID_Manager_typedef pid;

float Calculate_Ctrl(PID_Param_TypeDef* pid, float current_temp, uint8_t channel);
void Update_Fan_Status(uint8_t channel);

bool Check_Temperature_Sensor(uint8_t channel, float current_temp);
bool Check_Safety_Temperature(uint8_t channel, float current_temp);
void Control_Fan_By_Temperature(uint8_t channel, float current_temp, float target_temp);

void Set_PWM_Output(uint8_t channel, uint8_t duty_cycle);

void Init_PID_Controllers(void);
bool Check_Temperature_Rise_Rate(uint8_t channel, float current_temp);

void PID_Set_Target_Temp(uint8_t channel, float target_temp);

#endif /* __PID_H */
