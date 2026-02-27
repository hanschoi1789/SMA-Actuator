/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "common_defs.h"
#include "fdcan.h"
#include "max31855.h"
#include "pid.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	FDCAN_TxHeaderTypeDef	tx_header;
	FDCAN_RxHeaderTypeDef	rx_header;
}Databuf_FDCAN_typedef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM 

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define FSW0_Pin GPIO_PIN_15
#define FSW0_GPIO_Port GPIOC
#define FSW1_Pin GPIO_PIN_0
#define FSW1_GPIO_Port GPIOA
#define FSW2_Pin GPIO_PIN_1
#define FSW2_GPIO_Port GPIOA
#define FSW3_Pin GPIO_PIN_2
#define FSW3_GPIO_Port GPIOA
#define FSW4_Pin GPIO_PIN_4
#define FSW4_GPIO_Port GPIOA
#define I2C2_INT_Pin GPIO_PIN_4
#define I2C2_INT_GPIO_Port GPIOC
#define FSW5_Pin GPIO_PIN_2
#define FSW5_GPIO_Port GPIOB
#define TMC_CS0_Pin GPIO_PIN_10
#define TMC_CS0_GPIO_Port GPIOB
#define TMC_CS1_Pin GPIO_PIN_11
#define TMC_CS1_GPIO_Port GPIOB
#define TMC_CS2_Pin GPIO_PIN_12
#define TMC_CS2_GPIO_Port GPIOB
#define TMC_SCK_Pin GPIO_PIN_13
#define TMC_SCK_GPIO_Port GPIOB
#define TMC_MISO_Pin GPIO_PIN_14
#define TMC_MISO_GPIO_Port GPIOB
#define TMC_CS3_Pin GPIO_PIN_15
#define TMC_CS3_GPIO_Port GPIOB
#define TMC_CS4_Pin GPIO_PIN_10
#define TMC_CS4_GPIO_Port GPIOA
#define TMC_CS5_Pin GPIO_PIN_11
#define TMC_CS5_GPIO_Port GPIOA
#define SW_Pin GPIO_PIN_12
#define SW_GPIO_Port GPIOA
#define SW_EXTI_IRQn EXTI15_10_IRQn
#define CAN_FLT_Pin GPIO_PIN_15
#define CAN_FLT_GPIO_Port GPIOA
#define CAN_FLT_EXTI_IRQn EXTI15_10_IRQn
#define UART3_TX_Pin GPIO_PIN_10
#define UART3_TX_GPIO_Port GPIOC
#define UART3_RX_Pin GPIO_PIN_11
#define UART3_RX_GPIO_Port GPIOC
#define CAN_RX_Pin GPIO_PIN_3
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_4
#define CAN_TX_GPIO_Port GPIOB
#define I2C4_INT_Pin GPIO_PIN_5
#define I2C4_INT_GPIO_Port GPIOB
#define I2C1_INT_Pin GPIO_PIN_6
#define I2C1_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//------------------------------------------------------------------------------------------------------------------------
typedef enum {
	SYSTEM_INIT		= 0,
	SYSTEM_READY	= 1,
	SYSTEM_GO		= 2
}System_State_typedef;

//Flash Addresses
#define FLASH_ADDR_PAGE_125			((uint32_t)0x0803E800)
#define FLASH_ADDR_PAGE_126			((uint32_t)0x0803F000)
#define FLASH_ADDR_PAGE_127			((uint32_t)0x0803F800)

typedef struct {
	uint8_t pwm[N_MOTION][CTRL_CH];
	uint8_t t[N_MOTION][N_TIMEBIN];
}param_struct;

#define FLASH_LENGTH				((N_MOTION * (N_TIMEBIN+CTRL_CH))/8)

typedef union {
	param_struct 	param_struc;
	uint64_t 		param_uint64[FLASH_LENGTH];
}param_union;

//------------------------------------------------------------------------------------------------------------------------
#define TX_BYTE_FDCAN				24
#define TX_BYTE_MONITOR				(CTRL_CH*4)
#define TX_BYTE_RESERVED			(TX_BYTE_FDCAN-TX_BYTE_MONITOR)

typedef struct {
	uint8_t pwm[CTRL_CH];
	uint8_t fan[CTRL_CH];
	uint16_t temp[CTRL_CH];
//	uint8_t reserved[TX_BYTE_RESERVED];
}Buf_FDCAN_Tx_typedef;

typedef union {
	uint8_t uint8[TX_BYTE_FDCAN];
	Buf_FDCAN_Tx_typedef struc;
}Buf_FDCAN_Tx_Union_typedef;

//------------------------------------------------------------------------------------------------------------------------
#define RX_BYTE_CTRL_PARAM				(CTRL_CH*5)

typedef struct {
	uint8_t pwm[CTRL_CH];
	uint8_t fan[CTRL_CH];
	uint8_t enable_pid[CTRL_CH];
	uint16_t target_temp[CTRL_CH];
}Ctrl_Param_typedef;

typedef struct {
	System_State_typedef state_level;
	Buf_FDCAN_Tx_Union_typedef buf_fdcan_tx;
	Ctrl_Param_typedef ctrl_param_now;
	Ctrl_Param_typedef ctrl_param_save;
	volatile uint32_t* pnt_pwm[CTRL_CH];
	uint8_t lock_motion;
	uint8_t state_fsw[CTRL_CH];
	uint8_t state_pwm[CTRL_CH];
//	uint8_t flag_first_param;
	uint8_t flag_lock_motion;
	char uart_char[100];
	uint16_t n_rx_motion;
	uint16_t n_rx_motion_limit;
	uint16_t t_count;
	uint16_t t_target;
	uint32_t t_sec;

//	PID_Param_TypeDef pid_controllers[CTRL_CH];  // 각 채널별 PID 제어기
//	float target_temp[CTRL_CH];            // 목표 온도
//	uint8_t pid_enable[CTRL_CH];           // PID 제어 활성화 플래그
//	Shared_Data_TypeDef shared_data;
//    uint32_t last_fan_toggle[CTRL_CH];
//    uint32_t fan_hysteresis_ms[CTRL_CH];
////
//    Buf_FDCAN_PID_Tuning_Union_typedef buf_fdcan_pid_tuning;
//    uint8_t pid_params_updated;   // PID 파라미터 업데이트 플래그
//    uint8_t system_startup_phase;
}System_typedef;

extern System_typedef system;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
