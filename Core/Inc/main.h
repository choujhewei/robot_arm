/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern bool start_trans_mx ;
// UART TX
typedef union {
	int16_t data;
	uint8_t C[2];
// C[0] is lower-byte of current_index, C[1] is higher-byte of current_index
} SplitterTypeDef;

typedef struct {
	float u[3];
// u[0] means u[k], u[1] means u[k-1], ..., and so on.
}SignalTypeDef;

typedef struct {
	// related to CoM
	float hip2[3];
	float hip3[3];
	float knee[3];
	float ankle[3];
	float CoM[3];
	// rotation vector
	float wa1[3];
	float wa2[3];
	float wa3[3];
	float wa4[3];
	float wa5[3];
	float wa6[3];
	// related to world coordinate
	float hip2_w[3];
	float hip3_w[3];
	float knee_w[3];
	float ankle_w[3];
	float CoM_w[3];

	float wa1_w[3];
	float wa2_w[3];
	float wa3_w[3];
	float wa4_w[3];
	float wa5_w[3];
	float wa6_w[3];
	// rotation
	float R_sole[9];
	float Phi_sole[3];

	float T_sole[9];
} LegTypeDef;

typedef enum
{
  ON = 1U,
  OFF = !ON
} START_ROBOT_CONTROL;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */
#define PI 3.1415f

#define DEG2RAD 0.017453f
#define RAD2DEG 57.296f
#define RADs2RPM 9.5493f
#define RPM2RADs 0.10472f
//-----------    For dynamixel converting units    ------------
#define DEG2POS 11.378f       //DEG2POS  11.375     = 4096/360
#define POS2DEG 0.087891f     //POS2DEG  0.087891   = 360/4096
#define RAD2POS 651.90f		   //RAD2POS  651.89     = 4096/(2pi)
#define POS2RAD 0.0015339f    //POS2RAD  0.0015339  = 2pi/4096
#define DEGs2VEL 0.72780f     //DEGs2VEL 0.72780    = 1/360*60/0.229
#define VEL2DEGs 1.3740f      //VEL2DEGs 1.3740     = 0.229*360/60
#define RADs2VEL 41.700f      //RADs2VEL 41.700     = 1/(2pi)*60/0.229
#define VEL2RADs 0.023981f    //VEL2RADs 0.023981   = 0.229*2*pi/60
//--------------------------------------------------------------

// Motor with SEA
#define S_MOTOR_1 0
#define S_MOTOR_2 1
#define S_MOTOR_7 2
#define S_MOTOR_8 3

// Walking Phase
#define BALANCING 1
#define CoM_SHIFT 2
#define R_SUPPORT 3
#define L_SUPPORT 4

// Double Support Phase
#define BEGINNING 1
#define CONNECT_R2L 2
#define CONNECT_L2R 3
#define FINISH 4

// TEST MODE
#define NORMAL_MODE 1
#define RIGHT_STANDING_MODE 2
#define LEFT_STANDING_MODE 3
#define STEPPING_IN_PLACE_MODE 4
#define WALKING_MODE 5
#define TESTING_MODE 6

// Define walking phase period
#define Shift_Start_period  151
#define Shift_Finish_period  151
#define R_support_start_period 17
#define L_support_finish_period 17
#define Swing_leg_period 17

#define R_support_half_period 111
#define R_support_period 401
#define L_support_period 110
#define Shift_L2R_period 6
#define Shift_R2L_period 6
#define Shift_fin_period 100

// The walking phase follows the sequence :
//                            Idle    >    DS_Start   >   SS_Start   >      DS       >     SS        >     DS       >      SS        > ... > DS > SS_Finish > DS_Finish
// Walking Phase :        (BALANCING)     (CoM_SHIFT)    (R_SUPPORT)     (CoM_SHIFT)    (L_SUPPORT)     (CoM_SHIFT)     (R_SUPPORT)
// Double Support Phase :                 (BEGINNING)                   (CONNECT_R2L)                  (CONNECT_L2R)                                           (FINISH)
// First single support flag :  1              1              1              0              0               0              0            ...   0        0            0
// Last single suport flag :    0              0              0              0              0               0              0            ...   0        1            1
//#define Double_Support_Start_period 201
//#define Single_Support_Start_period 201
//#define Double_Support_period 401
//#define Single_Support_period 201
//#define Single_Support_Finish_period 201
//#define Double_Support_Finish_period 201

#define Double_Support_Start_period 200
#define Single_Support_Start_period 100
#define Double_Support_period 50//100
#define Single_Support_period 100
#define Single_Support_Finish_period 100
#define Double_Support_Finish_period 200

// COM measure leg
#define RIGHT_LEG 1
#define LEFT_LEG 2
// Leg mode
#define DOUBLE_SUPPORT 0
#define SINGLE_SUPPORT 1
#define SWING 2

// Sampling time
#define Ts 0.01f  // unit:sec  = 10ms
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
