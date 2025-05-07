/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "mainpp.h"
#include "Mx106v2.h"
#include "Mx106v2_CRC.h"
#include "stm32f4xx_it.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool start_trans_mx = 0;
float right_joint_angle[7] = { 0.0f };
float right_joint_angle_old[7] = { 0.0f };
float left_joint_angle[7] = { 0.0f };
float left_joint_angle_old[7] = { 0.0f };

float right_joint_omega[7] = { 0.0f };
float left_joint_omega[7] = { 0.0f };

//CoM
// Note CoM_x_current[2] is used to control passive foot in DS. Before entering DS, we record CoM in CoM_x_current[2]
// and integral different in every sample to estimated the CoM by the passive foot.
// ���覡�O���F�קK���}�����X�{�~�t�o���������}�P�˪���J�I
//x walking direction
float CoM_x_ref[3] = { 0.0f, 0.0f, 0.0f }; // ��߫e���Vreference {xd,xd_dot}}
float CoM_x_current[2] = { 0.0f, 0.0f }; //[position, velocity] relative to global coordinate
float CoM_x_last = 0.0f;
float CoM_x_shift = 0.0f;

float controller_output_x_L = 0.0f;
float controller_output_x_R = 0.0f;
float DS_output_x_L = 0.0f;
float DS_output_x_R = 0.0f;
float SS_output_x_L = 0.0f;
float SS_output_x_R = 0.0f;
// y lateral of robot
float CoM_y_ref[3] = { 0.0f, 0.0f, 0.0f };  // double support
float CoM_y_current[2] = { 0.0f, 0.0f };  //[position velocity]
float CoM_y_last = 0.0f;       //  only position

float controller_output_y_L = 0.0f;
float controller_output_y_R = 0.0f;
float DS_output_y_L = 0.0f;
float DS_output_y_R = 0.0f;
float SS_output_y_L = 0.0f;
float SS_output_y_R = 0.0f;

float CoM_x_relative_to_right_leg = 0.0f;
float CoM_x_relative_to_left_leg = 0.0f;
float CoM_x_relative_to_right_leg_old = 0.0f;
float CoM_x_relative_to_left_leg_old = 0.0f;

float CoM_y_relative_to_right_leg = 0.0f;
float CoM_y_relative_to_left_leg = 0.0f;
float CoM_y_relative_to_right_leg_old = 0.0f;
float CoM_y_relative_to_left_leg_old = 0.0f;

float double_support_feedforward_x = 0.0f;
float double_support_feedforward_y = 0.0f;
float single_support_feedforward_x = 0.0f;
float single_support_feedforward_y = 0.0f;

float CoM_x_L_integration = 0.0;
float CoM_x_R_integration = 0.0;
float CoM_y_L_integration = 0.0;
float CoM_y_R_integration = 0.0;

//For encoder velocity
float velocity_x = 0.0;  // raw data
float velocity_x_f = 0.0;  // Lowpass
float velocity_y = 0.0;
float velocity_y_f = 0.0f;
//For attitude
float robot_acc_x = 0.0f;
float robot_acc_y = 0.0f;
float robot_roll = 0.0f;
float robot_roll_old = 0.0f;
float robot_pitch = 0.0f;
float robot_pitch_old = 0.0f;
float robot_yaw_init = 0.0f;
float robot_yaw = 0.0f;
float robot_yaw_old = 0.0f;
float robot_yaw_f = 0;
float yaw_angle = 0;
float angular_velocity_roll = 0.0f;
float angular_velocity_roll_f = 0.0f;
float angular_velocity_pitch = 0.0f;
float angular_velocity_pitch_f = 0.0f;

float accel_w[3] = {0.0f,0.0f,0.0f};
//IMU yaw rate
float yaw_rate[2] = {0.0f};
float yaw_rate_hpf[3] = {0.0f};
float yaw_rate_lpf[3] = {0.0f};
//Sole yaw angle
float yaw_left_foot = 0.0f;
float yaw_right_foot = 0.0f;
float yaw_wrt_rf = 0.0f;
float yaw_wrt_lf = 0.0f;

uint8_t CoM_measuring_leg = 0;
uint16_t step_counter = 0;

// Geometric parameters

const float mass_of_robot = 5.86f;
const float mass_without_free_leg = 4.91f;//4.41f //4.91f
const float mass_of_swing_leg = 1.45f;
const float mass_of_upper_body = 3.46f;//2.96f //3.46f

const float CoM_bias_X = 0.015f; //0.011
const float CoM_bias_Y = 0.0f;

float left_fraction = 0;
float right_fraction = 0;

float xL = 0;
float xR = 0;
float yL = 0;
float yR = 0;

float a = 110;
float b = 160;
float c = 200;
float d = 180;
float e = 300;
float f = 180;
int32_t In[6] = {110, 160, 200, 180, 300, 180};
int32_t An[2] = {100,200};
int32_t vel[6] = {100};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Parse_Status_Errors(uint8_t error_byte) {
    if (error_byte & 0x01) printf("Instruction Error\n");
    if (error_byte & 0x02) printf("Overload Error\n");
    if (error_byte & 0x04) printf("Checksum Error\n");
    if (error_byte & 0x08) printf("Range Error\n");
    if (error_byte & 0x10) printf("Overheating Error\n");
    if (error_byte & 0x20) printf("Angle Limit Error\n");
    if (error_byte & 0x40) printf("Input Voltage Error\n");
    if (error_byte & 0x80) printf("Communication Error\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_Init1msTick(180000000);			//Add here to fix MX generated code
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SYSTICK_EnableIT();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_ClearFlag_UPDATE(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);
//  uart4_dma_tx_start();
  usart6_dma_tx_start();
  printf("start\r\n");
  LL_mDelay(100);
  UART4_DMA_Config();
  USART6_DMA_Config();
  uint8_t ID_list[1] = { 1 };
  SyncWrite_StatusReturnLevel(1, ID_list, 1);
//  LL_mDelay(10);
//  PING();
  SyncWrite_DisableDynamixels(1, ID_list);
//
//  for(int id = 3; id < 9; id++) {
	  TorqueEnable(1, 0);
//      LL_mDelay(1);
//      }
//
//  for(int id = 3; id < 9; id++) {
      OperatingMode(1, POSITION);
//      LL_mDelay(1);
//      }
//
//  for(int id = 3; id < 9; id++) {
      TorqueEnable(1, 1);
//      LL_mDelay(1);
//      }
//
  SyncWrite_EnableDynamixels(1, ID_list);

	while(1){
		SyncLED_Enable(1, ID_list);
		LL_mDelay(100);
		SyncLED_Disable(1, ID_list);
		LL_mDelay(100);
//		for(a=110; a>=20;a-=90){
//			PositionWithVelocity(3,a/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(b=160; b<=240;b+=80){
//			PositionWithVelocity(4,b/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(c=190; c<=220;c+=30){
//			PositionWithVelocity(5,c/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(d=180; d<=330;d+=150){
//			PositionWithVelocity(6,d/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(e=250; e<=310;e+=60){
//			PositionWithVelocity(7,e/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(f=180; f>=0;f-=180){
//			PositionWithVelocity(8,f/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(a=20; a<=110;a+=90){
//			PositionWithVelocity(3,a/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(b=240; b>=160;b-=80){
//			PositionWithVelocity(4,b/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(c=220; c>=190;c-=30){
//			PositionWithVelocity(5,c/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(d=330; d>=180;d-=150){
//			PositionWithVelocity(6,d/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(e=310; e>=250;e-=60){
//			PositionWithVelocity(7,e/0.088,100);
//			LL_mDelay(1000);
//				}
//		for(f=0; f<=180;f+=180){
//			PositionWithVelocity(8,f/0.088,100);
//			LL_mDelay(1000);
//				}
//		SyncWrite_PositionWithVelocityProfile(6, ID_list, In[6], vel[6]);

	}
//	PING();
//	SyncRead_Position(2, ID_list);
//	SyncWrite_StatusReturnLevel(2, ID_list, 1);

//	uint8_t ID_list[1] = { 1 };
//	SyncWrite_StatusReturnLevel(1, ID_list, 1);
//	printf("SyncWrite\r\n");
//	LL_mDelay(100);
//	SyncRead_Position(1, ID_list);
//	USART6_DMA_Config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //////////////////////////// Work Flow ////////////////////////////////////////////
	  //Timer1 Interrupt will make start_trans_mx = 1 , then Read all motors' positions
	  //After Read Position is finished , Is_dynamixel_GetData = 1 , and then goto Robot_main
	  ///////////////////////////////////////////////////////////////////////////////////
//	  	if(start_trans_mx == 1) {
////	  		LL_mDelay(1000);
//	  		start_trans_mx = 0;
////	  		printf("start_tran\r\n");
////	  		LL_mDelay(100);
//	  		uint8_t ID_list[2] = { 1,2 };
//	  		SyncRead_Position(2, ID_list); // Transmit Instruction Packet to MX106 through DMA1_stream4
//	  	}
//
//	  	if(Is_dynamixel_GetData == 1) { // changed in DMA1_stream2_IRQHandler after finishing receiving data-packet from MX106
//	  		Is_dynamixel_GetData = 0;
//	  		printf("Is_dynamixel_GetData\r\n");
//	  		main_function();
//	  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_EnableOverDriveMode();
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 180, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(180000000);
  LL_SetSystemCoreClock(180000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for(DataIdx = 0; DataIdx < len; DataIdx++) {
		LL_USART_TransmitData8(USART2, (uint8_t)*ptr++);
		while(LL_USART_IsActiveFlag_TXE(USART2) == RESET)
			;
	}
	return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
