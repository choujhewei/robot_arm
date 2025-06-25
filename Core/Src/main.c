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
#include "ROS2STM.h"
#include "ps4_parser.h"
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
_Bool start_trans_mx = 0;

// Geometric parameters
float a = 110;
float b = 160;
//float c = 200;
//float d = 180;
//float e = 300;
//float f = 180;
//int32_t In[6] = {110, 160, 200, 180, 300, 180};
//int32_t An[2] = {100,200};
//int32_t vel[6] = {100};
int32_t vel = 200;
int32_t cmd[7] = {0,200.74/0.088,206.63/0.088,184.13/0.088,159.43/0.088,178.95/0.088,300};
int32_t cmd2[3] = {0,2272,2273};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_ClearFlag_UPDATE(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
  LL_mDelay(1); // ★ 加入延遲穩定方向切換（必要！）
  uart4_dma_tx_start();
  usart6_dma_tx_start();
  LL_USART_EnableIT_RXNE(USART3);
  LL_USART_Enable(USART3);
  LL_USART_EnableIT_RXNE(UART5);
  LL_USART_Enable(UART5);
  NVIC_SetPriority(USART3_IRQn, 0);
  NVIC_EnableIRQ(USART3_IRQn);
  NVIC_SetPriority(UART5_IRQn, 0);
  NVIC_EnableIRQ(UART5_IRQn);
  printf("start\r\n");
  LL_mDelay(100);
  UART4_DMA_Config();
  USART6_DMA_Config();
  LL_USART_Enable(UART4);
  LL_USART_Enable(USART6);
  uint8_t ID_list[6] = { 1, 2, 3, 4, 5, 6 };
  SyncWrite_StatusReturnLevel(6, ID_list, 1);
  LL_mDelay(10);
//  PING(1);
  for (int i = 0;i <= 10; i++){
  	  SyncWrite_DisableDynamixels(6, ID_list);
    }

  for(int id = 1; id <= 6; id++) {
  	  TorqueEnable(id,0);
  	  while (dynamixel_Ready != 1);
        LL_mDelay(1);
    }

    for(int id = 1; id <= 6; id++) {
        OperatingMode(id, POSITION);
        while (dynamixel_Ready != 1);
        LL_mDelay(1);
    }

    for(int id = 1; id <= 6; id++) {
  	  TorqueEnable(id,1);
  	  while (dynamixel_Ready != 1);
        LL_mDelay(1);
    }
  SyncWrite_EnableDynamixels(6, ID_list);
  LL_mDelay(1);

	while(1){
		loop_check_uart5(uart5_rx_buffer, &uart5_packet_ready);
		if (data_ready)
		{
			data_ready = 0;
			printf("interrupt\r\n");
			parse_and_control((char *)rx_buffer);  // 呼叫解析與控制函數
		}
//		SyncLED_Enable(2, ID_list);
//		LL_mDelay(100);
//		SyncLED_Disable(2, ID_list);
//		LL_mDelay(100);
//		SyncWrite_VelocityProfile(2, ID_list, 100);
//		SyncWrite_Position(2, ID_list, cmd);
//		LL_mDelay(1000);
//		SyncWrite_VelocityProfile(2, ID_list, 100);
//		SyncWrite_Position(2, ID_list, cmd2);
//		LL_mDelay(1000);
//		SyncWrite_PositionWithVelocityProfile(6, ID_list, cmd, vel);
//		LL_mDelay(1000);
//		SyncWrite_PositionWithVelocityProfile(2, ID_list, cmd2, vel);
//		LL_mDelay(1000);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  }
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
