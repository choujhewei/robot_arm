/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "Mx106v2.h"
#include "mainpp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_SIZE     16
#define PACKET_LIMIT    3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t tx_data[] = {0xFF, 0xFF, 0x01, 0x03, 0x02, 0x1E, 0x20, 0x00, 0xD5}; // 根據你的 Instruction Packet 自訂
uint8_t rx_buffer[PACKET_SIZE];
uint8_t packet_count = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC2(DMA1)) {
			LL_DMA_ClearFlag_TC2(DMA1);
			printf("stream2 it\r\n");
			readStatusPacket_pos_DMA(dynamixel_position);
//			readStatusPacket_PING(dynamixel_position);
//	        readStatusPacket_pos_vel_DMA(dynamixel_position, dynamixel_velocity);
//	        readStatusPacket_pos_vel_cur_DMA(dynamixel_position, dynamixel_velocity, dynamixel_current);
			LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
			Packet_Return -= 1;
			if(Packet_Return == 0) {
				Is_dynamixel_GetData = 1;
				dynamixel_Ready = 1;
			}
			else {
			    uart4_dma_rx_start();
				LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
			}
		}
  /* USER CODE END DMA1_Stream2_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC4(DMA1) == 1) {
		    printf("DMA TX Complete\r\n");
			LL_DMA_ClearFlag_TC4(DMA1);
			LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
			LL_USART_DisableDMAReq_TX(UART4);
//			uart4_dma_rx_start();
			dynamixel_Ready = 1;
		}
  /* USER CODE END DMA1_Stream4_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC5(DMA1) == 1) {
			LL_DMA_ClearFlag_TC5(DMA1);
			LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);

		}
  /* USER CODE END DMA1_Stream5_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC6(DMA1) == 1) {
			LL_DMA_ClearFlag_TC6(DMA1);

			LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
		}
  /* USER CODE END DMA1_Stream6_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
			LL_TIM_ClearFlag_UPDATE(TIM1);
			start_trans_mx = 1;
		}

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC7(DMA1)) {
		LL_DMA_ClearFlag_TC7(DMA1);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7); //DMA TX�@finished disable stream

	}
  /* USER CODE END DMA1_Stream7_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	printf("uart4 it\r\n");
	LL_mDelay(100);
	if(LL_USART_IsActiveFlag_TC(UART4) == 1) {
		    printf("TX complete, switching to RX mode\r\n");
		    LL_USART_ClearFlag_TC(UART4);
	#if USE_THREE_STATE_GATE == 1
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
	#else
			LL_USART_SetTransferDirection(UART4, LL_USART_DIRECTION_RX); //change UART direction
	#endif

				if(Packet_Return == 0) {
					dynamixel_Ready = 1;
					return;
				}
//				else if(Packet_Return == 1) {
//					dynamixel_Ready = 2;
//					return;
//				}
//				else {
//					printf("Packet_Return \r\n");
//					dynamixel_Ready = 2;
//
//					LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, Status_packet_length + 4);
//					LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
//					LL_USART_EnableDMAReq_RX(UART4);
//				}
			}
  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC6(DMA2) == 1) {
			LL_DMA_ClearFlag_TC6(DMA2);

			LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_6);
		}
  /* USER CODE END DMA2_Stream6_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void uart4_dma_tx_start(void)
{
    printf("DMA started\r\n");
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_4));

    LL_DMA_ClearFlag_TC4(DMA1);
    LL_DMA_ClearFlag_TE4(DMA1);
    LL_DMA_ClearFlag_HT4(DMA1);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)tx_data);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)&UART4->DR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, sizeof(tx_data));

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
    LL_USART_EnableDMAReq_TX(UART4);
}
void uart4_dma_rx_start(void)
{
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_2));

    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_ClearFlag_TE2(DMA1);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_2, (uint32_t)Status_Packet_Array);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(UART4));
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, Status_packet_length + 4);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_2);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
    LL_USART_EnableDMAReq_RX(UART4);
}
/* USER CODE END 1 */
