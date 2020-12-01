/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include "Lora.h"
#include "usart.h"
#include "ssd1306.h"
#include "fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern SX1278_t lora;
uint8_t RX_lora[50];
uint8_t leng;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_RX_BUFFER_SIZE          250
extern uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
extern uint8_t TC_flag;
extern uint8_t TC_Ok;
extern uint8_t TC_buffer[10];
extern uint8_t SF;
extern uint8_t BW;
extern uint8_t RX_flag;
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
extern void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

extern void DMA_IrqHandler (DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart);

//extern uint8_t cont;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  * @brief This function handles Prefetch fault, memory access fault.
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
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	if(lora.Modem == MODEM_LORA)
	{
			leng = SX1278_ReadByte(&lora, REG_LR_RXNBBYTES);
			//printf("%d\n\n",leng);
	}
	HAL_GPIO_WritePin(rx_lora_GPIO_Port, rx_lora_Pin, GPIO_PIN_RESET);
	SX1278_WriteByte(&lora, REG_LR_FIFOADDRPTR, SX1278_ReadByte(&lora, REG_LR_FIFORXCURRENTADDR) );
	SX1278_ReadBurst(&lora, REG_LR_FIFO, RX_lora, leng);
	RX_lora[leng] = 0;	// '/0' terminator
	
	if( 1 )
	HAL_UART_Transmit(&huart1, RX_lora, leng, 200);
	HAL_GPIO_WritePin(rx_lora_GPIO_Port, rx_lora_Pin, GPIO_PIN_SET);
	SX1278_receive(&lora);
	if(TC_Ok == 1)
				{
					HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_RESET);
					if(DMA_RX_Buffer[1] == 'T')
					{
						TC_buffer[0] = 0x0D;
						TC_buffer[1] = 'T';
						TC_buffer[2] = DMA_RX_Buffer[2];
						sx1278_transmit(&lora, TC_buffer, 3);
						DMA_RX_Buffer[0]=0;
//						HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_RESET);
//						HAL_Delay(10);
//						HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_SET);
					}
					
					else if(DMA_RX_Buffer[1] == 'C')
					{
						TC_buffer[0] = 0x0D;
						TC_buffer[1] = 'C';
						//TC_buffer[2] = DMA_RX_Buffer[2];
						//SX1278_receive(&lora);
						sx1278_transmit(&lora, TC_buffer, 2);
						DMA_RX_Buffer[0]=0;
//						HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_RESET);
//						HAL_Delay(10);
//						HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_SET);
					}
					else if(DMA_RX_Buffer[1] == 'L')
					{
						TC_buffer[0] = 0x0D;
						TC_buffer[1] = 'L';
						TC_buffer[2] = DMA_RX_Buffer[2]&0x0F;		// BW
						TC_buffer[3] = (DMA_RX_Buffer[3]&0x0F) + 6;		// SF
						TC_buffer[4] = 20;		// POWER
						SF = TC_buffer[3];
						BW = TC_buffer[2];
						//SX1278_receive(&lora);
						sx1278_transmit(&lora, TC_buffer, 5);
						DMA_RX_Buffer[0]=0;
						SX1278_BeginLora(&lora, TC_buffer[2], 48, TC_buffer[3], CR_6, 0x0008, 100, 0, 0x02, 20);
					}
					TC_Ok = 0;
					TC_flag = 0;
					HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_SET);
				}
				RX_flag = 1;
				HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
				
//				SSD1306_GotoXY (10, 30); 
//				SSD1306_Puts ("OK !!", &Font_16x26, SSD1306_COLOR_WHITE); 
//				SSD1306_UpdateScreen(); // update screen
	
	
	
	//printf("RX = %s",RX_lora);
	SX1278_receive(&lora);
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  //HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */
	//cont++;
	//DMA_IrqHandler (&hdma_usart1_rx, &huart1);
	
  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  //HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	USART_IrqHandler (&huart1, &hdma_usart1_rx);
	
	
	
	TC_flag = 1;
	
	
	//cont++;
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
