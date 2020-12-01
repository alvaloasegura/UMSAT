/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "Lora.h"
#include "DMA_iddle.h"
#include "string.h"
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DMA_RX_BUFFER_SIZE          250
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define UART_BUFFER_SIZE            256
uint8_t UART_Buffer[UART_BUFFER_SIZE];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buffer_tx[10];
uint16_t check_value =0;
uint8_t cont=0;
	
uint8_t TC_flag=0;
uint8_t TC_Ok=0;
uint8_t TC_buffer[10];

uint8_t SF;
uint8_t BW;
uint8_t RX_flag = 1;
char OLED_buffer[10];
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SX1278_hw_t lora_hw;
SX1278_t lora;
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	lora_hw.nss.port = SPI1_NSS_GPIO_Port;
	lora_hw.nss.pin = SPI1_NSS_Pin;
	lora_hw.reset.port = SPI1_RST_GPIO_Port;
	lora_hw.reset.pin = SPI1_RST_Pin;
	lora_hw.spi = &hspi1;  
	lora.hw = &lora_hw;
	
	//HAL_Delay(100);
	//RESET
	SX1278_reset(&lora);
	//HAL_Delay(200);
	HAL_GPIO_TogglePin(rx_lora_GPIO_Port,rx_lora_Pin);
	HAL_GPIO_TogglePin(tx_lora_GPIO_Port,tx_lora_Pin);
	//printf("hola\n");
	// ch = 48 (439MHz)
	// while( SX1278_BeginLora(&lora, BW_125, 48, SF_8, CR_7, 0x0008, 100, 0, 0x02, 5) == 0)
	SF = SF_11;
	BW = BW_125;
	while( SX1278_BeginLora(&lora, BW_125, 48, SF_11, CR_6, 0x0008, 100, 0, 0x02, 20) == 0)
	{
		//printf("Error Lora\n");
		HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
		HAL_Delay(500);
	}
	//printf("Lora Ok\n");
	//HAL_Delay(1000);
	
	uint8_t status = SSD1306_Init (); // initialize the diaply
//	printf("status = 0x%.2X\n",status);
//	printf("OLED Ok\n");
	//HAL_Delay(100);
	SSD1306_Clear();
	//HAL_Delay(5000);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);   // enable idle line interrupt
	__HAL_DMA_ENABLE_IT (&hdma_usart1_rx, DMA_IT_TC);  // enable DMA Tx cplt interrupt
	
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); 	// discable half complete interrupt
//	hdma_usart1_rx.Instance->CCR &= ~DMA_SxCR_HTIE;
	HAL_UART_Receive_DMA (&huart1, DMA_RX_Buffer, 250);
	
	//HAL_Delay(1000);
	SSD1306_GotoXY (20,20); // goto 10, 10 
	SSD1306_Puts ("ETUMSAT", &Font_11x18, SSD1306_COLOR_WHITE); // print Hello 
//	SSD1306_GotoXY (10, 30); 
//	SSD1306_Puts ("OK !!", &Font_16x26, SSD1306_COLOR_WHITE); 
	SSD1306_UpdateScreen(); // update screen
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(rx_lora_GPIO_Port, rx_lora_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(rx_lora_GPIO_Port, rx_lora_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(rx_lora_GPIO_Port, rx_lora_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	SX1278_receive(&lora);
	uint8_t TX_Buffer[256] = "1_1234567 2_1234567 3_1234567 4_1234567 5_1234567 6_1234567 7_1234567 8_1234567 9_1234567 10_123456 11_123456 12_123456 13_123456 14_123456 15_123456 16_123456 17_123456 18_123456 19_123456 20_123456 21_123456 22_123456 23_123456 24_123456 25_123456 12345";
	DMA_RX_Buffer[1]=0x30;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
		if(TC_flag == 1)
			{
				if(DMA_RX_Buffer[0] == '0')
				{
					//printf("TC = %s\n",DMA_RX_Buffer);
					TC_Ok = 1;
					HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
					HAL_Delay(5000);
				}
				if(TC_Ok == 1)
				{
					if(DMA_RX_Buffer[1] == 'T')
					{
						TC_buffer[0] = 0x0D;
						TC_buffer[1] = 'T';
						TC_buffer[2] = DMA_RX_Buffer[2];
						sx1278_transmit(&lora, TX_Buffer, 50);
						//HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
						DMA_RX_Buffer[0]=0;
//						HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_RESET);
//						HAL_Delay(10);
//						HAL_GPIO_WritePin(tx_lora_GPIO_Port, tx_lora_Pin, GPIO_PIN_SET);
					}
					
					if(DMA_RX_Buffer[1] == 'C')
					{
						TC_buffer[0] = 0x0D;
						TC_buffer[1] = 'C';
						//TC_buffer[2] = DMA_RX_Buffer[2];
						sx1278_transmit(&lora, TC_buffer, 2);
						//HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
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
						SF = TC_buffer[3];
						BW = TC_buffer[2];
						//SX1278_receive(&lora);
						sx1278_transmit(&lora, TC_buffer, 4);
						DMA_RX_Buffer[0]=0;
						SX1278_BeginLora(&lora, TC_buffer[2], 48, TC_buffer[3], CR_6, 0x0008, 100, 0, 0x02, 5);
					}
					HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
					TC_Ok = 0;
				}
				TC_flag = 0;
				RX_flag = 1;
				
				//memset(&DMA_RX_Buffer, 0, sizeof(DMA_RX_Buffer));
				
			}
//			DMA_RX_Buffer[0]=0x31;
//			
//			sx1278_transmit(&lora, TX_Buffer, 190);
//			if(DMA_RX_Buffer[1] == 0x39)
//				DMA_RX_Buffer[1] = 0x30;
//			else
//				DMA_RX_Buffer[1]=DMA_RX_Buffer[1]+1;
			
			//HAL_Delay(3000);
			if(RX_flag == 1)
			{
				sprintf (OLED_buffer, "%d ",SF);
				SSD1306_GotoXY (1,1);
				SSD1306_Puts ("SF:", &Font_7x10, SSD1306_COLOR_BLACK);
				SSD1306_Puts (OLED_buffer, &Font_7x10, SSD1306_COLOR_WHITE);
				
				if(BW == 0) sprintf (OLED_buffer, "7.80 KHz ");
				else if(BW == 1) sprintf (OLED_buffer, "10.40 kHz");
				else if(BW == 2) sprintf (OLED_buffer, "15.60 kHz");
				else if(BW == 3) sprintf (OLED_buffer, "20.80 kHz");
				else if(BW == 4) sprintf (OLED_buffer, "31.25 kHz");
				else if(BW == 5) sprintf (OLED_buffer, "41.70 kHz");
				else if(BW == 6) sprintf (OLED_buffer, "62.50 kHz");
				else if(BW == 7) sprintf (OLED_buffer, "125.0 kHz");
				else if(BW == 8) sprintf (OLED_buffer, "250.0 kHz");
				else if(BW == 9) sprintf (OLED_buffer, "500.0 kHz");
				
				SSD1306_GotoXY (42,1);
				SSD1306_Puts ("BW:", &Font_7x10, SSD1306_COLOR_BLACK);
				SSD1306_Puts (OLED_buffer, &Font_7x10, SSD1306_COLOR_WHITE);
				SSD1306_UpdateScreen(); // update screen
				RX_flag = 0;
			
			}

		



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
