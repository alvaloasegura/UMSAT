/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include "sensor.h"
#include "C1098.h"
#include "lora.h"
#include "spi.h"
#include "usart.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t TX_Buffer[256] = "1_1234567 2_1234567 3_1234567 4_1234567 5_1234567 6_1234567 7_1234567 8_1234567 9_1234567 10_123456 11_123456 12_123456 13_123456 14_123456 15_123456 16_123456 17_123456 18_123456 19_123456 20_123456 21_123456 22_123456 23_123456 24_123456 25_123456 12345";
uint8_t trama[50];
GPS_t GPS;
char	*str;

sensor_handle sensor;

SX1278_hw_t lora_hw;
SX1278_t lora;

SX1278_hw_t morse_hw;
SX1278_t morse;

c1098_handle camera;

extern uint8_t GPS_flag;
uint8_t rx_flag = 0;
uint8_t camera_flag = 0;
uint8_t telemetry_flag = 1;
uint32_t retardo = 2000;
#define max_size_JPEG 15000
uint8_t 	C1098_buffer[max_size_JPEG];
uint16_t	C1098_image_size;
float tempx=0;
char buffer[20];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#define DMA_RX_BUFFER_SIZE          600
extern uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
extern uint8_t rx_flag;
/* USER CODE END Variables */
/* Definitions for telemetriaTask */
osThreadId_t telemetriaTaskHandle;
const osThreadAttr_t telemetriaTask_attributes = {
  .name = "telemetriaTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 512 * 4
};
/* Definitions for MorseTask */
osThreadId_t MorseTaskHandle;
const osThreadAttr_t MorseTask_attributes = {
  .name = "MorseTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};
/* Definitions for secundaryTask */
osThreadId_t secundaryTaskHandle;
const osThreadAttr_t secundaryTask_attributes = {
  .name = "secundaryTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void scan_I2C(void);
void parse(sensor_handle *sensor);
/* USER CODE END FunctionPrototypes */

void telemetriaTask01(void *argument);
void morseTask02(void *argument);
void secundaryTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of telemetriaTask */
  telemetriaTaskHandle = osThreadNew(telemetriaTask01, NULL, &telemetriaTask_attributes);

  /* creation of MorseTask */
  MorseTaskHandle = osThreadNew(morseTask02, NULL, &MorseTask_attributes);

  /* creation of secundaryTask */
  secundaryTaskHandle = osThreadNew(secundaryTask03, NULL, &secundaryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_telemetriaTask01 */
/**
  * @brief  Function implementing the telemetriaTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_telemetriaTask01 */
void telemetriaTask01(void *argument)
{
  /* USER CODE BEGIN telemetriaTask01 */
	lora_hw.nss.port = SPI1_NSS_GPIO_Port;
	lora_hw.nss.pin = SPI1_NSS_Pin;
	lora_hw.reset.port = SPI1_RST_GPIO_Port;
	lora_hw.reset.pin = SPI1_RST_Pin;
	lora_hw.spi = &hspi1;  
	lora.hw = &lora_hw;
	
	camera.uart_port = &huart2;
	camera.packet_size = 256;
	camera.baudrate = 460800;
	camera.resolution = QVGA;
	CAMERARESULT camera_res;
	
	sensor.i2c_port = &hi2c2;
	/* Lora */
	SX1278_reset(&lora);
	osDelay(500);
	// ch = 48 (439MHz)
	// while( SX1278_BeginLora(&lora, BW_125, 48, SF_8, CR_7, 0x0008, 100, 0, 0x02, 5) == 0)
	while(SX1278_BeginLora(&lora, BW_125, 48, SF_11, CR_6, 0x0008, 100, 0, 0x02, 20) == 0){
		printf("Error Lora\n");
		HAL_GPIO_TogglePin(TX_LED_GPIO_Port, TX_LED_Pin);
		HAL_GPIO_TogglePin(RX_LED_GPIO_Port, RX_LED_Pin);
		osDelay(500);
	}
	printf("Lora Ok\n");
	while(camera_init(&camera) != CAMERA_OK){
		printf("Error Camera\n");
		HAL_GPIO_TogglePin(TX_LED_GPIO_Port, TX_LED_Pin);
		HAL_GPIO_TogglePin(RX_LED_GPIO_Port, RX_LED_Pin);
		osDelay(500);
	}
	printf("Camera Ok\n");
	HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_RESET);
	osDelay(100);
	HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_SET);
	osDelay(100);
	HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_RESET);
	osDelay(100);
	HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_SET);
	
	SX1278_receive(&lora);
	uint8_t cam_buffer[256];
	uint8_t packet_size=250;
	cam_buffer[0]=0x0F;
	osDelay(1000);
	scan_I2C();
	bmpconfig(&sensor);
	TMPconfig(&sensor);
	IMUconfig(&sensor);
	INAbegin(&sensor);
  /* Infinite loop */
  for(;;)
  {
	/************** cámara JPEG **************/
		if (camera_flag == 1)
		{
			while(snap_shot(&camera) != CAMERA_OK);
			camera_res = get_picture(&camera, C1098_buffer, max_size_JPEG, &C1098_image_size);
			
			uint8_t i=0;
			uint8_t packet=C1098_image_size/250;
			while( C1098_image_size > i*250 ) {
				if( (i+1)*250 > C1098_image_size ) packet_size = C1098_image_size - i*250;
				
				cam_buffer[1] = packet--;
				cam_buffer[2] = packet_size;
				memcpy(&cam_buffer[3],C1098_buffer+i*250, packet_size);
				//HAL_UART_Transmit(&huart3,header,3, HAL_MAX_DELAY);
				sx1278_transmit(&lora, cam_buffer, packet_size + 3);
				//HAL_UART_Transmit(&huart3,C1098_buffer+i*250, packet_size,HAL_MAX_DELAY);
				i++;
				osDelay(100);
			}
		packet_size = 250;
		camera_flag = 0;
		}
		
		/************** telemetría **************/
		if (telemetry_flag == 1)
		{
			parse(&sensor);
			sx1278_transmit(&lora, trama, 44);
			
		}
//			sx1278_transmit(&lora, (uint8_t *)"hola1", 5); //envío de telemetría
			
		/* detect valid header */
//		if( ((SX1278_ReadByte(&lora, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_VALIDHEADER_MASK) == RFLR_IRQFLAGS_VALIDHEADER) )
//		{
//			HAL_GPIO_WritePin(TX_LED_GPIO_Port,TX_LED_Pin, GPIO_PIN_RESET);
////////			osDelay(5);
////////			HAL_GPIO_WritePin(TX_LED_GPIO_Port,TX_LED_Pin, GPIO_PIN_SET);
//////			sx1278_clearIRQ(&lora);
//////			printf("CRC header\n");
//////			//osDelay(20000);
//		}
		
		osDelay(retardo);
		//HAL_GPIO_TogglePin(TX_LED_GPIO_Port,TX_LED_Pin);

		
		
  }
  /* USER CODE END telemetriaTask01 */
}

/* USER CODE BEGIN Header_morseTask02 */
/**
* @brief Function implementing the MorseTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_morseTask02 */
void morseTask02(void *argument)
{
  /* USER CODE BEGIN morseTask02 */
	morse_hw.nss.port = SPI2_NSS_GPIO_Port;
	morse_hw.nss.pin = SPI2_NSS_Pin;
	morse_hw.reset.port = SPI2_RST_GPIO_Port;
	morse_hw.reset.pin = SPI2_RST_Pin;
	morse_hw.spi = &hspi2;  
	morse.hw = &morse_hw;
	SX1278_reset(&morse);
	SX1278_BeginMorse(&morse, 5, 19);
  /* Infinite loop */
  for(;;)
  {
		sprintf (buffer, "UMSAT-T%.2f",tempx);
		SX1278_transmitMorse(&morse, buffer, 11);
		//SX1278_transmitMorse(&morse, "UMSAT-1", 7);
//		HAL_GPIO_WritePin(LED1W_GPIO_Port, LED1W_Pin, GPIO_PIN_SET);
//		osDelay(10);
//		HAL_GPIO_WritePin(LED1W_GPIO_Port, LED1W_Pin, GPIO_PIN_RESET);
    osDelay(10000);
  }
  /* USER CODE END morseTask02 */
}

/* USER CODE BEGIN Header_secundaryTask03 */
/**
* @brief Function implementing the secundaryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_secundaryTask03 */
void secundaryTask03(void *argument)
{
  /* USER CODE BEGIN secundaryTask03 */
  /* Infinite loop */
  for(;;)
  {
		//char dest[100];
		//strcpy(dest,"$GNGGA,123519.542,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n");
		//printf("%s\n",dest);
		if(GPS_flag == 1){
		//if(1){
			GPS_Process(&GPS,(uint8_t *) DMA_RX_Buffer);
			

			//printf("LON = %f\n",GPS.GNGGA.LongitudeDecimal);
			//printf("LAT = %f\n",GPS.GNGGA.LatitudeDecimal);
			//printf("%s",DMA_RX_Buffer);
			//HAL_GPIO_WritePin(TX_LED_GPIO_Port,TX_LED_Pin, GPIO_PIN_RESET);
			//osDelay(50);
			//HAL_GPIO_WritePin(TX_LED_GPIO_Port,TX_LED_Pin, GPIO_PIN_SET);
			GPS_flag=0;
		}
		
		/* decodificación de comando */
		
		

		
		
    //osDelay(1000);
  }
  /* USER CODE END secundaryTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void scan_I2C(void)
{
		HAL_StatusTypeDef result;
		printf("Scanning...\n");
		for (int i=1; i<128; i++)
		{
			/*
			 * the HAL wants a left aligned i2c address
			 * &hi2c1 is the handle
			 * (uint16_t)(i<<1) is the i2c address left aligned
			 * retries 2
			 * timeout 2
			 */
			result = HAL_I2C_IsDeviceReady(&hi2c2, (uint8_t)(i<<1), 2, 2);
			if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
			{
				//printf("."); // No ACK received at that address
			}
			if (result == HAL_OK)
			{
				printf("0x%.2X\n", i); // Received an ACK at that address
			}
		}
		printf("END\n");

		
}

void parse(sensor_handle *sensor)
{
		trama[0]= 0x0D;
//		trama[1]= tiempo++;
//		trama[2]= 0x00;
		trama[1]= 5;
		trama[2]= 0x00;
		//presion
		float presion = bmpPress(sensor) * 100;
		  printf("P = %f [PA*100]\n", presion/100);
		uint32_t presion32 = (uint32_t) presion;
		trama[3]= (presion32 & 0xFF);
		trama[4]= (presion32>>8 & 0xFF);
		trama[5]= (presion32>>16 & 0xFF);
		//altura
		float altura = bmpAlt(sensor, 1033.25) * 10;
		// printf("A = %f [m*10]\n", altura/10);
		uint16_t altura16 = (uint16_t) altura;
		trama[6]= (altura16 & 0xFF);
		trama[7]= (altura16>>8 & 0xFF);
		//temperatura
		float temperatura = TMPtemp(sensor) * 10;
		tempx=temperatura/10;
		// printf("T = %f [C*10]\n", temperatura/10);
		uint16_t temp16 = (uint16_t) temperatura;
		trama[8]= (temp16 & 0xFF);
		trama[9]= (temp16>>8 & 0xFF);
		////////////////////////////////////////////////////////
		// printf("T2 = %f\n",bmpTemp(sensor));
		////////////////////////////////////////////////////////
		
//		// voltaje panel
//		trama[10]= 0x01;
//		trama[11]= 0x00;
//		// corriente panel
//		trama[12]= 0x64;
//		trama[13]= 0x00;
//		
//		// voltaje bateria
//		trama[14]= 0x01;
//		trama[15]= 0x00;
//		// corriente bateria
//		trama[16]= 0x64;
//		trama[17]= 0x00;
		
		// voltaje solar
		float VoltajeSol = INA219_BusV(sensor,INA219solar)*100;
		//printf("VSOL = %f [v]\n", VoltajeSol/100);
		  //printf("V= %f\n", VoltajeBat);
		uint16_t Voltaje16 = (uint16_t)VoltajeSol;
		trama[10]= (Voltaje16 & 0xFF);
		trama[11]= (Voltaje16>>8 & 0xFF);
		//corriente solar
		float CorrienteSol = INA219_Current(sensor,INA219solar)*100;
		//printf("CSOL = %f [A]\n", CorrienteSol/100);
		  //printf("C= %f\n\n", CorrienteBat);
		uint16_t current16 = (uint16_t) CorrienteSol;
		trama[12]= (current16 & 0xFF);
		trama[13]= (current16>>8 & 0xFF);
//	
		// voltaje bateria
		float VoltajeBat = INA219_BusV(sensor,INA219cell)*100;
		//printf("VBAT = %f [v]\n", VoltajeBat/100);
		  //printf("V= %f\n", VoltajeBat);
		Voltaje16 = (uint16_t)VoltajeBat;
		trama[14]= (Voltaje16 & 0xFF);
		trama[15]= (Voltaje16>>8 & 0xFF);
		//corriente bateria
		float CorrienteBat = INA219_Current(sensor,INA219cell)*100;
		//printf("CBAT = %f [A]\n", CorrienteBat/100);
		  //printf("C= %f\n\n", CorrienteBat);
		current16 = (uint16_t) CorrienteBat;
		trama[16]= (current16 & 0xFF);
		trama[17]= (current16>>8 & 0xFF);
//	
		if(GPS_flag != 1)
		{
			//GPS lat
			//lat = lat + 0.0001;
			int32_t lat32 = (int32_t) (GPS.GNGGA.LatitudeDecimal*10000000);
			trama[18]= lat32 & 0xFF;
			trama[19]= lat32>>8 & 0xFF;
			trama[20]= lat32>>16 & 0xFF;
			trama[21]= lat32>>24 & 0xFF;
			
			//GPA lon
			//lon = lon + 0.0001;
			int32_t lon32 = (int32_t) (GPS.GNGGA.LongitudeDecimal*10000000);
			trama[22]= lon32 & 0xFF;
			trama[23]= lon32>>8 & 0xFF;

			trama[24]= lon32>>16 & 0xFF;
			trama[25]= lon32>>24 & 0xFF;
		}
		
		readIMU(sensor);
		//MAG XYZ +- 6000 uT
		float magX = IMUread_MagX(sensor);
		float magY = IMUread_MagY(sensor);
		float magZ = IMUread_MagZ(sensor);
		int16_t magX16 = (int16_t)magX;
		int16_t magY16 = (int16_t)magY;
		int16_t magZ16 = (int16_t)magZ;
		trama[26]= (magX16 & 0xFF);		
		trama[27]= (magX16>>8 & 0xFF);
		trama[28]= (magY16 & 0xFF);		
		trama[29]= (magY16>>8 & 0xFF);
		trama[30]= (magZ16 & 0xFF);		
		trama[31]= (magZ16>>8 & 0xFF);
//		
		//ACEL XYZ +- 20.0 m/s/s
		float acelX =IMUread_AcelX(sensor)*10;
		float acelY =IMUread_AcelY(sensor)*10;
		float acelZ =IMUread_AcelZ(sensor)*10;
		//printf("%f,%f,%f\n",acelX,acelY,acelZ);
		 //printf("%f,%f,%f\n",IMUread_AcelX(sensor), IMUread_AcelY(sensor),IMUread_AcelZ(sensor));
		int16_t acelX16 = (int16_t)acelX;
		int16_t acelY16 = (int16_t)acelY;
		int16_t acelZ16 = (int16_t)acelZ;
		trama[32]= (acelX16 & 0xFF);		
		trama[33]= (acelX16>>8 & 0xFF);
		trama[34]= (acelY16 & 0xFF);		
		trama[35]= (acelY16>>8 & 0xFF);
		trama[36]= (acelZ16 & 0xFF);		
		trama[37]= (acelZ16>>8 & 0xFF);
//		
		//GYR XYZ +- 100
		float gyrX = IMUread_GyrX(sensor)*10;
		float gyrY = IMUread_GyrY(sensor)*10;
		float gyrZ = IMUread_GyrZ(sensor)*10;
		int16_t gyrX16 = (int16_t)gyrX;
		int16_t gyrY16 = (int16_t)gyrY;
		int16_t gyrZ16 = (int16_t)gyrZ;
		trama[38]= (gyrX16 & 0xFF);		
		trama[39]= (gyrX16>>8 & 0xFF);
		trama[40]= (gyrY16 & 0xFF);		
		trama[41]= (gyrY16>>8 & 0xFF);
		trama[42]= (gyrZ16 & 0xFF);		
		trama[43]= (gyrZ16>>8 & 0xFF);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
