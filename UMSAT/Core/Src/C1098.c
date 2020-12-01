#include "C1098.h"
#include "main.h"

#include <string.h>

//#include <Lora.h>
//#include "FreeRTOS.h"
//#include "task.h"

//#include "cmsis_os.h"


//extern uint8_t C1098buffer[300];

/**
  * @brief  Retardo milisegundos
  */
void Delay(uint32_t miliseconds){
	HAL_Delay(miliseconds);
}
	
/**
  * @brief  Cambia velocidad puerto UART
  */
HAL_StatusTypeDef Change_UART_BaudRate(UART_HandleTypeDef *huart ,uint32_t baudrate){
	huart->Init.BaudRate = baudrate;
	return HAL_UART_Init(huart);
}

/**
  * @brief  Envia sincronizacion inicial (interno)
  */
CAMERARESULT sync(c1098_handle *handle){
	uint8_t sync[6] ={0xAA,0x0D,0x00,0x00,0x00,0x00};
	uint8_t ack[6] = {0xAA,0x0E,0x00,0x00,0x00,0x00};
	uint8_t c_data[12];

	HAL_UART_Transmit( handle->uart_port, sync, 6, 6 );
	HAL_UART_Receive( handle->uart_port, c_data, 12, 100); 
//	while(1){
//	if(HAL_UART_Receive( handle->uart_port, c_data, 12, 2000) == HAL_OK)
//	{	for(int i=0; i<12; i++)
//	printf("0x%.2X ",c_data[i]);}
//	}
	if (c_data[0]==0xAA && c_data[1]==0x0E && c_data[2]==0x0D  && c_data[6]==0xAA && c_data[7]==0x0D && c_data[8]==0x00){
		HAL_UART_Transmit( handle->uart_port, ack, 6, 6 );
		return CAMERA_OK;
	}
	return CAMERA_COM_ERR;
}
/**********************************************************************************************************************************************/

/**
  * @brief  Sincronizacion camara (interno)
  */
CAMERARESULT camera_sync(c1098_handle *handle){
	
	// OPCION 1 (camara en frio)
	// Inicializa UART a 14400 baudios (velocidad por defecto de la camara al encender)
	if (Change_UART_BaudRate(handle->uart_port,14400)!=HAL_OK){
		return CAMERA_FAILED_CHANGE_BAUDRATE;
	}

	// Manda SYNC 5 veces
	for (int i =0 ; i<5 ; i ++ ){
		if (sync(handle) == CAMERA_OK){
			return CAMERA_OK;
		}
	}
	//OPCION 2 (uC luego de un reset y la camara con una configuracion anterior)
	// Cambia velocidad UART del uC a la definida por el usuario
	if (Change_UART_BaudRate(handle->uart_port,handle->baudrate)!=HAL_OK){
		return CAMERA_FAILED_CHANGE_BAUDRATE;
	}
	// SYNC con la nueva velocidad
	if (sync(handle) == CAMERA_OK){
		return CAMERA_OK;
	}

	//Change default baudrate = 14400 bauds
	if (Change_UART_BaudRate(handle->uart_port,14400)!=HAL_OK){
		return CAMERA_FAILED_CHANGE_BAUDRATE;
	}

	
	for (int i =0 ; i<6 ; i ++ ){
		if (sync(handle) == CAMERA_OK){
			return CAMERA_OK;
		}
	}
	return CAMERA_COM_ERR;
}
/**********************************************************************************************************************************************/

/**
  * @brief  Inicializa camara
  */
CAMERARESULT camera_init(c1098_handle *handle){
	uint8_t initial[6] = {0xAA,0x01,0x07,0x07,0x00,0x00}; //AA 01 p1 07 00 p4 (p1: baudrate, p4: resolution)
	uint8_t set_package_size[6] = {0xAA,0x06,0x08,0x00,0x01,0x00}; //AA 06 08 p2 p3 00 (p3: MSB, p2: LSB)

	uint8_t c_data[12];

	if (handle->resolution == QVGA)initial[5]=0x05;
	if (handle->resolution ==  VGA)initial[5]=0x07;

	//set_package_size[3] = (handle->packet_size) ;
	//set_package_size[4] = (handle->packet_size) >>8;

	if (handle->baudrate == 14400)initial[2]=0x07;
	if (handle->baudrate == 28800)initial[2]=0x06;
	if (handle->baudrate == 57600)initial[2]=0x05;
	if (handle->baudrate == 115200)initial[2]=0x04;
	if (handle->baudrate == 230400)initial[2]=0x03;
	if (handle->baudrate == 460800)initial[2]=0x02;

	//sync
	//printf("-entra sync\n");
	if (camera_sync(handle) != CAMERA_OK){
		printf("error sync\n");
		return CAMERA_COM_ERR;
		
	}
	//printf("sale sync\n");
	//Change BaudRate
	HAL_UART_Transmit( handle->uart_port, initial, 6, 6 );
	HAL_UART_Receive(handle->uart_port, c_data, 6, 100);
	//printf("0x%.2x   0x%.2x   0x%.2x\n",c_data[0],c_data[1],c_data[2]);
	//printf("-entra baud\n");
	if (c_data[0]==0xAA && c_data[1]==0x0E && c_data[2]==0x01){

		if (Change_UART_BaudRate(handle->uart_port,handle->baudrate) != HAL_OK){
			return CAMERA_FAILED_CHANGE_BAUDRATE;
		}
	}else{
		//printf("error baud\n");
		return CAMERA_ERR;
	}
	//printf("sale baud\n");
	Delay(60);

	//Change Package Size
	HAL_UART_Transmit(handle->uart_port, set_package_size, 6, 6 );
	HAL_UART_Receive(handle->uart_port, c_data, 6, 100);
	//printf("0x%.2x   0x%.2x   0x%.2x\n",c_data[0],c_data[1],c_data[2]);
	//printf("-entra paq\n");
	//if (c_data[0]==0xAA && c_data[1]==0x0F && c_data[2]==0x06){		//en revision
		if (c_data[0]==0xAA && c_data[1]==0x0E && c_data[2]==0x06){
		//printf("OK\n\n");
		return CAMERA_OK;
	}else{
		//printf("error paq\n\n");
		return CAMERA_ERR;
	}
	
}
//*******************************************************************************************************************
/**
  * @brief  Toma un captura
  */
CAMERARESULT snap_shot(c1098_handle *handle){
	uint8_t c_data[6];
	uint8_t snap_shot[6] = {0xAA,0x05,0x00,0x00,0x00,0x00};
	HAL_UART_Transmit(handle->uart_port, snap_shot, 6, 6 );
	HAL_UART_Receive(handle->uart_port, c_data, 6, 100);
	if (c_data[0]==0xAA && c_data[1]==0x0E && c_data[2]==0x05){
		return CAMERA_OK;
	}else{
		return CAMERA_ERR;
	}
}

//*******************************************************************************************************************
CAMERARESULT get_picture(	c1098_handle *handle,
													uint8_t *buffer,
													uint32_t buffer_size,
													uint16_t *received_size){
	uint8_t get_picture[6] = {0xAA,0x04,0x01,0x00,0x00,0x00};
	uint8_t  c_data[6];
	uint32_t data_length =0;
	uint16_t packet_size =0;
	uint16_t id =0;

	*received_size = 0;

	//Transmit GET PICTURE
	HAL_UART_Transmit(handle->uart_port, get_picture, 6, 6 );

	//Receive ACK
	HAL_UART_Receive(handle->uart_port, c_data, 6, 100);
	if (c_data[0]!=0xAA || c_data[1]!=0x0E || c_data[2]!=0x04){
		return CAMERA_ERR;
	}

	//Receive Data Length
	HAL_UART_Receive(handle->uart_port, c_data, 6, 1000);
	if (c_data[0]!=0xAA || c_data[1]!=0x0A || c_data[2]!=0x01){
		return CAMERA_ERR;
	}

	//Data Length
	data_length |= c_data[5]<<16;
	data_length |= c_data[4]<<8;
	data_length |= c_data[3];

	//printf("data=%d",data_length);
	
	//
	if ( data_length >= buffer_size ){
		return CAMERA_NOT_ENOUGH_CORE;
	}

	//
	
	uint8_t ack[6] = {0xAA,0x0E,0x00,0x00,0x00,0x00};
	while(1){
		//ACK with Package ID
		ack[4]= (uint8_t)  id;
		ack[5]= (uint8_t) (id>>8);
		HAL_UART_Transmit(handle->uart_port, ack, 6, 6 );

		//Image Data Package
		HAL_UART_Receive(handle->uart_port, c_data, 4, 100);	//Packet ID and Data size
		packet_size  = c_data[3]<<8;
		packet_size |= c_data[2];
		HAL_UART_Receive(handle->uart_port, buffer + (handle->packet_size-6)*id, packet_size, packet_size + 100);
		//HAL_UART_Receive(handle->uart_port, C1098buffer+2, packet_size, packet_size + 100);
		
//		for(int j=2; j<packet_size; j++)
//		printf("%c",C1098buffer[j]);
		
		//C1098buffer[0] = 0x0F;

		
		
		//transmit Lora
		
		
		HAL_UART_Receive(handle->uart_port, c_data, 2, 100);	//Checksum
				*received_size += packet_size;

		
		if ( *received_size ==  data_length ){
			break;
		}
		id++;
	}
	//End packet reception
	ack[4] = 0xF0;
	ack[5] = 0xF0;
	HAL_UART_Transmit(handle->uart_port, get_picture, 6, 6 );
	
	return CAMERA_OK;
}

