
#ifndef __C1098_H__
#define __C1098_H__

#include "gpio.h"
#include <stdio.h>
#include "main.h"
//#include <Lora.h>




typedef enum {
		CAMERA_OK = 0,				  
		CAMERA_ERR,			         
		CAMERA_COM_ERR,               
		CAMERA_NOT_ENOUGH_CORE,       
		CAMERA_FAILED_CHANGE_BAUDRATE 
	 }CAMERARESULT;

typedef enum {
		QVGA = 0,
		VGA
	 }CAMERA_RESOLUTION;

typedef struct {
		UART_HandleTypeDef *uart_port;
		uint32_t packet_size;
		uint32_t baudrate;
		CAMERA_RESOLUTION resolution;
	 }c1098_handle;

CAMERARESULT camera_init(c1098_handle *handle);
CAMERARESULT snap_shot(c1098_handle *handle);
CAMERARESULT get_picture(c1098_handle *handle,uint8_t *buffer,uint32_t size , uint16_t *data_size);

 
#endif
