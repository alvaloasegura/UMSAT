/*  To enable circular buffer, you have to enable IDLE LINE DETECTION interrupt 

__HAL_UART_ENABLE_IT (UART_HandleTypeDef *huart, UART_IT_IDLE);   // enable idle line interrupt
__HAL_DMA_ENABLE_IT (DMA_HandleTypeDef *hdma, DMA_IT_TC);  // enable DMA Tx cplt interrupt

also enable RECEIVE DMA

HAL_UART_Receive_DMA (UART_HandleTypeDef *huart, DMA_RX_Buffer, 64);


IF you want to transmit the received data uncomment lines 81 and 90


*/

#include "DMA_iddle.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

#define DMA_RX_BUFFER_SIZE          600
extern uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define UART_BUFFER_SIZE            256
extern uint8_t UART_Buffer[UART_BUFFER_SIZE];
extern uint8_t cont;
extern uint8_t GPS_flag;
size_t Write;
size_t len, tocopy;
uint8_t* ptr;

extern uint8_t check_value;

uint8_t check_for (char *string)
{
	uint16_t so_far = 0;
	for (uint16_t i=0; i<DMA_RX_BUFFER_SIZE; i++)
	{
		if (DMA_RX_Buffer[i] == string[so_far])
		{
			so_far++;
			if (string[so_far] == 0)
				return so_far;
			
		}
  }
}

void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
	if (huart->Instance->SR & UART_FLAG_IDLE)           /* if Idle flag is set */
	{
		volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */
        tmp = huart->Instance->SR;                       /* Read status register */
        tmp = huart->Instance->DR;                       /* Read data register */
    __HAL_DMA_DISABLE (hdma);       /* Disabling DMA will force transfer complete interrupt if enabled */
		
		DMA_IrqHandler (hdma, huart);
	}
}

void DMA_IrqHandler (DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart)
{
	typedef struct
	{
		__IO uint32_t ISR;   /*!< DMA interrupt status register */
		__IO uint32_t Reserved0;
		__IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
	} DMA_Base_Registers;

	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->DmaBaseAddress;
	
	if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != RESET)   // if the source is TC
	{
		//cont++;
		/* Clear the transfer complete flag */
      //regs->IFCR = DMA_FLAG_ <StreamIndex;
		//regs->IFCR = DMA_FLAG_TCIF0_4 << hdma->ChannelIndex;
		//__HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_TC5);
		
		__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		//__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		//__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
	  
	     /* Get the length of the data */
	  len = DMA_RX_BUFFER_SIZE - hdma->Instance->CNDTR;  
	  
	  /* Get number of bytes we can copy to the end of buffer */
	  tocopy = UART_BUFFER_SIZE - Write;      
	  
	  /* Check how many bytes to copy */
    if (tocopy > len) 
		{
      tocopy = len;
    }
		
		 /* Write received data for UART main buffer for manipulation later */
        ptr = DMA_RX_Buffer;
        //memcpy(&UART_Buffer[Write], ptr, tocopy);   /* Copy first part */
				//memset(&DMA_RX_Buffer, 0, sizeof(DMA_RX_Buffer)); 
				//memcpy(&UART_Buffer[0], ptr, len); 		
		/* Correct values for remaining data */
        Write += tocopy;
        len -= tocopy;
        ptr += tocopy;
		
		/* UNCOMMENT BELOW TO transmit the data via uart */

//		HAL_UART_Transmit(&huart1, &UART_Buffer[Write-tocopy], tocopy, 10);
		
		check_value = check_for ("pinku");
		
		/* If still data to write for beginning of buffer */
        if (len) 
				{
            //memcpy(&UART_Buffer[0], ptr, len);      /* Don't care if we override Read pointer now */
            Write = len;
			
						/* UNCOMMENT BELOW TO transmit the data via uart */
//						HAL_UART_Transmit(&huart1, UART_Buffer, len, 10);  // transmit the remaining data
        }
		
		/* Prepare DMA for next transfer */
        /* Important! DMA stream won't start if all flags are not cleared first */
 
        //regs->IFCR = 0x3FU << hdma->ChannelIndex; // clear all interrupts
				hdma->DmaBaseAddress->IFCR = 0x0FFFFFFF; 
				hdma->Instance->CMAR = (uint32_t)DMA_RX_Buffer;   /* Set memory address for DMA again */
        hdma->Instance->CNDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
        hdma->Instance->CCR |= DMA_CCR_EN;            /* Start DMA transfer */
	}
}	
