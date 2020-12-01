

#ifndef __Lora_H__
#define __Lora_H__

#include <stdint.h>
#include <stdbool.h>
//#include "main.h"
//#include "spi.h"
//#include "gpio.h"
#include "main.h"
#include "stdio.h"

#include "sx1278Regs-LoRa.h"
#include "sx1278Regs-Fsk.h"

#define	BW_7_8									0x00
#define BW_10_4									0x01
#define BW_15_6									0x02
#define BW_20_8									0x03
#define BW_31_25								0x04
#define BW_41_7									0x05
#define BW_62_5									0x06
#define BW_125									0x07
#define BW_250									0x08
#define BW_500									0x09

#define	FREQ_433								0x00
#define	FREQ_433_5							0x04

#define	SF_6										0x06
#define	SF_7										0x07
#define	SF_8										0x08
#define	SF_9										0x09
#define	SF_10										0x0A
#define	SF_11										0x0B
#define	SF_12										0x0C

#define	CR_5										0x01
#define	CR_6										0x02
#define	CR_7										0x03
#define	CR_8										0x04

#define	PREAMBLE_8							0x0008

#define	HEAD_IMPL								1
#define	HEAD_EXPL								0

#define	GAIN_G1									0x01
#define	GAIN_G2									0x02
#define	GAIN_G3									0x03
#define	GAIN_G4									0x04
#define	GAIN_G5									0x05
#define	GAIN_G6									0x06

//int aux;

typedef struct{
  char c;                                             // ASCII character
  char m[7];                                          // Morse code representation
}Morse_t;





typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}SX1278_RadioModems_t;



typedef struct{
		int8_t   Power;


}SX1278_FskSettings;

typedef struct{
    int8_t   Power;
    uint8_t  BW;
    uint8_t  SF;
    //bool     LowDatarateOptimize;
    uint8_t  CR;
    uint16_t PreambleLen;
    bool     CrcOn;
    //bool     FreqHopOn;
		uint8_t	 LNAgain;


}SX1278_LoraSettings;


typedef struct{
		SX1278_FskSettings Settings;
		//Futuras expansiones
}SX1278_Fsk;


typedef struct{
		SX1278_LoraSettings Settings;
		//Futuras expansiones
}SX1278_Lora;

typedef struct {
		int pin;
		void * port;
} SX1278_hw_gpio_t;

typedef struct {
		SX1278_hw_gpio_t reset;
		SX1278_hw_gpio_t nss;
		SX1278_hw_gpio_t led;
		void * spi;
} SX1278_hw_t;

typedef	struct{
		SX1278_hw_t *hw;
		SX1278_Lora Lora;
		SX1278_Fsk 	Fsk;
	  SX1278_RadioModems_t Modem;
		uint8_t State;
} SX1278_t;


 void gpio_write(SX1278_hw_gpio_t * gpio, int value);
 void SX1278_ReadBurst(SX1278_t * sw, uint8_t addr, uint8_t *buffer, uint8_t size);
 void SX1278_WriteBurst(SX1278_t * sw, uint8_t addr, uint8_t *buffer, uint8_t size);
 uint8_t SX1278_ReadByte(SX1278_t * sw, uint8_t addr);
 void SX1278_WriteByte(SX1278_t * sw, uint8_t addr, uint8_t byte);
 void SX1278_SetModem(SX1278_t * sw, SX1278_RadioModems_t modem );
 void	SX1278_SetOpMode(SX1278_t * sw, uint8_t mode);

//uint8_t bandwidth: [0x1D]
//	0x00 [7.80 KHz]
//	0x01 [10.40 kHz]
//	0x02 [15.60 kHz]
//	0x03 [20.80 kHz]
//	0x04 [31.25 kHz]
//	0x05 [41.70 kHz]
//	0x06 [62.50 kHz]
//	0x07 [125.0 kHz] //Default
//	0x08 [250.0 kHz]
//	0x09 [500.0 kHz]

//uint8_t channel: [0x06, 0x07, 0x08]
//	0x00 [433 MHz]
//	0x01 [433.125 MHz]
//	0x02 [433.250 MHz]
//	0x03 [433.375 MHz]
//	0x04 [433.500 MHz]
//	...

//uint8_t spreadingFactor: [0x1E]
//	0x06 [SF6]
//	0x07 [SF7] //Default
//	0x08 [SF8]
//	0x09 [SF9]
//	0x0A [SF10]
//	0x0B [SF11]
//	0x0C [SF12]

//SX1278_BeginLora(&SX1278, 0x07, 0x00, 0x09, 0x04, 0x0008, 100, 0, 0x02);

//uint8_t codingRate: [0x1D]
//	0x01 [CR 4/5] //Default
//	0x02 [CR 4/6]
//	0x03 [CR 4/7]
//	0x04 [CR 4/8]

//uint16_t preambleLength: [0x20, 0x21]
//	0x0008 [8 symbols + 4.25] //Default

//uint8_t currentLimit: [0x0B]
//	0 	[disable OCP]
//	45-120 	[step 5]
//	130-240 [step 10]

//bool impHeader: [0x1D]
//	0	[Explicit Header mode]
//	1	[Implicit Header mode]

//uint8_t gain: [0x26, 0x0C]
//	0x00	[AGC]
//	0x01	[G1 max gain]
//	0x02	[G2]
//	0x03	[G3]
//	0x04	[G4]
//	0x05	[G5]
//	0x06	[G6 min gain]

 int SX1278_BeginLora(SX1278_t * sw, uint8_t bandwidth , uint8_t channel, uint8_t spreadingFactor, uint8_t codingRate, uint16_t preambleLength, uint8_t currentLimit, bool impHeader, uint8_t gain, uint8_t outputPower);
//Ajusta el límite de corriente
//[IN] CurrentLimit: 0 to 120 mAh (step: 5 mAh)   or   130 to 240 mAh (step: 10 mAh)
//		 Default: 100 mAh
 uint8_t SetCurrentLimit(SX1278_t * sw, uint8_t CurrentLimit);

//Retorna tipo de modo utilizado
//[OUT]: 
//	MODEM_FSK (0)
//	MODEM_LORA (1)
 SX1278_RadioModems_t SX1278_GetModem(SX1278_t * sw);
 
 //Ajustar tamaño del preámbulo FSK o LORA
 //[OUT]: -1 (Error) 1(Ok)
 //				Default: 8 para Lora
 // Ver registros 0x210 y 0x21 para Lora
 // Ver registros 0x25 y 0x26 para Fsk
 uint8_t SX1278_SetPreambleLength(SX1278_t * sw, uint16_t PreambleLength);
 
 //Inicia frecuencia de TX/RX Lora o FSK
 //[IN]: Channel 0 -> n
 //			 433MHz + Channel*125 MHz
 void SX1278_SetChannel(SX1278_t * sw, uint8_t Channel);
 
 //Adjust Power Output on PABOOST always
 //[IN]: 2 to 17 and 20
 void sx1278_SetOutputPower(SX1278_t * sw, uint8_t power);
 
 //Set Gain LNA
 //[IN]: 0 to 6  (0 reserved for AGC On)
 void SX1278_SetLNA(SX1278_t * sw, uint8_t gain);
 
 void sx1278_transmit(SX1278_t * sw, uint8_t * data, size_t len);
 
 void sx1278_startTransmit(SX1278_t *sw, uint8_t * data, uint8_t len);
 
 void sx1278_clearIRQ(SX1278_t * sw);
 
 void SX1278_reset(SX1278_t * sw);
 
 void SX1278_printBeginLora(SX1278_t * sw);
 
 void SX1278_receive(SX1278_t *sw);
 
 void SX1278_startReceive(SX1278_t * sw);
 
 
 
 //Set Bit Rate ej: 300 Kbps (bitRate = 3000)  1.2 Kbps (bitRate = 12)
 void SX1278_setBitRate(SX1278_t * sw, uint16_t bitRate);
 
 //Set Frequency Desviation ej: 200 KHz (freqDev = 200)
 void SX1278_setFreqDev(SX1278_t * sw, uint8_t freqDev);
 
 //Set RX Bandwidth ej: 0 (2.6 KHz)  21 (250 KHz) ver pag. 88
 void SX1278_setRXBW(SX1278_t * sw, uint8_t rxBW);
 
 //Set address filtering ej: filtering = 0 (disable addressfiltering)
 void SX1278_addressFiltering(SX1278_t * sw, bool filtering, uint8_t addressFiltering);
 
void SX1278_BeginFSK(SX1278_t * sw, uint8_t channel, uint16_t bitRate, uint8_t freqDev,
																		 uint16_t preambleLength, uint8_t rxBW, uint8_t currentLimit,
																		 uint8_t bt, uint8_t outputPower);
 
 //Set Data Shaping FSK
 //[0]: No data shaping
 //[1]: BT = 1
 //[2]: BT = 0.5
 //[3]: BT = 0.3
 void SX1278_dataShaping(SX1278_t * sw, uint8_t bt);
 
 void SX1278_writeMorse(SX1278_t * sw, char str);

 void SX1278_transmitMorse(SX1278_t * sw, char * str, uint8_t len);

 void SX1278_BeginMorse(SX1278_t * sw, uint8_t channel, uint8_t speed);
 
#endif
