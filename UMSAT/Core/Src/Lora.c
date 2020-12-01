
#include "Lora.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//#include "stm32f1xx_it.h"
//#define debug_lora
	
#ifdef debug_lora
	uint8_t _index=0;
#endif
//int aux;
//extern int aux;
uint32_t _dot;

const Morse_t MorseTable[53] = {
  {'A', ".-"},
  {'B',"-..."},
  {'C', "-.-."},
  {'D',"-.."},
  {'E',"."},
  {'F',"..-."},
  {'G',"--."},
  {'H',"...."},
  {'I',".."},
  {'J',".---"},
  {'K',"-.-"},
  {'L',".-.."},
  {'M',"--"},
  {'N',"-."},
  {'O',"---"},
  {'P',".--."},
  {'Q',"--.-"},
  {'R',".-."},
  {'S',"..."},
  {'T',"-"},
  {'U',"..-"},
  {'V',"...-"},
  {'W',".--"},
  {'X',"-..-"},
  {'Y',"-.--"},
  {'Z',"--.."},
  {'1',".----"},
  {'2',"..---"},
  {'3',"...--"},
  {'4',"....-"},
  {'5',"....."},
  {'6',"-...."},
  {'7',"--..."},
  {'8',"---.."},
  {'9',"----."},
  {'0',"-----"},
  {'.',".-.-.-"},
  {',',"--..--"},
  {':',"---..."},
  {'?',"..--.."},
  {'\'',".----."},
  {'-',"-....-"},
  {'/',"-..-."},
  {'(',"-.--."},
  {')',"-.--.-"},
  {'\"',".-..-."},
  {'=',"-...-"},
  {'+',".-.-."},
  {'@',".--.-."},
  {' ',"_"},                               // space is used to separate words
  {0x01,"-.-.-"},                          // ASCII SOH (start of heading) is used as alias for start signal
  {0x02,".-.-."}                           // ASCII EOT (end of transmission) is used as alias for stop signal
};

void delai(uint32_t delay){
	osDelay(delay);

}
void gpio_write(SX1278_hw_gpio_t * gpio, int value) 
{
	HAL_GPIO_WritePin(gpio->port, gpio->pin,
			(value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//[in]: hw
//		  addr
//			*buffer
//			size

void SX1278_ReadBurst(SX1278_t * sw, uint8_t addr, uint8_t *buffer, uint8_t size)
{
		#ifdef debug_lora
			_index=0;
		#endif

    
	
		gpio_write( &sw->hw->nss, 0 );					//NSS = 0;
		
		uint8_t temp = addr&0x7F;
    HAL_SPI_Transmit(sw->hw->spi, &temp, 1, 100);

    for(uint8_t i = 0; i < size; i++ )
    {
      HAL_SPI_Receive(sw->hw->spi, &buffer[i], 1, 100);
			
			#ifdef debug_lora
			printf("[0x%.2X] = 0x%.2X \n",temp + _index, buffer[i]);
			_index++;
			#endif
    }

    gpio_write( &sw->hw->nss, 1 );					//NSS = 1;
}

uint8_t SX1278_ReadByte(SX1278_t * sw, uint8_t addr)
{
		uint8_t byte;
		SX1278_ReadBurst(sw, addr, &byte, 1);
		return byte;
}


void SX1278_WriteBurst(SX1278_t * sw, uint8_t addr, uint8_t *buffer, uint8_t size)
{
		#ifdef debug_lora
			_index=0;
		#endif

    
	
		gpio_write( &sw->hw->nss, 0 );					//NSS = 0;
		
		uint8_t temp = addr | 0x80;
    HAL_SPI_Transmit(sw->hw->spi, &temp, 1, 100);

    for(uint8_t i = 0; i < size; i++ )
    {
      HAL_SPI_Transmit(sw->hw->spi, &buffer[i], 1, 100);
			
			#ifdef debug_lora
			printf("[0x%.2X] = 0x%.2X \n",temp + _index, buffer[i]);
			_index++;
			#endif
    }

    gpio_write( &sw->hw->nss, 1 );					//NSS = 1;
}


void SX1278_WriteByte(SX1278_t * sw, uint8_t addr, uint8_t byte)
{
		SX1278_WriteBurst(sw, addr, &byte, 1);
}

int SX1278_BeginLora(SX1278_t * sw, uint8_t bandwidth , uint8_t channel, uint8_t spreadingFactor, uint8_t codingRate,
																		 uint16_t preambleLength, uint8_t currentLimit,
																		 bool impHeader, uint8_t gain, uint8_t outputPower)	
{
		bool crcOn = 1;
		uint16_t symbTimeout = 0x0064;
//Try to find chip version
		//bool find=false;
	
		sw->Lora.Settings.BW = bandwidth;
		//printf("leyendo\n");
		if( SX1278_ReadByte(sw, REG_LR_VERSION) != CHIP_VERSION )
				return	0;
		//printf("Dispositivo encontrado\n");		
		
		//Lora modem
		SX1278_SetModem(sw, MODEM_LORA);
		
		//Set current protection
		uint8_t a = SetCurrentLimit(sw, currentLimit);
		
		//Set preamble length
		SX1278_SetPreambleLength(sw, preambleLength);
		
		//Set Frequency
		SX1278_SetChannel(sw, channel);	//channel 0 -> 433 MHz (Ver RFLR_FRF_initial)
		
		//Set bandwidth, coding rate and explicit/implicit header
		SX1278_WriteByte(sw, REG_LR_MODEMCONFIG1, ( SX1278_ReadByte(sw, REG_LR_MODEMCONFIG1) &
																								RFLR_MODEMCONFIG1_BW_MASK &
																								RFLR_MODEMCONFIG1_CODINGRATE_MASK &
																								RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
																								( bandwidth << 4 ) | ( codingRate << 1 ) | impHeader );
		
		//Set spreadFactor, CRCOn, symbTimeout
		SX1278_WriteByte(sw, REG_LR_MODEMCONFIG2, ( SX1278_ReadByte(sw, REG_LR_MODEMCONFIG2) &
																								RFLR_MODEMCONFIG2_SF_MASK &
																								RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
																								RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
																								( spreadingFactor << 4 ) | ( crcOn << 2 ) |
																								( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );
		
		SX1278_WriteByte(sw, REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );
		
		//TX Continous mode ON	
		//SX1278_WriteByte(sw, REG_LR_MODEMCONFIG2, (SX1278_ReadByte(sw, REG_LR_MODEMCONFIG2) |
		//																					 RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON));
		
		//Set optimizations SF
		if(spreadingFactor == 0x06)
		{
				SX1278_WriteByte(sw, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
				SX1278_WriteByte(sw, REG_LR_DETECTOPTIMIZE, (SX1278_ReadByte(sw, REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF6 );
		}
		else
		{
				SX1278_WriteByte(sw, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
				SX1278_WriteByte(sw, REG_LR_DETECTOPTIMIZE, (SX1278_ReadByte(sw, REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
		}
		
		//Set Gain LNA
		SX1278_SetLNA(sw, gain);
		
		//Set Output Power (BOOST only)
		sx1278_SetOutputPower(sw, outputPower);
		return 1;
}

void SX1278_BeginFSK(SX1278_t * sw, uint8_t channel, uint16_t bitRate, uint8_t freqDev,
																		 uint16_t preambleLength, uint8_t rxBW, uint8_t currentLimit,
																		 uint8_t bt, uint8_t outputPower)
{
		//Try to find chip
		bool find;
		find = false;
		for(uint8_t i = 0; i < 2; i++ )
		{
				if( SX1278_ReadByte(sw, REG_LR_VERSION) == CHIP_VERSION )
					find = true;
		}
		if(find == false)
		{
				//printf("Dispositivo no encontrado\n");
				while(1);
		}
		//printf("Dispositivo encontrado\n");	
		
		//FSK modem
		SX1278_SetModem(sw, MODEM_FSK);
		
		//Set current protection
		uint8_t a = SetCurrentLimit(sw, currentLimit);
		
		//Set Bit Rate
		SX1278_setBitRate(sw, bitRate);
		
		//Set Frequency Desviation
		SX1278_setFreqDev(sw, freqDev);
		
		//Set RX Bandwidth
		SX1278_setRXBW(sw, rxBW);
		
		//Set Preamble Length
		SX1278_SetPreambleLength(sw, preambleLength);
		
		//Config parameters
		//RSSI threshold (está por defecto)
		SX1278_WriteByte(sw, REG_RSSITHRESH, RF_RSSITHRESH_THRESHOLD);
		
		//Packet Configuration 1
		SX1278_WriteByte(sw, REG_PACKETCONFIG1, RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE |
																						RF_PACKETCONFIG1_DCFREE_WHITENING |
																						RF_PACKETCONFIG1_CRC_ON |
																						RF_PACKETCONFIG1_CRCAUTOCLEAR_ON |
																						RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT);
		//Address Filtering	OFF
		SX1278_addressFiltering(sw, 0, 0);
		
		//Packet Configuration 2 (Packet Mode and IOHome OFF)
		SX1278_WriteByte(sw, REG_PACKETCONFIG2, (SX1278_ReadByte(sw, REG_PACKETCONFIG2) & RF_PACKETCONFIG2_DATAMODE_MASK & RF_PACKETCONFIG2_IOHOME_MASK) |
																						RF_PACKETCONFIG2_DATAMODE_PACKET |
																						RF_PACKETCONFIG2_IOHOME_OFF);
		
		//Adjust preamble polarity (está por defecto)
		SX1278_WriteByte(sw, REG_SYNCCONFIG, (SX1278_ReadByte(sw, REG_SYNCCONFIG) & RF_SYNCCONFIG_PREAMBLEPOLARITY_MASK) |
																				 RF_SYNCCONFIG_PREAMBLEPOLARITY_55);
																				 
		//Set FIFO threshold (está por defecto)
		//Condición de TX start y FIFO threshold para FIFOLevel interrupt
		SX1278_WriteByte(sw, REG_FIFOTHRESH,(SX1278_ReadByte(sw, REG_FIFOTHRESH) & RF_FIFOTHRESH_TXSTARTCONDITION_MASK & RF_FIFOTHRESH_FIFOTHRESHOLD_MASK) |
																				RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY |
																				RF_FIFOTHRESH_FIFOTHRESHOLD_THRESHOLD);
		
		//Disable RX timeout (está por defecto)
		SX1278_WriteByte(sw, REG_RXTIMEOUT1, RF_RXTIMEOUT1_TIMEOUTRXRSSI);
		SX1278_WriteByte(sw, REG_RXTIMEOUT1, RF_RXTIMEOUT2_TIMEOUTRXPREAMBLE);
		SX1278_WriteByte(sw, REG_RXTIMEOUT1, RF_RXTIMEOUT3_TIMEOUTSIGNALSYNC);
		
		//Preamble configuration
		SX1278_WriteByte(sw, REG_PREAMBLEDETECT, RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_2 | RF_PREAMBLEDETECT_DETECTORTOL_10);
		
		//Set Data Shaping Gaussian filter
		SX1278_dataShaping(sw, bt);
		
		//Set Frequency
		SX1278_SetChannel(sw, channel);	//channel 0 -> 433 MHz (Ver RFLR_FRF_initial)
		
		//Set Output Power (BOOST only)
		sx1278_SetOutputPower(sw, outputPower);
		
}
	
void SX1278_BeginMorse(SX1278_t * sw, uint8_t channel, uint8_t speed)
{
		SX1278_BeginFSK(sw, channel, 480, 0, 8, 18, 100, 0, 13);
	
		//Duración de un punto
		_dot = 1200/speed;
}

void SX1278_transmitMorse(SX1278_t * sw, char * str, uint8_t len)
{
	

		for(uint8_t i=0; i<len; i++)
		{
				SX1278_writeMorse(sw, str[i]);
		}
}

void SX1278_writeMorse(SX1278_t * sw, char str)
{
			Morse_t morse;
			bool found = false;
			//printf("morse: entry\n");
			for(uint8_t i=0; i<52; i++)
			{
					
					memcpy(&morse, &MorseTable[i], sizeof(Morse_t));
					//printf("morse1: %c\n",morse.c);
					//printf("morse2: %c\n",str);
					if(morse.c == str)
					{
							//printf("morse: found\n");
							found = true;
							break;
					}
			}
			if(found == 1)
			{
					for(uint8_t i =0; i<strlen(morse.m); i++)
					{
							switch(morse.m[i])
							{
								case '.':
									//printf("morse: punto\n");
									SX1278_SetOpMode(sw, RFLR_OPMODE_TRANSMITTER);
									osDelay(_dot);
									break;
								case '-':
									SX1278_SetOpMode(sw, RFLR_OPMODE_TRANSMITTER);
								//printf("morse: raya\n");
									osDelay(_dot*3);
									break;
								default:
									break;
							}
							
							SX1278_SetOpMode(sw, RFLR_OPMODE_STANDBY);
							osDelay(_dot);
					}
					
					//Espacio entre letras
					osDelay(_dot*3);
			}

}

void SX1278_setBitRate(SX1278_t * sw, uint16_t bitRate)
{
		uint16_t _br = 320000/bitRate;
		SX1278_WriteByte(sw, REG_BITRATEMSB, (_br>>8) & 0x00FF);
		SX1278_WriteByte(sw, REG_BITRATELSB, _br & 0x00FF);
}
		
void SX1278_setFreqDev(SX1278_t * sw, uint8_t freqDev)
{
		uint32_t div = 1<<19;
		uint16_t _fd = (freqDev*div)/32000;
		SX1278_WriteByte(sw, REG_FDEVMSB, (_fd>>8) & 0x00FF);
		SX1278_WriteByte(sw, REG_FDEVLSB, _fd & 0x00FF);
}

void SX1278_setRXBW(SX1278_t * sw, uint8_t rxBW)
{
		//Mantisa
		uint8_t mant = rxBW%3;
		//Exponente
		uint8_t exp = 7 - (rxBW/3);
	
		SX1278_WriteByte(sw, REG_AFCBW, (mant<<3) | exp);
		SX1278_WriteByte(sw, REG_RXBW, (mant<<3) | exp);		
}			

//FSK only
void SX1278_addressFiltering(SX1278_t * sw, bool filtering, uint8_t addressFiltering)
{
		if(filtering == 1)
		{
				SX1278_WriteByte(sw, REG_PACKETCONFIG1, (SX1278_ReadByte(sw, REG_PACKETCONFIG1) & RF_PACKETCONFIG1_ADDRSFILTERING_MASK) |
																								RF_PACKETCONFIG1_ADDRSFILTERING_NODE);
				SX1278_WriteByte(sw, REG_NODEADRS, addressFiltering);
		}
		else
				SX1278_WriteByte(sw, REG_PACKETCONFIG1, (SX1278_ReadByte(sw, REG_PACKETCONFIG1) & RF_PACKETCONFIG1_ADDRSFILTERING_MASK) |
																								RF_PACKETCONFIG1_ADDRSFILTERING_OFF);

}

//FSK only
//Set Data Shaping
void SX1278_dataShaping(SX1278_t * sw, uint8_t bt)
{
		//Standby		
		SX1278_SetOpMode(sw, RFLR_OPMODE_STANDBY);
	
		SX1278_WriteByte(sw, REG_PARAMP, (SX1278_ReadByte(sw, REG_PARAMP) & RF_PARAMP_MODULATIONSHAPING_MASK) |
																		 ((bt<<5) & 0x60));

}
																		 
void sx1278_SetOutputPower(SX1278_t * sw, uint8_t power)
{
		if( !((power >= 2 && power <= 17) || power == 20) )
				{
				//printf("Error Power Output \n");
				while(1);
				}
				
		//Standby		
		SX1278_SetOpMode(sw, RFLR_OPMODE_STANDBY);	
				
		if( (power >= 2) && (power <= 17) )
		{
				SX1278_WriteByte(sw, REG_LR_PACONFIG, RFLR_PACONFIG_PASELECT_PABOOST | ((power - 2) & 0x0F));
				SX1278_WriteByte(sw, REG_LR_PADAC, (SX1278_ReadByte(sw, REG_LR_PADAC) & RFLR_PADAC_20DBM_MASK) | RFLR_PADAC_20DBM_OFF);
		}
		else if( power == 20 )
		{
				SX1278_WriteByte(sw, REG_LR_PACONFIG, RFLR_PACONFIG_PASELECT_PABOOST | ((power - 5) & 0x7F));
				SX1278_WriteByte(sw, REG_LR_PADAC, (SX1278_ReadByte(sw, REG_LR_PADAC) & RFLR_PADAC_20DBM_MASK) | RFLR_PADAC_20DBM_ON);
		}	
}

//No compatible con FSK
void SX1278_SetLNA(SX1278_t * sw, uint8_t gain)
{
		if(gain > 6)
		{
				//printf("Ganancia fuera de rango");
				while(1);
		}
		SX1278_SetOpMode(sw, RFLR_OPMODE_STANDBY);
		if(gain == 0)
		{
				SX1278_WriteByte(sw, REG_LR_MODEMCONFIG3, (SX1278_ReadByte(sw, REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_AGCAUTO_MASK) | RFLR_MODEMCONFIG3_AGCAUTO_ON );
		}
		else
		{
				SX1278_WriteByte(sw, REG_LR_MODEMCONFIG3, (SX1278_ReadByte(sw, REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_AGCAUTO_MASK) | RFLR_MODEMCONFIG3_AGCAUTO_OFF );
				SX1278_WriteByte(sw, REG_LR_LNA, (SX1278_ReadByte(sw, REG_LR_LNA) & RFLR_LNA_BOOST_LF_MASK & RFLR_LNA_GAIN_MASK) | (gain << 5) | RFLR_LNA_BOOST_LF_DEFAULT);
		}
}

//CurrentLimit: 0 to 120 mAh (step: 5 mAh)      130 to 240 mAh (step: 10 mAh)
//Lora y FSK
uint8_t SetCurrentLimit(SX1278_t * sw, uint8_t CurrentLimit)	//********************************************************************
{
	uint8_t raw;
		if (!((CurrentLimit >= 45 && CurrentLimit <= 240) || (CurrentLimit == 0)))
				return 0;
		
		SX1278_SetOpMode( sw, RFLR_OPMODE_STANDBY);		//Standby
		if (CurrentLimit == 0)
				{
					SX1278_WriteByte(sw, REG_LR_OCP, (SX1278_ReadByte(sw, REG_LR_OCP) & RFLR_OCP_MASK) | RFLR_OCP_OFF);
					return 1;
				}
	
		if ( (CurrentLimit >= 45 && CurrentLimit <= 120))
					raw = (CurrentLimit - 45) / 5;	
		if ((CurrentLimit >= 130 && CurrentLimit <= 240))
					raw = (CurrentLimit + 30) / 10;
		SX1278_WriteByte(sw, REG_LR_OCP, (SX1278_ReadByte(sw, REG_LR_OCP) &
																				RFLR_OCPTRIM_MASK) |
																				raw);
	return 1;
}

//Lora and FSK mode
uint8_t SX1278_SetPreambleLength(SX1278_t * sw, uint16_t PreambleLength)
{
	uint8_t modem = SX1278_GetModem(sw);
		SX1278_SetOpMode( sw, RF_OPMODE_STANDBY);		//Standby
		if(modem == MODEM_LORA)
		{
				if(PreambleLength < 6)									//Rango no admitido para Lora
					return 0;
				SX1278_WriteByte(sw, REG_LR_PREAMBLEMSB, (PreambleLength & 0xFF00) >> 8);
				SX1278_WriteByte(sw, REG_LR_PREAMBLELSB, (PreambleLength & 0x00FF));
				return 1;
		}
		else if(modem == MODEM_FSK)
		{
				SX1278_WriteByte(sw, REG_PREAMBLEMSB, (PreambleLength & 0xFF00) >> 8);
				SX1278_WriteByte(sw, REG_PREAMBLELSB, (PreambleLength & 0x00FF));
				return 1;
		}
		return 0;
}

//Lora and FSK mode
//new_freq(ch) = 6c4000h (433MHz) + ch*800h (125KHz)
void SX1278_SetChannel(SX1278_t * sw, uint8_t Channel)
{
		if(SX1278_GetModem(sw) == MODEM_LORA)
		{
				// sensitivity optimization for 500kHz Bandwidth
				if(sw->Lora.Settings.BW == 0x09)
				{
						//Errata, section 2.1 Sensitivity Optimizacion 500 KHz
						SX1278_WriteByte(sw, 0x36, 0x02);
						SX1278_WriteByte(sw, 0x3A, 0x7F);
						//printf("** [0x36]: 0x%.2X \n", SX1278_ReadByte(sw, 0x36));
						//printf("** [0x3A]: 0x%.2X \n", SX1278_ReadByte(sw, 0x3A));
						//Errata, section 2.3 Mitigation of receiver spurious response 500 KHz
						//printf("** [0x31]: 0x%.2X \n", SX1278_ReadByte(sw, 0x31));
						SX1278_WriteByte(sw, 0x31, SX1278_ReadByte(sw, 0x31) | 0x80);
						//printf("** [0x31]: 0x%.2X \n", SX1278_ReadByte(sw, 0x31));
				}
				else if(sw->Lora.Settings.BW == 0x08)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 250 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x40);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				else if(sw->Lora.Settings.BW == 0x07)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 125 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x40);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				else if(sw->Lora.Settings.BW == 0x06)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 62.5 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x40);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				else if(sw->Lora.Settings.BW == 0x05)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 41.7 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x44);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				else if(sw->Lora.Settings.BW == 0x04)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 31.25 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x44);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				else if(sw->Lora.Settings.BW == 0x03)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 20.8 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x44);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				else if(sw->Lora.Settings.BW == 0x02)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 15.6 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x44);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				else if(sw->Lora.Settings.BW == 0x01)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 10.4 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x44);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				else if(sw->Lora.Settings.BW == 0x00)
				{
						//Errata, section 2.3 Mitigation of receiver spurious response 7.8 KHz
						SX1278_WriteByte(sw, 0x31, (SX1278_ReadByte(sw, 0x31) & (0x80)) | 0x00);
						SX1278_WriteByte(sw, 0x2F, 0x48);
						SX1278_WriteByte(sw, 0x30, 0x00);	
				}
				
		}

		SX1278_SetOpMode( sw, RF_OPMODE_STANDBY);
		
		//uint32_t FRF = 0x6C4000; //20*step
		
		uint32_t FRF = RFLR_FRF_initial + (uint32_t)Channel*RFLR_FRF_step;
		//printf("frecuencia: 0x%.4X \n", FRF);
		SX1278_WriteByte(sw, REG_LR_FRFMSB, (FRF & 0xFF0000) >> 16);
		SX1278_WriteByte(sw, REG_LR_FRFMID, (FRF & 0x00FF00) >> 8);
		SX1278_WriteByte(sw, REG_LR_FRFLSB, (FRF & 0x0000FF));
		
}


//[mode]: 0 (Sleep), 1 (Standby)...
void	SX1278_SetOpMode(SX1278_t * sw, uint8_t mode)
{
	//printf("OpMode: 0x%.2X\n",SX1278_ReadByte(hw, REG_LR_OPMODE));
		SX1278_WriteByte(sw, REG_LR_OPMODE, (SX1278_ReadByte(sw, REG_LR_OPMODE) &
																				RFLR_OPMODE_MASK) |
																				mode);
		sw->State = mode;
	//printf("OpMode: 0x%.2X\n",SX1278_ReadByte(hw, REG_LR_OPMODE));
}



void SX1278_SetModem(SX1278_t * sw, SX1278_RadioModems_t modem )
{
    sw->Modem = modem;
    
		
    
    switch( sw->Modem )
    {
    default:
    case MODEM_FSK:
				if ((SX1278_ReadByte(sw, REG_LR_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) == RFLR_OPMODE_LONGRANGEMODE_OFF)
						break;
        SX1278_SetOpMode( sw, RFLR_OPMODE_SLEEP);
        SX1278_WriteByte(sw, REG_LR_OPMODE, ( SX1278_ReadByte(sw, REG_LR_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF ); //Fsk ON

        //SX1276Write( REG_DIOMAPPING1, 0x00 );
        //SX1276Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        if ((SX1278_ReadByte(sw, REG_LR_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) == RFLR_OPMODE_LONGRANGEMODE_ON)
						break;
				SX1278_SetOpMode( sw, RF_OPMODE_SLEEP);
				SX1278_WriteByte(sw, REG_LR_OPMODE, ( SX1278_ReadByte(sw, REG_LR_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON ); //Lora ON
        //SX1276Write( REG_DIOMAPPING1, 0x00 );
        //SX1276Write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}

SX1278_RadioModems_t SX1278_GetModem(SX1278_t * sw)
{
		if((SX1278_ReadByte(sw, REG_OPMODE) & 0x80) == 0x80)
			return MODEM_LORA;
		else
			return MODEM_FSK;

}

//*********************************************************************************
//*********************************************************************************
void sx1278_transmit(SX1278_t * sw, uint8_t * data, size_t len)
{
	
		//extern int flag;
		SX1278_SetOpMode(sw, RF_OPMODE_STANDBY);
	
		if(sw->Modem == MODEM_LORA)
		{
				sx1278_startTransmit(sw, data, len);
				int temp = HAL_GetTick();
				//printf("Lora mode transmit\n");
				HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);	
				//wait
				while( ((SX1278_ReadByte(sw, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_TXDONE_MASK) != RFLR_IRQFLAGS_TXDONE) )
				//while(flag == 0)
				{
						osDelay(1);
				
				}

				//flag=0;
				//HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
				//osDelay(20000);
				HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
				//printf("time elapsed = %d\n", HAL_GetTick() - temp);
				sx1278_clearIRQ(sw);
				SX1278_receive(sw);
		
		}
			else if(sw->Modem == MODEM_FSK)
		{
				sx1278_startTransmit(sw, data, len);
				int temp = HAL_GetTick();
				//printf("fsk mode \n");
				HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);	
				//wait
				while( ((SX1278_ReadByte(sw, REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) != RF_IRQFLAGS2_PACKETSENT) )
				//while(flag == 0)
				{
						osDelay(1);
		
				}
				HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
				//printf("time elapsed = %d\n", HAL_GetTick() - temp);
				sx1278_clearIRQ(sw);
				
				SX1278_SetOpMode(sw, RF_OPMODE_STANDBY);
		}
		

}

//extern int aux;
 void sx1278_startTransmit(SX1278_t *sw, uint8_t * data, uint8_t len)
{
	
		if(sw->Modem == MODEM_LORA)
		{
	
		//DIO0 to TX DONE
		//SX1278_WriteByte(sw, REG_LR_DIOMAPPING1, (SX1278_ReadByte(sw, REG_LR_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
			
		//clear IRQ
		sx1278_clearIRQ(sw);
		//aux = 0;
	
		//Set payload length
		SX1278_WriteByte(sw, REG_LR_PAYLOADLENGTH, len);
	
		//Adjust FIFO pointer
		SX1278_WriteByte(sw, REG_LR_FIFOTXBASEADDR, RFLR_FIFOTXBASEADDR_MAX);
		SX1278_WriteByte(sw, REG_LR_FIFOADDRPTR, RFLR_FIFOTXBASEADDR_MAX);
	
		//Write FIFO
		SX1278_WriteBurst(sw, REG_LR_FIFO, data, len);
	
		//RF transmission
		SX1278_SetOpMode(sw, RFLR_OPMODE_TRANSMITTER);
		

		
		} 
		else if(sw->Modem == MODEM_FSK)
		{
				//DIO0 to Packet sent
				SX1278_WriteByte(sw, REG_DIOMAPPING1, (SX1278_ReadByte(sw, REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK) | RF_DIOMAPPING1_DIO1_00);
			
				//clear IRQ
				sx1278_clearIRQ(sw);
			
				//Set payload length
				SX1278_WriteByte(sw, REG_FIFO, len);
			
				//Write FIFO
				SX1278_WriteBurst(sw, REG_FIFO, data, len);
			
				//RF transmission
				SX1278_SetOpMode(sw, RFLR_OPMODE_TRANSMITTER);
				
		}
}

void SX1278_receive(SX1278_t *sw)
{
		SX1278_SetOpMode(sw, RF_OPMODE_STANDBY);
	
		if(sw->Modem == MODEM_LORA )
		{
				SX1278_startReceive(sw);
				
		
		}


}

void SX1278_startReceive(SX1278_t * sw)
{
		if(sw->Modem == MODEM_LORA)
		{
				//DIO0 to RX DONE & DIO3 to VALID HEADER
				SX1278_WriteByte(sw, REG_LR_DIOMAPPING1, (SX1278_ReadByte(sw, REG_LR_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO1_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO3_01);
			
				//clear IRQ
				sx1278_clearIRQ(sw);
				//aux = 0;

				//Set MAX payload length
				SX1278_WriteByte(sw, REG_LR_PAYLOADLENGTH, RFLR_PAYLOADMAXLENGTH);
			
				//Adjust FIFO pointer
				SX1278_WriteByte(sw, REG_LR_FIFORXBASEADDR, RFLR_FIFORXBASEADDR);
				SX1278_WriteByte(sw, REG_LR_FIFOADDRPTR, RFLR_FIFORXBASEADDR);
			
				//RX Continuos Mode
				SX1278_SetOpMode(sw, RFLR_OPMODE_RECEIVER);
		
		}
}

void sx1278_clearIRQ(SX1278_t * sw)
{
		if(sw->Modem == MODEM_LORA)
				SX1278_WriteByte(sw, REG_LR_IRQFLAGS, 0xFF);
		else if(sw->Modem == MODEM_FSK)
		{
				SX1278_WriteByte(sw, REG_IRQFLAGS1, 0xFF);
				SX1278_WriteByte(sw, REG_IRQFLAGS2, 0xFF);
		}
}

void SX1278_reset(SX1278_t * sw)
{
		gpio_write( &sw->hw->reset, 0 );					//RST = 0;
		osDelay(10);
		gpio_write( &sw->hw->reset, 1 );					//RST = 1;
		osDelay(10);
}

void SX1278_printBeginLora(SX1278_t * sw)
{
		//printf("LoRa Module \n");
	
		//printf("bandwidth [0x1D]: 0x%.2X \n", SX1278_ReadByte(sw, 0x1D));
		
		//printf("FRF [0x06] MSB: 0x%.2X \n", SX1278_ReadByte(sw, 0x06));
		//printf("FRF: 0x%.2X \n", SX1278_ReadByte(sw, 0x07));
		//printf("FRF LSB: 0x%.2X \n", SX1278_ReadByte(sw, 0x08));
		
		//printf("SF [0x1E]: 0x%.2X \n", SX1278_ReadByte(sw, 0x1E));
		
		//printf("CR [0x1D]: 0x%.2X \n", SX1278_ReadByte(sw, 0x1D));
		
		//printf("PREAMBLE LENGTH MSB [0x20]: 0x%.2X \n", SX1278_ReadByte(sw, 0x20));
		//printf("PREAMBLE LENGTH LSB [0x21]: 0x%.2X \n", SX1278_ReadByte(sw, 0x21));
		
		//printf("OCP [0x0B]: 0x%.2X \n", SX1278_ReadByte(sw, 0x0B));
		
		//printf("IMP HEADER [0x1D]: 0x%.2X \n", SX1278_ReadByte(sw, 0x1D));
		
		//printf("GAIN [0x26]: 0x%.2X \n", SX1278_ReadByte(sw, 0x26));
		//printf("GAIN [0x27]: 0x%.2X \n", SX1278_ReadByte(sw, 0x0C));
}

