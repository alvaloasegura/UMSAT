
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "main.h"
#include <stdio.h>

#define	bmp280addr	0x76
#define	TMP102addr	0x48
#define	INA219cell	0x41
#define	INA219solar	0x40
#define	IMUaddr			0x68


typedef struct
{
	uint8_t			UTC_Hour;
	uint8_t			UTC_Min;
	uint8_t			UTC_Sec;
	uint16_t		UTC_MicroSec;
	
	float				Latitude;
	double			LatitudeDecimal;
	char				NS_Indicator;
	float				Longitude;
	double			LongitudeDecimal;
	char				EW_Indicator;
	
	uint8_t			PositionFixIndicator;
	uint8_t			SatellitesUsed;
	float				HDOP;
	float				MSL_Altitude;
	char				MSL_Units;
	float				Geoid_Separation;
	char				Geoid_Units;
	
	uint16_t		AgeofDiffCorr;
	char				DiffRefStationID[4];
	char				CheckSum[2];	
	
}GNGGA_t;

typedef struct 
{
	uint8_t		rxBuffer[512];
	uint16_t	rxIndex;
	uint8_t		rxTmp;	
	uint32_t	LastTime;	
	
	GNGGA_t		GNGGA;
	
}GPS_t;

typedef enum 
    {
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    }GyroRange;
typedef enum
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G    
    }AccelRange;
typedef enum
    {
      DLPF_BANDWIDTH_184HZ,
      DLPF_BANDWIDTH_92HZ,
      DLPF_BANDWIDTH_41HZ,
      DLPF_BANDWIDTH_20HZ,
      DLPF_BANDWIDTH_10HZ,
      DLPF_BANDWIDTH_5HZ
    }DlpfBandwidth;
typedef enum
    {
      LP_ACCEL_ODR_0_24HZ = 0,
      LP_ACCEL_ODR_0_49HZ = 1,
      LP_ACCEL_ODR_0_98HZ = 2,
      LP_ACCEL_ODR_1_95HZ = 3,
      LP_ACCEL_ODR_3_91HZ = 4,
      LP_ACCEL_ODR_7_81HZ = 5,
      LP_ACCEL_ODR_15_63HZ = 6,
      LP_ACCEL_ODR_31_25HZ = 7,
      LP_ACCEL_ODR_62_50HZ = 8,
      LP_ACCEL_ODR_125HZ = 9,
      LP_ACCEL_ODR_250HZ = 10,
      LP_ACCEL_ODR_500HZ = 11
    }LpAccelOdr;
		
typedef enum {
    /** 1 ms standby. */
    STANDBY_MS_1 = 0x00,
    /** 62.5 ms standby. */
    STANDBY_MS_63 = 0x01,
    /** 125 ms standby. */
    STANDBY_MS_125 = 0x02,
    /** 250 ms standby. */
    STANDBY_MS_250 = 0x03,
    /** 500 ms standby. */
    STANDBY_MS_500 = 0x04,
    /** 1000 ms standby. */
    STANDBY_MS_1000 = 0x05,
    /** 2000 ms standby. */
    STANDBY_MS_2000 = 0x06,
    /** 4000 ms standby. */
    STANDBY_MS_4000 = 0x07
  }standby_duration;
	
typedef enum {
    /** No filtering. */
    FILTER_OFF = 0x00,
    /** 2x filtering. */
    FILTER_X2 = 0x01,
    /** 4x filtering. */
    FILTER_X4 = 0x02,
    /** 8x filtering. */
    FILTER_X8 = 0x03,
    /** 16x filtering. */
    FILTER_X16 = 0x04
  }sensor_filter;	

 typedef enum {
    /** No over-sampling. */
    SAMPLING_NONE = 0x00,
    /** 1x over-sampling. */
    SAMPLING_X1 = 0x01,
    /** 2x over-sampling. */
    SAMPLING_X2 = 0x02,
    /** 4x over-sampling. */
    SAMPLING_X4 = 0x03,
    /** 8x over-sampling. */
    SAMPLING_X8 = 0x04,
    /** 16x over-sampling. */
    SAMPLING_X16 = 0x05
  }sensor_sampling;

  /** Operating mode for the sensor. */
typedef  enum {
    /** Sleep mode. */
    MODE_SLEEP = 0x00,
    /** Forced mode. */
    MODE_FORCED = 0x01,
    /** Normal mode. */
    MODE_NORMAL = 0x03,
    /** Software reset. */
    MODE_SOFT_RESET_CODE = 0xB6
  }sensor_mode;
	
typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;
	

typedef struct {
	I2C_HandleTypeDef *i2c_port;

}sensor_handle;









void bmpconfig(sensor_handle *handle);
float bmpTemp(sensor_handle *handle);
float bmpPress(sensor_handle *handle);
float bmpAlt(sensor_handle *handle, float nm_hPA);

void TMPconfig(sensor_handle *handle);
float TMPtemp(sensor_handle *handle);

void INAbegin(sensor_handle *handle);
float INA219_ShuntV (sensor_handle *handle, uint16_t INAaddress);
float INA219_BusV (sensor_handle *handle, uint16_t INAaddress);
float INA219_Power (sensor_handle *handle, uint16_t INAaddress);
float INA219_Current (sensor_handle *handle, uint16_t INAaddress);

void IMUconfig(sensor_handle *handle);
HAL_StatusTypeDef IMUwriteRegister(sensor_handle *handle, uint8_t reg, uint8_t data);
HAL_StatusTypeDef IMUreadRegister(sensor_handle *handle, uint8_t reg, uint8_t *data, uint8_t size);
int AK8963writeRegister(sensor_handle *handle, uint8_t reg, uint8_t data);
HAL_StatusTypeDef AK8963readRegister(sensor_handle *handle, uint8_t reg, uint8_t *data, uint8_t size);
void calGyro(sensor_handle *handle);
void readIMU(sensor_handle *handle);
float IMUread_Temperatura(sensor_handle *handle);
float IMUread_AcelX(sensor_handle *handle);
float IMUread_AcelY(sensor_handle *handle);
float IMUread_AcelZ(sensor_handle *handle);
float IMUread_GyrX(sensor_handle *handle);
float IMUread_GyrY(sensor_handle *handle);
float IMUread_GyrZ(sensor_handle *handle);
float IMUread_MagX(sensor_handle *handle);
float IMUread_MagY(sensor_handle *handle);
float IMUread_MagZ(sensor_handle *handle);

double convertDegMinToDecDeg (float degMin);
void	GPS_Process(GPS_t *GPS, uint8_t *rxBuffer);

#endif
