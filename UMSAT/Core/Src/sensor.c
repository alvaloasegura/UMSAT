#include "sensor.h"
#include <string.h>
#include "math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

	    // MPU9250 registers and constants
		size_t _numSamples = 100;
    const uint8_t ACCEL_OUT = 0x3B;
    const uint8_t GYRO_OUT = 0x43;
    const uint8_t TEMP_OUT = 0x41;
    uint8_t EXT_SENS_DATA_00 = 0x49;
    const uint8_t ACCEL_CONFIG = 0x1C;
    const uint8_t ACCEL_FS_SEL_2G = 0x00;
    const uint8_t ACCEL_FS_SEL_4G = 0x08;
    const uint8_t ACCEL_FS_SEL_8G = 0x10;
		
#define			ACCEL_FS_SEL_16G		0x18
//    const uint8_t ACCEL_FS_SEL_16G = 0x18;
    const uint8_t GYRO_CONFIG = 0x1B;
    const uint8_t GYRO_FS_SEL_250DPS = 0x00;
    const uint8_t GYRO_FS_SEL_500DPS = 0x08;
    const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
    const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
    const uint8_t ACCEL_CONFIG2 = 0x1D;
    const uint8_t ACCEL_DLPF_184 = 0x01;
    const uint8_t ACCEL_DLPF_92 = 0x02;
    const uint8_t ACCEL_DLPF_41 = 0x03;
    const uint8_t ACCEL_DLPF_20 = 0x04;
    const uint8_t ACCEL_DLPF_10 = 0x05;
    const uint8_t ACCEL_DLPF_5 = 0x06;
    const uint8_t CONFIG = 0x1A;
    const uint8_t GYRO_DLPF_184 = 0x01;
    const uint8_t GYRO_DLPF_92 = 0x02;
    const uint8_t GYRO_DLPF_41 = 0x03;
    const uint8_t GYRO_DLPF_20 = 0x04;
    const uint8_t GYRO_DLPF_10 = 0x05;
    const uint8_t GYRO_DLPF_5 = 0x06;
    const uint8_t SMPDIV = 0x19;
    const uint8_t INT_PIN_CFG = 0x37;
    const uint8_t INT_ENABLE = 0x38;
    const uint8_t INT_DISABLE = 0x00;
    const uint8_t INT_PULSE_50US = 0x00;
    const uint8_t INT_WOM_EN = 0x40;
    const uint8_t INT_RAW_RDY_EN = 0x01;
    const uint8_t PWR_MGMNT_1 = 0x6B;
    const uint8_t PWR_CYCLE = 0x20;
    const uint8_t PWR_RESET = 0x80;
    const uint8_t CLOCK_SEL_PLL = 0x01;
    const uint8_t PWR_MGMNT_2 = 0x6C;
    const uint8_t SEN_ENABLE = 0x00;
    const uint8_t DIS_GYRO = 0x07;
    const uint8_t USER_CTRL = 0x6A;
    const uint8_t I2C_MST_EN = 0x20;
    const uint8_t I2C_MST_CLK = 0x0D;
    const uint8_t I2C_MST_CTRL = 0x24;
    const uint8_t I2C_SLV0_ADDR = 0x25;
    const uint8_t I2C_SLV0_REG = 0x26;
    const uint8_t I2C_SLV0_DO = 0x63;
    const uint8_t I2C_SLV0_CTRL = 0x27;
    const uint8_t I2C_SLV0_EN = 0x80;
    const uint8_t I2C_READ_FLAG = 0x80;
    const uint8_t MOT_DETECT_CTRL = 0x69;
    const uint8_t ACCEL_INTEL_EN = 0x80;
    const uint8_t ACCEL_INTEL_MODE = 0x40;
    const uint8_t LP_ACCEL_ODR = 0x1E;
    const uint8_t WOM_THR = 0x1F;
    const uint8_t WHO_AM_I = 0x75;
    const uint8_t FIFO_EN = 0x23;
    const uint8_t FIFO_TEMP = 0x80;
    const uint8_t FIFO_GYRO = 0x70;
    const uint8_t FIFO_ACCEL = 0x08;
    const uint8_t FIFO_MAG = 0x01;
    const uint8_t FIFO_COUNT = 0x72;
    const uint8_t FIFO_READ = 0x74;
    // AK8963 registers
    const uint8_t AK8963_I2C_ADDR = 0x0C;
    const uint8_t AK8963_HXL = 0x03; 
    const uint8_t AK8963_CNTL1 = 0x0A;
    const uint8_t AK8963_PWR_DOWN = 0x00;
    const uint8_t AK8963_CNT_MEAS1 = 0x12;
    const uint8_t AK8963_CNT_MEAS2 = 0x16;
    const uint8_t AK8963_FUSE_ROM = 0x0F;
    const uint8_t AK8963_CNTL2 = 0x0B;
    const uint8_t AK8963_RESET = 0x01;
    const uint8_t AK8963_ASA = 0x10;
    const uint8_t AK8963_WHO_AM_I = 0x00;
		// transformation matrix XYZ
		const int16_t tX[3] = {0,  1,  0}; 
    const int16_t tY[3] = {1,  0,  0};
    const int16_t tZ[3] = {0,  0, -1};
		const float G = 9.807f;
		// temperature constant
		const float _tempScale = 333.87f;
    const float _tempOffset = 21.0f;

  uint8_t BMP280_REGISTER_DIG_T1 = 0x88;
  uint8_t BMP280_REGISTER_DIG_T2 = 0x8A;
  uint8_t BMP280_REGISTER_DIG_T3 = 0x8C;
  uint8_t BMP280_REGISTER_DIG_P1 = 0x8E;
  uint8_t BMP280_REGISTER_DIG_P2 = 0x90;
  uint8_t BMP280_REGISTER_DIG_P3 = 0x92;
  uint8_t BMP280_REGISTER_DIG_P4 = 0x94;
  uint8_t BMP280_REGISTER_DIG_P5 = 0x96;
  uint8_t BMP280_REGISTER_DIG_P6 = 0x98;
  uint8_t BMP280_REGISTER_DIG_P7 = 0x9A;
  uint8_t BMP280_REGISTER_DIG_P8 = 0x9C;
  uint8_t BMP280_REGISTER_DIG_P9 = 0x9E;
  uint8_t BMP280_REGISTER_CHIPID = 0xD0;
  uint8_t BMP280_REGISTER_VERSION = 0xD;
  uint8_t BMP280_REGISTER_SOFTRESET = 0xE0;
  uint8_t BMP280_REGISTER_CAL26 = 0xE1; /**< R calibration = 0xE1-0xF0 */
  uint8_t BMP280_REGISTER_STATUS = 0xF3;
  uint8_t BMP280_REGISTER_CONTROL = 0xF4;
  uint8_t BMP280_REGISTER_CONFIG = 0xF5;
  uint8_t BMP280_REGISTER_PRESSUREDATA = 0xF7;
  uint8_t BMP280_REGISTER_TEMPDATA = 0xFA;
	
	float _accelScale, _gyroScale;
	float _magScaleX, _magScaleY, _magScaleZ;
	const float _d2r = 3.14159265359f/180.0f;
	AccelRange _accelRange;
  GyroRange _gyroRange;
	DlpfBandwidth _bandwidth;
	uint8_t _srd;
	uint8_t IMUbuffer[25];
	
	// data counts
	int16_t _axcounts,_aycounts,_azcounts;
	int16_t _gxcounts,_gycounts,_gzcounts;
	int16_t _hxcounts,_hycounts,_hzcounts;
	int16_t _tcounts;
	
	// bias
	float _gxb=0, _gyb=0, _gzb=0;
	float _axb=0, _ayb=0, _azb=0;
	float _hxb=0, _hyb=0, _hzb=0;
	// scale
	float _axs = 1.0f;
  float _ays = 1.0f;
  float _azs = 1.0f;
	// data buffer
	float _ax, _ay, _az;
	float _gx, _gy, _gz;
	float _hx, _hy, _hz;
	float _hxs = 1.0f;
  float _hys = 1.0f;
  float _hzs = 1.0f;
	// temperature
	float _t;
	
	uint8_t TMP102_REGISTER_CONFIG = 0x01;
	uint8_t TMP102_REGISTER_TEMPDATA = 0x00;
	
	uint8_t INA219_REGISTER_CONFIG = 0x00;
	uint8_t raw_INA8[2];
	uint16_t raw_INA;
	
	uint8_t		dig[2];
	uint8_t tempReg[3];
	bmp280_calib_data bmp;
	int32_t t_fine;

void bmpconfig(sensor_handle *handle)
{
	//Configuracion
//*********************************************
		tempReg[0] = BMP280_REGISTER_CONFIG;
		tempReg[1] = (STANDBY_MS_250 << 5) | (FILTER_X4 << 2) | 1;
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, tempReg , 2, HAL_MAX_DELAY);
		
		tempReg[0] = BMP280_REGISTER_CONTROL;
		tempReg[1] = (SAMPLING_X4 << 5) | (SAMPLING_X4 << 2) | (MODE_NORMAL);
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, tempReg , 2, HAL_MAX_DELAY);
		
//*********************************************
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_T1 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_T1 = dig[1]<<8 | dig[0];
		//printf("0x%.2X\n", bmp.dig_T1);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_T2 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_T2 = (int16_t)(dig[1]<<8 | dig[0]);
		//printf("0x%.2X\n", bmp.dig_T2);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_T3 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_T3 = (int16_t)(dig[1]<<8 | dig[0]);
		//printf("0x%.2X\n", bmp.dig_T3);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P1 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P1 = dig[1]<<8 | dig[0];
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P2 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P2 = (int16_t)(dig[1]<<8 | dig[0]);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P3 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P3 = (int16_t)(dig[1]<<8 | dig[0]);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P4 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P4 = (int16_t)(dig[1]<<8 | dig[0]);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P5 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P5 = (int16_t)(dig[1]<<8 | dig[0]);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P6 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P6 = (int16_t)(dig[1]<<8 | dig[0]);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P7 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P7 = (int16_t)(dig[1]<<8 | dig[0]);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P8 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P8 = (int16_t)(dig[1]<<8 | dig[0]);
		
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_DIG_P9 , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, dig, 2, HAL_MAX_DELAY);
		bmp.dig_P9 = (int16_t)(dig[1]<<8 | dig[0]);
}
void TMPconfig(sensor_handle *handle)
{
		tempReg[0] = TMP102_REGISTER_CONFIG;
		tempReg[1] = 0x02;
		tempReg[2] = 0x20;
		HAL_I2C_Master_Transmit(handle->i2c_port, TMP102addr<<1, tempReg , 3, HAL_MAX_DELAY);

}

void INAbegin(sensor_handle *handle)
{
		tempReg[0] = INA219_REGISTER_CONFIG;
		tempReg[1] = 0x39;
		tempReg[2] = 0x9F;
	  HAL_I2C_Master_Transmit(handle->i2c_port, INA219cell<<1, tempReg, 3, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(handle->i2c_port, INA219solar<<1, tempReg, 3, HAL_MAX_DELAY);
		//Calibration
		tempReg[0] = 0x05;           //APUNTAR AL REGISTRO DE CONFIGURACION
    tempReg[1] = 0x10;           //
		tempReg[2] = 0x00;           //
		HAL_I2C_Master_Transmit(handle->i2c_port, INA219cell<<1, tempReg, 3, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(handle->i2c_port, INA219solar<<1, tempReg, 3, HAL_MAX_DELAY);
}

float INA219_ShuntV (sensor_handle *handle, uint16_t INAaddress)
{
		tempReg[0] = 0x01; 					//APUNTAR AL REGISTRO DE CONFIGURACION
    HAL_I2C_Master_Transmit(handle->i2c_port, INAaddress<<1, tempReg, 1, HAL_MAX_DELAY);
		
    raw_INA8[0] = 0x00;
		HAL_I2C_Master_Receive(handle->i2c_port, INAaddress<<1, raw_INA8, 2, HAL_MAX_DELAY);
    raw_INA = (raw_INA8[0] << 8 | raw_INA8[1]);
		
		return raw_INA*0.01;
}

float INA219_BusV (sensor_handle *handle, uint16_t INAaddress)
{
		tempReg[0] = 0x02; 					//APUNTAR AL REGISTRO DE CONFIGURACION
    HAL_I2C_Master_Transmit(handle->i2c_port, INAaddress<<1, tempReg, 1, HAL_MAX_DELAY);
		
    raw_INA8[0] = 0x00;
		HAL_I2C_Master_Receive(handle->i2c_port, INAaddress<<1, raw_INA8, 2, HAL_MAX_DELAY);
    raw_INA = ((raw_INA8[0] << 8 | raw_INA8[1])>>3);
		
		return raw_INA*0.004;
}

float INA219_Power (sensor_handle *handle, uint16_t INAaddress)
{
		tempReg[0] = 0x03; 					//APUNTAR AL REGISTRO DE CONFIGURACION
    HAL_I2C_Master_Transmit(handle->i2c_port, INAaddress<<1, tempReg, 1, HAL_MAX_DELAY);
		
    raw_INA8[0] = 0x00;
		HAL_I2C_Master_Receive(handle->i2c_port, INAaddress<<1, raw_INA8, 2, HAL_MAX_DELAY);
    raw_INA = (raw_INA8[0] << 8 | raw_INA8[1]);
		
		return raw_INA/2;
}

float INA219_Current (sensor_handle *handle, uint16_t INAaddress)
{
		tempReg[0] = 0x04; 					//APUNTAR AL REGISTRO DE CONFIGURACION
    HAL_I2C_Master_Transmit(handle->i2c_port, INAaddress<<1, tempReg, 1, HAL_MAX_DELAY);
		
		int16_t rawCurrent;
    raw_INA8[0] = 0x00;
		HAL_I2C_Master_Receive(handle->i2c_port, INAaddress<<1, raw_INA8, 2, HAL_MAX_DELAY);
    rawCurrent = (raw_INA8[0] << 8 | raw_INA8[1]);
		
		return rawCurrent/10;
}



float TMPtemp(sensor_handle *handle)
{
		uint8_t raw_TMP8[2];
		HAL_I2C_Master_Transmit(handle->i2c_port, TMP102addr<<1, &TMP102_REGISTER_TEMPDATA , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, TMP102addr<<1, raw_TMP8, 2, HAL_MAX_DELAY);
		int16_t raw_TMP = (raw_TMP8[0]<<8 | raw_TMP8[1]);
		return (float)raw_TMP*0.0078125;
}
float bmpTemp(sensor_handle *handle)
{
		int32_t var1, var2;
		uint8_t adc_T8[3];
		//int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_TEMPDATA , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, adc_T8, 3, HAL_MAX_DELAY);
		uint32_t adc_T = adc_T8[0]<<16 | adc_T8[1]<<8 | adc_T8[2];
		
		adc_T >>= 4;

		var1 = ((((adc_T >> 3) - ((int32_t)bmp.dig_T1 << 1))) *
						((int32_t)bmp.dig_T2)) >>
					 11;

		var2 = (((((adc_T >> 4) - ((int32_t)bmp.dig_T1)) *
							((adc_T >> 4) - ((int32_t)bmp.dig_T1))) >>
						 12) *
						((int32_t)bmp.dig_T3)) >>
					 14;

		t_fine = var1 + var2;

		float T = (t_fine * 5 + 128) >> 8;
		return T/100;
}

float bmpPress(sensor_handle *handle)
{
		int64_t var1, var2, p;
		uint8_t adc_P8[3];

		// Must be done first to get the t_fine variable set up
		bmpTemp(handle);

		HAL_I2C_Master_Transmit(handle->i2c_port, bmp280addr<<1, &BMP280_REGISTER_PRESSUREDATA , 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(handle->i2c_port, bmp280addr<<1, adc_P8, 3, HAL_MAX_DELAY);
		uint32_t adc_P = adc_P8[0]<<16 | adc_P8[1]<<8 | adc_P8[2];
		adc_P >>= 4;

		var1 = ((int64_t)t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)bmp.dig_P6;
		var2 = var2 + ((var1 * (int64_t)bmp.dig_P5) << 17);
		var2 = var2 + (((int64_t)bmp.dig_P4) << 35);
		var1 = ((var1 * var1 * (int64_t)bmp.dig_P3) >> 8) +
					 ((var1 * (int64_t)bmp.dig_P2) << 12);
		var1 =
				(((((int64_t)1) << 47) + var1)) * ((int64_t)bmp.dig_P1) >> 33;

		if (var1 == 0) {
			return 0; // avoid exception caused by division by zero
		}
		p = 1048576 - adc_P;
		p = (((p << 31) - var2) * 3125) / var1;
		var1 = (((int64_t)bmp.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2 = (((int64_t)bmp.dig_P8) * p) >> 19;

		p = ((p + var1 + var2) >> 8) + (((int64_t)bmp.dig_P7) << 4);
		return (float)p / 256;
}

float bmpAlt(sensor_handle *handle, float nm_hPA)
{
		float altitude;

		float pressure = bmpPress(handle); // in Si units for Pascal
		pressure /= 100;

		altitude = 44330 * (1.0 - pow(pressure / nm_hPA, 0.1903));

		return altitude;
}

//******************************************************************
// MPU 9250
void IMUconfig(sensor_handle *handle)
{
		uint8_t temp[10];
		// select clock source to gyro
		if(IMUwriteRegister(handle, PWR_MGMNT_1, CLOCK_SEL_PLL) != HAL_OK){}
			//printf("*1\n");
		// enable I2C master mode
		if(IMUwriteRegister(handle, USER_CTRL, I2C_MST_EN) != HAL_OK){}
			//printf("*2\n");
		// set the I2C bus speed to 400 kHz	
		if(IMUwriteRegister(handle, I2C_MST_CTRL, I2C_MST_CLK) != HAL_OK){}
			//printf("*3\n");
		// set AK8963 to Power Down
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_PWR_DOWN) != HAL_OK){}
			//printf("*A4\n");
		// reset the MPU9250
		IMUwriteRegister(handle, PWR_MGMNT_1, PWR_RESET);
		//	printf("*5\n");
		osDelay(1);
		// reset the AK8963
		AK8963writeRegister(handle, AK8963_CNTL2, AK8963_RESET);
		//	printf("*A6?\n");
		// select clock source to gyro
		if(IMUwriteRegister(handle, PWR_MGMNT_1, CLOCK_SEL_PLL) != HAL_OK){}
			//printf("*7n");
		//printf("TODO OK");
		// Check Who Am I
		IMUreadRegister(handle, WHO_AM_I, temp,1);
		//printf("Who am I IMU= 0x%.2X\n", temp[0]);
		
		// enable accelerometer and gyro
		if(IMUwriteRegister(handle, PWR_MGMNT_2, SEN_ENABLE) != HAL_OK){}
			//printf("*8\n");
		// setting accel range to 16G as default
		if(IMUwriteRegister(handle, ACCEL_CONFIG, ACCEL_FS_SEL_16G) != HAL_OK){}
			//printf("*9\n");
		_accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
		_accelRange = ACCEL_RANGE_16G;
		// setting the gyro range to 2000DPS as default
		if(IMUwriteRegister(handle, GYRO_CONFIG, GYRO_FS_SEL_2000DPS) != HAL_OK){}
			//printf("*10\n");
		_gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
		_gyroRange = GYRO_RANGE_2000DPS;
		// setting bandwidth to 184Hz as default
		if(IMUwriteRegister(handle, ACCEL_CONFIG2, ACCEL_DLPF_184) != HAL_OK){}
			//printf("*11\n");
		if(IMUwriteRegister(handle, CONFIG, GYRO_DLPF_184) != HAL_OK){}
			//printf("*12\n");
		_bandwidth = DLPF_BANDWIDTH_184HZ;
		// setting the sample rate divider to 0 as default
		if(IMUwriteRegister(handle, SMPDIV, 0x00) != HAL_OK){}
			//printf("*13\n");
		_srd = 0;
		// enable I2C master mode
		if(IMUwriteRegister(handle, USER_CTRL, I2C_MST_EN) != HAL_OK){}
			//printf("*14\n");
		// set the I2C bus speed to 400 kHz
		if(IMUwriteRegister(handle, I2C_MST_CTRL, I2C_MST_CLK) != HAL_OK){}
			//printf("*15\n");
		// check AK8963 WHO AM I register, expected value is 0x48
		if(IMUreadRegister(handle, AK8963_WHO_AM_I, temp,1) != HAL_OK){}
			//printf("*16\n");
		//printf("Who am I AK8963= 0x%.2X\n", temp[0]);
		
		/* get the magnetometer calibration */
  // set AK8963 to Power Down
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_PWR_DOWN) != HAL_OK){}
			//printf("*A17\n");
		osDelay(100);
		// set AK8963 to FUSE ROM access
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_FUSE_ROM) != HAL_OK){}
			//printf("*A18\n");
		osDelay(100);
		// read the AK8963 ASA registers and compute magnetometer scale factors
		if(AK8963readRegister(handle, AK8963_ASA,temp,3) != HAL_OK){}
			//printf("*A19\n");
		_magScaleX = ((((float)temp[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
		_magScaleY = ((((float)temp[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
		_magScaleZ = ((((float)temp[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
		// set AK8963 to Power Down
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_PWR_DOWN) != HAL_OK){}
			////printf("*A20\n");
		osDelay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_CNT_MEAS2) != HAL_OK){}
			////printf("*A21\n");
		osDelay(100);
		// select clock source to gyro
		if(IMUwriteRegister(handle, PWR_MGMNT_1, CLOCK_SEL_PLL) != HAL_OK){}
			////printf("*22\n");
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		AK8963readRegister(handle, AK8963_HXL,temp,7);
		// calculate Gyro bias
		calGyro(handle);
		
}

//******************************************************************
//******************************************************************
HAL_StatusTypeDef IMUwriteRegister(sensor_handle *handle, uint8_t reg, uint8_t data)
{
		uint8_t temp[3];
		temp[0] = reg;
		temp[1] = data;
		if(HAL_I2C_Master_Transmit(handle->i2c_port, IMUaddr<<1, temp , 2, HAL_MAX_DELAY) != HAL_OK )
			return HAL_ERROR;
		if(IMUreadRegister(handle, reg, temp,1) != HAL_OK)
			return HAL_ERROR;
		if(temp[0] == data)
			return HAL_OK;
		else
			return HAL_ERROR;	
}

HAL_StatusTypeDef IMUreadRegister(sensor_handle *handle, uint8_t reg, uint8_t *data, uint8_t size)
{

		HAL_I2C_Master_Transmit(handle->i2c_port, IMUaddr<<1, &reg , 1, HAL_MAX_DELAY);
		return HAL_I2C_Master_Receive(handle->i2c_port, IMUaddr<<1, data, size, HAL_MAX_DELAY);
}

int AK8963writeRegister(sensor_handle *handle, uint8_t reg, uint8_t data)
{
		uint8_t temp;
		if(IMUwriteRegister(handle, I2C_SLV0_ADDR, AK8963_I2C_ADDR) != HAL_OK)
			return HAL_ERROR;
		if(IMUwriteRegister(handle, I2C_SLV0_REG, reg) != HAL_OK)
			return HAL_ERROR;
		if(IMUwriteRegister(handle, I2C_SLV0_DO, data) != HAL_OK)
			return HAL_ERROR;
		if(IMUwriteRegister(handle, I2C_SLV0_CTRL, I2C_SLV0_EN) != HAL_OK)
			return HAL_ERROR;
		
		if(AK8963readRegister(handle, reg, &temp, 1) != HAL_OK )
			return HAL_ERROR;
		if(temp == data)
			return HAL_OK;
		else
			return HAL_ERROR;
}
		
HAL_StatusTypeDef AK8963readRegister(sensor_handle *handle, uint8_t reg, uint8_t *data, uint8_t size)
{
		if(IMUwriteRegister(handle, I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG) != HAL_OK)
			return HAL_ERROR;
		if(IMUwriteRegister(handle, I2C_SLV0_REG, reg) != HAL_OK)
			return HAL_ERROR;
		if(IMUwriteRegister(handle, I2C_SLV0_CTRL, I2C_SLV0_EN |size) != HAL_OK)
			return HAL_ERROR;
		osDelay(1);
		HAL_I2C_Master_Transmit(handle->i2c_port, IMUaddr<<1, &EXT_SENS_DATA_00 , 1, HAL_MAX_DELAY);
		return HAL_I2C_Master_Receive(handle->i2c_port, IMUaddr<<1, data, size, HAL_MAX_DELAY);
}

void readIMU(sensor_handle *handle)
{
		// read RAW values
		IMUreadRegister(handle, ACCEL_OUT, IMUbuffer, 20);
		_axcounts = (((int16_t)IMUbuffer[0]) << 8) | IMUbuffer[1];  
		_aycounts = (((int16_t)IMUbuffer[2]) << 8) | IMUbuffer[3];
		_azcounts = (((int16_t)IMUbuffer[4]) << 8) | IMUbuffer[5];
		//printf(" %d,%d,%d,",_axcounts, _aycounts,_azcounts);
		_gxcounts = (((int16_t)IMUbuffer[8]) << 8) | IMUbuffer[9];
		_gycounts = (((int16_t)IMUbuffer[10]) << 8) | IMUbuffer[11];
		_gzcounts = (((int16_t)IMUbuffer[12]) << 8) | IMUbuffer[13];
		//////printf(" %d,%d,%d\n",_gxcounts, _gycounts,_gzcounts);
		_hxcounts = (((int16_t)IMUbuffer[15]) << 8) | IMUbuffer[14];
		_hycounts = (((int16_t)IMUbuffer[17]) << 8) | IMUbuffer[16];
		_hzcounts = (((int16_t)IMUbuffer[19]) << 8) | IMUbuffer[18];
		//////printf(" %d,%d,%d\n",_hxcounts, _hycounts,_hzcounts);
		_tcounts = (((int16_t)IMUbuffer[6]) << 8) | IMUbuffer[7];
	
		//Convert to float values
		_ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
		_ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
		_az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
		_gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
		_gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
		_gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
		_hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
		_hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
		_hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
		_t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
}

void calGyro(sensor_handle *handle)
{
		uint8_t temp[10];
		// setting the gyro range to 250DPS
		if(IMUwriteRegister(handle, GYRO_CONFIG, GYRO_FS_SEL_250DPS) != HAL_OK){}
			////printf("*calG_1\n");
		// setting bandwidth to 20Hz
		if(IMUwriteRegister(handle, ACCEL_CONFIG2, ACCEL_DLPF_20) != HAL_OK){} // setting accel bandwidth to 20Hz
			////printf("*calG_2\n");
		if(IMUwriteRegister(handle, CONFIG, GYRO_DLPF_20) != HAL_OK) {}// setting gyro bandwidth to 20Hz
			////printf("*calG_3\n");
		// set SRD
		if(IMUwriteRegister(handle, SMPDIV, 19) != HAL_OK){} // setting gyro bandwidth to 20Hz
			////printf("*calG_4\n");
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_PWR_DOWN) != HAL_OK) {}// setting gyro bandwidth to 20Hz
			////printf("*calG_5\n");
		osDelay(100);
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_CNT_MEAS1) != HAL_OK){} // setting gyro bandwidth to 20Hz
			//printf("*calG_5\n");
		osDelay(100);
		AK8963readRegister(handle, AK8963_HXL,temp,7);
		// calculate bias
		double _gxbD = 0;
		double _gybD = 0;
		double _gzbD = 0;
		for(size_t i=0; i < _numSamples; i++)
		{
				readIMU(handle);
				_gxbD += (_gx + _gxb)/((double)_numSamples);
				_gybD += (_gy + _gyb)/((double)_numSamples);
				_gzbD += (_gz + _gzb)/((double)_numSamples);
				osDelay(20);
		}
		_gxb = (float)_gxbD;
		_gyb = (float)_gybD;
		_gzb = (float)_gzbD;
		
		// return the gyro range to 2000DPS as default
		if(IMUwriteRegister(handle, GYRO_CONFIG, GYRO_FS_SEL_2000DPS) != HAL_OK){}
			//printf("*calG_6\n");
		// return bandwidth to 184Hz as default
		if(IMUwriteRegister(handle, ACCEL_CONFIG2, ACCEL_DLPF_184) != HAL_OK){}
			//printf("*calG_7\n");
		if(IMUwriteRegister(handle, CONFIG, GYRO_DLPF_184) != HAL_OK){}{}
			//printf("*calG_8\n");
		// return SRD
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_PWR_DOWN) != HAL_OK){}
			//printf("*calG_9\n");
		osDelay(100);
		if(AK8963writeRegister(handle, AK8963_CNTL1, AK8963_CNT_MEAS2) != HAL_OK){}
			//printf("*calG_10\n");
		osDelay(100);
		AK8963readRegister(handle, AK8963_HXL,temp,7);
		if(IMUwriteRegister(handle, SMPDIV, 0x00) != HAL_OK){}
			//printf("*calG_11\n");
		
}

float IMUread_Temperatura(sensor_handle *handle)
{
		return _t;
}
float IMUread_AcelX(sensor_handle *handle)
{
		return _ax;
}
float IMUread_AcelY(sensor_handle *handle)
{
		return _ay;
}
float IMUread_AcelZ(sensor_handle *handle)
{
		return _az;
}
float IMUread_GyrX(sensor_handle *handle)
{
		return _gx;
}
float IMUread_GyrY(sensor_handle *handle)
{
		return _gy;
}
float IMUread_GyrZ(sensor_handle *handle)
{
		return _gz;
}
float IMUread_MagX(sensor_handle *handle)
{
		return _hx;
}
float IMUread_MagY(sensor_handle *handle)
{
		return _hy;
}
float IMUread_MagZ(sensor_handle *handle)
{
		return _hz;
}

double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}


void	GPS_Process(GPS_t *GPS, uint8_t *rxBuffer)
{

		char	*str;
		#if (_GPS_DEBUG==1)
		//printf("%s",GPS.rxBuffer);
		#endif
		//printf("*%s*\n",rxBuffer);
		//memset(dest, '\0', sizeof(dest));
		//strcpy(dest,"$GNGGA,123519.542,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n");
		str=strstr((char*)rxBuffer,"$GNGGA,");
		//str=strstr((char*)dest,"$GNGGA,");

		//printf("%s\n\n",str);
		if(str!=NULL)
		{
			memset(&GPS->GNGGA,0,sizeof(GPS->GNGGA));
			
			sscanf(str,"$GNGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%2hhd,%f,%f,%c,%hd,%s,*%2s\r\n",
													&GPS->GNGGA.UTC_Hour,
													&GPS->GNGGA.UTC_Min,
													&GPS->GNGGA.UTC_Sec,
													&GPS->GNGGA.UTC_MicroSec,
													&GPS->GNGGA.Latitude,
													&GPS->GNGGA.NS_Indicator,
													&GPS->GNGGA.Longitude,
													&GPS->GNGGA.EW_Indicator,
													&GPS->GNGGA.PositionFixIndicator,
													&GPS->GNGGA.SatellitesUsed,
													&GPS->GNGGA.HDOP,
													&GPS->GNGGA.MSL_Altitude,
													&GPS->GNGGA.MSL_Units,
													&GPS->GNGGA.AgeofDiffCorr,
													GPS->GNGGA.DiffRefStationID,
													GPS->GNGGA.CheckSum);
			if(GPS->GNGGA.NS_Indicator==0)
				GPS->GNGGA.NS_Indicator='-';
			if(GPS->GNGGA.EW_Indicator==0)
				GPS->GNGGA.EW_Indicator='-';
			if(GPS->GNGGA.Geoid_Units==0)
				GPS->GNGGA.Geoid_Units='-';
			if(GPS->GNGGA.MSL_Units==0)
				GPS->GNGGA.MSL_Units='-';
			GPS->GNGGA.LatitudeDecimal=convertDegMinToDecDeg(GPS->GNGGA.Latitude);
			if(GPS->GNGGA.NS_Indicator == 'S') {
					GPS->GNGGA.LatitudeDecimal = -GPS->GNGGA.LatitudeDecimal;
			}
			
			GPS->GNGGA.LongitudeDecimal=convertDegMinToDecDeg(GPS->GNGGA.Longitude);
			if(GPS->GNGGA.EW_Indicator == 'W') {
					GPS->GNGGA.LongitudeDecimal = -GPS->GNGGA.LongitudeDecimal;
			}
		}
		//printf("LN = %f\n",GPS->GNGGA.LongitudeDecimal);

		
		//memset(rxBuffer,0,sizeof((uint8_t *)rxBuffer));
		memset(rxBuffer,0,10);

}
//******************************************************************
