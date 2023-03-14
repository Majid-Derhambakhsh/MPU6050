![Banner](Banner.png)

# MPU6050
"Easy To Use" MPU6050 library for using in AVR - ARM Cortex M

### Version : 2.0.0

- #### Type : Embedded Software.

- #### Support : All C/C++ compiler.

- #### Program Language : C

- #### Properties :

### Initialization and de-initialization functions:
```c++
MPU_StatusTypeDef MPU6050_Init(MPU_TypeDef *MPUx, uint16_t Timeout);
MPU_StatusTypeDef MPU6050_AutoInit(MPU_TypeDef *MPUx, uint16_t Timeout);
MPU_StatusTypeDef MPU6050_DefInit(MPU_TypeDef *MPUx, uint16_t Timeout);
```  

### Operation functions:
```c++  
/* ............................ Check .......................... */
MPU_StatusTypeDef MPU6050_IsReady(MPU_TypeDef *MPUx, uint8_t Trials, uint16_t Timeout);
MPU_StatusTypeDef MPU6050_Reset(MPU_TypeDef *MPUx, uint16_t Timeout);

/* ....................... Configuration ....................... */
MPU_StatusTypeDef MPU6050_SetDeviceID(MPU_TypeDef *MPUx, uint8_t ID, uint16_t Timeout);
MPU_StatusTypeDef MPU6050_GetDeviceID(MPU_TypeDef *MPUx, uint8_t *ID, uint16_t Timeout);

/* ....................... Get Raw Value ....................... */
MPU_StatusTypeDef MPU6050_GetRawAccel(MPU_TypeDef *MPUx, MPU_RawTypeDef *AccelRaw, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_GetRawGyro(MPU_TypeDef *MPUx, MPU_RawTypeDef *GyroRaw, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_GetRawTemp(MPU_TypeDef *MPUx, int16_t *Temp, uint16_t Timeout);

/* ......................... Get Value ......................... */
MPU_StatusTypeDef MPU6050_GetAccel(MPU_TypeDef *MPUx, MPU_XYZTypeDef *Accel, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_GetGyro(MPU_TypeDef *MPUx, MPU_XYZTypeDef *Gyro, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_GetTemp(MPU_TypeDef *MPUx, float *Temp, uint16_t Timeout);

/* ................. Angle with accelerometer .................. */
MPU_StatusTypeDef MPU6050_GetRoll(MPU_TypeDef *MPUx, float *Roll, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_GetPitch(MPU_TypeDef *MPUx, float *Pitch, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_GetYaw(MPU_TypeDef *MPUx, float *Yaw, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_GetRPY(MPU_TypeDef *MPUx, MPU_RPYTypeDef *RPY, uint16_t Timeout);

``` 
### Macros:
```c++  
- None 
``` 

## How to use this library

### The MPU6050 library can be used as follows:
#### 1.  Add .h and source file in project.      
#### 2.  Config i2c in 'stm32_i2c_conf.h' for STM32 or 'i2c_unit_conf.h' for AVR header, for example:  
##### AVR Example:  
  ```c++  
  /* -------------------- Define -------------------- */

  #define _F_SCL      100000UL 
  #define _PRESCALER  _PRE1 

  /*
	  Guide :
			  _F_SCL	   : Specifies the clock frequency.
						  This parameter must be set to a value lower than 400kHz
			
			  _PRESCALER : i2c prescaler value argument is
						  _PRE1 , _PRE4 , _PRE16 , _PRE64
  */

  /* ------------------------------------------------ */
  ``` 
  
#### 3.  Config Chipset in 'mpu6050_conf.h' header, for example:  
   * Options:  
   
      ```c++
      /* ~~~~~~~~~~~~~~ Required Headers ~~~~~~~~~~~~~ */
      #include "MATH_EX/math_ex.h" // Don't remove this line

      /* Driver-library for AVR */
      //#include "I2C_UNIT/i2c_unit.h"

      /* Driver-library for STM32 */
      #include "STM32_I2C/stm32_i2c.h"

      /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
      
      ```
          
          
#### 4.  Using initialize methods for initialize hardware and chipset, for example:  
```c++  
MPU_TypeDef IMU1;

IMU1.I2Cx = &hi2c1; // For STM32
IMU1.Address = MPU_ADD_LOW;

MPU6050_AutoInit(&IMU1, 100);
```  
#### 5.  Using operation methods, for example:  
#### Example 1:  
```c++  
int main(void)
{
	I2C_Init();
	
	/* ---------- MPU6050 Setup --------- */
	MPU_TypeDef    IMU1;
	MPU_XYZTypeDef AccData;
	MPU_XYZTypeDef GyroData;
	
	IMU1.I2Cx                       = &hi2c1;
	IMU1.Address                    = MPU_ADD_LOW;
	IMU1.SampleRateDivider          = MPU_CLOCK_DIVIDER_8;
	IMU1.DigitalLowPassFilter       = MPU_DLPF_CFG_5_HZ;
	IMU1.InterruptEnable            = MPU_INT_DATA_RDY_EN;
	IMU1.ExtSync                    = MPU_ES_INPUT_DISABLE;
	IMU1.InterruptConfig.IntOpen    = MPU_INT_OPEN_PUSH_PULL;
	IMU1.InterruptConfig.IntLevel   = MPU_INT_LEVEL_ACTIVE_HIGH;
	IMU1.InterruptConfig.LatchIntEn = MPU_LATCH_INT_EN_50US_PULSE;
	IMU1.GyroFullScaleRange         = MPU_GYRO_FULL_SCALE_RANGE_2000;
	IMU1.AccelFullScaleRange        = MPU_ACCEL_FULL_SCALE_RANGE_16G;
	IMU1.ClockSelection             = MPU_CLKSEL_X_AXIS_GYROSCOPE_REFERENCE; 
	
	MPU6050_Init(&IMU1, 100);
	
	/* Device Check */
	if (MPU6050_IsReady(&IMU1, 10, 100) == MPU_OK)
	{
		HAL_UART_Transmit(&huart6, (uint8_t *)"Is Ready\r\n", strlen("Is Ready\r\n"), 100);
	}
	else
	{
		HAL_UART_Transmit(&huart6, (uint8_t *)"Not Ready\r\n", strlen("Not Ready\r\n"), 100);
	}
	
	while (1) 
	{
		/* :::::::::: Read Sensor Data :::::::::: */
		MPU6050_GetAccel(&IMU1, &AccData, 100);
		MPU6050_GetGyro(&IMU1, &GyroData, 100);
		HAL_Delay(100);
	
		sprintf(msg ,"AX:%f,AY:%f,AZ:%f,GX:%f,GY:%f,GZ:%f\r\n", AccData.X, AccData.Y, AccData.Z, GyroData.X, GyroData.Y, GyroData.Z);
		HAL_UART_Transmit(&huart6, (uint8_t *)msg, strlen(msg), 100);
	
	}
}
   
``` 

## Supported Chipset:
- [x] MPU60X0 Series  

#### Developer: Majid Derhambakhsh

