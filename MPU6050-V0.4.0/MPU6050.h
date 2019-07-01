/*
------------------------------------------------------------------------------
~ File   : MPU6050.h
~ Author : Majid Derhambakhsh
~ Version: V0.4.0
~ Created: 06/12/2019 07:38:00 PM
~ Brief  :
~ Support: Majid.do16@gmail.com
------------------------------------------------------------------------------
~ Description:

~ Attention  :    This file is for AVR/ARM microcontroller
------------------------------------------------------------------------------
*/

#ifndef __MPU6050_H_
#define __MPU6050_H_

/******************************************* Include *******************************************/

/*----------------------------------------------------------*/

#ifdef __CODEVISIONAVR__  /* Check compiler */

#include <io.h>            /* Import AVR IO library */
#include "I2C_UNIT/I2C_UNIT.h" /* Import I2C library */

/*----------------------------------------------------------*/

#elif defined(__GNUC__)   /* Check compiler */

#include <avr/io.h>        /* Import AVR IO library */
#include "I2C_UNIT/I2C_UNIT.h" /* Import I2C library */

/*----------------------------------------------------------*/

#else                     /* Compiler not found */

#error Compiler not supported  /* Send error */

#endif /* __CODEVISIONAVR__ */

#include "MPU6050_CONFIG.h" /* Import config file */
#include "MATH/MATH.h" /* Import math lib */
#include <stdint.h> /* Import standard integer */ 

/******************************************* Defines *******************************************/

/* ---------------------- Type size ---------------------- */

#ifndef ENUM_U8_T
 
 #define ENUM_U8_T(ENUM_NAME)   Enum_##ENUM_NAME; typedef uint8_t ENUM_NAME /* Config enum size */

#endif

/* ----------------------- MPU60X0 ----------------------- */

/* ---------------- Device Address ---------------- */

#define _MPU6050_ADDRESS_W 0xD0 /* MPU6050 address for write */
#define _MPU6050_ADDRESS_R 0xD1 /* MPU6050 address for read */

/* ---------------- AD0 Pin State ----------------- */

#define _AD0_HIGH 0x02 /* AD0 Bit value in the high state */
#define _AD0_LOW  0x00 /* AD0 Bit value in the low state */

/* -------------------- Temp ---------------------- */

#define _TEMP_CONST 36.53f /* Temp constant value */
#define _TEMP_DIVIDER 340.00f /* Divider for calculate temp */
#define _TEMP_REGISTERS 2 /* Temp registers */

/* -------------------- Accel --------------------- */

#define _ACCEL_SENSITIVITY_2G  16384.0f /* Divider for calculate accelerometer value */
#define _ACCEL_SENSITIVITY_4G  8192.0f  /* Divider for calculate accelerometer value */
#define _ACCEL_SENSITIVITY_8G  4096.0f  /* Divider for calculate accelerometer value */
#define _ACCEL_SENSITIVITY_16G 2048.0f  /* Divider for calculate accelerometer value */

/* -------------------- Gyro --------------------- */

#define _GYRO_SENSITIVITY_250  131.0f /* Divider for calculate gyroscope value */
#define _GYRO_SENSITIVITY_500  65.5f  /* Divider for calculate gyroscope value */
#define _GYRO_SENSITIVITY_1000 32.8f  /* Divider for calculate gyroscope value */
#define _GYRO_SENSITIVITY_2000 16.4f  /* Divider for calculate gyroscope value */

/* -------------------- Angle --------------------- */

#define _MINIMUM_VALUE   265 /* Minimum value for map */
#define _MAXIMUM_VALUE   402 /* Maximum value for map */
#define _POSITIVE_ANGLE  90U /* Maximum positive value */
#define _NEGATIVE_ANGLE  -90 /* Maximum negative value */

/* -------------------- Public -------------------- */

#define _TRUE                    1 /* True value */
#define _FALSE                   0 /* False value */
#define _SINGLE_WRITE_STEPS      4 /* Quantity of single write steps */
#define _BURST_WRITE_STEPS       3 /* Quantity of burst write steps */
#define _SINGLE_READ_STEPS       6 /* Quantity of single read steps */
#define _BURST_READ_STEPS        5 /* Quantity of burst read steps */
#define _INITIALIZE_INSTRUCTIONS 7 /* Quantity of initialize instructions */
#define _HIGH_BYTE               8 /* High byte shift value */
#define _DIVIDER_8               8 /* Divider */
#define _BIT_3                   3 /* 3 Bit */
#define _BIT_5                   5 /* 5 Bit */
#define _BIT_6                   6 /* 6 Bit */
#define _BIT_7                   7 /* 7 Bit */
#define _ALL_AXIS                3 /* Axis */
#define _ONE_AXIS_REGISTERS      2 /* One Axis Registers */
#define _ALL_AXIS_REGISTERS      6 /* All Axis Registers */
#define _RAD_TO_DEG              57.2957786f /* Value for convert */
#define _PI                      3.1415926535897932384626433832795f /* pi value */

/******************************************** Enum *********************************************/

typedef enum /* Enum for digital low pass filter config */
{
	
	_DLPF_CFG_0 = 0, /* Config to 0 */
	_DLPF_CFG_1 = 1, /* Config to 1 */
	_DLPF_CFG_2 = 2, /* Config to 2 */
	_DLPF_CFG_3 = 3, /* Config to 3 */
	_DLPF_CFG_4 = 4, /* Config to 4 */
	_DLPF_CFG_5 = 5, /* Config to 5 */
	_DLPF_CFG_6 = 6, /* Config to 6 */
	
	/* _DLPF_CFG_7 Is Reserved */
	
}ENUM_U8_T(MPU6050_DLPF_CFG_t);

typedef enum /* Enum for external sync config */
{
	
	_ES_INPUT_DISABLE = 0, /* Configures the FSYNC pin sampling. */
	_ES_TEMP_OUT_L    = 1, /* Configures the FSYNC pin sampling. */
	_ES_GYRO_XOUT_L   = 2, /* Configures the FSYNC pin sampling. */
	_ES_GYRO_YOUT_L   = 3, /* Configures the FSYNC pin sampling. */
	_ES_GYRO_ZOUT_L   = 4, /* Configures the FSYNC pin sampling. */
	_ES_ACCEL_XOUT_L  = 5, /* Configures the FSYNC pin sampling. */
	_ES_ACCEL_YOUT_L  = 6, /* Configures the FSYNC pin sampling. */
	_ES_ACCEL_ZOUT_L  = 7  /* Configures the FSYNC pin sampling. */
	
}ENUM_U8_T(MPU6050_EXT_SYNC_SET_t);

typedef enum /* Enum for gyro full scale range */
{
	
	_GYRO_FULL_SCALE_RANGE_250  = 0, /* Full scale range +/- 250 degree/S  */
	_GYRO_FULL_SCALE_RANGE_500  = 1, /* Full scale range +/- 500 degree/S  */
	_GYRO_FULL_SCALE_RANGE_1000 = 2, /* Full scale range +/- 1000 degree/S */
	_GYRO_FULL_SCALE_RANGE_2000 = 3  /* Full scale range +/- 2000 degree/S */
	
}ENUM_U8_T(MPU6050_FS_SEL_t);

typedef enum /* Enum for accel full scale range */
{
	
	_ACCEL_FULL_SCALE_RANGE_2G  = 0, /* Full scale range +/- 2g  */
	_ACCEL_FULL_SCALE_RANGE_4G  = 1, /* Full scale range +/- 4g  */
	_ACCEL_FULL_SCALE_RANGE_8G  = 2, /* Full scale range +/- 8g  */
	_ACCEL_FULL_SCALE_RANGE_16G = 3  /* Full scale range +/- 16g */
	
}ENUM_U8_T(MPU6050_AFS_SEL_t);

typedef enum /* Enum for interrupt config */
{
	
	_INT_LEVEL_ACTIVE_HIGH          = 0, /* the logic level for the INT pin is active high. */
	_INT_LEVEL_ACTIVE_LOW           = 1, /* the logic level for the INT pin is active low. */
	_INT_OPEN_PUSH_PULL             = 0, /* the INT pin is configured as push-pull. */
	_INT_OPEN_OPEN_DRAIN            = 1, /* the INT pin is configured as open drain. */
	_LATCH_INT_EN_50US_PULSE        = 0, /* the INT pin emits a 50us long pulse. */
	_LATCH_INT_EN_INTERRUPT_CLEARED = 1  /* the INT pin is held high until the interrupt is cleared. */
	
}ENUM_U8_T(MPU6050_INT_CONFIG_t);

typedef enum /* Enum for interrupt enable */
{
	_INT_DISABLE        = 0x00, /* Disable Interrupt */
	_INT_DATA_RDY_EN    = 0x01, /* this bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed. */
	_INT_I2C_MST_INT_EN = 0x08, /* this bit enables any of the I2C Master interrupt sources to generate an interrupt. */
	_INT_FIFO_OFLOW_EN  = 0x10  /* this bit enables a FIFO buffer overflow to generate an interrupt. */
	
}ENUM_U8_T(MPU6050_INT_ENABLE_t);

typedef enum /* Enum for Clock select value */
{
	
	_CLKSEL_INTERNAL_8MHZ_OSCILLATOR    = 0, /* Internal 8MHz oscillator              */
	_CLKSEL_X_AXIS_GYROSCOPE_REFERENCE  = 1, /* PLL with X axis gyroscope reference   */
	_CLKSEL_Y_AXIS_GYROSCOPE_REFERENCE  = 2, /* PLL with Y axis gyroscope reference   */
	_CLKSEL_Z_AXIS_GYROSCOPE_REFERENCE  = 3, /* PLL with Z axis gyroscope reference   */
	_CLKSEL_EXTERNAL_32768HZ_REFERENCE  = 4, /* PLL with external 32.768kHz reference */
	_CLKSEL_EXTERNAL_19200KHZ_REFERENCE = 5, /* PLL with external 19.2MHz reference   */
	_CLKSEL_STOP                        = 7  /* Stops the clock and keeps the timing generator in reset */ 
	
}ENUM_U8_T(MPU6050_CLKSEL_t);

typedef enum /* Enum for Register address */
{
	
	_REG_SELF_TEST_X        = 0x0D, /* Serial I-F = RW */
	_REG_SELF_TEST_Y        = 0x0E, /* Serial I-F = RW */
	_REG_SELF_TEST_Z        = 0x0F, /* Serial I-F = RW */
	_REG_SELF_TEST_A        = 0x10, /* Serial I-F = RW */
	_REG_SMPLRT_DIV         = 0x19, /* Serial I-F = RW */
	_REG_CONFIG             = 0x1A, /* Serial I-F = RW */
	_REG_GYRO_CONFIG        = 0x1B, /* Serial I-F = RW */
	_REG_ACCEL_CONFIG       = 0x1C, /* Serial I-F = RW */
	_REG_FIFO_EN            = 0x23, /* Serial I-F = RW */
	_RES_I2C_MST_CTRL       = 0x24, /* Serial I-F = RW */
	_REG_I2C_SLV0_ADDR      = 0x25, /* Serial I-F = RW */
	_REG_I2C_SLV0_REG       = 0x26, /* Serial I-F = RW */
	_REG_I2C_SLV0_CTRL      = 0x27, /* Serial I-F = RW */
	_REG_I2C_SLV1_ADDR      = 0x28, /* Serial I-F = RW */
	_REG_I2C_SLV1_REG       = 0x29, /* Serial I-F = RW */
	_REG_I2C_SLV1_CTRL      = 0x2A, /* Serial I-F = RW */
	_REG_I2C_SLV2_ADDR      = 0x2B, /* Serial I-F = RW */
	_REG_I2C_SLV2_REG       = 0x2C, /* Serial I-F = RW */
	_REG_I2C_SLV2_CTRL      = 0x2D, /* Serial I-F = RW */
	_REG_I2C_SLV3_ADDR      = 0x2E, /* Serial I-F = RW */
	_REG_I2C_SLV3_REG       = 0x2F, /* Serial I-F = RW */
	_REG_I2C_SLV3_CTRL      = 0x30, /* Serial I-F = RW */
	_REG_I2C_SLV4_ADDR      = 0x31, /* Serial I-F = RW */
	_REG_I2C_SLV4_REG       = 0x32, /* Serial I-F = RW */
	_REG_I2C_SLV4_DO        = 0x33, /* Serial I-F = RW */
	_REG_I2C_SLV4_CTRL      = 0x34, /* Serial I-F = RW */
	_REG_I2C_SLV4_DI        = 0x35, /* Serial I-F = R  */
	_REG_I2C_MST_STATUS     = 0x36, /* Serial I-F = R  */
	_REG_INT_PIN_CFG        = 0x37, /* Serial I-F = RW */
	_REG_INT_ENABLE         = 0x38, /* Serial I-F = RW */
	_REG_INT_STATUS         = 0x3A, /* Serial I-F = R  */
    _REG_ACCEL_XOUT_H       = 0x3B, /* Serial I-F = R  */
	_REG_ACCEL_XOUT_L       = 0x3C, /* Serial I-F = R  */
	_REG_ACCEL_YOUT_H       = 0x3D, /* Serial I-F = R  */
    _REG_ACCEL_YOUT_L       = 0x3E, /* Serial I-F = R  */
    _REG_ACCEL_ZOUT_H       = 0x3F, /* Serial I-F = R  */
	_REG_ACCEL_ZOUT_L       = 0x40, /* Serial I-F = R  */
	_REG_TEMP_OUT_H         = 0x41, /* Serial I-F = R  */
	_REG_TEMP_OUT_L         = 0x42, /* Serial I-F = R  */
	_REG_GYRO_XOUT_H        = 0x43, /* Serial I-F = R  */
	_REG_GYRO_XOUT_L        = 0x44, /* Serial I-F = R  */
	_REG_GYRO_YOUT_H        = 0x45, /* Serial I-F = R  */
	_REG_GYRO_YOUT_L        = 0x46, /* Serial I-F = R  */
	_REG_GYRO_ZOUT_H        = 0x47, /* Serial I-F = R  */
	_REG_GYRO_ZOUT_L        = 0x48, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_00   = 0x49, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_01   = 0x4A, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_02   = 0x4B, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_03   = 0x4C, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_04   = 0x4D, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_05   = 0x4E, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_06   = 0x4F, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_07   = 0x50, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_08   = 0x51, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_09   = 0x52, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_10   = 0x53, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_11   = 0x54, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_12   = 0x55, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_13   = 0x56, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_14   = 0x57, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_15   = 0x58, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_16   = 0x59, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_17   = 0x5A, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_18   = 0x5B, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_19   = 0x5C, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_20   = 0x5D, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_21   = 0x5E, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_22   = 0x5F, /* Serial I-F = R  */
	_REG_EXT_SENS_DATA_23   = 0x60, /* Serial I-F = R  */
	_REG_I2C_SLV0_DO        = 0x63, /* Serial I-F = RW */
	_REG_I2C_SLV1_DO        = 0x64, /* Serial I-F = RW */
	_REG_I2C_SLV2_DO        = 0x65, /* Serial I-F = RW */
	_REG_I2C_SLV3_DO        = 0x66, /* Serial I-F = RW */
	_REG_I2C_MST_DELAY_CTRL = 0x67, /* Serial I-F = RW */
	_REG_SIGNAL_PATH_RESET  = 0x68, /* Serial I-F = RW */
	_REG_USER_CTRL          = 0x6A, /* Serial I-F = RW */
	_REG_PWR_MGMT_1         = 0x6B, /* Serial I-F = RW */
	_REG_PWR_MGMT_2         = 0x6C, /* Serial I-F = RW */
	_REG_FIFO_COUNTH        = 0x72, /* Serial I-F = RW */
	_REG_FIFO_COUNTL        = 0x73, /* Serial I-F = RW */
	_REG_FIFO_R_W           = 0x74, /* Serial I-F = RW */
	_REG_WHO_AM_I           = 0x75  /* Serial I-F = R  */
	
}ENUM_U8_T(MPU6050_REGISTER_ADDRESS_t);

/******************************************* Struct ********************************************/

typedef struct /* Struct for config interrupt in ic */
{
	
	uint8_t IntLevel   : 1; /* this bit set the logic level for the INT pin */
	uint8_t IntOpen    : 1; /* this bit set the INT pin mode (Push pull / Open drain) */
	uint8_t LatchIntEn : 1; /* this bit set the INT pin pulse mode */
	
}MPU6050_INT_CONFIG_OPTION_t;

typedef struct /* Struct for config ic */
{
	
	uint8_t SampleRateDivider; /* The Sample Rate is determined by dividing the gyroscope output rate by this value. */
	uint8_t ExtSync;  /* Configures the FSYNC pin sampling. */
	uint8_t DigitalLowPassFilter; /* Configures the DLPF setting. */
	uint8_t GyroFullScaleRange; /* Selects the full scale range of gyroscopes. */
	uint8_t AccelFullScaleRange; /* Selects the full scale range of accelerometers. */
	MPU6050_INT_CONFIG_OPTION_t InterruptConfig; /* Config interrupt option */
	uint8_t InterruptEnable; /* Interrupt enable */
	uint8_t ClockSelection; /* Set clock source */
	
}Mpu6050_Config_t;

extern Mpu6050_Config_t Mpu6050_Config; /* Struct for config ic */

/****************************************** Prototype ******************************************/

uint8_t Mpu6050_ConnectionTest(void); /* Function for check connection */
/*
  Example :
  
           uint8_t status;
           
		   -> status = Mpu6050_ConnectionTest();
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */

uint8_t Mpu6050_Init(void); /* Function for initialize MPU6050 */
/*
  Example :
  
           uint8_t status;
           
		   -> status = Mpu6050_Init();
		      
		     -> status is _TRUE/_FALSE           
		   
*/

uint8_t Mpu6050_AutoInit(void); /* Function for initialize MPU6050 */
/*
  Example :
  
           uint8_t status;
           
		   -> status = Mpu6050_AutoInit();
		      
		     -> status is _TRUE/_FALSE            
		   
*/

/* ^^^^^^^^^^^^^^^^^^^ Raw Value ^^^^^^^^^^^^^^^^^^^ */

uint8_t Mpu6050_GetRawAccelX(int16_t *raw_accelx_value); /* Function for take Accelerometer (x) value */
/*
  Example :
  
           uint8_t status;
		   int16_t x;
           
		   -> status = Mpu6050_GetRawAccelX( &x );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetRawAccelY(int16_t *raw_accely_value); /* Function for take Accelerometer (y) value */
/*
  Example :
  
           uint8_t status;
		   int16_t y;
           
		   -> status = Mpu6050_GetRawAccelY( &y );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetRawAccelZ(int16_t *raw_accelz_value); /* Function for take Accelerometer (z) value */
/*
  Example :
  
           uint8_t status;
		   int16_t z;
           
		   -> status = Mpu6050_GetRawAccelZ( &z );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetRawAccel( int16_t *raw_accel_str ); /* Function for take Accelerometer value */
/*
  Example :
  
           uint8_t status;
		   int16_t x[3];
           
		   -> status = Mpu6050_GetRawAccel( x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_GetRawTemp(int16_t *raw_temp_value); /* Function for take Temperature value */
/*
  Example :
  
           uint8_t status;
		   int16_t t;
           
		   -> status = Mpu6050_GetRawTemp( &t );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_GetRawGyroX(int16_t *raw_gyrox_value); /* Function for take Gyroscope (x) value */
/*
  Example :
  
           uint8_t status;
		   int16_t x;
           
		   -> status = Mpu6050_GetRawGyroX( &x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetRawGyroY(int16_t *raw_gyroy_value); /* Function for take Gyroscope (y) value */
/*
  Example :
  
           uint8_t status;
		   int16_t y;
           
		   -> status = Mpu6050_GetRawGyroY( &y );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetRawGyroZ(int16_t *raw_gyroz_value); /* Function for take Gyroscope (z) value */
/*
  Example :
  
           uint8_t status;
		   int16_t z;
           
		   -> status = Mpu6050_GetRawGyroZ( &z );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetRawGyro( int16_t *raw_gyro_str ); /* Function for take Gyroscope value */
/*
  Example :
  
           uint8_t status;
		   int16_t x[3];
           
		   -> status = Mpu6050_GetRawGyro( x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ^^^^^^^^^^^^^^^^^^^^^ Value ^^^^^^^^^^^^^^^^^^^^^ */

uint8_t Mpu6050_GetAccelX(float *accelx_value); /* Function for take Accelerometer (x) value */
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = Mpu6050_GetAccelX( &x );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetAccelY(float *accely_value); /* Function for take Accelerometer (y) value */
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = Mpu6050_GetAccelY( &y );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetAccelZ(float *accelz_value); /* Function for take Accelerometer (z) value */
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = Mpu6050_GetAccelZ( &z );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetAccel( float *accel_str ); /* Function for take Accelerometer value */
/*
  Example :
  
           uint8_t status;
		   float x[3];
           
		   -> status = Mpu6050_GetAccel( x );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_GetTemp(float *temp_value); /* Function for take Temperature value */
/*
  Example :
  
           uint8_t status;
		   float t;
           
		   -> status = Mpu6050_GetTemp( &t );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_GetGyroX(float *gyrox_value); /* Function for take Gyroscope (x) value */
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = Mpu6050_GetGyroX( &x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetGyroY(float *gyroy_value); /* Function for take Gyroscope (y) value */
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = Mpu6050_GetGyroY( &y );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetGyroZ(float *gyroz_value); /* Function for take Gyroscope (z) value */
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = Mpu6050_GetGyroZ( &z );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetGyro( float *gyro_str ); /* Function for take Gyroscope value */
/*
  Example :
  
           uint8_t status;
		   float x[3];
           
		   -> status = Mpu6050_GetGyro( x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ^^^^^^^^^^^^^^^^^^^^^ Angle ^^^^^^^^^^^^^^^^^^^^^ */

uint8_t Mpu6050_GetAngleX(float *ang_x); /* Function for take x angle */
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = Mpu6050_GetAngleX( &x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetAngleY(float *ang_y); /* Function for take y angle */
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = Mpu6050_GetAngleY( &y );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetAngleZ(float *ang_z); /* Function for take z angle */
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = Mpu6050_GetAngleZ( &z );
		      
		     -> status is _TRUE/_FALSE
		   
*/

#endif /* __MPU6050_H_ */
