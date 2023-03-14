/*
------------------------------------------------------------------------------
~ File   : mpu6050.h
~ Author : Majid Derhambakhsh
~ Version: V2.0.0
~ Created: 06/10/2022 05:00:00 PM
~ Brief  :
~ Support: 
           E-Mail : Majid.Derhambakhsh@Outlook.com (subject : Embedded Library Support)
		   
           Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:    

~ Attention  :    

~ Changes    :
------------------------------------------------------------------------------
*/

#ifndef __MPU6050_H_
#define __MPU6050_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>

#include "mpu6050_conf.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ---------------------------- MPU60X0 ---------------------------- */
/* .............. Device Description .............. */
#define MPU_DEV_ID_BIT_SET        0x3F

/* .................... Temp ...................... */
#define MPU_TEMP_RAW_TO_C_OFFSET  36.53f /* Temp offset value */
#define MPU_TEMP_RAW_TO_C_RATIO   340.0f /* Ratio for calculate temp */
#define MPU_TEMP_REG_LEN          2      /* Temp register length */

/* .................... Angle ..................... */
#define MPU_ANG_MIN  265 /* Minimum value for map */
#define MPU_ANG_MAX  402 /* Maximum value for map */
#define MPU_ANG_POS  90U /* Maximum positive value */
#define MPU_ANG_NEG  -90 /* Maximum negative value */

/* ................ Register bits ................. */
#define MPU_DEVICE_RESET_BIT 7

/* .................... Timing .................... */
#define MPU_WAKEUP_TIME_MS 50

/* .................... Common .................... */
#define MPU_BIT_3            3
#define MPU_BIT_5            5
#define MPU_BIT_6            6
#define MPU_BIT_7            7

#define MPU_HIGH_BYTE_SHIFT  8

#define MPU_INIT_PARAMS_ALL  7 /* Quantity of initialize instructions */
#define MPU_INIT_PARAMS_DEF  3

#define MPU_CLOCK_DIVIDER_8  8 /* Divider */
#define MPU_RAD_TO_DEG       57.2957786f /* Value for convert */

#define MPU_ACCEL_GYRO_DATA_REG_LEN 6

/* .................... Macro ..................... */
#define __MPU_SET_ID(ID) (((ID) & MPU_DEV_ID_BIT_SET) << 1)
#define __MPU_GET_ID(ID) (((ID) >> 1) & MPU_DEV_ID_BIT_SET)

/* ----------------------- Define by compiler ---------------------- */
#ifdef __CODEVISIONAVR__  /* Check compiler */
	
	#define MPU_MEMADD_SIZE                         _I2C_MEMADD_SIZE_8BIT /* Memory address size */
	
	#define I2C_IS_DEVICE_READY(da,tr,tim)          I2C_IsDeviceReady((da),(tr),(tim))
	#define I2C_MEM_WRITE(da,ma,mas,md,qu,tim)      I2C_Mem_Write((da),(ma),(mas),(md),(qu),(tim))
	#define I2C_MEM_READ(da,ma,mas,md,qu,tim)       I2C_Mem_Read((da),(ma),(mas),(md),(qu),(tim))
	
	#ifndef DELAY_MS
		#define DELAY_MS(t)                         delay_ms((t)) /* Change function */
	#endif /* _DELAY_MS */

/* ------------------------------------------------------------------ */
#elif defined(__GNUC__) && !defined(USE_HAL_DRIVER)  /* Check compiler */
	
	#define MPU_MEMADD_SIZE                         _I2C_MEMADD_SIZE_8BIT /* Memory address size */
	
	#define I2C_IS_DEVICE_READY(da,tr,tim)          I2C_IsDeviceReady((da),(tr),(tim)) /* Change function */
	#define I2C_MEM_WRITE(da,ma,mas,md,qu,tim)      I2C_Mem_Write((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	#define I2C_MEM_READ(da,ma,mas,md,qu,tim)       I2C_Mem_Read((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	
	#ifndef DELAY_MS
		#define DELAY_MS(t)                         _delay_ms((t)) /* Change function */
	#endif /* _DELAY_MS */
	
/* ------------------------------------------------------------------ */
#elif defined(USE_HAL_DRIVER)  /* Check driver */
	
	#define MPU_MEMADD_SIZE                         I2C_MEMADD_SIZE_8BIT /* Memory address size */
	
	#define I2C_IS_DEVICE_READY(i2c,da,tr,tim)      HAL_I2C_IsDeviceReady((i2c),(da),(tr),(tim)) /* Change function */
	#define I2C_MEM_WRITE(i2c,da,ma,mas,md,qu,tim)  HAL_I2C_Mem_Write2((i2c),(da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	#define I2C_MEM_READ(i2c,da,ma,mas,md,qu,tim)   HAL_I2C_Mem_Read2((i2c),(da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	
	#ifndef DELAY_MS
		#define DELAY_MS(t)                         HAL_Delay((t)) /* Change function */
	#endif /* _DELAY_MS */
	
#endif /* __CODEVISIONAVR__ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ---------------------------- Address ---------------------------- */
typedef enum
{
	
	MPU_ADD_LOW  = 0xD0,
	MPU_ADD_HIGH = 0xD2,
	
}MPU_AddTypeDef;

/* ------------------------- Configurations ------------------------ */
typedef enum /* Digital Low Pass Filter Config */
{
	
	MPU_DLPF_CFG_260A_256G_HZ = 0,
	MPU_DLPF_CFG_184A_188G_HZ = 1,
	MPU_DLPF_CFG_94A_98G_HZ   = 2,
	MPU_DLPF_CFG_44A_42G_HZ   = 3,
	MPU_DLPF_CFG_21A_20G_HZ   = 4,
	MPU_DLPF_CFG_10_HZ        = 5,
	MPU_DLPF_CFG_5_HZ         = 6
	
}MPU_DLPF_CFG_TypeDef;

typedef enum /* External Sync Config */
{
	
	/* Configures the FSYNC pin sampling. */
	MPU_ES_INPUT_DISABLE = 0,
	MPU_ES_TEMP_OUT_L    = 1,
	MPU_ES_GYRO_XOUT_L   = 2,
	MPU_ES_GYRO_YOUT_L   = 3,
	MPU_ES_GYRO_ZOUT_L   = 4,
	MPU_ES_ACCEL_XOUT_L  = 5,
	MPU_ES_ACCEL_YOUT_L  = 6,
	MPU_ES_ACCEL_ZOUT_L  = 7
	
}MPU_EXT_SYNC_SET_TypeDef;

typedef enum /* Gyro Full Scale Range */
{
	
	MPU_GYRO_FULL_SCALE_RANGE_250  = 0, /* Full scale range +/- 250 degree/S  */
	MPU_GYRO_FULL_SCALE_RANGE_500  = 1, /* Full scale range +/- 500 degree/S  */
	MPU_GYRO_FULL_SCALE_RANGE_1000 = 2, /* Full scale range +/- 1000 degree/S */
	MPU_GYRO_FULL_SCALE_RANGE_2000 = 3  /* Full scale range +/- 2000 degree/S */
	
}MPU_GFS_SEL_TypeDef;

typedef enum /* Accelerator Full Scale Range */
{
	
	MPU_ACCEL_FULL_SCALE_RANGE_2G  = 0, /* Full scale range +/- 2g  */
	MPU_ACCEL_FULL_SCALE_RANGE_4G  = 1, /* Full scale range +/- 4g  */
	MPU_ACCEL_FULL_SCALE_RANGE_8G  = 2, /* Full scale range +/- 8g  */
	MPU_ACCEL_FULL_SCALE_RANGE_16G = 3  /* Full scale range +/- 16g */
	
}MPU_AFS_SEL_TypeDef;

typedef enum /* Interrupt Config */
{
	
	MPU_INT_LEVEL_ACTIVE_HIGH          = 0, /* the logic level for the INT pin is active high. */
	MPU_INT_LEVEL_ACTIVE_LOW           = 1, /* the logic level for the INT pin is active low. */
	MPU_INT_OPEN_PUSH_PULL             = 0, /* the INT pin is configured as push-pull. */
	MPU_INT_OPEN_OPEN_DRAIN            = 1, /* the INT pin is configured as open drain. */
	MPU_LATCH_INT_EN_50US_PULSE        = 0, /* the INT pin emits a 50us long pulse. */
	MPU_LATCH_INT_EN_INTERRUPT_CLEARED = 1  /* the INT pin is held high until the interrupt is cleared. */
	
}MPU_INT_CFG_TypeDef;

typedef enum /* Interrupt Enable */
{
	
	MPU_INT_DISABLE        = 0x00, /* Disable Interrupt */
	MPU_INT_DATA_RDY_EN    = 0x01, /* This bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed. */
	MPU_INT_I2C_MST_INT_EN = 0x08, /* This bit enables any of the I2C Master interrupt sources to generate an interrupt. */
	MPU_INT_FIFO_OFLOW_EN  = 0x10  /* This bit enables a FIFO buffer overflow to generate an interrupt. */
	
}MPU_INT_EN_TypeDef;

typedef enum /* Clock Select */
{
	
	MPU_CLKSEL_INTERNAL_8MHZ_OSCILLATOR    = 0, /* Internal 8MHz oscillator              */
	MPU_CLKSEL_X_AXIS_GYROSCOPE_REFERENCE  = 1, /* PLL with X axis gyroscope reference   */
	MPU_CLKSEL_Y_AXIS_GYROSCOPE_REFERENCE  = 2, /* PLL with Y axis gyroscope reference   */
	MPU_CLKSEL_Z_AXIS_GYROSCOPE_REFERENCE  = 3, /* PLL with Z axis gyroscope reference   */
	MPU_CLKSEL_EXTERNAL_32768HZ_REFERENCE  = 4, /* PLL with external 32.768kHz reference */
	MPU_CLKSEL_EXTERNAL_19200KHZ_REFERENCE = 5, /* PLL with external 19.2MHz reference   */
	MPU_CLKSEL_STOP                        = 7  /* Stops the clock and keeps the timing generator in reset */
	
}MPU_CLKSEL_TypeDef;

/* --------------------------- Registers --------------------------- */
typedef enum /* Register Address */
{
	
	MPU_REG_SELF_TEST_X        = 0x0D, /* Serial I-F = RW */
	MPU_REG_SELF_TEST_Y        = 0x0E, /* Serial I-F = RW */
	MPU_REG_SELF_TEST_Z        = 0x0F, /* Serial I-F = RW */
	MPU_REG_SELF_TEST_A        = 0x10, /* Serial I-F = RW */
	MPU_REG_SMPLRT_DIV         = 0x19, /* Serial I-F = RW */
	MPU_REG_CONFIG             = 0x1A, /* Serial I-F = RW */
	MPU_REG_GYRO_CONFIG        = 0x1B, /* Serial I-F = RW */
	MPU_REG_ACCEL_CONFIG       = 0x1C, /* Serial I-F = RW */
	MPU_REG_FIFO_EN            = 0x23, /* Serial I-F = RW */
	MPU_REG_I2C_MST_CTRL       = 0x24, /* Serial I-F = RW */
	MPU_REG_I2C_SLV0_ADDR      = 0x25, /* Serial I-F = RW */
	MPU_REG_I2C_SLV0_REG       = 0x26, /* Serial I-F = RW */
	MPU_REG_I2C_SLV0_CTRL      = 0x27, /* Serial I-F = RW */
	MPU_REG_I2C_SLV1_ADDR      = 0x28, /* Serial I-F = RW */
	MPU_REG_I2C_SLV1_REG       = 0x29, /* Serial I-F = RW */
	MPU_REG_I2C_SLV1_CTRL      = 0x2A, /* Serial I-F = RW */
	MPU_REG_I2C_SLV2_ADDR      = 0x2B, /* Serial I-F = RW */
	MPU_REG_I2C_SLV2_REG       = 0x2C, /* Serial I-F = RW */
	MPU_REG_I2C_SLV2_CTRL      = 0x2D, /* Serial I-F = RW */
	MPU_REG_I2C_SLV3_ADDR      = 0x2E, /* Serial I-F = RW */
	MPU_REG_I2C_SLV3_REG       = 0x2F, /* Serial I-F = RW */
	MPU_REG_I2C_SLV3_CTRL      = 0x30, /* Serial I-F = RW */
	MPU_REG_I2C_SLV4_ADDR      = 0x31, /* Serial I-F = RW */
	MPU_REG_I2C_SLV4_REG       = 0x32, /* Serial I-F = RW */
	MPU_REG_I2C_SLV4_DO        = 0x33, /* Serial I-F = RW */
	MPU_REG_I2C_SLV4_CTRL      = 0x34, /* Serial I-F = RW */
	MPU_REG_I2C_SLV4_DI        = 0x35, /* Serial I-F = R  */
	MPU_REG_I2C_MST_STATUS     = 0x36, /* Serial I-F = R  */
	MPU_REG_INT_PIN_CFG        = 0x37, /* Serial I-F = RW */
	MPU_REG_INT_ENABLE         = 0x38, /* Serial I-F = RW */
	MPU_REG_INT_STATUS         = 0x3A, /* Serial I-F = R  */
	MPU_REG_ACCEL_XOUT_H       = 0x3B, /* Serial I-F = R  */
	MPU_REG_ACCEL_XOUT_L       = 0x3C, /* Serial I-F = R  */
	MPU_REG_ACCEL_YOUT_H       = 0x3D, /* Serial I-F = R  */
	MPU_REG_ACCEL_YOUT_L       = 0x3E, /* Serial I-F = R  */
	MPU_REG_ACCEL_ZOUT_H       = 0x3F, /* Serial I-F = R  */
	MPU_REG_ACCEL_ZOUT_L       = 0x40, /* Serial I-F = R  */
	MPU_REG_TEMP_OUT_H         = 0x41, /* Serial I-F = R  */
	MPU_REG_TEMP_OUT_L         = 0x42, /* Serial I-F = R  */
	MPU_REG_GYRO_XOUT_H        = 0x43, /* Serial I-F = R  */
	MPU_REG_GYRO_XOUT_L        = 0x44, /* Serial I-F = R  */
	MPU_REG_GYRO_YOUT_H        = 0x45, /* Serial I-F = R  */
	MPU_REG_GYRO_YOUT_L        = 0x46, /* Serial I-F = R  */
	MPU_REG_GYRO_ZOUT_H        = 0x47, /* Serial I-F = R  */
	MPU_REG_GYRO_ZOUT_L        = 0x48, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_00   = 0x49, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_01   = 0x4A, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_02   = 0x4B, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_03   = 0x4C, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_04   = 0x4D, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_05   = 0x4E, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_06   = 0x4F, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_07   = 0x50, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_08   = 0x51, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_09   = 0x52, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_10   = 0x53, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_11   = 0x54, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_12   = 0x55, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_13   = 0x56, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_14   = 0x57, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_15   = 0x58, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_16   = 0x59, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_17   = 0x5A, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_18   = 0x5B, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_19   = 0x5C, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_20   = 0x5D, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_21   = 0x5E, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_22   = 0x5F, /* Serial I-F = R  */
	MPU_REG_EXT_SENS_DATA_23   = 0x60, /* Serial I-F = R  */
	MPU_REG_I2C_SLV0_DO        = 0x63, /* Serial I-F = RW */
	MPU_REG_I2C_SLV1_DO        = 0x64, /* Serial I-F = RW */
	MPU_REG_I2C_SLV2_DO        = 0x65, /* Serial I-F = RW */
	MPU_REG_I2C_SLV3_DO        = 0x66, /* Serial I-F = RW */
	MPU_REG_I2C_MST_DELAY_CTRL = 0x67, /* Serial I-F = RW */
	MPU_REG_SIGNAL_PATH_RESET  = 0x68, /* Serial I-F = RW */
	MPU_REG_USER_CTRL          = 0x6A, /* Serial I-F = RW */
	MPU_REG_PWR_MGMT_1         = 0x6B, /* Serial I-F = RW */
	MPU_REG_PWR_MGMT_2         = 0x6C, /* Serial I-F = RW */
	MPU_REG_FIFO_COUNTH        = 0x72, /* Serial I-F = RW */
	MPU_REG_FIFO_COUNTL        = 0x73, /* Serial I-F = RW */
	MPU_REG_FIFO_R_W           = 0x74, /* Serial I-F = RW */
	MPU_REG_WHO_AM_I           = 0x75  /* Serial I-F = R  */
	
}MPU_REG_ADD_TypeDef;

/* --------------------------- Data Types -------------------------- */
typedef struct /* Struct for config interrupt in IC */
{
	
	MPU_INT_CFG_TypeDef IntLevel   : 1; /* This bit set the logic level for the INT pin */
	MPU_INT_CFG_TypeDef IntOpen    : 1; /* This bit set the INT pin mode (Push pull / Open drain) */
	MPU_INT_CFG_TypeDef LatchIntEn : 1; /* This bit set the INT pin pulse mode */

}MPU_INT_CFG_OPT_TypeDef;

typedef struct
{
	
	int16_t X;
	int16_t Y;
	int16_t Z;
	
}MPU_RawTypeDef;

typedef struct
{
	
	float X;
	float Y;
	float Z;
	
}MPU_XYZTypeDef;

typedef struct
{
	
	float Roll;
	float Pitch;
	float Yaw;
	
}MPU_RPYTypeDef;

/* ---------------------------- Common ----------------------------- */
#ifdef USE_HAL_DRIVER

typedef enum 
{
	
	MPU_ERROR = HAL_ERROR,
	MPU_OK    = HAL_OK,
	
}MPU_StatusTypeDef;

#elif

typedef enum
{
	
	MPU_ERROR = 0,
	MPU_OK    = 1,
	
}MPU_StatusTypeDef;

#endif // USE_HAL_DRIVER

typedef struct
{
	
	#ifdef USE_HAL_DRIVER
	
	I2C_HandleTypeDef *I2Cx;
	
	#endif
	
	uint8_t                  SampleRateDivider; /* The Sample Rate is determined by dividing the gyroscope output rate by this value. */
	MPU_AddTypeDef           Address;
	MPU_EXT_SYNC_SET_TypeDef ExtSync;  /* Configures the FSYNC pin sampling. */
	MPU_DLPF_CFG_TypeDef     DigitalLowPassFilter; /* Configures the DLPF setting. */
	MPU_GFS_SEL_TypeDef      GyroFullScaleRange; /* Selects the full scale range of gyroscopes. */
	MPU_AFS_SEL_TypeDef      AccelFullScaleRange; /* Selects the full scale range of accelerometers. */
	MPU_INT_CFG_OPT_TypeDef  InterruptConfig; /* Config interrupt option */
	MPU_INT_EN_TypeDef       InterruptEnable; /* Interrupt enable */
	MPU_CLKSEL_TypeDef       ClockSelection; /* Set clock source */
	
	float GyroSensitivity;
	float AccelSensitivity;
	
}MPU_TypeDef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ............................ Check .......................... */
MPU_StatusTypeDef MPU6050_IsReady(MPU_TypeDef *MPUx, uint8_t Trials, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_Reset(MPU_TypeDef *MPUx, uint16_t Timeout);

/* ......................... Initialize ........................ */
MPU_StatusTypeDef MPU6050_Init(MPU_TypeDef *MPUx, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_AutoInit(MPU_TypeDef *MPUx, uint16_t Timeout);

MPU_StatusTypeDef MPU6050_DefInit(MPU_TypeDef *MPUx, uint16_t Timeout);

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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#endif
