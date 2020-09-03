/*
------------------------------------------------------------------------------
~ File   : mpu6050.h
~ Author : Majid Derhambakhsh
~ Version: V1.0.0
~ Created: 09/01/2020 02:00:00 AM
~ Brief  :
~ Support:
           E-Mail : Majid.Derhambakhsh@gmail.com (subject : Embedded Library Support)
           
           Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:    STM32-AVR MPU60X0 Library.

~ Attention  :    

~ Changes    :    - Change library name
                  - Change Mpu6050 to MPU6050
                  - Change MPU6050_GetAngleY to MPU6050_GetAccelAngleY
                  - Change MPU6050_GetAngleZ to MPU6050_GetAccelAngleZ
                  - Change MPU6050_ConnectionTest to MPU6050_IsReady
                  - Change _TRUE/_FALSE to _MPU_OK/_MPU_ERROR
                  
                  - Add : _time_out parameter in input argument of all functions
                  - Add : MPU6050_DefInit function
                  - Add : MPU6050_Reset function
                  - Add : MPU6050_SetDeviceID function
                  - Add : MPU6050_GetDeviceID function
                  - Add : required header section
                  - Add : AD0 Pin level modification
                  - Add : STM32 supporting
                  
                  - Improve : algorithm, memory usage & code speeds
------------------------------------------------------------------------------
*/

#ifndef __MPU6050_H_
#define __MPU6050_H_

/* ------------------------------------------------ Includes ------------------------------------------------- */

#include "mpu6050_conf.h" /* Import config file */
#include <stdint.h> /* Import standard integer lib */

#ifndef __MATH_EX_H_
	#include "MATH_EX/math_ex.h" /* Import math lib */
#endif

/* ------------------------------------------------------------------ */

#ifdef __CODEVISIONAVR__  /* Check compiler */

#pragma warn_unref_func- /* Disable 'unused function' warning */

	#ifndef __GPIO_UNIT_H_
		#include "GPIO/gpio_unit.h" /* Import gpio lib */
	#endif
	
	#ifndef __I2C_UNIT_H_
		#include "I2C_UNIT/i2c_unit.h" /* Import i2c lib */
	#endif

	#include <delay.h>       /* Import delay library */

/* ------------------------------------------------------------------ */

#elif defined(__GNUC__) && !defined(USE_HAL_DRIVER)  /* Check compiler */

#pragma GCC diagnostic ignored "-Wunused-function" /* Disable 'unused function' warning */

	#ifndef __GPIO_UNIT_H_
		#include "GPIO/gpio_unit.h" /* Import gpio lib */
	#endif
	
	#ifndef __I2C_UNIT_H_
		#include "I2C_UNIT/i2c_unit.h" /* Import i2c lib */
	#endif

	#include <util/delay.h>  /* Import delay library */

/* ------------------------------------------------------------------ */

#elif defined(USE_HAL_DRIVER)  /* Check driver */

/* ------------------------------------------------------- */

	#if defined ( __ICCARM__ ) /* ICCARM Compiler */

		#pragma diag_suppress=Pe177   /* Disable 'unused function' warning */

	#elif defined   (  __GNUC__  ) /* GNU Compiler */

		#pragma diag_suppress 177     /* Disable 'unused function' warning */

	#endif /* __ICCARM__ */

/* ------------------------------------------------------- */

#ifndef __STM32_I2C_H_
	#include "STM32_I2C/stm32_i2c.h" /* Import i2c lib */
#endif

/* ------------------------------------------------------------------ */

#else                     /* Compiler not found */

	#error Chip or I2C Library not supported  /* Send error */

#endif /* __CODEVISIONAVR__ */

/* ------------------------------------------------------------------ */

/* ------------------------------------------------- Defines ------------------------------------------------- */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~ Type size ~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#ifndef ENUM_U8_T
 
 #define ENUM_U8_T(ENUM_NAME)   Enum_##ENUM_NAME; typedef uint8_t ENUM_NAME /* Config enum size */

#endif

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~ MPU60X0 ~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ---------------- Device Address ---------------- */
#ifndef _MPU_AD0_LEVEL
	#define _MPU_AD0_LEVEL _MPU_AD0_LOW
#elif (_MPU_AD0_LEVEL != _MPU_AD0_LOW) && (_MPU_AD0_LEVEL != _MPU_AD0_HIGH) /* Set AD0 status */
	#define _MPU_AD0_LEVEL _MPU_AD0_LOW
#endif

#define _MPU6050_ADD         0xD0                 /* MPU6050 address for write */
#define _MPU6050_MEMADD_SIZE I2C_MEMADD_SIZE_8BIT /* Memory address size */

/* ---------------- AD0 Pin State ----------------- */
#define _MPU_AD0_HIGH 0x02 /* AD0 Bit value in the high state */
#define _MPU_AD0_LOW  0x00 /* AD0 Bit value in the low state */

/* -------------- Device Description -------------- */
#define _MPU6050_DEV_ID_BITS 0x3F

/* -------------------- Temp ---------------------- */
#define _MPU_TEMP_CONST      36.53f /* Temp constant value */
#define _MPU_TEMP_DIV        340.0f /* Divider for calculate temp */
#define _MPU_TEMP_REG_LENGTH 2      /* Temp registers */

/* -------------------- Accel --------------------- */
/* Divider for calculate accelerometer value */
#define _MPU_ACCEL_SENS_2G_SENS  16384.0f
#define _MPU_ACCEL_SENS_4G_SENS  8192.0f
#define _MPU_ACCEL_SENS_8G_SENS  4096.0f
#define _MPU_ACCEL_SENS_16G_SENS 2048.0f

/* -------------------- Gyro --------------------- */
/* Divider for calculate gyroscope value */
#define _MPU_GYRO_SENS_250_SENS  131.0f
#define _MPU_GYRO_SENS_500_SENS  65.5f
#define _MPU_GYRO_SENS_1000_SENS 32.8f
#define _MPU_GYRO_SENS_2000_SENS 16.4f

/* -------------------- Angle --------------------- */
#define _MPU_ANG_MIN  265 /* Minimum value for map */
#define _MPU_ANG_MAX  402 /* Maximum value for map */
#define _MPU_ANG_POS  90U /* Maximum positive value */
#define _MPU_ANG_NEG  -90 /* Maximum negative value */

/* ---------------- Register bits ----------------- */
#define _MPU_REG_PWR_MGMT_1_DEVICE_RESET_BIT 7

/* -------------------- Timing -------------------- */
#define _MPU_WAKEUP_TIME_MS 50

/* -------------------- Public -------------------- */
#define _MPU_INIT_PARAM_ALL     7 /* Quantity of initialize instructions */
#define _MPU_INIT_PARAM_DEF     3
#define _MPU_HIGH_BYTE_SHIFT    8 /* High byte shift value */
#define _MPU_CLOCK_DIVIDER_8    8 /* Divider */
#define _MPU_REG_BIT_3          3 /* 3 Bit */
#define _MPU_REG_BIT_5          5 /* 5 Bit */
#define _MPU_REG_BIT_6          6 /* 6 Bit */
#define _MPU_REG_BIT_7          7 /* 7 Bit */
#define _MPU_AXIS_ALL           3 /* Axis */
#define _MPU_AXIS_REG_LENGTH    2 /* One Axis Registers */
#define _MPU_AXIS_ALL_REG_LEGTH 6 /* All Axis Registers */
#define _MPU_RAD_TO_DEG         57.2957786f /* Value for convert */
#define _MPU_TRIALS             5 /* Number of i2c trials */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ By compiler ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#ifdef __CODEVISIONAVR__  /* Check compiler */

	#define _MPU_ERROR             _STAT_ERROR             /* OK status value */
	#define _MPU_OK                _STAT_OK                /* OK status value */
	#define I2C_MEMADD_SIZE_8BIT   _I2C_MEMADD_SIZE_8BIT   /* Memory Address Size */
	#define I2C_MEMADD_SIZE_16BIT  _I2C_MEMADD_SIZE_16BIT  /* Memory Address Size */

	#define _I2C_MEM_READY(da,tr,tim)            I2C_IsDeviceReady((da),(tr),(tim)) /* Change function */
	#define _I2C_MEM_WRITE(da,ma,mas,md,qu,tim)  I2C_Mem_Write((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	#define _I2C_MEM_READ(da,ma,mas,md,qu,tim)   I2C_Mem_Read((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	#define _MEM_WP_CONTROL(ps)                  GPIO_WritePin(&_WRITE_PROTECT_PORT,(1 << _WRITE_PROTECT_PIN),(ps)) /* Change function */

#ifndef _DELAY_MS
	#define _DELAY_MS(t)                     delay_ms((t)) /* Change function */
#endif /* _DELAY_MS */

/* ------------------------------------------------------------------ */

#elif defined(__GNUC__) && !defined(USE_HAL_DRIVER)  /* Check compiler */

	#define _MPU_ERROR             _STAT_ERROR             /* OK status value */
	#define _MPU_OK                _STAT_OK                /* OK status value */
	#define I2C_MEMADD_SIZE_8BIT   _I2C_MEMADD_SIZE_8BIT   /* Memory Address Size */
	#define I2C_MEMADD_SIZE_16BIT  _I2C_MEMADD_SIZE_16BIT  /* Memory Address Size */

	#define _I2C_MEM_READY(da,tr,tim)            I2C_IsDeviceReady((da),(tr),(tim)) /* Change function */
	#define _I2C_MEM_WRITE(da,ma,mas,md,qu,tim)  I2C_Mem_Write((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	#define _I2C_MEM_READ(da,ma,mas,md,qu,tim)   I2C_Mem_Read((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	#define _MEM_WP_CONTROL(ps)                  GPIO_WritePin(&_WRITE_PROTECT_PORT,(1 << _WRITE_PROTECT_PIN),(ps)) /* Change function */

#ifndef _DELAY_MS
	#define _DELAY_MS(t)                     _delay_ms((t)) /* Change function */
#endif /* _DELAY_MS */

/* ------------------------------------------------------------------ */

#elif defined(USE_HAL_DRIVER)  /* Check driver */

	#define _MPU_ERROR                          HAL_ERROR      /* OK status value */
	#define _MPU_OK                             HAL_OK         /* OK status value */
	#define _GPIO_PIN_RESET                     GPIO_PIN_RESET /* Select GPIO reset instruction */
	#define _GPIO_PIN_SET                       GPIO_PIN_SET   /* Select GPIO set instruction */

	#define _I2C_MEM_READY(da,tr,tim)            HAL_I2C_IsDeviceReady(&_CONNECTED_I2C,(da),(tr),(tim)) /* Change function */
	#define _I2C_MEM_WRITE(da,ma,mas,md,qu,tim)  HAL_I2C_Mem_Write2(&_CONNECTED_I2C,(da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	#define _I2C_MEM_READ(da,ma,mas,md,qu,tim)   HAL_I2C_Mem_Read2(&_CONNECTED_I2C,(da),(ma),(mas),(md),(qu),(tim)) /* Change function */
	#define _MEM_WP_CONTROL(ps)                  HAL_GPIO_WritePin(_WRITE_PROTECT_PORT,(1 << _WRITE_PROTECT_PIN),(ps)) /* Change function */

#ifndef _DELAY_MS
	#define _DELAY_MS(t)                     HAL_Delay((t)) /* Change function */
#endif /* _DELAY_MS */

/* ------------------------------------------------------------------ */

#else
#endif /* __CODEVISIONAVR__ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* -------------------------------------------------- Types -------------------------------------------------- */
/* -------------------------------------------------- Enum --------------------------------------------------- */
typedef enum /* Enum for digital low pass filter config */
{
	
	_DLPF_CFG_260A_256G_HZ = 0, /* Config to 0 */
	_DLPF_CFG_184A_188G_HZ = 1, /* Config to 1 */
	_DLPF_CFG_94A_98G_HZ   = 2, /* Config to 2 */
	_DLPF_CFG_44A_42G_HZ   = 3, /* Config to 3 */
	_DLPF_CFG_21A_20G_HZ   = 4, /* Config to 4 */
	_DLPF_CFG_10_HZ        = 5, /* Config to 5 */
	_DLPF_CFG_5_HZ         = 6  /* Config to 6 */
	
}MPU6050_DLPF_CFG_t;

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
	
}MPU6050_EXT_SYNC_SET_t;

typedef enum /* Enum for gyro full scale range */
{
	
	_GYRO_FULL_SCALE_RANGE_250  = 0, /* Full scale range +/- 250 degree/S  */
	_GYRO_FULL_SCALE_RANGE_500  = 1, /* Full scale range +/- 500 degree/S  */
	_GYRO_FULL_SCALE_RANGE_1000 = 2, /* Full scale range +/- 1000 degree/S */
	_GYRO_FULL_SCALE_RANGE_2000 = 3  /* Full scale range +/- 2000 degree/S */
	
}MPU6050_GFS_SEL_t;

typedef enum /* Enum for accelerator full scale range */
{
	
	_ACCEL_FULL_SCALE_RANGE_2G  = 0, /* Full scale range +/- 2g  */
	_ACCEL_FULL_SCALE_RANGE_4G  = 1, /* Full scale range +/- 4g  */
	_ACCEL_FULL_SCALE_RANGE_8G  = 2, /* Full scale range +/- 8g  */
	_ACCEL_FULL_SCALE_RANGE_16G = 3  /* Full scale range +/- 16g */
	
}MPU6050_AFS_SEL_t;

typedef enum /* Enum for interrupt config */
{
	
	_INT_LEVEL_ACTIVE_HIGH          = 0, /* the logic level for the INT pin is active high. */
	_INT_LEVEL_ACTIVE_LOW           = 1, /* the logic level for the INT pin is active low. */
	_INT_OPEN_PUSH_PULL             = 0, /* the INT pin is configured as push-pull. */
	_INT_OPEN_OPEN_DRAIN            = 1, /* the INT pin is configured as open drain. */
	_LATCH_INT_EN_50US_PULSE        = 0, /* the INT pin emits a 50us long pulse. */
	_LATCH_INT_EN_INTERRUPT_CLEARED = 1  /* the INT pin is held high until the interrupt is cleared. */
	
}MPU6050_INT_CFG_t;

typedef enum /* Enum for interrupt enable */
{
	_INT_DISABLE        = 0x00, /* Disable Interrupt */
	_INT_DATA_RDY_EN    = 0x01, /* this bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed. */
	_INT_I2C_MST_INT_EN = 0x08, /* this bit enables any of the I2C Master interrupt sources to generate an interrupt. */
	_INT_FIFO_OFLOW_EN  = 0x10  /* this bit enables a FIFO buffer overflow to generate an interrupt. */
	
}MPU6050_INT_ENABLE_t;

typedef enum /* Enum for Clock select value */
{
	
	_CLKSEL_INTERNAL_8MHZ_OSCILLATOR    = 0, /* Internal 8MHz oscillator              */
	_CLKSEL_X_AXIS_GYROSCOPE_REFERENCE  = 1, /* PLL with X axis gyroscope reference   */
	_CLKSEL_Y_AXIS_GYROSCOPE_REFERENCE  = 2, /* PLL with Y axis gyroscope reference   */
	_CLKSEL_Z_AXIS_GYROSCOPE_REFERENCE  = 3, /* PLL with Z axis gyroscope reference   */
	_CLKSEL_EXTERNAL_32768HZ_REFERENCE  = 4, /* PLL with external 32.768kHz reference */
	_CLKSEL_EXTERNAL_19200KHZ_REFERENCE = 5, /* PLL with external 19.2MHz reference   */
	_CLKSEL_STOP                        = 7  /* Stops the clock and keeps the timing generator in reset */ 
	
}MPU6050_CLKSEL_t;

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
	
}MPU6050_REG_ADD_t;

/* Enum U8 */
/*
typedef enum / * Enum for digital low pass filter config * /
{
	
	_DLPF_CFG_260A_256G_HZ = 0, / * Config to 0 * /
	_DLPF_CFG_184A_188G_HZ = 1, / * Config to 1 * /
	_DLPF_CFG_94A_98G_HZ   = 2, / * Config to 2 * /
	_DLPF_CFG_44A_42G_HZ   = 3, / * Config to 3 * /
	_DLPF_CFG_21A_20G_HZ   = 4, / * Config to 4 * /
	_DLPF_CFG_10_HZ        = 5, / * Config to 5 * /
	_DLPF_CFG_5_HZ         = 6  / * Config to 6 * /
	
}ENUM_U8_T(MPU6050_DLPF_CFG_t);

typedef enum / * Enum for external sync config * /
{
	
	_ES_INPUT_DISABLE = 0, / * Configures the FSYNC pin sampling. * /
	_ES_TEMP_OUT_L    = 1, / * Configures the FSYNC pin sampling. * /
	_ES_GYRO_XOUT_L   = 2, / * Configures the FSYNC pin sampling. * /
	_ES_GYRO_YOUT_L   = 3, / * Configures the FSYNC pin sampling. * /
	_ES_GYRO_ZOUT_L   = 4, / * Configures the FSYNC pin sampling. * /
	_ES_ACCEL_XOUT_L  = 5, / * Configures the FSYNC pin sampling. * /
	_ES_ACCEL_YOUT_L  = 6, / * Configures the FSYNC pin sampling. * /
	_ES_ACCEL_ZOUT_L  = 7  / * Configures the FSYNC pin sampling. * /
	
}ENUM_U8_T(MPU6050_EXT_SYNC_SET_t);

typedef enum / * Enum for gyro full scale range * /
{
	
	_GYRO_FULL_SCALE_RANGE_250  = 0, / * Full scale range +/- 250 degree/S  * /
	_GYRO_FULL_SCALE_RANGE_500  = 1, / * Full scale range +/- 500 degree/S  * /
	_GYRO_FULL_SCALE_RANGE_1000 = 2, / * Full scale range +/- 1000 degree/S * /
	_GYRO_FULL_SCALE_RANGE_2000 = 3  / * Full scale range +/- 2000 degree/S * /
	
}ENUM_U8_T(MPU6050_GFS_SEL_t);

typedef enum / * Enum for accel full scale range * /
{
	
	_ACCEL_FULL_SCALE_RANGE_2G  = 0, / * Full scale range +/- 2g  * /
	_ACCEL_FULL_SCALE_RANGE_4G  = 1, / * Full scale range +/- 4g  * /
	_ACCEL_FULL_SCALE_RANGE_8G  = 2, / * Full scale range +/- 8g  * /
	_ACCEL_FULL_SCALE_RANGE_16G = 3  / * Full scale range +/- 16g * /
	
}ENUM_U8_T(MPU6050_AFS_SEL_t);

typedef enum / * Enum for interrupt config * /
{
	
	_INT_LEVEL_ACTIVE_HIGH          = 0, / * the logic level for the INT pin is active high. * /
	_INT_LEVEL_ACTIVE_LOW           = 1, / * the logic level for the INT pin is active low. * /
	_INT_OPEN_PUSH_PULL             = 0, / * the INT pin is configured as push-pull. * /
	_INT_OPEN_OPEN_DRAIN            = 1, / * the INT pin is configured as open drain. * /
	_LATCH_INT_EN_50US_PULSE        = 0, / * the INT pin emits a 50us long pulse. * /
	_LATCH_INT_EN_INTERRUPT_CLEARED = 1  / * the INT pin is held high until the interrupt is cleared. * /
	
}ENUM_U8_T(MPU6050_INT_CONFIG_t);

typedef enum / * Enum for interrupt enable * /
{
	_INT_DISABLE        = 0x00, / * Disable Interrupt * /
	_INT_DATA_RDY_EN    = 0x01, / * this bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed. * /
	_INT_I2C_MST_INT_EN = 0x08, / * this bit enables any of the I2C Master interrupt sources to generate an interrupt. * /
	_INT_FIFO_OFLOW_EN  = 0x10  / * this bit enables a FIFO buffer overflow to generate an interrupt. * /
	
}ENUM_U8_T(MPU6050_INT_ENABLE_t);

typedef enum / * Enum for Clock select value * /
{
	
	_CLKSEL_INTERNAL_8MHZ_OSCILLATOR    = 0, / * Internal 8MHz oscillator              * /
	_CLKSEL_X_AXIS_GYROSCOPE_REFERENCE  = 1, / * PLL with X axis gyroscope reference   * /
	_CLKSEL_Y_AXIS_GYROSCOPE_REFERENCE  = 2, / * PLL with Y axis gyroscope reference   * /
	_CLKSEL_Z_AXIS_GYROSCOPE_REFERENCE  = 3, / * PLL with Z axis gyroscope reference   * /
	_CLKSEL_EXTERNAL_32768HZ_REFERENCE  = 4, / * PLL with external 32.768kHz reference * /
	_CLKSEL_EXTERNAL_19200KHZ_REFERENCE = 5, / * PLL with external 19.2MHz reference   * /
	_CLKSEL_STOP                        = 7  / * Stops the clock and keeps the timing generator in reset * /
	
}ENUM_U8_T(MPU6050_CLKSEL_t);

typedef enum / * Enum for Register address * /
{
	
	_REG_SELF_TEST_X        = 0x0D, / * Serial I-F = RW * /
	_REG_SELF_TEST_Y        = 0x0E, / * Serial I-F = RW * /
	_REG_SELF_TEST_Z        = 0x0F, / * Serial I-F = RW * /
	_REG_SELF_TEST_A        = 0x10, / * Serial I-F = RW * /
	_REG_SMPLRT_DIV         = 0x19, / * Serial I-F = RW * /
	_REG_CONFIG             = 0x1A, / * Serial I-F = RW * /
	_REG_GYRO_CONFIG        = 0x1B, / * Serial I-F = RW * /
	_REG_ACCEL_CONFIG       = 0x1C, / * Serial I-F = RW * /
	_REG_FIFO_EN            = 0x23, / * Serial I-F = RW * /
	_RES_I2C_MST_CTRL       = 0x24, / * Serial I-F = RW * /
	_REG_I2C_SLV0_ADDR      = 0x25, / * Serial I-F = RW * /
	_REG_I2C_SLV0_REG       = 0x26, / * Serial I-F = RW * /
	_REG_I2C_SLV0_CTRL      = 0x27, / * Serial I-F = RW * /
	_REG_I2C_SLV1_ADDR      = 0x28, / * Serial I-F = RW * /
	_REG_I2C_SLV1_REG       = 0x29, / * Serial I-F = RW * /
	_REG_I2C_SLV1_CTRL      = 0x2A, / * Serial I-F = RW * /
	_REG_I2C_SLV2_ADDR      = 0x2B, / * Serial I-F = RW * /
	_REG_I2C_SLV2_REG       = 0x2C, / * Serial I-F = RW * /
	_REG_I2C_SLV2_CTRL      = 0x2D, / * Serial I-F = RW * /
	_REG_I2C_SLV3_ADDR      = 0x2E, / * Serial I-F = RW * /
	_REG_I2C_SLV3_REG       = 0x2F, / * Serial I-F = RW * /
	_REG_I2C_SLV3_CTRL      = 0x30, / * Serial I-F = RW * /
	_REG_I2C_SLV4_ADDR      = 0x31, / * Serial I-F = RW * /
	_REG_I2C_SLV4_REG       = 0x32, / * Serial I-F = RW * /
	_REG_I2C_SLV4_DO        = 0x33, / * Serial I-F = RW * /
	_REG_I2C_SLV4_CTRL      = 0x34, / * Serial I-F = RW * /
	_REG_I2C_SLV4_DI        = 0x35, / * Serial I-F = R  * /
	_REG_I2C_MST_STATUS     = 0x36, / * Serial I-F = R  * /
	_REG_INT_PIN_CFG        = 0x37, / * Serial I-F = RW * /
	_REG_INT_ENABLE         = 0x38, / * Serial I-F = RW * /
	_REG_INT_STATUS         = 0x3A, / * Serial I-F = R  * /
	_REG_ACCEL_XOUT_H       = 0x3B, / * Serial I-F = R  * /
	_REG_ACCEL_XOUT_L       = 0x3C, / * Serial I-F = R  * /
	_REG_ACCEL_YOUT_H       = 0x3D, / * Serial I-F = R  * /
	_REG_ACCEL_YOUT_L       = 0x3E, / * Serial I-F = R  * /
	_REG_ACCEL_ZOUT_H       = 0x3F, / * Serial I-F = R  * /
	_REG_ACCEL_ZOUT_L       = 0x40, / * Serial I-F = R  * /
	_REG_TEMP_OUT_H         = 0x41, / * Serial I-F = R  * /
	_REG_TEMP_OUT_L         = 0x42, / * Serial I-F = R  * /
	_REG_GYRO_XOUT_H        = 0x43, / * Serial I-F = R  * /
	_REG_GYRO_XOUT_L        = 0x44, / * Serial I-F = R  * /
	_REG_GYRO_YOUT_H        = 0x45, / * Serial I-F = R  * /
	_REG_GYRO_YOUT_L        = 0x46, / * Serial I-F = R  * /
	_REG_GYRO_ZOUT_H        = 0x47, / * Serial I-F = R  * /
	_REG_GYRO_ZOUT_L        = 0x48, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_00   = 0x49, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_01   = 0x4A, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_02   = 0x4B, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_03   = 0x4C, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_04   = 0x4D, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_05   = 0x4E, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_06   = 0x4F, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_07   = 0x50, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_08   = 0x51, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_09   = 0x52, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_10   = 0x53, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_11   = 0x54, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_12   = 0x55, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_13   = 0x56, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_14   = 0x57, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_15   = 0x58, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_16   = 0x59, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_17   = 0x5A, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_18   = 0x5B, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_19   = 0x5C, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_20   = 0x5D, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_21   = 0x5E, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_22   = 0x5F, / * Serial I-F = R  * /
	_REG_EXT_SENS_DATA_23   = 0x60, / * Serial I-F = R  * /
	_REG_I2C_SLV0_DO        = 0x63, / * Serial I-F = RW * /
	_REG_I2C_SLV1_DO        = 0x64, / * Serial I-F = RW * /
	_REG_I2C_SLV2_DO        = 0x65, / * Serial I-F = RW * /
	_REG_I2C_SLV3_DO        = 0x66, / * Serial I-F = RW * /
	_REG_I2C_MST_DELAY_CTRL = 0x67, / * Serial I-F = RW * /
	_REG_SIGNAL_PATH_RESET  = 0x68, / * Serial I-F = RW * /
	_REG_USER_CTRL          = 0x6A, / * Serial I-F = RW * /
	_REG_PWR_MGMT_1         = 0x6B, / * Serial I-F = RW * /
	_REG_PWR_MGMT_2         = 0x6C, / * Serial I-F = RW * /
	_REG_FIFO_COUNTH        = 0x72, / * Serial I-F = RW * /
	_REG_FIFO_COUNTL        = 0x73, / * Serial I-F = RW * /
	_REG_FIFO_R_W           = 0x74, / * Serial I-F = RW * /
	_REG_WHO_AM_I           = 0x75  / * Serial I-F = R  * /
	
}ENUM_U8_T(MPU6050_REGISTER_ADDRESS_t);
*/

/* -------------------------------------------------- Struct ------------------------------------------------- */
typedef struct /* Struct for config interrupt in IC */
{
	
	MPU6050_INT_CFG_t IntLevel   : 1; /* this bit set the logic level for the INT pin */
	MPU6050_INT_CFG_t IntOpen    : 1; /* this bit set the INT pin mode (Push pull / Open drain) */
	MPU6050_INT_CFG_t LatchIntEn : 1; /* this bit set the INT pin pulse mode */
	
}MPU6050_INT_CFG_OPT_t;

typedef struct /* Struct for config IC */
{
	
	uint8_t                SampleRateDivider; /* The Sample Rate is determined by dividing the gyroscope output rate by this value. */
	
	MPU6050_EXT_SYNC_SET_t ExtSync;  /* Configures the FSYNC pin sampling. */
	MPU6050_DLPF_CFG_t     DigitalLowPassFilter; /* Configures the DLPF setting. */
	MPU6050_GFS_SEL_t      GyroFullScaleRange; /* Selects the full scale range of gyroscopes. */
	MPU6050_AFS_SEL_t      AccelFullScaleRange; /* Selects the full scale range of accelerometers. */
	MPU6050_INT_CFG_OPT_t  InterruptConfig; /* Config interrupt option */
	MPU6050_INT_ENABLE_t   InterruptEnable; /* Interrupt enable */
	MPU6050_CLKSEL_t       ClockSelection; /* Set clock source */
	
}MPU6050_Config_t;

extern MPU6050_Config_t MPU6050_Config; /* Struct for config IC */

/* ------------------------------------------------ Prototype ------------------------------------------------ */
/* ............................ Check .......................... */
uint8_t MPU6050_IsReady(uint16_t _time_out); /* Function for check connection */
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_IsReady(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ......................... Initialize ........................ */
uint8_t MPU6050_Init(uint16_t _time_out); /* Function for initialize MPU6050 */
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_Init(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_AutoInit(uint16_t _time_out); /* Function for initialize MPU6050 */
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_AutoInit(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_DefInit(uint16_t _time_out); /* Function for initialize MPU6050 */
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_DefInit(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_Reset(uint16_t _time_out); /* Function for reset MPU6050 */
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_Reset(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ....................... Configuration ....................... */
uint8_t MPU6050_SetDeviceID(uint8_t _id, uint16_t _time_out); /* Function for set MPU6050 id */
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_SetDeviceID(16, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetDeviceID(uint8_t *_id, uint16_t _time_out); /* Function for get MPU6050 id */
/*
  Example :
  
           uint8_t status;
           uint8_t devID;
           
		   -> status = MPU6050_GetDeviceID(&devID, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ....................... Get Raw Value ....................... */
/* ------------- Accel ------------ */
uint8_t MPU6050_GetRawAccelX(int16_t *raw_accelx_value, uint16_t _time_out); /* Function for take Accelerometer (x) value */
/*
  Example :
  
           uint8_t status;
		   int16_t x;
           
		   -> status = MPU6050_GetRawAccelX( &x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetRawAccelY(int16_t *raw_accely_value, uint16_t _time_out); /* Function for take Accelerometer (y) value */
/*
  Example :
  
           uint8_t status;
		   int16_t y;
           
		   -> status = MPU6050_GetRawAccelY( &y, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetRawAccelZ(int16_t *raw_accelz_value, uint16_t _time_out); /* Function for take Accelerometer (z) value */
/*
  Example :
  
           uint8_t status;
		   int16_t z;
           
		   -> status = MPU6050_GetRawAccelZ( &z, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetRawAccel( int16_t *raw_accel_str, uint16_t _time_out); /* Function for take Accelerometer value */
/*
  Example :
  
           uint8_t status;
		   int16_t x[3];
           
		   -> status = MPU6050_GetRawAccel( x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ------------- Temp ------------- */
uint8_t MPU6050_GetRawTemp(int16_t *raw_temp_value, uint16_t _time_out); /* Function for take Temperature value */
/*
  Example :
  
           uint8_t status;
		   int16_t t;
           
		   -> status = MPU6050_GetRawTemp( &t, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ------------- Gyro ------------- */
uint8_t MPU6050_GetRawGyroX(int16_t *raw_gyrox_value, uint16_t _time_out); /* Function for take Gyroscope (x) value */
/*
  Example :
  
           uint8_t status;
		   int16_t x;
           
		   -> status = MPU6050_GetRawGyroX( &x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetRawGyroY(int16_t *raw_gyroy_value, uint16_t _time_out); /* Function for take Gyroscope (y) value */
/*
  Example :
  
           uint8_t status;
		   int16_t y;
           
		   -> status = MPU6050_GetRawGyroY( &y, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetRawGyroZ(int16_t *raw_gyroz_value, uint16_t _time_out); /* Function for take Gyroscope (z) value */
/*
  Example :
  
           uint8_t status;
		   int16_t z;
           
		   -> status = MPU6050_GetRawGyroZ( &z, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetRawGyro( int16_t *raw_gyro_str, uint16_t _time_out); /* Function for take Gyroscope value */
/*
  Example :
  
           uint8_t status;
		   int16_t x[3];
           
		   -> status = MPU6050_GetRawGyro( x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ......................... Get Value ......................... */
/* ------------- Accel ------------ */
uint8_t MPU6050_GetAccelX(float *accelx_value, uint16_t _time_out); /* Function for take Accelerometer (x) value */
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = MPU6050_GetAccelX( &x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetAccelY(float *accely_value, uint16_t _time_out); /* Function for take Accelerometer (y) value */
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = MPU6050_GetAccelY( &y, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetAccelZ(float *accelz_value, uint16_t _time_out); /* Function for take Accelerometer (z) value */
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = MPU6050_GetAccelZ( &z, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetAccel( float *accel_str, uint16_t _time_out); /* Function for take Accelerometer value */
/*
  Example :
  
           uint8_t status;
		   float x[3];
           
		   -> status = MPU6050_GetAccel( x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

/* ------------- Temp ------------- */
uint8_t MPU6050_GetTemp(float *temp_value, uint16_t _time_out); /* Function for take Temperature value */
/*
  Example :
  
           uint8_t status;
		   float t;
           
		   -> status = MPU6050_GetTemp( &t, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ------------- Gyro ------------- */
uint8_t MPU6050_GetGyroX(float *gyrox_value, uint16_t _time_out); /* Function for take Gyroscope (x) value */
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = MPU6050_GetGyroX( &x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetGyroY(float *gyroy_value, uint16_t _time_out); /* Function for take Gyroscope (y) value */
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = MPU6050_GetGyroY( &y, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetGyroZ(float *gyroz_value, uint16_t _time_out); /* Function for take Gyroscope (z) value */
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = MPU6050_GetGyroZ( &z, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetGyro( float *gyro_str, uint16_t _time_out); /* Function for take Gyroscope value */
/*
  Example :
  
           uint8_t status;
		   float x[3];
           
		   -> status = MPU6050_GetGyro( x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ................. Angle with accelerometer .................. */
uint8_t MPU6050_GetAccelAngleX(float *ang_x, uint16_t _time_out); /* Function for take x angle */
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = MPU6050_GetAngleX( &x, 100 );
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetAccelAngleY(float *ang_y, uint16_t _time_out); /* Function for take y angle */
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = MPU6050_GetAngleY( &y, 100 );
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetAccelAngleZ(float *ang_z, uint16_t _time_out); /* Function for take z angle */
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = MPU6050_GetAngleZ( &z, 100 );
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

#endif /* __MPU6050_H_ */
