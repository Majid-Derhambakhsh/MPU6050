/*
------------------------------------------------------------------------------
~ File   : mpu6050.c
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

#include "MPU6050.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Var ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static uint8_t g_mpu_com_resp = _MPU_ERROR;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static struct MPU6050_Parameter_t
{
	
	float GyroSensitivity;
	float AccelSensitivity;
	
}MPU6050_Parameter;

MPU6050_Config_t MPU6050_Config;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ===================================================================== */
uint8_t MPU6050_SingleWrite(uint8_t address , uint8_t udata, uint16_t _time_out) /* Function for send Single Byte Data to MPU6050 register */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	g_mpu_com_resp = _MPU_ERROR; /* Variable for check status */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( MPU6050_IsReady(_time_out) == _MPU_OK ) /* Check Connection */
	{
		g_mpu_com_resp = _I2C_MEM_WRITE(_MPU6050_ADD, address, _MPU6050_MEMADD_SIZE, &udata, 1, _time_out); /* Write data */
	}
		
	return g_mpu_com_resp;
	
}
/*
  Example :
           
		   uint8_t status;
		   
		   -> status = MPU6050_SingleWrite( _REG_INT_ENABLE, 0x00, 100);
		   -> status = MPU6050_SingleWrite( 12, 0x00, 100);
		   -> status = MPU6050_SingleWrite( _REG_INT_PIN_CFG, 8, 100);
		   
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ---------------------------- */
uint8_t MPU6050_SingleRead(uint8_t address , uint8_t *udata, uint16_t _time_out) /* Function for read Single Byte Data from MPU6050 register */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	g_mpu_com_resp = _MPU_ERROR; /* Variable for check status */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( MPU6050_IsReady(_time_out) == _MPU_OK ) /* Check Connection */
	{
		g_mpu_com_resp = _I2C_MEM_READ(_MPU6050_ADD, address, _MPU6050_MEMADD_SIZE, udata, 1, _time_out); /* Write data */
	}
	
	return g_mpu_com_resp;
	
}
/*
  Example :
           
		   uint8_t status;
		   uint8_t udata;
		   
		   -> status = MPU6050_SingleRead( _REG_INT_ENABLE, &udata, 100);
		   -> status = MPU6050_SingleRead( 12, &udata, 100);
		   
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ---------------------------- */
uint8_t MPU6050_BurstWrite(uint8_t address, uint8_t *udata, uint8_t size, uint16_t _time_out) /* Function for send Burst Byte Data to MPU6050 register */
{
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	g_mpu_com_resp = _MPU_ERROR; /* Variable for check status */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( MPU6050_IsReady(_time_out) == _MPU_OK ) /* Check Connection */
	{
		
		g_mpu_com_resp = _I2C_MEM_WRITE(_MPU6050_ADD, address, _MPU6050_MEMADD_SIZE, udata, size, _time_out); /* Write data */
		
	}

	return g_mpu_com_resp;

}
/*
  Example :
           
		   uint8_t status;
		   
		   -> status = MPU6050_BurstWrite(_REG_INT_ENABLE, array_x, 3, 100 );
		   -> status = MPU6050_BurstWrite(0x68, array_x, 10, 100 );
		   
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ---------------------------- */
uint8_t MPU6050_BurstRead(uint8_t address , uint8_t *udata, uint8_t size, uint16_t _time_out) /* Function for read Burst Byte Data from MPU6050 register */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	g_mpu_com_resp = _MPU_ERROR; /* Variable for check status */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( MPU6050_IsReady(_time_out) == _MPU_OK ) /* Check Connection */
	{
		
		g_mpu_com_resp = _I2C_MEM_READ(_MPU6050_ADD, address, _MPU6050_MEMADD_SIZE, udata, size, _time_out); /* Write data */
		
	}
	
	return g_mpu_com_resp;

}
/*
  Example :
           
		   uint8_t status;
		   uint8_t array_x[];
		   
		   -> status = MPU6050_BurstRead(_REG_GYRO_XOUT_H, array_x, 2, 100 );
		   -> status = MPU6050_BurstRead(0x68, array_x, 10, 100 );
		   
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ===================================================================== */
void MPU6050_SetParameters(void)
{
	
	/* ..... Set accelerator sensitivity ..... */
	switch (MPU6050_Config.AccelFullScaleRange) /* Check Scale Range */
	{
		case _ACCEL_FULL_SCALE_RANGE_2G:
		{
			MPU6050_Parameter.AccelSensitivity = _MPU_ACCEL_SENS_2G_SENS;
		}
		break;
		case _ACCEL_FULL_SCALE_RANGE_4G:
		{
			MPU6050_Parameter.AccelSensitivity = _MPU_ACCEL_SENS_4G_SENS;
		}
		break;
		case _ACCEL_FULL_SCALE_RANGE_8G:
		{
			MPU6050_Parameter.AccelSensitivity = _MPU_ACCEL_SENS_8G_SENS;
		}
		break;
		case _ACCEL_FULL_SCALE_RANGE_16G:
		{
			MPU6050_Parameter.AccelSensitivity = _MPU_ACCEL_SENS_16G_SENS;
		}
		break;
		
		default: /* Default Scale */
		{
			MPU6050_Parameter.AccelSensitivity = _MPU_ACCEL_SENS_2G_SENS;
		}
		break;
	}
	
	/* ..... Set gyroscope sensitivity ..... */
	switch (MPU6050_Config.GyroFullScaleRange) /* Check Scale Range */
	{
		case _GYRO_FULL_SCALE_RANGE_250:
		{
			MPU6050_Parameter.GyroSensitivity = _MPU_GYRO_SENS_250_SENS;
		}
		break;
		case _GYRO_FULL_SCALE_RANGE_500:
		{
			MPU6050_Parameter.GyroSensitivity = _MPU_GYRO_SENS_500_SENS;
		}
		break;
		case _GYRO_FULL_SCALE_RANGE_1000:
		{
			MPU6050_Parameter.GyroSensitivity = _MPU_GYRO_SENS_1000_SENS;
		}
		break;
		case _GYRO_FULL_SCALE_RANGE_2000:
		{
			MPU6050_Parameter.GyroSensitivity = _MPU_GYRO_SENS_2000_SENS;
		}
		break;
		
		default: /* Default Scale */
		{
			MPU6050_Parameter.GyroSensitivity = _MPU_GYRO_SENS_250_SENS;
		}
		break;
	}
	
}

/* ............................ Check .......................... */
uint8_t MPU6050_IsReady(uint16_t _time_out) /* Function for check connection */
{
	return _I2C_MEM_READY(_MPU6050_ADD,_MPU_TRIALS,_time_out); /* Check device */
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_IsReady(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ......................... Initialize ........................ */
uint8_t MPU6050_Init(uint16_t _time_out) /* Function for initialize MPU6050 */
{
	/* ----------- Initialize Register ----------- */
	g_mpu_com_resp  = (uint8_t)MPU6050_SingleWrite( _REG_INT_ENABLE, MPU6050_Config.InterruptEnable, _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_SMPLRT_DIV, (MPU6050_Config.SampleRateDivider - 1), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_GYRO_CONFIG, (MPU6050_Config.GyroFullScaleRange << _MPU_REG_BIT_3), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_ACCEL_CONFIG, (MPU6050_Config.AccelFullScaleRange << _MPU_REG_BIT_3), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_CONFIG, ( (MPU6050_Config.ExtSync << _MPU_REG_BIT_3) | MPU6050_Config.DigitalLowPassFilter ), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_INT_PIN_CFG, ( (MPU6050_Config.InterruptConfig.IntLevel << _MPU_REG_BIT_7) | (MPU6050_Config.InterruptConfig.IntOpen << _MPU_REG_BIT_6) | (MPU6050_Config.InterruptConfig.LatchIntEn << _MPU_REG_BIT_5) ), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_PWR_MGMT_1, MPU6050_Config.ClockSelection, _time_out); 
	
	/* ------------------------------------------- */
	
	if ( g_mpu_com_resp == (_MPU_INIT_PARAM_ALL * _MPU_OK) ) /* The steps are complete */
	{
		/* ..... Set parameters ..... */
		MPU6050_SetParameters();
		
		/* ..... Return response ..... */
		return _MPU_OK;
		
	}

	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_Init(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_AutoInit(uint16_t _time_out) /* Function for initialize MPU6050 */
{
	/* ------------------------------------------- */
	#ifndef USE_HAL_DRIVER
		I2C_Init(); /* Initialize I2C */
	#endif
	
	/* ----------- Config MPU60X0 ----------- */
	MPU6050_Config.SampleRateDivider          = _MPU_CLOCK_DIVIDER_8; /* Set divider to 8 */
	MPU6050_Config.DigitalLowPassFilter       = _DLPF_CFG_260A_256G_HZ;/* Set DLPF */
	MPU6050_Config.InterruptEnable            = _INT_DATA_RDY_EN; /* Enable data ready interrupt */
	MPU6050_Config.ExtSync                    = _ES_INPUT_DISABLE; /* Disable external sync */
	MPU6050_Config.InterruptConfig.IntOpen    = _INT_OPEN_PUSH_PULL; /* Set pin mode to push pull */
	MPU6050_Config.InterruptConfig.IntLevel   = _INT_LEVEL_ACTIVE_HIGH; /* Set pin level to Active high */
	MPU6050_Config.InterruptConfig.LatchIntEn = _LATCH_INT_EN_50US_PULSE; /* Set the INT pin emits a 50us long pulse */
	MPU6050_Config.GyroFullScaleRange         = _GYRO_FULL_SCALE_RANGE_2000; /* Full scale range +/- 2000 degree/S */
	MPU6050_Config.AccelFullScaleRange        = _ACCEL_FULL_SCALE_RANGE_16G; /* Full scale range 16g */
	MPU6050_Config.ClockSelection             = _CLKSEL_X_AXIS_GYROSCOPE_REFERENCE; /* X axis gyroscope reference frequency */
	
	/* ----------- Initialize Register ----------- */
	g_mpu_com_resp  = (uint8_t)MPU6050_SingleWrite( _REG_INT_ENABLE, MPU6050_Config.InterruptEnable, _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_SMPLRT_DIV, (MPU6050_Config.SampleRateDivider - 1), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_GYRO_CONFIG, (MPU6050_Config.GyroFullScaleRange << _MPU_REG_BIT_3), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_ACCEL_CONFIG, (MPU6050_Config.AccelFullScaleRange << _MPU_REG_BIT_3), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_CONFIG, ( (MPU6050_Config.ExtSync << _MPU_REG_BIT_3) | MPU6050_Config.DigitalLowPassFilter ), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_INT_PIN_CFG, ( (MPU6050_Config.InterruptConfig.IntLevel << _MPU_REG_BIT_7) | (MPU6050_Config.InterruptConfig.IntOpen << _MPU_REG_BIT_6) | (MPU6050_Config.InterruptConfig.LatchIntEn << _MPU_REG_BIT_5) ), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_PWR_MGMT_1, MPU6050_Config.ClockSelection, _time_out); 
	
	/* ------------------------------------------- */
	if ( g_mpu_com_resp == (_MPU_INIT_PARAM_ALL * _MPU_OK) ) /* The steps are complete */
	{
		/* ..... Set parameters ..... */
		MPU6050_SetParameters();
		
		/* ..... Return response ..... */
		return _MPU_OK;
		
	}

	return _MPU_ERROR;
	 
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_AutoInit(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_DefInit(uint16_t _time_out) /* Function for initialize MPU6050 */
{
	/* ------------------------------------------- */
	#ifndef USE_HAL_DRIVER
		I2C_Init(); /* Initialize I2C */
	#endif
	
	/* ----------- Config MPU60X0 ----------- */
	MPU6050_Config.ClockSelection             = _CLKSEL_X_AXIS_GYROSCOPE_REFERENCE; /* X axis gyroscope reference frequency */
	MPU6050_Config.GyroFullScaleRange         = _GYRO_FULL_SCALE_RANGE_250; /* Full scale range +/- 250 degree/S */
	MPU6050_Config.AccelFullScaleRange        = _ACCEL_FULL_SCALE_RANGE_2G; /* Full scale range 2g */

	/* ----------- Initialize Register ----------- */
	g_mpu_com_resp  = (uint8_t)MPU6050_SingleWrite( _REG_PWR_MGMT_1, MPU6050_Config.ClockSelection, _time_out); 
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_GYRO_CONFIG, (MPU6050_Config.GyroFullScaleRange << _MPU_REG_BIT_3), _time_out);
	g_mpu_com_resp += (uint8_t)MPU6050_SingleWrite( _REG_ACCEL_CONFIG, (MPU6050_Config.AccelFullScaleRange << _MPU_REG_BIT_3), _time_out);

	/* ------------------------------------------- */
	if ( g_mpu_com_resp == (_MPU_INIT_PARAM_DEF * _MPU_OK) ) /* The steps are complete */
	{
		/* ..... Set parameters ..... */
		MPU6050_SetParameters();
		
		/* ..... Return response ..... */
		return _MPU_OK;
		
	}

	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_DefInit(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_DMPInit() /* Function for initialize MPU6050 DMP */
{
	// 	/* ..... Reset MPU ..... */
	// 	MPU6050_Reset();
	// 	_DELAY_MS(_MPU_WAKEUP_TIME_MS); // wait after reset
	//
	// 	MPU6050_DisableSleepMode();
	//
	// 	/* get MPU hardware revision */
	// 	MPU6050_SetMemoryBank(0x10, true, true);
	// 	MPU6050_SetMemoryStartAddress(0x06);
	// 	MPU6050_SetMemoryBank(0, false, false);
	//
	// 	/* setup weird slave stuff (?) */
	// 	MPU6050_SetSlaveAddress(0, 0x7F);
	// 	MPU6050_SetI2CMasterModeEnabled(false);
	// 	MPU6050_SetSlaveAddress(0, 0x68);
	// 	MPU6050_ResetI2CMaster();
	// 	_DELAY_MS(_MPU_WAKEUP_TIME_MS);
	// 	MPU6050_SetClockSource(_CLKSEL_Z_AXIS_GYROSCOPE_REFERENCE);
	//
	// 	MPU6050_SetIntEnabled(1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT | 1 << MPU6050_INTERRUPT_DMP_INT_BIT);
	//
	// 	MPU6050_SetRate(4); /* 1khz / (1 + 4) = 200 Hz */
	//
	// 	MPU6050_SetExternalFrameSync(_ES_TEMP_OUT_L);
	//
	// 	MPU6050_SetDLPFMode(_DLPF_CFG_44A_42G_HZ);
	//
	// 	MPU6050_SetFullScaleGyroRange(_GYRO_FULL_SCALE_RANGE_2000);
	//
	// 	if ( !MPU6050_WriteProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE) )
	// 	{
	// 		return 1;
	// 	}
	//
	// 	/* Set the FIFO Rate Divisor int the DMP Firmware Memory */
	// 	uint8_t dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
	// 	MPU6050_WriteMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16); /* Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16 */
	//
	// 	/* write start address MSB into register */
	// 	MPU6050_SetDMPConfig1(0x03);
	// 	/* write start address LSB into register */
	// 	MPU6050_SetDMPConfig2(0x00);
	//
	// 	MPU6050_SetOTPBankValid(false);
	//
	// 	MPU6050_SetMotionDetectionThreshold(2);
	//
	// 	MPU6050_SetZeroMotionDetectionThreshold(156);
	//
	// 	MPU6050_SetMotionDetectionDuration(80);
	//
	// 	MPU6050_SetZeroMotionDetectionDuration(0);
	// 	MPU6050_SetFIFOEnabled(true);
	//
	// 	MPU6050_ResetDMP();
	//
	// 	MPU6050_SetDMPEnabled(false);
	//
	// 	dmpPacketSize = 42;
	//
	// 	MPU6050_ResetFIFO();
	// 	MPU6050_GetIntStatus();
	
	return 0; /* success */
}

uint8_t MPU6050_Reset(uint16_t _time_out) /* Function for reset MPU6050 */
{
	
	/* ----------- Create variable ----------- */
	uint8_t mpuRegData = 0;
	
	g_mpu_com_resp = _MPU_ERROR;
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_SingleRead(_REG_PWR_MGMT_1, &mpuRegData, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		mpuRegData |= (1 << _MPU_REG_PWR_MGMT_1_DEVICE_RESET_BIT);
		
		g_mpu_com_resp = (uint8_t)MPU6050_SingleWrite( _REG_PWR_MGMT_1, mpuRegData, _time_out);
		
	}
	
	return g_mpu_com_resp;
	
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_Reset(100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ....................... Configuration ....................... */
uint8_t MPU6050_SetDeviceID(uint8_t _id, uint16_t _time_out) /* Function for set MPU6050 id */
{
	
	_id = ((_id & _MPU6050_DEV_ID_BITS) << 1);
	
	return (uint8_t)MPU6050_SingleWrite( _REG_WHO_AM_I, _id, _time_out);
	
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = MPU6050_SetDeviceID(16, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetDeviceID(uint8_t *_id, uint16_t _time_out) /* Function for get MPU6050 id */
{
	
	/* ----------- Create variable ----------- */
	uint8_t mpuDevID = 0;
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_SingleRead(_REG_WHO_AM_I, &mpuDevID, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		*_id = ((mpuDevID >> 1) & _MPU6050_DEV_ID_BITS);
		return _MPU_OK;
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
           uint8_t devID;
           
		   -> status = MPU6050_GetDeviceID(&devID, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ....................... Get Raw Value ....................... */
/* ------------- Accel ------------ */
uint8_t MPU6050_GetRawAccelX(int16_t *raw_accelx_value, uint16_t _time_out) /* Function for take Accelerometer (x) value */
{
	/* ----------- Create variable ----------- */
	uint8_t raw_accel_x[_MPU_AXIS_REG_LENGTH]; /* Array for take Accelerometer register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_ACCEL_XOUT_H, raw_accel_x, _MPU_AXIS_REG_LENGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		*raw_accelx_value = ( ( (int16_t)raw_accel_x[0] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_accel_x[1] ); /* Return Accelerometer value */
		return _MPU_OK;
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t x;
           
		   -> status = MPU6050_GetRawAccelX( &x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetRawAccelY(int16_t *raw_accely_value, uint16_t _time_out) /* Function for take Accelerometer (y) value */
{
	/* ----------- Create variable ----------- */
	uint8_t raw_accel_y[_MPU_AXIS_REG_LENGTH]; /* Array for take Accelerometer register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_ACCEL_YOUT_H, raw_accel_y, _MPU_AXIS_REG_LENGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		*raw_accely_value = ( ( (int16_t)raw_accel_y[0] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_accel_y[1] ); /* Return Accelerometer value */
		return _MPU_OK;
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t y;
           
		   -> status = MPU6050_GetRawAccelY( &y, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetRawAccelZ(int16_t *raw_accelz_value, uint16_t _time_out) /* Function for take Accelerometer (z) value */
{
	/* ----------- Create variable ----------- */
	uint8_t raw_accel_z[_MPU_AXIS_REG_LENGTH]; /* Array for take Accelerometer register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_ACCEL_ZOUT_H, raw_accel_z, _MPU_AXIS_REG_LENGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		*raw_accelz_value = ( ( (int16_t)raw_accel_z[0] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_accel_z[1] ); /* Return Accelerometer value */
		return _MPU_OK;
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t z;
           
		   -> status = MPU6050_GetRawAccelZ( &z, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetRawAccel( int16_t *raw_accel_str, uint16_t _time_out) /* Function for take Accelerometer value */
{
	/* ----------- Create variable ----------- */
	uint8_t write_counter = 0; /* Variable for count */
	uint8_t raw_accel[_MPU_AXIS_ALL_REG_LEGTH]; /* String for take value from registers */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_ACCEL_XOUT_H, raw_accel, _MPU_AXIS_ALL_REG_LEGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* --- Save Value in str --- */
		for ( ; write_counter < _MPU_AXIS_ALL_REG_LEGTH ; write_counter += _MPU_AXIS_REG_LENGTH ) /* Loop for save data to string */
		{
			
			*raw_accel_str = ( (int16_t)raw_accel[write_counter] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_accel[write_counter + 1]; /* Write data */
			raw_accel_str++; /* Select next word */
			
		}
		
		/* ------------------------- */
		
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t x[3];
           
		   -> status = MPU6050_GetRawAccel( x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ------------- Temp ------------- */
uint8_t MPU6050_GetRawTemp(int16_t *raw_temp_value, uint16_t _time_out) /* Function for take Temperature value */
{
	/* ----------- Create variable ----------- */
	uint8_t raw_temp[_MPU_TEMP_REG_LENGTH]; /* Array for take Temperature register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_TEMP_OUT_H, raw_temp, _MPU_TEMP_REG_LENGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		*raw_temp_value = ( ((int16_t)raw_temp[0] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_temp[1] ); /* Return Temperature */
		return _MPU_OK;
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t t;
           
		   -> status = MPU6050_GetRawTemp( &t, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ------------- Gyro ------------- */
uint8_t MPU6050_GetRawGyroX(int16_t *raw_gyrox_value, uint16_t _time_out) /* Function for take Gyroscope (x) value */
{
	/* ----------- Create variable ----------- */
	uint8_t raw_gyro_x[_MPU_AXIS_REG_LENGTH]; /* Array for take Gyroscope register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_GYRO_XOUT_H, raw_gyro_x, _MPU_AXIS_REG_LENGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		*raw_gyrox_value = ( ( (int16_t)raw_gyro_x[0] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_gyro_x[1] ); /* Return Gyroscope value */
		return _MPU_OK;
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t x;
           
		   -> status = MPU6050_GetRawGyroX( &x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetRawGyroY(int16_t *raw_gyroy_value, uint16_t _time_out) /* Function for take Gyroscope (y) value */
{
	/* ----------- Create variable ----------- */
	uint8_t raw_gyro_y[_MPU_AXIS_REG_LENGTH]; /* Array for take Gyroscope register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_GYRO_YOUT_H, raw_gyro_y, _MPU_AXIS_REG_LENGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		*raw_gyroy_value = ( ( (int16_t)raw_gyro_y[0] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_gyro_y[1] ); /* Return Gyroscope value */
		return _MPU_OK;
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t y;
           
		   -> status = MPU6050_GetRawGyroY( &y, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetRawGyroZ(int16_t *raw_gyroz_value, uint16_t _time_out) /* Function for take Gyroscope (z) value */
{
	/* ----------- Create variable ----------- */
	uint8_t raw_gyro_z[_MPU_AXIS_REG_LENGTH]; /* Array for take Gyroscope register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_GYRO_ZOUT_H, raw_gyro_z, _MPU_AXIS_REG_LENGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		*raw_gyroz_value = ( ( (int16_t)raw_gyro_z[0] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_gyro_z[1] );  /* Return Gyroscope value */
		return _MPU_OK;
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t z;
           
		   -> status = MPU6050_GetRawGyroZ( &z, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetRawGyro( int16_t *raw_gyro_str, uint16_t _time_out) /* Function for take Gyroscope value */
{
	
	/* ----------- Create variable ----------- */
	uint8_t write_counter; /* Variable for count */
	uint8_t raw_gyro[_MPU_AXIS_ALL_REG_LEGTH]; /* String for take value from registers */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_GYRO_XOUT_H, raw_gyro, _MPU_AXIS_ALL_REG_LEGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* --- Save Value in str --- */
		for ( write_counter = 0 ; write_counter < _MPU_AXIS_ALL_REG_LEGTH ; write_counter += _MPU_AXIS_REG_LENGTH ) /* Loop for save data to string */
		{
			
			*raw_gyro_str = ( (int16_t)raw_gyro[write_counter] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)raw_gyro[write_counter + 1]; /* Write data */
			raw_gyro_str++; /* Select next word */
			
		}
		
		/* ------------------------- */
		
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t x[3];
           
		   -> status = MPU6050_GetRawGyro( x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ......................... Get Value ......................... */
/* ------------- Accel ------------ */
uint8_t MPU6050_GetAccelX(float *accelx_value, uint16_t _time_out) /* Function for take Accelerometer (x) value */
{
	/* ----------- Create variable ----------- */
	int16_t accel_x; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_GetRawAccelX( &accel_x, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------------------------------- */
		*accelx_value = ( (float)accel_x / MPU6050_Parameter.AccelSensitivity);
		
		/* ------------------------------- */
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = MPU6050_GetAccelX( &x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetAccelY(float *accely_value, uint16_t _time_out) /* Function for take Accelerometer (y) value */
{
	/* ----------- Create variable ----------- */
	int16_t accel_y; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_GetRawAccelY( &accel_y, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------------------------------- */
		*accely_value = ( (float)accel_y / MPU6050_Parameter.AccelSensitivity);
		
		/* ------------------------------- */
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = MPU6050_GetAccelY( &y, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetAccelZ(float *accelz_value, uint16_t _time_out) /* Function for take Accelerometer (z) value */
{
	/* ----------- Create variable ----------- */
	int16_t accel_z; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_GetRawAccelZ( &accel_z, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------------------------------- */
		*accelz_value = ( (float)accel_z / MPU6050_Parameter.AccelSensitivity);
		
		/* ------------------------------- */
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = MPU6050_GetAccelZ( &z, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

uint8_t MPU6050_GetAccel( float *accel_str, uint16_t _time_out) /* Function for take Accelerometer value */
{
	/* ----------- Create variable ----------- */
	uint8_t write_counter = 0; /* Variable for count */
	int16_t accel[_MPU_AXIS_ALL]; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_GetRawAccel(accel, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------ Save Value in str ------ */
		for ( ; write_counter < _MPU_AXIS_ALL ; write_counter++ ) /* Loop for save data to string */
		{
			*accel_str = ( (float)accel[write_counter] / MPU6050_Parameter.AccelSensitivity);
			accel_str++; /* Select next word */
		}
		
		/* ------------------------------- */
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float x[3];
           
		   -> status = MPU6050_GetAccel( x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR 
		   
*/

/* ------------- Temp ------------- */
uint8_t MPU6050_GetTemp(float *temp_value, uint16_t _time_out) /* Function for take Temperature value */
{
	/* ----------- Create variable ----------- */
	uint8_t temp[_MPU_TEMP_REG_LENGTH]; /* Array for take Temperature register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_BurstRead(_REG_TEMP_OUT_H, temp, _MPU_TEMP_REG_LENGTH, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		*temp_value = ( (float)( ((int16_t)temp[0] << _MPU_HIGH_BYTE_SHIFT ) | (int16_t)temp[1] ) / _MPU_TEMP_DIV ) + _MPU_TEMP_CONST; /* Return Temperature */
		
		/* ------------------------------- */
		
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float t;
           
		   -> status = MPU6050_GetTemp( &t, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ------------- Gyro ------------- */
uint8_t MPU6050_GetGyroX(float *gyrox_value, uint16_t _time_out) /* Function for take Gyroscope (x) value */
{
	/* ----------- Create variable ----------- */
	int16_t gyro_x; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_GetRawGyroX( &gyro_x, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------------------------------- */
		*gyrox_value = ( (float)gyro_x / MPU6050_Parameter.GyroSensitivity);
		
		/* ------------------------------- */
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = MPU6050_GetGyroX( &x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetGyroY(float *gyroy_value, uint16_t _time_out) /* Function for take Gyroscope (y) value */
{
	/* ----------- Create variable ----------- */
	int16_t gyro_y; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_GetRawGyroY( &gyro_y, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------------------------------- */
		*gyroy_value = ( (float)gyro_y / MPU6050_Parameter.GyroSensitivity);
		
		/* ------------------------------- */
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = MPU6050_GetGyroY( &y, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetGyroZ(float *gyroz_value, uint16_t _time_out) /* Function for take Gyroscope (z) value */
{
	/* ----------- Create variable ----------- */
	int16_t gyro_z; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_GetRawGyroZ( &gyro_z, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------------------------------- */
		*gyroz_value = ( (float)gyro_z / MPU6050_Parameter.GyroSensitivity);
		
		/* ------------------------------- */
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = MPU6050_GetGyroZ( &z, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetGyro( float *gyro_str, uint16_t _time_out) /* Function for take Gyroscope value */
{
	/* ----------- Create variable ----------- */
	uint8_t write_counter = 0; /* Variable for count */
	int16_t gyro[_MPU_AXIS_ALL]; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	if ( MPU6050_GetRawGyro(gyro, _time_out) == _MPU_OK ) /* The instructions are complete */
	{
		/* ------------------------------- */
		for ( ; write_counter < _MPU_AXIS_ALL ; write_counter++ ) /* Loop for save data to string */
		{
			*gyro_str = ( (float)gyro[write_counter] / MPU6050_Parameter.GyroSensitivity);
			gyro_str++; /* Select next word */
		}
		
		/* ------------------------------- */
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
}
/*
  Example :
  
           uint8_t status;
		   float x[3];
           
		   -> status = MPU6050_GetGyro( x, 100);
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ................. Angle with accelerometer .................. */
uint8_t MPU6050_GetAccelAngleX(float *ang_x, uint16_t _time_out) /* Function for take x angle */
{
	
	/* ----------- Create variable ----------- */
	int16_t ayRAW = 0; /* Variable for get accel value */
	int16_t azRAW = 0; /* Variable for get accel value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	g_mpu_com_resp  = MPU6050_GetRawAccelY(&ayRAW, _time_out); /* Get accel y value */
	g_mpu_com_resp += MPU6050_GetRawAccelZ(&azRAW, _time_out); /* Get accel z value */
	
	/* ----------- Save Value in str ----------- */
	if ( g_mpu_com_resp == (_MPU_OK * _MPU_AXIS_REG_LENGTH) ) /* The instructions are complete */
	{
		
		ayRAW = Map(ayRAW , _MPU_ANG_MIN , _MPU_ANG_MAX , _MPU_ANG_NEG , _MPU_ANG_POS); /* Calculating the MAP */
		azRAW = Map(azRAW , _MPU_ANG_MIN , _MPU_ANG_MAX , _MPU_ANG_NEG , _MPU_ANG_POS); /* Calculating the MAP */
		
		*ang_x = ( _MPU_RAD_TO_DEG * ( atan2( -ayRAW , -azRAW ) + _MATH_PI ) ); /* Export new value */
		
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
	/* Function End */
}
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = MPU6050_GetAngleX( &x, 100 );
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetAccelAngleY(float *ang_y, uint16_t _time_out) /* Function for take y angle */
{
	
	/* ----------- Create variable ----------- */
	int16_t axRAW = 0; /* Variable for get accel value */
	int16_t azRAW = 0; /* Variable for get accel value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	g_mpu_com_resp  = MPU6050_GetRawAccelX(&axRAW, _time_out); /* Get accel x value */
	g_mpu_com_resp += MPU6050_GetRawAccelZ(&azRAW, _time_out); /* Get accel z value */
	
	/* ----------- Save Value in str ----------- */
	if ( g_mpu_com_resp == (_MPU_OK * _MPU_AXIS_REG_LENGTH) ) /* The instructions are complete */
	{
		
		axRAW = Map(axRAW , _MPU_ANG_MIN , _MPU_ANG_MAX , _MPU_ANG_NEG , _MPU_ANG_POS); /* Calculating the MAP */
		azRAW = Map(azRAW , _MPU_ANG_MIN , _MPU_ANG_MAX , _MPU_ANG_NEG , _MPU_ANG_POS); /* Calculating the MAP */
		
		*ang_y = ( _MPU_RAD_TO_DEG * ( atan2( -axRAW , -azRAW ) + _MATH_PI ) ); /* Export new value */
		
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
	/* Function End */
}
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = MPU6050_GetAngleY( &y, 100 );
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

uint8_t MPU6050_GetAccelAngleZ(float *ang_z, uint16_t _time_out) /* Function for take z angle */
{
	
	/* ----------- Create variable ----------- */
	int16_t axRAW = 0; /* Variable for get accel value */
	int16_t ayRAW = 0; /* Variable for get accel value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	g_mpu_com_resp  = MPU6050_GetRawAccelX(&axRAW, _time_out); /* Get accel x value */
	g_mpu_com_resp += MPU6050_GetRawAccelY(&ayRAW, _time_out); /* Get accel y value */
	
	/* ----------- Save Value in str ----------- */
	
	if ( g_mpu_com_resp == (_MPU_OK * _MPU_AXIS_REG_LENGTH) ) /* The instructions are complete */
	{
		
		axRAW = Map(axRAW , _MPU_ANG_MIN , _MPU_ANG_MAX , _MPU_ANG_NEG , _MPU_ANG_POS); /* Calculating the MAP */
		ayRAW = Map(ayRAW , _MPU_ANG_MIN , _MPU_ANG_MAX , _MPU_ANG_NEG , _MPU_ANG_POS); /* Calculating the MAP */
		
		*ang_z = ( _MPU_RAD_TO_DEG * ( atan2( -ayRAW , -axRAW ) + _MATH_PI ) ); /* Export new value */
		
		return _MPU_OK;
		
	}
	
	return _MPU_ERROR;
	
	/* Function End */
}
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = MPU6050_GetAngleZ( &z, 100 );
		      
		     -> status is _MPU_OK/_MPU_ERROR
		   
*/

/* ===================================================================== */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Program End */