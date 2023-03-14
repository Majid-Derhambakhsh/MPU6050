/*
------------------------------------------------------------------------------
~ File   : mpu6050.c
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "mpu6050.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
const uint16_t MPU_ACCEL_SENS_VALUE[4] = {16384U, 8192U, 4096U, 2048U};
const float    MPU_GYRO_SENS_VALUE[4]  = {131.0F, 65.5F, 32.8F, 16.4F};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
MPU_StatusTypeDef MPU6050_SingleWrite(MPU_TypeDef *MPUx, uint8_t Address , uint8_t uData, uint16_t Timeout)
{
	#ifdef USE_HAL_DRIVER
	
	return I2C_MEM_WRITE(MPUx->I2Cx, MPUx->Address, Address, MPU_MEMADD_SIZE, &uData, 1, Timeout);
	
	#elif
	
	return I2C_MEM_WRITE(MPUx->Address, Address, MPU_MEMADD_SIZE, &uData, 1, Timeout);
	
	#endif // USE_HAL_DRIVER
}

MPU_StatusTypeDef MPU6050_SingleRead(MPU_TypeDef *MPUx, uint8_t Address , uint8_t *uData, uint16_t Timeout)
{
	#ifdef USE_HAL_DRIVER
	
	return I2C_MEM_READ(MPUx->I2Cx, MPUx->Address, Address, MPU_MEMADD_SIZE, uData, 1, Timeout);
	
	#elif
	
	return I2C_MEM_READ(MPUx->Address, Address, MPU_MEMADD_SIZE, uData, 1, Timeout);
	
	#endif // USE_HAL_DRIVER
}

MPU_StatusTypeDef MPU6050_BurstWrite(MPU_TypeDef *MPUx, uint8_t Address, uint8_t *uData, uint8_t Size, uint16_t Timeout)
{
	#ifdef USE_HAL_DRIVER
	
	return I2C_MEM_WRITE(MPUx->I2Cx, MPUx->Address, Address, MPU_MEMADD_SIZE, uData, Size, Timeout);
	
	#elif
	
	return I2C_MEM_WRITE(MPUx->Address, Address, MPU_MEMADD_SIZE, uData, Size, Timeout);
	
	#endif // USE_HAL_DRIVER
}

MPU_StatusTypeDef MPU6050_BurstRead(MPU_TypeDef *MPUx, uint8_t Address , uint8_t *uData, uint8_t Size, uint16_t Timeout)
{
	#ifdef USE_HAL_DRIVER
	
	return I2C_MEM_READ(MPUx->I2Cx, MPUx->Address, Address, MPU_MEMADD_SIZE, uData, Size, Timeout);
	
	#elif
	
	return I2C_MEM_READ(MPUx->Address, Address, MPU_MEMADD_SIZE, uData, Size, Timeout);
	
	#endif // USE_HAL_DRIVER
}

/* ............................ Check .......................... */
MPU_StatusTypeDef MPU6050_IsReady(MPU_TypeDef *MPUx, uint8_t Trials, uint16_t Timeout)
{
	#ifdef USE_HAL_DRIVER
	
	return I2C_IS_DEVICE_READY(MPUx->I2Cx, MPUx->Address, Trials, Timeout);
	
	#elif
	
	return I2C_IS_DEVICE_READY(MPUx->Address, Trials, Timeout);
	
	#endif // USE_HAL_DRIVER
}

MPU_StatusTypeDef MPU6050_Reset(MPU_TypeDef *MPUx, uint16_t Timeout)
{
	
	uint8_t regData = 0;
	
	MPU_StatusTypeDef status = MPU_ERROR;
	
	/* ----------- Read current status ----------- */
	if (MPU6050_SingleRead(MPUx, MPU_REG_PWR_MGMT_1, &regData, Timeout) == MPU_OK)
	{
		
		regData |= (1 << MPU_DEVICE_RESET_BIT);
		
		status = (MPU_StatusTypeDef)MPU6050_SingleWrite(MPUx, MPU_REG_PWR_MGMT_1, regData, Timeout);
		
	}
	
	return status;
	
}

/* ......................... Initialize ........................ */
MPU_StatusTypeDef MPU6050_Init(MPU_TypeDef *MPUx, uint16_t Timeout)
{
	
	MPU_StatusTypeDef status = MPU_ERROR;
	
	/* ---------- Initialize Parameters ---------- */
	status  = MPU6050_SingleWrite(MPUx, MPU_REG_INT_ENABLE, MPUx->InterruptEnable, Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_SMPLRT_DIV, (MPUx->SampleRateDivider - 1), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_GYRO_CONFIG, (MPUx->GyroFullScaleRange << MPU_BIT_3), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_ACCEL_CONFIG, (MPUx->AccelFullScaleRange << MPU_BIT_3), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_CONFIG, ((MPUx->ExtSync << MPU_BIT_3) | MPUx->DigitalLowPassFilter), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_INT_PIN_CFG, ((MPUx->InterruptConfig.IntLevel << MPU_BIT_7) | (MPUx->InterruptConfig.IntOpen << MPU_BIT_6) | (MPUx->InterruptConfig.LatchIntEn << MPU_BIT_5)), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_PWR_MGMT_1, MPUx->ClockSelection, Timeout);
	
	/* ------------------------------------------- */
	if (status == (MPU_INIT_PARAMS_ALL * MPU_OK))
	{
		
		/* ..... Set parameters ..... */
		MPUx->AccelSensitivity = MPU_ACCEL_SENS_VALUE[MPUx->AccelFullScaleRange];
		MPUx->GyroSensitivity  = MPU_GYRO_SENS_VALUE[MPUx->GyroFullScaleRange];
		
		/* ..... Return response ..... */
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

MPU_StatusTypeDef MPU6050_AutoInit(MPU_TypeDef *MPUx, uint16_t Timeout)
{
	
	MPU_StatusTypeDef status = MPU_ERROR;
	
	/* ------------------------------------------- */
	#ifndef USE_HAL_DRIVER
		I2C_Init(); /* Initialize I2C */
	#endif
	
	/* ---------- Set default parameter ---------- */
	MPUx->SampleRateDivider          = MPU_CLOCK_DIVIDER_8; /* Set divider to 8 */
	MPUx->DigitalLowPassFilter       = MPU_DLPF_CFG_260A_256G_HZ; /* Set DLPF */
	MPUx->InterruptEnable            = MPU_INT_DATA_RDY_EN; /* Enable data ready interrupt */
	MPUx->ExtSync                    = MPU_ES_INPUT_DISABLE; /* Disable external sync */
	MPUx->InterruptConfig.IntOpen    = MPU_INT_OPEN_PUSH_PULL; /* Set pin mode to push pull */
	MPUx->InterruptConfig.IntLevel   = MPU_INT_LEVEL_ACTIVE_HIGH; /* Set pin level to Active high */
	MPUx->InterruptConfig.LatchIntEn = MPU_LATCH_INT_EN_50US_PULSE; /* Set the INT pin emits a 50us long pulse */
	MPUx->GyroFullScaleRange         = MPU_GYRO_FULL_SCALE_RANGE_2000; /* Full scale range +/- 2000 degree/S */
	MPUx->AccelFullScaleRange        = MPU_ACCEL_FULL_SCALE_RANGE_16G; /* Full scale range 16g */
	MPUx->ClockSelection             = MPU_CLKSEL_X_AXIS_GYROSCOPE_REFERENCE; /* X axis gyroscope reference frequency */
	
	/* ---------- Initialize Parameters ---------- */
	status  = MPU6050_SingleWrite(MPUx, MPU_REG_INT_ENABLE, MPUx->InterruptEnable, Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_SMPLRT_DIV, (MPUx->SampleRateDivider - 1), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_GYRO_CONFIG, (MPUx->GyroFullScaleRange << MPU_BIT_3), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_ACCEL_CONFIG, (MPUx->AccelFullScaleRange << MPU_BIT_3), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_CONFIG, ((MPUx->ExtSync << MPU_BIT_3) | MPUx->DigitalLowPassFilter), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_INT_PIN_CFG, ((MPUx->InterruptConfig.IntLevel << MPU_BIT_7) | (MPUx->InterruptConfig.IntOpen << MPU_BIT_6) | (MPUx->InterruptConfig.LatchIntEn << MPU_BIT_5)), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_PWR_MGMT_1, MPUx->ClockSelection, Timeout);
	
	/* ------------------------------------------- */
	if (status == (MPU_INIT_PARAMS_ALL * MPU_OK))
	{
		
		/* ..... Set parameters ..... */
		MPUx->AccelSensitivity = MPU_ACCEL_SENS_VALUE[MPUx->AccelFullScaleRange];
		MPUx->GyroSensitivity  = MPU_GYRO_SENS_VALUE[MPUx->GyroFullScaleRange];
		
		/* ..... Return response ..... */
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
		 
}

MPU_StatusTypeDef MPU6050_DefInit(MPU_TypeDef *MPUx, uint16_t Timeout)
{
	
	MPU_StatusTypeDef status = MPU_ERROR;
	
	/* ------------------------------------------- */
	#ifndef USE_HAL_DRIVER
		I2C_Init(); /* Initialize I2C */
	#endif
	
	/* ---------- Set default parameter ---------- */
	MPUx->ClockSelection      = MPU_CLKSEL_X_AXIS_GYROSCOPE_REFERENCE; /* X axis gyroscope reference frequency */
	MPUx->GyroFullScaleRange  = MPU_GYRO_FULL_SCALE_RANGE_250; /* Full scale range +/- 250 degree/S */
	MPUx->AccelFullScaleRange = MPU_ACCEL_FULL_SCALE_RANGE_2G; /* Full scale range 2g */

	/* ----------- Initialize Register ----------- */
	status  = MPU6050_SingleWrite(MPUx, MPU_REG_PWR_MGMT_1, MPUx->ClockSelection, Timeout); 
	status += MPU6050_SingleWrite(MPUx, MPU_REG_GYRO_CONFIG, (MPUx->GyroFullScaleRange << MPU_BIT_3), Timeout);
	status += MPU6050_SingleWrite(MPUx, MPU_REG_ACCEL_CONFIG, (MPUx->AccelFullScaleRange << MPU_BIT_3), Timeout);

	/* ------------------------------------------- */
	if (status == (MPU_INIT_PARAMS_DEF * MPU_OK))
	{
		
		/* ..... Set parameters ..... */
		MPUx->AccelSensitivity = MPU_ACCEL_SENS_VALUE[MPUx->AccelFullScaleRange];
		MPUx->GyroSensitivity  = MPU_GYRO_SENS_VALUE[MPUx->GyroFullScaleRange];
		
		/* ..... Return response ..... */
		return MPU_OK;
		
	}

	return MPU_ERROR;
	
}

/* ....................... Configuration ....................... */
MPU_StatusTypeDef MPU6050_SetDeviceID(MPU_TypeDef *MPUx, uint8_t ID, uint16_t Timeout)
{
	
	ID = __MPU_SET_ID(ID);
	
	return MPU6050_SingleWrite(MPUx, MPU_REG_WHO_AM_I, ID, Timeout);
	
}

MPU_StatusTypeDef MPU6050_GetDeviceID(MPU_TypeDef *MPUx, uint8_t *ID, uint16_t Timeout)
{
		
	/* -------------- Read Device ID ------------- */
	if (MPU6050_SingleRead(MPUx, MPU_REG_WHO_AM_I, ID, Timeout) == MPU_OK)
	{
		
		*ID = __MPU_GET_ID(*ID);
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

/* ....................... Get Raw Value ....................... */
MPU_StatusTypeDef MPU6050_GetRawAccel(MPU_TypeDef *MPUx, MPU_RawTypeDef *AccelRaw, uint16_t Timeout)
{
	
	uint8_t regData[MPU_ACCEL_GYRO_DATA_REG_LEN];
	
	/* ---------------- Read Data ---------------- */
	if (MPU6050_BurstRead(MPUx, MPU_REG_ACCEL_XOUT_H, regData, MPU_ACCEL_GYRO_DATA_REG_LEN, Timeout) == MPU_OK)
	{
		
		AccelRaw->X = ((int16_t)regData[0] << MPU_HIGH_BYTE_SHIFT) | (int16_t)regData[1];
		AccelRaw->Y = ((int16_t)regData[2] << MPU_HIGH_BYTE_SHIFT) | (int16_t)regData[3];
		AccelRaw->Z = ((int16_t)regData[4] << MPU_HIGH_BYTE_SHIFT) | (int16_t)regData[5];
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

MPU_StatusTypeDef MPU6050_GetRawGyro(MPU_TypeDef *MPUx, MPU_RawTypeDef *GyroRaw, uint16_t Timeout)
{
	
	uint8_t regData[MPU_ACCEL_GYRO_DATA_REG_LEN];
	
	/* ---------------- Read Data ---------------- */
	if (MPU6050_BurstRead(MPUx, MPU_REG_GYRO_XOUT_H, regData, MPU_ACCEL_GYRO_DATA_REG_LEN, Timeout) == MPU_OK)
	{
		
		GyroRaw->X = ((int16_t)regData[0] << MPU_HIGH_BYTE_SHIFT) | (int16_t)regData[1];
		GyroRaw->Y = ((int16_t)regData[2] << MPU_HIGH_BYTE_SHIFT) | (int16_t)regData[3];
		GyroRaw->Z = ((int16_t)regData[4] << MPU_HIGH_BYTE_SHIFT) | (int16_t)regData[5];
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

MPU_StatusTypeDef MPU6050_GetRawTemp(MPU_TypeDef *MPUx, int16_t *Temp, uint16_t Timeout)
{
	
	uint8_t regData[MPU_TEMP_REG_LEN];
	
	/* ---------------- Read Data ---------------- */
	if (MPU6050_BurstRead(MPUx, MPU_REG_TEMP_OUT_H, regData, MPU_TEMP_REG_LEN, Timeout) == MPU_OK)
	{
		
		*Temp = ((int16_t)regData[0] << MPU_HIGH_BYTE_SHIFT) | (int16_t)regData[1];
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
		
}

/* ......................... Get Value ......................... */
MPU_StatusTypeDef MPU6050_GetAccel(MPU_TypeDef *MPUx, MPU_XYZTypeDef *Accel, uint16_t Timeout)
{
	
	MPU_RawTypeDef rawData;
	
	/* ---------------- Read Data ---------------- */
	if (MPU6050_GetRawAccel(MPUx, &rawData, Timeout) == MPU_OK)
	{
		
		Accel->X = ((float)rawData.X / MPUx->AccelSensitivity);
		Accel->Y = ((float)rawData.Y / MPUx->AccelSensitivity);
		Accel->Z = ((float)rawData.Z / MPUx->AccelSensitivity);
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

MPU_StatusTypeDef MPU6050_GetGyro(MPU_TypeDef *MPUx, MPU_XYZTypeDef *Gyro, uint16_t Timeout)
{
	
	MPU_RawTypeDef rawData;
	
	/* ---------------- Read Data ---------------- */
	if (MPU6050_GetRawGyro(MPUx, &rawData, Timeout) == MPU_OK)
	{
		
		Gyro->X = ((float)rawData.X / MPUx->GyroSensitivity);
		Gyro->Y = ((float)rawData.Y / MPUx->GyroSensitivity);
		Gyro->Z = ((float)rawData.Z / MPUx->GyroSensitivity);
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

MPU_StatusTypeDef MPU6050_GetTemp(MPU_TypeDef *MPUx, float *Temp, uint16_t Timeout)
{
	
	uint8_t regData[MPU_TEMP_REG_LEN];
	
	/* ---------------- Read Data ---------------- */
	if (MPU6050_BurstRead(MPUx, MPU_REG_TEMP_OUT_H, regData, MPU_TEMP_REG_LEN, Timeout) == MPU_OK)
	{
		
		*Temp = ((float)(((int16_t)regData[0] << MPU_HIGH_BYTE_SHIFT) | (int16_t)regData[1]) / MPU_TEMP_RAW_TO_C_RATIO) + MPU_TEMP_RAW_TO_C_OFFSET;
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

/* ................. Angle with accelerometer .................. */
MPU_StatusTypeDef MPU6050_GetRoll(MPU_TypeDef *MPUx, float *Roll, uint16_t Timeout)
{
	
	MPU_RawTypeDef rawData;
	
	if (MPU6050_GetRawAccel(MPUx, &rawData, Timeout) == MPU_OK)
	{
		
		rawData.Y = Map(rawData.Y , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		rawData.Z = Map(rawData.Z , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		
		*Roll = (MPU_RAD_TO_DEG * (atan2(-rawData.Y, -rawData.Z) + _MATH_PI));
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

MPU_StatusTypeDef MPU6050_GetPitch(MPU_TypeDef *MPUx, float *Pitch, uint16_t Timeout)
{
	
	MPU_RawTypeDef rawData;
	
	if (MPU6050_GetRawAccel(MPUx, &rawData, Timeout) == MPU_OK)
	{
		
		rawData.X = Map(rawData.X , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		rawData.Z = Map(rawData.Z , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		
		*Pitch = (MPU_RAD_TO_DEG * (atan2(-rawData.X, -rawData.Z) + _MATH_PI));
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

MPU_StatusTypeDef MPU6050_GetYaw(MPU_TypeDef *MPUx, float *Yaw, uint16_t Timeout)
{
	
	MPU_RawTypeDef rawData;
	
	if (MPU6050_GetRawAccel(MPUx, &rawData, Timeout) == MPU_OK)
	{
		
		rawData.X = Map(rawData.X , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		rawData.Y = Map(rawData.Y , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		
		*Yaw = (MPU_RAD_TO_DEG * (atan2(-rawData.Y, -rawData.X) + _MATH_PI));
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

MPU_StatusTypeDef MPU6050_GetRPY(MPU_TypeDef *MPUx, MPU_RPYTypeDef *RPY, uint16_t Timeout)
{
	
	MPU_RawTypeDef rawData;
	MPU_RawTypeDef tmpData;
	
	if (MPU6050_GetRawAccel(MPUx, &rawData, Timeout) == MPU_OK)
	{
		
		/* Calculate Roll */
		tmpData.Y = Map(rawData.Y , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		tmpData.Z = Map(rawData.Z , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		
		RPY->Roll = (MPU_RAD_TO_DEG * (atan2(-tmpData.Y, -tmpData.Z) + _MATH_PI));
		
		/* Calculate Pitch */
		tmpData.X = Map(rawData.X , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		tmpData.Z = Map(rawData.Z , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		
		RPY->Pitch = (MPU_RAD_TO_DEG * (atan2(-tmpData.X, -tmpData.Z) + _MATH_PI));
		
		/* Calculate Yaw */
		tmpData.X = Map(rawData.X , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		tmpData.Y = Map(rawData.Y , MPU_ANG_MIN , MPU_ANG_MAX , MPU_ANG_NEG , MPU_ANG_POS);
		
		RPY->Yaw = (MPU_RAD_TO_DEG * (atan2(-tmpData.Y, -tmpData.X) + _MATH_PI));
		
		return MPU_OK;
		
	}
	
	return MPU_ERROR;
	
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
