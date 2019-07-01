/*
------------------------------------------------------------------------------
~ File   : MPU6050.c
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

#include "MPU6050.h"

/********************************* Struct **********************************/

Mpu6050_Config_t Mpu6050_Config;

/******************************** Functions ********************************/

/* =================================================================== */

uint8_t Mpu6050_SingleWrite(uint8_t register_address , uint8_t register_data) /* Function for send Single Byte Data to MPU6050 register */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint8_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( Mpu6050_ConnectionTest() == _TRUE ) /* Check Connection */
	{
		
		i2c_status = I2C_BeginTransmission(); /* Begin Transmission */
		
		/* ------------------ */
		
		if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( _MPU6050_ADDRESS_W | _AD0_LEVEL ); /* Send MPU6050 Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_Transmit( register_address ); /* Send MPU6050 Register Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_Transmit( register_data ); /* Send value to register */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			I2C_EndTransmission(); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
			
		/* --------------------------------- */
		
		if ( step_check == _SINGLE_WRITE_STEPS ) /* The steps are complete */
		{
			return _TRUE;
		}
		
	}
		
	return _FALSE;
	
}
/*
  Example :
           
		   uint8_t status;
		   
		   -> status = Mpu6050_SingleWrite( _REG_INT_ENABLE , 0x00 );
		   -> status = Mpu6050_SingleWrite( 12 , 0x00 );
		   -> status = Mpu6050_SingleWrite( _REG_INT_PIN_CFG , 8 );
		   
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_SingleRead(uint8_t register_address , uint8_t *data) /* Function for read Single Byte Data from MPU6050 register */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint8_t reg_data = 0; /* Variable for take register data */
	uint8_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( Mpu6050_ConnectionTest() == _TRUE ) /* Check Connection */
	{
		
		i2c_status = I2C_BeginTransmission(); /* Begin Transmission */
		
		/* ------------------ */
		
		if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( _MPU6050_ADDRESS_W | _AD0_LEVEL ); /* Send MPU6050 Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_Transmit( register_address ); /* Send MPU6050 Register Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_BeginTransmission(); /* Repeat Start */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_REP_START_TRANSMITTED ) /* A repeated START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( _MPU6050_ADDRESS_R | _AD0_LEVEL ); /* Send MPU6050 Address */
			step_check++; /* The step is completed */
		}
		else{}
			
		/* --------------------------------- */
		
		if ( i2c_status == _MR_SLA_R_TRANSMITTED_ACK ) /* SLA+R has been transmitted, and ACK has been received. */
		{
			reg_data = I2C_ReceiveNACK(); /* Receive Data with send NACK */	
			i2c_status = I2C_Status(); /* I2C status take */
			step_check++; /* The step is completed */
		}
		else{}
				
		/* --------------------------------- */
		
		if ( i2c_status == _MR_DATA_RECEIVED_NACK ) /* Data byte has been received; NOT ACK has been returned */
		{
			I2C_EndTransmission(); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( step_check == _SINGLE_READ_STEPS ) /* The steps are complete */
		{
			*data = reg_data; /* Save value in data variable */
			return _TRUE;
		}
		
	}
	
	return _FALSE;
	
}
/*
  Example :
           
		   uint8_t status;
		   uint8_t register_data;
		   
		   -> status = Mpu6050_SingleRead( _REG_INT_ENABLE , &register_data );
		   -> status = Mpu6050_SingleRead( 12 , &register_data );
		   
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_BurstWrite(uint8_t *str , uint8_t start_register , uint8_t quantity) /* Function for send Burst Byte Data to MPU6050 register */
{
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint8_t write_quantity = quantity; /* Variable for check write quantity */
	uint8_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( Mpu6050_ConnectionTest() == _TRUE ) /* Check Connection */
	{
		
		i2c_status = I2C_BeginTransmission(); /* Begin Transmission */
		
		/* ------------------ */
		
		if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( _MPU6050_ADDRESS_W | _AD0_LEVEL ); /* Send MPU6050 Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_Transmit( start_register ); /* Send MPU6050 Register Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		for ( ; quantity > 0 ; quantity-- ) /* Loop for write data to register */
		{
			
			if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
			{
				
				i2c_status = I2C_Transmit( *str ); /* Send value to register */
				step_check++; /* The step is completed */
				str++; /* Select next byte */
			
			}
			else{}
				
		}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			I2C_EndTransmission(); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
			
		/* --------------------------------- */
		
		if ( step_check == (_BURST_WRITE_STEPS + write_quantity) ) /* The steps are complete */
		{
			return _TRUE;
		}
		
	}

	return _FALSE;

}
/*
  Example :
           
		   uint8_t status;
		   
		   -> status = Mpu6050_BurstWrite( array_x , _REG_INT_ENABLE , 3 );
		   -> status = Mpu6050_BurstWrite( array_x , 0x68 , 10 );
		   
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_BurstRead(uint8_t *str , uint8_t start_register , uint8_t quantity ) /* Function for read Burst Byte Data from MPU6050 register */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint8_t read_quantity = quantity; /* Variable for check read quantity */
	uint8_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( Mpu6050_ConnectionTest() == _TRUE ) /* Check Connection */
	{
		
		i2c_status = I2C_BeginTransmission(); /* Begin Transmission */
		
		/* ------------------ */
		
		if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( _MPU6050_ADDRESS_W | _AD0_LEVEL ); /* Send MPU6050 Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_Transmit( start_register ); /* Send MPU6050 Register Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_BeginTransmission(); /* Repeat Start */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_REP_START_TRANSMITTED ) /* A repeated START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( _MPU6050_ADDRESS_R | _AD0_LEVEL ); /* Send MPU6050 Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		for ( ; quantity > 1 ; quantity-- ) /* Loop for write data to register */
		{
			
			if ( ( i2c_status == _MR_SLA_R_TRANSMITTED_ACK ) || ( i2c_status == _MR_DATA_RECEIVED_ACK ) ) /* SLA+R/DATA has been transmitted, and ACK has been received. */
			{
				
				*str = I2C_ReceiveACK(); /* Receive Data with send ACK */
				i2c_status = I2C_Status(); /* I2C status take */
				step_check++; /* The step is completed */
				str++; /* Select next byte */
				
			}
			else{}
				
		}
		
		/* --------------------------------- */
		
		if ( ( i2c_status == _MR_SLA_R_TRANSMITTED_ACK ) || ( i2c_status == _MR_DATA_RECEIVED_ACK ) ) /* SLA+R/DATA has been transmitted, and ACK has been received. */
		{
			
			*str = I2C_ReceiveNACK(); /* Receive Data with send NACK */
			i2c_status = I2C_Status(); /* I2C status take */
			step_check++; /* The step is completed */
		
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MR_DATA_RECEIVED_NACK ) /* Data byte has been received; NOT ACK has been returned */
		{
			I2C_EndTransmission(); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
			
		/* --------------------------------- */
		
		if ( step_check == (_BURST_READ_STEPS + read_quantity) ) /* The steps are complete */
		{
			return _TRUE;
		}
		
	}
	
	return _FALSE;

}
/*
  Example :
           
		   uint8_t status;
		   uint8_t array_x[];
		   
		   -> status = Mpu6050_BurstRead( array_x , _REG_GYRO_XOUT_H , 2 );
		   -> status = Mpu6050_BurstRead( array_x , 0x68 , 10 );
		   
		     -> status is _TRUE/_FALSE
		   
*/

/* =================================================================== */

uint8_t Mpu6050_ConnectionTest(void) /* Function for check connection */
{
		
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	i2c_status = I2C_BeginTransmission(); /* Start condition */
	
	/* ------------------ */
	
	if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
	{
		
		i2c_status = I2C_Transmit( _MPU6050_ADDRESS_W | _AD0_LEVEL ); /* Send MPU6050 Address */
		
	}
	else /* Start condition is lost */
	{
		/* ------------------ */
		
		i2c_status = I2C_BeginTransmission(); /* Repeat Start condition */
		
		/* ------------------ */
		
		while ( !(i2c_status == _MT_REP_START_TRANSMITTED) ) /* Repeat START condition has been transmitted */
		{
			
			i2c_status = I2C_BeginTransmission(); /* Repeat Start condition */
			
		}
		
		/* ------------------ */
		
		i2c_status = I2C_Transmit( _MPU6050_ADDRESS_W | _AD0_LEVEL ); /* Send MPU6050 Address */
	
	}
	
	/* --------------------------------- */
	
	if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted and ACK has been received */
	{
		/* ------------------ */
		
		I2C_EndTransmission(); /* End Transmission */
		
		/* ------------------ */
		return _TRUE;
		
	}
	else /* SLA+W has been transmitted and NACK has been received */
	{
		/* ------------------ */
		
		I2C_EndTransmission(); /* End Transmission */
		
		/* ------------------ */
		return _FALSE;
		
	}
	
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = Mpu6050_ConnectionTest();
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */

uint8_t Mpu6050_Init(void) /* Function for initialize MPU6050 */
{
	/* ------------------------------------------- */
	
	uint16_t instruction_status = 0; /* Variable for check instruction status */

	/* ----------- Initialize Register ----------- */
	
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_INT_ENABLE , Mpu6050_Config.InterruptEnable );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_PWR_MGMT_1 , Mpu6050_Config.ClockSelection ); 
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_SMPLRT_DIV , (Mpu6050_Config.SampleRateDivider - 1) );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_GYRO_CONFIG , (Mpu6050_Config.GyroFullScaleRange << _BIT_3) );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_ACCEL_CONFIG , (Mpu6050_Config.AccelFullScaleRange << _BIT_3) );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_CONFIG , ( (Mpu6050_Config.ExtSync << _BIT_3) | Mpu6050_Config.DigitalLowPassFilter ) );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_INT_PIN_CFG , ( (Mpu6050_Config.InterruptConfig.IntLevel << _BIT_7) | (Mpu6050_Config.InterruptConfig.IntOpen << _BIT_6) | (Mpu6050_Config.InterruptConfig.LatchIntEn << _BIT_5) ) );
	
	/* ------------------------------------------- */
	
	if ( instruction_status == (_INITIALIZE_INSTRUCTIONS * _TRUE) ) /* The steps are complete */
	{
		return _TRUE;
	}

	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = Mpu6050_Init();
		      
		     -> status is _TRUE/_FALSE           
		   
*/

uint8_t Mpu6050_AutoInit(void) /* Function for initialize MPU6050 */
{
	/* ------------------------------------------- */
	
	uint16_t instruction_status = 0; /* Variable for check instruction status */
	
	/* ------------------------------------------- */
	
	I2C_Init(); /* Initialize I2C */
	
	/* ----------- Config MPU60X0 ----------- */
	
	Mpu6050_Config.SampleRateDivider          = _DIVIDER_8; /* Set divider to 8 */
	Mpu6050_Config.DigitalLowPassFilter       = _DLPF_CFG_0; /* Set DLPF */
	Mpu6050_Config.InterruptEnable            = _INT_DATA_RDY_EN; /* Enable data ready interrupt */
	Mpu6050_Config.ExtSync                    = _ES_INPUT_DISABLE; /* Disable external sync */
	Mpu6050_Config.InterruptConfig.IntOpen    = _INT_OPEN_PUSH_PULL; /* Set pin mode to push pull */
	Mpu6050_Config.InterruptConfig.IntLevel   = _INT_LEVEL_ACTIVE_HIGH; /* Set pin level to Active high */
	Mpu6050_Config.InterruptConfig.LatchIntEn = _LATCH_INT_EN_50US_PULSE; /* Set the INT pin emits a 50us long pulse */
	Mpu6050_Config.GyroFullScaleRange         = _GYRO_FULL_SCALE_RANGE_2000; /* Full scale range +/- 2000 degree/S */
	Mpu6050_Config.AccelFullScaleRange        = _ACCEL_FULL_SCALE_RANGE_16G; /* Full scale range 16g */
	Mpu6050_Config.ClockSelection             = _CLKSEL_X_AXIS_GYROSCOPE_REFERENCE; /* X axis gyroscope reference frequency */
	
	/* ----------- Initialize Register ----------- */
	
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_INT_ENABLE , Mpu6050_Config.InterruptEnable );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_PWR_MGMT_1 , Mpu6050_Config.ClockSelection ); 
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_SMPLRT_DIV , (Mpu6050_Config.SampleRateDivider - 1) );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_GYRO_CONFIG , (Mpu6050_Config.GyroFullScaleRange << _BIT_3) );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_ACCEL_CONFIG , (Mpu6050_Config.AccelFullScaleRange << _BIT_3) );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_CONFIG , ( (Mpu6050_Config.ExtSync << _BIT_3) | Mpu6050_Config.DigitalLowPassFilter ) );
	instruction_status += (uint16_t)Mpu6050_SingleWrite( _REG_INT_PIN_CFG , ( (Mpu6050_Config.InterruptConfig.IntLevel << _BIT_7) | (Mpu6050_Config.InterruptConfig.IntOpen << _BIT_6) | (Mpu6050_Config.InterruptConfig.LatchIntEn << _BIT_5) ) );
	
	/* ------------------------------------------- */
	
	if ( instruction_status == (_INITIALIZE_INSTRUCTIONS * _TRUE) ) /* The steps are complete */
	{
		return _TRUE;
	}

	return _FALSE;
	 
}
/*
  Example :
  
           uint8_t status;
           
		   -> status = Mpu6050_AutoInit();
		      
		     -> status is _TRUE/_FALSE            
		   
*/

/* ^^^^^^^^^^^^^^^^^^^ Raw Value ^^^^^^^^^^^^^^^^^^^ */

uint8_t Mpu6050_GetRawAccelX(int16_t *raw_accelx_value) /* Function for take Accelerometer (x) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t raw_accel_x[_ONE_AXIS_REGISTERS]; /* Array for take Accelerometer register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_accel_x , _REG_ACCEL_XOUT_H , _ONE_AXIS_REGISTERS); /* Take value from Accelerometer register */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		*raw_accelx_value = ( ( (int16_t)raw_accel_x[0] << _HIGH_BYTE ) | (int16_t)raw_accel_x[1] ); /* Return Accelerometer value */
		return _TRUE;
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t x;
           
		   -> status = Mpu6050_GetRawAccelX( &x );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetRawAccelY(int16_t *raw_accely_value) /* Function for take Accelerometer (y) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t raw_accel_y[_ONE_AXIS_REGISTERS]; /* Array for take Accelerometer register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_accel_y , _REG_ACCEL_YOUT_H , _ONE_AXIS_REGISTERS); /* Take value from Accelerometer register */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		*raw_accely_value = ( ( (int16_t)raw_accel_y[0] << _HIGH_BYTE ) | (int16_t)raw_accel_y[1] ); /* Return Accelerometer value */
		return _TRUE;
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t y;
           
		   -> status = Mpu6050_GetRawAccelY( &y );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetRawAccelZ(int16_t *raw_accelz_value) /* Function for take Accelerometer (z) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t raw_accel_z[_ONE_AXIS_REGISTERS]; /* Array for take Accelerometer register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_accel_z , _REG_ACCEL_ZOUT_H , _ONE_AXIS_REGISTERS); /* Take value from Accelerometer register */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		*raw_accelz_value = ( ( (int16_t)raw_accel_z[0] << _HIGH_BYTE ) | (int16_t)raw_accel_z[1] ); /* Return Accelerometer value */
		return _TRUE;
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t z;
           
		   -> status = Mpu6050_GetRawAccelZ( &z );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetRawAccel( int16_t *raw_accel_str ) /* Function for take Accelerometer value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t write_counter; /* Variable for count */
	uint8_t raw_accel[_ALL_AXIS_REGISTERS]; /* String for take value from registers */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_accel , _REG_ACCEL_XOUT_H , _ALL_AXIS_REGISTERS); /* Take value from Accelerometer register */
	
	/* ----------- Save Value in str ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------- */
		
		for ( write_counter = 0 ; write_counter < _ALL_AXIS_REGISTERS ; write_counter += _ONE_AXIS_REGISTERS ) /* Loop for save data to string */
		{
			
			*raw_accel_str = ( (int16_t)raw_accel[write_counter] << _HIGH_BYTE ) | (int16_t)raw_accel[write_counter + 1]; /* Write data */
			raw_accel_str++; /* Select next word */
			
		}
		
		/* ------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t x[3];
           
		   -> status = Mpu6050_GetRawAccel( x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_GetRawTemp(int16_t *raw_temp_value) /* Function for take Temperature value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t raw_temp[_TEMP_REGISTERS]; /* Array for take Temperature register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_temp , _REG_TEMP_OUT_H , _TEMP_REGISTERS); /* Take value from Temperature register */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		*raw_temp_value = ( ((int16_t)raw_temp[0] << _HIGH_BYTE ) | (int16_t)raw_temp[1] ); /* Return Temperature */
		return _TRUE;
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t t;
           
		   -> status = Mpu6050_GetRawTemp( &t );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_GetRawGyroX(int16_t *raw_gyrox_value) /* Function for take Gyroscope (x) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t raw_gyro_x[_ONE_AXIS_REGISTERS]; /* Array for take Gyroscope register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_gyro_x , _REG_GYRO_XOUT_H , _ONE_AXIS_REGISTERS); /* Take value from Gyroscope register */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		*raw_gyrox_value = ( ( (int16_t)raw_gyro_x[0] << _HIGH_BYTE ) | (int16_t)raw_gyro_x[1] ); /* Return Gyroscope value */
		return _TRUE;
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t x;
           
		   -> status = Mpu6050_GetRawGyroX( &x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetRawGyroY(int16_t *raw_gyroy_value) /* Function for take Gyroscope (y) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t raw_gyro_y[_ONE_AXIS_REGISTERS]; /* Array for take Gyroscope register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_gyro_y , _REG_GYRO_YOUT_H , _ONE_AXIS_REGISTERS); /* Take value from Gyroscope register */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		*raw_gyroy_value = ( ( (int16_t)raw_gyro_y[0] << _HIGH_BYTE ) | (int16_t)raw_gyro_y[1] ); /* Return Gyroscope value */
		return _TRUE;
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t y;
           
		   -> status = Mpu6050_GetRawGyroY( &y );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetRawGyroZ(int16_t *raw_gyroz_value) /* Function for take Gyroscope (z) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t raw_gyro_z[_ONE_AXIS_REGISTERS]; /* Array for take Gyroscope register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_gyro_z , _REG_GYRO_ZOUT_H , _ONE_AXIS_REGISTERS); /* Take value from Gyroscope register */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		*raw_gyroz_value = ( ( (int16_t)raw_gyro_z[0] << _HIGH_BYTE ) | (int16_t)raw_gyro_z[1] );  /* Return Gyroscope value */
		return _TRUE;
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t z;
           
		   -> status = Mpu6050_GetRawGyroZ( &z );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetRawGyro( int16_t *raw_gyro_str ) /* Function for take Gyroscope value */
{
	
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t write_counter; /* Variable for count */
	uint8_t raw_gyro[_ALL_AXIS_REGISTERS]; /* String for take value from registers */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(raw_gyro , _REG_GYRO_XOUT_H , _ALL_AXIS_REGISTERS); /* Take value from Accelerometer register */
	
	/* ----------- Save Value in str ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------- */
		
		for ( write_counter = 0 ; write_counter < _ALL_AXIS_REGISTERS ; write_counter += _ONE_AXIS_REGISTERS ) /* Loop for save data to string */
		{
			
			*raw_gyro_str = ( (int16_t)raw_gyro[write_counter] << _HIGH_BYTE ) | (int16_t)raw_gyro[write_counter + 1]; /* Write data */
			raw_gyro_str++; /* Select next word */
			
		}
		
		/* ------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   int16_t x[3];
           
		   -> status = Mpu6050_GetRawGyro( x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ^^^^^^^^^^^^^^^^^^^^^ Value ^^^^^^^^^^^^^^^^^^^^^ */

uint8_t Mpu6050_GetAccelX(float *accelx_value) /* Function for take Accelerometer (x) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	int16_t accel_x; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_GetRawAccelX( &accel_x ); /* Take Raw value */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		switch (Mpu6050_Config.AccelFullScaleRange) /* Check Scale Range */
		{
			case _ACCEL_FULL_SCALE_RANGE_2G:
			{
				*accelx_value = ( (float)accel_x / _ACCEL_SENSITIVITY_2G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_4G:
			{
				*accelx_value = ( (float)accel_x / _ACCEL_SENSITIVITY_4G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_8G:
			{
				*accelx_value = ( (float)accel_x / _ACCEL_SENSITIVITY_8G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_16G:
			{
				*accelx_value = ( (float)accel_x / _ACCEL_SENSITIVITY_16G); /* Return Accelerometer value */
			}
			break;
			
			default: /* Default Scale */
			{
				*accelx_value = ( (float)accel_x / _ACCEL_SENSITIVITY_2G); /* Return Accelerometer value */
			}
			break;
		}
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = Mpu6050_GetAccelX( &x );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetAccelY(float *accely_value) /* Function for take Accelerometer (y) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	int16_t accel_y; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_GetRawAccelY( &accel_y ); /* Take Raw value */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		switch (Mpu6050_Config.AccelFullScaleRange) /* Check Scale Range */
		{
			case _ACCEL_FULL_SCALE_RANGE_2G:
			{
				*accely_value = ( (float)accel_y / _ACCEL_SENSITIVITY_2G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_4G:
			{
				*accely_value = ( (float)accel_y / _ACCEL_SENSITIVITY_4G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_8G:
			{
				*accely_value = ( (float)accel_y / _ACCEL_SENSITIVITY_8G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_16G:
			{
				*accely_value = ( (float)accel_y / _ACCEL_SENSITIVITY_16G); /* Return Accelerometer value */
			}
			break;
			
			default: /* Default Scale */
			{
				*accely_value = ( (float)accel_y / _ACCEL_SENSITIVITY_2G); /* Return Accelerometer value */
			}
			break;
		}
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = Mpu6050_GetAccelY( &y );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetAccelZ(float *accelz_value) /* Function for take Accelerometer (z) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	int16_t accel_z; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_GetRawAccelZ( &accel_z ); /* Take Raw value */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		switch (Mpu6050_Config.AccelFullScaleRange) /* Check Scale Range */
		{
			case _ACCEL_FULL_SCALE_RANGE_2G:
			{
				*accelz_value = ( (float)accel_z / _ACCEL_SENSITIVITY_2G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_4G:
			{
				*accelz_value = ( (float)accel_z / _ACCEL_SENSITIVITY_4G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_8G:
			{
				*accelz_value = ( (float)accel_z / _ACCEL_SENSITIVITY_8G); /* Return Accelerometer value */
			}
			break;
			case _ACCEL_FULL_SCALE_RANGE_16G:
			{
				*accelz_value = ( (float)accel_z / _ACCEL_SENSITIVITY_16G); /* Return Accelerometer value */
			}
			break;
			
			default: /* Default Scale */
			{
				*accelz_value = ( (float)accel_z / _ACCEL_SENSITIVITY_2G); /* Return Accelerometer value */
			}
			break;
		}
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = Mpu6050_GetAccelZ( &z );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

uint8_t Mpu6050_GetAccel( float *accel_str ) /* Function for take Accelerometer value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t write_counter; /* Variable for count */
	int16_t accel[_ALL_AXIS]; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_GetRawAccel(accel); /* Take Raw value */
	
	/* ----------- Save Value in str ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		switch (Mpu6050_Config.AccelFullScaleRange) /* Check Scale Range */
		{
			
			case _ACCEL_FULL_SCALE_RANGE_2G:
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*accel_str = ( (float)accel[write_counter] / _ACCEL_SENSITIVITY_2G);
					accel_str++; /* Select next word */
				}
				
			}
			break;
			
			case _ACCEL_FULL_SCALE_RANGE_4G:
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*accel_str = ( (float)accel[write_counter] / _ACCEL_SENSITIVITY_4G);
					accel_str++; /* Select next word */
					
				}
				
			}
			break;
			
			case _ACCEL_FULL_SCALE_RANGE_8G:
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*accel_str = ( (float)accel[write_counter] / _ACCEL_SENSITIVITY_8G);
					accel_str++; /* Select next word */
					
				}
				
			}
			break;
			
			case _ACCEL_FULL_SCALE_RANGE_16G:
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*accel_str = ( (float)accel[write_counter] / _ACCEL_SENSITIVITY_16G);
					accel_str++; /* Select next word */
				}
				
			}
			break;
			
			default: /* Default Scale */
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*accel_str = ( (float)accel[write_counter] / _ACCEL_SENSITIVITY_2G);
					accel_str++; /* Select next word */
					
				}
				
			}
			break;
			
		}
		/* End Switch */
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float x[3];
           
		   -> status = Mpu6050_GetAccel( x );
		      
		     -> status is _TRUE/_FALSE 
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_GetTemp(float *temp_value) /* Function for take Temperature value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t temp[_TEMP_REGISTERS]; /* Array for take Temperature register value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_BurstRead(temp , _REG_TEMP_OUT_H , _TEMP_REGISTERS); /* Take value from Temperature register */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		*temp_value = ( (float)( ((int16_t)temp[0] << _HIGH_BYTE ) | (int16_t)temp[1] ) / _TEMP_DIVIDER ) + _TEMP_CONST; /* Return Temperature */
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float t;
           
		   -> status = Mpu6050_GetTemp( &t );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ---------------------------- */

uint8_t Mpu6050_GetGyroX(float *gyrox_value) /* Function for take Gyroscope (x) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	int16_t gyro_x; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_GetRawGyroX( &gyro_x ); /* Take Raw value */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		switch (Mpu6050_Config.GyroFullScaleRange) /* Check Scale Range */
		{
			case _GYRO_FULL_SCALE_RANGE_250:
			{
				*gyrox_value = ( (float)gyro_x / _GYRO_SENSITIVITY_250); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_500:
			{
				*gyrox_value = ( (float)gyro_x / _GYRO_SENSITIVITY_500); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_1000:
			{
				*gyrox_value = ( (float)gyro_x / _GYRO_SENSITIVITY_1000); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_2000:
			{
				*gyrox_value = ( (float)gyro_x / _GYRO_SENSITIVITY_2000); /* Return Accelerometer value */
			}
			break;
			
			default: /* Default Scale */
			{
				*gyrox_value = ( (float)gyro_x / _GYRO_SENSITIVITY_2000); /* Return Accelerometer value */
			}
			break;
		}
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float x;
           
		   -> status = Mpu6050_GetGyroX( &x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetGyroY(float *gyroy_value) /* Function for take Gyroscope (y) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	int16_t gyro_y; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_GetRawGyroY( &gyro_y ); /* Take Raw value */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		switch (Mpu6050_Config.GyroFullScaleRange) /* Check Scale Range */
		{
			case _GYRO_FULL_SCALE_RANGE_250:
			{
				*gyroy_value = ( (float)gyro_y / _GYRO_SENSITIVITY_250); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_500:
			{
				*gyroy_value = ( (float)gyro_y / _GYRO_SENSITIVITY_500); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_1000:
			{
				*gyroy_value = ( (float)gyro_y / _GYRO_SENSITIVITY_1000); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_2000:
			{
				*gyroy_value = ( (float)gyro_y / _GYRO_SENSITIVITY_2000); /* Return Accelerometer value */
			}
			break;
			
			default: /* Default Scale */
			{
				*gyroy_value = ( (float)gyro_y / _GYRO_SENSITIVITY_2000); /* Return Accelerometer value */
			}
			break;
		}
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float y;
           
		   -> status = Mpu6050_GetGyroY( &y );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetGyroZ(float *gyroz_value) /* Function for take Gyroscope (z) value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	int16_t gyro_z; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_GetRawGyroZ( &gyro_z ); /* Take Raw value */
	
	/* ----------- Return Value ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		switch (Mpu6050_Config.GyroFullScaleRange) /* Check Scale Range */
		{
			case _GYRO_FULL_SCALE_RANGE_250:
			{
				*gyroz_value = ( (float)gyro_z / _GYRO_SENSITIVITY_250); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_500:
			{
				*gyroz_value = ( (float)gyro_z / _GYRO_SENSITIVITY_500); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_1000:
			{
				*gyroz_value = ( (float)gyro_z / _GYRO_SENSITIVITY_1000); /* Return Accelerometer value */
			}
			break;
			case _GYRO_FULL_SCALE_RANGE_2000:
			{
				*gyroz_value = ( (float)gyro_z / _GYRO_SENSITIVITY_2000); /* Return Accelerometer value */
			}
			break;
			
			default: /* Default Scale */
			{
				*gyroz_value = ( (float)gyro_z / _GYRO_SENSITIVITY_2000); /* Return Accelerometer value */
			}
			break;
		}
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float z;
           
		   -> status = Mpu6050_GetGyroZ( &z );
		      
		     -> status is _TRUE/_FALSE
		   
*/

uint8_t Mpu6050_GetGyro( float *gyro_str ) /* Function for take Gyroscope value */
{
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status = 0; /* Variable for check instruction status */
	uint8_t write_counter; /* Variable for count */
	int16_t gyro[_ALL_AXIS]; /* Variable for take Raw Accelerometer value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status = Mpu6050_GetRawGyro(gyro); /* Take Raw value */
	
	/* ----------- Save Value in str ----------- */
	
	if ( instruction_status == _TRUE ) /* The instructions are complete */
	{
		/* ------------------------------- */
		
		switch (Mpu6050_Config.GyroFullScaleRange) /* Check Scale Range */
		{
			case _GYRO_FULL_SCALE_RANGE_250:
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*gyro_str = ( (float)gyro[write_counter] / _GYRO_SENSITIVITY_250);
					gyro_str++; /* Select next word */
					
				}
				
			}
			break;
			
			case _GYRO_FULL_SCALE_RANGE_500:
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*gyro_str = ( (float)gyro[write_counter] / _GYRO_SENSITIVITY_500);
					gyro_str++; /* Select next word */
					
				}
				
			}
			break;
			
			case _GYRO_FULL_SCALE_RANGE_1000:
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*gyro_str = ( (float)gyro[write_counter] / _GYRO_SENSITIVITY_1000);
					gyro_str++; /* Select next word */
					
				}
				
			}
			break;
			
			case _GYRO_FULL_SCALE_RANGE_2000:
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*gyro_str = ( (float)gyro[write_counter] / _GYRO_SENSITIVITY_2000);
					gyro_str++; /* Select next word */
					
				}
				
			}
			break;
			
			default: /* Default Scale */
			{
				
				for ( write_counter = 0 ; write_counter < _ALL_AXIS ; write_counter++ ) /* Loop for save data to string */
				{
					
					*gyro_str = ( (float)gyro[write_counter] / _GYRO_SENSITIVITY_2000);
					gyro_str++; /* Select next word */
					
				}
				
			}
			break;
			
		}
		/* End Switch */
		
		/* ------------------------------- */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
}
/*
  Example :
  
           uint8_t status;
		   float x[3];
           
		   -> status = Mpu6050_GetGyro( x );
		      
		     -> status is _TRUE/_FALSE
		   
*/

/* ^^^^^^^^^^^^^^^^^^^^^ Angle ^^^^^^^^^^^^^^^^^^^^^ */

uint8_t Mpu6050_GetAngleX(float *ang_x) /* Function for take x angle */
{
	
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status_y = 0; /* Variable for check instruction status */
	uint8_t instruction_status_z = 0; /* Variable for check instruction status */
	int16_t accel_y_value = 0; /* Variable for get accel value */
	int16_t accel_z_value = 0; /* Variable for get accel value */
	int32_t angle_y = 0; /* Variable for map value */
	int32_t angle_z = 0; /* Variable for map value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status_y = Mpu6050_GetRawAccelY(&accel_y_value); /* Get accel y value */
	instruction_status_z = Mpu6050_GetRawAccelZ(&accel_z_value); /* Get accel z value */
	
	/* ----------- Save Value in str ----------- */
	
	if ( (instruction_status_y == _TRUE) && (instruction_status_z == _TRUE) ) /* The instructions are complete */
	{
		
		angle_y = Map(accel_y_value , _MINIMUM_VALUE , _MAXIMUM_VALUE , _NEGATIVE_ANGLE , _POSITIVE_ANGLE); /* Calculating the MAP */
		angle_z = Map(accel_z_value , _MINIMUM_VALUE , _MAXIMUM_VALUE , _NEGATIVE_ANGLE , _POSITIVE_ANGLE); /* Calculating the MAP */
		
		*ang_x = ( _RAD_TO_DEG * ( atan2( -angle_y , -angle_z ) + _PI ) ); /* Export new value */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
	/* Function End */
}

uint8_t Mpu6050_GetAngleY(float *ang_y) /* Function for take y angle */
{
	
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status_x = 0; /* Variable for check instruction status */
	uint8_t instruction_status_z = 0; /* Variable for check instruction status */
	int16_t accel_x_value = 0; /* Variable for get accel value */
	int16_t accel_z_value = 0; /* Variable for get accel value */
	int32_t angle_x = 0; /* Variable for map value */
	int32_t angle_z = 0; /* Variable for map value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status_x = Mpu6050_GetRawAccelX(&accel_x_value); /* Get accel x value */
	instruction_status_z = Mpu6050_GetRawAccelZ(&accel_z_value); /* Get accel z value */
	
	/* ----------- Save Value in str ----------- */
	
	if ( (instruction_status_x == _TRUE) && (instruction_status_z == _TRUE) ) /* The instructions are complete */
	{
		
		angle_x = Map(accel_x_value , _MINIMUM_VALUE , _MAXIMUM_VALUE , _NEGATIVE_ANGLE , _POSITIVE_ANGLE); /* Calculating the MAP */
		angle_z = Map(accel_z_value , _MINIMUM_VALUE , _MAXIMUM_VALUE , _NEGATIVE_ANGLE , _POSITIVE_ANGLE); /* Calculating the MAP */
		
		*ang_y = ( _RAD_TO_DEG * ( atan2( -angle_x , -angle_z ) + _PI ) ); /* Export new value */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
	/* Function End */
}

uint8_t Mpu6050_GetAngleZ(float *ang_z) /* Function for take z angle */
{
	
	/* ----------- Create variable ----------- */
	
	uint8_t instruction_status_x = 0; /* Variable for check instruction status */
	uint8_t instruction_status_y = 0; /* Variable for check instruction status */
	int16_t accel_x_value = 0; /* Variable for get accel value */
	int16_t accel_y_value = 0; /* Variable for get accel value */
	int32_t angle_x = 0; /* Variable for map value */
	int32_t angle_y = 0; /* Variable for map value */
	
	/* ----------- Read value from MPU60X0 ----------- */
	
	instruction_status_x = Mpu6050_GetRawAccelX(&accel_x_value); /* Get accel x value */
	instruction_status_y = Mpu6050_GetRawAccelY(&accel_y_value); /* Get accel y value */
	
	/* ----------- Save Value in str ----------- */
	
	if ( (instruction_status_x == _TRUE) && (instruction_status_y == _TRUE) ) /* The instructions are complete */
	{
		
		angle_x = Map(accel_x_value , _MINIMUM_VALUE , _MAXIMUM_VALUE , _NEGATIVE_ANGLE , _POSITIVE_ANGLE); /* Calculating the MAP */
		angle_y = Map(accel_y_value , _MINIMUM_VALUE , _MAXIMUM_VALUE , _NEGATIVE_ANGLE , _POSITIVE_ANGLE); /* Calculating the MAP */
		
		*ang_z = ( _RAD_TO_DEG * ( atan2( -angle_y , -angle_x ) + _PI ) ); /* Export new value */
		
		return _TRUE;
		
	}
	
	return _FALSE;
	
	/* Function End */
}

/* =================================================================== */

/***************************************************************************/

/* Program End */