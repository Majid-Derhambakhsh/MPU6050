/**
******************************************************************************
* @file    I2C_UNIT.c
* @author  Majid Derhambakhsh
* @version V0.0.2
* @date    1-18-2019
* @brief
* @support Majid.do16@gmail.com
******************************************************************************
* @description
*
* @attention
******************************************************************************
*/

#include "I2C_UNIT.h"

/* ------------------ Functions ------------------ */

uint8_t I2C_Status(void) /* Function for take I2C status */
{
	
	return (TWSR & _STATUS); /* Return I2C status */

}

uint8_t I2C_BeginTransmission(void) /* Function for send START condition */
{
	/* --------------------------- */
	TWCR = (1 << TWINT)|(1 << TWSTA)|(1 << TWEN); /* Send START condition */
	
	/* --------------------------- */
	while ( !(TWCR & (1 << TWINT)) ); /* Wait for TWINT Flag set. This indicates that the START condition has been transmitted. */
	
	/* --------------------------- */
	return (I2C_Status()); /* Return status */
	
}

uint8_t I2C_Transmit(uint8_t data) /* Function for transmit data */
{
	/* --------------------------- */
	TWDR = data; /* Data to be transmitted */
	
	/* --------------------------- */
	TWCR = (1 << TWINT) | (1 << TWEN); /* Start transmission */
	
	/* --------------------------- */
	while ( !(TWCR & (1 << TWINT)) ); /* Wait for TWINT Flag set. This indicates that the START condition has been transmitted. */
	
	/* --------------------------- */
	return (I2C_Status());
	
}

uint8_t I2C_ReceiveACK(void) /* Function for receive data with ACK */
{
	/* --------------------------- */
	TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN); /* Acknowledge Send */
	
	/* --------------------------- */
	while ( !(TWCR & (1 << TWINT)) ); /* Wait for TWINT Flag set. This indicates that the START condition has been transmitted. */

	/* --------------------------- */
	return TWDR; /* Return received data */

}

uint8_t I2C_ReceiveNACK(void) /* Function for receive data with no ACK */
{
	/* --------------------------- */
	TWCR = (1 << TWINT) | (1 << TWEN);

	/* --------------------------- */
	while ( !(TWCR & (1 << TWINT)) ); /* Wait for TWINT Flag set. This indicates that the START condition has been transmitted. */

	/* --------------------------- */
	return TWDR; /* Return received data */
	
}

void I2C_EndTransmission(void) /* Function for stop transmission I2C */
{
	/* --------------------------- */
	TWCR = (1 << TWINT)|(1 << TWEN)|(1 << TWSTO); /* Transmit STOP condition */
	
	/* --------------------------- */
	while ( TWCR & (1 << TWSTO) ); /* Wait for TWSTO Flag reset. */
	
}

void I2C_SetAddress(uint8_t address) /* Function for self I2C address */
{
	/* --------------------------- */
	TWAR = address << 1; /* Set TWI slave address (upper 7 bits) */

}

void I2C_Init(void) /* Function for initialize I2C */
{
	
	/* ------- Initialize Prescaler & Bit rate ------- */
	
	TWSR = _PRESCALER; /* Prescaler = 1 */
	TWBR = (uint8_t)( ( (_F_CPU / _F_SCL) - 16 ) / (2 * _PRESCALER) ); /* Calculate and set i2c bit rate */

	/* ----------------------------------------------- */
	TWCR = (1 << TWEN); /* Enable TWI  module */
	
}

/* Program End */