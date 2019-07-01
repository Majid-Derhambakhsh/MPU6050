/**
******************************************************************************
* @file    I2C_UNIT.h
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

#ifndef __I2C_UNIT_H_
#define __I2C_UNIT_H_

/*************************************** Include ***************************************/

/*----------------------------------------------------------*/

#ifdef __CODEVISIONAVR__  /* Chech compiler */

#include <io.h>            /* Import AVR IO library */

/*----------------------------------------------------------*/

#elif defined(__GNUC__)   /* Chech compiler */

#include <avr/io.h>        /* Import AVR IO library */

/*----------------------------------------------------------*/

#else                     /* Compiler not found */

#error Compiler not supported  /* Send error */

#endif /* __CODEVISIONAVR__ */

#include <stdint.h> /* Import standard integer type */
#include "I2C_UNIT_CONFIG.h" /* Import i2c config file */

/*************************************** Defines ***************************************/

/* ---------------------- Type size ---------------------- */

#define ENUM_U8_T(ENUM_NAME)   Enum_##ENUM_NAME; typedef uint8_t ENUM_NAME /* Config enum size */

/* ----------------- MCU Clock definition ---------------- */

#ifdef __CODEVISIONAVR__ /* Chech compiler */

#define _F_CPU _MCU_CLOCK_FREQUENCY_ /* Define CPU clock */

#elif defined(__GNUC__) /* Chech compiler */

#define _F_CPU F_CPU /* Define CPU clock */

#endif /* __CODEVISIONAVR__ */

/* ---------------------- Prescaler ---------------------- */

#define _PRE1  1  /* Prescaler = 1  */
#define _PRE4  4  /* Prescaler = 4  */
#define _PRE16 16 /* Prescaler = 16 */
#define _PRE64 64 /* Prescaler = 64 */

/* ------ */
#if _PRESCALER == 1 /* Check prescaler */

#define _PRE_CODE 0x00 /* Prescaler value for set register */

/* ------ */
#elif _PRESCALER == 4 /* Check prescaler */

#define _PRE_CODE 0x01 /* Prescaler value for set register */

/* ------ */
#elif _PRESCALER == 16 /* Check prescaler */

#define _PRE_CODE 0x02 /* Prescaler value for set register */

/* ------ */
#elif _PRESCALER == 64 /* Check prescaler */

#define _PRE_CODE 0x03 /* Prescaler value for set register */

#endif

/* ------------------------ Public ----------------------- */

#define _STATUS 0xF8 /* Status flag bits */

/**************************************** Enums ****************************************/

typedef enum /* Enum Status Codes for Master Transmitter Mode */
{
	_MT_START_TRANSMITTED				= 0x08,
	_MT_REP_START_TRANSMITTED			= 0x10,
	_MT_SLA_W_TRANSMITTED_ACK			= 0x18,
	_MT_SLA_W_TRANSMITTED_NACK			= 0x20,
	_MT_DATA_TRANSMITTED_ACK			= 0x28,
	_MT_DATA_TRANSMITTED_NACK			= 0x30,
	_MT_SLA_W_ARB_LOST					= 0x38
}ENUM_U8_T(I2C_Status_MT_t);

typedef enum /* Enum Status codes for Master Receiver Mode */
{
	_MR_START_TRANSMITTED				= 0x08,
	_MR_REP_START_TRANSMITTED			= 0x10,
	_MR_SLA_R_ARB_LOST					= 0x38,
	_MR_SLA_R_TRANSMITTED_ACK			= 0x40,
	_MR_SLA_R_TRANSMITTED_NACK			= 0x48,
	_MR_DATA_RECEIVED_ACK				= 0x50,
	_MR_DATA_RECEIVED_NACK				= 0x58
}ENUM_U8_T(I2C_Status_MR_t);

typedef enum /* Enum Status Codes for Slave Receiver Mode */
{
	_SR_SLA_W_RECEIVED_ACK				= 0x60,
	_SR_SLA_RW_ARB_LOST					= 0x68,
	_SR_GEN_ADDR_RECEIVED_ACK			= 0x70,
	_SSR_SLA_RW_ARB_LOST_2				= 0x78,
	_SR_DATA_RECEIVED_SLA_ACK			= 0x80,
	_SR_DATA_RECEIVED_SLA_NACK			= 0x88,
	_SR_DATA_RECEIVED_GEN_ADDR_ACK		= 0x90,
	_SR_DATA_RECEIVED_GEN_ADDR_NACK		= 0x98,
	_SR_STOP_REP_START					= 0xA0
}ENUM_U8_T(I2C_Status_SR_t);

typedef enum /* Enum Status Codes for Slave Transmitter Mode */
{
	_ST_SLA_R_RECEIVED_ACK				= 0xA8,
	_ST_SLA_RW_ARB_LOST					= 0xB0,
	_ST_DATA_TRANSMITTED_ACK			= 0xB8,
	_ST_DATA_TRANSMITTED_NACK			= 0xC0,
	_ST_LAST_DATA_TRANSMITTED_ACK		= 0xC8
}ENUM_U8_T(I2C_Status_ST_t);

/************************************** Prototype **************************************/

uint8_t I2C_Status(void); /* Function for take I2C status */

uint8_t I2C_BeginTransmission(void); /* Function for send START condition */

uint8_t I2C_Transmit(uint8_t data); /* Function for transmit data */

uint8_t I2C_ReceiveACK(void); /* Function for receive data with ACK */

uint8_t I2C_ReceiveNACK(void); /* Function for receive data with no ACK */

void I2C_EndTransmission(void); /* Function for stop transmission I2C */

void I2C_SetAddress(uint8_t address); /* Function for self I2C address */

void I2C_Init(void); /* Function for initialize I2C */

#endif /* __I2C_UNIT_H_ */
