/*
------------------------------------------------------------------------------
~ File   : mpu6050_conf.h
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

#ifndef __MPU6050_CONF_H_
#define __MPU6050_CONF_H_

/* ~~~~~~~~~~~~~~ Required Headers ~~~~~~~~~~~~~ */
#include "MATH_EX/math_ex.h" /* Import math lib */

/* Driver-library for AVR */
//#include "GPIO/gpio_unit.h" /* Import gpio lib */
//#include "I2C_UNIT/i2c_unit.h" /* Import i2c lib */

/* Driver-library for STM32 */
#include "STM32_I2C/stm32_i2c.h" /* Import i2c lib */

/* ~~~~~~~~~~~~~~~~ MPU Address ~~~~~~~~~~~~~~~~ */
#define _MPU_AD0_LEVEL _MPU_AD0_LOW /* Set AD0 status */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* __MPU6050_CONF_H_ */
