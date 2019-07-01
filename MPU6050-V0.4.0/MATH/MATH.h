/*
------------------------------------------------------------------------------
~ File   : MATH.h
~ Author : Majid Derhambakhsh
~ Version: V0.0.0
~ Created: 06/12/2019 07:20:00 PM
~ Brief  :
~ Support: Majid.do16@gmail.com
------------------------------------------------------------------------------
~ Description:

~ Attention  :
------------------------------------------------------------------------------
*/

#ifndef __MATH_H_
#define __MATH_H_

/* ----------------------------------- Include ----------------------------------- */

#include <stdint.h> /* Import standard integer lib */
#include <math.h> /* Import math lib */

/* ---------------------------------- Prototype ---------------------------------- */

int32_t Map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max); /* Function for calculating the mean arterial pressure (MAP) */
/*
  Example :
  
           int32_t value;
           
		   -> value = Map(10 , 0 , 100 , -50 , 50);
		      
		     -> value is -40
		   
*/

#endif /* __MATH_H_ */
