/*
------------------------------------------------------------------------------
~ File   : math_ex.c
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

#include "math_ex.h"

/* ------------------------------- Function ------------------------------- */

int32_t Map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) /* Function for calculating the mean arterial pressure (MAP) */
{
	
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; /* Calculate and return value */
	
	/* Function End */
}
