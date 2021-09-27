#include "Quaternion.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"


#define _180_DIV_PI 57.295779515f // = 180 / PI

float BNO080_Roll;
float BNO080_Pitch;
float BNO080_Yaw;

extern void Quaternion_Update(float* q)
{
	float q1, q2, q3, q4;
	float norm;

	norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);    // normalize quaternion
	
	q1 = q[0] * norm; //x
	q2 = q[1] * norm; //y
	q3 = q[2] * norm; //z
	q4 = q[3] * norm; //w

	BNO080_Roll = atan2f(2.0f * (q2*q3 + q1*q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);
	BNO080_Pitch  = -asinf(2.0f * (q2*q4 - q1*q3));
	BNO080_Yaw   = atan2f(2.0f * (q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);

	BNO080_Roll *= _180_DIV_PI;
	BNO080_Pitch  *= _180_DIV_PI;
	BNO080_Yaw   *= _180_DIV_PI;
	
	if(BNO080_Yaw>=0)
		BNO080_Yaw = 360.f - BNO080_Yaw;
	else	
		BNO080_Yaw = -BNO080_Yaw;
	
	
	if(BNO080_Roll>=0)
		BNO080_Roll = 180.f - BNO080_Roll;
	else
		BNO080_Roll = -(BNO080_Roll + 180.f);
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}