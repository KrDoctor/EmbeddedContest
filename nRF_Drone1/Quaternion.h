#ifndef _QUATERNION_H
#define _QUATERNION_H

#include <math.h>
#include "mbed.h"
extern float BNO080_Roll;
extern float BNO080_Pitch;
extern float BNO080_Yaw;

extern void Quaternion_Update(float* q);
float invSqrt(float x);

#endif
