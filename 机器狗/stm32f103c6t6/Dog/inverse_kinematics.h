#ifndef __IK_H
#define __IK_H
#include "stm32f10x.h"                  // Device header

void inverse_funtion(float thigh_len, float shank_len, float tx, float ty, float* thigh_ang, float* shank_ang);

#endif
