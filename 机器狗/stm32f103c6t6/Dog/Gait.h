#ifndef __GAIT_H
#define __GAIT_H
#include "stm32f10x.h"                  // Device header

typedef struct {
	float x1, x2, x3, x4;
    float z1, z2, z3, z4;
    float swing_xf, swing_zf;
    float support_xf, support_zf;
} Motion_Trajectory;

void swing_curve_generate(float t, float Tf, float xt, float zh, Motion_Trajectory* MT);
void support_curve_generate(float t, float Tf, float xt, Motion_Trajectory* MT);
void trot(float t, float xt, float zt, int8_t r1, int8_t r4, int8_t r2, int8_t r3, Motion_Trajectory* MT);

#endif
