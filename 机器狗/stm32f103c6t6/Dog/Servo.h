#ifndef __SERVO_H
#define __SERVO_H
#include "stm32f10x.h"                  // Device header

void Servo_SetAngle(uint8_t num, float Angle);
void PWM_SetCompare(TIM_TypeDef* TIMx, uint16_t Compare, uint8_t num);

#endif
