#include "Servo.h"

void Servo_SetAngle(uint8_t num, float Angle) {
	TIM_TypeDef* TIMx;
	TIMx = num < 4 ? TIM2 : TIM3;
	num = (num % 4) + 1;
	PWM_SetCompare(TIMx, Angle / 180 * 2000 + 500, num);
}

void PWM_SetCompare(TIM_TypeDef* TIMx, uint16_t Compare, uint8_t num) {
	switch (num) {
		case 1:
			TIM_SetCompare1(TIMx, Compare);
			break;
		case 2:
			TIM_SetCompare2(TIMx, Compare);
			break;
		case 3:
			TIM_SetCompare3(TIMx, Compare);
			break;
		case 4:
			TIM_SetCompare4(TIMx, Compare);
			break;
		default:
			break;
	}
}
