
#include "stm32f10x.h"
#include <stdlib.h>
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "iic.h"
#include "MPU6050.h"
#include "NRF24L01.h"
#include "math.h"
#include "Servo.h"

uint8_t Buf[32] = {0};
float i2c_freq = 400000; // Hz

void remote_control_run(float* target_ang);
void commander_run(uint8_t* debug_flag);

int main(void)
{
	MPU6050_TypeDef MPU;
	NRF24L01_Init();
	uart_init(115200);
	TIM1_10us_Init();
	TIM2_PWM_Init();
	TIM3_PWM_Init();
	I2C_Init_(I2C1, i2c_freq);
	MPU6050_Init(&MPU, I2C1);
	printf("MPU ID: %x\r\n", MPU6050_GetID(&MPU));
	Servo_SetAngle(0, 90);
	Servo_SetAngle(1, 90);
	Servo_SetAngle(2, 90);
	Servo_SetAngle(3, 90);
	Servo_SetAngle(4, 90);
	Servo_SetAngle(5, 90);
	Servo_SetAngle(6, 90);
	Servo_SetAngle(7, 90);
	delay_ms(1000);            //Wait for the system to stabilize

	if (0) {
		MPU6050_CalcGyroOffsets(&MPU);
		return 0;
	}

	int pknt = 0;
	unsigned long timestamp_prev = 0;
	uint8_t debug_flag = 1;
	float target_ang = 90;
	while(1) {
		MPU6050_Update(&MPU);		//获取MPU6050的数据
		float mpu_pitch = -MPU.angleY;
		
		Servo_SetAngle(0, target_ang);

		if (debug_flag) {
			unsigned long now_us;
			float Ts;
			now_us = time_cntr;
			if (now_us < timestamp_prev) {
				Ts = (ulong_max_value - timestamp_prev + now_us) * 1e-5;
			} else {
				Ts = (now_us - timestamp_prev) * 1e-5;
			}
			timestamp_prev = now_us;

			if (pknt == 0) {
				printf("%.5f, %.4f, %.4f, %.4f\r\n", Ts, mpu_pitch, MPU.angleAccY, MPU.gyroY);
				pknt = 100;
			}
			delay_ms(3);
			pknt -= 1;
		}
		remote_control_run(&target_ang);
		commander_run(&debug_flag);
	}
}

void remote_control_run(float* target_ang) {
	uint16_t X;
	if (NRF24L01_Get_Value_Flag() == 0) {
		NRF24L01_GetRxBuf(Buf);
		X = Buf[1] * 256 + Buf[0];
		if (X >= 3000) {
			*target_ang = 50;
		} else if (X <= 1000) {
			*target_ang = 130;
		}
	}
}

void commander_run(uint8_t* debug_flag) {
	if((USART_RX_STA&0x8000) != 0)
	{
		switch(USART_RX_BUF[0])
		{
			case 'D': // debug
				*debug_flag = atoi((const char *)(USART_RX_BUF+1));
				printf("debug_flag=%d\r\n", *debug_flag);
				break;
		}
		USART_RX_STA=0;
	}
}
