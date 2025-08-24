
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
#include "inverse_kinematics.h"
#include "Gait.h"

uint8_t Buf[32] = {0};
uint16_t Lx, Ly, Rx, Ry;
uint8_t Key0, Key1, Key2, Key3, Key4, Key5;
float i2c_freq = 400000; // Hz
float body_length = 112;
float body_width = 104.8;
float thigh_length = 80;
float shank_length = 60;
float rise_height = 30;
float gravity_shift_x = 0;
float gravity_shift_z = 110;
uint8_t init_ang = 90;
uint8_t init_ang_arr[4][2] = {{94, 91}, {87, 89}, {88, 91}, {93, 90}};

void remote_control_run(void);
void servo_output(float ham1, float ham2, float ham3, float ham4, float shank1, float shank2, float shank3, float shank4);

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

	if (0) { // 陀螺仪校准
		MPU6050_CalcGyroOffsets(&MPU);
		return 0;
	}

    Motion_Trajectory MT;

	int pknt = 0;
	unsigned long timestamp_prev = 0;
	uint8_t debug_flag = 1;
	uint8_t leg_number = 0, leg_part = 0;
	uint8_t calibration = 0;
	while(1) {
		remote_control_run();
		if (calibration == 0) {
			if (Key0) { // 舵机标定
				leg_number = 0;
				leg_part = 0;
				for (int i = 0; i < 4; ++i) {
					for (int j = 0; j < 2; ++j) {
						init_ang_arr[i][j] = init_ang;
						Servo_SetAngle(i*2+j, init_ang_arr[i][j]);
					}
				}
				calibration = 1;
				printf("servo calibration, init angle: 90\r\n");
			} else { // 站立
                float thigh_angle, shank_angle;
                inverse_funtion(thigh_length, shank_length, -gravity_shift_x, -gravity_shift_z, &thigh_angle, &shank_angle);
                servo_output(thigh_angle, thigh_angle, thigh_angle, thigh_angle, shank_angle, shank_angle, shank_angle, shank_angle);
            }
		} else if (calibration) { // 进入舵机标定
			uint8_t press_flag = 0;
			if (Key1) { // 选择哪条腿
				press_flag = 1;
				leg_number = (leg_number + 1) % 4;
			} else if (Key2) { // 选择大腿或小腿
				press_flag = 1;
				leg_part = (leg_part + 1) % 2;
			} else if (Key3) { // 舵机角度 + 1
				press_flag = 1;
				init_ang_arr[leg_number][leg_part] += 1;
			} else if (Key4) { // 舵机角度 - 1
				press_flag = 1;
				init_ang_arr[leg_number][leg_part] -= 1;
			} else if (Key5) { // 退出舵机标定
				calibration = 0;
				printf("exit servo calibration\r\n");
				for (int i = 0; i < 4; ++i) {
					for (int j = 0; j < 2; ++j) {
						printf("    calibration leg %d, part %d, angle %d\r\n", i, j, init_ang_arr[i][j]);
					}
				}
			}
			if (press_flag) {
				Servo_SetAngle(leg_number*2+leg_part, init_ang_arr[leg_number][leg_part]);
				printf("    choose leg %d, part %d, current angle %d\r\n", leg_number, leg_part, init_ang_arr[leg_number][leg_part]);
			}
		}
		delay_ms(200);

		// MPU6050_Update(&MPU);		//获取MPU6050的数据
		// float mpu_pitch = -MPU.angleY;
		// if (debug_flag) {
		// 	unsigned long now_us;
		// 	float Ts;
		// 	now_us = time_cntr;
		// 	if (now_us < timestamp_prev) {
		// 		Ts = (ulong_max_value - timestamp_prev + now_us) * 1e-5;
		// 	} else {
		// 		Ts = (now_us - timestamp_prev) * 1e-5;
		// 	}
		// 	timestamp_prev = now_us;

		// 	if (pknt == 0) {
		// 		printf("%.5f, %.4f, %.4f, %.4f\r\n", Ts, mpu_pitch, MPU.angleAccY, MPU.gyroY);
		// 		pknt = 100;
		// 	}
		// 	delay_ms(3);
		// 	pknt -= 1;
		// }
	}
}

void remote_control_run(void) {
	if (NRF24L01_Get_Value_Flag() == 0) {
		NRF24L01_GetRxBuf(Buf);
		Lx = Buf[1] * 256 + Buf[0];
		Ly = Buf[3] * 256 + Buf[2];
		Rx = Buf[5] * 256 + Buf[4];
		Ry = Buf[7] * 256 + Buf[6];
		Key0 = Buf[8] ^ 1;
		Key1 = Buf[9] ^ 1;
		Key2 = Buf[10] ^ 1;
		Key3 = Buf[11] ^ 1;
		Key4 = Buf[12] ^ 1;
		Key5 = Buf[13] ^ 1;
	}
}

void servo_output(float ham1, float ham2, float ham3, float ham4, float shank1, float shank2, float shank3, float shank4) {
    Servo_SetAngle(0, init_ang_arr[0][0] + init_ang - ham1);
    Servo_SetAngle(1, init_ang_arr[0][1] - init_ang + shank1);
    Servo_SetAngle(2, init_ang_arr[1][0] - init_ang + ham2);
    Servo_SetAngle(3, init_ang_arr[1][1] + init_ang - shank2);
    Servo_SetAngle(4, init_ang_arr[2][0] - init_ang + ham3);
    Servo_SetAngle(5, init_ang_arr[2][1] + init_ang - shank3);
    Servo_SetAngle(6, init_ang_arr[3][0] + init_ang - ham4);
    Servo_SetAngle(7, init_ang_arr[3][1] - init_ang + shank4);
}
