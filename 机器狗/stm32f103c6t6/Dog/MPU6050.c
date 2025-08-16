#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "timer.h"
#include "Delay.h"
#include "math.h"
#include "usart.h"
#include "dog_utils.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址

unsigned char mpu_data[12] = {0};

void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									//给定超时计数时间
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//循环等待指定事件
	{
		Timeout --;										//等待时，计数值自减
		if (Timeout == 0)								//自减到0后，等待超时
		{
			/*超时的错误处理代码，可以添加到此处*/
			printf("MPU6050_WaitEvent failed \r\n");
			break;										//跳出等待，不等了
		}
	}
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data, I2C_TypeDef* I2Cx)
{
	I2C_GenerateSTART(I2Cx, ENABLE);										//硬件I2C生成起始条件
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2Cx, MPU6050_ADDRESS, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2Cx, RegAddress);											//硬件I2C发送寄存器地址
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			//等待EV8
	
	I2C_SendData(I2Cx, Data);												//硬件I2C发送数据
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTOP(I2Cx, ENABLE);											//硬件I2C生成终止条件
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress, I2C_TypeDef* I2Cx)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2Cx, ENABLE);										//硬件I2C生成起始条件
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2Cx, MPU6050_ADDRESS, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2Cx, RegAddress);											//硬件I2C发送寄存器地址
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTART(I2Cx, ENABLE);										//硬件I2C生成重复起始条件
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2Cx, MPU6050_ADDRESS, I2C_Direction_Receiver);		//硬件I2C发送从机地址，方向为接收
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		//等待EV6
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);									//在接收最后一个字节之前提前将应答失能
	I2C_GenerateSTOP(I2Cx, ENABLE);											//在接收最后一个字节之前提前申请停止条件
	
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);				//等待EV7
	Data = I2C_ReceiveData(I2Cx);											//接收数据寄存器
	
	I2C_AcknowledgeConfig(I2Cx, ENABLE);									//将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
	
	return Data;
}

void i2c_ReadMulti(uint8_t RegAddress, I2C_TypeDef* I2Cx, int n, unsigned char* data)
{
	volatile int temp;
	while (I2Cx->SR2 & I2C_SR2_BUSY){;}
	I2Cx->CR1|=I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){;}
	I2Cx->DR=MPU6050_ADDRESS;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){;}
	temp=I2Cx->SR2;
	while(!(I2Cx->SR1&I2C_SR1_TXE)){;}
	I2Cx->DR = RegAddress;
	while(!(I2Cx->SR1&I2C_SR1_TXE)){;}
	I2Cx->CR1|=I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){;}
	I2Cx->DR=MPU6050_ADDRESS|1;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){;}
	temp=I2Cx->SR2;
	I2Cx->CR1|=I2C_CR1_ACK;
	while(n>0U) {
		if(n==1U) {
			I2Cx->CR1&=~I2C_CR1_ACK;
			I2Cx->CR1|=I2C_CR1_STOP;
			while(!(I2Cx->SR1&I2C_SR1_RXNE)){;}
			*data++=I2Cx->DR;
			break;
		} else {
			while(!(I2Cx->SR1&I2C_SR1_RXNE)){;}
			(*data++)=I2Cx->DR;
			n--;
		}	
	}			
	I2Cx->CR1  |= 0x0400;//CR1_ACK_Set;
}

void MPU6050_Init(MPU6050_TypeDef* MPU, I2C_TypeDef* I2Cx) {
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01, I2Cx);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00, I2Cx);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x00, I2Cx);				//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x00, I2Cx);					//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x10, I2Cx);			//陀螺仪配置寄存器，±1000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00, I2Cx);			//加速度计配置寄存器，±2g
	
	MPU->gyro_config = 1.0 / (32768.0 / 1000 * 180.0 / _PI);
	MPU->accel_config = 1.0 / (32768.0 / 2);
	MPU->accCoef = 0.02;
   	MPU->gyroCoef = 0.98;
	MPU->angleY = 0;
	MPU->preInterval = 0;
	MPU->gyroYoffset = -0.0464;
	MPU->i2c = I2Cx;
}

uint8_t MPU6050_GetID(MPU6050_TypeDef* MPU)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I, MPU->i2c);		//返回WHO_AM_I寄存器的值
}

void MPU6050_CalcGyroOffsets(MPU6050_TypeDef* MPU) {
	uint8_t DataH, DataL;			
	float y = 0;
	int16_t ry;

	delay_ms(1000);
	int nknt = 3000;
	for(int i = 0; i < nknt; i++){
		DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H, MPU->i2c);		//读取陀螺仪Y轴的高8位数据
		DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L, MPU->i2c);		//读取陀螺仪Y轴的低8位数据
		ry = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
		y += ((float)ry) * MPU->gyro_config;
	}
	MPU->gyroYoffset = y / nknt;
	printf("MPU offset: %.4f\r\n", MPU->gyroYoffset);
	delay_ms(1000);
}

void MPU6050_Update(MPU6050_TypeDef* MPU) {
	i2c_ReadMulti(MPU6050_ACCEL_XOUT_H, MPU->i2c, 12, mpu_data);
	MPU->rawAccX = (mpu_data[0] << 8) | mpu_data[1];
	MPU->rawAccY = (mpu_data[2] << 8) | mpu_data[3];
	MPU->rawAccZ = (mpu_data[4] << 8) | mpu_data[5];
	MPU->rawGyroY = (mpu_data[10] << 8) | mpu_data[11];
	// printf("2: %d, %d, %d, %d, \r\n", MPU->rawAccX, MPU->rawAccY, MPU->rawAccZ, MPU->rawGyroY);

	unsigned long now_us = time_cntr;
	float interval;
	if (now_us < MPU->preInterval) {
		interval = (ulong_max_value - MPU->preInterval + now_us) * 1e-5;
	} else {
		interval = (now_us - MPU->preInterval) * 1e-5;
	}
	if(interval <= 0 || interval > 0.5) interval = 1e-3;
	MPU->preInterval = now_us;

	float accX = MPU->rawAccX * MPU->accel_config;
	float accY = MPU->rawAccY * MPU->accel_config;
	float accZ = MPU->rawAccZ * MPU->accel_config;
	int sign = accZ > 0 ? 1 : -1;
	MPU->angleAccY = _atan2(accX, sign * _sqrtApprox(accZ * accZ + accY * accY));
	MPU->gyroY = -(((float)MPU->rawGyroY) * MPU->gyro_config - MPU->gyroYoffset);
	MPU->angleY = (MPU->gyroCoef * (MPU->angleY + MPU->gyroY * interval)) + (MPU->accCoef * MPU->angleAccY);
}
