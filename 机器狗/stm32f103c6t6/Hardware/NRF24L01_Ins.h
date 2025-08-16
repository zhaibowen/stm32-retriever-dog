#ifndef __nRF24L01_H
#define __nRF24L01_H

/**********  NRF24L01���Ŷ���  ***********/
#define IRQ_Port	GPIOA
#define CE_Port		GPIOB
#define CSN_Port	GPIOB
#define SCK_Port	GPIOB
#define MOSI_Port	GPIOB
#define MISO_Port	GPIOB

#define IRQ_Pin		GPIO_Pin_8
#define CE_Pin		GPIO_Pin_11
#define CSN_Pin		GPIO_Pin_12
#define SCK_Pin		GPIO_Pin_13
#define MOSI_Pin	GPIO_Pin_14
#define MISO_Pin	GPIO_Pin_15

/**********  NRF24L01�Ĵ�����������  ***********/
#define nRF_READ_REG        0x00 
#define nRF_WRITE_REG       0x20 
#define RD_RX_PLOAD     0x61  
#define WR_TX_PLOAD     0xA0  
#define FLUSH_TX        0xE1  
#define FLUSH_RX        0xE2  
#define REUSE_TX_PL     0xE3  
#define NOP             0xFF  
/**********  NRF24L01�Ĵ�����ַ   *************/
#define CONFIG          0x00                           
#define EN_AA           0x01  
#define EN_RXADDR       0x02  
#define SETUP_AW        0x03  
#define SETUP_RETR      0x04  
#define RF_CH           0x05  
#define RF_SETUP        0x06  
#define STATUS          0x07  
#define OBSERVE_TX      0x08  
#define CD              0x09  
#define RX_ADDR_P0      0x0A  
#define RX_ADDR_P1      0x0B  
#define RX_ADDR_P2      0x0C  
#define RX_ADDR_P3      0x0D  
#define RX_ADDR_P4      0x0E  
#define RX_ADDR_P5      0x0F  
#define TX_ADDR         0x10  
#define RX_PW_P0        0x11   
#define RX_PW_P1        0x12   
#define RX_PW_P2        0x13   
#define RX_PW_P3        0x14   
#define RX_PW_P4        0x15  
#define RX_PW_P5        0x16  
#define FIFO_STATUS     0x17  

/******   STATUS�Ĵ���bitλ����      *******/
#define MAX_TX  	0x10  	  //�ﵽ����ʹ����ж�
#define TX_OK   	0x20  	  //TX��������ж�
#define RX_OK   	0x40  	  //���յ������ж�

#endif
