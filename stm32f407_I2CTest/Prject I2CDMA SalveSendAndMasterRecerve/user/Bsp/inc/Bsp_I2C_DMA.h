
#ifndef _BSP_I2C_DMA_H_
#define _BSP_I2C_DMA_H_
#include"stm32f4xx.h"
typedef enum i2cState
{
	COMM_DOWN=0,    //success 
	COMM_PRE=1,     //准备通信
	COMM_IN_PRECESS=2,  // 正在通信中
	COMM_EXIT=3					// 通信失败退出
}I2Cx_STATE;
typedef enum i2cDir
{
	MSTER_SEND=0,				// 主发送模式
	MSTER_RECERVE=1			// 主接收模式
}I2Cx_DIRSTATE;
typedef struct I2CxInformation{
	I2Cx_STATE comState;				//通信状态
	I2Cx_DIRSTATE comDir;       //通信方式 主接收还是发送
	uint8_t slveAddress;				//将要发送的从机地址
	uint8_t configLength;       //配置是否可变长度发送
	
}I2CxInformation;

extern uint8_t I2C1_MasterSendBuff[10];   // I2C1 发送变量区
extern uint8_t I2C1_MasterReceBuff[10];	  // I2C1 接收变量区
/*
*   I2C1 错误中断处理函数
**/
void I2C1_EE_IRQ_Delay(void);

/*
*   I2C1 事件中断处理函数
**/
void I2C1_EV_IRQ_Delay(void);

/* 存放I2C配置文件*/
void SetConfig(I2CxInformation * data);
	
/* 读取I2C配置文件*/
I2CxInformation * GetConfig(void);

void Init_I2CAnd_DMA(void);
/*
*  主机发送 数据
*  salveAddress : 从机地址
*  sendData[] 发送数据指针
*  length     发送数据的数量
**/
void masterSendData(uint8_t salveAddress,uint8_t sendData[],uint8_t lenght);
/*
*  主机发送 数据
*  salveAddress : 从机地址
*  RecerveData[] 发送数据指针
*  length     发送数据的数量
**/
void masterRecerveData(uint8_t salveAddress,uint8_t RecerveData[],uint8_t lenght);
#endif 
/**************************************END LINE ****************************************/
