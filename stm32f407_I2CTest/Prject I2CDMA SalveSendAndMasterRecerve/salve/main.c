/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include"Bsp_Led.h"
#include"Bsp_I2C.h"
/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
typedef enum i2c_state
{
  COMM_DONE  = 0,  // done successfully
  COMM_PRE = 1,    // 准备通信 
  COMM_IN_PROCESS = 2,   // 通信中
  CHECK_IN_PROCESS = 3,
  COMM_EXIT = 4     // exit since failure
    
}I2C_STATE;
/* Private define ------------------------------------------------------------*/
#define I2C1_DR_Address        0x40005410
// #define I2C2_DR_Address        0x40005810
#define I2C1_SLAVE_ADDRESS7    0x30
#define I2C1_SLAVE_ADDRESS7    0x30
#define BufferSize             6
#define ClockSpeed             100000

/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef  I2C_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;
u8 I2C1_Buffer_Tx[8] = {1, 2, 3, 4, 5, 6, 7, 8};
u8 I2C1_Buffer_Rx[8];
u8 Tx_Idx = 0, Rx_Idx = 0;
volatile TestStatus TransferStatus;
ErrorStatus HSEStartUpStatus;
I2C_STATE i2c_comm_state;  // I2C 状态标志
static bool check_begin = FALSE;  //检测开始标志(不太懂)
/* Private function prototypes -----------------------------------------------*/
TestStatus Buffercmp(u8* pBuffer, u8* pBuffer1, u16 BufferLength);
void Delay(vu32 nCount);
void SlaveRecerve(void);    
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/


void recerveDataI2C_x(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel=DMA_Channel_1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)I2C1_Buffer_Rx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 指定外设地址不增加 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // 指定内存地址加一
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;   // 传输数据 字大小 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
	// DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	i2c_comm_state=COMM_PRE;
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	/*   事件中断 和错误中断 使能 */
	I2C_ITConfig(I2C1 , I2C_IT_EVT |I2C_IT_ERR , ENABLE); // ADDR BTF only
}
/* 
*  用于从机发送数据
**/
uint8_t slaveSendBuffTx[8]={0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
void slaveSend(void)
{
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED))   // 从发送地址匹配 
	{
	}	
	I2C_ITConfig(I2C1, I2C_IT_BUF |I2C_IT_EVT , DISABLE); // close TxE int
  I2C_DMACmd(I2C1, ENABLE);
  DMA_Cmd(DMA1_Stream6, ENABLE); 
	while(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET)  // 等待 DMA 接收结束 
	{
	}
	DMA_Cmd(DMA1_Stream6, DISABLE);
  DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6 );	
	while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_AF))     //EV4 
	{
		// if((CODECTimeout--) == 0) goto start;
	}		
	I2C_DMACmd(I2C1, DISABLE);
	I2C_ClearFlag(I2C1, I2C_FLAG_AF);
	
}
int main(void)
{ 

	uint8_t i;
  GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the CODEC_I2C peripheral clock */
  RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CODEC_I2C_GPIO_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
  /* CODEC_I2C SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = CODEC_I2C_SCL_PIN | CODEC_I2C_SDA_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(CODEC_I2C_GPIO, &GPIO_InitStructure);     
  /* Connect pins to I2C peripheral */
  GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2S_SCL_PINSRC, CODEC_I2C_GPIO_AF);  
  GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2S_SDA_PINSRC, CODEC_I2C_GPIO_AF);

  /* DMA1 channel_5 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel=DMA_Channel_1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)I2C1_Buffer_Rx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 指定外设地址不增加 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // 指定内存地址加一
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;   // 传输数据 字大小 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	// DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
  /* DMA1 channel_6 configuration as I2C1 TX ----------------------------------------------*/
  DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel=DMA_Channel_1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)slaveSendBuffTx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;      // 从内存到 外设 
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 指定外设地址不增加 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // 指定内存地址加一
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;   // 传输数据 字大小 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);	
	
	I2C_DeInit(CODEC_I2C); 
  /* I2C1 configuration ------------------------------------------------------*/
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
	I2C_Cmd(I2C1, ENABLE);
  I2C_Init(I2C1, &I2C_InitStructure);	

	Bsp_InitLed();

  while(1)
	{
		// SlaveRecerve();
		 slaveSend();
		 LED_Change(0);
//		 if(i2c_comm_state == COMM_DONE)
//		 {
//			 recerveDataI2C_x();
//		 }
		 // salveSend();
	  
		// 检测 地址匹配 


	}
	
}
/*
*   I2C从机接收数据 6个数据量 
**/
void SlaveRecerve(void)
{
	uint8_t i;
  DMA_InitTypeDef  DMA_InitStructure;	
	
//  DMA_DeInit(DMA1_Stream5);
//	DMA_InitStructure.DMA_Channel=DMA_Channel_1;
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)I2C1_Buffer_Rx;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//  DMA_InitStructure.DMA_BufferSize = BufferSize;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 指定外设地址不增加 
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // 指定内存地址加一
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;   // 传输数据 字大小 
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED))
		{
		}
		I2C_DMACmd(I2C1,ENABLE);
    DMA_Cmd(DMA1_Stream5, ENABLE);
		while(DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)==RESET)  // 开启转换就是这个样子的行当的啊真是
		{
		}
		I2C_DMACmd(I2C1,DISABLE);
		DMA_Cmd(DMA1_Stream5, DISABLE);
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5 );
   /* Test on I2C2 EV4 */
    while(I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_STOP_DETECTED)!= SUCCESS); 
		
   /* Clear I2C2 STOPF flag */
	(void)(I2C_GetITStatus(I2C1, I2C_IT_STOPF));
	I2C_Cmd(I2C1, ENABLE);
		
	for(i=0;i<6;i++)
	{
		if(I2C1_Buffer_Rx[i]!=i+1) break;
		else 
			I2C1_Buffer_Rx[i]=0;
		
		if(i==5)
		{
			LED_Change(0);
		}
	}
}
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
