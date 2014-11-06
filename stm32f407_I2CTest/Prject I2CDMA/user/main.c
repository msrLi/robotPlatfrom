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
#include"Bsp_Hcs4051.h"
#include"Bsp_send_Data.h"
/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/
#define I2C1_DR_Address        0x40005410
#define I2C2_DR_Address        0x40005410
#define I2C1_SLAVE_ADDRESS7    0x30
#define I2C2_SLAVE_ADDRESS7    0x30
#define BufferSize             5
#define ClockSpeed             100000

/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef  I2C_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;
u8 I2C1_Buffer_Tx[8] = {1, 2, 3, 4, 5, 6, 7, 8};
u8 I2C2_Buffer_Rx[8];
u8 Tx_Idx = 0, Rx_Idx = 0;
volatile TestStatus TransferStatus;
ErrorStatus HSEStartUpStatus;
/* Private function prototypes -----------------------------------------------*/
TestStatus Buffercmp(u8* pBuffer, u8* pBuffer1, u16 BufferLength);
void Delay(vu32 nCount);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/
void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}
uint8_t rece[4];
int main(void)
{ 
//	RCC_ClocksTypeDef Rcc_get;   // 各路时钟 
//	RCC_GetClocksFreq(&Rcc_get);
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//	Bsp_InitLed();
//	GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_RESET);
//	LEDDelay(10);
//	GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_SET);
//	Bsp_I2C_init();
////#ifndef SAlVE1
//	Bsp_Hcs4051_Init();
//	Bsp_HcsSlect(1);
//	Bsp_Hcs_disEn(ENABLE);
//#endif
	
//	SysTick_Config(Rcc_get.HCLK_Frequency/100);
	//BSP_Init();	
	//BSP_Tick_Init();
  GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
  /* Enable the CODEC_I2C peripheral clock */
  RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CODEC_I2C_GPIO_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
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
	
  /* DMA1 channel_6 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel=DMA_Channel_1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)I2C1_Buffer_Tx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;      // 从内存到 外设 
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 指定外设地址不增加 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // 指定内存地址加一
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;   // 传输数据 字大小 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
 
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream6, ENABLE);
  I2C_Cmd(I2C1, ENABLE); 
  /* I2C1 configuration ------------------------------------------------------*/
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
  I2C_Init(I2C1, &I2C_InitStructure);	
	Bsp_InitLed();
  while(1)
	{
		
		/*----- Transmission Phase -----*/
		/* wait for bus is not busy  */
		while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
		{
		}
		/* Send I2C1 START condition */
		I2C_GenerateSTART(I2C1, ENABLE);
		/* Test on I2C1 EV5 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  
		/* Send I2C2 slave Address for write */
		I2C_Send7bitAddress(I2C1, I2C2_SLAVE_ADDRESS7, I2C_Direction_Transmitter);
		/* Test on I2C1 EV6 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 
		I2C_DMACmd(I2C1, ENABLE);
		
		/* DMA1 Channel6 transfer complete test */
		while(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET)
		{
		}
		DMA_Cmd(DMA1_Stream5, DISABLE);
		I2C_DMACmd(I2C1,DISABLE);
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5 );
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);		
		Delay(0xAFFFF);		
		LED_Change(0);
//	  LEDDelay(100);
//		Bsp_HcsSlect(0);
//		// Codec_WriteRegister(0x02,0x78);
//	  SendPWM(rece);
//		// hostRead(rece);
//		// 
	}
	
}

  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
