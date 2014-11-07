
#include"I2C_SlaveComInit.h"
u8 I2C1_Buffer_Rx[8];
void Init_I2C1(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
/*---------------------- open clock  -------------------------------*/
  /* Enable I2S and I2C GPIO clocks */
  RCC_AHB1PeriphClockCmd(CODEC_I2C_GPIO_CLOCK, ENABLE);
  /* Enable the CODEC_I2C peripheral clock */
  RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);
	
/* CODEC_I2C SCL and SDA pins configuration ------------------------*/
  GPIO_InitStructure.GPIO_Pin = CODEC_I2C_SCL_PIN | CODEC_I2C_SDA_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(CODEC_I2C_GPIO, &GPIO_InitStructure);     
  /* Connect pins to I2C peripheral */
  GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2S_SCL_PINSRC, CODEC_I2C_GPIO_AF);  
  GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2S_SDA_PINSRC, CODEC_I2C_GPIO_AF);  

/* CODEC_I2C peripheral configuration -------------------------------*/
  I2C_DeInit(CODEC_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = CODEC_ADDRESS;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  /* Enable the I2C peripheral */
  I2C_Cmd(CODEC_I2C, ENABLE);  
  I2C_Init(CODEC_I2C, &I2C_InitStructure);
	
/************** I2C NVIC configuration *************************/  
// none 
}

void DMA_ConfigForI2C1Recerve(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
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
}
void SalveRecerveI2C(void)
{
	DMA_ConfigForI2C1Recerve();
	I2C_AcknowledgeConfig(I2C1, ENABLE); 
	I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);	
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)); 
	I2C_DMACmd(I2C1,ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	while(DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)==RESET)  // 开启转换就是这个样子的行当的啊真是
	{
	}
  I2C_DMACmd(I2C2, DISABLE);
  DMA_Cmd(DMA1_Stream5, DISABLE); 	
	while(I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_STOP_DETECTED)!= SUCCESS); 
	(void)(I2C_GetITStatus(I2C2, I2C_IT_STOPF));
	I2C_Cmd(I2C2, ENABLE);
}

