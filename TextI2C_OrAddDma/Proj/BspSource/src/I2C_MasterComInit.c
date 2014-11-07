#include"I2C_MasterComInit.h"
u8 I2C1_Buffer_Tx[8] = {1, 2, 3, 4, 5, 6, 7, 8};
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
void DMA_ConfigForI2C1Send(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
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
}
void SendDataBy_I2C1()
{
	DMA_ConfigForI2C1Send();
	I2C_AcknowledgeConfig(I2C1, ENABLE); 
	I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);
	while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))   //wait for bus is not busy
	{
	}
  /* Start the config sequence */
  I2C_GenerateSTART(CODEC_I2C, ENABLE);
	
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))   // EV5 
	{
	}
  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(CODEC_I2C, CODEC_ADDRESS, I2C_Direction_Transmitter);
	DMA_Cmd(DMA1_Stream6, ENABLE);	
 	I2C_DMACmd(I2C2, ENABLE);
		/* DMA1 Channel6 transfer complete test */
	while(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET)
	{
	}	
  I2C_DMACmd(I2C1, DISABLE);
  DMA_Cmd(DMA1_Stream6, DISABLE);	
  // wait until BTF
  while (!(I2C1->SR1 & 0x04));
  I2C_GenerateSTOP(I2C1, ENABLE);
  // wait until BUSY clear
  while (I2C1->SR2 & 0x02);	
  if(I2C1->CR1 & 0x200)
      I2C1->CR1 &= 0xFDFF;
			
}
/******************* (C) COPYRIGHT 2014 MacRobot *****END OF FILE****/
