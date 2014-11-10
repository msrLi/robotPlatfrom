
#include"Bsp_I2C_DMA.h"
#define I2CDMAANDINVC  0
#define I2C1_DR_Address        0x40005410
#define I2C1_OWN_ADDRESS7      0x30
#define I2C1ClockSpeed         100000
#define MAXBUFFDATALENGTH      10   							// 最大接收数量
#define BUFFSIZE               6                  // 接收和发送的数据长度
uint8_t I2C1_MasterSendBuff[10];   // I2C1 发送变量区
uint8_t I2C1_MasterReceBuff[10];	  // I2C1 接收变量区
I2CxInformation __IO * I2C1info;

/*
*   I2C1 错误中断处理函数
**/
void I2C1_EE_IRQ_Delay(void)
{
	I2CxInformation __IO * I2CConfigPoint;
	I2CConfigPoint=GetConfig();	
	if(I2C_GetFlagStatus(I2C1, I2C_FLAG_AF))   //应答失败错误中断
	{
		if (I2C1->SR2 &0x01)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2CConfigPoint->comState=COMM_EXIT; 
			// i2c_comm_state = COMM_EXIT;		
			
		}
		I2C_ClearFlag(I2C1, I2C_FLAG_AF);
	}
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BERR))
    {
      if (I2C1->SR2 &0x01)
      {
        I2C_GenerateSTOP(I2C1, ENABLE);
				I2CConfigPoint->comState=COMM_EXIT; 
      }
      
      I2C_ClearFlag(I2C1, I2C_FLAG_BERR);
    }
}
void I2C1_EV_IRQ_Delay(void)
{
	uint32_t FlageBuff;
	I2CxInformation __IO * I2CConfigPoint;
	I2CConfigPoint=GetConfig();
	FlageBuff=I2C_GetLastEvent(I2C1);
	switch(FlageBuff)
	{
/*********************Master Send strat recerve EV5***************************/
		case I2C_EVENT_MASTER_MODE_SELECT:  //EV5 
			I2CConfigPoint->comState=COMM_IN_PRECESS; 			// 修改通信状态为正在通信
		  if(I2CConfigPoint->comDir==MSTER_SEND){         // 判断通信方向
			/* I2C 主发送模式 */
				I2C_Send7bitAddress(I2C1, I2CConfigPoint->slveAddress, I2C_Direction_Transmitter);				
			}else{
			/* I2C 主接收模式 */
				I2C_Send7bitAddress(I2C1, I2CConfigPoint->slveAddress, I2C_Direction_Receiver); 
			}
			if(I2CConfigPoint->configLength==0){
				I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);
			}else{
				I2C_ITConfig(I2C1, I2C_IT_BUF , ENABLE);
			}
			break;
			
/********************** Master Receiver events *******************************/
		case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:  //EV6
			 // MSL BUSY ADDR 0x30002
		  I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);  
		  I2C_DMALastTransferCmd(I2C1, ENABLE);
			I2C_DMACmd(I2C1, ENABLE);
			DMA_Cmd(DMA1_Stream5, ENABLE); 	
			break;
		case I2C_EVENT_MASTER_BYTE_RECEIVED:    /* EV7 */
			// MSL BUSY RXNE 0x30040
				break;
/************************* Master Transmitter events *************************/
		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:   // EV8 just after EV6 
			I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);
			I2C_DMACmd(I2C1, ENABLE);
			DMA_Cmd(DMA1_Stream6, ENABLE);			
			break;
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED:       /* EV8-2 */
			//TRA, BUSY, MSL, TXE and BTF 0x70084
				break;
	}
}
/* 存放I2C配置文件*/
void SetConfig(I2CxInformation * data)
{
	I2C1info=data;
}
/* 读取I2C配置文件*/
I2CxInformation * GetConfig(void)
{
	return (I2CxInformation *)I2C1info;
}
void Init_I2CAnd_DMA(void)
{
/*  marco */
  GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
#if I2CDMAANDINVC
	NVIC_InitTypeDef NVIC_InitStructure;
#endif 
/* open Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	
	
/*I2C1 SDA CLK Pin Config  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;   // scl--PB6 ; SDA--PB9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
/* IO AF used */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

/* DMA1 channel_6 configuration as I2C1 master Send Tx  --------*/
  DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel=DMA_Channel_1;												 // 触发通道为 通道1 
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;     // I2C1 DR 寄存器地址 
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)I2C1_MasterSendBuff;    // 发送数据内存存放数组
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;              // 数据发送方向 从内存到外设 
  DMA_InitStructure.DMA_BufferSize = BUFFSIZE;											   // 发送数据长度
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;     // 指定外设地址不增加 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;              // 指定内存地址自动加一
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;  // 传输数据 一个字节大小发送
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			 // 内存数据一个字节大小操作
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;												 // DMA工作在普通模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;							 // DMA 优先级非常的高
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);													 //  初始化MDA1 的数据6 
/* DMA1 channel_5 configuration as I2C1 master Recerve Rx  --------*/
  DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel=DMA_Channel_1;                         // DMA 数据流由通道1 触发
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;		 // 数据源地址为I2C1 DR数据寄存器
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)I2C1_MasterReceBuff;		 // 数据目的地址为接收缓冲区
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;							 // 数据由外设向存储设备转移
  DMA_InitStructure.DMA_BufferSize = BUFFSIZE;											   // 转移数据长度
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;     // 指定外设地址不增加 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  						 // 指定内存地址加一
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;  // 传输数据 字大小 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;      // 内存数据一个字节大小操作
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;												 // DMA工作在普通模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High; 							   // DMA 优先级非常的高
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);												   // 初始化MDA1 的数据流5
 /* I2C1 configuration ------------------------------------------------------*/
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C1_OWN_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C1ClockSpeed;
	I2C_Cmd(I2C1, ENABLE); 
  I2C_Init(I2C1, &I2C_InitStructure);		
#if I2CDMAANDINVC
/* I2C1 EV_IRQ config */
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn; //嵌套中断通道为  USART1_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级 0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //中断使能
	NVIC_Init(&NVIC_InitStructure);	
/* I2C1 EV_IRQ config */
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
/* DMA1 Stream5 Interrupt config */	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
/* DMA1 Stream6 Interrupt config */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
#endif 

}
void masterSendData(uint8_t salveAddress,uint8_t sendData[],uint8_t lenght)
{
	uint8_t i;
	/* 将数据 复制到发送内存中 */
	for(i=0;i<lenght;i++)     
	{
		if(i>=BUFFSIZE) break;
		I2C1_MasterSendBuff[i]=sendData[i];
	}
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
	{
	}
	I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);
	/* Send I2C1 START condition */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* Test on I2C1 EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  
	/* Send I2C2 slave Address for write */
	I2C_Send7bitAddress(I2C1, salveAddress, I2C_Direction_Transmitter);
	/* Test on I2C1 EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 
	I2C_DMACmd(I2C1, ENABLE);
	DMA_Cmd(DMA1_Stream6, ENABLE);
	/* DMA1 Channel6 transfer complete test */
	while(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET)
	{
	}
	I2C_DMACmd(I2C1,DISABLE);
	DMA_Cmd(DMA1_Stream6, DISABLE);
	DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6 );
	// wait until BTF
	while (!(I2C1->SR1 & 0x04));
	I2C_GenerateSTOP(I2C1, ENABLE);
	// wait until BUSY clear
	while (I2C1->SR2 & 0x02);
	// DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF5 );
	/* Send I2C1 STOP Condition */
	// I2C_GenerateSTOP(I2C1, ENABLE);		
	//Delay(0xAFFFF);		
	// LED_Change(0);
}

void masterRecerveData(uint8_t salveAddress,uint8_t RecerveData[],uint8_t lenght)
{
	uint8_t i;
	CPU_SR_ALLOC(); 
	CPU_CRITICAL_ENTER(); // 禁止中断
	I2C_AcknowledgeConfig(I2C1, ENABLE);	
  /* Send START condition */
  if(I2C1->CR1 & 0x200)
    I2C1->CR1 &= 0xFDFF;
  I2C_GenerateSTART(I2C1, ENABLE);
  /* Test on I2C1 EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 	
	
	I2C_Send7bitAddress(I2C1, salveAddress, I2C_Direction_Receiver);	// send address 
  /* Test on I2C1 EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));  
	// I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
	//without it, no NAK signal on bus after last Data
	//I2C data line would be hold low ~~~
	I2C_DMALastTransferCmd(I2C1, ENABLE);
	I2C_DMACmd(I2C1, ENABLE);
  DMA_Cmd(DMA1_Stream5, ENABLE); 	
	while(DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)==RESET)  // 等待 DMA 接收结束 
	{
	}	
	
  I2C_DMACmd(I2C1, DISABLE);
	/* 清除DMA数据 */
	
  I2C_GenerateSTOP(I2C1, ENABLE);	
	DMA_Cmd(DMA1_Stream5, DISABLE);
	DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5 ); 	
	I2C_ClearFlag(I2C1, I2C_FLAG_AF);
	CPU_CRITICAL_EXIT();  // k、 开启中断
	for(i=0;i<BUFFSIZE;i++)
	{
		if(i>=lenght) break;
		RecerveData[i]=I2C1_MasterReceBuff[i];
		I2C1_MasterReceBuff[i]=0;
	}
 // clean ACK and STOP	
}
/**************************************END LINE ****************************************/
