#include"Bsp_I2C.h"
#ifdef SAlVE1  
	#define OWNADD   0x90
#else 
	#define OWNADD  0x33
#endif 
uint32_t CODECTimeout;
I2C_InformationSendAndRecerve * GolbeI2C_Buff;   // 缓冲 I2C配置变量地址
uint32_t Codec_TIMEOUT_UserCallback(void)
{   
	while(1)
	{
	}
  //return (0);
}

static void _I2C_IOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable I2S and I2C GPIO clocks */
  RCC_AHB1PeriphClockCmd(CODEC_I2C_GPIO_CLOCK, ENABLE);

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
}
/**
  * @brief  Initializes the Audio Codec control interface (I2C).
  * @param  None
  * @retval None
  */
static void Codec_CtrlInterface_Init(void)
{
  I2C_InitTypeDef I2C_InitStructure;
  
  /* Enable the CODEC_I2C peripheral clock */
  RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);
  
  /* CODEC_I2C peripheral configuration */
  I2C_DeInit(CODEC_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x91;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  /* Enable the I2C peripheral */
  I2C_Cmd(CODEC_I2C, ENABLE);  
  I2C_Init(CODEC_I2C, &I2C_InitStructure);
}
void Bsp_I2C_init()
{
	_I2C_IOInit();
	Codec_CtrlInterface_Init();
//  allow I2C interrupt 
	#ifndef SAlVE1
	I2C_ITConfig(CODEC_I2C,I2C_IT_EVT,ENABLE);
	NVIC_I2C_Interrupt();
	#endif 
}
void NVIC_I2C_Interrupt(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn; //嵌套中断通道为  USART1_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_1_IRQ_PRE_PRI; //抢占优先级 0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_1_IRQ_SUB_PRI;    //响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //中断使能
	NVIC_Init(&NVIC_InitStructure);	
}
void ARC_SetI2C_Information(I2C_InformationSendAndRecerve * P_setValue)
{
	GolbeI2C_Buff=P_setValue;
}
I2C_InformationSendAndRecerve * ARC_GetI2C_Information(void)
{
	return GolbeI2C_Buff;
}
#ifdef SAlVE1  
/*********************************************************************************************
																			I2C 作为 从机
**********************************************************************************************/
void salveRead(uint8_t * dataBuff)
{
	/* wait for address matched*/ 
//	start:
	I2C_AcknowledgeConfig(CODEC_I2C,ENABLE);
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED))   
	{
		 // if((CODECTimeout--) == 0) goto start;
	}
	/* wait for recerve a data */
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_BYTE_RECEIVED))   
	{
		// if((CODECTimeout--) == 0) goto start;
	}
	dataBuff[0]=I2C_ReceiveData(CODEC_I2C);
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_BYTE_RECEIVED))   
	{
		// if((CODECTimeout--) == 0) goto start;
	}
	dataBuff[1]=I2C_ReceiveData(CODEC_I2C);
	//
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_BYTE_RECEIVED))   
	{
		// if((CODECTimeout--) == 0) goto start;
	}	
	dataBuff[2]=I2C_ReceiveData(CODEC_I2C);
	//
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_STOP_DETECTED))   
	{
		// if((CODECTimeout--) == 0) goto start;
	}	
  /* End the configuration sequence */
  I2C_GenerateSTOP(CODEC_I2C, ENABLE);  
}
void salveSend(void)
{
	CODECTimeout = CODEC_LONG_TIMEOUT;
	 // EV1 as  salve transmitter  从发送 状态  and EV3-1
	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED))    
	{
		 // if((CODECTimeout--) == 0) return ;
	}	
//	while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_TXE)==RESET);
//	while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF)==RESET);
//   while(!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_BYTE_TRANSMITTED));   //   EV3
	// send tx datas 
	I2C_SendData(CODEC_I2C,0x89);
	while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF)==SET);
	while(!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_BYTE_TRANSMITTING))		//   EV3-1 
	{
	}
	  /*!< Disable Acknowledgment */
//   I2C_AcknowledgeConfig(CODEC_I2C, DISABLE);   
	I2C_SendData(CODEC_I2C,0x12);
	while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF)==SET);
	while(!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_SLAVE_ACK_FAILURE))
	{
	}
  /*!< Re-Enable Acknowledgment to be ready for another reception */
//  I2C_AcknowledgeConfig(CODEC_I2C, ENABLE);  
  
  /* Clear AF flag for next communication */
//  I2C_ClearFlag(CODEC_I2C, I2C_FLAG_AF); 
  /* End the configuration sequence */
      (void)(I2C_GetITStatus(I2C2, I2C_IT_STOPF));   //产生结束标志 通过读写I2C_SR1 然后通过写  I2C_CR1
       I2C_Cmd(I2C2, ENABLE);		 
   // I2C_GenerateSTOP(CODEC_I2C, ENABLE);  	
}

#else
/*********************************************************************************************
																			I2C 作为 主机部分 
**********************************************************************************************/
I2C_Return I2C_Master_Transmitter(I2C_InformationSendAndRecerve * I2CPram)
{
	I2C_Return retStat;  
	retStat=I2C_Ok;   
	ARC_SetI2C_Information(I2CPram);   // 存储数据到缓冲变量
// 	I2C_InterruptBuff=I2CPram;	
    /* Enable EVT IT*/
  I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	
	I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);   // 缓冲中断使能 
	/* wait for Bus is not busy */
  CODECTimeout = CODEC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
  {
    if((CODECTimeout--) == 0)  return I2C_Error;    // return this function 
  }

  /* Start the config sequence */
  I2C_GenerateSTART(CODEC_I2C, ENABLE);	  
	/* Then to go to interrupt to deal  */ 
	//  while()//   发送数据可以不等带 。。主机程序中必须   等待发送成功，，可以适当延时
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  if(I2CPram->TX_Generate_stop == 0)
        I2C_AcknowledgeConfig(I2C1, ENABLE);
	return retStat;
}
I2C_Return I2C_Master_Recerve(I2C_InformationSendAndRecerve * I2CPram)
{
	return 0;
}
uint32_t Codec_WriteRegister(uint8_t RegisterAddr, uint8_t RegisterValue)
{
  uint32_t result = 0;

  /*!< While the bus is busy */
  CODECTimeout = CODEC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
  {
    // if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }
  
  /* Start the config sequence */
  I2C_GenerateSTART(CODEC_I2C, ENABLE);   

  /* Test on EV5 and clear it */
  CODECTimeout = CODEC_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {
    // if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }
  
  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(CODEC_I2C, 0x91, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  CODECTimeout = CODEC_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
   //  if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }

  /* Transmit the first address for write operation */
  I2C_SendData(CODEC_I2C, RegisterAddr);

  /* Test on EV8 and clear it */
  CODECTimeout = CODEC_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
  {
    // if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }
  I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
  /* Prepare the register value to be sent */
  I2C_SendData(CODEC_I2C, RegisterValue);
  
  /*!< Wait till all data have been physically transferred on the bus */
  CODECTimeout = CODEC_LONG_TIMEOUT;
  while(!I2C_CheckEvent(CODEC_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED))// I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF))
  {
    // if((CODECTimeout--) == 0) Codec_TIMEOUT_UserCallback();
  }
  
  /* End the configuration sequence */
  I2C_GenerateSTOP(CODEC_I2C, ENABLE);  
 
	
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return result;  
}
uint8_t hostRead(uint8_t * datas)
{
  uint32_t result = 0;

  /*!< While the bus is busy */
  CODECTimeout = CODEC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
  {
    if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }
  
  /* Start the config sequence */
  I2C_GenerateSTART(CODEC_I2C, ENABLE);

  /* Test on EV5 and clear it */
  CODECTimeout = CODEC_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {
    // if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }
  
  /*!< Send Codec address for read */
  I2C_Send7bitAddress(CODEC_I2C,CODEC_ADDRESS,I2C_Direction_Receiver);  
  
  /* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
  CODECTimeout = CODEC_FLAG_TIMEOUT;
  while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_ADDR) == RESET)
  {
    // if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }     

  CODECTimeout = CODEC_FLAG_TIMEOUT;
  while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_RXNE) == RESET)
  {
    // if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }  
  /*!< Read the byte received from the Codec */
  datas[0] = I2C_ReceiveData(CODEC_I2C);
  
  /*!< Disable Acknowledgment */
  I2C_AcknowledgeConfig(CODEC_I2C, DISABLE);   
  
  /* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
  (void)CODEC_I2C->SR2;
  
  /*!< Send STOP Condition */
  I2C_GenerateSTOP(CODEC_I2C, ENABLE);
  
  /* Wait for the byte to be received */
  CODECTimeout = CODEC_FLAG_TIMEOUT;
  while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_RXNE) == RESET)
  {
    // if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }
  
  /*!< Read the byte received from the Codec */
  datas[1] = I2C_ReceiveData(CODEC_I2C);
  
  /* Wait to make sure that STOP flag has been cleared */
  CODECTimeout = CODEC_FLAG_TIMEOUT;
  while(CODEC_I2C->CR1 & I2C_CR1_STOP)
  {
    // if((CODECTimeout--) == 0) return Codec_TIMEOUT_UserCallback();
  }  
  
  /*!< Re-Enable Acknowledgment to be ready for another reception */
  I2C_AcknowledgeConfig(CODEC_I2C, ENABLE);  
  
  /* Clear AF flag for next communication */
  I2C_ClearFlag(CODEC_I2C, I2C_FLAG_AF); 
  
  /* Return the byte read from Codec */
  return result;
}

#endif
