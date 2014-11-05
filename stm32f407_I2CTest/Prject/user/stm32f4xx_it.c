/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include"stm32f4xx_it.h"
#include"Bsp_I2C.h"

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint32_t LedTimeDelay;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
void I2C1_EV_IRQHandler(void)
{
	uint32_t i2cEvent;
	I2C_InformationSendAndRecerve __IO * I2C_InterruptBuff;
  I2C_InterruptBuff=ARC_GetI2C_Information();  // get config address 
	i2cEvent=I2C_GetLastEvent(I2C1);  // h获取状态标志位
	switch(i2cEvent)  
	{
		 /*　设备　处于　I2C 主机 成功 */
		case  I2C_EVENT_MASTER_MODE_SELECT: // EV5
			I2C_AcknowledgeConfig(I2C1, ENABLE);  // 允许发送 响应信号 
			// send adds as Transmitter
		  // i2cEvent=ARC_I2C_DIRECTION_TX
		  // 发送 从机地址  和 传输数据 命令 
		// if(I2C_InterruptBuff->)
			if(I2C_InterruptBuff->I2C_DIRECTION == ARC_I2C_DIRECTION_TX)
			{
				I2C_Send7bitAddress(I2C1, I2C_InterruptBuff->DeviceAddr, I2C_Direction_Transmitter); 
			}else{
				I2C_Send7bitAddress(I2C1, I2C_InterruptBuff->DeviceAddr, I2C_Direction_Receiver);   // 发送从机地址和请求数据命令
			}				
			break;
		/* Master Transmitter -------------------------------------------------------*/
		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:  // EV 6 ADDR=1  /* 地址发送成功 */
			I2C_SendData(I2C1,I2C_InterruptBuff->TxData[I2C_InterruptBuff->Tx_I2C_Index++]);  // send data 
		  // end and disable TX buff 
			if(I2C_InterruptBuff->TxNumOfBytes==1)
			{
				I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);  // 禁止缓冲中断，，发送结束 TxE=1 不进中断
			}
			break;
		case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  /* Without BTF, EV8 */   // 发送 数据寄存器空 发送 为完成 
			if(I2C_InterruptBuff->RX_I2C_Index<I2C_InterruptBuff->TxNumOfBytes)
			{
				I2C_SendData(I2C1,I2C_InterruptBuff->TxData[I2C_InterruptBuff->Tx_I2C_Index++]);
			}else{
				I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);  // end and disable TX buff 
			}		
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED: /* With BTF EV8-2 */   // 发送最后一个数据完成程序停止请求
		/*    产生结束信号 */
			if(I2C_InterruptBuff->TX_Generate_stop==1)
			{
				I2C_GenerateSTOP(I2C1, ENABLE);                // 产生停止位 
				I2C_InterruptBuff->ok=1;
				I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);		
			}else{  
				/* 不产生结束   转换成接受模式 开始接收*/
				I2C_InterruptBuff->I2C_DIRECTION=ARC_I2C_DIRECTION_RX; // 转换到 接收 模式
				I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);                //  开启缓冲中断
				I2C_GenerateSTART(I2C1, ENABLE);										 // 产生开始信号 转到主机接收模式
			}	
		break;
			
		/* Master Recerve ------------------------------------------------------------*/
		case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:   //  EV6  数据地址和接收模式匹配  总线处于忙状态   BUSY, MSL and ADDR flags 
			/* 判断 发送最后一个字节前  停止 ACK应答 并产生 停止信号  */
		/*
		if(pI2C_param->RX_I2C_Index == (pI2C_param->RxNumOfBytes - 1))
		{
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				I2C_GenerateSTOP(I2C1, ENABLE);
		}*/
		break;
		case I2C_EVENT_MASTER_BYTE_RECEIVED:  // EV7  数据接收成功 
			/* 注意 接收最后一个数据前 停止发送响应信号并且发送停止信号*/
//			pI2C_param->RxData[pI2C_param->RX_I2C_Index++] = I2C_ReceiveData (I2C1);
//			if(pI2C_param->RX_I2C_Index == (pI2C_param->RxNumOfBytes - 1))
//			{
//				I2C_AcknowledgeConfig(I2C1, DISABLE);
//				I2C_GenerateSTOP(I2C1, ENABLE);
//			}			
		break;
	}
}
/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    if(LedTimeDelay)
			LedTimeDelay--;
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles External line 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{

}

#if defined MEDIA_USB_KEY
/**
  * @brief  EXTI0_IRQHandler
  *         This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{

}


/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  // USB_OTG_BSP_TimerIRQ();
}



/**
  * @brief  This function handles USB-On-The-Go FS global interrupt request.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  // USBH_OTG_ISR_Handler(&USB_OTG_Core);
}
#endif /* MEDIA_USB_KEY */

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
