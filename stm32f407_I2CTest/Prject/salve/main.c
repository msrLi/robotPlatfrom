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
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/
uint8_t recerveData[3];
int main(void)
{ 
//	RCC_ClocksTypeDef Rcc_get;   // ¸÷Â·Ê±ÖÓ 
//	RCC_GetClocksFreq(&Rcc_get);
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	Bsp_InitLed();
//	GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_RESET);
//	LEDDelay(10);
//	GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_SET);
	Bsp_I2C_init();
	RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, DISABLE);
	RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);
//	SysTick_Config(Rcc_get.HCLK_Frequency/100);
	//BSP_Init();	
	//BSP_Tick_Init();
  while(1)
	{
		
		 // salveSend();
	  LED_Change(0);
	  salveRead(recerveData);
//		if(recerveData[0]==0x02 || recerveData[1]== 0x78)
//		{
//			LED_Change(1);
//		}
		// Codec_WriteRegister(0x02,0x01);
	}
	
}

  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
