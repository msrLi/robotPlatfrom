#include"Bsp_Led.h"
uint32_t LedTimeDelay;
void Bsp_InitLed(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_ClocksTypeDef Rcc_get;   // ¸÷Â·Ê±ÖÓ 
	RCC_GetClocksFreq(&Rcc_get);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	SysTick_Config(Rcc_get.HCLK_Frequency/100);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LEDLeft | LEDUp | LEDRight | LEDDown | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_SetBits(LEDLeft_PORT,LEDLeft);  // 
  GPIO_SetBits(LEDUp_PORT,LEDUp);
  GPIO_SetBits(LEDRight_PORT,LEDRight);
  GPIO_SetBits(LEDDown_PORT,LEDDown);
}
void LEDDelay(uint32_t times)
{
		if(times) LedTimeDelay=times;
		while(LedTimeDelay);
}
void LED_Change(uint8_t sw)
{
	switch(sw)
	{
		case 0:
					 GPIO_ToggleBits(LEDLeft_PORT,LEDLeft);
					 GPIO_ToggleBits(LEDUp_PORT,LEDUp);
					 GPIO_ToggleBits(LEDRight_PORT,LEDRight);
					 GPIO_ToggleBits(LEDDown_PORT,LEDDown);			
			break;
		case 1:
			GPIO_ResetBits(LEDUp_PORT,LEDUp);
			break;
		case 2:break;
		case 3:break;
		default:break;
			
	}
}
