#include"PinBase.h"

void Pin_Init(PinName IO_x,Mode Mo,Station St)
{
	 GPIO_InitTypeDef GPIO_InitStructure;	 
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	 GPIO_InitStructure.GPIO_Pin=IO_x;
	 GPIO_InitStructure.GPIO_Mode=
	 if(IO_x)
	 {
		  
	 }else{
		 
	 }
}
