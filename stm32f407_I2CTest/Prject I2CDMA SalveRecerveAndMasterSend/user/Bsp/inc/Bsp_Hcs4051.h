#ifndef _Bsp_Hcs4051_H_
#define  _Bsp_Hcs4051_H_
#include"stm32f4xx.h"
#define  ABC_IO_CLOCK  RCC_AHB1Periph_GPIOA
#define ABC_PORT   GPIOA
#define A_PIN    GPIO_Pin_0
#define B_PIN    GPIO_Pin_1
#define C_PIN    GPIO_Pin_2

#define EN_IO_CLOCK    RCC_AHB1Periph_GPIOA
#define EN_PORT  GPIOA
#define EN_PIN   GPIO_Pin_3

void Bsp_Hcs4051_Init(void);
void Bsp_HcsSlect(uint8_t adds);
void Bsp_Hcs_disEn(FunctionalState NewState);

#endif
