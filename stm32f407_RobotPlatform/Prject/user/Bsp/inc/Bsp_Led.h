#ifndef _BSP_LED_H_
#define _BSP_LED_H_
#include "stm32f4xx.h"
#include "bsp.h"
#define LEDLeft_PORT  GPIOD
#define LEDLeft  GPIO_Pin_12 
#define LEDUp_PORT  GPIOD
#define LEDUp  GPIO_Pin_13 
#define LEDRight_PORT  GPIOD
#define LEDRight  GPIO_Pin_14 
#define LEDDown_PORT  GPIOD
#define LEDDown  GPIO_Pin_15 
void Bsp_InitLed(void);
void LED_Change(uint8_t sw);
#endif
