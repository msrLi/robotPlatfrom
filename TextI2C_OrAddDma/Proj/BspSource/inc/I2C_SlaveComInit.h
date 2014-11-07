
#ifndef _I2C_SlaveComInit_H_
#define _I2C_SlaveComInit_H_
#include"stm32f4xx.h"
/* I2C peripheral configuration defines (control interface of the audio codec) */
#define CODEC_I2C                      I2C1
#define CODEC_I2C_CLK                  RCC_APB1Periph_I2C1
#define CODEC_I2C_GPIO_CLOCK           RCC_AHB1Periph_GPIOB
#define CODEC_I2C_GPIO_AF               GPIO_AF_I2C1
#define CODEC_I2C_GPIO                 GPIOB
#define CODEC_I2C_SCL_PIN              GPIO_Pin_6
#define CODEC_I2C_SDA_PIN              GPIO_Pin_9
#define CODEC_I2S_SCL_PINSRC           GPIO_PinSource6
#define CODEC_I2S_SDA_PINSRC           GPIO_PinSource9
/* config as defines */
#define I2C_SPEED                      400000
#define CODEC_ADDRESS    							 0x90
#define I2C1_DR_Address        0x40005410
#define BufferSize     7

void Init_I2C1(void);
void DMA_ConfigForI2C1Recerve(void);
void SalveRecerveI2C(void);
#endif 
/******************* (C) COPYRIGHT 2014 MacRobot *****END OF FILE****/
