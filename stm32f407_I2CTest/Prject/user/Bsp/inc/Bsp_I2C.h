#ifndef _BSP_I2C_H_
#define _BSP_I2C_H_
#include "stm32f4xx.h"
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
#define I2C_SPEED                      100000
#define CODEC_ADDRESS    							 0x90

#define Interrupt 1    // allow interrupt 
#define I2C_1_IRQ_PRE_PRI    1 
#define I2C_1_IRQ_SUB_PRI    0

/* used as times */
#define CODEC_FLAG_TIMEOUT             ((uint32_t)0x1000)
#define CODEC_LONG_TIMEOUT             ((uint32_t)(300 * CODEC_FLAG_TIMEOUT))
static void _I2C_IOInit(void);
static void Codec_CtrlInterface_Init(void);
void Bsp_I2C_init(void);
uint32_t Codec_WriteRegister(uint8_t RegisterAddr, uint8_t RegisterValue);
uint8_t hostRead(uint8_t * datas);
void salveRead(uint8_t * dataBuff);
void salveSend(void);
static void NVIC_I2C_Interrupt(void);
#endif
