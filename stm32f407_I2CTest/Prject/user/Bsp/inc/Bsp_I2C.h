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
typedef enum{
	ARC_I2C_DIRECTION_TX=0,ARC_I2C_DIRECTION_RX=!ARC_I2C_DIRECTION_TX
}I2C_Statiom;
typedef enum{
	I2C_Ok=0,I2C_Error=1
}I2C_Return;
typedef struct{
  uint8_t ok;
	uint8_t DeviceAddr; 
	I2C_Statiom I2C_DIRECTION;  // used this I2C is Transmitter or Receiver
	/* Master Transmitter */
	uint8_t TX_Generate_stop;  // if 1 send stop byte  else turn to as station of T
	uint8_t TxNumOfBytes;    // send numbers of byte data 
	uint8_t Tx_I2C_Index;    // numbers of had Sends 
	uint8_t * TxData;        // Tx data Point 
	
	/* Master Receiver */
	uint8_t RxNumOfBytes;    // recerve number of byte data 
	uint8_t RX_I2C_Index;    // numbers of had recerve 
	uint8_t * RxData;			   // Rx data Point 
}I2C_InformationSendAndRecerve;


// extern I2C_InformationSendAndRecerve * I2C_InterruptBuff;

static void _I2C_IOInit(void);
static void Codec_CtrlInterface_Init(void);
void Bsp_I2C_init(void);
uint32_t Codec_WriteRegister(uint8_t RegisterAddr, uint8_t RegisterValue);
uint8_t hostRead(uint8_t * datas);
void salveRead(uint8_t * dataBuff);
void salveSend(void);
static void NVIC_I2C_Interrupt(void);
void ARC_SetI2C_Information(I2C_InformationSendAndRecerve * P_setValue);
I2C_InformationSendAndRecerve * ARC_GetI2C_Information(void);
/* 
  I2C  ·¢ËÍ 
  return 0 ok 
				 1 error 
*/ 
I2C_Return  I2C_Master_Transmitter(I2C_InformationSendAndRecerve * I2CPram);   
I2C_Return  I2C_Master_Recerve(I2C_InformationSendAndRecerve * I2CPram);
#endif
