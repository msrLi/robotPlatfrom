
#include"DeviceAll.h"
#define Address  0x30
/* 
*   开始 标志 请求 
**/
uint16_t FlagForOK=0;
void AccessToInformation(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t Das;
	OS_ERR   err;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;    //  | GPIO_Pin_9;   // scl--PB6 ; SDA--PB9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
	Das=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
	if(Das==1)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
		GPIO_SetBits(GPIOB,GPIO_Pin_7);	

			
		FlagForOK=FlagForOK<<1;
		FlagForOK++;
	}
}
void askForData(void)
{
	I2CxInformation __IO * I2CConfigPoint;
	OS_ERR      err;
	uint8_t recerveBuff[6];
	/* 配置 发送配置 */
	I2CConfigPoint=GetConfig();	
	I2CConfigPoint->comState=COMM_PRE;      // 发送前面
	I2CConfigPoint->comDir=MSTER_RECERVE;		// 接收模式
	I2CConfigPoint->configLength=0;         // 可变长度0 
	I2CConfigPoint->slveAddress=Address;
	masterRecerveData(I2CConfigPoint->slveAddress,recerveBuff,6);  // 等待接收函数

}
void slectPortUesd(uint8_t Port,uint8_t isOk)
{
	GPIO_TypeDef* gpioBase;
	uint16_t pinNumber;
	uint8_t x;
	Port++;
	switch(Port)
	{
		case 1: 
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
	  case 12:
		case 13:
		case 14:	
		case 15:  
		case 16:	gpioBase=GPIOB;pinNumber=GPIO_Pin_7;break;
		default :break;
	}
	if(isOk){
		GPIO_ResetBits(gpioBase,pinNumber);
	}else{
		GPIO_SetBits(gpioBase,pinNumber);
	}
}



