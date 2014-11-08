/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
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
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include"SHT10.h"
#include<stdio.h>
#include<SHT10_01.h>
// #include"Usart.h"
/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#define LED_GPIO_PORT  (GPIOD)
#define LED_GPIO_PINS  (GPIO_PIN_3)

#define I2C_SPEED 2000000 

uint8_t GetUart1Data[4];

//static void init_usart1(void);
static void init_I2C(void);
void Send(uint8_t dat);
void SendSting(char * str);
void recerve_usart_intdeal(void);
void readI2C(uint8_t * buff);
void slaveSend(void);
void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0 )
  {   
    nCount--;
  }
}
uint8_t sendBuff[8]={0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
void slaveSend(void)
{
  uint8_t i;
   //EV1 ADDR=1 通过读 SR1 然后读 SR3 将该事件清除 
  /* TRA, BUSY, TXE and ADDR flags */
  while(!I2C_CheckEvent(I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED));  
  
  I2C_SendData(sendBuff[0]);
  for(i=1;i<6;i++)
  {
    while(!I2C_CheckEvent(I2C_EVENT_SLAVE_BYTE_TRANSMITTED));  //EV3
    I2C_SendData(sendBuff[i]);
  }
  while(!I2C_CheckEvent(I2C_EVENT_SLAVE_ACK_FAILURE));
  I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
  
}
void main(void)
{   
  /* Initialize I/Os in Output Mode */
//  GPIO_Init(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
//   GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_MODE_OUT_PP_LOW_FAST);
// GPIO_WriteLow(GPIOB,GPIO_PIN_5);
  init_I2C();


  // addressUsart=0x02;
  //enableInterrupts(); 
  while (1)
  {
    // Delay(0xffff);
    // GPIO_WriteReverse(GPIOB,GPIO_PIN_5);
    slaveSend();
    // readI2C(GetUart1Data);
  }
  
}
void readI2C(uint8_t * buff)
{
  
  while(!I2C_CheckEvent(I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED));
  while (!I2C_CheckEvent(I2C_EVENT_SLAVE_BYTE_RECEIVED));  // wait fo data recerved
  buff[0]= I2C_ReceiveData();
  while (!I2C_CheckEvent(I2C_EVENT_SLAVE_BYTE_RECEIVED));  // wait fo data recerved
  buff[1]= I2C_ReceiveData();
  // wait for end recerve 
  while (!I2C_CheckEvent(I2C_EVENT_SLAVE_STOP_DETECTED));
  I2C_GenerateSTOP(ENABLE);  
  
}
static void init_I2C(void)
{
  I2C_DeInit();
  I2C_Cmd(ENABLE);
  I2C_Init(200000, 0x30, I2C_DUTYCYCLE_2,\
		I2C_ACK_CURR, I2C_ADDMODE_7BIT, CLK_GetClockFreq()/2000000);
  // I2C_AcknowledgeConfig(I2C_ACK_CURR);   // enable ack 
}
//static void init_usart1(void)
//{
//  UART1_DeInit();
//  /* 波特率 9600 9位数据  1个停止位 。  无奇偶校验  */
//  UART1_Init((u32)115200, UART1_WORDLENGTH_9D, UART1_STOPBITS_1, UART1_PARITY_NO, 
//           UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
//  UART1_SetAddress(addressUsart);   // set add as 0x01 
//  UART1_ReceiverWakeUpCmd(ENABLE);  // RWU set as 1 
//  UART1_WakeUpConfig(UART1_WAKEUP_ADDRESSMARK);//  Address wakup   地址唤醒 
//  UART1_ClearITPendingBit(UART1_IT_RXNE); 
//  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
//}
//void recerve_usart_intdeal(void)
//{
//  uint8_t i;
//  if(GetUart1Data[0]==addressUsart)  //地址判断，，符合则将自己的信息输出 不符合那么退出运行
//  { 
//    GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS);
//    GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
//    Send(addressUsart);
//    Send('A');
//    for(i=0;i<9/*4*sizeof(uint16_t)*/;i++)
//    {
//       Send(sendBuff[i]);
//    }
//    GPIO_WriteLow(GPIOA,GPIO_PIN_3);
//  }
//  ReceFLag=0;
//}
//void Send(uint8_t dat)
//{
//  while(( UART1_GetFlagStatus(UART1_FLAG_TXE)==RESET));	
//    UART1_SendData8(dat);	
//}
//void SendSting(char * str)
//{
//  while(*str++)
//  {
//    Send(*str);
//  }
//}
///*
// *   fputc
// */
//int fputc(int ch, FILE *f)
//{
//	/* ??Printf?úèY·￠íù′??ú */
//    UART1_SendData8((uint8_t) ch);
//    while(( UART1_GetFlagStatus(UART1_FLAG_TXE)!=SET));	
//    return (ch);
//}
#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE*****/

