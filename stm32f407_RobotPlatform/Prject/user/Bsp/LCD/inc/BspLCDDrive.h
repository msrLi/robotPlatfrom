#ifndef _BSPLCD_DRIVE_H_
#define _BSPLCD_DRIVE_H_
#include"stm32f4xx.h"
#include"BspLcdHal_Init.h"
#define DATA_SEND_BASE  ((uint32_t)0x60000000) // *((__IO uint16_t*) 0x60000000) volatile
//#define DA    
/*
#define DATA    (*(volatile unsigned short int *) DATA_SEND_BASE)
#define COMAND   (*(volatile unsigned short int *) (DATA_SEND_BASE | 0x20000)
*/

#define DATA    (*(volatile uint16_t *)((uint32_t)0x60020000))    //RS=0 A16=0
#define COMAND   (*(volatile uint16_t *) ((uint32_t)0x60000000))  //RSA16=1

#define  writeData(x)    DATA=x
#define  writeComd(x)  	 COMAND=x
   /*************************Globe inter ***************************/
// #include"stm32f10x_conf.h"
#define MAX_W 800
#define MAX_H 480
/* color define */
#define White          0xFFFF
#define Black          0x0000
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0
//-------------------------显示颜色-------------------------------------// 
/** 
 *name:	   	 Lcd_Init
 *function:  屏幕初始化
**/
void Lcd_Init(void);
/** 
 *name:	   	 Pant
 *function:  清屏或刷屏
**/
void Pant(unsigned int color);
/** 
 *name:	   	 Pant_pix
 *function:  写数据点
**/
void Pant_pix(unsigned int x,unsigned int y,unsigned int color);
/** 
 *name:	   	 Red_pix
 *function:  读显示点
**/
int Red_pix(unsigned int x,unsigned int y);
/** 
 *name:	   	 Address_set
 *function:  设置写卡区域
**/
void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2);
void Pant_clear(unsigned int x0,unsigned int y0,unsigned int x1,unsigned int y1,unsigned int color);
#endif
