
#ifndef _I2CConfigDeclaration_H_
#define _I2CConfigDeclaration_H_
#include"includes.h"
#define MAXINPUT   16
#define PORT1 0
#define PORT2 1
#define PORT3 2
typedef enum{
	LED=0,
	LCD=1
}Mode;

typedef struct      // 用于存放端口数据
{
	char name[10];
	Mode modeType;
	uint8_t dir;     // 数据传输方向
	uint8_t Online;  // 模块取得数据成功否 
	uint16_t * DataPoint;  // 数据指针 
	uint8_t DeviveID;      // 设备ID
}COMDATA;
extern COMDATA comData[MAXINPUT];
extern uint16_t DatasUsedToInput[16][2];
/*
*  modes    模块类型号 
*  comPort  管脚号
*  buff     模块的名字，必须和其他的模块名字不同
**/
void PortConfig(Mode modes,uint8_t comPort,char buff[]);
void DriverComfig(Mode modes,uint8_t comPort,char buff[]);
/* 通过 名称返回取得的数据 */
uint16_t getValue(char buff[]);
void ModeConfig(void);
#endif 

