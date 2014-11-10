
#include"I2CConfigDeclaration.h"
#include"String.h"
uint16_t DatasUsedToInput[MAXINPUT][2];
uint16_t DatasUsedToOutput[16][2];
COMDATA comData[MAXINPUT];
uint8_t comUsedNumber=0;
extern void Config(void);
/* 通过 名称返回取得的数据 */
uint16_t getValue(char buff[])
{
	uint8_t i=0;
	for(i=0;i<16;i++)
	{
		if(strcmp(buff,comData[i].name)==0)  // 两个相等
		{
			return (*comData[i].DataPoint);
		}			
	}
	return 0xffff;
}
/*  用于配置输入设备  */
void PortConfig(Mode modes,uint8_t comPort,char buff[])
{
	uint8_t i;
	if(comUsedNumber >=16) return ;        // 超出数据最大限度
	comData[comUsedNumber].modeType=modes; // 函数模式  
	comData[comUsedNumber].dir=1;          // 数据输出 
	i=strlen(buff);
	if(i>0 && i<10){
	  strcpy(comData[comUsedNumber].name,buff);
	}
	comData[comUsedNumber].Online=0;
	comData[comUsedNumber].DataPoint= DatasUsedToInput[comUsedNumber];
	comUsedNumber++;	
}
/*  用于配置输出设备 */
void DriverComfig(Mode modes,uint8_t comPort,char buff[])
{
	uint8_t i;
	if(comUsedNumber >=16) return ;  // 超出数据最大限度
	comData[comUsedNumber].dir=0;   // 数据输出 
	i=strlen(buff);
	if(i>0 && i<10){
	  strcpy(comData[comUsedNumber].name,buff);
	}
	comData[comUsedNumber].Online=0;
	comData[comUsedNumber].DataPoint= DatasUsedToOutput[comUsedNumber];
	comUsedNumber++;
}
void ModeConfig(void)
{
	// 
	Config();
	
}

