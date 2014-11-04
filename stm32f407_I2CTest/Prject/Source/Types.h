#ifndef _TYPES_H_
#define _TYPES_H_
/* 用于管脚 查询*/
typedef  uint8_t PinName;

/* InPut and OutPut mode */
typedef enum Mode
{
	 In=0,
	 Out
}Mode;
/* used as OutPut Pins Low or High */
typedef enum Station
{ 
	Low=0,
	High,
	Tristate
}Station;

#endif
