
#include"stm32f4xx.h"
#ifdef I2C_Salve
	#include"I2C_SlaveComInit.h"
#else
	#include"I2C_MasterComInit.h"
#endif

int main(void)
{
	// return 0;
	Init_I2C1();
	while(1)
	{
		#ifdef I2C_Salve
		{   // ´úÂë¿é 1 
			SalveRecerveI2C();
		}
		#else
		{		 // ´úÂë¿é 2 
			SendDataBy_I2C1();
		}
		#endif
	}
}
/******************* (C) COPYRIGHT 2014 MacRobot *****END OF FILE****/
