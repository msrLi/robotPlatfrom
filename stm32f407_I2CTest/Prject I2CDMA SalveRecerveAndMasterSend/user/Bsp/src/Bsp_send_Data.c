
#include"Bsp_send_Data.h"
I2C_InformationSendAndRecerve PWMSendI2C_Par;
uint8_t buff[3]={0x89,0x66,0x32};
void SendPWM(uint8_t *PWMValue)
{
	I2C_Return PwmRe;
	PWMSendI2C_Par.DeviceAddr=0x91;
	PWMSendI2C_Par.I2C_DIRECTION=ARC_I2C_DIRECTION_TX; 
	PWMSendI2C_Par.ok=0;
	PWMSendI2C_Par.TxData=buff;
	PWMSendI2C_Par.TxNumOfBytes=2;
	PWMSendI2C_Par.Tx_I2C_Index=0;
	PWMSendI2C_Par.TX_Generate_stop=1;
	PwmRe=I2C_Master_Transmitter(&PWMSendI2C_Par);
	if(PwmRe==I2C_Ok)
	{
		PwmRe=I2C_Ok;
	}
}
