#include"Bsp_Hcs4051.h"
/** 
* name : 
* function £º ³õÊ¼»¯ IO¶Ë¿Ú 
*/
void Bsp_Hcs4051_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	// enable IO  clock 
	RCC_AHB1PeriphClockCmd(ABC_IO_CLOCK | EN_IO_CLOCK, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = A_PIN | B_PIN | C_PIN;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ABC_PORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = EN_PIN;
	GPIO_Init(EN_PORT, &GPIO_InitStructure);	
	// let HCS4051 as not used  out put 
	GPIO_SetBits(EN_PORT,EN_PIN);
}
/** 
*
*/
void Bsp_HcsSlect(uint8_t adds)
{
	uint8_t i;
	BitAction enumData[2]={Bit_RESET,Bit_SET};
	GPIO_ResetBits(ABC_PORT,B_PIN);
	GPIO_ResetBits(ABC_PORT,C_PIN);
	GPIO_SetBits(ABC_PORT,A_PIN);
//	if(adds>=8) return;
//	// set high Pin station 
//	i=adds%4;
//	// if(i!=0) GPIO_SetBits(ABC_PORT,C_PIN); else GPIO_ResetBits(ABC_PORT,C_PIN);
//	GPIO_WriteBit(ABC_PORT,C_PIN,enumData[i]);
//	
//	adds-=i*4;
//	i=adds%2;
//	  // Bit_RESET
//	GPIO_WriteBit(ABC_PORT,B_PIN,enumData[i]);
//	adds-=i*2;
//  GPIO_WriteBit(ABC_PORT,A_PIN,enumData[adds]);
		
}
void Bsp_Hcs_disEn(FunctionalState NewState)
{
	if(NewState==DISABLE)
		GPIO_SetBits(EN_PORT,EN_PIN);
	else 
		GPIO_ResetBits(EN_PORT,EN_PIN);
}

