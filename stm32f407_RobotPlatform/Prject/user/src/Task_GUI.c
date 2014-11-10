#include"Task_GUI.h"

void AppTaskGUI(void *p_arg)
{
	(void) p_arg;
	 while(1)
	 {
		  Function();
		  BSP_OS_TimeDly(1);
		  
	 }
}

void AppTaskGUIRefresh(void *p_arg)
{
	 (void)p_arg;
	while(1)
	{
		 BSP_OS_TimeDly(100);
	}
}
