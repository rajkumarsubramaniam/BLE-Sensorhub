/**************************************************************************************
 * @file Energy_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the sleep mode of the Leopard Gecko
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * Block Sleep mode
 *****************************************************************************/
void blockSleepMode(EnergyModes minSleepMode)
{
	INT_Disable();
	sleepMaintainer[minSleepMode]++;
	INT_Enable();
}
/*****************************************************************************
 * Unblock sleep mode
 *****************************************************************************/
void unblockSleepMode(EnergyModes minSleepMode)
{
	INT_Disable();
	sleepMaintainer[minSleepMode]--;
	INT_Enable();
}
/*****************************************************************************
 * Sleep control
 *****************************************************************************/
void sleep()
{
	if(sleepMaintainer[0]>0)
		return;
	else if(sleepMaintainer[1]>0)
	 	EMU_EnterEM1();		//Enters into EM1
	else if(sleepMaintainer[2]>0)
		EMU_EnterEM2(true);	//Enters EM2
	else if(sleepMaintainer[3]>0)
		EMU_EnterEM3(true); //Enters EM3
	else if(sleepMaintainer[4]>0)
		EMU_EnterEM4();		//Enters EM4
	else
	{}
	return;
}

