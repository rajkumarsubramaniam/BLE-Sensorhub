/**************************************************************************************
 * @file CMU_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This stores all the macros for the project
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * CMU configurations
 *****************************************************************************/
void CMU_Setup()
{
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	CMU_ClockEnable(cmuClock_HFPER, true);		//Setting up the clock tree
	CMU_ClockEnable(cmuClock_CORELE, true);		//Enables the core clock
	CMU_ClockEnable(cmuClock_GPIO, true);		//Enables GPIO peripheral
	CMU_ClockEnable(cmuClock_LESENSE, true);	//Enables LESENSE
	CMU_ClockEnable(cmuClock_ACMP0, true);		//Enables ACMP0
	CMU_ClockEnable(cmuClock_ACMP1, true);		//Enables ACMP1
	CMU_ClockEnable(cmuClock_I2C1, true);		//Enable I2C1 peripheral
	CMU_ClockEnable(cmuClock_LEUART0, true);    //Enable LEUART1 clock
	CMU_ClockEnable(cmuClock_LETIMER0, true);	//Enables LETIMER0
	CMU_ClockEnable(cmuClock_ADC0, true);		//Enable ADC0 peripheral
	CMU_ClockEnable(cmuClock_DMA, true);		//Enable ADC0 peripheral
}

