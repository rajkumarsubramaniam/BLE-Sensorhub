/**************************************************************************************
 * @file ACMP_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the ACMP functionalities
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * ACMP0 Light sense Configurations
 *****************************************************************************/
void ACMP0_Setup()
{
	/* Set configurations for LETIMER 0*/
	ACMP_Init_TypeDef acmp0Char;
	acmp0Char.biasProg = 7;						//Set the bias prog
	acmp0Char.enable = false;					//Starts the ACMP after initialization
	acmp0Char.fullBias = true;					//Set the full Bias current
	acmp0Char.halfBias = true;					//Set the half bias current
	acmp0Char.hysteresisLevel = acmpHysteresisLevel7;	//Hysteresis level
	acmp0Char.inactiveValue = false;			//Sets teh value of comparator to 0 when inactive
	acmp0Char.interruptOnFallingEdge = false;	//falling edge interrupt disabled
	acmp0Char.interruptOnRisingEdge = false; 	//Rising edge interrupt disabled
	acmp0Char.vddLevel = DARKNESS_REF;		 	//Set the VDD level to (61*VDD/63)
	acmp0Char.warmTime = acmpWarmTime256 ;	 	//Setting the warm up time

	ACMP_Init(ACMP0, &acmp0Char);				//Initializing ACMP
	ACMP_ChannelSet(ACMP0, ACMP_CHANNEL_REF, ACMP_CHANNEL_INPUT );		//Setting the POS and Neg inputs
}

/*****************************************************************************
 * ACMP1 Capacitive sense Configurations
 *****************************************************************************/
void ACMP1_Setup()
{
	static const ACMP_CapsenseInit_TypeDef acmpInit =
	{
		.fullBias                 = true,            //Configured according to application note
		.halfBias                 = true,            //Configured according to application note
		.biasProg                 = 0x5,             //Configured according to application note
		.warmTime                 = acmpWarmTime512, //LESENSE uses a fixed warm up time
		.hysteresisLevel          = acmpHysteresisLevel5, //Configured according to application note
		.resistor                 = acmpResistor0,   //Configured according to application note
		.lowPowerReferenceEnabled = false,           //LP-reference can introduce glitches with cap touch
		.vddLevel                 = VDD_LEVEL_75,    //Configured according to application note
		.enable                   = false            //LESENSE enables the ACMP
	};
	 ACMP_CapsenseInit(ACMP1, &acmpInit);
}
