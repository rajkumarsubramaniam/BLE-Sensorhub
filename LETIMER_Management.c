/**************************************************************************************
 * @file LETIMER_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the LETIMER operations
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * LETIMER configurations
 *****************************************************************************/
void LETIMER_Setup()
{
	int COMP0_SET;
	int COMP1_SET;
	int LETIMER_PS;
	int flagInt;
	/* Set configurations for LETIMER 0. This is referred from the example of Silicon Labs*/
	LETIMER_Init_TypeDef letimerInit;
	letimerInit.enable         = false;                 /* Start counting when init completed. */
	letimerInit.debugRun       = false;                 /* Counter shall not keep running during debug halt. */
	letimerInit.rtcComp0Enable = false;                 /* Don't start counting on RTC COMP0 match. */
	letimerInit.rtcComp1Enable = false;                 /* Don't start counting on RTC COMP1 match. */
	letimerInit.comp0Top       = true;                  /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	letimerInit.bufTop         = false;                 /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	letimerInit.out0Pol        = 0;                     /* Idle value for output 0. */
	letimerInit.out1Pol        = 1;                     /* Idle value for output 1. */
	letimerInit.ufoa0          = letimerUFOANone;       /* PWM output on output 0 */
	letimerInit.ufoa1          = letimerUFOANone;       /* Pulse output on output 1*/
	letimerInit.repMode        = letimerRepeatFree;     /* Count until stopped */
	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);
	/*Setting LED OFF TIME*/
	if(SELECTED_EM == ENERGYMODE3)
	{
		float COMP0_Count = (LED_OFF_TIME * ULFRCO_FREQUENCY * oscRatio);///(1<<LETIMER_PS);
		COMP0_SET = (COMP0_Count > (int)COMP0_Count)? (COMP0_Count+1): COMP0_Count;	//COMP0 register value will have the count for the OFF time of the LED.
	}
	else
	{		// Setting the LETIMER Pre-scalar value
		while((LED_OFF_TIME*LFXO_FREQUENCY/(1<<LETIMER_PS)) > MAX_COUNT)
		{
			LETIMER_PS++;
		}
		CMU->LFAPRESC0 &= ~CMU_LFAPRESC0_LETIMER0_DIV32768;					// Clearing the CMU_LFAPRESC0, LETIMER0 data
		CMU->LFAPRESC0 |= LETIMER_PS << SHIFT_EIGHT_BITS;				//Assigning pre-scalar value to the register
		float COMP0_Count = LED_OFF_TIME *(LFXO_FREQUENCY /(1<<LETIMER_PS));	// Fixing the counter value based on the pre-scalar for LFXO
		COMP0_SET = (COMP0_Count > (int)COMP0_Count)? (COMP0_Count+1): COMP0_Count;	//COMP0 register value will have the count for the OFF time of the LED.
	}
	LETIMER0->CNT = COMP0_SET;					//Load the COMP0 register to the counter
	LETIMER_CompareSet(LETIMER0, 0, COMP0_SET); // LOad the COMP0 register

	/*Setting LED ON TIME*/
	if(SELECTED_EM == ENERGYMODE3)
	{
		float COMP1_Count = (LED_ON_TIME * ULFRCO_FREQUENCY * oscRatio);///(1<<LETIMER_PS);
		COMP1_SET = (COMP1_Count > (int)COMP1_Count)? (COMP1_Count+1): COMP1_Count;
	}
	else
	{
		float COMP1_Count = LED_ON_TIME *(LFXO_FREQUENCY /(1<<LETIMER_PS));
		COMP1_SET = (COMP1_Count > (int)COMP1_Count)? (COMP1_Count+1): COMP1_Count;
	}
	LETIMER_CompareSet(LETIMER0, 1, COMP1_SET);
	while(LETIMER0->SYNCBUSY != 0);					//Waiting for the SYNC to complete
	flagInt = LETIMER0->IF;
	LETIMER0->IFC = flagInt;						// Clearing Interrupts
	LETIMER_IntEnable(LETIMER0, (LETIMER_IEN_UF | LETIMER_IEN_COMP1)); //Enabling UF and COMP1 Interrupts
	blockSleepMode(SELECTED_EM);					// Block the sleep mode to a particular mode
	NVIC_EnableIRQ(LETIMER0_IRQn);					// Enabling the NVIC
}

/*****************************************************************************
 * LETIMER Interrupt Handler
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{
	static int periodCount = 0;
	static int stablePeriod = 0;
	static int currThresh = 0;
	int flagInt;
	flagInt = LETIMER0->IF;
	LETIMER0->IFC = flagInt;		//Clear the interrupts
	if((flagInt & LETIMER_IF_UF) != 0)
	{
		ADC_Setup();					//Call to setup ADC
		if(DMA_NEEDED)
		{
			DMA_ActivateBasic(DMA_CHANNEL_ADC, true, false, (void *)ramBufferAdcData, (void *)&(ADC0->SINGLEDATA), ADC_TEMP_CONVERSIONS - 1);		//Start DMA Transfer
		}
		blockSleepMode(ENERGYMODE1);			//Minimum mode required for ADC and DMA
		ADC_Start(ADC0,adcStartSingle);			// Start ADC
		if(startTrackingSleep == true)			//Setting Threshold automatically and monitoring sleep
		{
			if(currThresh != ((ACMP0->INPUTSEL & ACMP0_INPUTSEL_VREFMASK) >> 8))
			{
				currThresh = (ACMP0->INPUTSEL & ACMP0_INPUTSEL_VREFMASK) >> 8;
			}
			else
			{
				startTrackingSleep = false;
				monitorSleep = true;
				LESENSE->CH[LESENSE_CHANNEL6].INTERACT &= ~LESENSE_CH_INTERACT_SETIF_NEGEDGE;	//Sets the posedge Interrupt
				LESENSE->CH[LESENSE_CHANNEL6].INTERACT |= LESENSE_CH_INTERACT_SETIF_POSEDGE;	//Sets the posedge Interrupt
			}
		}
		if(monitorWashingMachine)
		{
			if(periodCount == 1)			//Load power management in Period 1
			{
				loadPowerManagement(true);
				vibrationDetected = false;
				GPIO_PinOutSet(LEDPORT,CAPACITIVE_INDICATOR);
			}
			else if(periodCount == 2)		//Load management in Period 3
			{
				loadPowerManagement(false);
				periodCount = 0;
				GPIO_PinOutClear(LEDPORT,CAPACITIVE_INDICATOR);
				if(vibrationDetected == false)
				{
					stablePeriod++;
					if(stablePeriod == 5)
					{
						GPIO_PinOutClear(LEDPORT,LIGHT_INDICATOR);
						monitorWashingMachine = false;
						vibrationDetected = false;
						stablePeriod = 0;
						LEUART_TransmitData(0.0,true);
					}
				}
				else
				{
					stablePeriod = 0;
					vibrationDetected = false;
				}
			}
		}
		else
		{
			GPIO_PinOutClear(LEDPORT,CAPACITIVE_INDICATOR);
			loadPowerManagement(false);
			periodCount = 0;
		}
	}
	else if ((flagInt & LETIMER_IF_COMP1) != 0)
	{
			periodCount++;
			if(monitorWashingMachine && (periodCount == 1))
			{
				GPIO_PinOutSet(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, ACC_VDD_PIN);
			}
	}
	else
		;
}

