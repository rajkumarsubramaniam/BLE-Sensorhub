/**************************************************************************************
 * @file LESENSE_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the LESENSE operations
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * LESENSE Configurations
 *****************************************************************************/
void LESENSE_Setup()
{
	static const LESENSE_Init_TypeDef initLesense =
	  {
	    .coreCtrl         =
	    {
	      .scanStart    = lesenseScanStartPeriodic,
	      .prsSel       = lesensePRSCh0,			/*Based on Scan mode(Periodic,one shot,PRS), will not affect if PRS is not selected*/
	      .scanConfSel  = lesenseScanConfDirMap,	/* Maps the respective channel configurations*/
	      .invACMP0     = false,
	      .invACMP1     = false,
	      .dualSample   = false,					/*Set to true to sample both ACMPs simultaneously.*/
	      .storeScanRes = false,
	      .bufOverWr    = true,
	      .bufTrigLevel = lesenseBufTrigHalf,
	      .wakeupOnDMA  = lesenseDMAWakeUpDisable,
	      .biasMode     = lesenseBiasModeDutyCycle,
	      .debugRun     = false
	    },

	    .timeCtrl         =
	    {
	      .startDelay     = 0x0		/*Delay sensor interaction on each channel 2-bit value- LFACLK cycles*/
	    },

	    .perCtrl          =
	    {
	      .dacCh0Data     = lesenseDACIfData,
	      .dacCh0ConvMode = lesenseDACConvModeDisable,
	      .dacCh0OutMode  = lesenseDACOutModeDisable,
	      .dacCh1Data     = lesenseDACIfData,
	      .dacCh1ConvMode = lesenseDACConvModeDisable,
	      .dacCh1OutMode  = lesenseDACOutModeDisable,
	      .dacPresc       = 0,
	      .dacRef         = lesenseDACRefBandGap,
	      .acmp0Mode      = lesenseACMPModeMux,   // only acmp mux not controlled by lesense
	      .acmp1Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
	      .warmupMode     = lesenseWarmupModeNormal
	    },

	    .decCtrl          =
	    {
	      .decInput  = lesenseDecInputSensorSt,
	      .initState = 0,
	      .chkState  = false,
	      .intMap    = false,
	      .hystPRS0  = false,
	      .hystPRS1  = false,
	      .hystPRS2  = false,
	      .hystIRQ   = false,
	      .prsCount  = false,
	      .prsChSel0 = lesensePRSCh0,
	      .prsChSel1 = lesensePRSCh1,
	      .prsChSel2 = lesensePRSCh2,
	      .prsChSel3 = lesensePRSCh3
	    }
	  };
	  /* Channel configuration */
	  static const LESENSE_ChDesc_TypeDef initLesenseLightCh =
	  {
	    .enaScanCh     = false,
	    .enaPin        = false,
	    .enaInt        = true,					//LESENSE->IEN
	    .chPinExMode   = lesenseChPinExHigh,
	    .chPinIdleMode = lesenseChPinIdleDis,
	    .useAltEx      = true,
	    .shiftRes      = false,
	    .invRes        = false,
	    .storeCntRes   = true,
	    .exClk         = lesenseClkLF,
	    .sampleClk     = lesenseClkLF,
	    .exTime        = 0x01,
	    .sampleDelay   = 0x01,
	    .measDelay     = 0x0,
	    .acmpThres     = 0x0,                   // don't care, configured by ACMPInit
	    .sampleMode    = lesenseSampleModeACMP,
	    .intMode       = lesenseSetIntLevel,
	    .cntThres      = 0X00,                   // Configured later by calibration function
	    .compMode      = lesenseCompModeGreaterOrEq	//Makes no sense when sampling ACMP output
	  };
	  static const LESENSE_ChDesc_TypeDef initLesenseCh =
	  {
		.enaScanCh     = true,					//Change to true when Light is working
		.enaPin        = true,
		.enaInt        = true,					//LESENSE->IEN
		.chPinExMode   = lesenseChPinExDis,
		.chPinIdleMode = lesenseChPinIdleDis,
		.useAltEx      = false,
		.shiftRes      = false,
		.invRes        = false,
		.storeCntRes   = true,
		.exClk         = lesenseClkLF,
		.sampleClk     = lesenseClkLF,
		.exTime        = 0x0,
		.sampleDelay   = SAMPLE_DELAY,
		.measDelay     = 0x0,
		.acmpThres     = 0x0,                   // don't care, configured by ACMPInit
		.sampleMode    = lesenseSampleModeCounter,
		.intMode       = lesenseSetIntLevel,
		.cntThres      = TOUCH_COUNT_THRESHOLD,                   // Configured later by calibration function
		.compMode      = lesenseCompModeLess
	  };
	static const LESENSE_ConfAltEx_TypeDef initAltEx =
	{
		.altExMap = lesenseAltExMapALTEX,
		.AltEx[0] =
		{
			.enablePin = true,
			.idleConf  = lesenseAltExPinIdleDis,
			.alwaysEx  = true
		},
		.AltEx[1] = LESENSE_ALTEX_DIS_CH,
		.AltEx[2] = LESENSE_ALTEX_DIS_CH,
		.AltEx[3] = LESENSE_ALTEX_DIS_CH,
		.AltEx[4] = LESENSE_ALTEX_DIS_CH,
		.AltEx[5] = LESENSE_ALTEX_DIS_CH,
		.AltEx[6] = LESENSE_ALTEX_DIS_CH,
		.AltEx[7] = LESENSE_ALTEX_DIS_CH
	};

	  /* Initialize LESENSE interface _with_ RESET. */
	  LESENSE_Init(&initLesense, true);

	  /* Configure channels */
	  LESENSE_ChannelConfig(&initLesenseCh, LESENSE_CHANNEL8);
	  LESENSE_ChannelConfig(&initLesenseCh, LESENSE_CHANNEL9);
	  LESENSE_ChannelConfig(&initLesenseCh, LESENSE_CHANNEL10);
	  LESENSE_ChannelConfig(&initLesenseCh, LESENSE_CHANNEL11);
	  LESENSE_ChannelConfig(&initLesenseLightCh, LESENSE_CHANNEL6);

	  LESENSE_AltExConfig(&initAltEx);
	  /* Set scan frequency */
	  LESENSE_ScanFreqSet(0, LESENSE_SCAN_FREQUENCY);
	  /* Set clock divisor for LF clock. */
	  LESENSE_ClkDivSet(lesenseClkLF, LESENSE_CLK_PRESCALE);

	  blockSleepMode(ENERGYMODE2);
	  INT_Disable();
	  /* Enable interrupt in NVIC. */
	  NVIC_EnableIRQ(LESENSE_IRQn);

	  /* Start scan. */
	 // LESENSE_ScanStart();

	  int i;
	  /* it settles on values after potential startup transients */
	   for(i = 0; i < NUMBER_OF_CALIBRATION_VALUES * 10; i++)
	   {
	     LESENSE_ScanStart();
	     TouchSlider_Calib();
	   }
	   INT_Enable();
}

/**************************************************************************//**
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler( void )
{
	int presentVDDLevel = 0;
	INT_Disable();
	int intFlagset = LESENSE->IF;
	LESENSE->IFC = intFlagset;
	//for(enabled_channels = LESENSE_CHANNEL8;enabled_channels <= LESENSE_CHANNEL11; enabled_channels++ )
	if((intFlagset >> LESENSE_CHANNEL8) & 0x1)
	{
		if((LESENSE->CH[LESENSE_CHANNEL8].EVAL & LESENSE_CH_EVAL_COMP_GE) == 0)
		{
			LESENSE->CH[LESENSE_CHANNEL8].EVAL |= LESENSE_CH_EVAL_COMP_GE;
		}
		else
		{
			LESENSE->CH[LESENSE_CHANNEL8].EVAL &= ~LESENSE_CH_EVAL_COMP_GE;
			swipe_check[0] = true;
			//GPIO_PinOutToggle(LEDPORT,CAPACITIVE_INDICATOR);
		}
	}
	if((intFlagset >> LESENSE_CHANNEL9) & 0x1)
	{
		if((LESENSE->CH[LESENSE_CHANNEL9].EVAL & LESENSE_CH_EVAL_COMP_GE) == 0)
		{
			LESENSE->CH[LESENSE_CHANNEL9].EVAL |= LESENSE_CH_EVAL_COMP_GE;
		}
		else
		{
			LESENSE->CH[LESENSE_CHANNEL9].EVAL &= ~LESENSE_CH_EVAL_COMP_GE;
			if(swipe_check[0] == true)
			{
				swipe_check[1] = true;
			}
			else
			{
				swipe_check[0] = false;
				swipe_check[1] = false;
				swipe_check[2] = false;
				swipe_check[3] = false;
			}
			//GPIO_PinOutToggle(LEDPORT,CAPACITIVE_INDICATOR);
		}
	}
	if((intFlagset >> LESENSE_CHANNEL10) & 0x1)
	{
		if((LESENSE->CH[LESENSE_CHANNEL10].EVAL & LESENSE_CH_EVAL_COMP_GE) == 0)
		{
			LESENSE->CH[LESENSE_CHANNEL10].EVAL |= LESENSE_CH_EVAL_COMP_GE;
		}
		else
		{
			LESENSE->CH[LESENSE_CHANNEL10].EVAL &= ~LESENSE_CH_EVAL_COMP_GE;
			if((swipe_check[0] == true) && (swipe_check[1] == true))
			{
				swipe_check[2] = true;
			}
			else
			{
				swipe_check[0] = false;
				swipe_check[1] = false;
				swipe_check[2] = false;
				swipe_check[3] = false;
			}
			//GPIO_PinOutToggle(LEDPORT,CAPACITIVE_INDICATOR);
		}
	}
	if((intFlagset >> LESENSE_CHANNEL11) & 0x1)
	{
		if((LESENSE->CH[LESENSE_CHANNEL11].EVAL & LESENSE_CH_EVAL_COMP_GE) == 0)
		{
			LESENSE->CH[LESENSE_CHANNEL11].EVAL |= LESENSE_CH_EVAL_COMP_GE;
		}
		else
		{
			LESENSE->CH[LESENSE_CHANNEL11].EVAL &= ~LESENSE_CH_EVAL_COMP_GE;
			if((swipe_check[0] == true) && (swipe_check[1] == true) && (swipe_check[2] = true))
			{
				swipe_check[3] = true;
				//GPIO_PinOutToggle(LEDPORT,CAPACITIVE_INDICATOR);
			}
			else
			{
				swipe_check[0] = false;
				swipe_check[1] = false;
				swipe_check[2] = false;
				swipe_check[3] = false;
			}
		}
	}
	if((intFlagset >> LESENSE_CHANNEL6) & 0x1)
	{
		if(startTrackingSleep == true)
		{
			presentVDDLevel = (ACMP0->INPUTSEL & ACMP0_INPUTSEL_VREFMASK) >> 8;
			presentVDDLevel = presentVDDLevel + 2;
			ACMP0->INPUTSEL &= ~ACMP0_INPUTSEL_VREFMASK;
			ACMP0->INPUTSEL |= presentVDDLevel << 8;
		}
		else if (monitorSleep == true)
		{
			/*Light detected during sleep monitoring waking up*/
			LEUART_TransmitData(1.0,true);
			monitorSleep = false;
			LESENSE->CHEN &= ~(1 << LESENSE_CHANNEL6);
		}
	}
	if( swipe_check[0] && swipe_check[1] && swipe_check[2] && swipe_check[3])
	{

		monitorWashingMachine = !monitorWashingMachine;
		if(monitorWashingMachine)
		{
			GPIO_PinOutSet(LEDPORT,CAPACITIVE_INDICATOR);
			GPIO_PinOutSet(LEDPORT,LIGHT_INDICATOR);
		}
		else
		{
			GPIO_PinOutClear(LEDPORT,CAPACITIVE_INDICATOR);
			GPIO_PinOutClear(LEDPORT,LIGHT_INDICATOR);
		}
		swipe_check[0] = false;
		swipe_check[1] = false;
		swipe_check[2] = false;
		swipe_check[3] = false;
	}
	INT_Enable();
}

/*****************************************************************************
 * Touch Slider threshold value calibration routine
 *****************************************************************************/
void TouchSlider_Calib()
{
	uint8_t first_ptr,j;
	LESENSE_CHANNELS i;
	static uint16_t calibration_value_index = 0;
	int count_Thresh = 0xFFFF;
	while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

	  /* Get position for first channel data in count buffer from lesense write pointer */
	first_ptr= ((LESENSE->PTR & _LESENSE_PTR_WR_MASK) >> _LESENSE_PTR_WR_SHIFT);

	/* Handle circular buffer wraparound */
	if(first_ptr >= TOTAL_CHANNELS_ENABLED)
	{
	  first_ptr = first_ptr - TOTAL_CHANNELS_ENABLED;
	}
	else
	{
	  first_ptr = first_ptr - TOTAL_CHANNELS_ENABLED + LESENSE_CHANNEL15 + 1;
	}
	for(i = LESENSE_CHANNEL8; i <= LESENSE_CHANNEL11; i++)
	{
		touch_calib_value[i-LESENSE_CHANNEL8][calibration_value_index] = LESENSE_ScanResultDataBufferGet(first_ptr++);
	}

	/* Wrap around calibration_values_index */
	calibration_value_index++;
	if(calibration_value_index >= NUMBER_OF_CALIBRATION_VALUES)
	{
	  calibration_value_index = 0;
	}

	/* Calculate max/min-value for each channel and set threshold */
	for(i = 0; i < TOTAL_CHANNELS_ENABLED; i++)
	{
		count_Thresh = 0xFFFF;
		for(j=0; j<NUMBER_OF_CALIBRATION_VALUES; j++)
		{
			if(touch_calib_value[i][j] < count_Thresh)		//Finds the minimum value and loads it
			{
				count_Thresh = touch_calib_value[i][j];
			}
		}
		LESENSE_ChannelThresSet(i+LESENSE_CHANNEL8, 0x0,(uint16_t) count_Thresh - (count_Thresh /100));
	}
}
