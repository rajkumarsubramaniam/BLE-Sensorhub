/**************************************************************************************
 * @file ADC_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the ADC functionalities
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * ADC configurations
 *****************************************************************************/
void ADC_Setup()
{
	ADC_Init_TypeDef initADC;
	initADC.timebase = ADC_TimebaseCalc(0);							//Set time base based on HFPER
	initADC.prescale = ADC_PrescaleCalc(ADC_CLK_FREQUENCY,0);		//Setting ADC pre-sclae to 130KHz, so that 10,000 conversions is achieved with Ta = 1
	initADC.warmUpMode = ADC_WARMUP_MODE;
	initADC.tailgate = false;
	ADC_Init(ADC0, &initADC);

	ADC_InitSingle_TypeDef initADC_Single;
	initADC_Single.acqTime = ADC_ACQ_TIME;		// Using  3us * ADCFreq gives 0.39 cycles for acquisition,so setting it to 1
	initADC_Single.diff = false;
	initADC_Single.input = ADC_INPUT;	// Select the Temp Sensor as input
	initADC_Single.resolution = ADC_RESOLUTION;
	initADC_Single.rep = ADC_REPEAT;
	initADC_Single.prsEnable = false;
	initADC_Single.leftAdjust = false;
	ADC_InitSingle(ADC0, &initADC_Single);

	if(!DMA_NEEDED)	//Using Interrupt handler when not using DMA
	{
		ADC0->IFC = ADC_IFC_SINGLE;
		ADC0->IEN =  ADC_IEN_SINGLE;
		NVIC_EnableIRQ(ADC0_IRQn);
	}
}

/*****************************************************************************
 * Convert temperature readings to Celsius
 *****************************************************************************/
float convertToCelsius(int32_t adcSample )
{
	 float temp;
	 float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)>>_DEVINFO_CAL_TEMP_SHIFT);
	 float cal_value_0 = (float)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)>>_DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
	 float t_grad = TEMP_GRADIANT;
	 temp = (cal_temp_0 - ((cal_value_0 - adcSample)/t_grad));
	 return temp;
}

