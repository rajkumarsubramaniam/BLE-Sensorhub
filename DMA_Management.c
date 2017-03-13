/**************************************************************************************
 * @file DMA_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the DMA - ADC, LEUART Rx and TX functions
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * DMA configurations
 *****************************************************************************/
void DMA_Setup()
{
		DMA_Init_TypeDef initDMA;
		DMA_CfgChannel_TypeDef channelConfigADC;
		DMA_CfgDescr_TypeDef desConfigADC;
		DMA_CfgChannel_TypeDef channelConfigLEUART_TX;
		DMA_CfgDescr_TypeDef desConfigLEUART_TX;
		DMA_CfgChannel_TypeDef channelConfigLEUART_RX;
		DMA_CfgDescr_TypeDef desConfigLEUART_RX;

		initDMA.controlBlock = dmaControlBlock;
		initDMA.hprot = 0;
		DMA_Init(&initDMA);				//Initialize DMA
		/*DMA setup for ADC*/
		cb[DMA_CHANNEL_ADC].userPtr = NULL;
		cb[DMA_CHANNEL_ADC].cbFunc = intHandlerADC_DMA;	//User call back function
		channelConfigADC.enableInt = true;
		channelConfigADC.highPri = true;
		channelConfigADC.select = DMAREQ_ADC0_SINGLE;	//Setting ADC channel
		channelConfigADC.cb = &cb[DMA_CHANNEL_ADC];

		DMA_CfgChannel(DMA_CHANNEL_ADC,&channelConfigADC);	//Initialize channel

		/* Setting up channel descriptor */
		desConfigADC.dstInc  = ADC_DMA_DST_DATA_INC;
		desConfigADC.srcInc  = ADC_DMA_SRC_DATA_INC;
		desConfigADC.size    = ADC_DMA_DATA_SIZE;
		desConfigADC.arbRate = ADC_DMA_ARBIT_RATE;		//Have to be checked
		desConfigADC.hprot   = 0;
		DMA_CfgDescr(DMA_CHANNEL_ADC, true, &desConfigADC);	//Setting the increments

		/*DMA setup for LEUART Transmission*/
		cb[DMA_CHANNEL_LEUART_TX].userPtr = NULL;
		cb[DMA_CHANNEL_LEUART_TX].cbFunc = intHandlerLEUART_TX_DMA;		//User call back function
		channelConfigLEUART_TX.enableInt = true;
		channelConfigLEUART_TX.highPri = true;
		channelConfigLEUART_TX.select = DMAREQ_LEUART0_TXBL;	//Setting ADC channel
		channelConfigLEUART_TX.cb = &cb[DMA_CHANNEL_LEUART_TX];

		DMA_CfgChannel(DMA_CHANNEL_LEUART_TX,&channelConfigLEUART_TX);	//Initialize channel

		/* Setting up channel descriptor */
		desConfigLEUART_TX.dstInc  = LEUART_TX_DMA_DST_DATA_INC;
		desConfigLEUART_TX.srcInc  = LEUART_TX_DMA_SRC_DATA_INC;
		desConfigLEUART_TX.size    = LEUART_TX_DMA_DATA_SIZE;
		desConfigLEUART_TX.arbRate = LEUART_TX_DMA_ARBIT_RATE;
		desConfigLEUART_TX.hprot   = 0;
		DMA_CfgDescr(DMA_CHANNEL_LEUART_TX, true, &desConfigLEUART_TX);	//Setting the increments

		/*DMA setup for LEUART Reception*/
		cb[DMA_CHANNEL_LEUART_RX].userPtr = NULL;
		cb[DMA_CHANNEL_LEUART_RX].cbFunc = intHandlerLEUART_RX_DMA;		//User call back function
		channelConfigLEUART_RX.enableInt = true;
		channelConfigLEUART_RX.highPri = true;
		channelConfigLEUART_RX.select = DMAREQ_LEUART0_RXDATAV;	//Setting ADC channel
		channelConfigLEUART_RX.cb = &cb[DMA_CHANNEL_LEUART_RX];

		DMA_CfgChannel(DMA_CHANNEL_LEUART_RX,&channelConfigLEUART_RX);	//Initialize channel

		/* Setting up channel descriptor */
		desConfigLEUART_RX.dstInc  = LEUART_RX_DMA_DST_DATA_INC;
		desConfigLEUART_RX.srcInc  = LEUART_RX_DMA_SRC_DATA_INC;
		desConfigLEUART_RX.size    = LEUART_RX_DMA_DATA_SIZE;
		desConfigLEUART_RX.arbRate = LEUART_RX_DMA_ARBIT_RATE;
		desConfigLEUART_RX.hprot   = 0;
		DMA_CfgDescr(DMA_CHANNEL_LEUART_RX, true, &desConfigLEUART_RX);	//Setting the increments


		/* Starting the transfer. Using Basic Mode */
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX,                /* Activate channel selected */
		                    true,                       /* Use primary descriptor */
		                    false,                      /* No DMA burst */
		                    (void *) &leuartRxData,            /* Destination address */
		                    (void *) &LEUART0->RXDATA,  /* Source address*/
		                    1 - 1);               /* Size of buffer minus1 */

		/*Wake Up DMA on RXDATA*/
		LEUART0->CTRL |= LEUART_CTRL_RXDMAWU;

		DMA->IFC = DMA->IF;
		DMA->IEN = DMA_IEN_CH0DONE | DMA_IEN_CH1DONE | DMA_IEN_CH2DONE |DMA_IEN_ERR;
		NVIC_EnableIRQ(DMA_IRQn);
}

/*****************************************************************************
 * LEUART TX -DMA Interrupt Handler
******************************************************************************/
void intHandlerLEUART_TX_DMA(unsigned int channel, bool primary, void *user)
{
	int flagClear = DMA->IF;
	INT_Disable();
	DMA->IFC = flagClear;
	LEUART0->CTRL &= ~LEUART_CTRL_TXDMAWU;
	LEUART0->IEN |= LEUART_IEN_TXBL;
	if(CIR_BUFFER_NEEDED)
	{
		transmittingLEUART = false;
		strncpy(cirBuffer[cirBuff_Read_Ptr],"", LEUART_DATA_SIZE);			//Empty the buffer
		cirBuff_Read_Ptr = (cirBuff_Read_Ptr + 1) & LEUART_CIRBUFF_INC_MASK;
		if(cirBuff_Write_Ptr != cirBuff_Read_Ptr)
		{
			sendDataCirBuffer();
		}
		else
		{
			;
		}
	}
	unblockSleepMode(ENERGYMODE2);
	INT_Enable();
}


/*****************************************************************************
 * ADC-DMA Interrupt Handler
 *****************************************************************************/
void intHandlerADC_DMA(unsigned int channel, bool primary, void *user)
{
	int i;
	float avgTemp = 0;
	int sumTemp = 0;
	INT_Disable();
	DMA->IFC = DMA ->IF ;
	ADC_Reset(ADC0);
	unblockSleepMode(ENERGYMODE1);
	for(i = 0; i < ADC_TEMP_CONVERSIONS; i++)		//Calculate average of Temp samples
	{
		sumTemp += ramBufferAdcData[i];
	}
	avgTemp = convertToCelsius(sumTemp / ADC_TEMP_CONVERSIONS);
	LEUART_TransmitData(avgTemp,false);
	if(CIR_BUFFER_TEST)
	{
		LEUART_TransmitData(avgTemp,false);
		LEUART_TransmitData(avgTemp,false);
		//LEUART_TransmitData(0.0,true);
		//LEUART_TransmitData(1.0,true);
	}
	INT_Enable();
}

/*****************************************************************************
 * LEUART RX -DMA Interrupt Handler
******************************************************************************/
void intHandlerLEUART_RX_DMA(unsigned int channel, bool primary, void *user)
{
	int flagClear = DMA->IF;
	INT_Disable();
	DMA->IFC = flagClear;
	if(leuartRxData == 1 && (!startTrackingSleep && !monitorSleep))
	{
		startTrackingSleep = true;
		monitorSleep = false;
		LESENSE->CHEN |= 1 << LESENSE_CHANNEL6;
	}
	DMA_ActivateBasic(DMA_CHANNEL_LEUART_RX,                /* Activate channel selected */
			                    true,                       /* Use primary descriptor */
			                    false,                      /* No DMA burst */
			                    (void *) &leuartRxData,            /* Destination address */
			                    (void *) &(LEUART0->RXDATA),  /* Source address*/
			                    1 - 1);               /* Size of buffer minus1 */

	INT_Enable();
}
