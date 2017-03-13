/**************************************************************************************
 * @file LEUART_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the LEUART operations
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * LEUART configurations
 *****************************************************************************/
void LEUART_Setup()
{
	LEUART_Reset(LEUART0);

	LEUART_Init_TypeDef leuart0Init;
	leuart0Init.baudrate = LEUART_BAUD_RATE;    // Baud rate = 9600
	leuart0Init.databits = LEURT_BITS;     		// Each LEUART frame contains 8 data bits
	leuart0Init.enable   = leuartDisable;       // Activate data reception on LEUn_TX pin.
	leuart0Init.parity   = LEUART_PARITY;      // No parity bits in use
	leuart0Init.refFreq  = 0;                   // Inherit the clock frequency from the LEUART clock source
	leuart0Init.stopbits = LEUART_STOP_BITS;    // Setting the number of stop bits in a frame to 1 bit period
	LEUART_Init(LEUART0, &leuart0Init);			//Initializing the LEUART

	LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC0; //Selecting the Pin PD4 as TX pin
	//LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
	LEUART0->IFC = LEUART0->IF;
	if(LEUART_DMA_NEEDED)
	{
		LEUART0->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN;;				//Enable TX
		NVIC_EnableIRQ(LEUART0_IRQn);
	}
}

/*****************************************************************************
 * LEUART Interrupt Handler
 *****************************************************************************/
void LEUART0_IRQHandler(void)
{
	int flagClear = LEUART0->IF;
	INT_Disable();
	LEUART0->IFC = flagClear;
	if(flagClear & LEUART_IF_TXBL)
	{
		LEUART0->IEN &= ~LEUART_IEN_TXBL;
		LEUART0->CMD = LEUART_CMD_CLEARTX;
	}
	INT_Enable();
}

/*****************************************************************************
 * LEUART Transmission of data
 *****************************************************************************/
void LEUART_TransmitData(float data, bool light)
{
	blockSleepMode(ENERGYMODE2);
	if(light)
	{										//Send Light data
		uartData[0] = 'L';
		uartData[1] = (char)data;
		uartData[2] = '0';
		uartData[3] = '0';
		uartData[4] = '0';
		uartData[5] = '0';
	}
	else
	{
		snprintf(uartData,7,"%c%4.2f",'T',data);	//Send Temperature data
	}
	if(CIR_BUFFER_NEEDED)
	{
		strncpy(cirBuffer[cirBuff_Write_Ptr], "", LEUART_DATA_SIZE);//Empty the buffer
		strncpy(cirBuffer[cirBuff_Write_Ptr], uartData, LEUART_DATA_SIZE);	//Copy data to the buffer
		cirBuff_Write_Ptr = (cirBuff_Write_Ptr + 1) & LEUART_CIRBUFF_INC_MASK;
		sendDataCirBuffer();
	}
	else
	{
		/* Enable DMA wake-up from LEUART0 TX */
		LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;
		/* Starting the transfer. Using Basic Mode */
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX,    /* Activate channel selected */
						true,                       /* Use primary descriptor */
						false,                      /* No DMA burst */
						(void *)&(LEUART0->TXDATA), /* Keep destination address */
						(void *)uartData,           /* Keep source address*/
						LEUART_DATA_SIZE-1);        /* Size of buffer minus1*/
	}
}

/*****************************************************************************
 * LEUART loading data ainto circular buffer
 *****************************************************************************/
void sendDataCirBuffer()
{
	if(!transmittingLEUART)
	{
		transmittingLEUART = true;
		/* Enable DMA wake-up from LEUART0 TX */
		LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;
		DMA_ActivateBasic(DMA_CHANNEL_LEUART_TX,    							/* Activate channel selected */
									true,                      					/* Use primary descriptor */
									false,                      				/* No DMA burst */
									(void *)&(LEUART0->TXDATA), 				/* Keep destination address */
									(void *)&cirBuffer[cirBuff_Read_Ptr][0],   	/* Keep source address*/
									LEUART_DATA_SIZE-1);        				/* Size of buffer minus1*/
	}
}


