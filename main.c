/**************************************************************************//**
 * @file main.c
 * @brief Course Project - Smart Home System - Convenient Peripheral
 * @author Raj Kumar Subramaniam
 * @version 0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 ******************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * Global Variable Declarations
 *****************************************************************************/
int sleepMaintainer[5] = {0,0,0,0,0}; 		//Global variable for Maintaining sleep modes

bool prev_touch = false;

bool swipe_check[TOTAL_CHANNELS_ENABLED] = {false, false, false, false};

unsigned int cirBuff_Write_Ptr = 0;				//Write Pointer for Circular buffer

unsigned int cirBuff_Read_Ptr = 0;				//Read Pointer for Circular buffer

char uartData[6] = {'0','0','0','0','0','0'};	//Global variable to send data through UART

int uartCount = 0;

bool transmittingLEUART = false;

float oscRatio = 1;

bool monitorWashingMachine = false;

bool monitorSleep = false;

bool startTrackingSleep = false;

bool vibrationDetected = false;

/*****************************************************************************
 * Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  blockSleepMode(ENERGYMODE3);

  CMU_Setup();

  GPIO_Setup();

  I2C_Setup();

  LEUART_Setup();

  if(DMA_NEEDED || LEUART_DMA_NEEDED)
  {
	DMA_Setup();				//Call to setup DMA
  }

  ACMP0_Setup();

  ACMP1_Setup();

  LESENSE_Setup();

  LETIMER_Setup();				//Call to setup LETIMER0

  LETIMER_Enable(LETIMER0,true);	//Enable the LETIMER0

  /* Infinite loop */
  while(1)
  {
	  sleep();
  }
}
