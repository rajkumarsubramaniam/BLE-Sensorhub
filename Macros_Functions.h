/**************************************************************************************
 * @file Macros_Function.h - Course Project - Smart Home System - Convenient Peripheral
 * @brief This stores all the macros for the project
 * @author Raj Kumar Subramaniam
 **************************************************************************************/
/***************************************************************************
* The following  copyright applies to the functions
* blockSleepMode(),
* unblockSleepMode(),
* sleep(),
* convertToCelsius().
*
* These are taken from example of Silicon Labs
* @section License (C) Copyright 2014 Silicon Labs, http://www.silabs.com
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions
* 1. The origin of this software must not be mis-represented; you must not
*    claim that you wrote the original software.
* 2. Altered source versions must be plainly marked as such, and must not be
*    mis-represented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
* DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
* obligation to support this Software. Silicon Labs is providing the
* Software "AS IS", with no express or implied warranties of any kind,
* including, but not limited to, any implied warranties of merchantability
* or fitness for any particular purpose or warranties against infringement
* of any proprietary rights of a third party.
* Silicon Labs will not be liable for any consequential, incidental, or
* special damages, or any other relief, or for any claim by any third party,
* arising from your use of this Software.
***************************************************************************/

#ifndef MACROS_FUNCTIONS_H_
#define MACROS_FUNCTIONS_H_

#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_acmp.h"
#include "em_gpio.h"
#include "em_lesense.h"
#include "em_i2c.h"
#include "em_leuart.h"
#include "em_letimer.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "em_adc.h"
#include "bsp_trace.h"

/*This enum provides different Energy modes for better readability*/
typedef enum
{
	ENERGYMODE0 = 0,
	ENERGYMODE1 = 1,
	ENERGYMODE2 = 2,
	ENERGYMODE3 = 3,
	ENERGYMODE4 = 4
} EnergyModes;

typedef enum
{
	LESENSE_CHANNEL0 = 0,
	LESENSE_CHANNEL1,
	LESENSE_CHANNEL2,
	LESENSE_CHANNEL3,
	LESENSE_CHANNEL4,
	LESENSE_CHANNEL5,
	LESENSE_CHANNEL6,
	LESENSE_CHANNEL7,
	LESENSE_CHANNEL8,
	LESENSE_CHANNEL9,
	LESENSE_CHANNEL10,
	LESENSE_CHANNEL11,
	LESENSE_CHANNEL12,
	LESENSE_CHANNEL13,
	LESENSE_CHANNEL14,
	LESENSE_CHANNEL15
} LESENSE_CHANNELS;

#define LEDPORT gpioPortE 				//Macro for LED port

#define CAPACITIVE_INDICATOR 2			//Macro for LED0 PIN number - Washing Machine Indication

#define LIGHT_INDICATOR 3				//Macro for LED1 PIN number - Blinking LED to denote washing machine LED

#define LIGHT_TOUCH_SENSE_I2C_PORT gpioPortC

#define TOUCH_ACMP1_CH0 8

#define TOUCH_ACMP1_CH1 9

#define TOUCH_ACMP1_CH2 10

#define TOUCH_ACMP1_CH3 11

#define CHANNEL_DISABLED 0

#define CHANNEL_ENABLED 1

#define LESENSE_CHANNEL_ENABLE {\
	CHANNEL_DISABLED,\
	CHANNEL_DISABLED,\
	CHANNEL_DISABLED,\
	CHANNEL_DISABLED,\
	CHANNEL_DISABLED,\
	CHANNEL_DISABLED,\
	CHANNEL_ENABLED,\
	CHANNEL_DISABLED,\
	CHANNEL_ENABLED, \
	CHANNEL_ENABLED, \
	CHANNEL_ENABLED, \
	CHANNEL_ENABLED, \
	CHANNEL_DISABLED,\
	CHANNEL_DISABLED,\
	CHANNEL_DISABLED \
}

#define VDD_LEVEL_75 0x30

#define SAMPLE_DELAY 30

#define TOUCH_COUNT_THRESHOLD 0x100

#define LESENSE_SCAN_FREQUENCY 20

#define LESENSE_CLK_PRESCALE lesenseClkDiv_1

#define VALIDATE_CNT 10

#define NUMBER_OF_CALIBRATION_VALUES    10

#define CALIBRATION_INTERVAL            5

#define TOTAL_CHANNELS_ENABLED 4

#define AMB_SENSOR_EXITE_I2C_CONTROL_PORT gpioPortD 	//Macro for Sensor EXITE  port

#define AMB_SENSOR_EXITE_PIN 6				//Macro for Sensor EXITE PIN number

#define AMB_SENSOR_SENSE_PIN 6				//Macro for Sensor Sense PIN number

#define LESENSE_ALTEX_DIS_CH \
	{                                                                             \
		false,                  /* Alternate excitation enabled.*/                  \
		lesenseAltExPinIdleDis, /* Alternate excitation pin is disabled in idle. */ \
		false                   /* Excite only for corresponding channel. */        \
	}

#define ACMP_CHANNEL_INPUT acmpChannel6		//Macro for ACMP channel 6

#define ACMP_CHANNEL_REF acmpChannelVDD		//Macro for VDD reference

#define DARKNESS_REF 2					//This sets VDD for darkness

#define BRIGHTLIGHT_REF 61				//This sets the VDD for light

#define ACMP0_INPUTSEL_VREFMASK 0x3F00

/*I2C Macros*/

#define ACC_VDD_PIN 0

#define ACC_INT_PIN 1

#define ACC_SDA_PIN 4

#define ACC_SCL_PIN 5

#define I2C_WRITE_REQ 0

#define I2C_READ_REQ 1

#define ACC_SLAVE_ADDRESS 0x1D

#define REG_DEVICE_ID 0x0D

#define REG_XYZ_DATA_CONFIG 0x0E

#define REG_HP_FILTER_CUTOFF 0x0F

#define REG_FF_MT_CFG 0x15

#define REG_FF_MT_SRC 0x16

#define REG_FF_MT_THS 0x17

#define REG_FF_MT_COUNT 0x18

#define REG_CTRL_REG1 0x2A

#define REG_CTRL_REG2 0x2B

#define REG_CTRL_REG3 0x2C

#define REG_CTRL_REG4 0x2D

#define REG_CTRL_REG5 0x2E

#define REG_SYSMOD 0x0B

#define REG_INT_SOURCE 0x0C

/*Macros to control LEUART*/

#define LEUART_BAUD_RATE 9600

#define LEUART_TX_PIN 4

#define LEUART_RX_PIN 5

#define LEUART_PARITY leuartNoParity

#define LEURT_BITS leuartDatabits8

#define LEUART_STOP_BITS leuartStopbits1

#define LEUART_DATA_SIZE 6

#define LEUART_CIR_BUFF 64

#define LEUART_CIRBUFF_INC_MASK 0x3F

#define DMA_CHANNEL_LEUART_TX 1

#define LEUART_TX_DMA_DST_DATA_INC dmaDataIncNone	//DMA dst increment

#define LEUART_TX_DMA_SRC_DATA_INC dmaDataInc1		//DMA SRC increment

#define LEUART_TX_DMA_DATA_SIZE dmaDataSize1		//DMA data size

#define LEUART_TX_DMA_ARBIT_RATE dmaArbitrate1		//DMA arbit rate

#define DMA_CHANNEL_LEUART_RX 2

#define LEUART_RX_DMA_DST_DATA_INC dmaDataIncNone	//DMA dst increment

#define LEUART_RX_DMA_SRC_DATA_INC dmaDataIncNone	//DMA SRC increment

#define LEUART_RX_DMA_DATA_SIZE dmaDataSize1		//DMA data size

#define LEUART_RX_DMA_ARBIT_RATE dmaArbitrate1		//DMA arbit rate

#define DMA_NEEDED 1					//Macro to disable DMA for Assignment3

#define LEUART_DMA_NEEDED 1				//LEUART DMA Control

#define CIR_BUFFER_NEEDED 1				//Circular buffer needed

#define CIR_BUFFER_TEST 0				//Enable to set transmit buffer test data

/*Macros for ADC, DMA*/

#define ADC_INT_CLEAR_BITS 0x183			//Clearing ADC interrupts

#define DMA_INT_CLEAR_BITS 0x800007FF		//Clearing DMA interrupts

#define SHIFT_EIGHT_BITS 8					//Used to shift bits

#define DMA_CHANNELS 3						//Number of DMA channels used

#define DMA_CHANNEL_ADC 0					//DMA Channel for ADC

#define ADC_DMA_DST_DATA_INC dmaDataInc2	//DMA dst increment

#define ADC_DMA_SRC_DATA_INC dmaDataIncNone	//DMA SRC increment

#define ADC_DMA_DATA_SIZE dmaDataSize2		//DMA data size

#define ADC_DMA_ARBIT_RATE dmaArbitrate1	//DMA arbit rate

#define ADC_TEMP_CONVERSIONS 500			//Number of elements that needs to be transfered

#define TEMP_GRADIANT -6.27;				//Temperature Gradient for temperature conversion

#define MAX_COUNT 65535						//Maximum counter value of 16 bits

#define SHIFT_EIGHT_BITS 8					//Left shit 8 bits

#define CLEAR_LETPRES 0xfffff0ff			//Clear bits of LETIMER0 PRESCALAR

#define ADC_CLK_FREQUENCY 1300000			//ADC clock frequency

#define ADC_WARMUP_MODE adcWarmupKeepADCWarm

#define ADC_ACQ_TIME adcAcqTime1

#define ADC_INPUT adcSingleInputTemp

#define ADC_RESOLUTION adcRes12Bit

#define ADC_REPEAT true

/*Macros for LETIMER*/
#define SELECTED_EM ENERGYMODE2			//This is a macro to select different Energy modes. Use the enum.

#define LED_ON_TIME 0.004				//This is the ON-Duty cycle of the LED

#define LED_OFF_TIME 6.25				//This is the Period of the LETIMER

#define CALIBRATION_NEEDED 0			//Control to enable LFRCO calibration

#define HFRCOPERCLK_FREQ 14000000			//HFRCO Frequency at 14MHz

#define LFXO_FREQUENCY 32768				//LFXO operates at 32.678KHz

#define ULFRCO_FREQUENCY 1000				//ULFRCO operates at 1kHz


/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/
void blockSleepMode(EnergyModes);
void unblockSleepMode(EnergyModes);
void sleep();
void CMU_Setup();
void GPIO_Setup();
void LESENSE_Setup();
void ACMP1_Setup();
void TouchSlider_Calib();
void ACMP0_Setup();
void I2C_Setup();
void accelerometerSetup();
void accelerometerWrite(int, int);
int accelerometerRead(int);
void ADC_Setup();
void LEUART_Setup();
void DMA_Setup();
void intHandlerADC_DMA(unsigned int, bool, void *);
void sendDataCirBuffer();
void LEUART_TransmitData(float, bool);
void intHandlerLEUART_TX_DMA(unsigned int, bool, void *);
void intHandlerLEUART_RX_DMA(unsigned int, bool, void *);
float convertToCelsius(int32_t);
void LETIMER_Setup();
void loadPowerManagement(bool);


/*****************************************************************************
 * Global Variables
 *****************************************************************************/
extern int sleepMaintainer[5]; 		//Global variable for Maintaining sleep modes

int touch_calib_value[TOTAL_CHANNELS_ENABLED][NUMBER_OF_CALIBRATION_VALUES];

extern bool prev_touch;

extern bool swipe_check[TOTAL_CHANNELS_ENABLED];

extern char uartData[LEUART_DATA_SIZE];		//Data to be sent to BLE

extern int uartCount;						//GLobal counter

char cirBuffer[LEUART_CIR_BUFF][LEUART_DATA_SIZE];	//Circular buffer to transmit data

extern unsigned int cirBuff_Write_Ptr;				//Write Pointer for Circular buffer

extern unsigned int cirBuff_Read_Ptr;				//Read Pointer for Circular buffer

extern bool transmittingLEUART;

DMA_CB_TypeDef cb[DMA_CHANNELS];			//Call back declaration variable

volatile uint16_t ramBufferAdcData[ADC_TEMP_CONVERSIONS];	//Variable to store output of ADC in RAM

extern float oscRatio;

extern bool monitorWashingMachine;			//Flag to control monitoring of washing machine

extern bool monitorSleep;					//Flag to control Sleep tracking

int leuartRxData;

extern bool startTrackingSleep;

extern bool vibrationDetected;

#endif /* MACROS_FUNCTIONS_H_ */
