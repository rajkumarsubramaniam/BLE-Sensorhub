/**************************************************************************************
 * @file GPIO_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the GPIO operations and Pin configurations
 * @author Raj Kumar Subramaniam
 **************************************************************************************/
#include "Macros_Functions.h"

/*****************************************************************************
 * GPIO Configurations
 *****************************************************************************/
void GPIO_Setup()
{
	GPIO_DriveModeSet(LEDPORT, gpioDriveModeLowest);								//PE Drive Mode
	GPIO_DriveModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, gpioDriveModeStandard);			//PC Drive Mode
	GPIO_DriveModeSet(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, gpioDriveModeHigh );		//PD Drive Mode
	/*GPIO Mode for LEDs*/
	GPIO_PinModeSet(LEDPORT, CAPACITIVE_INDICATOR, gpioModePushPull, 0);			//PE2 - CAP_LED0
	GPIO_PinModeSet(LEDPORT, LIGHT_INDICATOR, gpioModePushPull, 0);					//PE3 - LIGHT_LED1
	/*Disable for Analog inputs for Touch Slider*/
	GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT,TOUCH_ACMP1_CH0,gpioModeDisabled,0);	//Pin mode for PC8
	GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT,TOUCH_ACMP1_CH1,gpioModeDisabled,0);	//Pin mode for PC9
	GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT,TOUCH_ACMP1_CH2,gpioModeDisabled,0);	//Pin mode for PC10
	GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT,TOUCH_ACMP1_CH3,gpioModeDisabled,0);	//Pin mode for PC11
	/*GPIO Mode for Ambient Light Sensor Pins*/
	GPIO_PinModeSet(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, AMB_SENSOR_EXITE_PIN, gpioModePushPull, 0);	//PD6 - AMB_EXITE
	GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT,AMB_SENSOR_SENSE_PIN,gpioModeDisabled,0);			//PC6 - AMB_SENSE
	/*I2C GPIO Modes*/
	GPIO_PinModeSet(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, ACC_VDD_PIN, gpioModePushPull, 0);	//PD0 - I2C VDD
	GPIO_PinModeSet(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, ACC_INT_PIN, gpioModeInput, 1);		//PD1 - I2C INT
	GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SCL_PIN, gpioModeDisabled, 0);			//PC5 - I2C SCL
	GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SDA_PIN, gpioModeDisabled, 0);			//PC4 - I2C SDA
	/*LEUART GPIO Setup*/
	GPIO_PinModeSet(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, LEUART_TX_PIN, gpioModePushPull, 1); //PD4 - LEAURT TX
	GPIO_PinModeSet(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, LEUART_RX_PIN, gpioModeInputPull, 1); //PD5 - LEAURT RX
}


/*****************************************************************************
 * GPIO ODD INT Handler Routine for Ambient light control
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	INT_Disable();
	GPIO->IFC = GPIO->IF;
	accelerometerRead(0x01);
	accelerometerRead(0x03);
	accelerometerRead(0x05);
	accelerometerRead(REG_FF_MT_SRC);
	vibrationDetected = true;
	INT_Enable();
}


