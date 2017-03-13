/**************************************************************************************
 * @file I2C_Management.c - Course Project - Smart Home System - Convenient Peripheral
 * @brief This manages the I2C functionalities
 * @author Raj Kumar Subramaniam
 **************************************************************************************/

#include "Macros_Functions.h"

/*****************************************************************************
 * I2C Initializations
 *****************************************************************************/

void I2C_Setup()
{
	int i;
	I2C_Init_TypeDef i2C_init;
	i2C_init.clhr 		= i2cClockHLRStandard;
	i2C_init.enable	 	=  true;
	i2C_init.freq 		= I2C_FREQ_STANDARD_MAX;			//Done with the FAST mode after discussion with Professor
	i2C_init.master 	= true;
	i2C_init.refFreq	= 0;
	/* In some situations (after a reset during an I2C transfer), the slave */
	/* device may be left in an unknown state. Send 9 clock pulses just in case. */
	/*Copyrights of Silicon Labs - Example code*/
	for (i = 0; i < 9; i++)
	{
		/* TBD: Seems to be clocking at approximately 80kHz-120kHz depending on compiler
		 * optimization when running at 14MHz. A bit high for standard mode devices,
		 * but DVK only has fast mode devices. Need however to add some time
		 * measurement in order to not be dependable on frequency and code executed.*/
		GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SCL_PIN, gpioModeWiredAnd, 0);
		GPIO_PinModeSet(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, ACC_SCL_PIN, gpioModeWiredAnd, 1);
	}
	I2C1->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | (_I2C_ROUTE_LOCATION_LOC0  << _I2C_ROUTE_LOCATION_SHIFT); //Setting up the location

	I2C_Init(I2C1, &i2C_init);

	if(I2C1->STATE & I2C_STATE_BUSY)
	{
		I2C1->CMD = I2C_CMD_ABORT;
	}

	I2C1->IFC = _I2C_IFC_MASK;
}

/*****************************************************************************
 * Accelerometer Setup as required
 *****************************************************************************/
void accelerometerSetup()
{
	 INT_Disable();
	 accelerometerRead(REG_DEVICE_ID);
	 accelerometerWrite(REG_XYZ_DATA_CONFIG,0x00);
	 accelerometerWrite(REG_HP_FILTER_CUTOFF,0x00);
	 accelerometerWrite(REG_CTRL_REG1,0x00);		//CTRL_REG1
	 accelerometerRead(REG_CTRL_REG1);
	 accelerometerWrite(REG_CTRL_REG2,0x00);		//CTRL_REG2
	 accelerometerWrite(REG_CTRL_REG3,0x00);		//CTRL_REG3
	 accelerometerWrite(REG_CTRL_REG4,0x04);		//CTRL_REG4
	 accelerometerRead(REG_CTRL_REG4);
	 accelerometerWrite(REG_CTRL_REG5,0x04);		//CTRL_REG5
	 accelerometerRead(REG_CTRL_REG5);
	 accelerometerWrite(REG_FF_MT_CFG,0xF8);		//FF_MT_CFG
	 accelerometerWrite(REG_FF_MT_THS,0x13);		//FF_MT_THS
	 accelerometerWrite(REG_FF_MT_COUNT,0x7F);		//FF_MT_CNT
	 accelerometerRead(REG_FF_MT_CFG);
	 accelerometerWrite(REG_CTRL_REG1,0x01);		//CTRL_REG1
	 INT_Enable();
}
/*****************************************************************************
 * Accelerometer write function through I2C
 *****************************************************************************/
void accelerometerWrite(int regCmd, int data)
{
	if(I2C1->STATE & I2C_STATE_BUSY)
	{
		while(I2C1->STATE & I2C_STATE_BUSY_DEFAULT);
	}
	I2C1->TXDATA = (ACC_SLAVE_ADDRESS << 1 ) |(I2C_WRITE_REQ);
	I2C1->CMD = I2C_CMD_START;
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IF_ACK;
	I2C1->TXDATA = regCmd;
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IF_ACK;
	I2C1->TXDATA = data;
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IF_ACK;
	I2C1->CMD = I2C_CMD_STOP;
	while(!(I2C1->IF & I2C_IF_MSTOP));
}

/*****************************************************************************
 * Accelerometer read function through I2C
 *****************************************************************************/
int accelerometerRead(int regAddr)
{	int data = 0;
	if(I2C1->STATE & I2C_STATE_BUSY)
	{
		while(I2C1->STATE & I2C_STATE_BUSY_DEFAULT);
	}
	I2C1->TXDATA = (ACC_SLAVE_ADDRESS << 1 ) |(I2C_WRITE_REQ);
	I2C1->CMD = I2C_CMD_START;
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IF_ACK;
	I2C1->TXDATA = regAddr;
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IF_ACK;
	I2C1->CMD = I2C_CMD_START;
	I2C1->TXDATA = (ACC_SLAVE_ADDRESS << 1 ) |(I2C_READ_REQ);
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IF_ACK;
	while((I2C1->IF & I2C_IF_RXDATAV) == 0);
	I2C1->CMD = I2C_CMD_NACK;
	while(I2C1->STATUS & I2C_STATUS_PNACK);
	I2C1->CMD = I2C_CMD_STOP;
	while(!(I2C1->IF & I2C_IF_MSTOP));
	I2C1->IFC = I2C_IF_NACK;
	data = I2C1->RXDATA;
	return data;
}

/*****************************************************************************
 * Load Power Management
 *****************************************************************************/
void loadPowerManagement(bool turnONSensor)
{
	if(turnONSensor)
	{
		int i;
		GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SCL_PIN, gpioModeWiredAnd, 1);			//PC5 - I2C SCL
		GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SDA_PIN, gpioModeWiredAnd, 1);			//PC4 - I2C SDA
		for (i = 0; i < 9; i++)
		{
			GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SCL_PIN, gpioModeWiredAnd, 0);		//PC5 - I2C SCL
			GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SCL_PIN, gpioModeWiredAnd, 1);		//PC5 - I2C SCL
		}
		accelerometerSetup();
		GPIO->IFC = GPIO->IF;		//INT enable
		GPIO_ExtIntConfig(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, ACC_INT_PIN,ACC_INT_PIN,false,true,true);
		NVIC_EnableIRQ(GPIO_ODD_IRQn);
	}
	else
	{
		GPIO_ExtIntConfig(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, ACC_INT_PIN,ACC_INT_PIN,false,true,false);
		NVIC_DisableIRQ(GPIO_ODD_IRQn);
		GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SCL_PIN, gpioModeDisabled, 1);			//PC5 - I2C SCL
		GPIO_PinModeSet(LIGHT_TOUCH_SENSE_I2C_PORT, ACC_SDA_PIN, gpioModeDisabled, 1);			//PC4 - I2C SDA
		GPIO_PinOutClear(AMB_SENSOR_EXITE_I2C_CONTROL_PORT, ACC_VDD_PIN);
	}

}
