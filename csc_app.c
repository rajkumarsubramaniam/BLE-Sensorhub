/**
 * file:csc_app.c
 * author: Raj Kumar Subramaniam
 * brief: Smart Home System - Convenient Peripheral 
 * description : This is the code for the entire BLE module for the Convenient Peripheral
 * 				 This controls the sensor hub and acts as the BLE peripheral device.
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
 *Support</a>
 */

/*- Includes -----------------------------------------------------------------------*/
#include <asf.h>
#include "platform.h"
#include "console_serial.h"
#include "at_ble_api.h"
#include "ble_manager.h"
#include "timer_hw.h"
#include "dma_sam_b.h"
#include "button.h" 
#include "csc_app.h"
#include "cscp.h"
#include "cscs.h"
/* =========================== GLOBALS =============================== */

#define APP_STACK_SIZE  (1024)
#define BLOCK_EXAMPLE_CODE 0

#define DATA_LENGTH 6		//Maximum Data length of UART transfer

#define TOTAL_TRANSFERS 6	//DMA transfers

#define BAUD_RATE 9600		//Baud Rate for UART

/*Connection Updates*/
#define CONNECTION_INTERVAL_MIN 50

#define CONNECTION_INTERVAL_MAX 100

#define SLAVE_LATENCY 0

/*Variables for BLE connection and advertising*/
volatile at_ble_status_t status;

at_ble_handle_t htpt_conn_handle;

at_ble_handle_t conn_handle_update;

volatile bool Timer_Flag = false;

volatile bool Temp_Notification_Flag = false;

/*Variables for UART and DMA*/
struct uart_module uart_instance;

struct dma_resource uart_dma_resource_rx;
struct dma_resource uart_dma_resource_tx;

struct dma_descriptor example_descriptor_rx;
struct dma_descriptor example_descriptor_tx;

/*Temperature Related Variables*/
static char destination_temp[DATA_LENGTH];	//Destination variable

static int dataToLG;			//Data to be transmitted to LG

float tempValue;							//Converted Temperature value

bool control_LED = true;					//Variable to control LED

volatile unsigned char app_stack_patch[APP_STACK_SIZE];

/* Received notification data structure */
csc_report_ntf_t recv_ntf_info;

/* Data length to be send over the air */
uint16_t send_length = 0;

/* Buffer data to be send over the air */
uint8_t send_data[APP_TX_BUF_SIZE];

/*Strings controlling data to CSC App*/
bool sendWMChat =  false;

bool askCoffee =  false;

uint8_t machineStarted[] = "Washing Finished";

uint8_t coffeeOn[] = "Wanna switch on?";

static const ble_event_callback_t app_gap_handle[] = {
		NULL,// AT_BLE_UNDEFINED_EVENT
		NULL,// AT_BLE_SCAN_INFO
		NULL,// AT_BLE_SCAN_REPORT
		NULL,// AT_BLE_ADV_REPORT
		NULL,// AT_BLE_RAND_ADDR_CHANGED
		app_connected_event_handler,// AT_BLE_CONNECTED
		app_disconnected_event_handler,// AT_BLE_DISCONNECTED
		app_connection_param_update_done,//NULL,// AT_BLE_CONN_PARAM_UPDATE_DONE
		NULL,// AT_BLE_CONN_PARAM_UPDATE_REQUEST
		ble_paired_cb,// AT_BLE_PAIR_DONE
		NULL,// AT_BLE_PAIR_REQUEST
		NULL,// AT_BLE_SLAVE_SEC_REQUEST
		NULL,// AT_BLE_PAIR_KEY_REQUEST
		NULL,// AT_BLE_ENCRYPTION_REQUEST
		NULL,// AT_BLE_ENCRYPTION_STATUS_CHANGED
		NULL,// AT_BLE_RESOLV_RAND_ADDR_STATUS
		NULL,// AT_BLE_SIGN_COUNTERS_IND
		NULL,// AT_BLE_PEER_ATT_INFO_IND
		NULL // AT_BLE_CON_CHANNEL_MAP_IND
};

/*Struct to handle HTPT callback*/
static const ble_event_callback_t app_htpt_handle[] = {
	NULL, // AT_BLE_HTPT_CREATE_DB_CFM
	NULL, // AT_BLE_HTPT_ERROR_IND
	NULL, // AT_BLE_HTPT_DISABLE_IND
	NULL, // AT_BLE_HTPT_TEMP_SEND_CFM
	NULL, // AT_BLE_HTPT_MEAS_INTV_CHG_IND
	app_htpt_cfg_indntf_ind_handler, // AT_BLE_HTPT_CFG_INDNTF_IND
	NULL, // AT_BLE_HTPT_ENABLE_RSP
	NULL, // AT_BLE_HTPT_MEAS_INTV_UPD_RSP
	NULL // AT_BLE_HTPT_MEAS_INTV_CHG_REQ
};

/* =========================== FUNCTIONS =================================== */
#if BLOCK_EXAMPLE_CODE
static void uart_rx_callback(uint8_t input)
{
	if(input == '\r') {
		if(send_length) {
			send_plf_int_msg_ind(UART_RX_COMPLETE, UART_RX_INTERRUPT_MASK_RX_FIFO_NOT_EMPTY_MASK, send_data, send_length);
			memset(send_data, 0, APP_TX_BUF_SIZE);
			send_length = 0;
			DBG_LOG(" ");
		}
	}
	else {
		send_data[send_length++] = input;
		DBG_LOG_CONT("%c", input);
		
		if(send_length >= APP_TX_BUF_SIZE) {
			send_plf_int_msg_ind(UART_RX_COMPLETE, UART_RX_INTERRUPT_MASK_RX_FIFO_NOT_EMPTY_MASK, send_data, send_length);
			memset(send_data, 0, APP_TX_BUF_SIZE);
			send_length = 0;
		}
	}
}
#endif

/**
* @brief app_connected_state BLE manager notifies the application about state
* @param[in] at_ble_conn_param_update_done_t
*/
static at_ble_status_t app_connection_param_update_done(void *params)
{
	at_ble_conn_param_update_done_t *updated_params = (at_ble_conn_param_update_done_t*)params;
	return AT_BLE_SUCCESS;
}
/**
* @brief app_connected_state ble manager notifies the application about state
* @param[in] at_ble_connected_t
*/
static at_ble_status_t app_connected_event_handler(void *params)
{
	at_ble_connection_params_t conn_params;
	at_ble_connected_t *connection_handle = (at_ble_connected_t*) params;
	conn_handle_update =  connection_handle->handle;
	
	conn_params.con_intv_min = CONNECTION_INTERVAL_MIN;
	conn_params.con_intv_max = CONNECTION_INTERVAL_MAX;
	conn_params.con_latency  = SLAVE_LATENCY;
	conn_params.ce_len_max = GAP_CE_LEN_MAX;
	conn_params.ce_len_min = GAP_CE_LEN_MIN;
	conn_params.superv_to = GAP_SUPERVISION_TIMOUT;

	status = at_ble_connection_param_update(conn_handle_update, &conn_params);
	
	for(int j=0;j<1500000;j++);
	if(status != AT_BLE_SUCCESS)
	{
		//control_LED = false;
		printf("Failure in Updating Connection Params");
	}
	return status;
}

/**
 * @brief app_connected_state ble manager notifies the application about state
 * @param[in] connected
 */
static at_ble_status_t app_disconnected_event_handler(void *params)
{
	printf("\nAssignment 3.2: Application disconnected ");
	/* Started advertisement */
	csc_prf_dev_adv();		
	//ALL_UNUSED(params);
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_CONNECTED event*/
static at_ble_status_t ble_paired_cb (void *param)
{
	at_ble_pair_done_t *pair_params = param;
	printf("\nApplication paired ");
	/* Enable the HTP Profile */
	printf("\nEnable health temperature service ");
	status = at_ble_htpt_enable(pair_params->handle, HTPT_CFG_INTERM_MEAS_NTF);
	if(status != AT_BLE_SUCCESS)
	{
		printf("Failure in HTP Profile Enable\n");
		while(true);
	}
	return AT_BLE_SUCCESS;
}

/*Routine to initialize Health temperature service*/
static void htp_init (void)
{
	printf("\nInit Health temperature service ");
	/* Create htp service in GATT database*/
	status = at_ble_htpt_create_db( HTPT_TEMP_TYPE_CHAR_SUP, HTP_TYPE_ARMPIT, 1, 30, 1, HTPT_AUTH, &htpt_conn_handle);
	if (status != AT_BLE_SUCCESS)
	{
		printf("HTP Data Base creation failed");
		while(true);
	}
}

/* Timer callback */
static void timer_callback_handler(void)
{
	/* Stop timer */
	hw_timer_stop();
	/* Set timer Alarm flag */
	Timer_Flag = true;
	/* Restart Timer */
	//hw_timer_start(10);
}

/* Sending the temperature value after reading it from IO1 Xplained Pro */
static void htp_temperature_send(void)
{
	at_ble_prf_date_time_t timestamp;
	float temperature;
	/* Read Temperature Value from IO1 Xplained Pro */
	//temperature = at30tse_read_temperature();
	temperature = tempValue;
	#ifdef HTPT_FAHRENHEIT
	temperature = (((temperature * 9.0)/5.0) + 32.0);
	#endif
	/* Read Temperature Value from IO1 Xplained Pro */
	timestamp.day = 1;
	timestamp.hour = 9;
	timestamp.min = 2;
	timestamp.month = 8;
	timestamp.sec = 36;
	timestamp.year = 15;
	/* Read Temperature Value from IO1 Xplained Pro */
	if(at_ble_htpt_temp_send(convert_ieee754_ieee11073_float((float)temperature),
	&timestamp,
	#ifdef HTPT_FAHRENHEIT
	(at_ble_htpt_temp_flags)(HTPT_FLAG_FAHRENHEIT | HTPT_FLAG_TYPE),
	#else
	(at_ble_htpt_temp_flags)(HTPT_FLAG_CELSIUS | HTPT_FLAG_TYPE),
	#endif
	HTP_TYPE_ARMPIT,
	1
	) == AT_BLE_SUCCESS)
	{
		#ifdef HTPT_FAHRENHEIT
		printf("\nTemperature: %d Fahrenheit", (uint16_t)temperature);
		#else
		printf("\nTemperature: %d Deg Celsius", (uint16_t)temperature);
		#endif
	}
}

/*Routine for notifying the Temperature*/
static at_ble_status_t app_htpt_cfg_indntf_ind_handler(void *params)
{
	at_ble_htpt_cfg_indntf_ind_t htpt_cfg_indntf_ind_params;
	memcpy((uint8_t *)&htpt_cfg_indntf_ind_params, params, sizeof(at_ble_htpt_cfg_indntf_ind_t));
	if (htpt_cfg_indntf_ind_params.ntf_ind_cfg == 0x03)
	{
		printf("Started HTP Temperature Notification");
		Temp_Notification_Flag = true;
	}
	else
	{
		printf("HTP Temperature Notification Stopped");
		Temp_Notification_Flag = false;
	}
	return AT_BLE_SUCCESS;
}

/*Routine to configure UART*/	
static void configure_usart(void)
{
	struct uart_config config_uart;
	uart_get_config_defaults(&config_uart);
	config_uart.baud_rate = BAUD_RATE;
	config_uart.pin_number_pad[0] = EDBG_CDC_SERCOM_PIN_PAD0;
	config_uart.pin_number_pad[1] = EDBG_CDC_SERCOM_PIN_PAD1;
	config_uart.pin_number_pad[2] = EDBG_CDC_SERCOM_PIN_PAD2;
	config_uart.pin_number_pad[3] = EDBG_CDC_SERCOM_PIN_PAD3;
	config_uart.pinmux_sel_pad[0] = EDBG_CDC_SERCOM_MUX_PAD0;
	config_uart.pinmux_sel_pad[1] = EDBG_CDC_SERCOM_MUX_PAD1;
	config_uart.pinmux_sel_pad[2] = EDBG_CDC_SERCOM_MUX_PAD2;
	config_uart.pinmux_sel_pad[3] = EDBG_CDC_SERCOM_MUX_PAD3;
	config_uart.data_bits = UART_8_BITS; 
	while (uart_init(&uart_instance,EDBG_CDC_MODULE, &config_uart) != STATUS_OK) 
	{
	}
	uart_enable_receive_dma(&uart_instance);
	uart_enable_transmit_dma(&uart_instance);
}


/*Routine to configure DMA resource Reception*/
static void configure_dma_resource(struct dma_resource *resource)
{
	struct dma_resource_config config;
	dma_get_config_defaults(&config);
	config.des.enable_inc_addr = true;
	config.src.enable_inc_addr = false;
	config.des.enable_proi_high = false;
	config.src.enable_proi_high = false;
	config.des.enable_proi_top = true;
	config.src.enable_proi_top = true;
	config.src.periph = UART0RX_DMA_PERIPHERAL;
	config.des.periph = MEMORY_DMA_PERIPHERAL;
	config.src.periph_delay = 1;
	config.src.max_burst = 1;
	config.src.tokens = 1;
	config.des.tokens = TOTAL_TRANSFERS;
	dma_allocate(resource, &config);
}


/*Routine to configure descriptors for DMA*/
static void setup_transfer_descriptor(struct dma_descriptor *descriptor)
{
	dma_descriptor_get_config_defaults(descriptor);
	descriptor->buffer_size = 6;
	descriptor->read_start_addr = (uint32_t)(&uart_instance.hw->RECEIVE_DATA.reg);
	descriptor->write_start_addr = (uint32_t)destination_temp;
}

/*Routine to DMA call back*/
static void transfer_done_rx(struct dma_resource* const resource )
{
	int i;
	printf("Callback\n");
	if(destination_temp[0] == 'L')			//Sensor data
	{
		if(destination_temp[1] == (char)1.0)	//Data to turn ON Coffee Machine
		{
			askCoffee = true;	
		}
		else if (destination_temp[1] == (char)0)	//Data to notify washing machine
		{
			sendWMChat = true;
		}
	}
	else if(destination_temp[0] == 'T')		//Temp data
	{
		char b[5];
		int j;
		for (i=1,j=0;j<5;i++,j++)
		b[j] = destination_temp[i];
		tempValue = atof (b);
		Timer_Flag = true;
	}
	for(i=0;i<6;i++)
	{
		destination_temp[i] = '0';
	}
	dma_start_transfer_job(&uart_dma_resource_rx);
}

/*DMA Transmission Configure Resource*/
static void configure_dma_resource_tx(struct dma_resource *resource)
{
	struct dma_resource_config config;
	dma_get_config_defaults(&config);
	
	config.des.enable_inc_addr = false;
	config.src.enable_inc_addr = false;	//true;
	config.des.enable_proi_high = false;
	config.src.enable_proi_high = false;
	config.des.enable_proi_top = true;
	config.src.enable_proi_top = true;
	config.src.periph = MEMORY_DMA_PERIPHERAL;
	config.des.periph = UART0TX_DMA_PERIPHERAL;
	config.src.periph_delay = 1;
	config.src.max_burst = 1;
	config.src.tokens = 1;
	config.des.tokens = 1;	//TOTAL_TRANSFERS;
	dma_allocate(resource, &config);

}

/*DMA Transmission Transfer Descriptor*/
static void setup_transfer_descriptor_tx(struct dma_descriptor *descriptor)
{
	dma_descriptor_get_config_defaults(descriptor);
	descriptor->buffer_size = 1;
	descriptor->read_start_addr = (uint32_t)&dataToLG;
	descriptor->write_start_addr = (uint32_t)(&uart_instance.hw->TRANSMIT_DATA.reg);
}

/*DMA Transmission done Callback*/
static void transfer_done_tx(struct dma_resource* const resource)
{}


/*Routine to configure DMA call back*/
static void configure_dma_callback(void)
{
	dma_register_callback(&uart_dma_resource_rx, transfer_done_rx, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&uart_dma_resource_rx, DMA_CALLBACK_TRANSFER_DONE);
	
	dma_register_callback(&uart_dma_resource_tx, transfer_done_tx, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&uart_dma_resource_tx, DMA_CALLBACK_TRANSFER_DONE);
	
	NVIC_EnableIRQ(PROV_DMA_CTRL0_IRQn);
}

/*Routine to configure LED pin*/
void configure_gpio_pins(void)
{
	struct gpio_config config_gpio_pin;
	gpio_get_config_defaults(&config_gpio_pin);
	config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
	gpio_pin_set_config(LED_0_PIN, &config_gpio_pin);
}

/* Function used for receive data from CSC profile*/
static void csc_app_recv_buf(uint8_t *recv_data, uint8_t recv_len)
{
	uint16_t ind = 0;
	if (recv_len){
		for (ind = 0; ind < recv_len; ind++){
			DBG_LOG_CONT("%c", recv_data[ind]);
		}
		if(recv_data[0] == 'G' || recv_data[0] == 'g')
		{
			dataToLG = 1;
			dma_start_transfer_job(&uart_dma_resource_tx);
		}
		else if(recv_data[0] == 'S' || recv_data[0] == 's')
		{
			control_LED = false;
		}
		else if(recv_data[0] == 'O' || recv_data[0] == 'o')
		{
			control_LED = true;
		}
		DBG_LOG("\r\n");
	}
}

/* Callback called for new data from remote device */
static void csc_prf_report_ntf_cb(csc_report_ntf_t *report_info)
{
	DBG_LOG("\r\n");
	csc_app_recv_buf(report_info->recv_buff, report_info->recv_buff_len);
}

/* Function used for send data */
static void csc_app_send_buf(void)
{
	uint16_t plf_event_type;
	uint16_t plf_event_data_len;
	uint8_t plf_event_data[APP_TX_BUF_SIZE] = {0, };
	platform_event_get(&plf_event_type, plf_event_data, &plf_event_data_len);
	if(plf_event_type == ((UART_RX_INTERRUPT_MASK_RX_FIFO_NOT_EMPTY_MASK << 8) | UART_RX_COMPLETE)) 
	{
		csc_prf_send_data(plf_event_data, plf_event_data_len);
	}
}

/*Main Routine*/
bool app_exec = true;
int main(void )
{
	platform_driver_init();
	acquire_sleep_lock();

	/* Initialize serial console  */
	serial_console_init();

	DBG_LOG("Initializing Custom Serial Chat Application");
	
	/* Initialize the buffer address and buffer length based on user input */
	csc_prf_buf_init(&send_data[0], APP_TX_BUF_SIZE);
	
	/* Hardware timer */
	hw_timer_init();
	
	/* Register the callback */
	hw_timer_register_callback(timer_callback_handler);
	
	/* Start timer */
	hw_timer_start(1);
	
	printf("\n\rSAMB11 BLE Application");
	
	/* initialize the ble chip  and Set the device mac address */
	ble_device_init(NULL);
	
	/* configure the GPIO*/
	configure_gpio_pins();
	
	/* Initializing the profile */
	csc_prf_init(NULL);
		
	/* Initialize the htp service */
	htp_init();
	
	/*Configure UART DMA*/
	system_clock_config(CLOCK_RESOURCE_XO_26_MHZ, CLOCK_FREQ_26_MHZ);
	
	configure_usart();
	
	configure_dma_resource_tx(&uart_dma_resource_tx);
	
	configure_dma_resource(&uart_dma_resource_rx);
	
	setup_transfer_descriptor_tx(&example_descriptor_tx);
	
	setup_transfer_descriptor(&example_descriptor_rx);
	
	dma_add_descriptor(&uart_dma_resource_tx, &example_descriptor_tx);
	
	dma_add_descriptor(&uart_dma_resource_rx, &example_descriptor_rx);
	
	configure_dma_callback();
	
	/*Start DMA transfer*/
	dma_start_transfer_job(&uart_dma_resource_rx);
	
	/*Registering GAP Event Call Back*/
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK,BLE_GAP_EVENT_TYPE,app_gap_handle);
	
	/*Registering call back for GATT HTPT Event type*/
	status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_HTPT_EVENT_TYPE,app_htpt_handle);
	if (status != true)
	{
		printf("\n##Error when Registering SAMB11 htpt callbacks");
	}
	if((status = at_ble_tx_power_set(AT_BLE_TX_PWR_LVL_NEG_05_DB)) != AT_BLE_SUCCESS)
	{
		 printf("Failed to set tx power\n",status);
	}
	/* Started advertisement */
	csc_prf_dev_adv();
	
	/* Register the notification handler */
	notify_recv_ntf_handler(csc_prf_report_ntf_cb);
	
	#if BLOCK_EXAMPLE_CODE
	/* Register the user event handler */
	register_ble_user_event_cb(csc_app_send_buf);
	
	register_uart_callback(uart_rx_callback);
	#endif
	
	/* Capturing the events  */
	while(app_exec)
	{
		ble_event_task(655);
		gpio_pin_set_output_level(LED_0_PIN, control_LED);	// LIGHT SENSOR LED
		if(sendWMChat)		//Send data to CSC Profile
		{
			csc_prf_send_data(machineStarted, 16);		
			sendWMChat = false;
		}
		if(askCoffee)		
		{
			askCoffee = false;
			csc_prf_send_data(coffeeOn,17);
		}
		if (Timer_Flag & Temp_Notification_Flag)
		{
			htp_temperature_send();
			Timer_Flag = false;
		}
	}
	return 0;
}



