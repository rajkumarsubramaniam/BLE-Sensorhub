/* 
 * file:csc_app.h
 * author: Raj Kumar Subramaniam
 * brief: Smart Home System - Convenient Peripheral 
 * description : This is the code for the entire BLE module for the Convenient Peripheral
 * 				 This controls the sensor hub and acts as the peripheral device
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
 * Support</a>
 */

#ifndef __CSC_APP_H__
#define __CSC_APP_H__
#include "stdio.h"

/****************************************************************************************
*							        Macros	                                     							*
****************************************************************************************/
/**@brief Keypad debounce time */
#define KEY_PAD_DEBOUNCE_TIME	(200)

/**@brief Application maximum transmit buffer size */
#define APP_TX_BUF_SIZE   (150)

/**@brief Enter button press to send data */
#define ENTER_BUTTON_PRESS (13)


/**
* @brief app_connected_state blemanager notifies the application about state
* @param[in] at_ble_connected_t
*/
static at_ble_status_t app_connected_event_handler(void *params);

/**
 * @brief app_connected_state ble manager notifies the application about state
 * @param[in] connected
 */
static at_ble_status_t app_disconnected_event_handler(void *params);

static at_ble_status_t app_connection_param_update_done(void *params);

static at_ble_status_t ble_paired_cb (void *param);

static at_ble_status_t app_htpt_cfg_indntf_ind_handler(void *params);

void configure_gpio_pins(void);

#endif /*__CSC_APP_H__*/
