/**
 * \file
 *
 * \brief BLE Startup Template
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
 * Support</a>
 */

/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the Startup Template
 */
/*- Includes ---------------------------------------------------------------*/
#include <asf.h>
#include <string.h>
#include "platform.h"
#include "at_ble_api.h"
#include "console_serial.h"
#include "timer_hw.h"
#include "ble_manager.h"
#include "ble_utils.h"
#include "button.h"
#include "gpio.h"
#include "samb11_xplained_pro.h"
#include "startup_template_app.h"

void configure_gpio_pins(void);

struct uart_module uart_instance;

struct dma_resource uart_dma_resource_tx;
struct dma_resource uart_dma_resource_rx;

#define BUFFER_LEN    5
#define LED_STATUS_BYTE 4
#define BAUD_RATE 9600
static uint8_t string_tx[BUFFER_LEN] = {0};
static uint8_t string_rx[BUFFER_LEN] = {0};
	
struct dma_descriptor example_descriptor_tx;
struct dma_descriptor example_descriptor_rx;

volatile at_ble_status_t status;
at_ble_handle_t htpt_conn_handle;
volatile bool Timer_Flag = false;
volatile bool Temp_Notification_Flag = false;

/* Function: void transfer_done_tx(struct dma_resource* const resource )
 * Parameters:
 *      struct dma_resource: a structure of type dma_resource
 * Return:
 *      void
 * Description:
 *      This is a callback function that will reset the dma call
 *      and make sure that it is ready to recive the next set of 
 *      data bytes on the tx.
 */
static void transfer_done_tx(struct dma_resource* const resource )
{
	dma_start_transfer_job(&uart_dma_resource_rx);
}

/* Function: void transfer_done_rx(struct dma_resource* const resource )
 * Parameters:
 *      struct dma_resource: a structure of type dma_resource
 * Return:
 *      void
 * Description:
 *    - This is a callback function that will reset the dma call
 *      and make sure that it is ready to recive the next set of 
 *      data bytes on the rx.
 *    - This will also handle the toggling of the LEDs based on 
 *      data reception.
 */
static void transfer_done_rx(struct dma_resource* const resource )
{
	/* Start parsing the data here! */
	dma_start_transfer_job(&uart_dma_resource_rx);
	
	if(string_rx[LED_STATUS_BYTE] == 1) {
		/* Toggle the LED to 0 */
		GPIO1->DATAOUT.reg |= (1 << (LED_0_PIN % 16)); // Turn OFF
		} else {
		/* Let the LED be ON */
		GPIO1->DATAOUT.reg &= ~(1 << (LED_0_PIN % 16)); // Turn ON
	}
}
	
/* Function: void configure_dma_resource_tx(struct dma_resource *resource)
 * Parameters:
 *      struct dma_resource: a structure of type dma_resource
 * Return:
 *      static void
 * Description:
 *    - does the basic config. for the DMA on TX.
 */
static void configure_dma_resource_tx(struct dma_resource *resource)
{
	struct dma_resource_config config;

  dma_get_config_defaults(&config);

	config.des.periph = UART1TX_DMA_PERIPHERAL;
	config.des.enable_inc_addr = false;
	config.src.periph = UART1TX_DMA_PERIPHERAL;

	dma_allocate(resource, &config);
}

/* Function: void setup_transfer_descriptor_tx(struct dma_descriptor *descriptor)
 * Parameters:
 *      struct dma_resource: a structure of type dma_resource
 * Return:
 *      static void
 * Description:
 *    - setup the transfer description on the TX DMA.
 */
static void setup_transfer_descriptor_tx(struct dma_descriptor *descriptor)
{

	dma_descriptor_get_config_defaults(descriptor);

	descriptor->buffer_size = BUFFER_LEN;
	descriptor->read_start_addr = (uint32_t)string_tx;
	descriptor->write_start_addr =
	(uint32_t)(&uart_instance.hw->TRANSMIT_DATA.reg);
}

/* Function: void configure_dma_resource_rx(struct dma_resource *resource)
 * Parameters:
 *      struct dma_resource: a structure of type dma_resource
 * Return:
 *      static void
 * Description:
 *    - does the basic config. for the DMA on RX.
 */
static void configure_dma_resource_rx(struct dma_resource *resource)
{
	struct dma_resource_config config;

	dma_get_config_defaults(&config);

	config.src.periph = UART1RX_DMA_PERIPHERAL;
	config.src.enable_inc_addr = false;
	config.src.periph_delay = 1;

	dma_allocate(resource, &config);
}

/* Function: void setup_transfer_descriptor_rx(struct dma_descriptor *descriptor)
 * Parameters:
 *      struct dma_resource: a structure of type dma_resource
 * Return:
 *      static void
 * Description:
 *    - setup the transfer description on the RX DMA.
 */
static void setup_transfer_descriptor_rx(struct dma_descriptor *descriptor)
{
	dma_descriptor_get_config_defaults(descriptor);

	descriptor->buffer_size = BUFFER_LEN;
	descriptor->read_start_addr =
	(uint32_t)(&uart_instance.hw->RECEIVE_DATA.reg);
	descriptor->write_start_addr = (uint32_t)string_rx;
}

/* Function: void configure_usart(void)
 * Parameters:
 *        void
 * Return:
 *      static void
 * Description:
 *      - Do the basic configuration for the usart.
 */
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

  /* Initialize the UART module */
	while (uart_init(&uart_instance,
	EDBG_CDC_MODULE, &config_uart) != STATUS_OK) {
	}

	uart_enable_transmit_dma(&uart_instance);
	uart_enable_receive_dma(&uart_instance);
}

/* Function: void configure_dma_callback(void)
 * Parameters:
 *        void
 * Return:
 *      static void
 * Description:
 *    - Configure the DMA callback functions.
 */
static void configure_dma_callback(void)
{
	dma_register_callback(&uart_dma_resource_tx, transfer_done_tx, DMA_CALLBACK_TRANSFER_DONE);
	dma_register_callback(&uart_dma_resource_rx, transfer_done_rx, DMA_CALLBACK_TRANSFER_DONE);

	dma_enable_callback(&uart_dma_resource_tx, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&uart_dma_resource_rx, DMA_CALLBACK_TRANSFER_DONE);

	NVIC_EnableIRQ(PROV_DMA_CTRL0_IRQn);
}

/* Function: void configure_gpio_pins(void)
 * Parameters:
 *        void
 * Return:
 *      static void
 * Description:
 *      - do the basic config. for the GPIO pins
 */
void configure_gpio_pins(void)
{
	struct gpio_config config_gpio_pin;
	gpio_get_config_defaults(&config_gpio_pin);

	config_gpio_pin.direction  = GPIO_PIN_DIR_INPUT;
	config_gpio_pin.input_pull = GPIO_PIN_PULL_UP;
	gpio_pin_set_config(BUTTON_0_PIN, &config_gpio_pin);

	config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
	gpio_pin_set_config(LED_0_PIN, &config_gpio_pin);
}

/* Function: void ble_advertise (void)
 * Parameters:
 *        void
 * Return:
 *      static void
 * Description:
 *      - Do the necessary instructions to adversitse the BLE device
 */
static void ble_advertise (void)
{
	printf("\nAssignment 2.1 : Start Advertising");
	status = ble_advertisement_data_set();
	if(status != AT_BLE_SUCCESS)
	{
		printf("\n\r## Advertisement data set failed : error %x",status);
		while(1);
	}
	/* Start of advertisement */
	status = at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED,\
	AT_BLE_ADV_GEN_DISCOVERABLE,\
	NULL,\
	AT_BLE_ADV_FP_ANY,\
	1000,\
	655,\
	0);
	if(status != AT_BLE_SUCCESS)
	{
		printf("\n\r## Advertisement data set failed : error %x",status);
		while(1);
	}
}
																																	
/* Function: at_ble_status_t ble_paired_cb (void *param)
 * Parameters:
 *        void *
 * Return:
 *      at_ble_status_t
 * Description:
 *    - enable the health temperature service post device pairing.
 */
static at_ble_status_t ble_paired_cb (void *param)
{
	at_ble_pair_done_t *pair_params = param; 
	printf("\nAssignment 3.2: Application paired ");
	/* Enable the HTP Profile */
	printf("\nAssignment 4.1: enable health temperature service ");
	status = at_ble_htpt_enable(pair_params->handle,
	HTPT_CFG_INTERM_MEAS_NTF);
	if(status != AT_BLE_SUCCESS){
		printf("*** Failure in HTP Profile Enable");
		while(true);
	}
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_DISCONNECTED event */
static at_ble_status_t ble_disconnected_cb (void *param)
{
	printf("\nAssignment 3.2: Application disconnected "); 
	ble_advertise();
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

static const ble_event_callback_t app_gap_cb[] = {
	NULL,                 // AT_BLE_UNDEFINED_EVENT
	NULL,                 // AT_BLE_SCAN_INFO
	NULL,                 // AT_BLE_SCAN_REPORT
	NULL,                 // AT_BLE_ADV_REPORT
	NULL,                 // AT_BLE_RAND_ADDR_CHANGED
	NULL,                 // AT_BLE_CONNECTED
	ble_disconnected_cb,  // AT_BLE_DISCONNECTED
	NULL,                 // AT_BLE_CONN_PARAM_UPDATE_DONE
	NULL,                 // AT_BLE_CONN_PARAM_UPDATE_REQUEST
	ble_paired_cb,        // AT_BLE_PAIR_DONE
	NULL,                 // AT_BLE_PAIR_REQUEST
	NULL,                 // AT_BLE_SLAVE_SEC_REQUEST
	NULL,                 // AT_BLE_PAIR_KEY_REQUEST
	NULL,                 // AT_BLE_ENCRYPTION_REQUEST
	NULL,                 // AT_BLE_ENCRYPTION_STATUS_CHANGED
	NULL,                 // AT_BLE_RESOLV_RAND_ADDR_STATUS
	NULL,                 // AT_BLE_SIGN_COUNTERS_IND
	NULL,                 // AT_BLE_PEER_ATT_INFO_IND
	NULL                  // AT_BLE_CON_CHANNEL_MAP_IND
};

/* Register GAP callbacks at BLE manager level*/
static void htp_init (void)
{
	printf("\nAssignment 4.1: Init Health temperature service ");
	/* Create htp service in GATT database*/
	status = at_ble_htpt_create_db(
	HTPT_TEMP_TYPE_CHAR_SUP,
	HTP_TYPE_ARMPIT,
	1,
	30,
	1,
	HTPT_AUTH,
	&htpt_conn_handle);
	if (status != AT_BLE_SUCCESS){
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
	hw_timer_start(10);
}


/* Sending the temperature value after reading it from IO1 Xplained Pro */
static void htp_temperature_send(void)
{
	at_ble_prf_date_time_t timestamp;
	float *temp; 
	temp = &string_rx;
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
	if(at_ble_htpt_temp_send(convert_ieee754_ieee11073_float((float)*temp),
	&timestamp,
	#ifdef HTPT_FAHRENHEIT
	(at_ble_htpt_temp_flags)(HTPT_FLAG_FAHRENHEIT | HTPT_FLAG_TYPE),
	#else
	(at_ble_htpt_temp_flags)(HTPT_FLAG_CELSIUS | HTPT_FLAG_TYPE),
	#endif
	HTP_TYPE_ARMPIT,
	1
	) == AT_BLE_SUCCESS) {
  }
}


static at_ble_status_t app_htpt_cfg_indntf_ind_handler(void *params)
{
	at_ble_htpt_cfg_indntf_ind_t htpt_cfg_indntf_ind_params;
	memcpy((uint8_t *)&htpt_cfg_indntf_ind_params, params,
	sizeof(at_ble_htpt_cfg_indntf_ind_t));
	if (htpt_cfg_indntf_ind_params.ntf_ind_cfg == 0x03) {
		printf("Started HTP Temperature Notification");
		Temp_Notification_Flag = true;
	}
	else {
		printf("HTP Temperature Notification Stopped");
		Temp_Notification_Flag = false;
	}
	return AT_BLE_SUCCESS;
}


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


static void register_ble_callbacks (void)
{
	/* Register GAP Callbacks */
	printf("\nAssignment 3.2: Register bluetooth events callbacks");
	status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,\
	BLE_GAP_EVENT_TYPE,app_gap_cb);
	if (status != true) {
		printf("\n##Error when Registering SAMB11 gap callbacks");
	}
	status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,\
	BLE_GATT_HTPT_EVENT_TYPE,app_htpt_handle);
	if (status != true) {
		printf("\n##Error when Registering SAMB11 htpt callbacks");
	}
}

int main (void)
{
	platform_driver_init();
	acquire_sleep_lock();
	/* Initialize serial console */
	serial_console_init();
	/* Register the callback */
	hw_timer_register_callback(timer_callback_handler);
	/* Start timer */
	hw_timer_start(1);

	/* Hardware timer */
	hw_timer_init();
	
	/* Do the initialization for the GPIO */
	configure_usart();

	// Configure the GPIO pins for the LED
	configure_gpio_pins();
	
	configure_dma_resource_tx(&uart_dma_resource_tx);
	configure_dma_resource_rx(&uart_dma_resource_rx);

	setup_transfer_descriptor_tx(&example_descriptor_tx);
	setup_transfer_descriptor_rx(&example_descriptor_rx);

	dma_add_descriptor(&uart_dma_resource_tx, &example_descriptor_tx);
	dma_add_descriptor(&uart_dma_resource_rx, &example_descriptor_rx);

	configure_dma_callback();
	
	printf("\n\rSAMB11 BLE Application");
	/* initialize the BLE chip and Set the Device Address */
	ble_device_init(NULL); 
	
	/* Initialize the temperature sensor */
	at30tse_init();
	/* configure the temperature sensor ADC */
	at30tse_write_config_register(AT30TSE_CONFIG_RES(AT30TSE_CONFIG_RES_12_bit));

  /* Initialize the htp service */
	htp_init();
	
	/* Start Advertising process */
	ble_advertise();
	
	dma_start_transfer_job(&uart_dma_resource_rx);

	/* Register Bluetooth events Callbacks */
	register_ble_callbacks();
	
	while(true) {
		ble_event_task(655);
		if (Timer_Flag & Temp_Notification_Flag)
		{
			htp_temperature_send();
		}
	}
}
