/**
 * \file
 *
 * \brief LORAWAN Demo Application
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
/****************************** INCLUDES **************************************/
#include "system_low_power.h"
#include "radio_driver_hal.h"
#include "lorawan.h"
#include "sys.h"
#include "system_init.h"
#include "system_assert.h"
#include "aes_engine.h"
#include "enddevice_demo.h"
#include "sio2host.h"
#include "extint.h"
#include "conf_app.h"
#include "sw_timer.h"
#ifdef CONF_PMM_ENABLE
#include "pmm.h"
#include  "conf_pmm.h"
#include "sleep_timer.h"
#include "sleep.h"
#endif
#include "conf_sio2host.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif
/************************** Macro definition ***********************************/
/* Button debounce time in ms */
#define APP_DEBOUNCE_TIME       50
/************************** Global variables ***********************************/
bool button_pressed = false;
bool factory_reset = false;
bool bandSelected = false;
uint32_t longPress = 0;
#ifdef CONF_PMM_ENABLE
bool deviceResetsForWakeup = false;
#endif
/************************** Extern variables ***********************************/

/************************** Function Prototypes ********************************/
static void driver_init(void);
static void button_read (void);
#if (DEBUG == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code);
#endif /* #if (DEBUG == 1) */
/*********************************************************************//**
 \brief      Configures the External Interrupt Controller to detect changes in the board
             button state.
*************************************************************************/
static void configure_extint(void);

/*********************************************************************//**
 \brief      Configures and registers the External Interrupt callback function with the
        driver.
*************************************************************************/
static void configure_eic_callback(void);
/*********************************************************************//**
 \brief      Uninitializes app resources before going to low power mode
*************************************************************************/
#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void);
#endif

/****************************** FUNCTIONS **************************************/

static void print_reset_causes(void)
{
	enum system_reset_cause rcause = system_get_reset_cause();
	printf("Last reset cause: ");
	if(rcause & (1 << 6)) {
		printf("System Reset Request\r\n");
	}
	if(rcause & (1 << 5)) {
		printf("Watchdog Reset\r\n");
	}
	if(rcause & (1 << 4)) {
		printf("External Reset\r\n");
	}
	if(rcause & (1 << 2)) {
		printf("Brown Out 33 Detector Reset\r\n");
	}
	if(rcause & (1 << 1)) {
		printf("Brown Out 12 Detector Reset\r\n");
	}
	if(rcause & (1 << 0)) {
		printf("Power-On Reset\r\n");
	}
}

#ifdef CONF_PMM_ENABLE
static void appWakeup(uint32_t sleptDuration)
{   
	HAL_Radio_resources_init();
	sio2host_init();
	printf("\r\nsleep_ok %ld ms\r\n", sleptDuration);
	
}
#endif

#if (DEBUG == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code)
{
	printf("\r\n%04x\r\n", code);
	(void)level;
}
#endif /* #if (DEBUG == 1) */

/** Configures and registers the External Interrupt callback function with the
 *  driver.
 */
static void configure_eic_callback(void)
{
	extint_register_callback(button_read,
			BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
}

/** Configures the External Interrupt Controller to detect changes in the board
 *  button state.
 */
static void configure_extint(void)
{
	struct extint_chan_conf eint_chan_conf;
	extint_chan_get_config_defaults(&eint_chan_conf);

	eint_chan_conf.gpio_pin           = BUTTON_0_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_FALLING;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &eint_chan_conf);
}


/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the LORAWAN Demo Application of EU Band
 */
int main(void)
{
	/* System Initialization */
	system_init();	
	/* Initialize the delay driver */
	delay_init();
	/* Initialize the board target resources */
	board_init();
	
	/*Configures the External Interrupt*/
	configure_extint();

	/*Configures the External Interrupt callback*/
	configure_eic_callback();

	INTERRUPT_GlobalInterruptEnable();
	/* Initialize Hardware and Software Modules */
	driver_init();
	/* Initialize the Serial Interface */
	sio2host_init();

	print_reset_causes();
#if (DEBUG == 1)
	SYSTEM_AssertSubscribe(assertHandler);
#endif	
	/* Initialize demo application */
	Stack_Init();
	mote_demo_init();

    while (1)
    {
		SYSTEM_RunTasks();
#ifdef CONF_PMM_ENABLE	
		if(bandSelected == true)
		{
            PMM_SleepReq_t sleepReq;
	    	/* Put the application to sleep */
		    sleepReq.sleepTimeMs = CONF_DEFAULT_APP_SLEEP_TIME_MS;
    		sleepReq.pmmWakeupCallback = appWakeup;
			if (CONF_PMM_SLEEPMODE_WHEN_IDLE == SLEEP_MODE_STANDBY)
			{ /* except standby, no other mode is support for SAMR34 */
				deviceResetsForWakeup = false;
			}
			if (true == LORAWAN_ReadyToSleep(deviceResetsForWakeup))
    		{   
				app_resources_uninit();		
				if (PMM_SLEEP_REQ_DENIED == PMM_Sleep(&sleepReq))
		    {
			    /*printf("\r\nsleep_not_ok\r\n");*/
			   HAL_Radio_resources_init();
			   sio2host_init();
		    }
			}
		}
#endif
    }
}

/* Initializes all the hardware and software modules used for Stack operation */
static void driver_init(void)
{
	/* Initialize the Radio Hardware */
	HAL_RadioInit();
	/* Initialize the AES Hardware Engine */
	AESInit();
	/* Initialize the Software Timer Module */
	SystemTimerInit();
#ifdef CONF_PMM_ENABLE
	/* Initialize the Sleep Timer Module */
	SleepTimerInit();
#endif
#if (ENABLE_PDS == 1)
	/* PDS Module Init */
	PDS_Init();
#endif
}


/* Read for button press */
static void button_read (void)
{
	uint8_t count = 0;

	/* Read the button level */
	if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
	{
#ifdef CONF_PMM_ENABLE
		PMM_Wakeup();
#endif		
		/* Wait for button debounce time */
		delay_ms(APP_DEBOUNCE_TIME);
		/* Check whether button is in default state */
		while(port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
		{
			delay_ms(500);
			count += 1;
		}
		if (count > 5)
		{
			factory_reset = true;
			longPress++;
		}
		else
		{
			button_pressed = true;	
		}
		
		/* Post task to application handler on button press */
		SYSTEM_PostTask(APP_TASK_ID);
	}
}
#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void)
{
	/* Disable USART TX and RX Pins */
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	pin_conf.powersave  = true;
	port_pin_set_config(HOST_SERCOM_PAD0_PIN, &pin_conf);
	port_pin_set_config(HOST_SERCOM_PAD1_PIN, &pin_conf);
	/* Disable UART module */
	sio2host_deinit();
	/* Disable Transceiver SPI Module */
	HAL_RadioDeInit();
}
#endif
/**
 End of File
 */
