/**
* \file  sleep_timer.c
*
* \brief Sleep timer implementation
*
* Copyright (C) 2016 Atmel Corporation. All rights reserved.
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

/**************************************** INCLUDES*****************************/
#include "sleep_timer.h"
#include <rtc_count.h>
#include <rtc_count_interrupt.h>

#ifdef CONF_PMM_ENABLE
/**************************************** EXTERNS ****************************/
struct rtc_module rtc;

/************************************** IMPLEMENTATION************************/
/**
* \brief Initializes the sleep timer module
*/
void SleepTimerInit(void)
{
	struct rtc_count_config rtc_config;
	rtc_count_get_config_defaults(&rtc_config);
	
	rtc_config.prescaler = RTC_COUNT_PRESCALER_OFF;
	rtc_config.enable_read_sync = true;
	rtc_config.compare_values[0] = COMPARE_COUNT_MAX_VALUE;
	rtc_config.compare_values[1] = COMPARE_COUNT_MAX_VALUE;
	rtc_count_init(&rtc, RTC, &rtc_config);
	rtc_count_enable(&rtc);
}

/**
* \brief Calculate the Elapsed Time from the previous call of this function
* \retval Elapsed time in ticks
*/
uint32_t SleepTimerGetElapsedTime(void)
{
	return rtc_count_get_count(&rtc);
}

/**
* \brief Initializes the sleep timer
*/
void SleepTimerStart(uint32_t sleepTicks, void (*cb)(void))
{
	rtc_count_set_count(&rtc, 0);
	rtc_count_register_callback(&rtc, cb, RTC_COUNT_CALLBACK_COMPARE_0);
	rtc_count_set_compare(&rtc, sleepTicks, RTC_COUNT_COMPARE_0);
	rtc_count_enable_callback(&rtc, RTC_COUNT_CALLBACK_COMPARE_0);
}

/**
* \brief Stop the sleep timer
*/
void SleepTimerStop(void)
{
	rtc_count_disable_callback(&rtc, RTC_COUNT_CALLBACK_COMPARE_0);
}

#endif /* CONF_PMM_ENABLE */

/* eof sleep_timer.c */
