/**
* \file  sleep_timer.h
*
* \brief Sleep timer interface
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

#ifndef SLEEP_TIMER_H_INCLUDED
#define SLEEP_TIMER_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
#ifdef CONF_PMM_ENABLE
/**************************************** INCLUDES*****************************/
#include <stdint.h>

#define     COMPARE_COUNT_MAX_VALUE     (0xFFFFFFFF)

#define     US_TO_SLEEP_TICKS(u)        ((u) * (0.03278f))
#define     SLEEP_TICKS_TO_US(s)        ((s) * (30.5175f))

#define     MS_TO_SLEEP_TICKS(m)        ((m) * (32.769f))
#define     SLEEP_TICKS_TO_MS(s)        ((s) * (0.0306f))

/***************************************PROTOTYPES**************************/
/**
* \brief Initializes the sleep timer module
*/
void SleepTimerInit(void);

/**
* \brief Start the sleep timer
*/
void SleepTimerStart(uint32_t time, void (*cb)(void));

/**
* \brief Stop the sleep timer
*/
void SleepTimerStop(void);

/**
* \brief Calculate the Elapsed Time from the previous call of this function
* \retval Elapsed time in ticks
*/
uint32_t SleepTimerGetElapsedTime(void);

#endif /* CONF_PMM_ENABLE */

#ifdef  __cplusplus
}
#endif

#endif /* SLEEP_TIMER_H_INCLUDED */

/* eof sleep_timer.h */

