/**
* @file sw_timer.h
*
* @brief
*
* Copyright (c) 2013-2015 Atmel Corporation. All rights reserved.
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
*/

/* Prevent double inclusion */
#ifndef COMMON_SW_TIMER_H
#define COMMON_SW_TIMER_H

/******************************************************************************
                     Includes section
******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "sys.h"
#include "stack_common.h"
#include "system_task_manager.h"

/******************************************************************************
                     Types section
******************************************************************************/
/*
* Type definition for callbacks for timer functions
*/
typedef void (*SwTimerCallbackFunc_t)(void *);

/* Timeout type */
typedef enum _SwTimeoutType {
	/** The timeout is relative to the current time. */
	SW_TIMEOUT_RELATIVE,
	/** The timeout is an absolute value. */
	SW_TIMEOUT_ABSOLUTE
} SwTimeoutType_t;

/*
* This defines the structure of the time type.
*/
typedef struct _SwTimer {
	/* Timeout in microseconds */
	uint32_t absoluteExpiryTime;

	/* Callback function to be executed on expiry of the timer */
	void (*timerCb)(void*);

	/* Parameter to be passed to callback function of the expired timer */
	void *paramCb;

	/* Next timer which was started or has expired */
	uint8_t nextTimer;

	/* Whether this time is loaded is actually loaded into timer or not? */
	bool loaded;
} SwTimer_t;

/******************************************************************************
                     Macros section
******************************************************************************/
/*
* Value to indicate end of timer in the array or queue
*/
#define SWTIMER_INVALID              (0xFF)

/*
* Shift mask to obtain the 16-bit system time out of a 32-bit timeout
*/
#define SWTIMER_SYSTIME_SHIFTMASK    (16)

/*
* Mask to obtain the 16-bit H/W time out of a 32-bit timeout
*/
#define SWTIMER_HWTIME_MASK          (0xFFFF)

/*
* The smallest timeout in microseconds
*/
#define SWTIMER_MIN_TIMEOUT          (0xff)

/*
* The largest timeout in microseconds
*/
#define SWTIMER_MAX_TIMEOUT          (0x7FFFFFFF)

/*
* Invalid timeout
*/
#define SWTIMER_INVALID_TIMEOUT      (0xFFFFFFFF)

/**
* Adds two time values
*/
#define ADD_TIME(a, b)               ((a) + (b))

/**
* Subtracts two time values
*/
#define SUB_TIME(a, b)               ((a) - (b))

/*
* Macro to convert milliseconds to microseconds
*/
#define MS_TO_US(m)                  ((m) * (1000))

/*
* Macro to convert microseconds to milliseconds
*/
#define US_TO_MS(u)                  ((u) / (1000))

/******************************************************************************
                     Prototypes section
******************************************************************************/

/**************************************************************************//**
\brief Returns a timer id to be used before starting a timer

\param[out] timerId Value of the id returned by the function

\return LORAWAN_SUCCESS if new timerId is allocated
        LORAWAN_RESOURCE_UNAVAILABLE if there is no more timerId to allocate
******************************************************************************/
StackRetStatus_t SwTimerCreate(uint8_t *timerId);

/**************************************************************************//**
\brief Starts a  timer

       This function starts a regular timer and installs the corresponding
       callback function handle the timeout event.

\param[in] timerId Timer identifier
\param[in] timerCount Timeout in microseconds
\param[in] timeoutType \ref SW_TIMEOUT_RELATIVE or \ref SW_TIMEOUT_ABSOLUTE
\param[in] timerCb Callback handler invoked upon timer expiry
\param[in] paramCb Argument for the callback handler

\return LORAWAN_INVALID_PARAMETER if at least one input parameter in invalid
        LORAWAN_INVALID_REQUEST if \timerId is already running
        LORAWAN_SUCCESS if \timerId is successfully queued for running
******************************************************************************/
StackRetStatus_t SwTimerStart(uint8_t timerId, uint32_t timerCount,
  SwTimeoutType_t timeoutType, void *timerCb, void *paramCb);

/**************************************************************************//**
\brief Stops a running timer. It stops a running timer with specified timerId
\param timer_id Timer identifier
\return
        LORAWAN_INVALID_PARAMETER if timerId is not valid
        LORAWAN_INVALID_REQUEST if timerId was not started before
        LORAWAN_SUCCESS if it is successfully stopped
******************************************************************************/
StackRetStatus_t SwTimerStop(uint8_t timerId);

/**************************************************************************//**
\brief Get current system time.
\return Returns current system time in microseconds
******************************************************************************/
uint64_t SwTimerGetTime(void);

/**************************************************************************//**
\brief Checks whether a given timer is running or not
\param[in] timerId Timer ID to be checked for running
\return True if the timer is running else False
******************************************************************************/
bool SwTimerIsRunning(uint8_t timerid);

/**************************************************************************//**
\brief Handles Queues and Callbacks for Expired Timers
******************************************************************************/
void SwTimersExecute(void);

/**************************************************************************//**
\brief Initializes the Software Timer module
******************************************************************************/
void SystemTimerInit(void);

/**************************************************************************//**
\brief Resets the Software Timer module
******************************************************************************/
void SwTimerReset(void);

/**************************************************************************//**
\brief Returns the remaining timeout for the given timerId
\param[in] timerId Timer ID to get the remaining time
\return Remaining time until expiry in microseconds
******************************************************************************/
uint32_t SwTimerReadValue(uint8_t timerId);

/**************************************************************************//**
\brief Run the running timer for the given offset
\param[in] offset New time duration for the running timer
******************************************************************************/
void SwTimerRunRemainingTime(uint32_t offset);

/**************************************************************************//**
\brief Returns the duration until the next timer expiry
\return Returns the duration until the next timeout in microseconds
******************************************************************************/
uint32_t SwTimerNextExpiryDuration(void);

/**************************************************************************//**
\brief Handler for the timer tasks
\return SYSTEM_TASK_SUCCESS after servicing the timer triggers
******************************************************************************/
SYSTEM_TaskStatus_t TIMER_TaskHandler(void);

/**************************************************************************//**
\brief Suspends the software timer
******************************************************************************/
void SystemTimerSuspend(void);

/**************************************************************************//**
\brief Resumes the software timer by offseting it with given time
\param[in] timeToSync Amount of duration to offset from known system time
******************************************************************************/
void SystemTimerSync(uint64_t timeToSync);

#endif /* COMMON_SW_TIMER_H */

/* eof sw_timer.h */
