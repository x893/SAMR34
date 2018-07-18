/******************************************************************************
@Company:
Microchip Technology Inc.

@File Name:
pmm.c

@Summary:
This is the implementation of LoRaWAN power management module

@Description:
This header file provides LoRa configuration.
Copyright (c) 2013 - 2016 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*************************************************************************
*                           pmm.c
* Power manager implementation
*
******************************************************************************/

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
/* Standard headers */
#include <stdint.h>

/* PMM headers */
#include "pmm.h"
#include "conf_pmm.h"

/* Timer headers */
#include "hw_timer.h"
#include "sw_timer.h"
#include "sleep_timer.h"

/* HAL sleep header */
#include "sleep.h"

/* Other required headers */
#include "atomic.h"
#include "system_task_manager.h"

#ifdef CONF_PMM_ENABLE
/************************************************************************/
/* Defines                                                              */
/************************************************************************/
#define	SW_TIMER_ERROR_CORRECTION_TICKS		2

/******************************************************************************
                     Prototypes section
******************************************************************************/
static inline bool validateSleepDuration(uint32_t durationMs);

/************************************************************************/
/* Static variables                                                     */
/************************************************************************/
static PMM_SleepReq_t *sleepReq = NULL;
static PMM_State_t pmmState = PMM_STATE_ACTIVE;

/************************************************************************/
/* Function definitions                                                 */
/************************************************************************/
static inline bool validateSleepDuration(uint32_t durationMs)
{
    return (PMM_SLEEPTIME_MIN_MS <= durationMs) && \
        (PMM_SLEEPTIME_MAX_MS >= durationMs) && \
        (SWTIMER_INVALID_TIMEOUT != durationMs);
}

/**
* \brief This function puts the system to sleep if possible
*
* \param[in]  *req  -  pointer to PMM_SleepReq_t request structure
*
* \return value of type PMM_Status_t
*         PMM_SLEEP_REQ_DENIED -- when sleep is not possible at the instance
*         PMM_SLEEP_REQ_PROCESSED -- when sleep is possible and have already done
*/
PMM_Status_t PMM_Sleep(PMM_SleepReq_t *req)
{
    PMM_Status_t status = PMM_SLEEP_REQ_DENIED;
    uint32_t sysSleepTime = ~0u; /* 0xffFFffFF is invalid */
    
    
    if (NULL != req)
    {
        if (SYSTEM_ReadyToSleep())
        {
            if (validateSleepDuration(req->sleepTimeMs))
            {
                    
                    sysSleepTime = SwTimerNextExpiryDuration();
                    if (SWTIMER_INVALID_TIMEOUT == sysSleepTime)
                    {
                        sysSleepTime = PMM_SLEEPTIME_MAX_MS;
                    }
                    else
                    {
                        sysSleepTime = US_TO_MS(sysSleepTime);
                    }

                    if (validateSleepDuration(sysSleepTime))
                    {
                        if (req->sleepTimeMs < sysSleepTime)
                        {
                            sysSleepTime = req->sleepTimeMs;
                        }

                        SystemTimerSuspend();
                        SleepTimerStart(
                            MS_TO_SLEEP_TICKS(sysSleepTime - PMM_WAKEUPTIME_MS),
                            PMM_Wakeup
                        );

                        pmmState = PMM_STATE_SLEEP;
                        sleepReq = req;

                        HAL_Sleep(CONF_PMM_SLEEPMODE_WHEN_IDLE);

                        status = PMM_SLEEP_REQ_PROCESSED;
                    }
                
            }
        }
    }
    
    return status;
}

/**
* \brief Wakeup from sleep
*/
void PMM_Wakeup(void)
{
    uint64_t sleptTimeUs = 0;

    if (PMM_STATE_SLEEP == pmmState)
    {
        
		pmmState = PMM_STATE_ACTIVE;
        sleptTimeUs = SLEEP_TICKS_TO_US(SleepTimerGetElapsedTime());
        SleepTimerStop();

        SystemTimerSync(sleptTimeUs);
        if (sleepReq && sleepReq->pmmWakeupCallback)
        {
            sleepReq->pmmWakeupCallback(US_TO_MS(sleptTimeUs));
            sleepReq = NULL;
        }
    }
}

#endif /* CONF_PMM_ENABLE */

/* eof pmm.c */
