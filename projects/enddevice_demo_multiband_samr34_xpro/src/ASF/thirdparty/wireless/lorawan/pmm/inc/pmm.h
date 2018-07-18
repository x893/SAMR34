/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    pmm.h

  @Summary:
    This is the interface of LoRaWAN power management module

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
 *                           pmm.h
 * Power manager interface
 *
 ******************************************************************************/
#ifndef PMM_H
#define PMM_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef CONF_PMM_ENABLE

/************************************************************************/
/* Constants                                                            */
/************************************************************************/

#define  PMM_WAKEUPTIME_MS         10u         // 10ms
#define  PMM_SLEEPTIME_MIN_MS      1000u       // 1s
#define  PMM_SLEEPTIME_MAX_MS      130990000u  // 36h23m10s

/************************************************************************/
/* Types                                                                */
/************************************************************************/

/* Describes the status of power manager for a sleep request */
typedef enum _PMM_Status_t
{
	/*
	 * PMM denies the request because system is not ready to sleep at
	 * the instance of sleep call.
	 */
	PMM_SLEEP_REQ_DENIED = 0,

	/*
	 * Power manager accepted and have already processed the request.
	 * i.e., sleep was done.
	 */
	PMM_SLEEP_REQ_PROCESSED
} PMM_Status_t;

/* Describes the states of power manager */
typedef enum _PMM_State_t
{
	/* PMM has come out of sleep */
	PMM_STATE_ACTIVE = 0,

	/* PMM is starting to sleep */
	PMM_STATE_SLEEP
} PMM_State_t;

/* Structure of sleep request */
typedef struct _PMM_SleepReq_t
{
	/* Sleep time requested to PMM. Unit is milliseconds */
	uint32_t sleepTimeMs;

	/* Callback from sleep request */
	void (*pmmWakeupCallback)(uint32_t sleptDuration);
} PMM_SleepReq_t;

/************************************************************************/
/* Function declarations                                                */
/************************************************************************/

/**
 * \brief This function puts the system to sleep if possible
 *
 * \param[in]  *req  -  pointer to PMM_SleepReq_t request structure
 *
 * \return value of type PMM_Status_t
 *  PMM_SLEEP_REQ_DENIED when sleep is not possible at the instance
 *  PMM_SLEEP_REQ_PROCESSED when sleep is possible and have already done
 */
PMM_Status_t PMM_Sleep(PMM_SleepReq_t *req);

/**
 * \brief Wakeup from sleep
 */
void PMM_Wakeup(void);

#ifdef	__cplusplus
}
#endif

#endif /* CONF_PMM_ENABLE */

#endif /* PMM_H */

/* eof pmm.h */
