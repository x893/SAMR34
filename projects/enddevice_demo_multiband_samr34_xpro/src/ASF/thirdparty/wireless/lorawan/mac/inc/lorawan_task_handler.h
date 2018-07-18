/******************************************************************************
 * Copyright (C) 2016 Microchip Technology Inc. and its subsidiaries
 * (Microchip).  All rights reserved.
 *
 * You are permitted to use the software and its derivatives with Microchip
 * products. See the license agreement accompanying this software, if any, for
 * more info about your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 * MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP, SMSC, OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH
 * OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY FOR ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT OR CONSEQUENTIAL DAMAGES, OR OTHER SIMILAR COSTS. To the fullest
 * extend allowed by law, Microchip and its licensors liability will not exceed
 * the amount of fees, if any, that you paid directly to Microchip to use this
 * software.
 ******************************************************************************
 *
 *                           lorawan_task_handler.h
 *
 * LoRaWAN Task Handler header file
 *
 ******************************************************************************/
#ifndef LORAWAN_TASK_HANDLER_H_
#define LORAWAN_TASK_HANDLER_H_

/******************************************************************************
                   Includes section
******************************************************************************/
#include <system_task_manager.h>

/******************************************************************************
                              Types section
 ******************************************************************************/
/** External req identifiers. */
typedef enum
{	
	LORAWAN_JOIN_TASK_ID	= 0u,
    LORAWAN_TX_TASK_ID		= 1u,
    LORAWAN_RX_TASK_ID		= 2u
}lorawanTaskID_t;

/******************************************************************************
                             Constants section
 ******************************************************************************/
#define LORAWAN_TASKS_SIZE         3u

/*************************** FUNCTIONS PROTOTYPE ******************************/

/** LORAWAN Subtask Handlers*/
SYSTEM_TaskStatus_t LORAWAN_TxHandler(void);

SYSTEM_TaskStatus_t LORAWAN_JoinReqHandler(void);

SYSTEM_TaskStatus_t LORAWAN_RxHandler(void);

/** Lorawan post task - Post a task to Lorawan Handler*/
void LORAWAN_PostTask(const lorawanTaskID_t taskID);

/** helper function for setting up radio for transmission */
void ConfigureRadioTx(radioConfig_t radioConfig);

#endif /* LORAWAN_TASK_HANDLER_H_ */
