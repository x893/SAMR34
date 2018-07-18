/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    radio_task_manager.h

  @Summary:
    This is the Radio Driver Task Manager header file which contains Radio task
	scheduler of the Radio Driver

  @Description:
    This header file provides LoRa-specific implementations for Radio Driver HAL APIs.
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
 *                           radio_task_manager.h
 *
 * Radio Driver Task Manager header file
 *
 ******************************************************************************/

#ifndef _RADIO_DRIVER_TASKMANAGER_H
#define _RADIO_DRIVER_TASKMANAGER_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include "system_task_manager.h"

/******************************************************************************
                   Defines section
******************************************************************************/
#define RADIO_TASKS_COUNT               5u

/******************************************************************************
                               Types section
*******************************************************************************/
typedef enum
{
  RADIO_TX_DONE_TASK_ID = (1 << 0),
  RADIO_RX_DONE_TASK_ID = (1 << 1),
  RADIO_TX_TASK_ID      = (1 << 2),
  RADIO_RX_TASK_ID      = (1 << 3),
  RADIO_SCAN_TASK_ID = (1 << 4),
  RADIO_SLEEP_TASK_ID = (1 << 5)
} RadioTaskIds_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
extern SYSTEM_TaskStatus_t RADIO_TxHandler(void);
extern SYSTEM_TaskStatus_t RADIO_RxHandler(void);
extern SYSTEM_TaskStatus_t RADIO_TxDoneHandler(void);
extern SYSTEM_TaskStatus_t RADIO_RxDoneHandler(void);
extern SYSTEM_TaskStatus_t RADIO_ScanHandler(void);
/* SYSTEM_TaskStatus_t RADIO_SleepHandler(void); */

/**************************************************************************//**
\brief Set task for RADIO task manager.

\param[in] id - a single value from the type RadioTaskIds_t
******************************************************************************/
void radioPostTask(RadioTaskIds_t id);

/**************************************************************************//**
\brief Clear task for RADIO task manager.

\param[in] id - a single value from the type RadioTaskIds_t
******************************************************************************/
void radioClearTask(RadioTaskIds_t id);

#endif  /*_RADIO_DRIVER_TASKMANAGER_H*/

/* eof radio_task_manager.h */
