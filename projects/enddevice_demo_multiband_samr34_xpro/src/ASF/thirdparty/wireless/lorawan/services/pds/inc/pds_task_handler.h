/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    pds_task_handler.h

  @Summary:
    This is the PDS Driver Task Manager header file which calls PDS task
	scheduler.

  @Description:
    This header file provides LoRa-specific implementations for Pds.
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
 *                           pds_task_handler.h
 *
 * Pds Driver Task Manager header file
 *
 ******************************************************************************/

#ifndef _PDS_DRIVER_TASKMANAGER_H
#define _PDS_DRIVER_TASKMANAGER_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include "system_task_manager.h"

/******************************************************************************
                   Defines section
******************************************************************************/
#define PDS_TASKS_COUNT               1u

/******************************************************************************
                               Types section
*******************************************************************************/
typedef enum
{
  PDS_STORE_DELETE_TASK_ID = (1 << 0)
} PdsTaskIds_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
SYSTEM_TaskStatus_t PDS_TaskHandler(void);

/**************************************************************************//**
\brief Set task for PDS task manager.

\param[in] id - a single value from the type PdsTaskIds_t
******************************************************************************/
void pdsPostTask(PdsTaskIds_t id);

/**************************************************************************//**
\brief Clear task for PDS task manager.

\param[in] id - a single value from the type PdsTaskIds_t
******************************************************************************/
void pdsClearTask(PdsTaskIds_t id);

#endif  /*_PDS_DRIVER_TASKMANAGER_H*/

/* eof pds_task_handler.h */
