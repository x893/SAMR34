/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    system_task_manager.h

  @Summary:
    This is the interface of LoRaWAN system task manager

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
 *                           system_task_manager.h
 * Task manager interface
 *
 ******************************************************************************/

#ifndef SYSTEM_TASK_MANAGER_H
#define SYSTEM_TASK_MANAGER_H
/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include <stdbool.h>
#include <stdint.h>

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/************************************************************************/
/* Types                                                                */
/************************************************************************/

/*! \brief Possible results of task handler execution */
typedef enum _SYSTEM_TaskStatus_t
{
  SYSTEM_TASK_SUCCESS = 0x00,
  SYSTEM_TASK_FAILURE = 0x01
} SYSTEM_TaskStatus_t;

/*! The list of task IDs. The IDs are sorted according to descending
priority. For each task ID there is the corresponding task handler function. */
typedef enum _SYSTEM_Task_t
{
  TIMER_TASK_ID   = 1 << 0,
  RADIO_TASK_ID   = 1 << 1,
  LORAWAN_TASK_ID = 1 << 2,
  APP_TASK_ID     = 1 << 3,
  PDS_TASK_ID     = 1 << 4
} SYSTEM_Task_t;

/************************************************************************/
/* Prototypes                                                           */
/************************************************************************/
/********************************************************************//**
\brief  This function is called by the stack or from the main()

If several tasks have been posted by the moment of the function's call,
they are executed in order of layers' priority: a task of the layer with the
highest priority is executed first.
*************************************************************************/
void SYSTEM_RunTasks(void);

/*********************************************************************//**
\brief Posts a task to the task manager, which is later processed by the
       task handler of the corresponding stack layer. A task is processed
       when the SYSTEM_RunTasks() function.

\param[in] task - ID of the posted task.
*************************************************************************/
/*
IDs of the tasks are listed in the SYSTEM_Task_t enum. Each task has its
own priority and is called only if there is no any task with higher priority.
A handler is called when respective task can be run. Each task has its
own task handler.
Correspondence between tasks and handlers is listed below:  \n
RADIO RADIO_TaskHandler()
HAL - HAL_TaskHandler()
LORAWAN - LORAWAN_TaskHandler()
APP - APP_TaskHandler()
 */
void SYSTEM_PostTask(SYSTEM_Task_t task);

/*********************************************************************//**
\brief Returns the readiness of the system for sleep

\return '1' if the system is ready, '0' otherwise
*************************************************************************/
bool SYSTEM_ReadyToSleep(void);

#endif /* SYSTEM_TASK_MANAGER_H */

/* eof system_task_manager.h */
