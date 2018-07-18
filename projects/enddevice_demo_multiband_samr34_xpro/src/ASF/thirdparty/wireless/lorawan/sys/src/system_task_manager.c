/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    system_task_manager.c

  @Summary:
    This is the implementation of LoRaWAN system task manager

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
 *                           system_task_manager.c
 * Task manager implementation
 *
 ******************************************************************************/
/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "system_init.h"
#include "atomic.h"
#include "system_task_manager.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/
#define SYSTEM_TASK_ID_COUNT 5u

/************************************************************************/
/* Externals                                                            */
/************************************************************************/
//! This function is called to process RADIO task. SHOULD be defined in RADIO.
extern SYSTEM_TaskStatus_t RADIO_TaskHandler(void);

//! This function is called to process LORAWAN task. SHOULD be defined in LORAWAN.
extern SYSTEM_TaskStatus_t LORAWAN_TaskHandler(void);

//! This function is called to process system timer task. SHOULD be defined in TIMER.
extern SYSTEM_TaskStatus_t TIMER_TaskHandler(void);

//! This function is called to process pds task. SHOULD be defined in PDS.
extern SYSTEM_TaskStatus_t PDS_TaskHandler(void);

//! This function is called to process APP task. SHOULD be defined in APP.
extern SYSTEM_TaskStatus_t APP_TaskHandler(void);

/************************************************************************/
/*  Static variables                                                    */
/************************************************************************/
static SYSTEM_TaskStatus_t (*taskHandlers[SYSTEM_TASK_ID_COUNT])(void) ={
  /* In the order of descending priority */
	TIMER_TaskHandler,
	RADIO_TaskHandler,
	LORAWAN_TaskHandler,
	APP_TaskHandler,
	PDS_TaskHandler
};

static volatile uint16_t sysTaskFlag = 0u;

/************************************************************************/
/* Implementations                                                      */
/************************************************************************/
/*********************************************************************//**
\brief System tasks execution entry point
*************************************************************************/
void SYSTEM_RunTasks(void)
{
	if ((1 << SYSTEM_TASK_ID_COUNT) > sysTaskFlag)
	{ /* Only valid task bits are set */
		while (sysTaskFlag)
		{ /* One or more task are pending to execute */
			for (uint16_t taskId = 0; taskId < SYSTEM_TASK_ID_COUNT; taskId++)
			{
				if ((1 << taskId) & sysTaskFlag)
				{
					/*
					* Reset the task bit since it is to be executed now.
					* It is done inside atomic section to avoid any interrupt context
					* corrupting the bits.
					*/
					ATOMIC_SECTION_ENTER
					sysTaskFlag &= ~(1 << taskId);
					ATOMIC_SECTION_EXIT

					/* Return value is not used now, can be used later */
					taskHandlers[taskId]();

					/* Break here so that higher priority task executes next, if any */
					break;
				}
			}
		}
	}
	else
	{
		/* 
		* Invalid task bits are set i.e., bits other than 4 LSB are set
		* Can happen only due to corruption, so halt
		* TODO : replace this with assert implementation
		*/
		while(1);
	}
}

/*********************************************************************//**
\brief Posts a task to the task manager, which is later processed by the
       task handler of the corresponding stack layer. A task is processed
       when the SYSTEM_RunTasks() function. \n

       IDs of the tasks are listed in the SYSTEM_Task_t enum. Each task has its
       own priority and is called only if there is no any task with higher
       priority. \n

       A handler is called when respective task can be run. Each task has its
       own task handler. \n

       Correspondence between tasks and handlers is listed below:  \n
       RADIO RADIO_TaskHandler() \n
       HAL - HAL_TaskHandler() \n
       LORAWAN - LORAWAN_TaskHandler() \n
       APP - APP_TaskHandler() \n

\param[in] task - ID of the posted task.
*************************************************************************/

void SYSTEM_PostTask(SYSTEM_Task_t task)
{
	ATOMIC_SECTION_ENTER
	sysTaskFlag |= task;
	ATOMIC_SECTION_EXIT
}

/*********************************************************************//**
\brief Returns the readiness of the system for sleep

\return 'true' if the system is ready, 'false' otherwise
*************************************************************************/
bool SYSTEM_ReadyToSleep(void)
{
	return !(sysTaskFlag & 0xffff);
}

/* eof system_task_manager.c */
