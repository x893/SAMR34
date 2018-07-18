/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    system_init.c

  @Summary:
    This is the implementation of LoRaWAN system initialization

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
 *                           system_init.c
 * System initialization implementation
 *
 ******************************************************************************/
/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "system_init.h"
#include "system_task_manager.h"

/******************************************************************************/
/* Implementations                                                            */
/******************************************************************************/
/**************************************************************************//**
\brief Initializes the system. This is the first stack function to be called
       by the application in the main() function.
******************************************************************************/
void Stack_Init(void)
{
	/* Write SAML21-related initialization here*/
	
	/* LORAWAN_Init() should have been called form here, 
	   but parser application has a round-about way to call it now */
 
	/* Post application task to initiate the execution from there */
	SYSTEM_PostTask(APP_TASK_ID);
}

/* eof system_init.c */
