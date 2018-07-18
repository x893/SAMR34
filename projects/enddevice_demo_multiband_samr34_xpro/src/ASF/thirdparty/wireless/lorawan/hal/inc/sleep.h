/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    sleep.h

  @Summary:
    This is the hardware interface for sleep

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
 *                           sleep.h
 * Hardware sleep interface
 *
 ******************************************************************************/

#ifndef SLEEP_H
#define SLEEP_H

/************************************************************************/
/* Types                                                                */
/************************************************************************/
/* Possible sleep modes */
typedef enum _HAL_SleepMode_t
{
#if defined(SAMR34) || defined(SAMR30)
  /* HAL Sleep modes for SAMR34, SAML21-compatible MCUs */
  SLEEP_MODE_IDLE    = 0,
  SLEEP_MODE_STANDBY,
  SLEEP_MODE_BACKUP,
  SLEEP_MODE_OFF
#else
  SLEEP_MODE_NONE
#endif
} HAL_SleepMode_t;

/************************************************************************/
/* Function declarations                                                */
/************************************************************************/

/**
 * \brief Puts the system in given sleep mode
 *
 * \param[in] mode - sleep mode
 */
void HAL_Sleep(HAL_SleepMode_t mode);

#endif /* SLEEP_H */

/* eof sleep.h */
