/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    radio_driver_SX1276.h

  @Summary:
    This is the Radio Driver SX1276 header file which contains LoRa-specific
    Radio Driver functions declarations and defines for SX1276

  @Description:
    This header file provides LoRa-specific implementations for Radio Driver for SX1276.
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
 *                           radio_driver_SX1276.h
 *
 * Radio Driver SX1276 header file
 *
 ******************************************************************************/

#ifndef RADIO_DRIVER_H
#define	RADIO_DRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif
    
/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "radio_interface.h"
#include <stdint.h>
#include <stddef.h>

/************************************************************************/
/* Defines                                                              */
/************************************************************************/
#define ENABLED						1
#define DISABLED					0
// The Freq greater than 862Mhz is HF frequency 
#define HF_FREQ_KHZ					862000UL
// For freq above 862Mhz the RSSI offset to be used
#ifndef RSSI_HF_OFFSET
#define RSSI_HF_OFFSET				-157
#endif
// For freq below 862Mhz the RSSI offset to be used
#ifndef RSSI_LF_OFFSET
#define RSSI_LF_OFFSET				-164
#endif

/************************************************************************/
/* Types                                                                */
/************************************************************************/

/************************************************************************/
/* External variables                                                   */
/************************************************************************/
extern RadioConfiguration_t radioConfiguration;

/************************************************************************/
/* Prototypes                                                           */
/************************************************************************/
/*********************************************************************//**
\brief This function reads the RSSI value for LoRa and FSK.

\param rssi	- The RSSI measured in the channel.
\return		- ERR_NONE. Other types are not used now.
*************************************************************************/
RadioError_t Radio_ReadRssi(int16_t *rssi);

#ifdef	__cplusplus
}
#endif

#endif	/* RADIO_DRIVER_H */

/**
 End of File
*/