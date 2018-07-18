/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    radio_get_set.h

  @Summary:
    This is the radio_get_set header file which contains interfaces for
	Getting and Setting the Radio and Transciver parameters.

  @Description:
    This header file provides interfaces for Getting and Setting the
	Radio and Transciver properties.
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
 *                           radio_get_set.h
 *
 * Radio Get Set header file
 *
 ******************************************************************************/
 
#ifndef _RADIO_GET_SET_H
#define _RADIO_GET_SET_H

 
/************************************************************************/
/*  Includes                                                            */
/************************************************************************/
#include "lorawan_defs.h"

/************************************************************************/
/*  Includes                                                            */
/************************************************************************/

#define RADIO_DEFAULT_FREQ DEFAULT_CALIBRATION_FREQ

/************************************************************************/
/*  Global variables                                                    */
/************************************************************************/

/************************************************************************/
/*  Function prototypes                                                 */
/************************************************************************/

/*********************************************************************//**
\brief	This function sets the transmit frequency. 

\param frequency	- Sets the transmit radio frequency.
\return				- none.
*************************************************************************/
void Radio_WriteFrequency(uint32_t frequency);

/*********************************************************************//**
\brief	This function prepares the transceiver for transmit and receive
		according to modulation set.

\param symbolTimeout	- Sets the symbolTimeout parameter.
\return					- none.
*************************************************************************/
void Radio_WriteConfiguration(uint16_t symbolTimeout);

/*********************************************************************//**
\brief	This function sets the radioMode and modulation in order to set
		the transceiver to transmit and receive at a set modulation. 

\param newMode			- Sets the transceiver mode.
\param newModulation	- Sets the modulation.
\param blocking			- Sets if its blocking call or not.
\return					- none.
*************************************************************************/
void Radio_WriteMode(RadioMode_t newMode, RadioModulation_t newModulation, uint8_t blocking);

#endif  /*_RADIO_GET_SET_H*/

// eof radio_get_set.h