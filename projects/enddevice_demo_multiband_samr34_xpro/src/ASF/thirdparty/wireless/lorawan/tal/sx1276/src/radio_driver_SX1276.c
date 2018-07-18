/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    radio_driver_SX1276.c

  @Summary:
    This is the Radio Driver SX1276 source file which contains LoRa-specific
    Radio Driver functions declarations and defines for SX1276

  @Description:
    This source file provides LoRa-specific implementations for Radio Driver for SX1276.
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
 *                           radio_driver_SX1276.c
 *
 * Radio Driver SX1276 source file
 *
 ******************************************************************************/

/************************************************************************/
/*  Includes                                                            */
/************************************************************************/
#ifdef UT
#include "lora_test_main.h"
#endif
#include "radio_interface.h"
#include "radio_registers_SX1276.h"
#include "radio_driver_SX1276.h"
#include "radio_driver_hal.h"
#include "radio_get_set.h"
#include "radio_transaction.h"
#include "sw_timer.h"
#include "sys.h"
#include "stdint.h"
#include "string.h"

/************************************************************************/
/*  Global variables                                                    */
/************************************************************************/
RadioConfiguration_t radioConfiguration;
#ifdef UT
uint16_t testRssi;
#endif // UT

/************************************************************************/
/*  external variables                                                    */
/************************************************************************/
extern volatile RadioCallbackMask_t radioCallbackMask;

/************************************************************************/
/*  Static functions                                                    */
/************************************************************************/

/************************************************************************/
/* Implementations                                                      */
/************************************************************************/

/*********************************************************************//**
\brief	This function sets the radioMode and modulation in order to set
		the transceiver to transmit and receive at a set modulation. 

\param newMode			- Sets the transceiver mode.
\param newModulation	- Sets the modulation.
\param blocking			- Sets if its blocking call or not.
\return					- none.
*************************************************************************/
void Radio_WriteMode(RadioMode_t newMode, RadioModulation_t newModulation, uint8_t blocking)
{
#ifndef UT
    uint8_t opMode;
    uint8_t dioMapping;
    RadioModulation_t currentModulation;
    RadioMode_t currentMode;

    if ((MODULATION_FSK == newModulation) &&
        ((MODE_RXSINGLE == newMode) || (MODE_CAD == newMode)))
    {
        // Unavailable modes for FSK. Just return.
        return;
    }

    // Sanity enforcement on parameters
    newMode &= 0x07;
    newModulation &= 0x01;

    opMode = RADIO_RegisterRead(REG_OPMODE);

    if ((opMode & 0x80) != 0)
    {
        currentModulation = MODULATION_LORA;
    }
    else
    {
        currentModulation = MODULATION_FSK;
    }

    currentMode = opMode & 0x07;

    // If we need to change modulation, we need to do this in sleep mode.
    // Otherwise, we can go straight to changing the current mode to newMode.
    if (newModulation != currentModulation)
    {
        // Go to sleep
        if (MODE_SLEEP != currentMode)
        {
            // Clear mode bits, effectively going to sleep
            RADIO_RegisterWrite(REG_OPMODE, opMode & (~0x07));
            currentMode = MODE_SLEEP;
        }
        // Change modulation
        if (MODULATION_FSK == newModulation)
        {
            // Clear MSB and sleep bits to make it stay in sleep
            opMode = opMode & (~0x87);
        }
        else
        {
            // LoRa mode. Set MSB and clear sleep bits to make it stay in sleep
            opMode = 0x80 | (opMode & (~0x87));
        }
        RADIO_RegisterWrite(REG_OPMODE, opMode);
    }

    // From here on currentModulation is no longer current, we will use
    // newModulation instead as it reflects the chip configuration.
    // opMode reflects the actual configuration of the chip.

    if (newMode != currentMode)
    {
        // If we need to block until the mode switch is ready, configure the
        // DIO5 pin to relay this information.
        if ((MODE_SLEEP != newMode) && (1 == blocking))
        {
            dioMapping = RADIO_RegisterRead(REG_DIOMAPPING2);
            if (MODULATION_FSK == newModulation)
            {
                // FSK mode
                dioMapping |= 0x30;     // DIO5 = 11 means ModeReady in FSK mode
            }
            else
            {
                // LoRa mode
                dioMapping &= ~0x30;    // DIO5 = 00 means ModeReady in LoRa mode
            }
            RADIO_RegisterWrite(REG_DIOMAPPING2, dioMapping);
        }

        // Do the actual mode switch.
        opMode &= ~0x07;                // Clear old mode bits
        opMode |= newMode;              // Set new mode bits
        RADIO_RegisterWrite(REG_OPMODE, opMode);

        // If required and possible, wait for switch to complete
        if (1 == blocking)
        {
            if (MODE_SLEEP != newMode)
            {
               // while (HAL_DIO5PinValue() == 0);
			   /* Wait for Mode change to happen */
			   delay_ms(1);
            }
            else
            {
                SystemBlockingWaitMs(1);
            }
        }
    }
#endif
}

/*********************************************************************//**
\brief	This function sets the receive frequency of the transceiver
		while hopping in FHSS.

\param		- none	
\return		- none.
*************************************************************************/
void RADIO_FHSSChangeChannel(void)
{
    uint32_t freq;
    RADIO_RegisterRead(REG_LORA_IRQFLAGS);

    if (radioConfiguration.frequencyHopPeriod)
    {
        if ((radioConfiguration.radioCallback) &&
            (1 == radioCallbackMask.BitMask.radioFhssfreqCallback))
        {
            radioConfiguration.radioCallback(RADIO_FHSS_NEXT_FREQ_CALLBACK, (void *)&freq);
            Radio_WriteFrequency(freq);
        }
    }

    // Clear FHSSChangeChannel interrupt
    RADIO_RegisterWrite(REG_LORA_IRQFLAGS, 1 << SHIFT1);
}

/*********************************************************************//**
\brief	This function generates random number using the transceiver
		and returns it.

\param		- none	
\return		- returns the random number generated.
*************************************************************************/
uint16_t RADIO_ReadRandom(void)
{
    uint8_t i;
    uint16_t retVal;
    retVal = 0;
	
	// Turn on the RF switch.
	HALEnableRFSwitch();
	
    // Mask all interrupts, do many measurements of RSSI
    Radio_WriteMode(MODE_SLEEP, MODULATION_LORA, 1);
    RADIO_RegisterWrite(REG_LORA_IRQFLAGSMASK, 0xFF);
    Radio_WriteMode(MODE_RXCONT, MODULATION_LORA, 1);
    for (i = 0; i < 16; i++)
    {
        SystemBlockingWaitMs(1);
        retVal <<= SHIFT1;
        retVal |= RADIO_RegisterRead(REG_LORA_RSSIWIDEBAND) & 0x01;
    }
	
	// Turning off the RF switch now.
	HALDisableRFSwitch();
	
    // Return radio to sleep
    Radio_WriteMode(MODE_SLEEP, MODULATION_LORA, 1);
    // Clear interrupts in case any have been generated
    RADIO_RegisterWrite(REG_LORA_IRQFLAGS, 0xFF);
    // Unmask all interrupts
    RADIO_RegisterWrite(REG_LORA_IRQFLAGSMASK, 0x00);
    return retVal;
}

/*********************************************************************//**
\brief This function reads the RSSI value for LoRa and FSK.

\param rssi	- The RSSI measured in the channel.
\return		- ERR_NONE. Other types are not used now.
*************************************************************************/
RadioError_t Radio_ReadRssi(int16_t *rssi)
{
	if (MODULATION_LORA == radioConfiguration.modulation)
	{
		if (radioConfiguration.frequency >= HF_FREQ_KHZ)
		{
#ifdef UT
			*rssi = testRssi;
#else // UT
			*rssi = RSSI_HF_OFFSET + RADIO_RegisterRead(REG_LORA_RSSIVALUE);
#endif // UT		
		}
		else
		{
#ifdef UT
			*rssi = testRssi;
#else // UT
			*rssi = RSSI_LF_OFFSET + RADIO_RegisterRead(REG_LORA_RSSIVALUE);
#endif // UT
		}
	}
	else if (MODULATION_FSK == radioConfiguration.modulation)
	{
#ifdef UT
		*rssi = testRssi;
#else // UT
		*rssi = -(RADIO_RegisterRead(REG_FSK_RSSIVALUE) / 2);
#endif // UT
	}
	else
	{
		return ERR_UNSUPPORTED_ATTR;
	}

	return ERR_NONE;
}

/**
 End of File
 */
