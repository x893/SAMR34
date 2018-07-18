/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    lorawan_defs.h

  @Summary:
    This is the LoRaWAN utility definitions file 

  @Description:
    This header file provides LoRaWAN required initialization APIs declarations.
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
 *                           lorawan_defs.h
 * 
 * LoRaWAN utility definitions header file
 *
 ******************************************************************************/

#ifndef MCC_LORA_DEFS_H
#define	MCC_LORA_DEFS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define DEF_CNF_UL_REPT_CNT							(0)

#define DEF_UNCNF_UL_REPT_CNT						(0)

#define MAC_CONFIRMABLE_UPLINK_REPITITIONS_MAX      (7)

#define MAC_UNCONFIRMABLE_UPLINK_REPITITIONS_MAX    (0)

#define DEF_MACSTATUS								(0)

#define MAC_LINK_CHECK_GATEWAY_COUNT				(0)

#define MAC_LINK_CHECK_MARGIN						(255)

#define MAC_LORA_MODULATION_SYNCWORD				(0x34)

#define MAC_AGGREGATED_DUTYCYCLE					(1)

#define MAC_DEVNONCE								(0)

#define TRANSMISSION_ERROR_TIMEOUT					(2000UL)

#define CLASS_C_RX2_WINDOW_SIZE						(0)

#define LORAWAN_SESSIONKEY_LENGTH					(16)

#define MAX_FCNT_PDS_UPDATE_VALUE               (1) // Keep this as power of 2. Easy for bit manipulation.

#ifdef	__cplusplus
}
#endif

#endif	/* MCC_LORA_DEFS_H */

/**
 End of File
*/