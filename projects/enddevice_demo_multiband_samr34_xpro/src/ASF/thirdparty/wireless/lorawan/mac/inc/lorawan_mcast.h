/********************************************************************
 * Copyright (C) 2017 Microchip Technology Inc. and its subsidiaries
 * (Microchip).  All rights reserved.
 *
 * You are permitted to use the software and its derivatives with Microchip
 * products. See the license agreement accompanying this software, if any, for
 * more info about your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 * MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP, SMSC, OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH
 * OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY FOR ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT OR CONSEQUENTIAL DAMAGES, OR OTHER SIMILAR COSTS. To the fullest
 * extend allowed by law, Microchip and its licensors liability will not exceed
 * the amount of fees, if any, that you paid directly to Microchip to use this
 * software.
 *************************************************************************
 *
 *                           lorawan_mcast.h
 *
 * LoRaWAN header file for multicast data & control definition(classB & classC)
 *
 ******************************************************************************/

#ifndef _LORAWAN_MCAST_H_
#define _LORAWAN_MCAST_H_

/***************************** TYPEDEFS ***************************************/
#define LORAWAN_MCAST_GROUP_COUNT_SUPPORTED         1

/*************************** FUNCTIONS PROTOTYPE ******************************/

/*********************************************************************//**
\brief	Multicast - initialization of variables and states

\return					- none.
*************************************************************************/
void LorawanMcastInit(void);

/*********************************************************************//**
\brief	Check if the incoming packet is a multicast group the device
        supports
\param[in]  hdr - mac header of the received frame
\param[in]  mType - message type of the received frame
\param[in]  fPort - frame port of the received frame
\return	    LORAWAN_SUCCESS, if successfully validated
            LORAWAN_INVALID_PARAMETER, otherwise
*************************************************************************/
StackRetStatus_t LorawanMcastValidateHdr(Hdr_t *hdr, uint8_t mType, uint8_t fPort);

/*********************************************************************//**
\brief	Decrypt the packet and provide callback to application
\param[in,out]  buffer  - placeholder for input and output data
\param[in]      bufferLength - length of the received packet
\param[in]      hdr - mac header of the received packet
\return	LORAWAN_SUCCESS, if successfully processed
        LORAWAN_INVALID_PARAMETER, otherwise
*************************************************************************/
StackRetStatus_t LorawanMcastProcessPkt(uint8_t* buffer, uint8_t bufferLength, Hdr_t *hdr);

/*********************************************************************//**
\brief	Multicast enable/disable configuration function
\param[in]  enable - notifies whether to enable or disable multicast
\return	LORAWAN_SUCCESS, if successfully enabled
        LORAWAN_INVALID_PARAMETER, otherwise
*************************************************************************/
StackRetStatus_t LorawanMcastEnable(bool enable);
#endif // _LORAWAN_MCAST_H_

//eof lorawan_mcast.h
