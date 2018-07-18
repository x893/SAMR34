/**
 * \file
 *
 * \brief Application Configuration
 *
 * Copyright (C) 2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/****************************** INCLUDES **************************************/

/****************************** MACROS **************************************/

/* Stack version string */
#define STACK_VER  "MS4_E_0"

/*Supported Bands*/
#define  EU868  	0
#define  EU433  	1
#define  NA915  	2
#define  AU915  	3
#define  KR920  	4
#define	 JPN920		5
#define  BRN923 	6
#define	 CMB923		7
#define	 INS923		8
#define	 LAOS923	9
#define	 NZ923		10
#define	 SP923		11
#define	 TWN923		12
#define	 THAI923	13
#define	 VTM923		14
#define	 IND865		15

/*ISM Band Supported*/
/*#if defined (EU_BAND)
#define ISM_BAND EU868
#elif defined(NA_BAND)
#define ISM_BAND NA915
#elif defined(AU_BAND)
#define ISM_BAND AU915
#elif defined(KR_BAND)
#define ISM_BAND KR920
#elif defined(AS_BAND)
//TODO Change the Band if required
#define ISM_BAND THAI923
#elif defined(JPN_BAND)
#define ISM_BAND JPN920
#elif defined(IND_BAND)
#define ISM_BAND IND865
#endif*/

/* Number of software timers */
#define TOTAL_NUMBER_OF_TIMERS            (25u)


/*Define the Sub band of Channels to be enabled by default for the application*/
#define SUBBAND 1
#if ((SUBBAND < 1 ) || (SUBBAND > 8 ) )
#error " Invalid Value of Subband"
#endif

/* Activation method constants */
#define OVER_THE_AIR_ACTIVATION           LORAWAN_OTAA
#define ACTIVATION_BY_PERSONALIZATION     LORAWAN_ABP

/* Message Type constants */
#define UNCONFIRMED                       LORAWAN_UNCNF
#define CONFIRMED                         LORAWAN_CNF

/* Enable one of the activation methods */
#define APP_ACTIVATION_TYPE               OVER_THE_AIR_ACTIVATION 
//#define APP_ACTIVATION_TYPE                ACTIVATION_BY_PERSONALIZATION

/* Select the Type of Transmission - Confirmed(CNF) / Unconfirmed(UNCNF) */
#define APP_TRANSMISSION_TYPE              UNCONFIRMED
//#define APP_TRANSMISSION_TYPE            CONFIRMED

/* FPORT Value (1-255) */
#define APP_FPORT                           1

/* Device Class - Class of the device (CLASS_A/CLASS_C) */
#define APP_ENDDEVICE_CLASS                 CLASS_A
//#define APP_ENDDEVICE_CLASS                 CLASS_C


/* ABP Join Parameters */

#define DEVICE_ADDRESS                     0xdeafface
#define APPLICATION_SESSION_KEY            {0x41, 0x63, 0x74, 0x69, 0x6C, 0x69, 0x74, 0x79, 0x00, 0x04, 0xA3, 0x0B, 0x00, 0x04, 0xA3, 0x0B}
#define NETWORK_SESSION_KEY                {0x61, 0x63, 0x74, 0x69, 0x6C, 0x69, 0x74, 0x79, 0x00, 0x04, 0xA3, 0x0B, 0x00, 0x04, 0xA3, 0x0B}


/* OTAA Join Parameters */

#define DEVICE_EUI                         {0xde, 0xaf, 0xfa, 0xce, 0xde, 0xaf, 0xfa, 0xce}
#define APPLICATION_EUI                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05}
#define APPLICATION_KEY                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05}

/* Multicast Parameters */
#define APP_MCAST_ENABLE                   true
#define APP_MCAST_GROUP_ADDRESS            0x0037CC56
#define APP_MCAST_APP_SESSION_KEY          {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6}
#define APP_MCAST_NWK_SESSION_KEY          {0x3C, 0x8F, 0x26, 0x27, 0x39, 0xBF, 0xE3, 0xB7, 0xBC, 0x08, 0x26, 0x99, 0x1A, 0xD0, 0x50, 0x4D}

/* This macro defines the application's default sleep duration in milliseconds */
#define CONF_DEFAULT_APP_SLEEP_TIME_MS     1000

#endif /* APP_CONFIG_H_ */
