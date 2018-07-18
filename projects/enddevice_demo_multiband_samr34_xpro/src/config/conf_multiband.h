/**
 * \file
 *
 * \brief LORAWAN Regional Parameter Configuration
 *
 * Copyright (C) 2016 Atmel Corporation. All rights reserved.
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
 
#ifndef CONF_MULTIBAND_H_INCLUDED
#define CONF_MULTIBAND_H_INCLUDED

/*****************************************************************************/
/* LoRaWAN Regional configuration parameters                                */
/*****************************************************************************/

#if (!(NA_BAND == 1)  && !(AS_BAND == 1) && !(AU_BAND == 1) && !(EU_BAND == 1) && !(IND_BAND == 1) && !(JPN_BAND == 1) && !(KR_BAND == 1) )
#error "Error: Atleast one regional band should be enabled."
#endif

#if (NA_BAND == 1)

#define MAC_DEF_TX_POWER_NA					(7)
#define MAC_DEF_TX_CURRENT_DATARATE_NA		(DR0)
#define MAC_DATARATE_MIN_NA					(DR4)
#define MAC_DATARATE_MAX_NA					(DR0)

#endif

#if (AS_BAND == 1)

#define MAC_DEF_TX_POWER_AS                                (1)
#define MAC_DEF_TX_CURRENT_DATARATE_AS                     (DR0)
#define MAC_DATARATE_MIN_AS                            (DR7)
#define MAC_DATARATE_MAX_AS                            (DR0)

#endif

#if (AU_BAND == 1)

#define MAC_DEF_TX_POWER_AU                                (7)
#define MAC_DEF_TX_CURRENT_DATARATE_AU                     (DR0)
#define MAC_DATARATE_MIN_AU                            (DR6)
#define MAC_DATARATE_MAX_AU                            (DR0)

#endif

#if (EU_BAND == 1)

#define MAC_DEF_TX_POWER_EU					(1)
#define MAC_DEF_TX_CURRENT_DATARATE_EU		(DR0)
#define MAC_DATARATE_MIN_EU					(DR7)
#define MAC_DATARATE_MAX_EU					(DR0)

#endif

#if (IND_BAND == 1)

#define MAC_DEF_TX_POWER_IN                                (1)
#define MAC_DEF_TX_CURRENT_DATARATE_IN                     (DR0)
#define MAC_DATARATE_MIN_IN                                (DR7)
#define MAC_DATARATE_MAX_IN                                (DR0)

#endif

#if (JPN_BAND == 1)

#define MAC_DEF_TX_POWER_JP                               (1)
#define MAC_DEF_TX_CURRENT_DATARATE_JP                     (DR0)
#define MAC_DATARATE_MIN_JP                            (DR7)
#define MAC_DATARATE_MAX_JP                            (DR0)

#define LBT_RSSI_SAMPLES_COUNT_JP						(5)

#endif

#if (KR_BAND == 1)

#define MAC_DEF_TX_POWER_KR                                (1)
#define MAC_DEF_TX_CURRENT_DATARATE_KR                     (DR0)
#define MAC_DATARATE_MIN_KR                            (DR5)
#define MAC_DATARATE_MAX_KR                            (DR0)

#define LBT_RSSI_SAMPLES_COUNT_KR						(10)

#endif

#if (KR_BAND == 1 || JPN_BAND == 1)

/*Number of Scan retries*/
#define LBT_MAX_RETRY_CHANNELS						(5)

#endif

#endif /* CONF_MULTIBAND_H_INCLUDED */

/* eof conf_multiband.h */