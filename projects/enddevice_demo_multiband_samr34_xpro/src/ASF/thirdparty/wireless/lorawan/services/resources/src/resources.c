/**
 * \file resource.c
 *
 * \brief  
 *
 * Copyright (c) 2017 Atmel Corporation. All rights reserved.
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
 */

/**
 * \page license License
 * Copyright (c) 2017 Atmel Corporation. All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/****************************** INCLUDES **************************************/
#include "asf.h"
#include "resources.h"
#include "temp_sensor.h"

/*************************STATIC FUNCTIONS ************************************/

/****************************** VARIABLES *************************************/

/*************************FUNCTION PROTOTYPES *********************************/

/*********************************************************************//**
 \brief      Function to get different resource data
 \param[in]  resource - Type of resource (Motor, Light, Temperature)
 \param[out] *data    - Pointer to the data from different resource
*************************************************************************/
void get_resource_data(uint8_t resource, uint8_t * data)
{

	switch(resource)
	{
		
		case TEMP_SENSOR:
		{
			get_temp_sensor_data(data);
			break;
		}
		default:
		{
			*data = UNSUPPORTED_RESOURCE;
			break;
		}
	}
}

/*********************************************************************//**
 \brief      Function to initialize different resources
*************************************************************************/
void resource_init(void)
{
	temp_sensor_init();
}
/*---------------------------------------------------------------------------*/
