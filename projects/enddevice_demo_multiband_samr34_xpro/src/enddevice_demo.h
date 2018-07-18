/**
 * \file
 *
 * \brief LORAWAN Demo Application
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

#ifndef DEMO_APP_H_
#define DEMO_APP_H_

/*********************************************************************//**
 \brief      Macro to check the LoRaWAN stack status attribute
			 network joined bit 
*************************************************************************/
#define LORAWAN_NW_JOINED  0x01

/*********************************************************************//**
 \brief      Function to Initialize the Demo application 
*************************************************************************/
void mote_demo_init(void);

/*********************************************************************//*
 \brief      Set join parameters function
 \param[in]  activation type - notifies the activation type (OTAA/ABP)
 \return	 LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t set_join_parameters(ActivationType_t activation_type);

/*********************************************************************//*
 \brief      Function to Initialize the device type
 \param[in]  ed_class - notifies the device class (CLASS_A/CLASS_C)
 \return	 LORAWAN_SUCCESS, if successfully set the device class
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t set_device_type(EdClass_t ed_class);

/***********************************************************************
 \brief      Function to Initialize set default parameters
 \param[in]  void 
 \return	 LORAWAN_SUCCESS, if successfully set all the parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t mote_set_parameters(IsmBand_t ismBand);

/*********************************************************************//*
 \brief      Function to Initialize the Multicast parameters
 ************************************************************************/
void set_multicast_params (void);

/*********************************************************************//**
 \brief      Callback function for the ending of Bidirectional communication of 
       Application data 
 *************************************************************************/
void demo_appdata_callback(void *appHandle, appCbParams_t *data);

/*********************************************************************//*
 \brief      Callback function for the ending of Activation procedure 
 ************************************************************************/
void demo_joindata_callback(bool status);

/*********************************************************************//*
 \brief      Function to Print stack default parameters
 ************************************************************************/
void print_default_parameters(void);

/*********************************************************************//*
 \brief      Function to Print application configuration
 ************************************************************************/
void  print_application_config (void);

/*********************************************************************//*
 \brief      Function to Print array of characters
 \param[in]  *array  - Pointer of the array to be printed
 \param[in]   length - Length of the array
 ************************************************************************/
void print_array (uint8_t *data, uint8_t length);

/*********************************************************************//*
 \brief      Function to Print stack return status
 \param[in]  status - Status from the stack
 ************************************************************************/
void print_stack_status(StackRetStatus_t status);

/*********************************************************************//*
 \brief      Function to Check stack return status
 \param[in]  Void 
 ************************************************************************/
void check_stack_status(void);

#endif /* DEMO_APP_H_ */