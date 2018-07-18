/**
 * \file
 *
 * \brief LORAWAN Demo Application
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
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
/****************************** INCLUDES **************************************/
#include "asf.h"
#include "lorawan.h"
#include "system_task_manager.h"
#include "enddevice_demo.h"
#include "conf_app.h"
#include "sio2host.h"
#include "resources.h"
#include "delay.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif
/******************************** MACROS ***************************************/

/************************** GLOBAL VARIABLES ***********************************/
bool joined = false;
float cel_val;
float fahren_val;
char temp_sen_str[25];
uint8_t data_len = 0;

static const char* bandStrings[] =
{
	"Continue current band",
	"Factory Reset Board",
#if (EU_BAND == 1)
	"EU868",
#endif
#if (NA_BAND == 1)
	"NA915",
#endif
#if (AU_BAND == 1)
	"AU915",
#endif
#if (AS_BAND == 1)
	"Thai923",
#endif
#if (JPN_BAND == 1)
	"Jpn923",
#endif
#if (KR_BAND == 1)
	"Kr920",
#endif
#if (IND_BAND == 1)
	"Ind865"
#endif
};

uint8_t bandTable[] = 
{
	0xFF,
	0xFF,
#if (EU_BAND == 1)
	ISM_EU868,
#endif
#if (NA_BAND == 1)
	ISM_NA915,
#endif
#if (AU_BAND == 1)
	ISM_AU915,
#endif
#if (AS_BAND == 1)
	ISM_THAI923,
#endif
#if (JPN_BAND == 1)
	ISM_JPN923,
#endif
#if (KR_BAND == 1)
	ISM_KR920,
#endif
#if (IND_BAND == 1)
	ISM_IND865
#endif
};

LorawanSendReq_t lorawanSendReq;

/*ABP Join Parameters */
uint32_t devAddr = DEVICE_ADDRESS;
uint8_t nwksKey[16] = NETWORK_SESSION_KEY;
uint8_t appsKey[16] = APPLICATION_SESSION_KEY;

/* OTAA join parameters */
uint8_t devEui[8] = DEVICE_EUI;
uint8_t appEui[8] = APPLICATION_EUI;
uint8_t appKey[16] = APPLICATION_KEY;

/* Muticast Parameters */
bool mcastEnable = APP_MCAST_ENABLE;
uint32_t mcastDevAddr = APP_MCAST_GROUP_ADDRESS;
uint8_t mcastNwksKey[16] = APP_MCAST_NWK_SESSION_KEY;
uint8_t mcastAppsKey[16] = APP_MCAST_APP_SESSION_KEY;
/************************** EXTERN VARIABLES ***********************************/
extern bool button_pressed;
extern bool factory_reset;
extern bool bandSelected;
extern uint32_t longPress;

/************************** FUNCTION PROTOTYPES ********************************/
SYSTEM_TaskStatus_t APP_TaskHandler(void);
static float convert_celsius_to_fahrenheit(float cel_val);

/***************************** FUNCTIONS ***************************************/

/*********************************************************************//**
\brief	Initialization the Demo application 
*************************************************************************/
void mote_demo_init(void)
{
	//StackRetStatus_t status = LORAWAN_SUCCESS;
	
	/* Initialize the resources */
	resource_init();
	
	LORAWAN_Init(demo_appdata_callback, demo_joindata_callback/*,ISM_BAND*/);
			
	/* Initialize the LORAWAN Stack */

	printf("\n\n\r*******************************************************\n\r");
	printf("\n\rMicrochip LoRaWAN Stack %s\r\n",STACK_VER);
	printf("\r\nInit - Successful\r\n");

#if (ENABLE_PDS == 1)
	printf("\n\r*********************PDS*******************************\n\r");
	printf("\r\nPDS Restoration Triggered....\r\n");
	if(PDS_IsRestorable())
	{
		PDS_RestoreAll();
		printf("\r\nData recovery from PDS is successful\r\n");
		check_stack_status();
	}
	else
#endif
	{
#if (ENABLE_PDS == 1)			
		printf("\r\nNo Data available in PDS. Proceeding with Normal procedure\r\n");
#endif			

	}

	if(!joined)
	{
		button_pressed = true;
		SYSTEM_PostTask(APP_TASK_ID);
	}

}


/*********************************************************************//**
\brief Callback function for the ending of Bidirectional communication of 
       Application data 
 *************************************************************************/
void demo_appdata_callback(void *appHandle, appCbParams_t *appdata)
{
	
	if (LORAWAN_EVT_RX_DATA_AVAILABLE == appdata->evt)
	{
		uint8_t *pData = appdata->param.rxData.pData;
		uint8_t dataLength = appdata->param.rxData.dataLength;
		StackRetStatus_t status = appdata->param.rxData.status;
		uint32_t devAddress = appdata->param.rxData.devAddr;
				switch(status)
		{
			case LORAWAN_SUCCESS:
			{
				 //Successful transmission
				 if((dataLength > 0U) && (NULL != pData))
				 {
					
					 printf("*** Received DL Data ***\n\r");
					 printf("\nFrame Received at port %d\n\r",pData[0]);
					 printf("\nFrame Length - %d\n\r",dataLength);
					 printf("\nAddress - 0x%lx\n\r", devAddress);
					 printf ("\nPayload: ");
					 for (uint8_t i =0; i<dataLength - 1; i++)
					 {
						 printf("%x",pData[i+1]);
					 }
					 printf("\r\n*************************\r\n");
				 }
				 else
				 {
					 printf("Received ACK for Confirmed data\r\n");
				 }
			}	
			break;

			case LORAWAN_RADIO_SUCCESS:
			if((dataLength > 0U) && (NULL != pData))
			{
				// Data received
				printf("\n\rRADIO_OK \n\r");
			}
			else
			{
				// No data received
			}
			break;

			case LORAWAN_RADIO_NO_DATA:
			{
				printf("\n\rRADIO_NO_DATA \n\r");
			}			
			break;
			case LORAWAN_RADIO_DATA_SIZE:
				printf("\n\rRADIO_DATA_SIZE \n\r");
			break;
			case LORAWAN_RADIO_INVALID_REQ:
				printf("\n\rRADIO_INVALID_REQ \n\r");
			break;
			case LORAWAN_RADIO_BUSY:
				printf("\n\rRADIO_BUSY \n\r");
			break;
			case LORAWAN_RADIO_OUT_OF_RANGE:
				printf("\n\rRADIO_OUT_OF_RANGE \n\r");
			break;
			case LORAWAN_RADIO_UNSUPPORTED_ATTR:
				printf("\n\rRADIO_UNSUPPORTED_ATTR \n\r");
			break;
			case LORAWAN_RADIO_CHANNEL_BUSY:
				printf("\n\rRADIO_CHANNEL_BUSY \n\r");
			break;
			case LORAWAN_NWK_NOT_JOINED:
				printf("\n\rNWK_NOT_JOINED \n\r");
			break;
			case LORAWAN_INVALID_PARAMETER:
				printf("\n\rINVALID_PARAMETER \n\r");
			break;
			case LORAWAN_KEYS_NOT_INITIALIZED:
				printf("\n\rKEYS_NOT_INITIALIZED \n\r");
			break;
			case LORAWAN_SILENT_IMMEDIATELY_ACTIVE:
				printf("\n\rSILENT_IMMEDIATELY_ACTIVE\n\r");
			break;
			case LORAWAN_FCNTR_ERROR_REJOIN_NEEDED:
				printf("\n\rFCNTR_ERROR_REJOIN_NEEDED \n\r");
			break;
			case LORAWAN_INVALID_BUFFER_LENGTH:
				printf("\n\rINVALID_BUFFER_LENGTH \n\r");
			break;
			case LORAWAN_MAC_PAUSED :
				printf("\n\rMAC_PAUSED  \n\r");
			break;
			case LORAWAN_NO_CHANNELS_FOUND:
				printf("\n\rNO_CHANNELS_FOUND \n\r");
			break;
			case LORAWAN_BUSY:
				printf("\n\rBUSY\n\r");
			break;
			case LORAWAN_NO_ACK:
				printf("\n\rNO_ACK \n\r");
			break;
			case LORAWAN_NWK_JOIN_IN_PROGRESS:
				printf("\n\rALREADY JOINING IS IN PROGRESS \n\r");
			break;
			case LORAWAN_RESOURCE_UNAVAILABLE:
				printf("\n\rRESOURCE_UNAVAILABLE \n\r");
			break;
			case LORAWAN_INVALID_REQUEST:
				printf("\n\rINVALID_REQUEST \n\r");
			break;
			case LORAWAN_FCNTR_ERROR:
				printf("\n\rFCNTR_ERROR \n\r");
			break;
			case LORAWAN_MIC_ERROR:
				printf("\n\rMIC_ERROR \n\r");
			break;
			case LORAWAN_INVALID_MTYPE:
				printf("\n\rINVALID_MTYPE \n\r");
			break;
			case LORAWAN_MCAST_HDR_INVALID:
				printf("\n\rMCAST_HDR_INVALID \n\r");
			break;	
			default:
				printf("UNKNOWN ERROR\n\r");
			break;
		}
	}
	else if(LORAWAN_EVT_TRANSACTION_COMPLETE == appdata->evt)
	{
		switch(appdata->param.transCmpl.status)
		{
			case LORAWAN_SUCCESS:
			case LORAWAN_RADIO_SUCCESS:
			{
				printf("Transmission Success\r\n");
			}
			break;
			case LORAWAN_RADIO_NO_DATA:
			{
				printf("\n\rRADIO_NO_DATA \n\r");
			}
			break;
			case LORAWAN_RADIO_DATA_SIZE:
				printf("\n\rRADIO_DATA_SIZE \n\r");
			break;
			case LORAWAN_RADIO_INVALID_REQ:
				printf("\n\rRADIO_INVALID_REQ \n\r");
			break;
			case LORAWAN_RADIO_BUSY:
				printf("\n\rRADIO_BUSY \n\r");
			break;
			case LORAWAN_RADIO_OUT_OF_RANGE:
				printf("\n\rRADIO_OUT_OF_RANGE \n\r");
			break;
			case LORAWAN_RADIO_UNSUPPORTED_ATTR:
				printf("\n\rRADIO_UNSUPPORTED_ATTR \n\r");
			break;
			case LORAWAN_RADIO_CHANNEL_BUSY:
				printf("\n\rRADIO_CHANNEL_BUSY \n\r");
			break;
			case LORAWAN_NWK_NOT_JOINED:
				printf("\n\rNWK_NOT_JOINED \n\r");
			break;
			case LORAWAN_INVALID_PARAMETER:
				printf("\n\rINVALID_PARAMETER \n\r");
			break;
			case LORAWAN_KEYS_NOT_INITIALIZED:
				printf("\n\rKEYS_NOT_INITIALIZED \n\r");
			break;
			case LORAWAN_SILENT_IMMEDIATELY_ACTIVE:
				printf("\n\rSILENT_IMMEDIATELY_ACTIVE\n\r");
			break;
			case LORAWAN_FCNTR_ERROR_REJOIN_NEEDED:
				printf("\n\rFCNTR_ERROR_REJOIN_NEEDED \n\r");
			break;
			case LORAWAN_INVALID_BUFFER_LENGTH:
				printf("\n\rINVALID_BUFFER_LENGTH \n\r");
			break;
			case LORAWAN_MAC_PAUSED :
				printf("\n\rMAC_PAUSED  \n\r");
			break;
			case LORAWAN_NO_CHANNELS_FOUND:
				printf("\n\rNO_CHANNELS_FOUND \n\r");
			break;
			case LORAWAN_BUSY:
				printf("\n\rBUSY\n\r");
			break;
			case LORAWAN_NO_ACK:
				printf("\n\rNO_ACK \n\r");
			break;
			case LORAWAN_NWK_JOIN_IN_PROGRESS:
				printf("\n\rALREADY JOINING IS IN PROGRESS \n\r");
			break;
			case LORAWAN_RESOURCE_UNAVAILABLE:
				printf("\n\rRESOURCE_UNAVAILABLE \n\r");
			break;
			case LORAWAN_INVALID_REQUEST:
				printf("\n\rINVALID_REQUEST \n\r");
			break;
			case LORAWAN_FCNTR_ERROR:
				printf("\n\rFCNTR_ERROR \n\r");
			break;
			case LORAWAN_MIC_ERROR:
				printf("\n\rMIC_ERROR \n\r");
			break;
			case LORAWAN_INVALID_MTYPE:
				printf("\n\rINVALID_MTYPE \n\r");
			break;
			case LORAWAN_MCAST_HDR_INVALID:
				printf("\n\rMCAST_HDR_INVALID \n\r");
			break;
			default:
				printf("\n\rUNKNOWN ERROR\n\r");
			break;
			
					}
		printf("\n\r*************************************************\n\r");
	}
		
}

/*********************************************************************//*
\brief Callback function for the ending of Activation procedure 
 ************************************************************************/
void demo_joindata_callback(bool status)
{
	/* This is called every time the join process is finished */
	if(true == status)
	{
		uint32_t devAddress;
        bool mcastEnabled;
		
		joined = true;
		printf("\nJoining Successful\n\r");
		LORAWAN_GetAttr(DEV_ADDR, NULL, &devAddress);
		LORAWAN_GetAttr(MCAST_ENABLE, NULL, &mcastEnabled);
		
		if (devAddress != APP_MCAST_GROUP_ADDRESS)
		{
			printf("\nEnd device Address - 0x%lx\n\r", devAddress);
		}
		else if ((devAddress == APP_MCAST_GROUP_ADDRESS) && (true == mcastEnabled))
		{
			printf("\nAddress conflict between Device Address and Multicast group address\n\r");
		}
        print_application_config();
	}
	else
	{
		joined = false;
		printf("\nJoining Denied\n\r");
	}
	printf("\n\r*******************************************************\n\r");	
}

/*********************************************************************//*
 \brief  Application Task Handler 
 \return processing status
 ************************************************************************/
SYSTEM_TaskStatus_t APP_TaskHandler(void)
{
	StackRetStatus_t status;
	if (button_pressed && (bandSelected == true))
	{
		if (!joined)
		{
			/* Send Join request */
			status = LORAWAN_Join(APP_ACTIVATION_TYPE);
			if (LORAWAN_SUCCESS == status)
			{
				printf("\nJoin Request Sent\n\r");
			}
			else
			{
				print_stack_status(status);
			}
			button_pressed = false;
		}
		else
		{
			/* Read temperature sensor value */
			get_resource_data(TEMP_SENSOR,(uint8_t *)&cel_val);
			fahren_val = convert_celsius_to_fahrenheit(cel_val);
			printf("\nTemperature:");
			snprintf(temp_sen_str,sizeof(temp_sen_str),"%.1fC/%.1fF\n", cel_val, fahren_val);
			printf("%.1f\xf8 C/%.1f\xf8 F\n\r", cel_val, fahren_val);
			
			data_len = strlen(temp_sen_str);
			lorawanSendReq.buffer = &temp_sen_str;
			lorawanSendReq.bufferLength = data_len - 1;
			lorawanSendReq.confirmed = APP_TRANSMISSION_TYPE;
			lorawanSendReq.port = APP_FPORT;
			status = LORAWAN_Send(&lorawanSendReq);
			if (LORAWAN_SUCCESS == status)
			{
				printf("\nTx Data Sent \r\n");
			}
			else
			{
				print_stack_status(status);
			}
			button_pressed = false;
		}
	}
	else if(factory_reset || (bandSelected == false))
	{
		static uint8_t choice = 0xFF;
		static uint32_t prevCnt = 0;
		static bool firstTime = true;
		static uint8_t prevChoice = 0xFF;
		
		if(firstTime)
		{
		    printf("\nLongPress to select band or to exit from current band\r\n");
		    printf("\nPlease select one of the options to proceed\r\n\n");
			printf("\nFollowing options are available\r\n");
			for (uint8_t i = 0; i < sizeof(bandTable); i++)
			{
				printf("%d. %s\r\n",i+1,bandStrings[i]);
			}
			delay_ms(5);
			printf("\n\n Current option\r\n");
			prevCnt = longPress;
			firstTime = false;
		}

		if((prevCnt != longPress) && (bandSelected == false))
		{
			if(choice >= 0 && choice < sizeof(bandTable))
			{
			    bandSelected = true;

				if(!choice)
				{
#if (ENABLE_PDS == 1)
					if(PDS_IsRestorable())
					{
						if(prevChoice != 0xFF)
						{
                   		    choice = prevChoice;
						}
						else
						{
							PDS_RestoreAll();
							LORAWAN_GetAttr(ISMBAND,NULL,&prevChoice);
							for(uint32_t i = 0; i < sizeof(bandTable); i++)
							{
								if(bandTable[i] == prevChoice)
								{
									choice = i;
									break;
								}
							}
								
						}
          		        status = LORAWAN_Reset(bandTable[choice]);
						if(status == LORAWAN_SUCCESS)
						{
							uint32_t joinStatus = 0;
							PDS_RestoreAll();
				            bandSelected = true;
							LORAWAN_GetAttr(LORAWAN_STATUS,NULL, &joinStatus);
							printf("\r\n Restoration success. Proceeding with the current band %s\r\n",bandStrings[choice]);
							if(joinStatus & LORAWAN_NW_JOINED)
							{
							    joined = true;								
							}
							else
							{
								joined = false;
							}
							print_application_config();
						}
						else
						{
                            printf("\r\n Restoration Failed. Proceed with the Normal band selection process \r\n");
							bandSelected = false;
						}

					}
					else
					{
						printf("\rNot a valid option. Should choose band atleast once\r\n");
						bandSelected = false;
						choice = 0xFF;
						prevChoice = 0xFF;
						SYSTEM_PostTask(APP_TASK_ID);
					}
#else
                    if(prevChoice == 0xFF)
					{
						printf("\rNot a valid option. Should choose band atleast once\r\n");
						bandSelected = false;
						choice = 0xFF;
						prevChoice = 0xFF;
						SYSTEM_PostTask(APP_TASK_ID);
					}
					else
					{
						choice = prevChoice;
      					status = LORAWAN_Reset(bandTable[choice]);
						bandSelected = true;
						mote_set_parameters(bandTable[choice]);
					}
#endif
				}
				else if(choice == 1)
				{
					#if (ENABLE_PDS == 1)
					PDS_DeleteAll();
					#endif
					factory_reset = false;
					NVIC_SystemReset();
				}
				else
				{
					mote_set_parameters(bandTable[choice]);
					prevChoice = choice;
				}
			}
		}
		else if( (bandSelected == false) && (choice < (sizeof(bandTable) -1) || choice == 0xFF))
		{
			choice++;
			printf("\r                                              ");
	        printf("\rSelect %s",bandStrings[choice]);
		}
		else
		{
			choice = 0xff;
			bandSelected = false;
			joined = false;
			SYSTEM_PostTask(APP_TASK_ID);
		}
		prevCnt = longPress;

	}

	return SYSTEM_TASK_SUCCESS;
}

/*********************************************************************//*
 \brief      Set join parameters function
 \param[in]  activation type - notifies the activation type (OTAA/ABP)
 \return	 LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t set_join_parameters(ActivationType_t activation_type)
{
	StackRetStatus_t status;
	
	printf("\n********************Join Parameters********************\n\r");
	
	if(ACTIVATION_BY_PERSONALIZATION == activation_type)
	{
		status = LORAWAN_SetAttr (DEV_ADDR, &devAddr);
		if (LORAWAN_SUCCESS == status)
		{
			status = LORAWAN_SetAttr (APPS_KEY, appsKey);
		}
		
		if (LORAWAN_SUCCESS == status)
		{
			printf("\nApplication Session Key - ");
			print_array((uint8_t *)&appsKey, sizeof(appsKey));
			status = LORAWAN_SetAttr (NWKS_KEY, nwksKey);
		}
		
		if (LORAWAN_SUCCESS == status)
		{
			printf("\nNetwork Session Key - ");
			print_array((uint8_t *)&nwksKey, sizeof(nwksKey));
		}
					
	}
	else
	{
		status = LORAWAN_SetAttr (DEV_EUI, devEui);
		if (LORAWAN_SUCCESS == status)
		{
			printf("\nDevice EUI - ");
			print_array((uint8_t *)&devEui, sizeof(devEui));
			status = LORAWAN_SetAttr (APP_EUI, appEui);
		}
		
		if (LORAWAN_SUCCESS == status)
		{
			printf("\nApplication EUI - ");
			print_array((uint8_t *)&appEui, sizeof(appEui));
			status = LORAWAN_SetAttr (APP_KEY, appKey);
		}
		
		if (LORAWAN_SUCCESS == status)
		{
			printf("\nApplication Key - ");
			print_array((uint8_t *)&appKey, sizeof(appKey));
		}	
	}
	return status;
}

/*********************************************************************//*
 \brief      Function to Initialize the device type
 \param[in]  ed_class - notifies the device class (CLASS_A/CLASS_B/CLASS_C)
 \return	 LORAWAN_SUCCESS, if successfully set the device class
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t set_device_type(EdClass_t ed_class)
{
	StackRetStatus_t status = LORAWAN_SUCCESS;
	
	status = LORAWAN_SetAttr(EDCLASS, &ed_class);
	
	if((LORAWAN_SUCCESS == status) && ((CLASS_C | CLASS_B) & ed_class) && (true == APP_MCAST_ENABLE))
	{
		set_multicast_params();
	}

	return status;
}

/*********************************************************************//*
 \brief      Function to Initialize the Multicast parameters
 ************************************************************************/
void set_multicast_params (void)
{
	StackRetStatus_t status;
	
    printf("\n***************Multicast Parameters********************\n\r");
	
    status = LORAWAN_SetAttr(MCAST_APPS_KEY, &mcastAppsKey);
	if (status == LORAWAN_SUCCESS)
	{
		printf("\nMulticast Application Session Key - ");
		print_array((uint8_t *)&mcastAppsKey, sizeof(mcastAppsKey));
		status = LORAWAN_SetAttr(MCAST_NWKS_KEY, &mcastNwksKey);
	}
	
	if(status == LORAWAN_SUCCESS)
	{
		printf("\nMulticast Network Session Key - ");
		print_array((uint8_t *)&mcastNwksKey, sizeof(mcastNwksKey));
		status = LORAWAN_SetAttr(MCAST_GROUP_ADDR, &mcastDevAddr);
	}
	
	if (status == LORAWAN_SUCCESS)
	{
		printf("\nMulticast Group Address - 0x%lx\n\r", mcastDevAddr);
		status = LORAWAN_SetAttr(MCAST_ENABLE, &mcastEnable);
	}
	else
	{
		printf("\nFailed to set Multicast Group Address\n\r");
	}
	
	if (status == LORAWAN_SUCCESS)
	{
		printf("\nMulticast Feature - Enabled\n\r");
	}
	else
	{
		printf("\nFailed to enable Multicast Feature\n\r");
	}  
}


/***********************************************************************
 \brief      Function to Initialize set default parameters
 \param[in]  void 
 \return	 LORAWAN_SUCCESS, if successfully set all the parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t mote_set_parameters(IsmBand_t ismBand)
{
	StackRetStatus_t status;

    LORAWAN_Reset(ismBand);

	if ((ismBand == ISM_NA915) || (ismBand == ISM_AU915))
	{
		#define MAX_NA_CHANNELS 72
		#define MAX_SUBBAND_CHANNELS 8
	
		ChannelParameters_t ch_params;
	
		uint8_t allowed_min_125khz_ch,allowed_max_125khz_ch,allowed_500khz_channel;
	
		allowed_min_125khz_ch = (SUBBAND-1)*MAX_SUBBAND_CHANNELS;
	
		allowed_max_125khz_ch = ((SUBBAND-1)*MAX_SUBBAND_CHANNELS) + 7 ;
	
		allowed_500khz_channel = SUBBAND+63;
	
		for (ch_params.channelId = 0; ch_params.channelId < MAX_NA_CHANNELS; ch_params.channelId++)
		{
			if((ch_params.channelId >= allowed_min_125khz_ch) && (ch_params.channelId <= allowed_max_125khz_ch))
			{
				ch_params.channelAttr.status = true;
			}
			else if(ch_params.channelId == allowed_500khz_channel)
			{
				ch_params.channelAttr.status = true;
			}
			else
			{
				ch_params.channelAttr.status = false;
			}
		
			LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
		}
	}

	/* Initialize the join parameters */
	status = set_join_parameters(APP_ACTIVATION_TYPE);
	if (LORAWAN_SUCCESS != status)
	{
		printf("\nJoin parameters initialization failed\n\r");
		return status;
	}
	
	/* Set the device type */
	status = set_device_type(APP_ENDDEVICE_CLASS);
	if (LORAWAN_SUCCESS != status)
	{
		printf("\nUnsupported Device Type\n\r");
		return status;
	}
	
	/* Send Join request */
	status = LORAWAN_Join(APP_ACTIVATION_TYPE);
	if (LORAWAN_SUCCESS == status)
	{
		printf("\nJoin Request Sent\n\r");
	}
	else
	{
		print_stack_status(status);
	}
	
	return status;
}

/*********************************************************************//*
 \brief      Function to Print stack default parameters
 ************************************************************************/
void print_default_parameters(void)
{
	printf("\n***************Default Stack Parameters****************\n\r");
	uint8_t retValue;
#ifdef EU_BAND
	LORAWAN_GetAttr(ISMBAND, NULL, &retValue);
	printf("\nSelected ISM Band -" );
	printf((retValue == 0)?"EU868 MHz\n\r":"EU433 MHz\n\r");
#endif
	LORAWAN_GetAttr(CURRENT_DATARATE, NULL, &retValue);
	printf("\nCurrent Data rate - DR%d\n\r",retValue);

    LORAWAN_GetAttr(TX_POWER,NULL,&retValue);
    printf("\nTX Power - %d\n\r",retValue);
}

/*********************************************************************//*
 \brief      Function to Print array of characters
 \param[in]  *array  - Pointer of the array to be printed
 \param[in]   length - Length of the array
 ************************************************************************/
void print_array (uint8_t *array, uint8_t length)
{
	printf("0x");
	for (uint8_t i =0; i < length; i++)
	{
		printf("%02x", *array);
		array++;
	}
	printf("\n\r");
}

/*********************************************************************//*
 \brief      Function to Print application configuration
 ************************************************************************/
void  print_application_config (void)
{
	EdClass_t edClass;
	printf("\n***************Application Configuration***************\n\r");
	LORAWAN_GetAttr(EDCLASS, NULL, &edClass);
	printf("\nDevice Type - ");
	printf((edClass == CLASS_A)?"CLASS A\n\r":"CLASS C\n\r");
	
	printf("\nActivation Type - ");
	printf((APP_ACTIVATION_TYPE == OVER_THE_AIR_ACTIVATION)?"OTAA\n\r":"ABP\n\r");
	
	printf("\nTransmission Type - ");
	printf((APP_TRANSMISSION_TYPE == CONFIRMED)?"CONFIRMED\n\r":"UNCONFIRMED\n\r");
	
	printf("\nFPort - %d\n\r", APP_FPORT);
	
	printf("\n*******************************************************\n\r");
}

/*********************************************************************//*
 \brief      Function to Print stack return status
 \param[in]  status - Status from the stack
 ************************************************************************/
void print_stack_status(StackRetStatus_t status)
{
	switch(status)
	{
		case LORAWAN_SUCCESS:
		     printf("\nLorawan Success\n\r");
		break;
		
		case LORAWAN_BUSY:
		     printf("\nLorawan state invalid - Stack Busy\n\r");
		break;
			 
		case LORAWAN_NWK_NOT_JOINED:
		    printf("\nDevice not joined to network yet\n\r");
		break;
		
		case LORAWAN_INVALID_PARAMETER:
			printf("\nInvalid parameter\n\r");
		break;
		
		case LORAWAN_KEYS_NOT_INITIALIZED:
			printf("\nKeys not initialized\n\r");
		break;
		
		case LORAWAN_SILENT_IMMEDIATELY_ACTIVE:
			printf("\nSilent immediately active\n\r");
		break;
		
		case LORAWAN_FCNTR_ERROR_REJOIN_NEEDED:
			printf("\nFramecounter Error - Rejoin needed\n\r");
		break;
		
		case LORAWAN_INVALID_BUFFER_LENGTH:
			printf("\nInvalid buffer length\n\r");
		break;
		
		case LORAWAN_MAC_PAUSED:
			printf("\nMAC Paused\n\r");
		break;
		
		case LORAWAN_NO_CHANNELS_FOUND:
			printf("\nNo free channels found\n\r");
		break;
		
		case LORAWAN_INVALID_REQUEST:
			printf("\nRequest invalid\n\r");
		break;
	    case LORAWAN_NWK_JOIN_IN_PROGRESS:
			printf("\nAlready, Joining is in Progress\n\r");
		break;
		default:
		   printf("\nRequest Failed %d\n\r",status);
		break;
	}
}

/*********************************************************************//*
 \brief      Function to convert Celsius value to Fahrenheit
 \param[in]  cel_val   - Temperature value in Celsius
 \param[out] fauren_val- Temperature value in Fahrenheit
 ************************************************************************/
static float convert_celsius_to_fahrenheit(float celsius_val)
{
	float fauren_val;
	/* T(°F) = T(°C) × 9/5 + 32 */
	fauren_val = (((celsius_val * 9)/5) + 32);
	
	return fauren_val;
	
}

/*********************************************************************//*
 \brief      Function to Check stack return status
 \param[in]  Void 
 ************************************************************************/
void check_stack_status()
{
	uint32_t macStatusMask;
	LORAWAN_GetAttr(LORAWAN_STATUS,NULL, &macStatusMask);
	// If PDS enabled and mote is already successfully joined, re-joining is not required.
	// Instead, data transmission can be triggered.
	if(macStatusMask & LORAWAN_NW_JOINED) 
	{
		joined = true;
		demo_joindata_callback(true);
	}
}

/* eof demo_app.c */
