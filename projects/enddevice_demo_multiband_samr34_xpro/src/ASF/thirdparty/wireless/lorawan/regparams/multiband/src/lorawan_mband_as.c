/********************************************************************
* Copyright (C) 2016 Microchip Technology Inc. and its subsidiaries
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
*                           lorawan_as.c
*
* LoRaWAN as file
*
******************************************************************************/
/************************ PRIVATE FUNCTION PROTOTYPES *************************/
#include "lorawan_multiband.h"

#if (AS_BAND == 1)
/*Init Functions's*/
static void InitDefault923Channels (void);


/*****************************CONSTANTS ***************************************/
static const ChannelParams_t DefaultChannels923[] = {
	LC0_923,
	LC1_923
};

static const OthChannelParams_t AdvChannels923[] = {
	ADV_LC0_923,
	ADV_LC1_923
};

static const DRParams_t DefaultDrParamsAS[] = {
	DR0_PARAMS_AS,
	DR1_PARAMS_AS,
	DR2_PARAMS_AS,
	DR3_PARAMS_AS,
	DR4_PARAMS_AS,
	DR5_PARAMS_AS,
	DR6_PARAMS_AS,
	DR7_PARAMS_AS,
};

#if (ENABLE_PDS == 1)
#define PDS_REG_AS_CH_PARAM_1_ADDR                      ((uint8_t *)&(RegParams.cmnParams.paramsType2.chParams[MAX_CHANNELS_T2 - MAX_CHANNELS_T2]))
#define PDS_REG_AS_CH_PARAM_2_ADDR                      ((uint8_t *)&(RegParams.cmnParams.paramsType2.othChParams[MAX_CHANNELS_T2 - MAX_CHANNELS_T2]))
#define PDS_REG_AS_BAND_ADDR                            ((uint8_t *)&(RegParams.band))

#define PDS_REG_AS_CH_PARAM_1_SIZE					    sizeof(RegParams.cmnParams.paramsType2.chParams)
#define PDS_REG_AS_CH_PARAM_2_SIZE					    sizeof(RegParams.cmnParams.paramsType2.othChParams)
#define PDS_REG_AS_BAND_SIZE                            sizeof(RegParams.band)

#define PDS_REG_AS_CH_PARAM_1_OFFSET                    (PDS_FILE_START_OFFSET)
#define PDS_REG_AS_CH_PARAM_2_OFFSET                    (PDS_REG_AS_CH_PARAM_1_OFFSET + PDS_SIZE_OF_ITEM_HDR + PDS_REG_AS_CH_PARAM_1_SIZE)
#define PDS_REG_AS_BAND_OFFSET                          (PDS_REG_AS_CH_PARAM_2_OFFSET + PDS_SIZE_OF_ITEM_HDR + PDS_REG_AS_CH_PARAM_2_SIZE)

/* PDS Reg Params NA Item declaration */

const ItemMap_t pds_reg_as_item_list[] = {
	DECLARE_ITEM(PDS_REG_AS_CH_PARAM_1_ADDR,
	PDS_FILE_REG_AS_05_IDX,
	(uint8_t)PDS_REG_AS_CH_PARAM_1,
	PDS_REG_AS_CH_PARAM_1_SIZE,
	PDS_REG_AS_CH_PARAM_1_OFFSET),
	DECLARE_ITEM(PDS_REG_AS_CH_PARAM_2_ADDR,
	PDS_FILE_REG_AS_05_IDX,
	(uint8_t)PDS_REG_AS_CH_PARAM_2,
	PDS_REG_AS_CH_PARAM_2_SIZE,
	PDS_REG_AS_CH_PARAM_2_OFFSET),	
	DECLARE_ITEM(PDS_REG_AS_BAND_ADDR,
	PDS_FILE_REG_AS_05_IDX,
	(uint8_t)PDS_REG_AS_BAND,
	PDS_REG_AS_BAND_SIZE,
	PDS_REG_AS_BAND_OFFSET)
};

PdsOperations_t aRegAsPdsOps[PDS_REG_AS_MAX_VALUE];

/* PDS Callback function */
void LorawanReg_AS_Pds_Cb(void);
#endif
#endif
/****************************** FUNCTIONS *************************************/

StackRetStatus_t LORAReg_InitAS(IsmBand_t ismBand)
{
	StackRetStatus_t result = LORAWAN_SUCCESS;
#if(AS_BAND == 1)
	static bool initialized = false;
	
	RegParams.TxCurDataRate = MAC_DEF_TX_CURRENT_DATARATE_AS;
	RegParams.maxChannels = MAX_CHANNELS_AS;
	RegParams.MacTxPower = MAC_DEF_TX_POWER_AS;
	RegParams.pChParams = &RegParams.cmnParams.paramsType2.chParams[0];
	RegParams.pDrParams = &RegParams.cmnParams.paramsType2.DRParams[0];
	RegParams.pOtherChParams = &RegParams.cmnParams.paramsType2.othChParams[0];
	RegParams.pDutyCycleTimer = &RegParams.cmnParams.paramsType2.DutyCycleTimer;
	RegParams.DefRx1DataRate = MAC_RX1_WINDOW_DATARATE_AS;
	RegParams.DefRx2DataRate = MAC_RX2_WINDOW_DATARATE_AS;
	RegParams.DefRx2Freq = MAC_RX2_WINDOW_FREQ_AS;	
	RegParams.MinNewChIndex = NEW_CHANNEL_INDEX_AS;
	RegParams.FeaturesSupport = PA_SUPPORT;
	RegParams.minDataRate = MAC_DATARATE_MIN_AS;
	RegParams.maxDataRate = MAC_DATARATE_MAX_AS;
	RegParams.Rx1DrOffset = 7;
	RegParams.maxTxPwrIndx = MAX_TX_PWR_INDEX_AS;
	RegParams.maxTxPwr = 16;
	RegParams.cmnParams.paramsType2.minNonDefChId = 2;
	
	RegParams.band = ismBand;
	
	if(ismBand >= ISM_BRN923 && ismBand <= ISM_VTM923)
	{
		InitDefault923Channels ();
		RegParams.cmnParams.paramsType2.txParams.maxEIRP = DEFAULT_EIRP_AS;
		memcpy (RegParams.pDrParams, DefaultDrParamsAS, sizeof(DefaultDrParamsAS) );
#if (ENABLE_PDS == 1)
		/*Fill PDS item id in RegParam Structure */
		RegParams.regParamItems.fileid = PDS_FILE_REG_AS_05_IDX;
		RegParams.regParamItems.alt_ch_item_id = 0;
		RegParams.regParamItems.ch_param_1_item_id = PDS_REG_AS_CH_PARAM_1;
		RegParams.regParamItems.ch_param_2_item_id = PDS_REG_AS_CH_PARAM_2;
		RegParams.regParamItems.sb_dc_prescalr_item_id = 0;
		RegParams.regParamItems.band_item_id = PDS_REG_AS_BAND;
		/* File ID AS923 - Register */
		PdsFileMarks_t filemarks;
		filemarks.fileMarkListAddr = aRegAsPdsOps;
		filemarks.numItems =  (uint8_t)(PDS_REG_AS_MAX_VALUE & 0x00FF);
		filemarks.itemListAddr = (ItemMap_t *)&pds_reg_as_item_list;
		filemarks.fIDcb = LorawanReg_AS_Pds_Cb;
		PDS_RegFile(PDS_FILE_REG_AS_05_IDX,filemarks);
#endif		
	}
	else
	{
		result =  LORAWAN_INVALID_PARAMETER;
	}
	
	if(!initialized)
	{
		initialized = true;
	}

    LORAREG_InitGetAttrFnPtrsAS();	
	LORAREG_InitValidateAttrFnPtrsAS();
	LORAREG_InitSetAttrFnPtrsAS();
#if (ENABLE_PDS == 1)
	PDS_STORE(RegParams.regParamItems.band_item_id);
#endif
	
#else
	result = UNSUPPORTED_BAND;
#endif
	return result;
}


/*
 * \brief This function initializes all the AS923 Channels to default values
 */
#if(AS_BAND == 1)
static void InitDefault923Channels (void)
{
	uint8_t i;

	memset (RegParams.pChParams, 0, sizeof(DefaultChannels923) );
	memset (RegParams.pOtherChParams, 0, sizeof(AdvChannels923) );
	memcpy (RegParams.pChParams, DefaultChannels923, sizeof(DefaultChannels923));
	memcpy (RegParams.pOtherChParams, AdvChannels923, sizeof(AdvChannels923));
	for (i = 2; i < RegParams.maxChannels; i++)
	{
		RegParams.pChParams[i].dataRange.value = UINT8_MAX;
	}
}
#if (ENABLE_PDS == 1)
void LorawanReg_AS_Pds_Cb(void)
{
	; // nothing to do
}
#endif
#endif
