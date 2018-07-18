/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    pds_task_handler.c

  @Summary:
    This is the Pds Driver Task Manager source file which contains Pds task
	scheduler.

  @Description:
    This source file provides LoRa-specific implementations for Pds scheduler for MLS.
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
 *                           pds_task_handler.c
 *
 * Pds Driver Task Manager source file
 *
 ******************************************************************************/

/******************************************************************************
                   Includes section
******************************************************************************/
#include "system_task_manager.h"
#include "pds_interface.h"
#include "pds_common.h"
#include "pds_task_handler.h"
#include "pds_wl.h"
#include "atomic.h"
#include <stdint.h>

/************************************************************************/
/*  Static variables                                                    */
/************************************************************************/

/**************************************************************************//**
\brief pdsTaskFlags - 8-bit bitmap for the tasks of PDS layer.
******************************************************************************/
static volatile uint8_t pdsTaskFlags = 0x0000u;

/************************************************************************/
/*  Extern variables                                                    */
/************************************************************************/
extern bool isFileSet[];
extern PdsFileMarks_t fileMarks[];

/******************************************************************************
                   Prototypes section
******************************************************************************/
#if (ENABLE_PDS == 1)
static SYSTEM_TaskStatus_t pdsStoreDeleteHandler(void);
static PdsStatus_t pdsStoreDelete(PdsFileItemIdx_t pdsFileItemIdx, uint8_t *buffer);
#endif

/**************************************************************************//**
\brief Array of pds task handlers.
******************************************************************************/
#if (ENABLE_PDS == 1)
static SYSTEM_TaskStatus_t (*pdsTaskHandlers[PDS_TASKS_COUNT])(void) = {
	
    /* In the order of descending priority */
    pdsStoreDeleteHandler
};
#endif

/******************************************************************************
                   Implementations section
******************************************************************************/
/**************************************************************************//**
\brief Set task for PDS task manager.

\param[in] id - a single value from the type PdsTaskIds_t
******************************************************************************/
void pdsPostTask(PdsTaskIds_t id)
{
    ATOMIC_SECTION_ENTER
    pdsTaskFlags |= id;
    ATOMIC_SECTION_EXIT

    /* Also post a PDS task to the system */
    SYSTEM_PostTask(PDS_TASK_ID);
}

/**************************************************************************//**
\brief Clear task for PDS task manager.

\param[in] id - a single value from the type PdsTaskIds_t
******************************************************************************/
void pdsClearTask(PdsTaskIds_t id)
{
    ATOMIC_SECTION_ENTER
    pdsTaskFlags &= ~id;
    ATOMIC_SECTION_EXIT
}

/**************************************************************************//**
\brief PDS task handler.
******************************************************************************/
SYSTEM_TaskStatus_t PDS_TaskHandler(void)
{
#if (ENABLE_PDS == 1)	
#ifdef UT_D
    printf("\n Starting PDS_TaskHandler() \n");
#endif

    if (pdsTaskFlags)
    {
        for (uint16_t taskId = 0; taskId < PDS_TASKS_COUNT; taskId++)
        {
            if ((1 << taskId) & (pdsTaskFlags))
            {
#ifdef UT_D
                printf("\n pdsTaskFlags : %d \n", pdsTaskFlags);
#endif

                ATOMIC_SECTION_ENTER
                pdsTaskFlags &= ~(1 << taskId);
                ATOMIC_SECTION_EXIT

#ifdef UT_D
                printf("\n pdsTaskFlags : %d \n", pdsTaskFlags);
#endif

                pdsTaskHandlers[taskId]();

				if (pdsTaskFlags)
                {
                    SYSTEM_PostTask(PDS_TASK_ID);
                }
                
                break;
            }
        }
    }
#endif
    return SYSTEM_TASK_SUCCESS;
}

#if (ENABLE_PDS == 1)
/**************************************************************************//**
\brief	This function checks if an operation is pending for a file and will
		initiate store/delete operation.

\param[out] status - The return status of the function's operation.
******************************************************************************/
static SYSTEM_TaskStatus_t pdsStoreDeleteHandler(void)
{
	PdsStatus_t status = SYSTEM_TASK_SUCCESS;

	PdsFileItemIdx_t fileId = PDS_FILE_MAC_01_IDX;
	PdsMem_t buffer;

	memset(&buffer, 0, sizeof(PdsMem_t));
	for (; fileId < PDS_MAX_FILE_IDX; fileId++)
	{
		if (true == isFileSet[fileId])
		{
			status = pdsStoreDelete(fileId, (uint8_t *)&(buffer));
			if (status != PDS_OK)
			{
				// assert;
			}
			isFileSet[fileId] = false;
			fileId++;
			break;
		}
	}
	for (; fileId < PDS_MAX_FILE_IDX; fileId++)
	{
		if (isFileSet[fileId])
		{
			pdsPostTask(PDS_STORE_DELETE_TASK_ID);
			break;
		}
	}

	return status;
}

/**************************************************************************//**
\brief This function stores and deletes the items in a file based on file marks set.

\param[in] pdsFileItemIdx - The file id to look for.
\param[in] buffer - The buffer to be used for reading and writing a file.
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
static PdsStatus_t pdsStoreDelete(PdsFileItemIdx_t pdsFileItemIdx, uint8_t *buffer)
{
	PdsStatus_t status = PDS_OK;

	uint8_t *ptr;
	ItemMap_t itemInfo;
	ItemHeader_t itemHeader;
	uint16_t size;

	memcpy((void *)&itemInfo, (void *)(fileMarks[pdsFileItemIdx].itemListAddr + (fileMarks[pdsFileItemIdx].numItems - 1)), sizeof(ItemMap_t));
	size = itemInfo.itemOffset + itemInfo.size + sizeof(ItemHeader_t);
	status = pdsWlRead(pdsFileItemIdx, (PdsMem_t *)buffer, size);

	if ((PDS_OK != status) && (PDS_NOT_FOUND != status))
	{
		return status;
	}

	itemHeader.magic = PDS_MAGIC;
	itemHeader.version = PDS_FILES_VERSION;

	for (uint8_t itemIdx = 0; itemIdx < fileMarks[pdsFileItemIdx].numItems; itemIdx++)
	{

		memcpy((void *)&itemInfo, (fileMarks[pdsFileItemIdx].itemListAddr) + itemIdx, sizeof(ItemMap_t));
		ptr = (uint8_t *)(&(((PdsMem_t *)(buffer))->NVM_Struct.pdsNvmData.WL_Struct.pdsWlData));
		ptr += itemInfo.itemOffset;

		if (PDS_OP_STORE == *(fileMarks[pdsFileItemIdx].fileMarkListAddr + itemIdx))
		{
			*(fileMarks[pdsFileItemIdx].fileMarkListAddr + itemIdx) = PDS_OP_NONE;
			itemHeader.size = itemInfo.size;
			itemHeader.itemId = itemInfo.itemId;
			itemHeader.delete = false;
			memcpy((void *)(ptr), (void *)&itemHeader, sizeof(ItemHeader_t));
			ptr += sizeof(ItemHeader_t);
			memcpy((void *)(ptr), (void *)itemInfo.ramAddress, itemInfo.size);
		}
		else if (PDS_OP_DELETE == *(fileMarks[pdsFileItemIdx].fileMarkListAddr + itemIdx))
		{
			*(fileMarks[pdsFileItemIdx].fileMarkListAddr + itemIdx) = PDS_OP_NONE;
			itemHeader.size = itemInfo.size;
			itemHeader.itemId = itemInfo.itemId;
			itemHeader.delete = true;
			memcpy((void *)(ptr), (void *)&itemHeader, sizeof(ItemHeader_t));
		}
	}

	memcpy((void *)&itemInfo, (void *)(fileMarks[pdsFileItemIdx].itemListAddr + fileMarks[pdsFileItemIdx].numItems), sizeof(ItemMap_t));
	size = itemInfo.itemOffset + itemInfo.size + sizeof(ItemHeader_t);
	status = pdsWlWrite(pdsFileItemIdx, (PdsMem_t *)buffer, PDS_WL_DATA_SIZE);

	return status;
}
#endif
/* eof pds_task_handler.c */
