/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    pds_wl.h

  @Summary:
    This is the Pds wear levelling header file which contains Pds wear levelling 
	headers.

  @Description:
    This header file provides LoRa-specific implementations for Pds.
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
 *                           pds_wl.h
 *
 * Pds wear levelling header file
 *
 ******************************************************************************/

#ifndef _PDS_WL_H_
#define _PDS_WL_H_

/******************************************************************************
                   Includes section
******************************************************************************/
#include "compiler.h"
#include "pds_nvm.h"
#include "pds_interface.h"

/******************************************************************************
                   Defines section
******************************************************************************/

/******************************************************************************
                               Types section
*******************************************************************************/
typedef struct _RowMap
{
    uint32_t counter;
    uint16_t memId;
    uint16_t previousIdx;
} RowMap_t;

typedef struct _FileMap
{
    uint16_t maxCounterRowIdx;
} FileMap_t;

typedef struct _UpdateFileMap
{
    uint32_t counter;
    uint16_t memId;
    uint16_t rowIdx;
} UpdateFileMap_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/

/**************************************************************************//**
\brief Initializes the WL PDS by updating the row and file map.

\param[in] none
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
PdsStatus_t pdsWlInit(void);

/**************************************************************************//**
\brief	This function will find the free row index to write to, updates the WL_Struct
		header and writes to NVM. If the nvm write is successful it updates the
		row and file map.

\param[in] 	pdsFileItemIdx - The file id to be written to.
\param[in] 	buffer - The buffer containing data to be written.
\param[in] 	size - The size of the data in the buffer.
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
PdsStatus_t pdsWlWrite(PdsFileItemIdx_t pdsFileItemIdx, PdsMem_t *buffer, uint16_t size);

/**************************************************************************//**
\brief	This function will find extract the row where the file is stored and 
		read from NVM.

\param[in] 	pdsFileItemIdx - The file id to be read from.
\param[in] 	buffer - The buffer containing data to be written.
\param[in] 	size - The size of the data in the buffer.
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
PdsStatus_t pdsWlRead(PdsFileItemIdx_t pdsFileItemIdx, PdsMem_t *buffer, uint16_t size);

/**************************************************************************//**
\brief This function checks if a file is found in the file map.

\param[out] - return true or false
******************************************************************************/
bool isFileFound(PdsFileItemIdx_t pdsFileItemIdx);

/**************************************************************************//**
\brief This function Erases Filemap and Rowmap array in WL and Initiates NVM Erase all.

\param[out] - void
******************************************************************************/
void pdsWlDeleteAll(void);

#endif  /* _PDS_WL_H_ */

/* eof pds_wl.h */