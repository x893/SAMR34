/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    pds_nvm.h

  @Summary:
    This is the PDS NVM header file which containes NVM abstractions for PDS.

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
 *                           pds_nvm.h
 *
 * Pds NVM header file.
 *
 ******************************************************************************/

#ifndef _PDS_NVM_H_
#define _PDS_NVM_H_

/******************************************************************************
                   Includes section
******************************************************************************/
#include "compiler.h"
#include "pds_interface.h"
#include "common_nvm.h"
#include "nvm.h"

/******************************************************************************
                   Defines section
******************************************************************************/

#define EEPROM_PAGE_SIZE        (NVMCTRL_PAGE_SIZE)
#define EEPROM_PAGE_PER_ROW     (NVMCTRL_ROW_PAGES)
#define EEPROM_ROW_SIZE         (EEPROM_PAGE_SIZE*EEPROM_PAGE_PER_ROW)
#define EEPROM_NUM_ROWS         (EEPROM_SIZE/EEPROM_ROW_SIZE)


/******************************************************************************
                               Types section
*******************************************************************************/

/******************************************************************************
                   Prototypes section
******************************************************************************/

/**************************************************************************//**
\brief	Initializes the NVM by checking the EEPROM size and calling nvm init.

\param[in] none
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
PdsStatus_t pdsNvmInit(void);

/**************************************************************************//**
\brief	This function will calculate the crc of the buffer contents and writes
		the buffer contents to nvm. It will also read the same content to verify
		if its written properly.

\param[in] 	pdsFileItemIdx - The file id to be written.
\param[in] 	buffer - The buffer containing data to be written.
\param[in] 	size - The size of the data in the buffer.
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
PdsStatus_t pdsNvmWrite(uint16_t rowId, PdsMem_t *buffer, uint16_t size);

/**************************************************************************//**
\brief	This function will read the contents of NVM and verify the crc.

\param[in] 	pdsFileItemIdx - The file id to be read.
\param[in] 	buffer - The buffer containing data to be read.
\param[in] 	size - The size of the data in the buffer.
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
PdsStatus_t pdsNvmRead(uint16_t rowId, PdsMem_t *buffer, uint16_t size);

/**************************************************************************//**
\brief	Will erase the contents of a row.

\param[in] 	rowId - The rowId to be erased.
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
PdsStatus_t pdsNvmErase(uint16_t rowId);

/**************************************************************************//**
\brief	Erases all the contents of NVM of all rows.

\param[in] 	none
\param[out] status - The return status of the function's operation of type PdsStatus_t.
******************************************************************************/
PdsStatus_t pdsNvmEraseAll(void);

#endif  /* _PDS_NVM_H_ */

/* eof pds_nvm.h */