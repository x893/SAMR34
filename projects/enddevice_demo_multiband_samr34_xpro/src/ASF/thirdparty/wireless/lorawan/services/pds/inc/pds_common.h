/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    pds_common.h

  @Summary:
    This is the PDS common header file.

  @Description:
    This header file provides PDS headers.
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
 *                           pds_common.h
 *
 * Pds Common header file
 *
 ******************************************************************************/

#ifndef _PDS_COMMON_H
#define _PDS_COMMON_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include "string.h"
#include "limits.h"
#include "conf_nvm.h"

/******************************************************************************
                               Defines and Types section
*******************************************************************************/
#define PDS_MEM_SIZE			8192U

#if (ENABLE_PDS == 1)
#ifndef EEPROM_SIZE
#error "EEPROM_SIZE is not defined."
#endif
#endif

typedef struct _PdsNvmHeader_t
{
	uint16_t crc;
	uint8_t size;
	uint8_t version;
} PdsNvmHeader_t;

#define PDS_NVM_MEM_SIZE		NVMCTRL_ROW_SIZE
#define PDS_NVM_HEADER_SIZE		sizeof(PdsNvmHeader_t)
#define PDS_NVM_DATA_SIZE		PDS_NVM_MEM_SIZE - PDS_NVM_HEADER_SIZE


COMPILER_PACK_SET(1)
typedef struct _PdsWlHeader_t
{
	uint8_t magicNo;
	uint8_t version;
	uint8_t size;
	uint16_t memId;
	uint32_t counter;
} PdsWlHeader_t;
COMPILER_PACK_RESET()

typedef uint8_t PdsWlData_t;
typedef uint8_t PdsWlMem_t;

#define PDS_WL_MEM_SIZE			PDS_NVM_DATA_SIZE
#define PDS_WL_HEADER_SIZE		sizeof(PdsWlHeader_t)
#define PDS_WL_DATA_SIZE		PDS_WL_MEM_SIZE - PDS_WL_HEADER_SIZE

COMPILER_PACK_SET(1)
typedef union _PdsWl_t
{
	struct _WL_Struct
	{
		PdsWlHeader_t pdsWlHeader;
		PdsWlData_t pdsWlData[PDS_WL_DATA_SIZE];
	} WL_Struct;
	struct _WL_Mem
	{
		PdsWlMem_t pdsWlMem[PDS_WL_MEM_SIZE];
	} WL_Mem;
} PdsWl_t;
COMPILER_PACK_RESET()

typedef PdsWl_t PdsNvmData_t;
typedef uint8_t PdsNvmMem_t;

COMPILER_PACK_SET(1)
typedef union _PdsNvm_t
{
	struct _NVM_Struct
	{
		PdsNvmHeader_t pdsNvmHeader;
		PdsNvmData_t pdsNvmData;
	} NVM_Struct;
	struct _NVM_Mem
	{
		PdsNvmMem_t pdsNvmMem[PDS_NVM_MEM_SIZE];
	} NVM_Mem;
} PdsNvm_t;
COMPILER_PACK_RESET()


typedef PdsNvm_t PdsMem_t;





/******************************************************************************
                   Prototypes section
******************************************************************************/

#endif  /*_PDS_COMMON_H */

/* eof pds_common.h */
