/*****************************************************************************
 *  nvmem.h  - CC3000 Host Driver Implementation.
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef _INCLUDE_NUTTX_WIRELESS_CC3000_NVMEM_H
#define _INCLUDE_NUTTX_WIRELESS_CC3000_NVMEM_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include "cc3000_common.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* NVMEM file ID - system files*/

#define NVMEM_NVS_FILEID                    (0)
#define NVMEM_NVS_SHADOW_FILEID             (1)
#define NVMEM_WLAN_CONFIG_FILEID            (2)
#define NVMEM_WLAN_CONFIG_SHADOW_FILEID     (3)
#define NVMEM_WLAN_DRIVER_SP_FILEID         (4)
#define NVMEM_WLAN_FW_SP_FILEID             (5)
#define NVMEM_MAC_FILEID                    (6)
#define NVMEM_FRONTEND_VARS_FILEID          (7)
#define NVMEM_IP_CONFIG_FILEID              (8)
#define NVMEM_IP_CONFIG_SHADOW_FILEID       (9)
#define NVMEM_BOOTLOADER_SP_FILEID          (10)
#define NVMEM_RM_FILEID                     (11)

/* NVMEM file ID - user files*/

#define NVMEM_AES128_KEY_FILEID             (12)
#define NVMEM_SHARED_MEM_FILEID             (13)

/* max entry in order to invalid nvmem */

#define NVMEM_MAX_ENTRY                     (16)

/*****************************************************************************
 * Public Data
 *****************************************************************************/

#ifdef  __cplusplus
extern "C"
{
#endif

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/******************************************************************************
 * Name: nvmem_read
 *
 * Description:
 *   Reads data from the file referred by the ulFileId parameter. Reads data
 *   from file ulOffset till length. Err if the file can't be used, is
 *   invalid, or if the read is out of bounds.
 *
 * Input Parameters:
 *   ulFileId   nvmem file id:
 *                NVMEM_NVS_FILEID, NVMEM_NVS_SHADOW_FILEID,
 *                NVMEM_WLAN_CONFIG_FILEID, NVMEM_WLAN_CONFIG_SHADOW_FILEID,
 *                NVMEM_WLAN_DRIVER_SP_FILEID, NVMEM_WLAN_FW_SP_FILEID,
 *                NVMEM_MAC_FILEID, NVMEM_FRONTEND_VARS_FILEID,
 *                NVMEM_IP_CONFIG_FILEID, NVMEM_IP_CONFIG_SHADOW_FILEID,
 *                NVMEM_BOOTLOADER_SP_FILEID, NVMEM_RM_FILEID,
 *                and user files 12-15.
 *   ulLength    number of bytes to read
 *   ulOffset    ulOffset in file from where to read
 *   buff        output buffer pointer
 *
 * Returned Value:
 *   Number of bytes read, otherwise error.
 *
 *****************************************************************************/

signed long nvmem_read(unsigned long ulFileId, unsigned long ulLength,
                       unsigned long ulOffset, uint8_t *buff);

/******************************************************************************
 * Name: nvmem_write
 *
 * Description:
 *   Write data to nvmem. Writes data to file referred by the ulFileId
 *   parameter. Writes data to file ulOffset till ulLength. The file id will be
 *   marked invalid till the write is done. The file entry doesn't need to be
 *   valid - only allocated.
 *
 * Input Parameters:
 *   ulFileId       nvmem file id:
 *                    NVMEM_WLAN_DRIVER_SP_FILEID, NVMEM_WLAN_FW_SP_FILEID,
 *                    NVMEM_MAC_FILEID, NVMEM_BOOTLOADER_SP_FILEID,
 *                    and user files 12-15.
 *   ulLength       number of bytes to write
 *   ulEntryOffset  offset in file to start write operation from
 *   buff           data to write
 *
 * Returned Value:
 *   On success 0, error otherwise.
 *
 *****************************************************************************/

signed long nvmem_write(unsigned long ulFileId, unsigned long ulLength,
                        unsigned long ulEntryOffset, uint8_t *buff);

/******************************************************************************
 * Name: nvmem_set_mac_address
 *
 * Description:
 *   Write MAC address to EEPROM. mac address as appears over the air (OUI
 *   first)
 *
 * Input Parameters:
 *   mac   mac address to be set
 *
 * Returned Value:
 *   On success 0, error otherwise.
 *
 *****************************************************************************/

uint8_t nvmem_set_mac_address(uint8_t *mac);

/******************************************************************************
 * Name: nvmem_get_mac_address
 *
 * Description:
 *   Read MAC address from EEPROM. mac address as appears over the air (OUI
 *   first)
 *
 * Input Parameters:
 *   mac   mac address
 *
 * Returned Value:
 *   On success 0, error otherwise.
 *
 *****************************************************************************/

uint8_t nvmem_get_mac_address(uint8_t *mac);

/******************************************************************************
 * Name: nvmem_write_patch
 *
 * Description:
 *   Program a patch to a specific file ID. The SP data is assumed to be
 *   organized in 2-dimensional. Each line is SP_PORTION_SIZE bytes long.
 *   Actual programming is applied in SP_PORTION_SIZE bytes portions.
 *
 * Input Parameters:
 *   ulFileId   nvmem file id:
 *                NVMEM_WLAN_DRIVER_SP_FILEID, NVMEM_WLAN_FW_SP_FILEID,
 *   spLength   number of bytes to write
 *   spData     SP data to write
 *
 * Returned Value:
 *   On success 0, error otherwise.
 *
 *****************************************************************************/

uint8_t nvmem_write_patch(unsigned long ulFileId, unsigned long spLength,
                          const uint8_t *spData);

/******************************************************************************
 * Name: nvmem_read_sp_version
 *
 * Description:
 *   Read patch version. read package version (WiFi FW patch,
 *   driver-supplicant-NS patch, bootloader patch)
 *
 * Input Parameters:
 *   patchVer    first number indicates package ID and the second
 *                 number indicates package build number
 *
 * Returned Value:
 *   On success  0, error otherwise.
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
uint8_t nvmem_read_sp_version(uint8_t *patchVer);
#endif

/******************************************************************************
 * Name: nvmem_create_entry
 *
 * Description:
 *   Create new file entry and allocate space on the NVMEM. Applies only to
 *   user files. Modify the size of file. If the entry is unallocated -
 *   allocate it to size ulNewLen (marked invalid). If it is allocated then
 *   deallocate it first. To just mark the file as invalid without resizing -
 *   Set ulNewLen=0.
 *
 * Input Parameters:
 *   ulFileId    nvmem file Id:
 *               * NVMEM_AES128_KEY_FILEID: 12
 *               * NVMEM_SHARED_MEM_FILEID: 13
 *               * and fileIDs 14 and 15
 *   ulNewLen    entry ulLength
 *
 * Returned Value:
 *   On success 0, error otherwise.
 *
 *****************************************************************************/

signed long nvmem_create_entry(unsigned long ulFileId, unsigned long ulNewLen);

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // _INCLUDE_NUTTX_WIRELESS_CC3000_NVMEM_H
