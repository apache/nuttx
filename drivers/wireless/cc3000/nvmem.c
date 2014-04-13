/*****************************************************************************
 *  nvmem.c  - CC3000 Host Driver Implementation.
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

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

#include <nuttx/wireless/cc3000/nvmem.h>
#include <nuttx/wireless/cc3000/hci.h>
#include <nuttx/wireless/cc3000/evnt_handler.h>
#include "cc3000.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#define NVMEM_READ_PARAMS_LEN   (12)
#define NVMEM_CREATE_PARAMS_LEN   (8)
#define NVMEM_WRITE_PARAMS_LEN  (16)

/******************************************************************************
 * Public Functions
 ******************************************************************************/
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
                       unsigned long ulOffset, uint8_t *buff)
{
  uint8_t ucStatus = 0xff;
  uint8_t *ptr;
  uint8_t *args;

  cc3000_lib_lock();

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in HCI packet structure */

  args = UINT32_TO_STREAM(args, ulFileId);
  args = UINT32_TO_STREAM(args, ulLength);
  args = UINT32_TO_STREAM(args, ulOffset);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_NVMEM_READ, ptr, NVMEM_READ_PARAMS_LEN);
  SimpleLinkWaitEvent(HCI_CMND_NVMEM_READ, &ucStatus);

  /* In case there is data - read it - even if an error code is returned
   * Note: It is the user responsibility to ignore the data in case of an
   * error code
   */

  /* Wait for the data in a synchronous way. Here we assume that the buffer is
   * big enough to store also parameters of nvmem
   */

  SimpleLinkWaitData(buff, 0, &ucStatus);

  cc3000_lib_unlock();

  return ucStatus;
}

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
                        unsigned long ulEntryOffset, uint8_t *buff)
{
  long iRes;
  uint8_t *ptr;
  uint8_t *args;

  cc3000_lib_lock();

  iRes = EFAIL;

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + SPI_HEADER_SIZE + HCI_DATA_CMD_HEADER_SIZE);

  /* Fill in HCI packet structure */

  args = UINT32_TO_STREAM(args, ulFileId);
  args = UINT32_TO_STREAM(args, 12);
  args = UINT32_TO_STREAM(args, ulLength);
  args = UINT32_TO_STREAM(args, ulEntryOffset);

  memcpy((ptr + SPI_HEADER_SIZE + HCI_DATA_CMD_HEADER_SIZE +
          NVMEM_WRITE_PARAMS_LEN),buff,ulLength);

  /* Initiate a HCI command but it will come on data channel */

  hci_data_command_send(HCI_CMND_NVMEM_WRITE, ptr, NVMEM_WRITE_PARAMS_LEN,
                        ulLength);

  SimpleLinkWaitEvent(HCI_EVNT_NVMEM_WRITE, &iRes);

  cc3000_lib_unlock();

  return iRes;
}

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

uint8_t nvmem_set_mac_address(uint8_t *mac)
{
  return  nvmem_write(NVMEM_MAC_FILEID, MAC_ADDR_LEN, 0, mac);
}

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

uint8_t nvmem_get_mac_address(uint8_t *mac)
{
  return  nvmem_read(NVMEM_MAC_FILEID, MAC_ADDR_LEN, 0, mac);
}

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
                          const uint8_t *spData)
{
  uint8_t   status = 0;
  uint16_t  offset = 0;
  uint8_t  *spDataPtr = (uint8_t*)spData;

  while ((status == 0) && (spLength >= SP_PORTION_SIZE))
    {
      status = nvmem_write(ulFileId, SP_PORTION_SIZE, offset, spDataPtr);
      offset += SP_PORTION_SIZE;
      spLength -= SP_PORTION_SIZE;
      spDataPtr += SP_PORTION_SIZE;
    }

  if (status !=0)
    {
      /* NVMEM error occurred */

      return status;
    }

  if (spLength != 0)
    {
      /* if reached here, a reminder is left */

      status = nvmem_write(ulFileId, spLength, offset, spDataPtr);
    }

  return status;
}

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
uint8_t nvmem_read_sp_version(uint8_t *patchVer)
{
  uint8_t *ptr;
  /* 1st byte is the status and the rest is the SP version */
  uint8_t  retBuf[5];

  cc3000_lib_lock();

  ptr = tSLInformation.pucTxCommandBuffer;

  /* Initiate a HCI command, no args are required */

  hci_command_send(HCI_CMND_READ_SP_VERSION, ptr, 0);
  SimpleLinkWaitEvent(HCI_CMND_READ_SP_VERSION, retBuf);

  /* Package ID */

  *patchVer = retBuf[3];

  /* Package build number */

  *(patchVer+1) = retBuf[4];

  cc3000_lib_unlock();

  return retBuf[0];
}
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

signed long nvmem_create_entry(unsigned long ulFileId, unsigned long ulNewLen)
{
  uint8_t *ptr;
  uint8_t *args;
  uint16_t retval;

  cc3000_lib_lock();

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /*( Fill in HCI packet structure */

  args = UINT32_TO_STREAM(args, ulFileId);
  args = UINT32_TO_STREAM(args, ulNewLen);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_NVMEM_CREATE_ENTRY,ptr, NVMEM_CREATE_PARAMS_LEN);

  SimpleLinkWaitEvent(HCI_CMND_NVMEM_CREATE_ENTRY, &retval);

  cc3000_lib_unlock();

  return retval;
}
