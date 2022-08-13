/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_progmem.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * See NXP UM10360 LPC176x/5x User manual, Rev 4.1, Chapter 32: LPC176x/5x
 * Flash memory interface and programming.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include <arch/chip/chip.h>
#include <arch/board/board.h>
#include <nuttx/progmem.h>
#include <nuttx/irq.h>

#include "lpc17_40_progmem.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void lpc17_40_iap(void *in, void *out);
static uint32_t
lpc17_40_iap_prepare_sector_for_write_operation(uint32_t sector);
static uint32_t lpc17_40_iap_erase_sector(uint32_t sector);
static uint32_t lpc17_40_iap_copy_ram_to_flash(void *flash,
                                               const void *ram,
                                               size_t count);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_iap
 *
 * Description (from UM10360):
 *   For in-application programming the IAP routine should be called with a
 *   word pointer in register r0 pointing to memory (RAM) containing command
 *   code and parameters. The result from the IAP command is returned in the
 *   table pointed to by register r1. The user can reuse the command table
 *   for the result by passing the same pointer in registers r0 and r1.
 *
 ****************************************************************************/

static void lpc17_40_iap(void *in, void *out)
{
  irqstate_t flags;

  flags = enter_critical_section();

  ((void (*)(void *, void *))LPC17_40_IAP_ENTRY_ADDR)(in, out);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc17_40_iap_prepare_sector_for_write_operation
 *
 * Description (from UM10360):
 *   This command must be executed before executing "Copy RAM to Flash" or
 *   "Erase Sector(s)" command.  Successful execution of the "Copy RAM to
 *   Flash" or "Erase Sector(s)" command causes relevant sectors to be
 *   protected again.
 *   To prepare a single sector use the same "Start" and "End" sector
 *   numbers.
 *
 ****************************************************************************/

static uint32_t
lpc17_40_iap_prepare_sector_for_write_operation(uint32_t sector)
{
  uint32_t inout[3];

  inout[0] = LPC17_40_IAP_CMD_PREPARE_SECTORS_FOR_WRITE_OPERATION;
  inout[1] = sector;
  inout[2] = sector;

  lpc17_40_iap(inout, inout);

  return inout[0];
}

/****************************************************************************
 * Name: lpc17_40_iap_erase_sector
 *
 * Description (from UM10360):
 *   This command is used to erase a sector or multiple sectors of on-chip
 *   flash memory. To erase a single sector use the same "Start" and "End"
 *   sector numbers.
 *
 ****************************************************************************/

static uint32_t lpc17_40_iap_erase_sector(uint32_t sector)
{
  uint32_t inout[4];

  inout[0] = LPC17_40_IAP_CMD_ERASE_SECTORS;
  inout[1] = sector;
  inout[2] = sector;
  inout[3] = LPC17_40_CCLK / 1000;

  lpc17_40_iap(inout, inout);

  return inout[0];
}

/****************************************************************************
 * Name: lpc17_40_iap_copy_ram_to_flash
 *
 * Description (from UM10360):
 *   This command is used to program the flash memory. The affected sectors
 *   should be prepared first by calling "Prepare Sector for Write Operation"
 *   command. The affected sectors are automatically protected again once the
 *   copy command is successfully executed.
 *
 ****************************************************************************/

static uint32_t lpc17_40_iap_copy_ram_to_flash(void *flash, const void *ram,
                                            size_t count)
{
  uint32_t inout[5];

  inout[0] = LPC17_40_IAP_CMD_COPY_RAM_TO_FLASH;
  inout[1] = (uint32_t)flash;
  inout[2] = (uint32_t)ram;
  inout[3] = (uint32_t)count;
  inout[4] = LPC17_40_CCLK / 1000;

  lpc17_40_iap(inout, inout);

  return inout[0];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_progmem_neraseblocks
 *
 * Description:
 *   Return number of erase blocks
 *
 ****************************************************************************/

size_t up_progmem_neraseblocks(void)
{
  return LPC17_40_FLASH_NUM_SECTORS;
}

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *   Is program memory uniform or page size differs?
 *
 ****************************************************************************/

bool up_progmem_isuniform(void)
{
  return false;
}

/****************************************************************************
 * Name: up_progmem_pagesize
 *
 * Description:
 *   Return read/write page size
 *
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  return up_progmem_erasesize(page);
}

/****************************************************************************
 * Name: up_progmem_erasesize
 *
 * Description:
 *   Return erase block size
 *
 ****************************************************************************/

size_t up_progmem_erasesize(size_t block)
{
  if (block >= LPC17_40_FLASH_NUM_SECTORS)
    {
      return 0;
    }

  if (block < LPC17_40_FLASH_NUM_4K_SECTORS)
    {
      return (size_t)(4 * 1024);
    }
  else
    {
      return (size_t)(32 * 1024);
    }
}

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to read/write page conversion
 *
 * Input Parameters:
 *   addr - Address with or without flash offset
 *          (absolute or aligned to page0)
 *
 * Returned Value:
 *   Page or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     -EFAULT: On invalid address
 *
 ****************************************************************************/

ssize_t up_progmem_getpage(size_t addr)
{
  ssize_t page;

  if (addr >= LPC17_40_FLASH_SIZE)
    {
      return -EFAULT;
    }

  if (addr < (LPC17_40_FLASH_NUM_4K_SECTORS * (4 * 1024)))
    {
      page = addr / (4 * 1024);
    }
  else
    {
      page = LPC17_40_FLASH_NUM_4K_SECTORS +
              ((addr - (LPC17_40_FLASH_NUM_4K_SECTORS *
              (4 * 1024))) / (32 * 1024));
    }

  return page;
}

/****************************************************************************
 * Name: up_progmem_getaddress
 *
 * Description:
 *   Read/write page to address conversion
 *
 * Input Parameters:
 *   page - page index
 *
 * Returned Value:
 *   Base address of given page, SIZE_MAX if page index is not valid.
 *
 ****************************************************************************/

size_t up_progmem_getaddress(size_t page)
{
  size_t addr;

  if (page >= LPC17_40_FLASH_NUM_SECTORS)
    {
      return SIZE_MAX;
    }

  if (page < LPC17_40_FLASH_NUM_4K_SECTORS)
    {
      addr = page * (4 * 1024);
    }
  else
    {
      addr = (LPC17_40_FLASH_NUM_4K_SECTORS * (4 * 1024)) +
              ((page - LPC17_40_FLASH_NUM_4K_SECTORS) * (32 * 1024));
    }

  return addr;
}

/****************************************************************************
 * Name: up_progmem_eraseblock
 *
 * Description:
 *   Erase selected block.
 *
 * Input Parameters:
 *   block - The erase block index to be erased.
 *
 * Returned Value:
 *   block size or negative value on error.  The following errors are
 *   reported (errno is not set!):
 *
 *     -EFAULT: On invalid page
 *     -EIO:    On unsuccessful erase
 *     -EROFS:  On access to write protected area
 *     -EACCES: Insufficient permissions (read/write protected)
 *     -EPERM:  If operation is not permitted due to some other constraints
 *              (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_eraseblock(size_t block)
{
  uint32_t rc;

  if (block >= LPC17_40_FLASH_NUM_SECTORS)
    {
      return -EFAULT;
    }

  rc = lpc17_40_iap_prepare_sector_for_write_operation((uint32_t)block);
  if (rc != LPC17_40_IAP_RC_CMD_SUCCESS)
    {
      return -EIO;
    }

  rc = lpc17_40_iap_erase_sector((uint32_t)block);

  if (rc != LPC17_40_IAP_RC_CMD_SUCCESS)
    {
      return -EIO;
    }

  return (ssize_t)up_progmem_erasesize(block);
}

/****************************************************************************
 * Name: up_progmem_ispageerased
 *
 * Description:
 *   Checks whether page is erased
 *
 * Input Parameters:
 *   page - The erase page index to be checked.
 *
 * Returned Value:
 *   Returns number of bytes NOT erased or negative value on error. If it
 *   returns zero then complete page is erased.
 *
 *   The following errors are reported:
 *     -EFAULT: On invalid page
 *
 ****************************************************************************/

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t page_size;
  const uint8_t *p;
  int i;

  if (page >= LPC17_40_FLASH_NUM_SECTORS)
    {
      return -EFAULT;
    }

  page_size = up_progmem_pagesize(page);
  p = (const uint8_t *)up_progmem_getaddress(page);

  for (i = 0; i < page_size; i++)
    {
      if (p[i] != LPC17_40_FLASH_ERASEDVAL)
        {
          break;
        }
    }

  return (ssize_t)(page_size - i);
}

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at given address
 *
 *   Note: this function is not limited to single page and nor it requires
 *   the address be aligned inside the page boundaries.
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *           (absolute or aligned to page0)
 *   buf   - Pointer to buffer
 *   count - Number of bytes to write
 *
 * Returned Value:
 *   Bytes written or negative value on error.  The following errors are
 *   reported (errno is not set!)
 *
 *     EINVAL: If count is not aligned with the flash boundaries (i.e.
 *             some MCU's require per half-word or even word access)
 *     EFAULT: On invalid address
 *     EIO:    On unsuccessful write
 *     EROFS:  On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  size_t page;
  uint32_t rc;

  if (addr % LPC17_40_WRITE_SIZE)
    {
      return -EFAULT;
    }

  if (count % LPC17_40_WRITE_SIZE)
    {
      return -EINVAL;
    }

  page = up_progmem_getpage(addr);

  rc = lpc17_40_iap_prepare_sector_for_write_operation((uint32_t)page);

  if (rc != LPC17_40_IAP_RC_CMD_SUCCESS)
    {
      return -EIO;
    }

  rc = lpc17_40_iap_copy_ram_to_flash((void *)addr, buf, count);

  if (rc != LPC17_40_IAP_RC_CMD_SUCCESS)
    {
      return -EIO;
    }

  return count;
}

/****************************************************************************
 * Name: up_progmem_erasestate
 *
 * Description:
 *   Return value of erase state.
 *
 ****************************************************************************/

uint8_t up_progmem_erasestate(void)
{
  return LPC17_40_FLASH_ERASEDVAL;
}
