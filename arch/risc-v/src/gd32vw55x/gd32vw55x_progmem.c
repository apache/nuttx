/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_progmem.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/progmem.h>

#include "riscv_internal.h"
#include "gd32vw55x_progmem.h"

/* The internal flash is a system-in-package NOR die: it is NOT programmed
 * through the FMC registers.  The vendor code always goes through the mask
 * ROM (raw_flash_api.c -> rom_flash_*), and so do we.  Poking FMC_CTL
 * directly does erase the array, but the programmed data never reads back:
 * the flash is behind the real-time decryption block (RTDEC), which the ROM
 * routines know how to handle and we do not.
 *
 * The ROM API table sits at a fixed address; rom_init() only sets up the
 * TRNG and mbedTLS, so the flash entry points are usable on their own.
 */

#include "rom_export.h"
#include "rom_region.h"

#ifdef CONFIG_GD32VW55X_PROGMEM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IMPORTANT:  The internal flash of the GD32VW55x is a system-in-package
 * NOR flash that is executed in place.  It holds:
 *
 *   - The running firmware image (NuttX is linked at 0x08000000),
 *   - The vendor NVDS area used by the Wi-Fi/BLE stacks, and
 *   - The RF/Wi-Fi trim (calibration) values programmed in the factory.
 *
 * Only the region delimited by CONFIG_GD32VW55X_PROGMEM_START_ADDR and
 * CONFIG_GD32VW55X_PROGMEM_SIZE is exposed here, and the board is
 * responsible for making sure that this region does not overlap any of the
 * above.  Erasing or programming the wrong region is destructive.
 *
 * NOTE also that the CPU fetches instructions from the very same flash
 * array.  The FMC stalls the AHB bus while an erase or a program operation
 * is in progress, so no code (including interrupt handlers) executes during
 * that time.  Interrupts are disabled around the erase/program sequences so
 * that no ISR can attempt to touch the flash while the FMC is busy.
 */

/* Timeout for the FMC busy flag, expressed as a poll count.  Erasing a
 * 4 KiB page takes tens of milliseconds in the worst case.
 */

#define FMC_TIMEOUT_COUNT  0x01000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_progmem_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_rom_api
 *
 * Description:
 *   The mask ROM API table.  Its address is fixed in ROM.
 *
 ****************************************************************************/

static inline struct rom_api_t *gd32_rom_api(void)
{
  return (struct rom_api_t *)ROM_API_ARRAY_BASE;
}

/****************************************************************************
 * Name: gd32_flash_erase
 *
 * Description:
 *   Erase a run of the flash through the mask ROM.  Offsets are relative to
 *   the start of the flash array, not absolute addresses.
 *
 ****************************************************************************/

static int gd32_flash_erase(uint32_t offset, int len)
{
  int ret;

  /* No critical section here: the ROM routines run from the mask ROM, not
   * from the flash being erased, and they need the system running (the
   * vendor's raw_flash_erase() does not disable interrupts either).
   * Holding interrupts off across them hangs the boot.
   */

  ret = gd32_rom_api()->flash_erase(offset, len);

  return ret == 0 ? OK : -EIO;
}

/****************************************************************************
 * Name: gd32_flash_write
 *
 * Description:
 *   Program a run of the flash through the mask ROM.  The ROM takes care of
 *   the word granularity, so any length is accepted.
 *
 ****************************************************************************/

static int gd32_flash_write(uint32_t offset, const void *buf, int len)
{
  int ret;

  ret = gd32_rom_api()->flash_write(offset, buf, len);

  return ret == 0 ? OK : -EIO;
}

/****************************************************************************
 * Name: gd32_flash_read
 ****************************************************************************/

static int gd32_flash_read(uint32_t offset, void *buf, int len)
{
  return gd32_rom_api()->flash_read(offset, buf, len) == 0 ? OK : -EIO;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_progmem_neraseblocks
 *
 * Description:
 *   Return the number of erase blocks
 *
 ****************************************************************************/

size_t up_progmem_neraseblocks(void)
{
  return GD32VW55X_PROGMEM_NPAGES;
}

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *   Is program memory uniform or does the page size differ?
 *
 ****************************************************************************/

bool up_progmem_isuniform(void)
{
  return true;
}

/****************************************************************************
 * Name: up_progmem_pagesize
 *
 * Description:
 *   Return the read/write page size
 *
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  return GD32VW55X_PROGMEM_PAGE_SIZE;
}

/****************************************************************************
 * Name: up_progmem_erasesize
 *
 * Description:
 *   Return the erase block size
 *
 ****************************************************************************/

size_t up_progmem_erasesize(size_t block)
{
  return GD32VW55X_PROGMEM_PAGE_SIZE;
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
  /* Accept both absolute addresses and offsets from the start of the
   * progmem region.
   */

  if (addr < GD32VW55X_PROGMEM_START)
    {
      addr += GD32VW55X_PROGMEM_START;
    }

  if (addr < GD32VW55X_PROGMEM_START || addr >= GD32VW55X_PROGMEM_END)
    {
      return -EFAULT;
    }

  return (ssize_t)((addr - GD32VW55X_PROGMEM_START) /
                   GD32VW55X_PROGMEM_PAGE_SIZE);
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
 *   Base address of the given page, SIZE_MAX if the page index is not
 *   valid.
 *
 ****************************************************************************/

size_t up_progmem_getaddress(size_t page)
{
  if (page >= GD32VW55X_PROGMEM_NPAGES)
    {
      return SIZE_MAX;
    }

  return GD32VW55X_PROGMEM_START + page * GD32VW55X_PROGMEM_PAGE_SIZE;
}

/****************************************************************************
 * Name: up_progmem_eraseblock
 *
 * Description:
 *   Erase the selected block.
 *
 * Input Parameters:
 *   block - The erase block index to be erased.
 *
 * Returned Value:
 *   Block size or a negative value on error.  The following errors are
 *   reported (errno is not set!):
 *
 *     -EFAULT: On invalid page
 *     -EIO:    On unsuccessful erase
 *     -EROFS:  On access to a write protected area
 *
 ****************************************************************************/

ssize_t up_progmem_eraseblock(size_t block)
{
  size_t addr;
  int ret;

  if (block >= GD32VW55X_PROGMEM_NPAGES)
    {
      return -EFAULT;
    }

  addr = up_progmem_getaddress(block);

  ret = nxmutex_lock(&g_progmem_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_flash_erase((uint32_t)addr - GD32VW55X_FLASH_BASE,
                         GD32VW55X_PROGMEM_PAGE_SIZE);
  if (ret < 0)
    {
      nxmutex_unlock(&g_progmem_lock);
      return ret;
    }

  nxmutex_unlock(&g_progmem_lock);

  /* Verify that the page was really erased */

  if (up_progmem_ispageerased(block) != 0)
    {
      return -EIO;
    }

  return (ssize_t)GD32VW55X_PROGMEM_PAGE_SIZE;
}

/****************************************************************************
 * Name: up_progmem_ispageerased
 *
 * Description:
 *   Check whether the page is erased
 *
 * Input Parameters:
 *   page - The erase page index to be checked.
 *
 * Returned Value:
 *   Returns the number of bytes NOT erased or a negative value on error.
 *   If it returns zero then the complete page is erased.
 *
 *   The following errors are reported:
 *     -EFAULT: On invalid page
 *
 ****************************************************************************/

ssize_t up_progmem_ispageerased(size_t page)
{
  const uint8_t *ptr;
  size_t count;
  size_t nerased = 0;

  if (page >= GD32VW55X_PROGMEM_NPAGES)
    {
      return -EFAULT;
    }

  ptr = (const uint8_t *)up_progmem_getaddress(page);

  for (count = 0; count < GD32VW55X_PROGMEM_PAGE_SIZE; count++)
    {
      if (ptr[count] != GD32VW55X_FLASH_ERASEDVAL)
        {
          nerased++;
        }
    }

  return (ssize_t)nerased;
}

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at the given address
 *
 *   Note: this function is not limited to a single page, nor does it
 *   require the address to be aligned inside the page boundaries.  The mask
 *   ROM handles the word granularity of the flash, so any length is
 *   accepted.
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *           (absolute or aligned to page0)
 *   buf   - Pointer to the buffer
 *   count - Number of bytes to write
 *
 * Returned Value:
 *   Bytes written or a negative value on error.  The following errors are
 *   reported (errno is not set!):
 *
 *     -EFAULT: On invalid address
 *     -EIO:    On unsuccessful write
 *     -EROFS:  On access to a write protected area
 *
 ****************************************************************************/

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  int ret;

  /* Accept both absolute addresses and offsets from the start of the
   * progmem region.
   */

  if (addr < GD32VW55X_PROGMEM_START)
    {
      addr += GD32VW55X_PROGMEM_START;
    }

  /* Check for a valid address range */

  if (addr < GD32VW55X_PROGMEM_START ||
      (addr + count) > GD32VW55X_PROGMEM_END)
    {
      return -EFAULT;
    }

  if (count == 0)
    {
      return 0;
    }

  ret = nxmutex_lock(&g_progmem_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_flash_write((uint32_t)addr - GD32VW55X_FLASH_BASE, buf,
                         (int)count);

  nxmutex_unlock(&g_progmem_lock);

  if (ret < 0)
    {
      return ret;
    }

  return (ssize_t)count;
}

/****************************************************************************
 * Name: up_progmem_read
 *
 * Description:
 *   Read data at the given address
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *           (absolute or aligned to page0)
 *   buf   - Pointer to the buffer
 *   count - Number of bytes to read
 *
 * Returned Value:
 *   Bytes read or a negative value on error.  The following errors are
 *   reported (errno is not set!):
 *
 *     -EFAULT: On invalid address
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_PROGMEM_READ
ssize_t up_progmem_read(size_t addr, void *buf, size_t count)
{
  /* Accept both absolute addresses and offsets from the start of the
   * progmem region.
   */

  if (addr < GD32VW55X_PROGMEM_START)
    {
      addr += GD32VW55X_PROGMEM_START;
    }

  /* Check for a valid address range */

  if (addr < GD32VW55X_PROGMEM_START ||
      (addr + count) > GD32VW55X_PROGMEM_END)
    {
      return -EFAULT;
    }

  /* The flash is memory mapped and executed in place */

  if (gd32_flash_read((uint32_t)addr - GD32VW55X_FLASH_BASE, buf,
                      (int)count) < 0)
    {
      return -EIO;
    }

  return (ssize_t)count;
}

#endif /* CONFIG_ARCH_HAVE_PROGMEM_READ */

/****************************************************************************
 * Name: up_progmem_erasestate
 *
 * Description:
 *   Return the value of the erase state.
 *
 ****************************************************************************/

uint8_t up_progmem_erasestate(void)
{
  return GD32VW55X_FLASH_ERASEDVAL;
}

#endif /* CONFIG_GD32VW55X_PROGMEM */
