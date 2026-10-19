/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_progmem.c
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

#include "arm_internal.h"
#include "gd32w51x_progmem.h"

#include "gd32w51x_memorymap.h"
#include "hardware/gd32w51x_fmc.h"

#ifdef CONFIG_GD32W51X_PROGMEM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IMPORTANT:  The internal flash of the GD32W51x is a system-in-package
 * NOR flash that is executed in place.  It also holds the running firmware
 * image (NuttX is linked at 0x08000000).
 *
 * Only the region delimited by CONFIG_GD32W51X_PROGMEM_START_ADDR and
 * CONFIG_GD32W51X_PROGMEM_SIZE is exposed here, and the board is
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

/* NuttX runs in secure state and the flash pages are marked secure
 * (FMC_SECMCFG0 covers the whole array): they are erased and programmed
 * through the secure register set of the FMC, reached through its secure
 * alias, and written through the secure alias of the flash.
 */

#define FMC_S(off) \
  ((off) + GD32W51X_FMC_BASE + GD32W51X_SECURE_ALIAS)
#define FMC_SECKEY_S        FMC_S(0x0024)
#define FMC_SECSTAT_S       FMC_S(0x002c)
#define FMC_SECCTL_S        FMC_S(0x0030)
#define FMC_SECADDR_S       FMC_S(0x0034)

#define FMC_SECSTAT_BUSY    (1 << 0)
#define FMC_SECSTAT_ERR     (1 << 3)
#define FMC_SECSTAT_WPERR   (1 << 4)
#define FMC_SECSTAT_ENDF    (1 << 5)

#define FMC_SECCTL_PG       (1 << 0)
#define FMC_SECCTL_PER      (1 << 1)
#define FMC_SECCTL_START    (1 << 6)
#define FMC_SECCTL_LK       (1 << 7)

#define FMC_UNLOCK_KEY0     0x45670123
#define FMC_UNLOCK_KEY1     0xcdef89ab

#define FLASH_S(off)        (GD32W51X_FLASH_S_BASE + (off))

/* Instruction cache in front of the flash */

#define ICACHE_CTL          (GD32W51X_ICACHE_BASE + 0x0000)
#define ICACHE_STAT         (GD32W51X_ICACHE_BASE + 0x0004)
#define ICACHE_FC           (GD32W51X_ICACHE_BASE + 0x000c)
#define ICACHE_CTL_EN       (1 << 0)
#define ICACHE_CTL_INVAL    (1 << 1)
#define ICACHE_STAT_END     (1 << 1)

/****************************************************************************
 * Name: gd32_fmc_wait
 *
 * Description:
 *   Wait for the end of an FMC operation and clear its status flags.
 *
 ****************************************************************************/

static int gd32_fmc_wait(void)
{
  uint32_t timeout = FMC_TIMEOUT_COUNT;
  uint32_t stat;

  while ((getreg32(FMC_SECSTAT_S) & FMC_SECSTAT_BUSY) != 0)
    {
      if (--timeout == 0)
        {
          return -ETIMEDOUT;
        }
    }

  stat = getreg32(FMC_SECSTAT_S);
  putreg32(FMC_SECSTAT_ERR | FMC_SECSTAT_WPERR | FMC_SECSTAT_ENDF,
           FMC_SECSTAT_S);

  return (stat & (FMC_SECSTAT_ERR | FMC_SECSTAT_WPERR)) != 0 ? -EIO : OK;
}

/****************************************************************************
 * Name: gd32_fmc_unlock and gd32_fmc_lock
 ****************************************************************************/

static void gd32_fmc_unlock(void)
{
  if ((getreg32(FMC_SECCTL_S) & FMC_SECCTL_LK) != 0)
    {
      putreg32(FMC_UNLOCK_KEY0, FMC_SECKEY_S);
      putreg32(FMC_UNLOCK_KEY1, FMC_SECKEY_S);
    }
}

static void gd32_fmc_lock(void)
{
  modifyreg32(FMC_SECCTL_S, 0, FMC_SECCTL_LK);
}

/****************************************************************************
 * Name: gd32_icache_invalidate
 *
 * Description:
 *   Drop the instruction cache after the flash contents changed.  The
 *   cache must be disabled while it is invalidated.
 *
 ****************************************************************************/

static void gd32_icache_invalidate(void)
{
  uint32_t ctl = getreg32(ICACHE_CTL);

  putreg32(ctl & ~ICACHE_CTL_EN, ICACHE_CTL);
  putreg32(ICACHE_STAT_END, ICACHE_FC);
  modifyreg32(ICACHE_CTL, 0, ICACHE_CTL_INVAL);

  while ((getreg32(ICACHE_STAT) & ICACHE_STAT_END) == 0)
    {
    }

  putreg32(ICACHE_STAT_END, ICACHE_FC);
  putreg32(ctl, ICACHE_CTL);
}

/****************************************************************************
 * Name: gd32_flash_erase
 *
 * Description:
 *   Erase the pages covering a run of the flash.  Offsets are relative to
 *   the start of the flash array, not absolute addresses.
 *
 ****************************************************************************/

static int gd32_flash_erase(uint32_t offset, size_t len)
{
  irqstate_t flags;
  uint32_t page;
  int ret = OK;

  flags = enter_critical_section();
  gd32_fmc_unlock();

  for (page = offset & ~(GD32W51X_FLASH_PAGE_SIZE - 1);
       page < offset + len && ret == OK;
       page += GD32W51X_FLASH_PAGE_SIZE)
    {
      modifyreg32(FMC_SECCTL_S, 0, FMC_SECCTL_PER);
      putreg32(FLASH_S(page), FMC_SECADDR_S);
      modifyreg32(FMC_SECCTL_S, 0, FMC_SECCTL_START);
      ret = gd32_fmc_wait();
      modifyreg32(FMC_SECCTL_S, FMC_SECCTL_PER, 0);
    }

  gd32_fmc_lock();
  gd32_icache_invalidate();
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: gd32_flash_write
 *
 * Description:
 *   Program a run of the flash, one 32-bit word at a time.  The unaligned
 *   head and tail are padded with 0xff, which leaves those bytes unchanged.
 *
 ****************************************************************************/

static int gd32_flash_write(uint32_t offset, const void *buf, size_t len)
{
  const uint8_t *src = buf;
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();
  gd32_fmc_unlock();

  while (len > 0 && ret == OK)
    {
      uint32_t word = 0xffffffff;
      uint32_t base = offset & ~3;
      size_t shift = offset & 3;
      size_t n = 4 - shift;

      if (n > len)
        {
          n = len;
        }

      memcpy((uint8_t *)&word + shift, src, n);

      modifyreg32(FMC_SECCTL_S, 0, FMC_SECCTL_PG);
      putreg32(word, FLASH_S(base));
      ret = gd32_fmc_wait();
      modifyreg32(FMC_SECCTL_S, FMC_SECCTL_PG, 0);

      offset += n;
      src    += n;
      len    -= n;
    }

  gd32_fmc_lock();
  gd32_icache_invalidate();
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: gd32_flash_read
 ****************************************************************************/

static int gd32_flash_read(uint32_t offset, void *buf, size_t len)
{
  memcpy(buf, (const void *)FLASH_S(offset), len);
  return OK;
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
  return GD32W51X_PROGMEM_NPAGES;
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
  return GD32W51X_PROGMEM_PAGE_SIZE;
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
  return GD32W51X_PROGMEM_PAGE_SIZE;
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

  if (addr < GD32W51X_PROGMEM_START)
    {
      addr += GD32W51X_PROGMEM_START;
    }

  if (addr < GD32W51X_PROGMEM_START || addr >= GD32W51X_PROGMEM_END)
    {
      return -EFAULT;
    }

  return (addr - GD32W51X_PROGMEM_START) / GD32W51X_PROGMEM_PAGE_SIZE;
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
  if (page >= GD32W51X_PROGMEM_NPAGES)
    {
      return SIZE_MAX;
    }

  return GD32W51X_PROGMEM_START + page * GD32W51X_PROGMEM_PAGE_SIZE;
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

  if (block >= GD32W51X_PROGMEM_NPAGES)
    {
      return -EFAULT;
    }

  addr = up_progmem_getaddress(block);

  ret = nxmutex_lock(&g_progmem_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_flash_erase(addr - GD32W51X_FLASH_BASE,
                         GD32W51X_PROGMEM_PAGE_SIZE);
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

  return GD32W51X_PROGMEM_PAGE_SIZE;
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

  if (page >= GD32W51X_PROGMEM_NPAGES)
    {
      return -EFAULT;
    }

  ptr = (const uint8_t *)up_progmem_getaddress(page);

  for (count = 0; count < GD32W51X_PROGMEM_PAGE_SIZE; count++)
    {
      if (ptr[count] != GD32W51X_FLASH_ERASEDVAL)
        {
          nerased++;
        }
    }

  return nerased;
}

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at the given address
 *
 *   Note: this function is not limited to a single page, nor does it
 *   require the address to be aligned inside the page boundaries.  The
 *   word granularity of the flash is handled by gd32_flash_write(), so any
 *   length is accepted.
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

  if (addr < GD32W51X_PROGMEM_START)
    {
      addr += GD32W51X_PROGMEM_START;
    }

  /* Check for a valid address range */

  if (addr < GD32W51X_PROGMEM_START ||
      (addr + count) > GD32W51X_PROGMEM_END)
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

  ret = gd32_flash_write(addr - GD32W51X_FLASH_BASE, buf, count);

  nxmutex_unlock(&g_progmem_lock);

  if (ret < 0)
    {
      return ret;
    }

  return count;
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

  if (addr < GD32W51X_PROGMEM_START)
    {
      addr += GD32W51X_PROGMEM_START;
    }

  /* Check for a valid address range */

  if (addr < GD32W51X_PROGMEM_START ||
      (addr + count) > GD32W51X_PROGMEM_END)
    {
      return -EFAULT;
    }

  /* The flash is memory mapped and executed in place */

  if (gd32_flash_read(addr - GD32W51X_FLASH_BASE, buf, count) < 0)
    {
      return -EIO;
    }

  return count;
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
  return GD32W51X_FLASH_ERASEDVAL;
}

#endif /* CONFIG_GD32W51X_PROGMEM */
