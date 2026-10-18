/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_progmem.c
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

/* Program flash storage for the PIC32MZ-W1.
 *
 * The last CONFIG_PIC32MZ_PROGMEM_NPAGES pages of the program flash are
 * kept out of the program image (see the linker script of the board) and
 * exposed through the progmem interface, which drivers/mtd/mtd_progmem.c
 * turns into an MTD device.
 *
 * Facts used below, with their source:
 *   [DS]  DS70005425 (PIC32MZ-W1 / WFI32E01 data sheet), Flash Program
 *         Memory chapter: 1 MB of flash, 4 KB pages, 1 KB rows, the
 *         NVMCON/NVMADDR/NVMSRCADDR/NVMKEY registers, the NVMOP codes,
 *         ECC written by the row and quad word operations.
 *   [DFP] PIC32MZ-W_DFP header pwfi32e01.h: register addresses and bits.
 *   [EX]  Microchip's Harmony NVM peripheral library (the example code of
 *         the reference design): the unlock sequence written to NVMKEY.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/progmem.h>

#include "mips_internal.h"

#include "hardware/pic32mz_memorymap.h"
#include "hardware/pic32mzw1_nvm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROGMEM_FLASHSIZE   (1024 * 1024)  /* [DS] Program flash size */
#define PROGMEM_PAGESIZE    4096           /* [DS] Erase size */
#define PROGMEM_ROWSIZE     1024           /* [DS] Program size */
#define PROGMEM_ERASED      0xff

#define PROGMEM_NPAGES      CONFIG_PIC32MZ_PROGMEM_NPAGES
#define PROGMEM_SIZE        (PROGMEM_NPAGES * PROGMEM_PAGESIZE)
#define PROGMEM_NROWS       (PROGMEM_SIZE / PROGMEM_ROWSIZE)

/* Start of the area, as seen through the cached and uncached windows, and
 * its physical address.  The flash is always read through the uncached
 * window, since the cache is not aware of the operations of the NVM.
 */

#define PROGMEM_OFFSET      (PROGMEM_FLASHSIZE - PROGMEM_SIZE)
#define PROGMEM_K0START     (PIC32MZ_PROGFLASH_K0BASE + PROGMEM_OFFSET)
#define PROGMEM_K1START     (PIC32MZ_PROGFLASH_K1BASE + PROGMEM_OFFSET)
#define PROGMEM_PSTART      (PIC32MZ_PROGFLASH_PBASE + PROGMEM_OFFSET)

#define PHYS_ADDR(va)       ((uint32_t)(va) & 0x1fffffff)

/* A flash operation takes a few milliseconds */

#define PROGMEM_TIMEOUT_US  200000
#define PROGMEM_POLL_US     20

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_progmem_lock = NXMUTEX_INITIALIZER;

/* The source of a row program operation is a physical address read by the
 * NVM itself: use a buffer that is aligned and flushed from the cache,
 * whatever the buffer of the caller is.
 */

static uint8_t g_rowbuf[PROGMEM_ROWSIZE] aligned_data(4);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: progmem_offset
 *
 * Description:
 *   Convert an address, in the cached window or relative to the start of the
 *   area, to an offset in the area.
 *
 * Returned Value:
 *   The offset, or a negated errno value if the address is not in the area.
 *
 ****************************************************************************/

static ssize_t progmem_offset(size_t addr)
{
  if (addr >= PROGMEM_K0START && addr < PROGMEM_K0START + PROGMEM_SIZE)
    {
      return addr - PROGMEM_K0START;
    }

  if (addr < PROGMEM_SIZE)
    {
      return addr;
    }

  return -EFAULT;
}

/****************************************************************************
 * Name: progmem_invalidate
 *
 * Description:
 *   Drop from the data cache the lines of a range of the area that the NVM
 *   changed, for those that read it through the cached window.
 *
 ****************************************************************************/

static void progmem_invalidate(size_t offset, size_t size)
{
  up_invalidate_dcache(PROGMEM_K0START + offset,
                       PROGMEM_K0START + offset + size);
}

/****************************************************************************
 * Name: progmem_operation
 *
 * Description:
 *   Run one operation of the NVM, with the unlock sequence, and wait for
 *   its end.
 *
 ****************************************************************************/

static int progmem_operation(uint32_t op, uint32_t paddr)
{
  irqstate_t flags;
  uint32_t regval;
  int timeout;

  /* NVMOP can only be written while WREN is clear, and the unlock sequence
   * must not be interrupted.
   */

  flags = up_irq_save();

  putreg32(paddr, PIC32MZ_NVM_ADDR);
  putreg32(NVM_CON_WREN, PIC32MZ_NVM_CONCLR);
  putreg32(NVM_CON_NVMOP_MASK, PIC32MZ_NVM_CONCLR);
  putreg32(op, PIC32MZ_NVM_CONSET);
  putreg32(NVM_CON_WREN, PIC32MZ_NVM_CONSET);

  putreg32(0, PIC32MZ_NVM_KEY);
  putreg32(NVM_KEY1, PIC32MZ_NVM_KEY);
  putreg32(NVM_KEY2, PIC32MZ_NVM_KEY);
  putreg32(NVM_CON_WR, PIC32MZ_NVM_CONSET);

  up_irq_restore(flags);

  /* The hardware clears WR when the operation is done */

  for (timeout = PROGMEM_TIMEOUT_US; timeout > 0; timeout -= PROGMEM_POLL_US)
    {
      if ((getreg32(PIC32MZ_NVM_CON) & NVM_CON_WR) == 0)
        {
          break;
        }

      up_udelay(PROGMEM_POLL_US);
    }

  regval = getreg32(PIC32MZ_NVM_CON);
  putreg32(NVM_CON_WREN, PIC32MZ_NVM_CONCLR);

  if (timeout <= 0 || (regval & (NVM_CON_WRERR | NVM_CON_LVDERR)) != 0)
    {
      /* The errors are cleared by a no operation command */

      if (op != NVM_CON_NVMOP_NOP)
        {
          progmem_operation(NVM_CON_NVMOP_NOP, paddr);
        }

      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_progmem_neraseblocks
 ****************************************************************************/

size_t up_progmem_neraseblocks(void)
{
  return PROGMEM_NPAGES;
}

/****************************************************************************
 * Name: up_progmem_isuniform
 ****************************************************************************/

bool up_progmem_isuniform(void)
{
  return false;
}

/****************************************************************************
 * Name: up_progmem_pagesize
 *
 * Description:
 *   The read/write page is the row of the flash.
 *
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  return page < PROGMEM_NROWS ? PROGMEM_ROWSIZE : 0;
}

/****************************************************************************
 * Name: up_progmem_erasesize
 ****************************************************************************/

size_t up_progmem_erasesize(size_t block)
{
  return block < PROGMEM_NPAGES ? PROGMEM_PAGESIZE : 0;
}

/****************************************************************************
 * Name: up_progmem_getpage
 ****************************************************************************/

ssize_t up_progmem_getpage(size_t addr)
{
  ssize_t offset = progmem_offset(addr);

  return offset < 0 ? offset : offset / PROGMEM_ROWSIZE;
}

/****************************************************************************
 * Name: up_progmem_getaddress
 ****************************************************************************/

size_t up_progmem_getaddress(size_t page)
{
  return page < PROGMEM_NROWS ?
         PROGMEM_K0START + page * PROGMEM_ROWSIZE : SIZE_MAX;
}

/****************************************************************************
 * Name: up_progmem_eraseblock
 ****************************************************************************/

ssize_t up_progmem_eraseblock(size_t block)
{
  int ret;

  if (block >= PROGMEM_NPAGES)
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_progmem_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = progmem_operation(NVM_CON_NVMOP_PAGEERASE,
                          PROGMEM_PSTART + block * PROGMEM_PAGESIZE);
  progmem_invalidate(block * PROGMEM_PAGESIZE, PROGMEM_PAGESIZE);

  nxmutex_unlock(&g_progmem_lock);
  return ret < 0 ? ret : PROGMEM_PAGESIZE;
}

/****************************************************************************
 * Name: up_progmem_ispageerased
 ****************************************************************************/

ssize_t up_progmem_ispageerased(size_t page)
{
  const uint8_t *row;
  ssize_t nwritten = 0;
  size_t i;

  if (page >= PROGMEM_NROWS)
    {
      return -EFAULT;
    }

  row = (const uint8_t *)(PROGMEM_K1START + page * PROGMEM_ROWSIZE);
  for (i = 0; i < PROGMEM_ROWSIZE; i++)
    {
      if (row[i] != PROGMEM_ERASED)
        {
          nwritten++;
        }
    }

  return nwritten;
}

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program whole rows of an erased area.
 *
 ****************************************************************************/

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  const uint8_t *src = buf;
  ssize_t offset;
  size_t done;
  int ret;

  offset = progmem_offset(addr);
  if (offset < 0)
    {
      return offset;
    }

  if ((offset % PROGMEM_ROWSIZE) != 0 || (count % PROGMEM_ROWSIZE) != 0)
    {
      return -EINVAL;
    }

  if (offset + count > PROGMEM_SIZE)
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_progmem_lock);
  if (ret < 0)
    {
      return ret;
    }

  for (done = 0; done < count; done += PROGMEM_ROWSIZE)
    {
      memcpy(g_rowbuf, src + done, PROGMEM_ROWSIZE);
      up_flush_dcache((uintptr_t)g_rowbuf,
                      (uintptr_t)g_rowbuf + PROGMEM_ROWSIZE);

      putreg32(PHYS_ADDR(g_rowbuf), PIC32MZ_NVM_SRCADDR);
      ret = progmem_operation(NVM_CON_NVMOP_ROW,
                              PROGMEM_PSTART + offset + done);
      if (ret < 0)
        {
          break;
        }
    }

  progmem_invalidate(offset, count);

  nxmutex_unlock(&g_progmem_lock);
  return ret < 0 ? ret : (ssize_t)count;
}

/****************************************************************************
 * Name: up_progmem_read
 ****************************************************************************/

ssize_t up_progmem_read(size_t addr, void *buf, size_t count)
{
  ssize_t offset = progmem_offset(addr);

  if (offset < 0)
    {
      return offset;
    }

  if (offset + count > PROGMEM_SIZE)
    {
      return -EFAULT;
    }

  memcpy(buf, (const void *)(PROGMEM_K1START + offset), count);
  return count;
}

/****************************************************************************
 * Name: up_progmem_erasestate
 ****************************************************************************/

uint8_t up_progmem_erasestate(void)
{
  return PROGMEM_ERASED;
}
