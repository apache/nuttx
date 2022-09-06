/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_flash.c
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

/* Provides standard flash access functions, to be used by the flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 *
 * Notes about this implementation:
 *  - HSI16 is automatically turned ON by MCU, if not enabled beforehand
 *  - Only Standard Programming is supported, no Fast Programming.
 *  - Low Power Modes are not permitted during write/erase
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/progmem.h>
#include <nuttx/mutex.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include "arm_internal.h"
#include "stm32wb_rcc.h"
#include "stm32wb_waste.h"
#include "stm32wb_flash.h"

#if !defined(CONFIG_STM32WB_FLASH_OVERRIDE_DEFAULT)
#  warning "Flash Configuration has been overridden - make sure it is correct"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_KEY1         0x45670123
#define FLASH_KEY2         0xcdef89ab
#define FLASH_ERASEDVALUE  0xff

#define OPTBYTES_KEY1      0x08192a3b
#define OPTBYTES_KEY2      0x4c5d6e7f

#define FLASH_PAGE_SIZE    STM32WB_FLASH_PAGESIZE
#define FLASH_PAGE_WORDS   (FLASH_PAGE_SIZE / 4)
#define FLASH_PAGE_MASK    (FLASH_PAGE_SIZE - 1)
#define FLASH_PAGE_SHIFT   (12)    /* 2**12  = 4096B */
#define FLASH_BYTE2PAGE(o) ((o) >> FLASH_PAGE_SHIFT)

#define FLASH_CR_PAGE_ERASE              FLASH_CR_PER
#define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPERR

/* All errors for Standard Programming, not for other operations. */

#define FLASH_SR_ALLERRS   (FLASH_SR_PGSERR | FLASH_SR_SIZERR | \
                            FLASH_SR_PGAERR | FLASH_SR_WRPERR | \
                            FLASH_SR_PROGERR)

#ifndef MIN
#  define MIN(a, b)        ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_lock = NXMUTEX_INITIALIZER;
static uint32_t g_page_buffer[FLASH_PAGE_WORDS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void flash_unlock(void)
{
  while (getreg32(STM32WB_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32wb_waste();
    }

  if (getreg32(STM32WB_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32WB_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32WB_FLASH_KEYR);
    }
}

static void flash_lock(void)
{
  modifyreg32(STM32WB_FLASH_CR, 0, FLASH_CR_LOCK);
}

static void flash_optbytes_unlock(void)
{
  flash_unlock();

  if (getreg32(STM32WB_FLASH_CR) & FLASH_CR_OPTLOCK)
    {
      /* Unlock Option Bytes sequence */

      putreg32(OPTBYTES_KEY1, STM32WB_FLASH_OPTKEYR);
      putreg32(OPTBYTES_KEY2, STM32WB_FLASH_OPTKEYR);
    }
}

static inline void flash_optbytes_lock(void)
{
  /* We don't need to set OPTLOCK here as it is automatically
   * set by MCU when flash_lock() sets LOCK.
   */

  flash_lock();
}

static inline void flash_erase(size_t page)
{
  finfo("erase page %u\n", page);

  modifyreg32(STM32WB_FLASH_CR, 0, FLASH_CR_PAGE_ERASE);
  modifyreg32(STM32WB_FLASH_CR, FLASH_CR_PNB_MASK,
              FLASH_CR_PNB(page & 0xff));
  modifyreg32(STM32WB_FLASH_CR, 0, FLASH_CR_STRT);

  while (getreg32(STM32WB_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32wb_waste();
    }

  modifyreg32(STM32WB_FLASH_CR, FLASH_CR_PAGE_ERASE, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32wb_flash_unlock(void)
{
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  flash_unlock();
  nxmutex_unlock(&g_lock);

  return ret;
}

int stm32wb_flash_lock(void)
{
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  flash_lock();
  nxmutex_unlock(&g_lock);

  return ret;
}

/****************************************************************************
 * Name: stm32wb_flash_user_optbytes
 *
 * Description:
 *   Modify the contents of the user option bytes (USR OPT) on the flash.
 *   This does not set OBL_LAUNCH so new options take effect only after
 *   next power reset.
 *
 * Input Parameters:
 *   clrbits - Bits in the option bytes to be cleared
 *   setbits - Bits in the option bytes to be set
 *
 * Returned Value:
 *   Option bytes after operation is completed
 *
 ****************************************************************************/

uint32_t stm32wb_flash_user_optbytes(uint32_t clrbits, uint32_t setbits)
{
  uint32_t regval;
  int ret;

  /* To avoid accidents, do not allow setting RDP via this function.
   * Remove these asserts if want to enable changing the protection level.
   * WARNING: level 2 protection is permanent!
   */

  DEBUGASSERT((clrbits & FLASH_OPTR_RDP_MASK) == 0);
  DEBUGASSERT((setbits & FLASH_OPTR_RDP_MASK) == 0);

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return 0;
    }

  flash_optbytes_unlock();

  /* Modify Option Bytes in register. */

  regval = getreg32(STM32WB_FLASH_OPTR);

  finfo("Flash option bytes before: 0x%" PRIx32 "\n", regval);

  regval = (regval & ~clrbits) | setbits;
  putreg32(regval, STM32WB_FLASH_OPTR);

  finfo("Flash option bytes after:  0x%" PRIx32 "\n", regval);

  /* Start Option Bytes programming and wait for completion. */

  modifyreg32(STM32WB_FLASH_CR, 0, FLASH_CR_OPTSTRT);

  while (getreg32(STM32WB_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32wb_waste();
    }

  flash_optbytes_lock();
  nxmutex_unlock(&g_lock);

  return regval;
}

size_t up_progmem_pagesize(size_t page)
{
  return STM32WB_FLASH_PAGESIZE;
}

size_t up_progmem_erasesize(size_t block)
{
  return STM32WB_FLASH_PAGESIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr >= STM32WB_FLASH_BASE)
    {
      addr -= STM32WB_FLASH_BASE;
    }

  if (addr >= STM32WB_FLASH_SIZE)
    {
      return -EFAULT;
    }

  return addr / STM32WB_FLASH_PAGESIZE;
}

size_t up_progmem_getaddress(size_t page)
{
  if (page >= STM32WB_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  return page * STM32WB_FLASH_PAGESIZE + STM32WB_FLASH_BASE;
}

size_t up_progmem_neraseblocks(void)
{
  return STM32WB_FLASH_NPAGES;
}

bool up_progmem_isuniform(void)
{
  return true;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  int ret;

  if (block >= STM32WB_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Erase single block */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  flash_unlock();

  flash_erase(block);

  flash_lock();
  nxmutex_unlock(&g_lock);

  /* Verify */

  if (up_progmem_ispageerased(block) == 0)
    {
      return up_progmem_erasesize(block);
    }
  else
    {
      return -EIO;
    }
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= STM32WB_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != FLASH_ERASEDVALUE)
        {
          bwritten++;
        }
    }

  return bwritten;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t buflen)
{
  uint32_t *dest;
  const uint32_t *src;
  size_t written;
  size_t xfrsize;
  size_t offset;
  size_t page;
  int i;
  int ret = OK;

  /* Check for valid address range. */

  offset = addr;
  if (addr >= STM32WB_FLASH_BASE)
    {
      offset -= STM32WB_FLASH_BASE;
    }

  if (offset + buflen > STM32WB_FLASH_SIZE)
    {
      return -EFAULT;
    }

  /* Get the page number corresponding to the flash offset and the byte
   * offset into the page. Align write destination to page boundary.
   */

  page = FLASH_BYTE2PAGE((uint32_t)offset);
  offset &= FLASH_PAGE_MASK;

  dest = (uint32_t *)((uint8_t *)addr - offset);
  written = 0;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Get flash ready and begin flashing. */

  flash_unlock();

  /* Loop until all of the data has been written */

  while (buflen > 0)
    {
      /* How much can we write into this page? */

      xfrsize = MIN((size_t)FLASH_PAGE_SIZE - offset, buflen);

      /* Do we need to use the intermediate buffer? */

      if (offset == 0 && xfrsize == FLASH_PAGE_SIZE)
        {
          /* No, we can take the data directly from the user buffer */

          src = (const uint32_t *)buf;
        }
      else
        {
          /* Yes, copy data into global page buffer */

          if (offset > 0)
            {
              memcpy(g_page_buffer, dest, offset);
            }

          memcpy((uint8_t *)g_page_buffer + offset, buf, xfrsize);

          if (offset + xfrsize < FLASH_PAGE_SIZE)
            {
              memcpy((uint8_t *)g_page_buffer + offset + xfrsize,
                     (const uint8_t *)dest + offset + xfrsize,
                     FLASH_PAGE_SIZE - offset - xfrsize);
            }

          src = g_page_buffer;
        }

      /* Erase the page. Unlike most flash chips, STM32WB is unable to
       * write back existing data read from page without erase.
       */

      flash_erase(page);

      /* Write the page. Must be with double-words. */

      modifyreg32(STM32WB_FLASH_CR, 0, FLASH_CR_PG);

      for (i = 0; i < FLASH_PAGE_WORDS; i += 2)
        {
          *dest++ = *src++;
          *dest++ = *src++;

          while (getreg32(STM32WB_FLASH_SR) & FLASH_SR_BSY)
            {
              stm32wb_waste();
            }

          /* Verify */

          if (getreg32(STM32WB_FLASH_SR) & FLASH_SR_WRITE_PROTECTION_ERROR)
            {
              modifyreg32(STM32WB_FLASH_CR, FLASH_CR_PG, 0);
              ret = -EROFS;
              goto out;
            }

          if (getreg32(dest - 1) != *(src - 1) ||
              getreg32(dest - 2) != *(src - 2))
            {
              modifyreg32(STM32WB_FLASH_CR, FLASH_CR_PG, 0);
              ret = -EIO;
              goto out;
            }
        }

      modifyreg32(STM32WB_FLASH_CR, FLASH_CR_PG, 0);

      /* Adjust pointers and counts for the next time through the loop */

      written += xfrsize;
      addr    += xfrsize;
      dest     = (uint32_t *)addr;
      buf      = (void *)((uintptr_t)buf + xfrsize);
      buflen  -= xfrsize;
      page++;
    }

out:
  /* If there was an error, clear all error flags in status register (rc_w1
   * register so do this by writing the error bits).
   */

  if (ret != OK)
    {
      ferr("flash write error: %d, status: 0x%" PRIx32 "\n",
           ret, getreg32(STM32WB_FLASH_SR));

      modifyreg32(STM32WB_FLASH_SR, 0, FLASH_SR_ALLERRS);
    }

  flash_lock();
  nxmutex_unlock(&g_lock);
  return (ret == OK) ? written : ret;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}
