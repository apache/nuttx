/****************************************************************************
 * arch/arm/src/stm32/stm32g4xxx_flash.c
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

/* Provides standard flash access functions, to be used by the flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 *
 * Notes about this implementation:
 *  - HSI16 is automatically turned ON by MCU, if not enabled beforehand
 *  - Only Standard Programming is supported, no Fast Programming.
 *  - Low Power Modes are not permitted during write/erase
 */

/* Differences vs STM32L4 (used as template):
 *   1. FLASH_CR - Bits (29:28) (SEC_PROT2, SEC_PROT1) added.
 *   2. FLASH_ECCR - Bits (29:28) (ECCD2, ECCC2) added.
 *        Note: Bits are set by hardware. Nothing to do
 *   3. FLASH_OPTR -
 *     a. DUALBANK moved from bit 21 to 22.
 *     b. NRST_MODE added - Bits 29:28
 *     c. IRHEN added - Bit 30
 *   4. FLASH_SEC1R - (New) Secure Area Bank 1 Register
 *     a. BOOT_LOCK - Forces boot from user flash area
 *     b. SEC_SIZE1[7:0] - Starts at 0x80000000, size = SEC_SIZE1 * page_size
 *   5. FLASH_SEC2R - (New) Secure Area Bank 2 Register
 *     a. BOOT_LOCK - Forces boot from user flash area
 *     b. SEC_SIZE1[7:0] - Starts at 0x80000000, size = SEC_SIZE1 * page_size
 *   6. FLASH_PAGE_SIZE - The page size of the STM32G47XX and STM32G48XX
 *      is dependent on the DBANK bit. If Dual Banks are used, the page size
 *      is 2K. If a single bank is used, the page size is 4K.
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
#include <inttypes.h>
#include <string.h>
#include <sys/param.h>

#include "stm32_rcc.h"
#include "stm32_waste.h"
#include "stm32_flash.h"
#include "arm_internal.h"

#if !(defined(CONFIG_STM32_STM32G43XX) || defined(CONFIG_STM32_STM32G47XX) || \
      defined(CONFIG_STM32_STM32G48XX) || defined(CONFIG_STM32_STM32G49XX))
#  error "Unrecognized STM32 chip"
#endif

#if !defined(CONFIG_STM32_FLASH_CONFIG_DEFAULT)
#  warning "Flash Configuration has been overridden - make sure it is correct"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_KEY1         0x45670123
#define FLASH_KEY2         0xCDEF89AB
#define FLASH_ERASEDVALUE  0xffu

#define OPTBYTES_KEY1      0x08192A3B
#define OPTBYTES_KEY2      0x4C5D6E7F

#define FLASH_CR_PAGE_ERASE              FLASH_CR_PER
#define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPERR

/* All errors for Standard Programming, not for other operations. */

#define FLASH_SR_ALLERRS   (FLASH_SR_PGSERR | FLASH_SR_SIZERR | \
                            FLASH_SR_PGAERR | FLASH_SR_WRPERR | \
                            FLASH_SR_PROGERR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t get_flash_page_size(void)
{
#if defined(CONFIG_STM32_STM32G47XX) || defined(CONFIG_STM32_STM32G48XX)
  if (getreg32(STM32_FLASH_OPTR) & FLASH_OPTR_DBANK)
    {
      return 2048;
    }
  else
    {
      return 4096;
    }
#else
  return STM32_FLASH_PAGESIZE;
#endif
}

static uint32_t get_flash_npages(void)
{
#if defined(CONFIG_STM32_STM32G47XX) || defined(CONFIG_STM32_STM32G48XX)
  if (getreg32(STM32_FLASH_OPTR) & FLASH_OPTR_DBANK)
    {
      return STM32_FLASH_SIZE / 2048;
    }
  else
    {
      return STM32_FLASH_SIZE / 4096;
    }
#else
  return STM32_FLASH_NPAGES;
#endif
}

static void flash_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32_waste();
    }

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_KEYR);
    }
}

static void flash_lock(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_LOCK);
}

static void flash_optbytes_unlock(void)
{
  flash_unlock();

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_OPTLOCK)
    {
      /* Unlock Option Bytes sequence */

      putreg32(OPTBYTES_KEY1, STM32_FLASH_OPTKEYR);
      putreg32(OPTBYTES_KEY2, STM32_FLASH_OPTKEYR);
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

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PAGE_ERASE);

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PNB_MASK,
              FLASH_CR_PNB(page));

#if (defined(CONFIG_STM32_STM32G47XX) || \
     defined(CONFIG_STM32_STM32G48XX))
  uint32_t half_npages = get_flash_npages() / 2;

  if (getreg32(STM32_FLASH_OPTR) & FLASH_OPTR_DBANK)
    {
      if (page < half_npages)
        {
          /* Select bank 1 */

          modifyreg32(STM32_FLASH_CR, FLASH_CR_BKER, 0);
        }
      else
        {
          /* Select bank 2 */

          modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_BKER);
        }
    }
#endif

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_START);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32_waste();
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PAGE_ERASE, 0);
}

#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
static void data_cache_disable(void)
{
  modifyreg32(STM32_FLASH_ACR, FLASH_ACR_DCEN, 0);
}

static void data_cache_enable(void)
{
  /* Reset data cache */

  modifyreg32(STM32_FLASH_ACR, 0, FLASH_ACR_DCRST);

  /* Enable data cache */

  modifyreg32(STM32_FLASH_ACR, 0, FLASH_ACR_DCEN);
}
#endif /* defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW) */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_flash_unlock(void)
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

int stm32_flash_lock(void)
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

uint32_t stm32_flash_user_optbytes(uint32_t clrbits, uint32_t setbits)
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

  regval = getreg32(STM32_FLASH_OPTR);

  finfo("Flash option bytes before: 0x%" PRIx32 "\n", regval);

  regval = (regval & ~clrbits) | setbits;

  putreg32(regval, STM32_FLASH_OPTR);

  finfo("Flash option bytes after:  0x%" PRIx32 "\n", regval);

  /* Start Option Bytes programming and wait for completion. */

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_OPTSTRT);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32_waste();
    }

  flash_optbytes_lock();
  nxmutex_unlock(&g_lock);

  return regval;
}

size_t up_progmem_pagesize(size_t page)
{
  return get_flash_page_size();
}

size_t up_progmem_erasesize(size_t block)
{
  return get_flash_page_size();
}

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if (addr >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  return addr / get_flash_page_size();
}

size_t up_progmem_getaddress(size_t page)
{
  if (page >= get_flash_npages())
    {
      return SIZE_MAX;
    }

  return page * get_flash_page_size() + STM32_FLASH_BASE;
}

size_t up_progmem_neraseblocks(void)
{
  return get_flash_npages();
}

bool up_progmem_isuniform(void)
{
  return true;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  int ret;

  if (block >= get_flash_npages())
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

  if (page >= get_flash_npages())
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
  bool set_pg_bit = false;
  int i;
  int ret = OK;
  const uint32_t flash_page_size = get_flash_page_size();
  const uint32_t flash_page_words = flash_page_size / 4;
  const uint32_t flash_page_mask = flash_page_size - 1;
  uint32_t *page_buffer = NULL;

  /* Check for valid address range. */

  offset = addr;
  if (addr >= STM32_FLASH_BASE)
    {
      offset -= STM32_FLASH_BASE;
    }

  if (offset + buflen > STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  /* Get the page number corresponding to the flash offset and the byte
   * offset into the page. Align write destination to page boundary.
   */

  if (flash_page_size == 4096)
    {
      page = ((uint32_t)offset >> 12);
    }
  else
    {
      page = ((uint32_t)offset >> 11);
    }

  offset &= flash_page_mask;

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

      xfrsize = MIN((size_t) flash_page_size - offset, buflen);

      /* Do we need to use the intermediate buffer? */

      if (offset == 0 && xfrsize == flash_page_size)
        {
          /* No, we can take the data directly from the user buffer */

          src = (const uint32_t *)buf;
        }
      else
        {
          /* Yes, copy data into the page buffer */

          page_buffer = malloc(flash_page_size);

          if (offset > 0)
            {
              memcpy(page_buffer, dest, offset);
            }

          memcpy((uint8_t *)page_buffer + offset, buf, xfrsize);

          if (offset + xfrsize < flash_page_size)
            {
              memcpy((uint8_t *)page_buffer + offset + xfrsize,
                     (const uint8_t *)dest + offset + xfrsize,
                     flash_page_size - offset - xfrsize);
            }

          src = page_buffer;
        }

      /* Erase the page. Unlike most flash chips, STM32 is unable to
       * write back existing data read from page without erase.
       */

      flash_erase(page);

      /* Write the page. Must be with double-words. */

#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
      data_cache_disable();
#endif

      modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);
      set_pg_bit = true;

      for (i = 0; i < flash_page_words; i += 2)
        {
          *dest++ = *src++;
          *dest++ = *src++;

          while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
            {
              stm32_waste();
            }

          /* Verify */

          if (getreg32(STM32_FLASH_SR) & FLASH_SR_WRITE_PROTECTION_ERROR)
            {
              ret = -EROFS;
              goto out;
            }

          if (getreg32(dest -1) != *(src - 1) ||
              getreg32(dest - 2) != *(src - 2))
            {
              ret = -EIO;
              goto out;
            }
        }

      modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
      set_pg_bit = false;

#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
      data_cache_enable();
#endif

      /* Adjust pointers and counts for the next time through the loop */

      written += xfrsize;
      addr    += xfrsize;
      dest     = (uint32_t *)addr;
      buf      = (void *)((uintptr_t)buf + xfrsize);
      buflen  -= xfrsize;
      page++;
    }

out:
  if (set_pg_bit)
    {
      modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
      data_cache_enable();
#endif
    }

  /* If there was an error, clear all error flags in status register (rc_w1
   * register so do this by writing the error bits).
   */

  if (ret != OK)
    {
      ferr("flash write error: %d, status: 0x%" PRIx32 "\n",
           ret, getreg32(STM32_FLASH_SR));

      modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_ALLERRS);
    }

  free(page_buffer);
  flash_lock();
  nxmutex_unlock(&g_lock);
  return (ret == OK) ? written : ret;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}
