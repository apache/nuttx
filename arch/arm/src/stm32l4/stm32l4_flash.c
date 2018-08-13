/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_flash.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Juha Niskanen <juha.niskanen@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/* Provides standard flash access functions, to be used by the flash mtd driver.
 * The interface is defined in the include/nuttx/progmem.h
 *
 * Notes about this implementation:
 *  - HSI16 is automatically turned ON by MCU, if not enabled beforehand
 *  - Only Standard Programming is supported, no Fast Programming.
 *  - Low Power Modes are not permitted during write/erase
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/progmem.h>

#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <string.h>

#include "stm32l4_rcc.h"
#include "stm32l4_waste.h"
#include "stm32l4_flash.h"

#include "up_arch.h"

#if !(defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4X5) || \
      defined(CONFIG_STM32L4_STM32L4X6) || defined(CONFIG_STM32L4_STM32L4XR))
#  error "Unrecognized STM32 chip"
#endif

#if !defined(CONFIG_STM32L4_FLASH_OVERRIDE_DEFAULT)
#  warning "Flash Configuration has been overridden - make sure it is correct"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define FLASH_KEY1         0x45670123
#define FLASH_KEY2         0xCDEF89AB

#define OPTBYTES_KEY1      0x08192A3B
#define OPTBYTES_KEY2      0x4C5D6E7F

#define FLASH_PAGE_SIZE    STM32L4_FLASH_PAGESIZE
#define FLASH_PAGE_WORDS   (FLASH_PAGE_SIZE / 4)
#define FLASH_PAGE_MASK    (FLASH_PAGE_SIZE - 1)
#if FLASH_PAGE_SIZE == 2048
#  define FLASH_PAGE_SHIFT   (11)    /* 2**11  = 2048B */
#elif FLASH_PAGE_SIZE == 8192
#  define FLASH_PAGE_SHIFT   (13)    /* 2**13  = 8192B */
#else
#  error Unsupported STM32L4_FLASH_PAGESIZE
#endif
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

/************************************************************************************
 * Private Data
 ************************************************************************************/

static sem_t g_sem = SEM_INITIALIZER(1);
static uint32_t g_page_buffer[FLASH_PAGE_WORDS];

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static inline void sem_lock(void)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

static inline void sem_unlock(void)
{
  nxsem_post(&g_sem);
}

static void flash_unlock(void)
{
  while (getreg32(STM32L4_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }

  if (getreg32(STM32L4_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32L4_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32L4_FLASH_KEYR);
    }
}

static void flash_lock(void)
{
  modifyreg32(STM32L4_FLASH_CR, 0, FLASH_CR_LOCK);
}

static void flash_optbytes_unlock(void)
{
  flash_unlock();

  if (getreg32(STM32L4_FLASH_CR) & FLASH_CR_OPTLOCK)
    {
      /* Unlock Option Bytes sequence */

      putreg32(OPTBYTES_KEY1, STM32L4_FLASH_OPTKEYR);
      putreg32(OPTBYTES_KEY2, STM32L4_FLASH_OPTKEYR);
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

  modifyreg32(STM32L4_FLASH_CR, 0, FLASH_CR_PAGE_ERASE);
  modifyreg32(STM32L4_FLASH_CR, FLASH_CR_PNB_MASK, FLASH_CR_PNB(page));
  modifyreg32(STM32L4_FLASH_CR, 0, FLASH_CR_START);

  while (getreg32(STM32L4_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }

  modifyreg32(STM32L4_FLASH_CR, FLASH_CR_PAGE_ERASE, 0);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void stm32l4_flash_unlock(void)
{
  sem_lock();
  flash_unlock();
  sem_unlock();
}

void stm32l4_flash_lock(void)
{
  sem_lock();
  flash_lock();
  sem_unlock();
}

/****************************************************************************
 * Name: stm32l4_flash_user_optbytes
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

uint32_t stm32l4_flash_user_optbytes(uint32_t clrbits, uint32_t setbits)
{
  uint32_t regval;

  /* To avoid accidents, do not allow setting RDP via this function.
   * Remove these asserts if want to enable changing the protection level.
   * WARNING: level 2 protection is permanent!
   */

  DEBUGASSERT((clrbits & FLASH_OPTCR_RDP_MASK) == 0);
  DEBUGASSERT((setbits & FLASH_OPTCR_RDP_MASK) == 0);

  sem_lock();
  flash_optbytes_unlock();

  /* Modify Option Bytes in register. */

  regval = getreg32(STM32L4_FLASH_OPTR);

  finfo("Flash option bytes before: 0x%x\n", regval);

  regval = (regval & ~clrbits) | setbits;
  putreg32(regval, STM32L4_FLASH_OPTR);

  finfo("Flash option bytes after:  0x%x\n", regval);

  /* Start Option Bytes programming and wait for completion. */

  modifyreg32(STM32L4_FLASH_CR, 0, FLASH_CR_OPTSTRT);

  while (getreg32(STM32L4_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }

  flash_optbytes_lock();
  sem_unlock();

  return regval;
}

size_t up_progmem_pagesize(size_t page)
{
  return STM32L4_FLASH_PAGESIZE;
}

size_t up_progmem_erasesize(size_t page)
{
  return STM32L4_FLASH_PAGESIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr >= STM32L4_FLASH_BASE)
    {
      addr -= STM32L4_FLASH_BASE;
    }

  if (addr >= STM32L4_FLASH_SIZE)
    {
      return -EFAULT;
    }

  return addr / STM32L4_FLASH_PAGESIZE;
}

size_t up_progmem_getaddress(size_t page)
{
  if (page >= STM32L4_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  return page * STM32L4_FLASH_PAGESIZE + STM32L4_FLASH_BASE;
}

size_t up_progmem_npages(void)
{
  return STM32L4_FLASH_NPAGES;
}

bool up_progmem_isuniform(void)
{
  return true;
}

ssize_t up_progmem_erasepage(size_t page)
{
  if (page >= STM32L4_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Erase single page */

  sem_lock();
  flash_unlock();

  flash_erase(page);

  flash_lock();
  sem_unlock();

  /* Verify */

  if (up_progmem_ispageerased(page) == 0)
    {
      return up_progmem_pagesize(page);
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

  if (page >= STM32L4_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != 0xff)
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
  if (addr >= STM32L4_FLASH_BASE)
    {
      offset -= STM32L4_FLASH_BASE;
    }

  if (offset + buflen > STM32L4_FLASH_SIZE)
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

  sem_lock();

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

      /* Erase the page. Unlike most flash chips, STM32L4 is unable to
       * write back existing data read from page without erase.
       */

      flash_erase(page);

      /* Write the page. Must be with double-words. */

      modifyreg32(STM32L4_FLASH_CR, 0, FLASH_CR_PG);

      for (i = 0; i < FLASH_PAGE_WORDS; i += 2)
        {
          *dest++ = *src++;
          *dest++ = *src++;

          while (getreg32(STM32L4_FLASH_SR) & FLASH_SR_BSY)
            {
              up_waste();
            }

          /* Verify */

          if (getreg32(STM32L4_FLASH_SR) & FLASH_SR_WRITE_PROTECTION_ERROR)
            {
              modifyreg32(STM32L4_FLASH_CR, FLASH_CR_PG, 0);
              ret = -EROFS;
              goto out;
            }

          if (getreg32(dest-1) != *(src-1) || getreg32(dest-2) != *(src-2))
            {
              modifyreg32(STM32L4_FLASH_CR, FLASH_CR_PG, 0);
              ret = -EIO;
              goto out;
            }
        }

      modifyreg32(STM32L4_FLASH_CR, FLASH_CR_PG, 0);

      /* Adjust pointers and counts for the next time through the loop */

      written += xfrsize;
      addr    += xfrsize;
      dest     = (uint32_t *)addr;
      buf      = (void *)((uintptr_t)buf + xfrsize);
      buflen  -= xfrsize;
      page++;
    }

out:
  /* If there was an error, clear all error flags in status
   * register (rc_w1 register so do this by writing the
   * error bits).
   */

  if (ret != OK)
    {
      ferr("flash write error: %d, status: 0x%x\n", ret, getreg32(STM32L4_FLASH_SR));
      modifyreg32(STM32L4_FLASH_SR, 0, FLASH_SR_ALLERRS);
    }

  flash_lock();
  sem_unlock();
  return (ret == OK) ? written : ret;
}
