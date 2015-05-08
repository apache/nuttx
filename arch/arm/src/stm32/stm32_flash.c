/************************************************************************************
 * arch/arm/src/stm32/stm32_flash.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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

/* Provides standard flash access functions, to be used by the  flash mtd driver.
 * The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <errno.h>

#include "stm32_flash.h"
#include "stm32_rcc.h"
#include "stm32_waste.h"

#include "up_arch.h"

/* Only for the STM32F[1|3|4]0xx family for now */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined (CONFIG_STM32_STM32F40XX)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#define FLASH_CR_PAGE_ERASE              FLASH_CR_PER
#define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPRT_ERR
#elif defined(CONFIG_STM32_STM32F40XX)
#define FLASH_CR_PAGE_ERASE              FLASH_CR_SER
#define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPERR
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

void stm32_flash_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_KEYR);
    }
}

void stm32_flash_lock(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_LOCK);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)

size_t up_progmem_pagesize(size_t page)
{
  return STM32_FLASH_PAGESIZE;
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

  return addr / STM32_FLASH_PAGESIZE;
}

size_t up_progmem_getaddress(size_t page)
{
  if (page >= STM32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  return page * STM32_FLASH_PAGESIZE + STM32_FLASH_BASE;
}

#endif /* defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) */

#ifdef CONFIG_STM32_STM32F40XX

size_t up_progmem_pagesize(size_t page)
{
  static const size_t page_sizes[STM32_FLASH_NPAGES] =
    {
      16 * 1024,
      16 * 1024,
      16 * 1024,
      16 * 1024,
      64 * 1024,
      128 * 1024,
      128 * 1024,
      128 * 1024,
    };

  if (page >= sizeof(page_sizes) / sizeof(*page_sizes))
    {
      return 0;
    }
  else
    {
      return page_sizes[page];
    }
}

ssize_t up_progmem_getpage(size_t addr)
{
  size_t page_end = 0;
  size_t i;

  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if (addr >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  for (i = 0; i < STM32_FLASH_NPAGES; ++i)
    {
      page_end += up_progmem_pagesize(i);
      if (page_end > addr)
        {
          return i;
        }
    }

  return -EFAULT;
}

size_t up_progmem_getaddress(size_t page)
{
  size_t base_address = STM32_FLASH_BASE;
  size_t i;

  if (page >= STM32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  for (i = 0; i < page; ++i)
    {
      base_address += up_progmem_pagesize(i);
    }

  return base_address;
}

#endif /* def CONFIG_STM32_STM32F40XX */

size_t up_progmem_npages(void)
{
  return STM32_FLASH_NPAGES;
}

bool up_progmem_isuniform(void)
{
#ifdef STM32_FLASH_PAGESIZE
  return true;
#else
  return false;
#endif /* def STM32_FLASH_PAGESIZE */
}

ssize_t up_progmem_erasepage(size_t page)
{
#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
  size_t page_address;
#endif /* defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) */

  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Get flash ready and begin erasing single page */

  if (!(getreg32(STM32_RCC_CR) & RCC_CR_HSION))
    {
      return -EPERM;
    }

  stm32_flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PAGE_ERASE);

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
  /* Must be valid - page index checked above */

  page_address = up_progmem_getaddress(page);
  putreg32(page_address, STM32_FLASH_AR);

#elif defined(CONFIG_STM32_STM32F40XX)
  modifyreg32(STM32_FLASH_CR, FLASH_CR_SNB_MASK, FLASH_CR_SNB(page));
#endif

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PAGE_ERASE, 0);

  /* Verify */
  if (up_progmem_ispageerased(page) == 0)
    {
      return up_progmem_pagesize(page); /* success */
    }
  else
    {
      return -EIO; /* failure */
    }
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= STM32_FLASH_NPAGES)
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

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  uint16_t *hword = (uint16_t *)buf;
  size_t written = count;

  /* STM32 requires half-word access */

  if (count & 1)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if ((addr+count) >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  /* Get flash ready and begin flashing */

  if (!(getreg32(STM32_RCC_CR) & RCC_CR_HSION))
    {
      return -EPERM;
    }

  stm32_flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

#if defined(CONFIG_STM32_STM32F40XX)
  /* TODO: implement up_progmem_write() to support other sizes than 16-bits */
  modifyreg32(STM32_FLASH_CR, FLASH_CR_PSIZE_MASK, FLASH_CR_PSIZE_X16);
#endif

  for (addr += STM32_FLASH_BASE; count; count-=2, hword++, addr+=2)
    {
      /* Write half-word and wait to complete */

      putreg16(*hword, addr);

      while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

      /* Verify */

      if (getreg32(STM32_FLASH_SR) & FLASH_SR_WRITE_PROTECTION_ERROR)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          return -EROFS;
        }

      if (getreg16(addr) != *hword)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          return -EIO;
        }
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
  return written;
}

#endif /* defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
          defined (CONFIG_STM32_STM32F40XX) */
