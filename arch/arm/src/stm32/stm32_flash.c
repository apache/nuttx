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

#include <stdbool.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include "stm32_flash.h"
#include "stm32_rcc.h"
#include "stm32_waste.h"

#include "up_arch.h"

/* Only for the STM32F[1|3|4]0xx family and STM32L15xx (EEPROM only) for now */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined (CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX)

#if defined(CONFIG_STM32_FLASH_CONFIG_DEFAULT) && \
    (defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX))
#  warning "Default Flash Configuration Used - See Override Flash Size Designator"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_STM32_STM32L15XX)
#  define FLASH_KEY1      0x8C9DAEBF
#  define FLASH_KEY2      0x13141516
#else
#  define FLASH_KEY1      0x45670123
#  define FLASH_KEY2      0xCDEF89AB
#  define FLASH_OPTKEY1   0x08192A3B
#  define FLASH_OPTKEY2   0x4C5D6E7F
#endif

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#  define FLASH_CR_PAGE_ERASE              FLASH_CR_PER
#  define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPRT_ERR
#elif defined(CONFIG_STM32_STM32F4XXX)
#  define FLASH_CR_PAGE_ERASE              FLASH_CR_SER
#  define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPERR
#endif

#if defined(CONFIG_STM32_STM32L15XX)
#  define EEPROM_KEY1     0x89ABCDEF
#  define EEPROM_KEY2     0x02030405
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

static sem_t g_sem = SEM_INITIALIZER(1);

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void sem_lock(void)
{
  while (sem_wait(&g_sem) < 0)
    {
      DEBUGASSERT(errno == EINTR);
    }
}

static inline void sem_unlock(void)
{
  sem_post(&g_sem);
}

#if !defined(CONFIG_STM32_STM32L15XX)

static void flash_unlock(void)
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

static void flash_lock(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_LOCK);
}

#endif /* !defined(CONFIG_STM32_STM32L15XX) */

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

#if defined(CONFIG_STM32_STM32L15XX)

static void stm32_eeprom_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }

  if (getreg32(STM32_FLASH_PECR) & FLASH_PECR_PELOCK)
    {
      /* Unlock sequence */

      putreg32(EEPROM_KEY1, STM32_FLASH_PEKEYR);
      putreg32(EEPROM_KEY2, STM32_FLASH_PEKEYR);
    }
}

static void stm32_eeprom_lock(void)
{
  modifyreg32(STM32_FLASH_PECR, 0, FLASH_PECR_PELOCK);
}

static void flash_unlock(void)
{
  if (getreg32(STM32_FLASH_PECR) & FLASH_PECR_PRGLOCK)
    {
      stm32_eeprom_unlock();

      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32_FLASH_PRGKEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_PRGKEYR);
    }
}

static void flash_lock(void)
{
  modifyreg32(STM32_FLASH_PECR, 0, FLASH_PECR_PRGLOCK);
  stm32_eeprom_lock();
}

static ssize_t stm32_eeprom_erase_write(size_t addr, const void *buf,
                                        size_t buflen)
{
  const char *cbuf = buf;
  size_t i;

  if (buflen == 0)
    {
      return 0;
    }

  /* Check for valid address range */

  if (addr >= STM32_EEPROM_BASE)
    {
      addr -= STM32_EEPROM_BASE;
    }

  if (addr >= STM32_EEPROM_SIZE)
    {
      return -EINVAL;
    }

  /* TODO: Voltage range must be range 1 or 2. Erase/program not allowed in
   * range 3.
   */

  stm32_eeprom_unlock();

  /* Clear pending status flags. */

  putreg32(FLASH_SR_WRPERR | FLASH_SR_PGAERR |
           FLASH_SR_SIZERR | FLASH_SR_OPTVERR |
           FLASH_SR_OPTVERRUSR | FLASH_SR_RDERR, STM32_FLASH_SR);

  /* Enable automatic erasing (by disabling 'fixed time' programming). */

  modifyreg32(STM32_FLASH_PECR, FLASH_PECR_FTDW, 0);

  /* Write buffer to EEPROM data memory. */

  addr += STM32_EEPROM_BASE;
  i = 0;
  while (i < buflen)
    {
      uint32_t writeval;
      size_t left = buflen - i;

      if ((addr & 0x03) == 0x00 && left >= 4)
        {
          /* Read/erase/write word */

          writeval = cbuf ? *(uint32_t *)cbuf : 0;
          putreg32(writeval, addr);
        }
      else if ((addr & 0x01) == 0x00 && left >= 2)
        {
          /* Read/erase/write half-word */

          writeval = cbuf ? *(uint16_t *)cbuf : 0;
          putreg16(writeval, addr);
        }
      else
        {
          /* Read/erase/write byte */

          writeval = cbuf ? *(uint8_t *)cbuf : 0;
          putreg8(writeval, addr);
        }

      /* ... and wait to complete. */

      while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
        {
          up_waste();
        }

      /* Verify */

      /* We do not check Options Byte invalid flags FLASH_SR_OPTVERR
       * and FLASH_SR_OPTVERRUSR for EEPROM erase/write. They are unrelated
       * and STM32L standard library does not check for these either.
       */

      if (getreg32(STM32_FLASH_SR) & (FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                                      FLASH_SR_SIZERR | FLASH_SR_RDERR))
        {
          stm32_eeprom_lock();
          return -EROFS;
        }

      if ((addr & 0x03) == 0x00 && left >= 4)
        {
          if (getreg32(addr) != writeval)
            {
              stm32_eeprom_lock();
              return -EIO;
            }

          addr += 4;
          i += 4;
          cbuf += !!(cbuf) * 4;
        }
      else if ((addr & 0x01) == 0x00 && left >= 2)
        {
          if (getreg16(addr) != writeval)
            {
              stm32_eeprom_lock();
              return -EIO;
            }

          addr += 2;
          i += 2;
          cbuf += !!(cbuf) * 2;
        }
      else
        {
          if (getreg8(addr) != writeval)
            {
              stm32_eeprom_lock();
              return -EIO;
            }

          addr += 1;
          i += 1;
          cbuf += !!(cbuf) * 1;
        }
    }

  stm32_eeprom_lock();
  return buflen;
}

#endif /* defined(CONFIG_STM32_STM32L15XX) */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void stm32_flash_unlock(void)
{
  sem_lock();
  flash_unlock();
  sem_unlock();
}

void stm32_flash_lock(void)
{
  sem_lock();
  flash_lock();
  sem_unlock();
}

#if defined(CONFIG_STM32_STM32L15XX)

size_t stm32_eeprom_size(void)
{
  return STM32_EEPROM_SIZE;
}

size_t stm32_eeprom_getaddress(void)
{
  return STM32_EEPROM_BASE;
}

ssize_t stm32_eeprom_write(size_t addr, const void *buf, size_t buflen)
{
  if (!buf)
    {
      return -EINVAL;
    }

  return stm32_eeprom_erase_write(addr, buf, buflen);
}

ssize_t stm32_eeprom_erase(size_t addr, size_t eraselen)
{
  return stm32_eeprom_erase_write(addr, NULL, eraselen);
}

#endif /* defined(CONFIG_STM32_STM32L15XX) */

/************************************************************************************
 * Name: stm32_flash_writeprotect
 *
 * Description:
 *   Enable or disable the write protection of a flash sector.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_STM32F4XXX
int stm32_flash_writeprotect(size_t page, bool enabled)
{
  uint32_t reg;
  uint32_t val;

#ifdef CONFIG_STM32_STM32F4XXX
  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }
#else
#  warning missing logic in stm32_flash_writeprotect
#endif

  /* Select the register that contains the bit to be changed */

  if (page < 12)
    {
      reg = STM32_FLASH_OPTCR;
    }
#if defined(CONFIG_STM32_FLASH_CONFIG_I) && defined(CONFIG_STM32_STM32F4XXX)
  else
    {
      reg = STM32_FLASH_OPTCR1;
      page -= 12;
    }
#else
  else
    {
      return -EFAULT;
    }
#endif

  /* Read the option status */

  val = getreg32(reg);

  /* Set or clear the protection */

  if (enabled)
    {
      val &= ~(1 << (16+page) );
    }
  else
    {
      val |=  (1 << (16+page) );
    }

  /* Unlock options */

  putreg32(FLASH_OPTKEY1, STM32_FLASH_OPTKEYR);
  putreg32(FLASH_OPTKEY2, STM32_FLASH_OPTKEYR);

  /* Write options */

  putreg32(val, reg);

  /* Trigger programmation */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  /* Wait for completion */

  while(getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  /* Relock options */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTLOCK);
  return 0;
}
#endif

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

#ifdef CONFIG_STM32_STM32F4XXX

size_t up_progmem_pagesize(size_t page)
{
  static const size_t page_sizes[STM32_FLASH_NPAGES] = STM32_FLASH_SIZES;

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

#endif /* def CONFIG_STM32_STM32F4XXX */

#if !defined(CONFIG_STM32_STM32L15XX)

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
#endif
}

ssize_t up_progmem_erasepage(size_t page)
{
#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
  size_t page_address;
#endif

  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  sem_lock();

#if !defined(CONFIG_STM32_STM32F4XXX)
  if (!(getreg32(STM32_RCC_CR) & RCC_CR_HSION))
    {
      sem_unlock();
      return -EPERM;
    }
#endif

  /* Get flash ready and begin erasing single page */

  flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PAGE_ERASE);

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
  /* Must be valid - page index checked above */

  page_address = up_progmem_getaddress(page);
  putreg32(page_address, STM32_FLASH_AR);

#elif defined(CONFIG_STM32_STM32F4XXX)
  modifyreg32(STM32_FLASH_CR, FLASH_CR_SNB_MASK, FLASH_CR_SNB(page));
#endif

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PAGE_ERASE, 0);
  sem_unlock();

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

  if ((addr+count) > STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  sem_lock();

#if !defined(CONFIG_STM32_STM32F4XXX)
  if (!(getreg32(STM32_RCC_CR) & RCC_CR_HSION))
    {
      sem_unlock();
      return -EPERM;
    }
#endif

  /* Get flash ready and begin flashing */

  flash_unlock();

#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
  data_cache_disable();
#endif

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

#if defined(CONFIG_STM32_STM32F4XXX)
  /* TODO: implement up_progmem_write() to support other sizes than 16-bits */
  modifyreg32(STM32_FLASH_CR, FLASH_CR_PSIZE_MASK, FLASH_CR_PSIZE_X16);
#endif

  for (addr += STM32_FLASH_BASE; count; count -= 2, hword++, addr += 2)
    {
      /* Write half-word and wait to complete */

      putreg16(*hword, addr);

      while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

      /* Verify */

      if (getreg32(STM32_FLASH_SR) & FLASH_SR_WRITE_PROTECTION_ERROR)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          sem_unlock();
          return -EROFS;
        }

      if (getreg16(addr) != *hword)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          sem_unlock();
          return -EIO;
        }
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);

#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
  data_cache_enable();
#endif

  sem_unlock();
  return written;
}

#endif /* !defined(CONFIG_STM32_STM32L15XX) */

#endif /* defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
          defined(CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX) */
