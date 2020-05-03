/************************************************************************************
 * arch/arm/src/stm32/stm3l15xx_flash.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * STM32L1 support:
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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
#include <nuttx/semaphore.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "stm32_flash.h"
#include "stm32_rcc.h"
#include "stm32_waste.h"

#include "arm_arch.h"

/* Only for the STM32L15xx family. */

#if defined(CONFIG_STM32_STM32L15XX)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define FLASH_KEY1        0x8c9daebf
#define FLASH_KEY2        0x13141516
#define FLASH_OPTKEY1     0xfbead9c8
#define FLASH_OPTKEY2     0x24252627
#define EEPROM_KEY1       0x89abcdef
#define EEPROM_KEY2       0x02030405

#define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPERR
#define FLASH_SR_ALLERRS                 (FLASH_SR_RDERR | FLASH_SR_SIZERR | \
                                          FLASH_SR_PGAERR | FLASH_SR_WRPERR)

/* STM32L1 internal flash is based on EEPROM-technology while most others
 * are NOR-flash, thus many things are different including the erase value.
 */

#define FLASH_ERASEDVALUE  0x00

/************************************************************************************
 * Private Data
 ************************************************************************************/

static sem_t g_sem = SEM_INITIALIZER(1);

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int sem_lock(void)
{
  return nxsem_wait_uninterruptible(&g_sem);
}

static inline void sem_unlock(void)
{
  nxsem_post(&g_sem);
}

static void stm32_eeprom_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32_waste();
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
          stm32_waste();
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

/************************************************************************************
 * Public Functions
 ************************************************************************************/

int stm32_flash_unlock(void)
{
  int ret;

  ret = sem_lock();
  if (ret < 0)
    {
      return ret;
    }

  flash_unlock();
  sem_unlock();

  return ret;
}

int stm32_flash_lock(void)
{
  int ret;

  ret = sem_lock();
  if (ret < 0)
    {
      return ret;
    }

  flash_lock();
  sem_unlock();

  return ret;
}

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
  ssize_t outlen;
  int ret;

  if (!buf)
    {
      return -EINVAL;
    }

  ret = sem_lock();
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  outlen = stm32_eeprom_erase_write(addr, buf, buflen);
  sem_unlock();

  return outlen;
}

ssize_t stm32_eeprom_erase(size_t addr, size_t eraselen)
{
  ssize_t outlen;
  int ret;

  ret = sem_lock();
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  outlen = stm32_eeprom_erase_write(addr, NULL, eraselen);
  sem_unlock();

  return outlen;
}

size_t up_progmem_pagesize(size_t page)
{
  return STM32_FLASH_PAGESIZE;
}

size_t up_progmem_erasesize(size_t block)
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

size_t up_progmem_neraseblocks(void)
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
      if (getreg8(addr) != FLASH_ERASEDVALUE)
        {
          bwritten++;
        }
    }

  return bwritten;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  size_t page_address;
  int ret;

  if (block >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  page_address = up_progmem_getaddress(block);

  /* Get flash ready and begin erasing single page */

  ret = sem_lock();
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  flash_unlock();

  modifyreg32(STM32_FLASH_PECR, 0, FLASH_PECR_ERASE);
  modifyreg32(STM32_FLASH_PECR, 0, FLASH_PECR_PROG);

  /* Erase is started by writing 0x00000000 to the first word
   * of the program page.
   */

  putreg32(0x00, page_address);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32_waste();
    }

  flash_lock();
  sem_unlock();

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

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  uint32_t *word = (uint32_t *)buf;
  size_t written = count;
  int ret = OK;

  /* STM32L1 requires word access and alignment. */

  if (addr & 3)
    {
      return -EINVAL;
    }

  if (count & 3)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if ((addr + count) > STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  /* Get flash ready and begin flashing */

  ret = sem_lock();
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  flash_unlock();

  for (addr += STM32_FLASH_BASE; count; count -= 4, word++, addr += 4)
    {
      /* Write word and wait to complete */

      putreg32(*word, addr);

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

      if (getreg32(addr) != *word)
        {
          ret = -EIO;
          goto out;
        }
    }

out:
  /* If there was an error, clear all error flags in status
   * register (rc_w1 register so do this by writing the
   * error bits).
   */

  if (ret != OK)
    {
      ferr("flash write error: %d, status: 0x%x\n", ret, getreg32(STM32_FLASH_SR));
      modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_ALLERRS);
    }

  flash_lock();
  sem_unlock();
  return (ret == OK) ? written : ret;
}
#endif /* defined(CONFIG_STM32_STM32L15XX) */
