/****************************************************************************
 * arch/arm/src/stm32f7/stm32_flash.c
 *
 *   Copyright (C) 2018 Wolpike LLC. All rights reserved.
 *   Author: Evgeniy Bobkov <evgen@wolpike.com>
 *
 * Ported from stm32f20xxf40xx_flash.c, this is the original license:
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
 ****************************************************************************/

/* Provides standard flash access functions, to be used by the  flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdbool.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include "barriers.h"

#include "hardware/stm32_flash.h"
#include "up_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_KEY1         0x45670123
#define FLASH_KEY2         0xcdef89ab
#define FLASH_OPTKEY1      0x08192a3b
#define FLASH_OPTKEY2      0x4c5d6e7f
#define FLASH_ERASEDVALUE  0xff

#if CONFIG_STM32F7_PROGMEM_STARTBLOCK < 0 || \
    CONFIG_STM32F7_PROGMEM_STARTBLOCK >= STM32_FLASH_NPAGES
#  error "Invalid CONFIG_STM32F7_PROGMEM_STARTBLOCK"
#endif

#if CONFIG_STM32F7_PROGMEM_LASTBLOCK < 0
#  undef CONFIG_STM32F7_PROGMEM_LASTBLOCK
#  define CONFIG_STM32F7_PROGMEM_LASTBLOCK (STM32_FLASH_NPAGES-1)
#endif

#define PROGMEM_NBLOCKS \
  (CONFIG_STM32F7_PROGMEM_LASTBLOCK - CONFIG_STM32F7_PROGMEM_STARTBLOCK + 1)

#define PAGESIZE 256

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_sem = SEM_INITIALIZER(1);

static const size_t page_sizes[STM32_FLASH_NPAGES] = STM32_FLASH_SIZES;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sem_lock(void)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = sem_wait(&g_sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

static inline void sem_unlock(void)
{
  sem_post(&g_sem);
}

static void flash_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
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

static uint32_t flash_size(void)
{
  static uint32_t size;
  int i;

  if (size == 0)
    {
      for (i = 0; i < PROGMEM_NBLOCKS; i++)
        {
          size += page_sizes[CONFIG_STM32F7_PROGMEM_STARTBLOCK + i];
        }
    }

  return size;
}

static uint32_t flash_base(void)
{
  static uint32_t base;
  int i;

  if (base == 0)
    {
      base = STM32_FLASH_BASE;
      for (i = 0; i < CONFIG_STM32F7_PROGMEM_STARTBLOCK; i++)
        {
          base += page_sizes[i];
        }
    }

  return base;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

/****************************************************************************
 * Name: stm32_flash_writeprotect
 *
 * Description:
 *   Enable or disable the write protection of a flash sector.
 *
 ****************************************************************************/

int stm32_flash_writeprotect(size_t block, bool enabled)
{
  uint32_t reg;
  uint32_t val;

  if (block >= PROGMEM_NBLOCKS)
    {
      return -EFAULT;
    }

  block = block + CONFIG_STM32F7_PROGMEM_STARTBLOCK;

  /* Select the register that contains the bit to be changed */

  if (block < 12)
    {
      reg = STM32_FLASH_OPTCR;
    }
#if defined(CONFIG_STM32F7_FLASH_CONFIG_I)
  else
    {
      reg = STM32_FLASH_OPTCR1;
      block -= 12;
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
      val &= ~(1 << (16 + block));
    }
  else
    {
      val |=  (1 << (16 + block));
    }

  /* Unlock options */

  putreg32(FLASH_OPTKEY1, STM32_FLASH_OPTKEYR);
  putreg32(FLASH_OPTKEY2, STM32_FLASH_OPTKEYR);

  /* Write options */

  putreg32(val, reg);

  /* Trigger programming */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  /* Wait for completion */

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
    }

  /* Re-lock options */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTLOCK);
  return 0;
}

size_t up_progmem_pagesize(size_t page)
{
  return PAGESIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr >= flash_base())
    {
      addr -= flash_base();
    }

  if (addr >= flash_size())
    {
      return -EFAULT;
    }

  return addr / PAGESIZE;
}

size_t up_progmem_getaddress(size_t page)
{
  size_t base_address = flash_base();

  if (page >= flash_size() / PAGESIZE)
    {
      return SIZE_MAX;
    }

  base_address += PAGESIZE * page;

  return base_address;
}

size_t up_progmem_neraseblocks(void)
{
  return PROGMEM_NBLOCKS;
}

bool up_progmem_isuniform(void)
{
  size_t size = up_progmem_pagesize(0);
  int i;

  for (i = 1; i < PROGMEM_NBLOCKS; i++)
    {
      if (up_progmem_pagesize(i) != size)
        {
          return false;
        }
    }

  return true;
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= flash_size() / PAGESIZE)
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
  if (block >= PROGMEM_NBLOCKS)
    {
      return -EFAULT;
    }

  sem_lock();

  /* Get flash ready and begin erasing single block */

  flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_SER);
  modifyreg32(STM32_FLASH_CR, FLASH_CR_SNB_MASK,
      FLASH_CR_SNB((block + CONFIG_STM32F7_PROGMEM_STARTBLOCK)));
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_SER, 0);
  sem_unlock();

  /* Verify */

  if (up_progmem_ispageerased(block) == 0)
    {
      return up_progmem_pagesize(block); /* success */
    }
  else
    {
      return -EIO; /* failure */
    }
}

size_t up_progmem_erasesize(size_t block)
{
  block = block + CONFIG_STM32F7_PROGMEM_STARTBLOCK;

  if (block >= sizeof(page_sizes) / sizeof(*page_sizes))
    {
      return 0;
    }
  else
    {
      return page_sizes[block];
    }
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  uint8_t *byte = (uint8_t *)buf;
  size_t written = count;
  uintptr_t flash_base;

  /* Check for valid address range */

  if (addr >= STM32_FLASH_BASE &&
      addr + count <= STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
      flash_base = up_progmem_getaddress(0);
    }
  else if (addr >= STM32_OPT_BASE &&
           addr + count <= STM32_OPT_BASE + STM32_OPT_SIZE)
    {
      flash_base = STM32_OPT_BASE;
    }
  else
    {
      return -EFAULT;
    }

  addr -= flash_base;

  sem_lock();

  /* Get flash ready and begin flashing */

  flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

  /* TODO: implement up_progmem_write() to support other sizes than 8-bits */

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PSIZE_MASK, FLASH_CR_PSIZE_X8);

  for (addr += flash_base; count; count -= 1, byte++, addr += 1)
    {
      /* Write half-word and wait to complete */

      putreg8(*byte, addr);

      /* Data synchronous Barrier (DSB) just after the write operation. This
       * will force the CPU to respect the sequence of instruction (no
       * optimization).
       */

      ARM_DSB();

      while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
        {
        }

      /* Verify */

      if (getreg32(STM32_FLASH_SR) & FLASH_CR_SER)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          sem_unlock();
          return -EROFS;
        }

      if (getreg8(addr) != *byte)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          sem_unlock();
          return -EIO;
        }
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);

  sem_unlock();
  return written;
}
