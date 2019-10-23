/****************************************************************************
 * arch/arm/src/stm32h7/stm32h7_flash.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david.sidrane@nscdg.com>
 *
 * Ported from stm32f7_flash.c, this is the original license:
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

/* Flash size is known from the chip selection:
 *
 *   When CONFIG_STM32H7_FLASH_OVERRIDE_DEFAULT is set the
 *   CONFIG_STM32H7_FLASH_CONFIG_x selects the default FLASH size based on the
 *   chip part number. This value can be overridden with
 *   CONFIG_STM32H7_FLASH_OVERRIDE_x
 *
 *   Parts STM32H74xxE have 512Kb of FLASH
 *   Parts STM32H74xxG have 1024Kb of FLASH
 *   Parts STM32H74xxI have 2048Kb of FLASH
 *
 *   N.B. Only Single bank mode is supported
 */

#define _K(x) ((x)*1024)
#define FLASH_SECTOR_SIZE  _K(128)

#if !defined(CONFIG_STM32H7_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32H7_FLASH_OVERRIDE_B) && \
    !defined(CONFIG_STM32H7_FLASH_OVERRIDE_G) && \
    !defined(CONFIG_STM32H7_FLASH_OVERRIDE_I) && \
    !defined(CONFIG_STM32H7_FLASH_CONFIG_B) && \
    !defined(CONFIG_STM32H7_FLASH_CONFIG_G) && \
    !defined(CONFIG_STM32H7_FLASH_CONFIG_I)
#  define CONFIG_STM32H7_FLASH_OVERRIDE_B
#  warning "Flash size not defined defaulting to 128KiB (B)"
#endif

#if !defined(CONFIG_STM32H7_FLASH_OVERRIDE_DEFAULT)

#  undef CONFIG_STM32H7_FLASH_CONFIG_B
#  undef CONFIG_STM32H7_FLASH_CONFIG_G
#  undef CONFIG_STM32H7_FLASH_CONFIG_I

#  if defined(CONFIG_STM32H7_FLASH_OVERRIDE_B)

#    define CONFIG_STM32H7_FLASH_CONFIG_B

#  elif defined(CONFIG_STM32H7_FLASH_OVERRIDE_G)

#    define CONFIG_STM32H7_FLASH_CONFIG_G

#  elif defined(CONFIG_STM32H7_FLASH_OVERRIDE_I)

#    define CONFIG_STM32H7_FLASH_CONFIG_I

#  endif
#endif

#if defined(CONFIG_STM32H7_FLASH_CONFIG_B)

#  define STM32_FLASH_NPAGES      1
#  define STM32_FLASH_SIZE        _K(1 * 128)

#elif defined(CONFIG_STM32H7_FLASH_CONFIG_G)

#  define STM32_FLASH_NPAGES      8
#  define STM32_FLASH_SIZE        _K(8 * 128)

#elif defined(CONFIG_STM32H7_FLASH_CONFIG_I)

#  define STM32_FLASH_NPAGES      16
#  define STM32_FLASH_SIZE        _K(16 * 128)

#endif

#define FLASH_KEY1         0x45670123
#define FLASH_KEY2         0xcdef89ab
#define FLASH_OPTKEY1      0x08192a3b
#define FLASH_OPTKEY2      0x4c5d6e7f
#define FLASH_ERASEDVALUE  0xff

#define PROGMEM_NBLOCKS STM32_FLASH_NPAGES

/*****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32h7_flash_priv_s
{
  sem_t    sem;     /* Bank exclusive */
  uint32_t ifbase;  /* FLASHIF interface base address */
  uint32_t base;    /* FLASH base address */
  uint32_t stblock; /* The first Block Number */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32h7_flash_priv_s stm32h7_flash_bank1_priv =
{
  .sem     = SEM_INITIALIZER(1),
  .ifbase  = STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET,
  .base    = STM32_FLASH_BANK1,
  .stblock = 0,
};
#if STM32_FLASH_NPAGES > 1
static struct stm32h7_flash_priv_s stm32h7_flash_bank2_priv =
{
  .sem = SEM_INITIALIZER(1),
  .ifbase = STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET,
  .base   = STM32_FLASH_BANK2,
  .stblock = PROGMEM_NBLOCKS / 2,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32h7_flash_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 *****************************************************************************/

static inline uint32_t stm32h7_flash_getreg32(FAR struct stm32h7_flash_priv_s
                                            *priv, uint8_t offset)
{
  return getreg32(priv->ifbase + offset);
}

/*****************************************************************************
 * Name: stm32h7_flash_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32h7_flash_putreg32(FAR struct stm32h7_flash_priv_s
                                          *priv, uint8_t offset,
                                          uint32_t value)
{
  putreg32(value, priv->ifbase + offset);
}

/*****************************************************************************
 * Name: stm32h7_flash_modifyreg32
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 *****************************************************************************/

static inline void stm32h7_flash_modifyreg32(FAR struct stm32h7_flash_priv_s
                                             *priv, uint8_t offset,
                                             uint32_t clearbits,
                                             uint32_t setbits)
{
  modifyreg32(priv->ifbase + offset, clearbits, setbits);
}

/*****************************************************************************
 * Name: stm32h7_flash_sem_lock
 *
 * Description:
 *   Take the Bank exclusive access semaphore
 *
 *****************************************************************************/

static void stm32h7_flash_sem_lock(FAR struct stm32h7_flash_priv_s *priv)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = sem_wait(&priv->sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/*****************************************************************************
 * Name: stm32h7_flash_sem_unlock
 *
 * Description:
 *   Release the Bank exclusive access semaphore
 *
 *****************************************************************************/

static inline void stm32h7_flash_sem_unlock(FAR struct stm32h7_flash_priv_s
                                            *priv)
{
  sem_post(&priv->sem);
}

/*****************************************************************************
 * Name: stm32h7_unlock_flash
 *
 * Description:
 *   Unlock the Bank
 *
 *****************************************************************************/

static void stm32h7_unlock_flash(FAR struct stm32h7_flash_priv_s *priv)
{
  while (stm32h7_flash_getreg32(priv, STM32_FLASH_SR1_OFFSET) & FLASH_SR_BSY)
    {
    }

  if (stm32h7_flash_getreg32(priv, STM32_FLASH_CR1_OFFSET) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      stm32h7_flash_putreg32(priv, STM32_FLASH_KEYR1_OFFSET, FLASH_KEY1);
      stm32h7_flash_putreg32(priv, STM32_FLASH_KEYR1_OFFSET, FLASH_KEY2);
    }
}

/*****************************************************************************
 * Name: stm32h7_lock_flash
 *
 * Description:
 *   Lock the Bank
 *
 *****************************************************************************/

static void stm32h7_lock_flash(FAR struct stm32h7_flash_priv_s *priv)
{
  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, 0, FLASH_CR_LOCK);
}

/*****************************************************************************
 * Name: stm32h7_flash_size
 *
 * Description:
 *   Returns the size in bytes of FLASH
 *
 *****************************************************************************/

static inline uint32_t stm32h7_flash_size(FAR struct stm32h7_flash_priv_s *priv)
{
  return FLASH_SECTOR_SIZE * PROGMEM_NBLOCKS;
}
/*****************************************************************************
 * Name: stm32h7_flash_bank
 *
 * Description:
 *   Returns the priv pointer to the correct bank
 *
 *****************************************************************************/

static inline
FAR struct stm32h7_flash_priv_s * stm32h7_flash_bank(size_t address)
{
  struct stm32h7_flash_priv_s *priv = &stm32h7_flash_bank1_priv;
  if (address < priv->base || address >= priv->base + stm32h7_flash_size(priv))
    {
      return NULL;
    }
#if STM32_FLASH_NPAGES > 1
  if (address >= stm32h7_flash_bank2_priv.base)
    {
      priv = &stm32h7_flash_bank2_priv;
    }
#endif

  return priv;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32h7_flash_unlock
 *
 * Description:
 *   Unlocks a bank
 *
 *****************************************************************************/

int stm32h7_flash_unlock(size_t addr)
{
  int rv = -ENODEV;
  struct stm32h7_flash_priv_s *priv = stm32h7_flash_bank(addr);

  if (priv)
    {
      rv = OK;
      stm32h7_flash_sem_lock(priv);
      stm32h7_unlock_flash(priv);
      stm32h7_flash_sem_unlock(priv);
    }

  return rv;
}

/*****************************************************************************
 * Name: stm32h7_flash_lock
 *
 * Description:
 *   Locks a bank
 *
 *****************************************************************************/

int stm32h7_flash_lock(size_t addr)
{
  int rv = -ENODEV;
  struct stm32h7_flash_priv_s *priv = stm32h7_flash_bank(addr);

  if (priv)
    {
      rv = OK;
      stm32h7_flash_sem_lock(priv);
      stm32h7_lock_flash(priv);
      stm32h7_flash_sem_unlock(priv);
    }

  return rv;
}

/****************************************************************************
 * Name: stm32h7_flash_writeprotect
 *
 * Description:
 *   Enable or disable the write protection of a flash sector.
 *
 ****************************************************************************/

int stm32h7_flash_writeprotect(size_t block, bool enabled)
{
  struct stm32h7_flash_priv_s *priv;
  uint32_t setbits   = 0;
  uint32_t clearbits = 0;
  int      rv        = -ENODEV;

  if (block >= PROGMEM_NBLOCKS)
    {
      return -EFAULT;
    }

  priv = stm32h7_flash_bank(STM32_FLASH_BANK1 + (block * FLASH_SECTOR_SIZE));

  if (priv)
    {
      if (enabled)
        {
          clearbits = 1 << block % (STM32_FLASH_NPAGES / 2);
        }
      else
        {
          setbits = 1 << block % (STM32_FLASH_NPAGES / 2);
        }

      stm32h7_flash_modifyreg32(priv, STM32_FLASH_WPSN_PRG1R_OFFSET,
                                clearbits, setbits);
      rv  = OK;
    }

  return rv;
}

size_t up_progmem_pagesize(size_t page)
{
  return FLASH_SECTOR_SIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  struct stm32h7_flash_priv_s *priv;

  priv = stm32h7_flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }
  return  priv->stblock + ((addr - priv->base) / FLASH_SECTOR_SIZE);
}

size_t up_progmem_getaddress(size_t page)
{
  struct stm32h7_flash_priv_s *priv;

  if (page >= STM32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  priv = stm32h7_flash_bank(STM32_FLASH_BANK1 + (page * FLASH_SECTOR_SIZE));
  return priv->base + (page - priv->stblock) * FLASH_SECTOR_SIZE;
}

size_t up_progmem_neraseblocks(void)
{
  return PROGMEM_NBLOCKS;
}

bool up_progmem_isuniform(void)
{
  return true;
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
  struct stm32h7_flash_priv_s *priv;

  if (block >= PROGMEM_NBLOCKS)
    {
      return -EFAULT;
    }

  priv = stm32h7_flash_bank(STM32_FLASH_BANK1 + (block * FLASH_SECTOR_SIZE));

  stm32h7_flash_sem_lock(priv);

  /* Get flash ready and begin erasing single block */

  stm32h7_unlock_flash(priv);

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, 0, FLASH_CR_SER);
  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, FLASH_CR_SNB_MASK,
                            FLASH_CR_SNB(block));

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, 0, FLASH_CR_START);

  while (stm32h7_flash_getreg32(priv, STM32_FLASH_CR1_OFFSET) & FLASH_SR_BSY)
    {
    }

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, FLASH_CR_SER, 0);
  stm32h7_flash_sem_unlock(priv);

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
  return FLASH_SECTOR_SIZE;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  struct stm32h7_flash_priv_s *priv;
  uint64_t     *fp;
  uint64_t     *rp;
  uint8_t      *byte      = (uint8_t *) buf;
  uint64_t     *ll        = (uint64_t *) buf;
  size_t       written    = count;
  const size_t blocksize  = 32; /* 256 bit, 32 bytes per block */
  const size_t llperblock = blocksize / sizeof(uint64_t);
  size_t       bcount     = count / blocksize;
  size_t       remaining  = count % blocksize;

  priv = stm32h7_flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }

  /* Check for valid address range */

  if (addr < priv->base ||
      addr + count > priv->base + (STM32_FLASH_SIZE / 2))
    {
      return -EFAULT;
    }

  stm32h7_flash_sem_lock(priv);

  /* Get flash ready and begin flashing */

  stm32h7_unlock_flash(priv);

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, 0, FLASH_CR_PG);

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, FLASH_CR_PSIZE_MASK,
                            FLASH_CR_PSIZE_X64);

  ARM_DSB();
  ARM_ISB();

  for (ll = (uint64_t *) buf; bcount;
      bcount -= 1, ll += llperblock, addr += blocksize)
    {
      fp = (uint64_t *) addr;
      rp = ll;

      /* Write 4 64 bit word and wait to complete */

      *fp++ = *rp++;
      *fp++ = *rp++;
      *fp++ = *rp++;
      *fp++ = *rp++;

      /* Data synchronous Barrier (DSB) just after the write operation. This
       * will force the CPU to respect the sequence of instruction (no
       * optimization).
       */

      ARM_DSB();
      ARM_ISB();

      while (stm32h7_flash_getreg32(priv, STM32_FLASH_SR1_OFFSET) &
             FLASH_SR_BSY)
        {
        }

      /* Verify */

      if (stm32h7_flash_getreg32(priv, STM32_FLASH_SR1_OFFSET) & FLASH_CR_SER)
        {
          written = -EROFS;
          break;
        }
      else
        {
          fp = (uint64_t *) addr;
          rp = ll;

          if (*fp++ != *rp++ ||
              *fp++ != *rp++ ||
              *fp++ != *rp++ ||
              *fp++ != *rp++)
            {
              written = -EIO;
              break;
            }
        }
    }

  if (remaining)
    {
      for (byte = (uint8_t *) ll, count = remaining; count;
           count -= 1, byte++, addr += 1)
        {
          /* Write the remaining */

          putreg8(*byte, addr);
        }

      /* Data synchronous Barrier (DSB) just after the write operation. This
       * will force the CPU to respect the sequence of instruction (no
       * optimization).
       */

      ARM_DSB();
      ARM_ISB();

      /* Force the fractional write */

      stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, 0, FLASH_CR_FW);

      ARM_DSB();
      ARM_ISB();

      while (stm32h7_flash_getreg32(priv, STM32_FLASH_CR1_OFFSET) &
             FLASH_CR_FW)
        {
        }

      while (stm32h7_flash_getreg32(priv, STM32_FLASH_SR1_OFFSET) &
             FLASH_SR_BSY)
        {
        }

      /* Verify */

      if (stm32h7_flash_getreg32(priv, STM32_FLASH_SR1_OFFSET) & FLASH_CR_SER)
        {
          written = -EROFS;
        }
      else
        {
          addr -= remaining;
          for (byte = (uint8_t *) ll, count = remaining; count;
               count -= 1, byte++, addr += 1)
            {
              if (getreg8(addr) != *byte)
                {
                  written = -EIO;
                  break;
                }
            }
        }
    }
  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, FLASH_CR_PG, 0);
  stm32h7_flash_sem_unlock(priv);
  return written;
}
