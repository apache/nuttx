/****************************************************************************
 * arch/arm/src/stm32h7/stm32h7b3xx_flash.c
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
 * Ported from stm32h743xx_flash.c, this is the original license:
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
#include <nuttx/mutex.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "barriers.h"

#include "hardware/stm32_flash.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash size is known from the chip selection:
 *
 *   When CONFIG_STM32H7_FLASH_OVERRIDE_DEFAULT is set the
 *   CONFIG_STM32H7_FLASH_CONFIG_x selects the default FLASH size based on
 *   the chip part number. This value can be overridden with
 *   CONFIG_STM32H7_FLASH_OVERRIDE_x
 *
 *   Parts STM32H7x3xG have 1024Kb of FLASH
 *   Parts STM32H7x3xI have 2048Kb of FLASH
 *
 *   N.B. Only Single bank mode is supported
 */

#define _K(x) ((x)*1024)

#define FLASH_SECTOR_SIZE  _K(8)

#define FLASH_PAGE_SIZE        16

#if !defined(CONFIG_STM32H7_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32H7_FLASH_OVERRIDE_G) && \
    !defined(CONFIG_STM32H7_FLASH_OVERRIDE_I) && \
    !defined(CONFIG_STM32H7_FLASH_CONFIG_G) && \
    !defined(CONFIG_STM32H7_FLASH_CONFIG_I)
#  define CONFIG_STM32H7_FLASH_OVERRIDE_G
#  warning "Flash size not defined defaulting to 1024KiB (G)"
#endif

#if !defined(CONFIG_STM32H7_FLASH_OVERRIDE_DEFAULT)

#  undef CONFIG_STM32H7_FLASH_CONFIG_B
#  undef CONFIG_STM32H7_FLASH_CONFIG_G
#  undef CONFIG_STM32H7_FLASH_CONFIG_I

#  if defined(CONFIG_STM32H7_FLASH_OVERRIDE_G)

#    define CONFIG_STM32H7_FLASH_CONFIG_G

#  elif defined(CONFIG_STM32H7_FLASH_OVERRIDE_I)

#    define CONFIG_STM32H7_FLASH_CONFIG_I

#  endif
#endif

#if defined(CONFIG_STM32H7_FLASH_CONFIG_G)

#  define STM32_FLASH_NBLOCKS      128
#  define STM32_FLASH_SIZE        _K(128 * 8)

#elif defined(CONFIG_STM32H7_FLASH_CONFIG_I)

#  define STM32_FLASH_NBLOCKS      256
#  define STM32_FLASH_SIZE        _K(256 * 8)

#endif

#define FLASH_KEY1           0x45670123
#define FLASH_KEY2           0xcdef89ab
#define FLASH_OPTKEY1        0x08192a3b
#define FLASH_OPTKEY2        0x4c5d6e7f
#define FLASH_ERASEDVALUE    0xffu
#define FLASH_ERASEDVALUE_DW 0xffffffffu
#define PROGMEM_NBLOCKS      STM32_FLASH_NBLOCKS
#define FLASH_NPAGES         (STM32_FLASH_SIZE / FLASH_PAGE_SIZE)

#define FLASH_TIMEOUT_VALUE 5000000  /* 5s */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32h7_flash_priv_s
{
  mutex_t  lock;    /* Bank exclusive */
  uint32_t ifbase;  /* FLASHIF interface base address */
  uint32_t base;    /* FLASH base address */
  uint32_t stblock; /* The first Block Number */
  uint32_t stpage;  /* The first Page Number */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32h7_flash_priv_s stm32h7_flash_bank1_priv =
{
  .lock    = NXMUTEX_INITIALIZER,
  .ifbase  = STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET,
  .base    = STM32_FLASH_BANK1,
  .stblock = 0,
  .stpage  = 0,
};
#if STM32_FLASH_NBLOCKS > 1
static struct stm32h7_flash_priv_s stm32h7_flash_bank2_priv =
{
  .lock    = NXMUTEX_INITIALIZER,
  .ifbase  = STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET,
  .base    = STM32_FLASH_BANK2,
  .stblock = PROGMEM_NBLOCKS / 2,
  .stpage  = FLASH_NPAGES / 2,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h7_flash_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t stm32h7_flash_getreg32(struct stm32h7_flash_priv_s
                                            *priv, uint32_t offset)
{
  return getreg32(priv->ifbase + offset);
}

/****************************************************************************
 * Name: stm32h7_flash_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32h7_flash_putreg32(struct stm32h7_flash_priv_s
                                          *priv, uint32_t offset,
                                          uint32_t value)
{
  putreg32(value, priv->ifbase + offset);
}

/****************************************************************************
 * Name: stm32h7_flash_modifyreg32
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32h7_flash_modifyreg32(struct stm32h7_flash_priv_s
                                             *priv, uint32_t offset,
                                             uint32_t clearbits,
                                             uint32_t setbits)
{
  modifyreg32(priv->ifbase + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: stm32h7_unlock_flash
 *
 * Description:
 *   Unlock the Bank
 *
 ****************************************************************************/

static void stm32h7_unlock_flash(struct stm32h7_flash_priv_s *priv)
{
  while (stm32h7_flash_getreg32(priv, STM32_FLASH_SR_OFFSET) & FLASH_SR_BSY)
    {
    }

  if (stm32h7_flash_getreg32(priv, STM32_FLASH_CR_OFFSET) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      stm32h7_flash_putreg32(priv, STM32_FLASH_KEYR_OFFSET, FLASH_KEY1);
      stm32h7_flash_putreg32(priv, STM32_FLASH_KEYR_OFFSET, FLASH_KEY2);
    }
}

/****************************************************************************
 * Name: stm32h7_lock_flash
 *
 * Description:
 *   Lock the Bank
 *
 ****************************************************************************/

static void stm32h7_lock_flash(struct stm32h7_flash_priv_s *priv)
{
  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET, 0, FLASH_CR_LOCK);
}

/****************************************************************************
 * Name: stm32h7_flash_size
 *
 * Description:
 *   Returns the size in bytes of FLASH
 *
 ****************************************************************************/

static inline uint32_t stm32h7_flash_size(
    struct stm32h7_flash_priv_s *priv)
{
  return FLASH_SECTOR_SIZE * PROGMEM_NBLOCKS;
}

/****************************************************************************
 * Name: stm32h7_flash_bank
 *
 * Description:
 *   Returns the priv pointer to the correct bank
 *
 ****************************************************************************/

static inline
struct stm32h7_flash_priv_s * stm32h7_flash_bank(size_t address)
{
  struct stm32h7_flash_priv_s *priv = &stm32h7_flash_bank1_priv;
  if (address < priv->base || address >=
      priv->base + stm32h7_flash_size(priv))
    {
      return NULL;
    }

#if STM32_FLASH_NBLOCKS > 1
  if (address >= stm32h7_flash_bank2_priv.base)
    {
      priv = &stm32h7_flash_bank2_priv;
    }
#endif

  return priv;
}

/****************************************************************************
 * Name: stm32h7_israngeerased
 *
 * Description:
 *   Returns count of non-erased words
 *
 ****************************************************************************/

static int stm32h7_israngeerased(size_t startaddress, size_t size)
{
  uint32_t *addr;
  uint8_t *baddr;
  size_t count = 0;
  size_t bwritten = 0;

  if (!stm32h7_flash_bank(startaddress) ||
      !stm32h7_flash_bank(startaddress + size - 1))
    {
      return -EIO;
    }

  addr = (uint32_t *)startaddress;
  while (count + 4 <= size)
    {
      if (getreg32(addr) != FLASH_ERASEDVALUE_DW)
        {
          bwritten++;
        }

      addr++;
      count += 4;
    }

  baddr = (uint8_t *)addr;
  while (count < size)
    {
      if (getreg8(baddr) != FLASH_ERASEDVALUE)
        {
          bwritten++;
        }

      baddr++;
      count++;
    }

  return bwritten;
}

/****************************************************************************
 * Name: stm32h7_wait_for_last_operation()
 *
 * Description:
 *   Wait for last write/erase operation to finish
 *   Return error in case of timeout
 *
 * Input Parameters:
 *   priv  - Flash bank based config
 *
 * Returned Value:
 *     Zero or error value
 *
 *     ETIME:  Timeout while waiting for previous write/erase operation to
 *             complete
 *
 ****************************************************************************/

static int stm32h7_wait_for_last_operation(struct stm32h7_flash_priv_s
                                           *priv)
{
  int i;
  bool timeout = true;

  ARM_DSB();

  for (i = 0; i < FLASH_TIMEOUT_VALUE; i++)
    {
      if (!(stm32h7_flash_getreg32(priv, STM32_FLASH_SR_OFFSET) &
          (FLASH_SR_QW | FLASH_SR_BSY | FLASH_SR_WBNE)))
        {
          timeout = false;
          break;
        }

      up_udelay(1);
    }

  if (timeout)
    {
      return -EBUSY;
    }

  return 0;
}

/****************************************************************************
 * Name: stm32h7_unlock_flashopt
 *
 * Description:
 *   Unlock the flash option bytes
 *   Returns true if the flash was locked before, false otherwise
 *
 ****************************************************************************/

static bool stm32h7_unlock_flashopt(struct stm32h7_flash_priv_s *priv)
{
  bool ret = false;

  while (stm32h7_flash_getreg32(priv, STM32_FLASH_SR_OFFSET) & FLASH_SR_BSY)
    {
    }

  if (stm32h7_flash_getreg32(priv, STM32_FLASH_OPTCR_OFFSET) &
                             FLASH_OPTCR_OPTLOCK)
    {
      /* Unlock sequence */

      stm32h7_flash_putreg32(priv, STM32_FLASH_OPTKEYR_OFFSET,
                             FLASH_OPTKEY1);
      stm32h7_flash_putreg32(priv, STM32_FLASH_OPTKEYR_OFFSET,
                             FLASH_OPTKEY2);

      /* Was locked before and now unlocked */

      ret = true;
    }

  return ret;
}

/****************************************************************************
 * Name: stm32h7_lock_flashopt
 *
 * Description:
 *   Lock the flash option bytes
 *
 ****************************************************************************/

static void stm32h7_lock_flashopt(struct stm32h7_flash_priv_s *priv)
{
  stm32h7_flash_modifyreg32(priv, STM32_FLASH_OPTCR_OFFSET, 0,
                            FLASH_OPTCR_OPTLOCK);
}

/****************************************************************************
 * Name: stm32h7_save_flashopt
 *
 * Description:
 *   Save the flash option bytes to non-volatile storage.
 *
 ****************************************************************************/

static void stm32h7_save_flashopt(struct stm32h7_flash_priv_s *priv)
{
  while (stm32h7_flash_getreg32(priv, STM32_FLASH_SR_OFFSET) &
        (FLASH_SR_BSY | FLASH_SR_CRCBUSY))
    {
    }

  /* Can only write flash options if the option control reg is unlocked */

  if (!(stm32h7_flash_getreg32(priv, STM32_FLASH_OPTCR_OFFSET) &
                               FLASH_OPTCR_OPTLOCK))
    {
      stm32h7_flash_modifyreg32(priv, STM32_FLASH_OPTCR_OFFSET, 0,
                                FLASH_OPTCR_OPTSTRT);
    }

  /* Wait for the update to complete */

  while (stm32h7_flash_getreg32(priv, STM32_FLASH_OPTSR_CUR_OFFSET) &
                                      FLASH_OPTSR_BUSYV)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h7_flash_unlock
 *
 * Description:
 *   Unlocks a bank
 *
 ****************************************************************************/

int stm32h7_flash_unlock(size_t addr)
{
  int ret = -ENODEV;
  struct stm32h7_flash_priv_s *priv = stm32h7_flash_bank(addr);

  if (priv)
    {
      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      stm32h7_unlock_flash(priv);
      nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32h7_flash_lock
 *
 * Description:
 *   Locks a bank
 *
 ****************************************************************************/

int stm32h7_flash_lock(size_t addr)
{
  int ret = -ENODEV;
  struct stm32h7_flash_priv_s *priv = stm32h7_flash_bank(addr);

  if (priv)
    {
      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      stm32h7_lock_flash(priv);
      nxmutex_unlock(&priv->lock);
    }

  return ret;
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
          clearbits = 1 << block % (STM32_FLASH_NBLOCKS / 2);
        }
      else
        {
          setbits = 1 << block % (STM32_FLASH_NBLOCKS / 2);
        }

      stm32h7_flash_modifyreg32(priv, STM32_FLASH_WPSN_PRGR_OFFSET,
                                clearbits, setbits);
      rv  = OK;
    }

  return rv;
}

/****************************************************************************
 * Name: stm32h7_flash_getopt
 *
 * Description:
 *   Returns the current flash option bytes from the FLASH_OPTSR_CR register.
 *
 ****************************************************************************/

uint32_t stm32h7_flash_getopt(void)
{
  struct stm32h7_flash_priv_s *priv;
  priv = stm32h7_flash_bank(STM32_FLASH_BANK1);
  if (priv)
    {
      return stm32h7_flash_getreg32(priv, STM32_FLASH_OPTSR_CUR_OFFSET);
    }

  return 0;
}

/****************************************************************************
 * Name: stm32h7_flash_optmodify
 *
 * Description:
 *   Modifies the current flash option bytes, given bits to set and clear.
 *
 ****************************************************************************/

void stm32h7_flash_optmodify(uint32_t clear, uint32_t set)
{
  struct stm32h7_flash_priv_s *priv;
  bool was_locked;

  priv = stm32h7_flash_bank(STM32_FLASH_BANK1);
  if (priv)
    {
      was_locked = stm32h7_unlock_flashopt(priv);
      stm32h7_flash_modifyreg32(priv, STM32_FLASH_OPTSR_PRG_OFFSET,
                                clear, set);
      stm32h7_save_flashopt(priv);
      if (was_locked)
        {
          stm32h7_lock_flashopt(priv);
        }
    }
}

/****************************************************************************
 * Name: stm32h7_flash_swapbanks
 *
 * Description:
 *   Swaps banks 1 and 2 in the processor's memory map.  Takes effect
 *   the next time the system is reset.
 *
 ****************************************************************************/

void stm32h7_flash_swapbanks(void)
{
  uint32_t opts = stm32h7_flash_getopt();
  if (opts & FLASH_OPTCR_SWAPBANK)
    {
      stm32h7_flash_optmodify(FLASH_OPTCR_SWAPBANK, 0);
    }
  else
    {
      stm32h7_flash_optmodify(0, FLASH_OPTCR_SWAPBANK);
    }
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

  return  priv->stpage + ((addr - priv->base) / FLASH_PAGE_SIZE);
}

size_t up_progmem_getaddress(size_t block)
{
  struct stm32h7_flash_priv_s *priv;
  if (block >= STM32_FLASH_NBLOCKS)
    {
      return SIZE_MAX;
    }

  priv = stm32h7_flash_bank(STM32_FLASH_BANK1 + (block * FLASH_SECTOR_SIZE));
  return priv->base + (block - priv->stblock) * FLASH_SECTOR_SIZE;
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

  if (page >= FLASH_NPAGES)
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

size_t up_progmem_erasesize(size_t block)
{
  return FLASH_SECTOR_SIZE;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  struct stm32h7_flash_priv_s *priv;
  int ret;
  size_t block_address = STM32_FLASH_BANK1 + (block * FLASH_SECTOR_SIZE);

  if (block >= PROGMEM_NBLOCKS)
    {
      return -EFAULT;
    }

  priv = stm32h7_flash_bank(block_address);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  if (stm32h7_wait_for_last_operation(priv))
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  /* Get flash ready and begin erasing single block */

  stm32h7_unlock_flash(priv);

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET, 0, FLASH_CR_SER);
  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET, FLASH_CR_SSN_MASK,
      FLASH_CR_SSN(block - priv->stblock));

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET,
                            0, FLASH_CR_SER | FLASH_CR_START);

  /* Wait for erase operation to complete */

  if (stm32h7_wait_for_last_operation(priv))
    {
      ret = -EIO;
      goto exit_with_unlock;
    }

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET, FLASH_CR_SER, 0);
  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET, FLASH_CR_SSN_MASK,
                            0);

  ret = 0;

  up_invalidate_dcache(block_address, block_address + FLASH_SECTOR_SIZE);

exit_with_unlock:
  stm32h7_lock_flash(priv);

exit_with_lock:
  nxmutex_unlock(&priv->lock);

  /* Verify */

  if (ret == 0 &&
      stm32h7_israngeerased(block_address, up_progmem_erasesize(block)) == 0)
    {
      ret = up_progmem_erasesize(block); /* success */
    }
  else
    {
      ret = -EIO; /* failure */
    }

  return ret;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  struct stm32h7_flash_priv_s *priv;
  uint32_t     *fp;
  uint32_t     *rp;
  uint32_t     *ll        = (uint32_t *)buf;
  size_t       faddr;
  size_t       written    = count;
  int          ret;
  const size_t pagesize   = 16;  /* 128 bit, 16 bytes per page */
  const size_t llperpage  = pagesize / sizeof(uint32_t);
  size_t       pcount     = count / pagesize;
  uint32_t     sr;

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

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Check address and count alignment */

  DEBUGASSERT(!(addr % pagesize));
  DEBUGASSERT(!(count % pagesize));

  if (stm32h7_wait_for_last_operation(priv))
    {
      written = -EIO;
      goto exit_with_lock;
    }

  /* Get flash ready and begin flashing */

  stm32h7_unlock_flash(priv);

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET, 0, FLASH_CR_PG);

  for (ll = (uint32_t *)buf, faddr = addr; pcount;
      pcount -= 1, ll += llperpage, faddr += pagesize)
    {
      fp = (uint32_t *)faddr;
      rp = ll;

      ARM_DSB();
      ARM_ISB();

      /* Write 4 32 bit word and wait to complete */

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

      if (stm32h7_wait_for_last_operation(priv))
        {
          written = -EIO;
          goto exit_with_unlock;
        }

      sr = stm32h7_flash_getreg32(priv, STM32_FLASH_SR_OFFSET);
      if (sr & (FLASH_SR_SNECCERR | FLASH_SR_DBECCERR))
        {
          stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET,
                                    FLASH_CR_PG,
                                    0);

          stm32h7_flash_modifyreg32(priv, STM32_FLASH_CCR_OFFSET,
                                    0, ~0);
          ret = -EIO;
          goto exit_with_unlock;
        }
    }

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CR_OFFSET, FLASH_CR_PG, 0);

  stm32h7_flash_modifyreg32(priv, STM32_FLASH_CCR_OFFSET,
                            0, ~0);
exit_with_unlock:
  stm32h7_lock_flash(priv);

  /* Verify */

  if (written > 0)
    {
      for (ll = (uint32_t *)buf, faddr = addr, pcount = count / pagesize;
          pcount; pcount -= 1, ll += llperpage, faddr += pagesize)
        {
          fp = (uint32_t *)faddr;
          rp = ll;

          stm32h7_flash_modifyreg32(priv, STM32_FLASH_CCR_OFFSET,
                                    0, ~0);
          if ((*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++))
            {
              written = -EIO;
              break;
            }

          sr = stm32h7_flash_getreg32(priv, STM32_FLASH_SR_OFFSET);
          if (sr & (FLASH_SR_SNECCERR | FLASH_SR_DBECCERR))
            {
              written = -EIO;
              break;
            }
        }

      stm32h7_flash_modifyreg32(priv, STM32_FLASH_CCR_OFFSET,
                                0, ~0);
    }

exit_with_lock:
  nxmutex_unlock(&priv->lock);
  return written;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}
