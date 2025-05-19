/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32g0_flash.c
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
 * driver. The interface is defined in the include/nuttx/progmem.h
 *
 * Notes:
 *   - Terminology: the G0xx reference manual [RM0444] refers to erase blocks
 *     as 'pages'. In this file, erase blocks are referred to as 'blocks' and
 *     the smallest write allowed is referred to as a 'page'. The STMicro
 *     reference manuals are not consistent in naming convention.
 *   - Blocking Nature: up_progmem_write() and up_progmem_eraseblock() will
 *     both block without releasing (up_udelay) while waiting for flash
 *     operations to complete. Take this into account for applications
 *     that use these functions.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_flash.h"

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <arch/barriers.h>

#include <arm_internal.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "hardware/stm32_flash.h"
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _K(x) ((x)*1024)
#define FLASH_BLOCK_SIZE _K(2)
#define FLASH_PAGE_SIZE     8

#if !defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_4) && \
    !defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_6) && \
    !defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_8) && \
    !defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_B) && \
    !defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_C) && \
    !defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_E) && \
    !defined(CONFIG_STM32F0L0G0_FLASH_OVERRIDE)
#  error "No valid flash configuration was defined."
#endif

#ifdef CONFIG_STM32F0L0G0_FLASH_OVERRIDE
#  undef CONFIG_STM32F0L0G0_FLASH_CONFIG_4
#  undef CONFIG_STM32F0L0G0_FLASH_CONFIG_6
#  undef CONFIG_STM32F0L0G0_FLASH_CONFIG_8
#  undef CONFIG_STM32F0L0G0_FLASH_CONFIG_B
#  undef CONFIG_STM32F0L0G0_FLASH_CONFIG_C
#  undef CONFIG_STM32F0L0G0_FLASH_CONFIG_E
#  if defined(CONFIG_STM32F0L0G0_FLASH_OVERRIDE_4)
#    define CONFIG_STM32F0L0G0_FLASH_CONFIG_4
#  elif defined(CONFIG_STM32F0L0G0_FLASH_OVERRIDE_6)
#    define CONFIG_STM32F0L0G0_FLASH_CONFIG_6
#  elif defined(CONFIG_STM32F0L0G0_FLASH_OVERRIDE_8)
#    define CONFIG_STM32F0L0G0_FLASH_CONFIG_8
#  elif defined(CONFIG_STM32F0L0G0_FLASH_OVERRIDE_B)
#    define CONFIG_STM32F0L0G0_FLASH_CONFIG_B
#  elif defined(CONFIG_STM32F0L0G0_FLASH_OVERRIDE_C)
#    define CONFIG_STM32F0L0G0_FLASH_CONFIG_C
#  elif defined(CONFIG_STM32F0L0G0_FLASH_OVERRIDE_E)
#    define CONFIG_STM32F0L0G0_FLASH_CONFIG_E
#  else
#    error "Invalid flash configuration override provided"
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_4)
#  define FLASH_NBLOCKS     8
#elif defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_6)
#  define FLASH_NBLOCKS     16
#elif defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_8)
#  define FLASH_NBLOCKS     32
#elif defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_B)
#  define FLASH_NBLOCKS     64
#elif defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_C)
#  define FLASH_NBLOCKS     128
#  define FLASH_DUAL_BANK   1
#  define FLASH_BANK2_BASE  0x08020000
#elif defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_E)
#  define FLASH_NBLOCKS     256
#  define FLASH_DUAL_BANK   1
#  define FLASH_BANK2_BASE  0x08040000
#else
#  error "Invalid flash configuration defined"
#endif

#ifdef FLASH_DUAL_BANK
#  define FLASH_BANKSIZE    (FLASH_NBLOCKS * FLASH_BLOCK_SIZE / 2)
#else
#  define FLASH_BANKSIZE    (FLASH_NBLOCKS * FLASH_BLOCK_SIZE)
#endif

/* Dual bank G0B1 MCUs have a non-linear mapping of block number between
 * banks. Bank 2 starts at block number 256, even if bank 1 ends at 63
 * or 127.
 */

#define FLASH_BANK2_START_BLOCKNUM 256

#define FLASH_TOTALSIZE     (FLASH_NBLOCKS * FLASH_BLOCK_SIZE)
#define FLASH_NPAGES        (FLASH_NBLOCKS * FLASH_BLOCK_SIZE / FLASH_PAGE_SIZE)
#define FLASH_KEY1           0x45670123
#define FLASH_KEY2           0xcdef89ab
#define FLASH_OPTKEY1        0x08192a3b
#define FLASH_OPTKEY2        0x4c5d6e7f
#define FLASH_ERASEDVALUE    0xffu
#define FLASH_ERASEDVALUE_DW 0xffffffffu

#define FLASH_TIMEOUT        5000000    /* 5s */

#define FLASH_SR_CLEAR_ERROR_FLAGS (FLASH_SR_OPERR|FLASH_SR_PROGERR|FLASH_SR_WRPERR|\
                                    FLASH_SR_PGAERR|FLASH_SR_SIZERR|FLASH_SR_PGSERR|\
                                    FLASH_SR_MISSERR|FLASH_SR_FASTERR|FLASH_SR_RDERR|FLASH_SR_OPTVERR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_flash_priv_s
{
  uint32_t base;    /* FLASH base address */
  uint32_t stblock; /* The first block number */
  uint32_t stpage;  /* The first page number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void flash_unlock_cr(void);
static void flash_lock_cr(void);
static bool flash_unlock_opt(void);
static void flash_lock_opt(void);
static int  flash_israngeerased(size_t startaddress, size_t size);
static inline struct stm32_flash_priv_s *flash_bank(size_t address);
static int flash_wait_for_operation(void);
static int flash_verify_blocknum(size_t block);
static uint32_t flash_block_address(size_t block);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_flash_priv_s flash_bank1_priv =
{
  .base    = STM32_FLASH_BASE,
  .stblock = 0,
  .stpage  = 0
};

#ifdef FLASH_DUAL_BANK
static struct stm32_flash_priv_s flash_bank2_priv =
{
  .base    = FLASH_BANK2_BASE,
  .stblock = FLASH_BANK2_START_BLOCKNUM,
  .stpage  = (FLASH_NPAGES / 2)
};
#endif

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flash_bank
 *
 * Description:
 *   Returns the priv pointer to the correct bank
 *
 ****************************************************************************/

static inline struct stm32_flash_priv_s *flash_bank(size_t address)
{
  struct stm32_flash_priv_s *priv = NULL;

  if (address >= flash_bank1_priv.base &&
      address < flash_bank1_priv.base + FLASH_BANKSIZE)
    {
      priv = &flash_bank1_priv;
    }
#ifdef FLASH_DUAL_BANK
  else if (address >= flash_bank2_priv.base &&
           address < flash_bank2_priv.base + FLASH_BANKSIZE)
    {
      priv = &flash_bank2_priv;
    }
#endif

  return priv;
}

/****************************************************************************
 * Name: flash_israngeerased
 *
 * Description:
 *   Returns count of non-erased words
 *
 ****************************************************************************/

static int  flash_israngeerased(size_t startaddress, size_t size)
{
  uint32_t *addr;
  uint8_t *baddr;
  size_t count = 0;
  size_t bwritten = 0;

  if (!flash_bank(startaddress) || !flash_bank(startaddress + size - 1))
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
          /* Technically counting more than once per word but OK since
           * anything that is non-zero is a failure anyways.
           */

          bwritten++;
        }

      baddr++;
      count++;
    }

  return bwritten;
}

/****************************************************************************
 * Name: flash_wait_for_operation()
 *
 * Description:
 *   Wait for last write/erase operation to finish
 *   Return error in case of timeout
 *
 * Returned Value:
 *     Zero or error value
 *
 *     -EBUSY: Timeout while waiting for previous write/erase operation to
 *             complete
 *
 ****************************************************************************/

static int flash_wait_for_operation(void)
{
  int i;
  bool timeout = true;

  UP_DSB();

  for (i = 0; i < FLASH_TIMEOUT; i += 10)
    {
      if (!(getreg32(STM32_FLASH_SR) &
          (FLASH_SR_CFGBSY | FLASH_SR_BSY1 | FLASH_SR_BSY2)))
        {
          timeout = false;
          break;
        }

      up_udelay(10);
    }

  if (timeout)
    {
      return -EBUSY;
    }

  return 0;
}

/****************************************************************************
 * Name: flash_unlock_cr
 *
 * Description:
 *   Unlock flash control register, if it is not already unlocked.
 *
 ****************************************************************************/

static void flash_unlock_cr(void)
{
  /* FLASH_CR cannot be written when BSY1 flag set */

  while (getreg32(STM32_FLASH_SR) & (FLASH_SR_BSY1 | FLASH_SR_CFGBSY))
    {
    }

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK)
    {
      putreg32(FLASH_KEY1, STM32_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_KEYR);
    }

  DEBUGASSERT((getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK) == 0);
}

/****************************************************************************
 * Name: flash_lock_cr
 *
 * Description:
 *   Lock flash control register.
 *
 ****************************************************************************/

static void flash_lock_cr(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_LOCK);
}

/****************************************************************************
 * Name: flash_unlock_opt
 *
 * Description:
 *   Unlock flash option bytes register, if it is not already unlocked.
 *
 ****************************************************************************/

static bool flash_unlock_opt(void)
{
  bool was_locked = false;
  flash_unlock_cr();

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_OPTLOCK)
    {
      was_locked = true;

      putreg32(FLASH_OPTKEY1, STM32_FLASH_OPTKEYR);
      putreg32(FLASH_OPTKEY2, STM32_FLASH_OPTKEYR);
    }

  DEBUGASSERT((getreg32(STM32_FLASH_CR) & FLASH_CR_OPTLOCK) == 0);

  return was_locked;
}

/****************************************************************************
 * Name: flash_lock_opt
 *
 * Description:
 *   Lock flash option bytes register.
 *
 ****************************************************************************/

static void flash_lock_opt(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_OPTLOCK);
}

/****************************************************************************
 * Name: flash_verify_blocknum
 *
 * Description:
 *   Verify the provided block number is valid based on the flash
 *   configuration. This is done because the reference implementation and
 *   reference manual refer to non-contiguous block (page) numbers for the
 *   flash layout on dual-bank devices.
 *
 * Returned Value:
 *   Zero or negated errno value.
 *
 *   -EFAULT: Block number provided falls outside of the ranges specified in
 *            reference manual.
 *
 ****************************************************************************/

static int flash_verify_blocknum(size_t block)
{
#ifdef FLASH_DUAL_BANK
#if defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_C)
  if ((block < 0 || block > 63) && (block < 256 || block > 319))
    {
      return -EFAULT;
    }
#elif defined(CONFIG_STM32F0L0G0_FLASH_CONFIG_E)
  if ((block < 0 || block > 127) && (block < 256 || block > 383))
    {
      return -EFAULT;
    }
#else
#  error "Dual bank flash config not supported by flash driver"
#endif
#else
  if (block > FLASH_NBLOCKS)
    {
      return -EFAULT;
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: flash_block_address
 *
 * Description:
 *   Find the start address for the given block number.
 *
 * Returned Value:
 *   Memory address corresponding to given block number.
 *
 * Assumptions:
 *   This function assumes the block number has already been verified. Take
 *   care to make sure the block number is valid for the specific chip using
 *   flash_verify_blocknum() first.
 *
 ****************************************************************************/

static uint32_t flash_block_address(size_t block)
{
  uint32_t addr;
#ifdef FLASH_DUAL_BANK
  if (block >= flash_bank2_priv.stblock)
    {
      addr = flash_bank2_priv.base +
             (block - flash_bank2_priv.stblock) * FLASH_BLOCK_SIZE;
    }
  else
    {
      addr = flash_bank1_priv.base + block * FLASH_BLOCK_SIZE;
    }
#else
  addr = flash_bank1_priv.base + block * FLASH_BLOCK_SIZE;
#endif
  return addr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_flash_unlock
 *
 * Description:
 *   Unlock flash control register (FLASH_CR)
 *
 ****************************************************************************/

void stm32_flash_unlock(void)
{
  nxmutex_lock(&g_lock);
  flash_unlock_cr();
  nxmutex_unlock(&g_lock);
}

/****************************************************************************
 * Name: stm32_flash_lock
 *
 * Description:
 *   Lock flash control register (FLASH_CR)
 *
 ****************************************************************************/

void stm32_flash_lock(void)
{
  nxmutex_lock(&g_lock);
  flash_lock_cr();
  nxmutex_unlock(&g_lock);
}

/****************************************************************************
 * Name: stm32_flash_getopt
 *
 * Description:
 *   Read the current flash option bytes from FLASH_OPTR
 *
 * Input Parameters:
 *   opt - location to store read of FLASH_OPTR
 *
 ****************************************************************************/

void stm32_flash_getopt(uint32_t *opt)
{
  *opt = getreg32(STM32_FLASH_OPTR);
}

/****************************************************************************
 * Name: stm32_flash_optmodify
 *
 * Description:
 *   Modifies the current flash option bytes, given bits to set and clear.
 *
 * Input Parameters:
 *   clear - clear bits for FLASH_OPTR
 *   set   - set bits for FLASH_OPTR
 *
 * Returned Value:
 *   Zero or error value
 *
 *     -EBUSY: Timeout waiting for previous FLASH operation to occur, or
 *             there was data in the flash data buffer.
 *
 * Notes:
 *   This function WILL BLOCK and NOT release the thread. This is a sensitive
 *   operation with the potential to brick the device if interrupted. So, for
 *   the actual opt modify start, this function uses a tight while loop to
 *   wait for completion.
 *
 ****************************************************************************/

int stm32_flash_optmodify(uint32_t clear, uint32_t set)
{
  int ret;
  bool was_locked;

  ret = flash_wait_for_operation();
  if (ret != 0)
    {
      return -EBUSY;
    }

  was_locked = flash_unlock_opt();
  modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_CLEAR_ERROR_FLAGS);

  modifyreg32(STM32_FLASH_OPTR, clear, set);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY1)
    {
    }

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_OPTSTRT);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY1)
    {
    }

  if (was_locked)
    {
      flash_lock_opt();
    }

  return 0;
}

#ifdef CONFIG_ARCH_HAVE_PROGMEM

/* up_progmem_x functions defined in nuttx/include/nuttx/progmem.h
 *
 * Notes on Implementation:
 *   - The driver implementations DO NOT enforce memory address boundaries.
 *     For processors with less than 2MB flash, the user is responsible for
 *     not writing to memory between banks.
 *
 */

size_t up_progmem_pagesize(size_t page)
{
  return FLASH_PAGE_SIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  struct stm32_flash_priv_s *priv;

  priv = flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }

  return priv->stpage + ((addr - priv->base) / FLASH_PAGE_SIZE);
}

size_t up_progmem_getaddress(size_t page)
{
  struct stm32_flash_priv_s *priv;

  if (page >= FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  priv = flash_bank(STM32_FLASH_BASE + (page * FLASH_PAGE_SIZE));

  if (!priv)
    {
      return SIZE_MAX;
    }

  return priv->base + (page - priv->stpage) * FLASH_PAGE_SIZE;
}

size_t up_progmem_neraseblocks(void)
{
  return FLASH_NBLOCKS;
}

bool up_progmem_isuniform(void)
{
  /* So... Every other implementation of this in STM chips returns this as
   * true. However, the description in include/nuttx/progmem.h states this to
   * mean "does size of erase 'page' == size of read/write 'page'". Which is
   * NOT true for most of these chips.
   *
   * On the G0, erase blocks are 2K and read/write page is 64 bit.
   */

  return false;
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
  return FLASH_BLOCK_SIZE;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  int ret;
  size_t block_address;

  ret = flash_verify_blocknum(block);
  if (ret < 0)
    {
      return -EFAULT;
    }

  block_address = flash_block_address(block);

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  if (flash_wait_for_operation())
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  /* Get flash ready and begin erasing single block */

  flash_unlock_cr();

  modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_CLEAR_ERROR_FLAGS);

  /* By now, know that the block number is valid and corresponds to a
   * bank (if dual bank). So, don't need to verify that it is in bounds.
   */

#ifdef FLASH_DUAL_BANK

  /* Note to future developers: The CR register definition in the reference
   * manual [RM0444] is not clear on if bank selection is necessary. The PNB
   * definition seems to imply that writing block numbers corresponding to
   * bank 2 should just work. This is NOT the case. Writing 256 to PNB will
   * cause block (page) 0 to be erased. Therefore, must switch BKER bit to
   * match the correct bank.
   */

  if (block >= flash_bank2_priv.stblock)
    {
      modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_BKER);
    }
  else
    {
      modifyreg32(STM32_FLASH_CR, FLASH_CR_BKER, 0);
    }
#endif

  /* Setup erase parameters and start */

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PNB_MASK,
              FLASH_CR_PER | (block << FLASH_CR_PNB_SHIFT));
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);

  /* Wait for erase operation to complete */

  if (flash_wait_for_operation())
    {
      ret = -EIO;
      goto exit_with_unlock;
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PNB_MASK | FLASH_CR_PER, 0);

  ret = 0;
  up_invalidate_dcache(block_address, block_address + FLASH_BLOCK_SIZE);

exit_with_unlock:
  flash_lock_cr();

exit_with_lock:
  nxmutex_unlock(&g_lock);

  if (ret == 0 &&
      flash_israngeerased(block_address, up_progmem_erasesize(block)) == 0)
    {
      ret = up_progmem_erasesize(block); /* Success */
    }
  else
    {
      ret = -EIO;
    }

  return ret;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  struct stm32_flash_priv_s *priv;
  uint32_t     *fp;
  uint32_t     *rp;
  uint32_t     *ll        = (uint32_t *)buf;
  size_t       faddr;
  size_t       written    = count;
  int          ret;
  const size_t pagesize   = up_progmem_pagesize(0); /* 64-bit, 8 bytes per page */
  const size_t llperpage  = pagesize / sizeof(uint32_t);
  size_t       pcount     = count / pagesize;

  priv = flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }

  /* Check for valid address range */

  if (addr < priv->base ||
      addr + count > priv->base + (FLASH_BANKSIZE))
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Check address and count alignment */

  DEBUGASSERT(!(addr % pagesize));
  DEBUGASSERT(!(count % pagesize));

  if (flash_wait_for_operation())
    {
      written = -EIO;
      goto exit_with_lock;
    }

  /* Get flash ready for write */

  flash_unlock_cr();

  modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_CLEAR_ERROR_FLAGS);
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

  /* Write */

  for (ll = (uint32_t *)buf, faddr = addr; pcount;
       pcount -= 1, ll += llperpage, faddr += pagesize)
    {
      fp = (uint32_t *)faddr;
      rp = ll;

      UP_MB();

      /* Write 2 32 bit word and wait to complete */

      *fp++ = *rp++;
      *fp++ = *rp++;

      /* Data synchronous Barrier (DSB) just after the write operation. This
       * will force the CPU to respect the sequence of instruction (no
       * optimization).
       */

      UP_MB();

      if (flash_wait_for_operation())
        {
          written = -EIO;
          goto exit_with_unlock;
        }

      /* Future improvements may add ECC checking here. */
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);

exit_with_unlock:
  flash_lock_cr();

  if (written > 0)
    {
      for (ll = (uint32_t *)buf, faddr = addr, pcount = count / pagesize;
           pcount; pcount -= 1, ll += llperpage, faddr += pagesize)
        {
          fp = (uint32_t *)faddr;
          rp = ll;

          modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_CLEAR_ERROR_FLAGS);

          if ((*fp++ != *rp++) ||
              (*fp++ != *rp++))
            {
              written = -EIO;
              break;
            }

          /* Future improvements may add ECC checking here. */
        }

      modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_CLEAR_ERROR_FLAGS);
    }

exit_with_lock:
  nxmutex_unlock(&g_lock);
  return written;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}

#endif /* CONFIG_ARCH_HAVE_PROGMEM*/