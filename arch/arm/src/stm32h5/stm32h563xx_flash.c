/****************************************************************************
 * arch/arm/src/stm32h5/stm32h563xx_flash.c
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
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 *
 * Notes:
 *   - RM0481 refers to erase blocks as "Sectors". This file uses block.
 *   - This file assumes dual bank flash memory.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <arch/barriers.h>

#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <syslog.h>
#include <sys/param.h>

#include "hardware/stm32_flash.h"
#include "hardware/stm32_memorymap.h"
#include "hardware/stm32_sbs.h"
#include "arm_internal.h"
#include "stm32_flash.h"
#include "stm32_icache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _K(x) ((x)*1024)
#define FLASH_BLOCK_SIZE _K(8)
#define FLASH_PAGE_SIZE     16

#if !defined(CONFIG_STM32_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32_FLASH_OVERRIDE_B) && \
    !defined(CONFIG_STM32_FLASH_OVERRIDE_C) && \
    !defined(CONFIG_STM32_FLASH_OVERRIDE_E) && \
    !defined(CONFIG_STM32_FLASH_OVERRIDE_G) && \
    !defined(CONFIG_STM32_FLASH_OVERRIDE_I) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_B) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_C) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_E) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_G) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_I)
#  define CONFIG_STM32_FLASH_OVERRIDE_E
#  warning "Flash size not defined defaulting to 512KiB (E)"
#endif

/* Override of the Flash has been chosen */

#if !defined(CONFIG_STM32_FLASH_OVERRIDE_DEFAULT)
#  undef CONFIG_STM32_FLASH_CONFIG_C
#  undef CONFIG_STM32_FLASH_CONFIG_E
#  if defined(CONFIG_STM32_FLASH_OVERRIDE_C)
#    define CONFIG_STM32_FLASH_CONFIG_C
#  elif defined(CONFIG_STM32_FLASH_OVERRIDE_E)
#    define CONFIG_STM32_FLASH_CONFIG_E
#  endif
#endif

#if defined(CONFIG_STM32_FLASH_CONFIG_I)
#  define H5_FLASH_BANK_NBLOCKS    128
#elif defined(CONFIG_STM32_FLASH_CONFIG_G)
#  define H5_FLASH_BANK_NBLOCKS    64
#elif defined(CONFIG_STM32_FLASH_CONFIG_E)
#  define H5_FLASH_BANK_NBLOCKS    32
#elif defined(CONFIG_STM32_FLASH_CONFIG_C)
#  define H5_FLASH_BANK_NBLOCKS    16
#elif defined(CONFIG_STM32_FLASH_CONFIG_B)
#  define H5_FLASH_BANK_NBLOCKS    8
#else
#  warning "No valid STM32_FLASH_CONFIG_x defined."
#endif

#define H5_FLASH_BANKSIZE   (FLASH_BLOCK_SIZE * H5_FLASH_BANK_NBLOCKS)
#define H5_FLASH_NBLOCKS    (2 * H5_FLASH_BANK_NBLOCKS)
#define H5_FLASH_TOTALSIZE  (2 * H5_FLASH_BANKSIZE)
#define H5_FLASH_NPAGES     (H5_FLASH_TOTALSIZE / FLASH_PAGE_SIZE)

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB
#define FLASH_OPTKEY1   0x08192A3B
#define FLASH_OPTKEY2   0x4C5D6E7F
#define FLASH_OBKKEY1   0x192A083B
#define FLASH_OBKKEY2   0x5E7F4C5D

#define FLASH_ERASEDVALUE     0xffu
#define FLASH_ERASEDVALUE_DW  0xffffffffu
#define FLASH_TIMEOUT_VALUE   5000000   /* 5s */

#define FLASH_OTP_SIZE              2048           /* OTP area size: 2048 bytes */
#define FLASH_OTP_BLOCK_SIZE        64             /* 32 words * 2 bytes = 64 bytes per block */
#define FLASH_OTP_TOTAL_BLOCKS      32             /* Total OTP blocks (0-31) */
#define FLASH_OTP_WORDS_PER_BLOCK   32             /* 32 words per block */
#define OTP_WORD_SIZE               2              /* 16-bit words as per manual */

#define OTP_ERASEDVALUE16    0xffffu
#define OTP_ERASEDVALUE32    0xffffffffu
#define OTP_ECCD             (FLASH_ECCDETR_ECCD | FLASH_ECCDETR_OTP_ECC)

#define FLASH_NSSR_ALL_ERRORS  (FLASH_NSSR_WRPERR | FLASH_NSSR_PGSERR |  \
                                FLASH_NSSR_STRBERR | FLASH_NSSR_INCERR | \
                                FLASH_NSSR_OBKERR | FLASH_NSSR_OBKWERR | \
                                FLASH_NSSR_OPTCHANGERR )

/* Flash high-cycle data (EDATA) */

#define EDATA_BANK_SIZE       (STM32_EDATA_BANK_NSECTORS * \
                               STM32_EDATA_SECTOR_SIZE)
#define EDATA_ERASEDVALUE     0xffffu
#define EDATA_ECCD            (FLASH_ECCDETR_ECCD | FLASH_ECCDETR_EDATA_ECC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32h5_flash_priv_s
{
  uint32_t base;    /* FLASH base address */
  uint32_t stblock; /* The first block number */
  uint32_t stpage;  /* The first page number */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32h5_flash_priv_s flash_bank1_priv =
{
  .base    = STM32_FLASH_BANK1,
  .stblock = 0,
  .stpage  = 0
};
static struct stm32h5_flash_priv_s flash_bank2_priv =
{
  .base    = STM32_FLASH_BANK2,
  .stblock = (H5_FLASH_NBLOCKS / 2),
  .stpage  = (H5_FLASH_NPAGES / 2),
};

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flash_bank
 *
 * Description:
 *    Returns the priv pointer to the correct bank
 *
 ****************************************************************************/

static inline
struct stm32h5_flash_priv_s * flash_bank(size_t address)
{
  struct stm32h5_flash_priv_s *priv = NULL;

  if (address >= flash_bank1_priv.base &&
      address < flash_bank1_priv.base + H5_FLASH_BANKSIZE)
    {
      priv = &flash_bank1_priv;
    }
  else if (address >= flash_bank2_priv.base &&
           address < flash_bank2_priv.base + H5_FLASH_BANKSIZE)
    {
      priv = &flash_bank2_priv;
    }

  return priv;
}

/****************************************************************************
 * Name: flash_icache_suspend
 *
 * Description:
 *   Disable the ICACHE before flash is modified (RM0481 8.4.5).  This also
 *   invalidates it.  Must be called with g_lock held.
 *
 * Returned Value:
 *   true if the ICACHE was enabled and must be restored with
 *   flash_icache_resume(); false otherwise.
 *
 ****************************************************************************/

static bool flash_icache_suspend(void)
{
#ifdef CONFIG_STM32_ICACHE
  if (stm32_icache_enabled())
    {
      stm32_disable_icache();
      return true;
    }
#endif

  return false;
}

/****************************************************************************
 * Name: flash_icache_resume
 *
 * Description:
 *   Enable the ICACHE again if flash_icache_suspend() disabled it.  Must be
 *   called with g_lock held.
 *
 * Input Parameters:
 *   enabled - The value returned by flash_icache_suspend()
 *
 ****************************************************************************/

static void flash_icache_resume(bool enabled)
{
#ifdef CONFIG_STM32_ICACHE
  if (enabled && stm32_enable_icache() != OK)
    {
      /* The ICACHE stays off, which is safe but slower */

      syslog(LOG_ERR, "ICACHE invalidate timed out, ICACHE left off\n");
    }
#else
  UNUSED(enabled);
#endif
}

/****************************************************************************
 * Name: flash_unlock_nscr
 *
 * Description:
 *    Unlock the non-secure control register.
 *
 ****************************************************************************/

static void flash_unlock_nscr(void)
{
  while (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_BSY)
    {
    }

  if (getreg32(STM32_FLASH_NSCR) & FLASH_NSCR_LOCK)
    {
      putreg32(FLASH_KEY1, STM32_FLASH_NSKEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_NSKEYR);
    }
}

/****************************************************************************
 * Name: flash_lock_nscr
 *
 * Description:
 *    Lock the non-secure control register.
 *
 ****************************************************************************/

static void flash_lock_nscr(void)
{
  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_LOCK);
}

/****************************************************************************
 * Name: stm32h5_israngeerased
 *
 * Description:
 *   Returns count of non-erased words
 *
 ****************************************************************************/

static int stm32h5_israngeerased(size_t startaddress, size_t size)
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

  for (i = 0; i < FLASH_TIMEOUT_VALUE; i++)
    {
      if (!(getreg32(STM32_FLASH_NSSR) &
          (FLASH_NSSR_BSY | FLASH_NSSR_DBNE | FLASH_NSSR_WBNE)))
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
 * Name: flash_unlock_opt
 *
 * Description:
 *   Unlock the flash option bytes
 *
 ****************************************************************************/

static bool flash_unlock_opt(void)
{
  bool was_locked = false;

  while (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_BSY)
    {
    }

  if (getreg32(STM32_FLASH_OPTCR) & FLASH_OPTCR_OPTLOCK)
    {
      was_locked = true;

      putreg32(FLASH_OPTKEY1, STM32_FLASH_OPTKEYR);
      putreg32(FLASH_OPTKEY2, STM32_FLASH_OPTKEYR);
    }

  return was_locked;
}

/****************************************************************************
 * Name: flash_lock_opt
 *
 * Description:
 *   Lock the flash option bytes
 *
 ****************************************************************************/

static void flash_lock_opt(void)
{
  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTLOCK);
}

#if defined(CONFIG_STM32_EDATA) || defined(CONFIG_STM32H5_OTP_WORD)

/****************************************************************************
 * Name: flash_read_eccsafe16
 *
 * Description:
 *   Read one 16-bit half-word of EDATA or OTP.  Both areas are mapped
 *   non-cacheable by the ICACHE driver, so the read bypasses the ICACHE.
 *   Reading a blank (erased, never programmed) half-word raises the Flash
 *   ECC NMI, which is masked in the SBS first and handled instead by
 *   checking ECCDETR afterwards.  The mask is restored before returning.
 *
 * Input Parameters:
 *   addr    - Address of the half-word
 *   eccd    - ECCDETR bits that flag an ECC error for this memory
 *             (EDATA_ECCD or OTP_ECCD)
 *   eccerr  - Set to true if ECCDETR flagged this read, whether because
 *             the half-word was blank or genuinely corrupt.  The returned
 *             value in that case comes from ECCDR, not from the bus, since
 *             a flagged read's data is not to be trusted.
 *
 ****************************************************************************/

static uint16_t flash_read_eccsafe16(uintptr_t addr, uint32_t eccd,
                                     bool *eccerr)
{
  irqstate_t flags;
  uint16_t   value;
  uint32_t   eccnmir;

  flags = up_irq_save();

  eccnmir = getreg32(STM32_SBS_ECCNMIR);
  putreg32(eccnmir | SBS_ECCNMIR_ECCNMI_MASK_EN, STM32_SBS_ECCNMIR);

  value = getreg16(addr);
  UP_DSB();

  *eccerr = (getreg32(STM32_FLASH_ECCDETR) & eccd) == eccd;
  if (*eccerr)
    {
      value = getreg32(STM32_FLASH_ECCDR) & FLASH_ECCDR_DATA_ECC_MASK;
      putreg32(FLASH_ECCDETR_ECCD, STM32_FLASH_ECCDETR);
    }

  putreg32(eccnmir, STM32_SBS_ECCNMIR);

  up_irq_restore(flags);

  return value;
}

#endif /* CONFIG_STM32_EDATA || CONFIG_STM32H5_OTP_WORD */

#ifdef CONFIG_STM32_EDATA

/****************************************************************************
 * Name: edata_logical_bank
 *
 * Description:
 *   Returns the logical bank (1 or 2) a physical bank is currently mapped
 *   to.  The swap only takes effect at reset, so this is only valid until
 *   the SWAP_BANK option is next changed.
 *
 ****************************************************************************/

static int edata_logical_bank(int bank)
{
  if (getreg32(STM32_FLASH_OPTSR_CUR) & FLASH_OPTSR_CUR_SWAP_BANK)
    {
      return 3 - bank;
    }

  return bank;
}

/****************************************************************************
 * Name: edata_read_hword
 *
 * Description:
 *   Read one EDATA half-word.  A blank (erased, never programmed) half-word
 *   reads as 0xffff.  If the half-word is corrupt, for example because
 *   power was lost while it was being programmed, the raw data is returned.
 *
 ****************************************************************************/

static uint16_t edata_read_hword(uintptr_t addr)
{
  bool eccerr;
  uint16_t value = flash_read_eccsafe16(addr, EDATA_ECCD, &eccerr);

  if (eccerr && value != EDATA_ERASEDVALUE)
    {
      ferr("ERROR: EDATA ECC error at %08" PRIxPTR ": %04x\n", addr, value);
    }

  return value;
}

/****************************************************************************
 * Name: edata_erase
 *
 * Description:
 *   Erase one EDATA sector.  Must be called with g_lock held.
 *
 ****************************************************************************/

static int edata_erase(int bank, unsigned int sector)
{
  uint32_t snb = H5_FLASH_BANK_NBLOCKS - STM32_EDATA_BANK_NSECTORS + sector;
  int ret = OK;

  if (flash_wait_for_operation())
    {
      return -EIO;
    }

  flash_unlock_nscr();
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);

  /* BKSEL selects the physical bank */

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_BKSEL | FLASH_NSCR_SNB_MASK,
              FLASH_NSCR_SER | FLASH_NSCR_SNB(snb) |
              (bank == 2 ? FLASH_NSCR_BKSEL : 0));
  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_STRT);

  if (flash_wait_for_operation() ||
      (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_ALL_ERRORS))
    {
      ret = -EIO;
    }

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_SER | FLASH_NSCR_SNB_MASK |
              FLASH_NSCR_BKSEL, 0);
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);
  flash_lock_nscr();

  return ret;
}

#endif /* CONFIG_STM32_EDATA */

#ifdef CONFIG_STM32H5_OTP_WORD

/****************************************************************************
 * Name: otp_read_eccsafe16
 *
 * Description:
 *   Read one 16-bit OTP word; see flash_read_eccsafe16().
 *
 ****************************************************************************/

static uint16_t otp_read_eccsafe16(uintptr_t addr, bool *eccerr)
{
  return flash_read_eccsafe16(addr, OTP_ECCD, eccerr);
}

/****************************************************************************
 * Name: otp_read_eccsafe32
 *
 * Description:
 *   32-bit counterpart of otp_read_eccsafe16().  ECC is computed per
 *   16-bit word (FLASH_ECCDR only ever holds 16 bits of recovered data),
 *   so a 32-bit read is done as its two halves, each independently
 *   recovered: a single native 32-bit access could only ever recover
 *   whichever half ECCDETR last reported and would have to discard the
 *   other half's real contents.
 *
 ****************************************************************************/

static uint32_t otp_read_eccsafe32(uintptr_t addr, FAR bool *eccerr)
{
  bool erclo;
  bool erchi;
  uint16_t lo = otp_read_eccsafe16(addr, &erclo);
  uint16_t hi = otp_read_eccsafe16(addr + sizeof(uint16_t), &erchi);

  *eccerr = erclo || erchi;
  return (uint32_t)lo | ((uint32_t)hi << 16);
}

#endif /* CONFIG_STM32H5_OTP_WORD */

/****************************************************************************
 * Name: stm32h5_otp_is_space_available
 *
 * Description:
 *   Validates if the memory region can be written. OTP memory may only be
 *   written once, and after writing data to a specific block - this block
 *   should be locked for writing to prevent potential overwrite attempt.
 *   Even if the 64-bytes block was only used partially, it should be locked
 *   because there is no mechanism to validate whether the memory was written
 *   or not, so the entire block is locked after write even a single bit
 *
 * Returned Value:
 *   True if there is enough consecutive bytes in OTP to store the data
 *   False otherwise
 *
 ****************************************************************************/

static bool stm32h5_otp_is_space_available(uint8_t start_block,
    uint8_t end_block)
{
  uint32_t lockbl_cur = getreg32(STM32_FLASH_OTBPBLR_CUR);

  for (uint8_t i = start_block; i <= end_block; i++)
    {
      if (lockbl_cur & (1 << i))
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: stm32h5_otp_clear_errors
 *
 * Description:
 *   Clear all OTP error flags from previous operations
 *
 * Returned Value:
 *   Zero on success or negative error value
 *
 ****************************************************************************/

static int stm32h5_otp_clear_errors(void)
{
  uint32_t error_flags = getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_ALL_ERRORS;

  UP_DSB();

  if (error_flags != 0)
    {
      putreg32(error_flags, STM32_FLASH_NSCCR);

      error_flags = getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_ALL_ERRORS;
      if (error_flags != 0)
        {
          return -EAGAIN;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32h5_otp_lock_block
 *
 * Description:
 *   Lock the OTP block to prevent further data changes
 *
 * Input Parameters:
 *   block_number - the number of block to lock
 *
 * Returned Value:
 *   Zero on success or negative error value
 *
 ****************************************************************************/

static int stm32h5_otp_lock_block(uint8_t block_number)
{
  int ret;
  uint32_t reg;
  bool was_locked;
  uint32_t lockbl_cur;

  if (block_number >= FLASH_OTP_TOTAL_BLOCKS)
    {
      return -EINVAL;
    }

  lockbl_cur = getreg32(STM32_FLASH_OTBPBLR_CUR);
  if (lockbl_cur & (1 << block_number))
    {
      /* Block is already locked */

      return -EACCES;
    }

  /* Wait for any ongoing flash operations */

  ret = flash_wait_for_operation();
  if (ret != 0)
    {
      return -EBUSY;
    }

  /* Check that data buffer is empty */

  reg = getreg32(STM32_FLASH_NSSR);
  if (reg & FLASH_NSSR_DBNE)
    {
      return -EBUSY;
    }

  /* Unlock option bytes for programming */

  was_locked = flash_unlock_opt();

  /* Set the bit in the OTP block lock programming register */

  modifyreg32(STM32_FLASH_OTBPBLR_PRG, 0, (1 << block_number));

  /* Start the option bytes programming sequence */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  /* Wait for programming operation to complete */

  while (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_BSY)
    {
    }

  /* Check for programming errors */

  reg = getreg32(STM32_FLASH_NSSR);
  if (reg & FLASH_NSSR_ALL_ERRORS)
    {
      /* Clear errors and return failure */

      putreg32(reg & FLASH_NSSR_ALL_ERRORS, STM32_FLASH_NSCCR);
      ret = -EIO;
    }
  else
    {
      /* Verify the lock was applied */

      lockbl_cur = getreg32(STM32_FLASH_OTBPBLR_CUR);
      if (!(lockbl_cur & (1 << block_number)))
        {
          ret = -EIO;
        }
      else
        {
          ret = OK;
        }
    }

  /* Re-lock option bytes if they were locked before */

  if (was_locked)
    {
      flash_lock_opt();
    }

  return OK;
}

/****************************************************************************
 * Name: stm32h5_otp_write_word
 *
 * Description:
 *   Write OTP word (16 bits total) following the manual sequence
 *   Follows steps 1-7 from the STM32H5 reference manual. Locking
 *   written block (as step 8) is done after all data is written
 *
 * Input Parameters:
 *   otp_address - OTP address (must be 4-byte aligned)
 *   data        - 16-bit data (one 16-bit words)
 *
 * Returned Value:
 *   Zero on success or negative error value
 *
 ****************************************************************************/

static int stm32h5_otp_write_word(uint32_t otp_address, const uint16_t *data)
{
  volatile uint16_t *otp_addr = (volatile uint16_t *)otp_address;
  int ret;

  /* Step 1: Check that no memory operations are ongoing */

  ret = flash_wait_for_operation();
  if (ret != OK)
    {
      return ret;
    }

  /* Verify data buffer is empty (DBNE bit) */

  if (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_DBNE)
    {
      return -EBUSY;
    }

  /* Step 2: Check and clear all error flags */

  ret = stm32h5_otp_clear_errors();
  if (ret != OK)
    {
      return ret;
    }

  /* Step 3: Set PG bit in FLASH_NSCR register */

  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_PG);

  UP_DSB();
  UP_ISB();

  /* Step 5: Write OTP word (16 bits total) */

  *otp_addr = *data;

  UP_DSB();
  UP_ISB();

  /* Step 6: Wait for BSY bit to be cleared */

  ret = flash_wait_for_operation();
  if (ret != OK)
    {
      modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);
      return ret;
    }

  /* Step 7: Clear PG bit */

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);

  /* Verify the write by reading back */

  if (*otp_addr != *data)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_flash_unlock
 *
 * Description:
 *   Unlock non-secure flash control
 *
 ****************************************************************************/

void stm32_flash_unlock(void)
{
  nxmutex_lock(&g_lock);
  flash_unlock_nscr();
  nxmutex_unlock(&g_lock);
}

/****************************************************************************
 * Name: stm32_flash_lock
 *
 * Description:
 *   Lock non-secure flash control
 *
 ****************************************************************************/

void stm32_flash_lock(void)
{
  nxmutex_lock(&g_lock);
  flash_lock_nscr();
  nxmutex_unlock(&g_lock);
}

/****************************************************************************
 * Name: stm32_flash_getopt
 *
 * Description:
 *   Read the current flash option bytes from FLASH_OPTSR_CUR and
 *   FLASH_OPTSR2_CUR registers.
 *
 * Input Parameters:
 *   opt1 - result from FLASH_OPTSR_CUR
 *   opt2 - result from FLASH_OPTSR2_CUR
 *
 ****************************************************************************/

void stm32_flash_getopt(uint32_t *opt1, uint32_t *opt2)
{
  *opt1 = getreg32(STM32_FLASH_OPTSR_CUR);
  *opt2 = getreg32(STM32_FLASH_OPTSR2_CUR);
}

/****************************************************************************
 * Name: stm32_flash_optmodify
 *
 * Description:
 *   Modifies the current flash option bytes, given bits to set and clear.
 *
 * Input Parameters:
 *   clear1 - clear bits for FLASH_OPTSR
 *   set1   - set bits for FLASH_OPTSR
 *   clear2 - clear bits for FLASH_OPTSR2
 *   set2   - set bits for FLASH_OPTSR2
 *
 * Returned Value:
 *   Zero or error value
 *
 *     -EBUSY: Timeout waiting for previous FLASH operation to occur, or
 *             there was data in the flash data buffer.
 *
 ****************************************************************************/

int stm32_flash_optmodify(uint32_t clear1, uint32_t set1,
                          uint32_t clear2, uint32_t set2)
{
  int ret;
  uint32_t reg;
  bool was_locked;

  ret = flash_wait_for_operation();
  if (ret != 0)
    {
      return -EBUSY;
    }

  reg = getreg32(STM32_FLASH_NSSR);
  if (reg & FLASH_NSSR_DBNE)
    {
      return -EBUSY;
    }

  was_locked = flash_unlock_opt();

  modifyreg32(STM32_FLASH_OPTSR_PRG, clear1, set1);
  modifyreg32(STM32_FLASH_OPTSR2_PRG, clear2, set2);

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  while (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_BSY)
    {
    }

  if (was_locked)
    {
      flash_lock_opt();
    }

  return 0;
}

/****************************************************************************
 * Name: stm32_flash_swapbanks
 *
 * Description:
 *   Swaps banks 1 and 2 in the processor's memory map.  Takes effect
 *   the next time the system is reset.
 *
 * Returned Value:
 *      Zero or error value
 *
 *      -ETIMEDOUT: Timeout occurred waiting for previous operation to occur.
 *
 ****************************************************************************/

int stm32_flash_swapbanks(void)
{
  uint32_t reg;
  bool was_locked;

  if (flash_wait_for_operation())
    {
      return -ETIMEDOUT;
    }

  was_locked = flash_unlock_opt();

  reg = getreg32(STM32_FLASH_OPTSR_PRG);
  reg ^= FLASH_OPTSR_PRG_SWAP_BANK;
  putreg32(reg, STM32_FLASH_OPTSR_PRG);

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  while ((getreg32(STM32_FLASH_OPTSR_CUR) >> 31) != (reg >> 31))
    {
    }

  if (was_locked)
    {
      flash_lock_opt();
    }

  return 0;
}

/****************************************************************************
 * Name: stm32_otp_write
 *
 * Description:
 *   Writes data to OTP section starting from the offset.
 *   The involved blocks will be locked afterward.
 *
 * Input Parameters:
 *   data   - Pointer to data buffer
 *   len    - Length in bytes of data to write
 *   offset - 4-aligned offset in bytes within OTP area
 *            (0 to FLASH_OTP_SIZE-4)
 *
 * Returned Value:
 *   Zero on success or negative error value
 *
 ****************************************************************************/

int stm32_otp_write(const uint16_t *data, uint16_t len, uint32_t offset)
{
  uint32_t otp_address;
  uint16_t remaining_bytes;
  int ret;
  uint16_t i;
  uint8_t start_block;
  uint8_t end_block;
  uint16_t words_to_write;

  if (data == NULL || len == 0)
    {
      return -EINVAL;
    }

  if (offset + len > FLASH_OTP_SIZE)
    {
      return -ENOMEM;
    }

  /* Ensure 4-byte alignment for writing */

  if (offset % 4 != 0)
    {
      return -EINVAL;
    }

  start_block = offset / FLASH_OTP_BLOCK_SIZE;
  end_block = (offset + len - 1) / FLASH_OTP_BLOCK_SIZE;

  /* Calculate actual OTP address */

  otp_address = STM32_OTP_BASE + offset;

  /* Calculate number of complete 16-bit words */

  words_to_write = len / OTP_WORD_SIZE;
  remaining_bytes = len % OTP_WORD_SIZE;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!stm32h5_otp_is_space_available(start_block, end_block))
    {
      nxmutex_unlock(&g_lock);
      return -EACCES;
    }

  /* Unlock flash for programming */

  flash_unlock_nscr();

  /* Write complete 16-bit pairs (two 16-bit words each) */

  for (i = 0; i < words_to_write; i++)
    {
      ret = stm32h5_otp_write_word(otp_address + (i * OTP_WORD_SIZE),
          data + i);

      if (ret != OK)
        {
          goto exit_with_unlock;
        }
    }

  /* Handle remaining bytes (less than OTP_WORD_SIZE bytes) */

  if (remaining_bytes > 0)
    {
      uint16_t write_word = 0xffff; /* Default erased value for unused bits */

      /* Fill the remaining bytes */

      memcpy(&write_word, data + words_to_write, remaining_bytes);

      ret = stm32h5_otp_write_word(
          otp_address + (words_to_write * OTP_WORD_SIZE), &write_word);
      if (ret != OK)
        {
          goto exit_with_unlock;
        }
    }

  for (i = start_block; i <= end_block; i++)
    {
      ret = stm32h5_otp_lock_block(i);
      if (ret != OK)
        {
          break;
        }
    }

exit_with_unlock:
  flash_lock_nscr();
  nxmutex_unlock(&g_lock);

  return ret;
}

/****************************************************************************
 * Name: stm32_otp_read
 *
 * Description:
 *   Reads data from OTP section starting from the offset
 *
 * Input Parameters:
 *   data   - Pointer to data buffer to store read data.
 *   len    - Length in bytes of data to read
 *   offset - 4-aligned offset in bytes within OTP area
 *            (0 to FLASH_OTP_SIZE-4)
 *
 * Returned Value:
 *   Zero on success or negative error value
 *
 ****************************************************************************/

int stm32_otp_read(uint16_t *data, uint16_t len, uint32_t offset)
{
  uint32_t otp_address;
  uint16_t remaining_bytes;
  int ret;
  uint16_t i;
  uint16_t words_to_read;

  if (data == NULL || len == 0)
    {
      return -EINVAL;
    }

  if (offset + len > FLASH_OTP_SIZE)
    {
      return -ENOMEM;
    }

  /* Ensure 4-byte alignment for OTP reading */

  if (offset % 4 != 0)
    {
      return -EINVAL;
    }

  /* Calculate actual OTP address */

  otp_address = STM32_OTP_BASE + offset;

  /* Calculate number of complete 16-bit words */

  words_to_read = len / OTP_WORD_SIZE;
  remaining_bytes = len % OTP_WORD_SIZE;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for any ongoing operations to complete */

  ret = flash_wait_for_operation();
  if (ret != OK)
    {
      nxmutex_unlock(&g_lock);
      return ret;
    }

  /* Read complete 16-bit words */

  for (i = 0; i < words_to_read; i++)
    {
      volatile uint16_t *otp_addr = (volatile uint16_t *)(otp_address +
          (i * OTP_WORD_SIZE));
      uint16_t *dest = data + i;

      *dest = *otp_addr;
    }

  /* Handle remaining bytes (less than OTP_WORD_SIZE bytes) */

  if (remaining_bytes > 0)
    {
      volatile uint16_t *otp_addr = (volatile uint16_t *)(otp_address +
          (words_to_read * OTP_WORD_SIZE));
      uint16_t read_data = *otp_addr;

      /* Copy only the needed bytes */

      memcpy(data + words_to_read, &read_data,
          remaining_bytes);
    }

  nxmutex_unlock(&g_lock);
  return OK;
}

/****************************************************************************
 * Name: stm32_otp_getlockstatus
 *
 * Description:
 *   Get the lock status of all OTP blocks
 *
 * Returned Value:
 *   32-bit value representing lock status of blocks 0-31
 *
 ****************************************************************************/

uint32_t stm32_otp_getlockstatus(void)
{
  return getreg32(STM32_FLASH_OTBPBLR_CUR);
}

#ifdef CONFIG_STM32_EDATA

/****************************************************************************
 * Name: stm32_flash_edata_getconfig
 *
 * Description:
 *   Returns the number of sectors of a physical bank (1 or 2) that are
 *   currently configured as EDATA, 0 if EDATA is disabled in that bank, or
 *   a negated errno value.
 *
 ****************************************************************************/

int stm32_flash_edata_getconfig(int bank)
{
  uint32_t regval;

  if (bank == 1)
    {
      regval = getreg32(STM32_FLASH_EDATA1R_CUR);
    }
  else if (bank == 2)
    {
      regval = getreg32(STM32_FLASH_EDATA2R_CUR);
    }
  else
    {
      return -EINVAL;
    }

  /* The EDATA1R and EDATA2R fields are laid out identically */

  if (!(regval & FLASH_EDATA1R_CUR_EDATA1_EN))
    {
      return 0;
    }

  return ((regval & FLASH_EDATA1R_CUR_EDATA1_STRT_MASK) >>
          FLASH_EDATA1R_CUR_EDATA1_STRT_SHIFT) + 1;
}

/****************************************************************************
 * Name: stm32_flash_edata_configure
 *
 * Description:
 *   Program the option bytes so that the last nsectors (0..8) sectors of a
 *   physical bank (1 or 2) are EDATA.  Sectors that change between user
 *   flash and EDATA are erased.  Nothing is done if the bank is already
 *   configured that way.
 *
 *   This refuses to convert sectors holding the running image, but the
 *   other bank is not checked.
 *
 * Returned Value:
 *   Zero or a negated errno value:
 *
 *     -EINVAL: Invalid bank or sector count
 *     -EBUSY:  The sectors hold the running image
 *     -EIO:    Programming the option bytes or erasing failed
 *
 ****************************************************************************/

int stm32_flash_edata_configure(int bank, unsigned int nsectors)
{
  uintptr_t cur;
  uintptr_t prg;
  uintptr_t addr;
  uint32_t  regval;
  unsigned int oldsectors;
  unsigned int sector;
  bool was_locked;
  int ret;

  if ((bank != 1 && bank != 2) || nsectors > STM32_EDATA_BANK_NSECTORS)
    {
      return -EINVAL;
    }

  ret = stm32_flash_edata_getconfig(bank);
  if (ret < 0)
    {
      return ret;
    }

  oldsectors = ret;
  if (oldsectors == nsectors)
    {
      return OK;
    }

  /* Make sure the running image is not in any sector that changes type */

  addr = STM32_FLASH_BASE +
         (edata_logical_bank(bank) - 1) * H5_FLASH_BANKSIZE +
         (H5_FLASH_BANK_NBLOCKS - MAX(oldsectors, nsectors)) *
         FLASH_BLOCK_SIZE;

  if (addr < (uintptr_t)_eronly + (uintptr_t)(_edata - _sdata))
    {
      ferr("ERROR: EDATA sectors overlap the running image\n");
      return -EBUSY;
    }

  if (bank == 1)
    {
      cur = STM32_FLASH_EDATA1R_CUR;
      prg = STM32_FLASH_EDATA1R_PRG;
    }
  else
    {
      cur = STM32_FLASH_EDATA2R_CUR;
      prg = STM32_FLASH_EDATA2R_PRG;
    }

  regval = 0;
  if (nsectors > 0)
    {
      regval = FLASH_EDATA1R_PRG_EDATA1_EN |
               FLASH_EDATA1R_PRG_EDATA1_STRT(nsectors);
    }

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (flash_wait_for_operation())
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  was_locked = flash_unlock_opt();

  modifyreg32(prg, FLASH_EDATA1R_PRG_EDATA1_EN |
              FLASH_EDATA1R_PRG_EDATA1_STRT_MASK, regval);
  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  if (flash_wait_for_operation())
    {
      ret = -EIO;
    }

  if (was_locked)
    {
      flash_lock_opt();
    }

  if (ret == OK &&
      (getreg32(cur) & (FLASH_EDATA1R_CUR_EDATA1_EN |
                        FLASH_EDATA1R_CUR_EDATA1_STRT_MASK)) != regval)
    {
      ferr("ERROR: EDATA%dR option bytes not updated\n", bank);
      ret = -EIO;
    }

  /* Erase the sectors that changed type.  Their contents are unreadable
   * with the other ECC layout.
   */

  for (sector = STM32_EDATA_BANK_NSECTORS - MAX(oldsectors, nsectors);
       ret == OK &&
       sector < STM32_EDATA_BANK_NSECTORS - MIN(oldsectors, nsectors);
       sector++)
    {
      ret = edata_erase(bank, sector);
    }

exit_with_lock:
  nxmutex_unlock(&g_lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_flash_edata_address
 *
 * Description:
 *   Returns the address of an EDATA sector (0..7) of a physical bank (1 or
 *   2), or 0 if the arguments are invalid.  The sector must be enabled with
 *   stm32_flash_edata_configure() before it is accessed.
 *
 ****************************************************************************/

uintptr_t stm32_flash_edata_address(int bank, unsigned int sector)
{
  if ((bank != 1 && bank != 2) || sector >= STM32_EDATA_BANK_NSECTORS)
    {
      return 0;
    }

  return STM32_EDATA_BASE +
         (edata_logical_bank(bank) - 1) * EDATA_BANK_SIZE +
         sector * STM32_EDATA_SECTOR_SIZE;
}

/****************************************************************************
 * Name: stm32_flash_edata_erase
 *
 * Description:
 *   Erase an EDATA sector (0..7) of a physical bank (1 or 2).
 *
 ****************************************************************************/

int stm32_flash_edata_erase(int bank, unsigned int sector)
{
  int ret;

  if ((bank != 1 && bank != 2) || sector >= STM32_EDATA_BANK_NSECTORS)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = edata_erase(bank, sector);

  nxmutex_unlock(&g_lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_flash_edata_read
 *
 * Description:
 *   Read from EDATA.  The address and count must be half-word aligned.
 *   Blank half-words read as 0xffff.
 *
 * Returned Value:
 *   The number of bytes read or a negated errno value.
 *
 ****************************************************************************/

ssize_t stm32_flash_edata_read(uintptr_t addr, void *buf, size_t count)
{
  uint8_t *dest = buf;
  uint16_t value;
  size_t   i;
  int      ret;

  if ((addr | count) & 1)
    {
      return -EINVAL;
    }

  if (addr < STM32_EDATA_BASE ||
      addr + count > STM32_EDATA_BASE + 2 * EDATA_BANK_SIZE)
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < count; i += sizeof(value))
    {
      value = edata_read_hword(addr + i);
      memcpy(dest + i, &value, sizeof(value));
    }

  nxmutex_unlock(&g_lock);
  return count;
}

/****************************************************************************
 * Name: stm32_flash_edata_write
 *
 * Description:
 *   Program EDATA.  The address and count must be half-word aligned.
 *
 *   Each half-word can only be programmed once after an erase.  Half-words
 *   that already hold the requested value are skipped, so writing 0xffff
 *   leaves a blank half-word blank.  Programming a half-word that holds a
 *   different value fails with -EIO.
 *
 * Returned Value:
 *   The number of bytes written or a negated errno value.
 *
 ****************************************************************************/

ssize_t stm32_flash_edata_write(uintptr_t addr, const void *buf,
                                size_t count)
{
  const uint8_t *src = buf;
  uint16_t value;
  uint16_t current;
  size_t   i;
  int      ret;

  if ((addr | count) & 1)
    {
      return -EINVAL;
    }

  if (addr < STM32_EDATA_BASE ||
      addr + count > STM32_EDATA_BASE + 2 * EDATA_BANK_SIZE)
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (flash_wait_for_operation())
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  flash_unlock_nscr();
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);

  for (i = 0; i < count; i += sizeof(value))
    {
      memcpy(&value, src + i, sizeof(value));

      current = edata_read_hword(addr + i);
      if (current == value)
        {
          continue;
        }

      if (current != EDATA_ERASEDVALUE)
        {
          ret = -EIO;
          break;
        }

      /* EDATA is programmed one half-word at a time */

      modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_PG);
      UP_MB();

      putreg16(value, addr + i);
      UP_MB();

      if (flash_wait_for_operation() ||
          (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_ALL_ERRORS))
        {
          ret = -EIO;
        }

      modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);

      if (ret == OK && edata_read_hword(addr + i) != value)
        {
          ret = -EIO;
        }

      if (ret < 0)
        {
          break;
        }
    }

  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);
  flash_lock_nscr();

exit_with_lock:
  nxmutex_unlock(&g_lock);
  return ret < 0 ? ret : count;
}

#endif /* CONFIG_STM32_EDATA */

#ifdef CONFIG_STM32H5_OTP_WORD

/****************************************************************************
 * Name: stm32_otp_word_read16
 *
 * Description:
 *   Read one 16-bit OTP word.  This is independent of, and does not
 *   interact with, the block-oriented stm32_otp_write()/stm32_otp_read()
 *   API above: no locking is involved or required, since reading never
 *   conflicts with anything.
 *
 *   A blank (never programmed) word reads back as 0xffff.  Since whether a
 *   word has been written is known (that is exactly what trips its ECC),
 *   that is reported through the return value as -ENODATA rather than as
 *   OK, even though *value is filled in either way.  -EIO is reserved for
 *   a word whose ECC genuinely does not check out: neither blank nor the
 *   value its own program operation wrote.
 *
 * Input Parameters:
 *   word  - 16-bit word index, 0 to (FLASH_OTP_SIZE / 2) - 1
 *   value - Receives the word's contents
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value otherwise.  *value is set
 *   in every case except -EINVAL:
 *
 *     -EINVAL:  value is NULL, or word is out of range
 *     -ENODATA: The word has never been programmed; *value is 0xffff
 *     -EIO:     The word's ECC does not check out
 *
 ****************************************************************************/

int stm32_otp_word_read16(uint32_t word, FAR uint16_t *value)
{
  bool eccerr;
  uint16_t raw;

  if (value == NULL || word >= FLASH_OTP_SIZE / OTP_WORD_SIZE)
    {
      return -EINVAL;
    }

  raw = otp_read_eccsafe16(STM32_OTP_BASE + word * OTP_WORD_SIZE, &eccerr);
  *value = raw;

  if (eccerr)
    {
      if (raw != OTP_ERASEDVALUE16)
        {
          ferr("ERROR: OTP word %" PRIu32 " ECC error: %04x\n", word, raw);
          return -EIO;
        }

      return -ENODATA;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_otp_word_read32
 *
 * Description:
 *   32-bit counterpart of stm32_otp_word_read16().  Note that "word" here
 *   is a 32-bit word index: it does not line up with the index used by
 *   the 16-bit functions, the same as the attached reference driver this
 *   was ported from.
 *
 ****************************************************************************/

int stm32_otp_word_read32(uint32_t word, FAR uint32_t *value)
{
  bool eccerr;
  uint32_t raw;

  if (value == NULL || word >= FLASH_OTP_SIZE / sizeof(uint32_t))
    {
      return -EINVAL;
    }

  raw = otp_read_eccsafe32(STM32_OTP_BASE + word * sizeof(uint32_t),
                           &eccerr);
  *value = raw;

  if (eccerr)
    {
      if (raw != OTP_ERASEDVALUE32)
        {
          ferr("ERROR: OTP word %" PRIu32 " ECC error: %08" PRIx32 "\n",
               word, raw);
          return -EIO;
        }

      return -ENODATA;
    }

  return OK;
}

#ifdef CONFIG_STM32H5_OTP_WRITE

/****************************************************************************
 * Name: stm32_otp_word_write16
 *
 * Description:
 *   Program one 16-bit OTP word.  Programming is IRREVERSIBLE: a word
 *   that already holds a value other than the one requested cannot be
 *   reprogrammed, because bits can only move from 1 to 0 and ECC was
 *   already computed over its current contents.  Writing a word that
 *   already holds exactly the requested value is a harmless no-op that
 *   returns success, so this is safe to call unconditionally for a value
 *   that may or may not have been written before.
 *
 * Returned Value:
 *   Zero (OK) on success (including the no-op case above); a negated
 *   errno value on failure:
 *
 *     -EINVAL: word is out of range
 *     -EEXIST: The word already holds a different value
 *     -EIO:    Programming failed, or the post-write readback did not
 *              match
 *
 ****************************************************************************/

int stm32_otp_word_write16(uint32_t word, uint16_t value)
{
  uintptr_t addr;
  uint16_t  current;
  bool      eccerr;
  int       ret;

  if (word >= FLASH_OTP_SIZE / OTP_WORD_SIZE)
    {
      return -EINVAL;
    }

  addr = STM32_OTP_BASE + word * OTP_WORD_SIZE;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  current = otp_read_eccsafe16(addr, &eccerr);
  if (!eccerr || current != OTP_ERASEDVALUE16)
    {
      /* Not blank: either already holds this exact value (success, the
       * word is already in the requested state) or holds something else
       * (this word can never be reprogrammed to the new value).
       */

      ret = current == value ? OK : -EEXIST;
      goto exit_with_lock;
    }

  if (flash_wait_for_operation())
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  flash_unlock_nscr();
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);

  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_PG);
  UP_MB();

  putreg16(value, addr);
  UP_MB();

  ret = OK;
  if (flash_wait_for_operation() ||
      (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_ALL_ERRORS))
    {
      ret = -EIO;
    }

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);
  flash_lock_nscr();

  if (ret == OK)
    {
      current = otp_read_eccsafe16(addr, &eccerr);
      if (eccerr || current != value)
        {
          ret = -EIO;
        }
    }

exit_with_lock:
  nxmutex_unlock(&g_lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_otp_word_write32
 *
 * Description:
 *   32-bit counterpart of stm32_otp_word_write16(); see there for the
 *   full explanation.  As with stm32_otp_word_read32(), "word" is a
 *   32-bit word index here.
 *
 ****************************************************************************/

int stm32_otp_word_write32(uint32_t word, uint32_t value)
{
  uintptr_t addr;
  uint32_t  current;
  bool      eccerr;
  int       ret;

  if (word >= FLASH_OTP_SIZE / sizeof(uint32_t))
    {
      return -EINVAL;
    }

  addr = STM32_OTP_BASE + word * sizeof(uint32_t);

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  current = otp_read_eccsafe32(addr, &eccerr);
  if (!eccerr || current != OTP_ERASEDVALUE32)
    {
      ret = current == value ? OK : -EEXIST;
      goto exit_with_lock;
    }

  if (flash_wait_for_operation())
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  flash_unlock_nscr();
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);

  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_PG);
  UP_MB();

  putreg32(value, addr);
  UP_MB();

  ret = OK;
  if (flash_wait_for_operation() ||
      (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_ALL_ERRORS))
    {
      ret = -EIO;
    }

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);
  flash_lock_nscr();

  if (ret == OK)
    {
      current = otp_read_eccsafe32(addr, &eccerr);
      if (eccerr || current != value)
        {
          ret = -EIO;
        }
    }

exit_with_lock:
  nxmutex_unlock(&g_lock);
  return ret;
}

#endif /* CONFIG_STM32H5_OTP_WRITE */
#endif /* CONFIG_STM32H5_OTP_WORD */

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
  struct stm32h5_flash_priv_s *priv;

  priv = flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }

  return priv->stpage + ((addr - priv->base) / FLASH_PAGE_SIZE);
}

size_t up_progmem_getaddress(size_t page)
{
  struct stm32h5_flash_priv_s *priv;

  if (page >= H5_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  priv = flash_bank(STM32_FLASH_BASE + (page * FLASH_PAGE_SIZE));
  return priv->base + (page - priv->stpage) * FLASH_PAGE_SIZE;
}

size_t up_progmem_neraseblocks(void)
{
  return H5_FLASH_NBLOCKS;
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

  if (page >= H5_FLASH_NPAGES)
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
  bool bank_swap;
  bool phy_bank1;
  bool icache;
  int ret;
  size_t block_address = STM32_FLASH_BASE + (block * FLASH_BLOCK_SIZE);

  if (block >= H5_FLASH_NBLOCKS)
    {
      return -EFAULT;
    }

  /* If SWAP_BANK == 0: Logical bank 2 corresponds to physical bank 2
   * If SWAP_BANK == 1: Logical bank 2 corresponds to physical bank 1
   */

  bank_swap = (bool)(getreg32(STM32_FLASH_OPTSR_CUR) &
                     FLASH_OPTSR_CUR_SWAP_BANK);

  if ((!bank_swap && block >= H5_FLASH_BANK_NBLOCKS) ||
      (bank_swap && block < H5_FLASH_BANK_NBLOCKS))
    {
      phy_bank1 = false;
    }
  else
    {
      phy_bank1 = true;
    }

  /* Convert logical block number into physical erase sector number for
   * placing in NSCR_SNB
   */

  block = block % H5_FLASH_BANK_NBLOCKS;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  icache = flash_icache_suspend();

  if (flash_wait_for_operation())
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  /* Get flash ready and begin erasing single block */

  flash_unlock_nscr();

  if (phy_bank1)
    {
      modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_BKSEL, FLASH_NSCR_SER);
    }
  else
    {
      modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_BKSEL | FLASH_NSCR_SER);
    }

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_SNB_MASK, FLASH_NSCR_SNB(block));

  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_STRT);

  /* Wait for erase operation to complete */

  if (flash_wait_for_operation())
    {
      ret = -EIO;
      goto exit_with_unlock;
    }

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_SER, 0);
  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_SNB_MASK, 0);

  ret = 0;
  up_invalidate_dcache(block_address, block_address + FLASH_BLOCK_SIZE);

exit_with_unlock:
  flash_lock_nscr();

exit_with_lock:
  flash_icache_resume(icache);
  nxmutex_unlock(&g_lock);

  /* Verify */

  if (ret == 0 &&
      stm32h5_israngeerased(block_address, up_progmem_erasesize(block)) == 0)
    {
      ret = up_progmem_erasesize(block); /* Success */
    }
  else
    {
      ret = -EIO; /* Failure */
    }

  return ret;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  struct stm32h5_flash_priv_s *priv;
  uint32_t     *fp;
  uint32_t     *rp;
  uint32_t     *ll        = (uint32_t *)buf;
  size_t       faddr;
  size_t       written    = count;
  bool         icache;
  int          ret;
  const size_t pagesize   = up_progmem_pagesize(0); /* 128bit, 16 bytes per page */
  const size_t llperpage  = pagesize / sizeof(uint32_t);
  size_t       pcount     = count / pagesize;

  priv = flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }

  /* Check for valid address range */

  if (addr < priv->base ||
      addr + count > priv->base + (H5_FLASH_TOTALSIZE / 2))
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

  icache = flash_icache_suspend();

  if (flash_wait_for_operation())
    {
      written = -EIO;
      goto exit_with_lock;
    }

  /* Get flash ready for write */

  flash_unlock_nscr();

  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_PG);

  /* Write */

  for (ll = (uint32_t *)buf, faddr = addr; pcount;
       pcount -= 1, ll += llperpage, faddr += pagesize)
    {
      fp = (uint32_t *)faddr;
      rp = ll;

      UP_MB();

      /* Write 4 32 bit word and wait to complete */

      *fp++ = *rp++;
      *fp++ = *rp++;
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

      /* H5 corrects single ECC errors, so only check double errors */

      if (getreg32(STM32_FLASH_ECCDETR) & FLASH_ECCDETR_ECCD)
        {
          modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);
          modifyreg32(STM32_FLASH_ECCDETR, 0, FLASH_ECCDETR_ECCD);
          ret = -EIO;
          goto exit_with_unlock;
        }
    }

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);

exit_with_unlock:
  flash_lock_nscr();

  if (written > 0)
    {
      for (ll = (uint32_t *)buf, faddr = addr, pcount = count / pagesize;
           pcount; pcount -= 1, ll += llperpage, faddr += pagesize)
        {
          fp = (uint32_t *)faddr;
          rp = ll;

          modifyreg32(STM32_FLASH_NSCCR, 0, ~0);

          if ((*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++))
            {
              written = -EIO;
              break;
            }

          if (getreg32(STM32_FLASH_ECCDETR) & FLASH_ECCDETR_ECCD)
            {
              modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);
              modifyreg32(STM32_FLASH_ECCDETR, 0, FLASH_ECCDETR_ECCD);
              written = -EIO;
              break;
            }
        }

      modifyreg32(STM32_FLASH_NSCCR, 0, ~0);
    }

exit_with_lock:
  flash_icache_resume(icache);
  nxmutex_unlock(&g_lock);
  return written;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}

#endif /* CONFIG_ARCH_HAVE_PROGMEM */
