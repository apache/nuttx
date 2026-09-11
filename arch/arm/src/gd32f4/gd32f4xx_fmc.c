/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_fmc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "gd32f4xx_fmc.h"
#include "gd32f4xx.h"

#if defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULTT)
#  warning "Default Flash Configuration Used - See Override Flash Size Designator"
#endif

/* Erase/program busy-waits MUST execute from SRAM: while FMC is busy the
 * same flash bank cannot fetch instructions, so a flash-resident wait loop
 * deadlocks (debugger still sees PC near gd32_fmc_page_erase / STAT idle).
 */

#ifdef CONFIG_ARCH_RAMFUNCS
#  include "arm_internal.h"
#  define FMC_RAMFUNC __ramfunc__
#else
#  define FMC_RAMFUNC locate_code(".ramfunc") farcall_function noinline_function
#endif

/* Inlined register helpers for .ramfunc paths — must not call flash-resident
 * modifyreg32/putreg32 while FMC is busy (instruction fetch from same bank stalls).
 */

#  define FMC_REG_GET(a)       (*(volatile uint32_t *)(a))
#  define FMC_REG_PUT(a, v)    do { *(volatile uint32_t *)(a) = (v); } while (0)
#  define FMC_REG_CLRSET(a, c, s) \
     do { uint32_t _v = FMC_REG_GET(a); _v &= ~(c); _v |= (s); FMC_REG_PUT(a, _v); } while (0)

static FMC_RAMFUNC inline irqstate_t fmc_irq_save(void)
{
  uint32_t primask;

  __asm__ volatile ("mrs %0, primask\n cpsid i" : "=r" (primask) :: "memory");
  return (irqstate_t)primask;
}

static FMC_RAMFUNC inline void fmc_irq_restore(irqstate_t flags)
{
  __asm__ volatile ("msr primask, %0" :: "r" (flags) : "memory");
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_gd32_fmc_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fmc_state_get
 *
 * Description:
 *   Get the FMC state
 *
 * Returned Value:
 *  State of FMC
 *
 ****************************************************************************/

static FMC_RAMFUNC gd32_fmc_state_enum gd32_fmc_state_get(void)
{
  uint32_t stat = FMC_REG_GET(GD32_FMC_STAT);
  gd32_fmc_state_enum fmc_state = FMC_READY;

  if (stat & FMC_STAT_BUSY)
    {
      fmc_state = FMC_BUSY;
    }
  else if (stat & FMC_STAT_RDDERR)
    {
      fmc_state = FMC_RDDERR;
    }
  else if (stat & FMC_STAT_PGSERR)
    {
      fmc_state = FMC_PGSERR;
    }
  else if (stat & FMC_STAT_PGMERR)
    {
      fmc_state = FMC_PGMERR;
    }
  else if (stat & FMC_STAT_WPERR)
    {
      fmc_state = FMC_WPERR;
    }
  else if (stat & FMC_STAT_OPERR)
    {
      fmc_state = FMC_OPERR;
    }
  else
    {
      fmc_state = FMC_READY;
    }

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_ready_wait
 *
 * Description:
 *   Check whether FMC is ready or not
 *
 * Returned Value:
 *  State of FMC
 *
 ****************************************************************************/

static FMC_RAMFUNC gd32_fmc_state_enum gd32_fmc_ready_wait(uint32_t timeout)
{
  gd32_fmc_state_enum fmc_state = FMC_BUSY;

  /* Wait for FMC ready */

  do
    {
      /* Get FMC state */

      fmc_state = gd32_fmc_state_get();
      timeout--;
    }
  while ((FMC_BUSY == fmc_state) && (timeout > 0));

  if (0 == timeout)
    {
      fmc_state = FMC_TOERR;
    }

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fmc_wscnt_set
 *
 * Description:
 *   Set the wait state counter value
 *
 * Parameters:
 *   wscnt - Wait state counter value
 *
 ****************************************************************************/

void gd32_fmc_wscnt_set(uint32_t wscnt)
{
  uint32_t regval;

  regval = getreg32(GD32_FMC_WS);

  /* set the wait state counter value */

  regval &= ~FMC_WS_WSCNT_MASK;
  regval |= wscnt;
  putreg32(regval, GD32_FMC_WS);
}

/****************************************************************************
 * Name: gd32_fmc_unlock
 *
 * Description:
 *   Unlock the main FMC operation
 *
 ****************************************************************************/

int gd32_fmc_unlock(void)
{
  int ret;

  ret = nxmutex_lock(&g_gd32_fmc_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (getreg32(GD32_FMC_CTL) & FMC_CTL_LK)
    {
      /* Write the FMC key */

      putreg32(FMC_UNLOCK_KEY0, GD32_FMC_KEY);
      putreg32(FMC_UNLOCK_KEY1, GD32_FMC_KEY);
    }

  nxmutex_unlock(&g_gd32_fmc_lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_fmc_lock
 *
 * Description:
 *   Lock the main FMC operation
 *
 ****************************************************************************/

int gd32_fmc_lock(void)
{
  int ret;

  ret = nxmutex_lock(&g_gd32_fmc_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the LK bit */

  modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_LK);

  nxmutex_unlock(&g_gd32_fmc_lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_fmc_unlock_ram
 *
 * Description:
 *   Unlock FMC from SRAM (no mutex / no flash-resident helpers).
 *
 ****************************************************************************/

FMC_RAMFUNC void gd32_fmc_unlock_ram(void)
{
  if (FMC_REG_GET(GD32_FMC_CTL) & FMC_CTL_LK)
    {
      FMC_REG_PUT(GD32_FMC_KEY, FMC_UNLOCK_KEY0);
      FMC_REG_PUT(GD32_FMC_KEY, FMC_UNLOCK_KEY1);
    }
}

/****************************************************************************
 * Name: gd32_fmc_lock_ram
 *
 * Description:
 *   Lock FMC from SRAM.
 *
 ****************************************************************************/

FMC_RAMFUNC void gd32_fmc_lock_ram(void)
{
  FMC_REG_CLRSET(GD32_FMC_CTL, 0, FMC_CTL_LK);
}

/****************************************************************************
 * Name: gd32_fmc_flag_clear_ram
 *
 * Description:
 *   Clear FMC status flags from SRAM (W1C).
 *
 ****************************************************************************/

FMC_RAMFUNC void gd32_fmc_flag_clear_ram(uint32_t fmc_flag)
{
  FMC_REG_PUT(GD32_FMC_STAT, fmc_flag);
}

#if defined(CONFIG_GD32F4_GD32F470)

/****************************************************************************
 * Name: gd32_fmc_page_erase
 *
 * Description:
 *   Erase page
 *
 * Parameters:
 *   fmc_page - Select the page to erase
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

FMC_RAMFUNC gd32_fmc_state_enum gd32_fmc_page_erase(uint32_t fmc_page)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;
  uint32_t regval;
  irqstate_t flags;

  /* Align to 4 KiB; refuse odd addresses (manual: misaligned page → hang). */

  if ((fmc_page & 0xfffu) != 0)
    {
      return FMC_PGSERR;
    }

  flags = fmc_irq_save();

  /* Abort any prior stuck START/SER/PE_EN before starting a new erase. */

  FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_SER | FMC_CTL_START | FMC_CTL_PG, 0);
  FMC_REG_PUT(GD32_FMC_PECFG, 0);
  FMC_REG_PUT(GD32_FMC_STAT, FMC_STAT_END | FMC_STAT_OPERR | FMC_STAT_WPERR |
              FMC_STAT_PGMERR | FMC_STAT_PGSERR | FMC_STAT_RDDERR);

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      FMC_REG_PUT(GD32_FMC_PEKEY, FMC_UNLOCK_PE_KEY);

      /* Manual 2.3.4: PE_EN + PE_ADDR, SN=0, SER, START */

      regval = FMC_PE_EN | (fmc_page & 0x1fffffffu);
      FMC_REG_PUT(GD32_FMC_PECFG, regval);
      FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_SN_MASK, FMC_CTL_SER);
      FMC_REG_CLRSET(GD32_FMC_CTL, 0, FMC_CTL_START);
      __asm__ volatile ("dsb" ::: "memory");
      __asm__ volatile ("isb" ::: "memory");

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Always drop command bits after the wait (success or timeout). */

      FMC_REG_PUT(GD32_FMC_PECFG, 0);
      FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_SER | FMC_CTL_START, 0);

      /* On success clear the status flags. On timeout/failure keep them (and
       * any stuck BUSY) so the caller can fail fast and diagnostics can read
       * the cause instead of silently continuing into a flash-read deadlock.
       */

      if (FMC_READY == fmc_state)
        {
          FMC_REG_PUT(GD32_FMC_STAT, FMC_STAT_END | FMC_STAT_OPERR |
                      FMC_STAT_WPERR | FMC_STAT_PGMERR | FMC_STAT_PGSERR |
                      FMC_STAT_RDDERR);
        }
    }

  fmc_irq_restore(flags);
  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_erase_range_and_reset
 *
 * Description:
 *   Erase [start, end) with 4 KiB page erase from RAM, then system reset.
 *   Used before MCUboot overwrite so the loader only programs into already
 *   erased primary flash (same-bank erase from flash-resident code hangs).
 *
 ****************************************************************************/

FMC_RAMFUNC void gd32_fmc_erase_range_and_reset(uint32_t start, uint32_t end)
{
  uint32_t addr;
  uint32_t i;
  irqstate_t flags;

  flags = fmc_irq_save();

  if (FMC_REG_GET(GD32_FMC_CTL) & FMC_CTL_LK)
    {
      FMC_REG_PUT(GD32_FMC_KEY, FMC_UNLOCK_KEY0);
      FMC_REG_PUT(GD32_FMC_KEY, FMC_UNLOCK_KEY1);
    }

  start &= ~0xfffu;

  for (addr = start; addr < end; addr += 4096u)
    {
      for (i = 0; i < 4096u; i += 4u)
        {
          if (*(volatile uint32_t *)(addr + i) != 0xffffffffu)
            {
              (void)gd32_fmc_page_erase(addr);
              break;
            }
        }
    }

  /* AIRCR SYSRESETREQ (key 0x05FA). Never return. */

  FMC_REG_PUT(0xe000ed0c, 0x05fa0004);
  for (; ; )
    {
    }

  fmc_irq_restore(flags); /* unreachable; keeps compiler quiet if any */
}

#endif /* page erase / erase-range: CONFIG_GD32F4_GD32F470 */
/****************************************************************************
 * Name: gd32_fmc_sector_erase
 *
 * Description:
 *   Erase sector
 *
 * Parameters:
 *   fmc_sector - Select the sector to erase
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

FMC_RAMFUNC gd32_fmc_state_enum gd32_fmc_sector_erase(uint32_t fmc_sector)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;
  irqstate_t flags;

  flags = fmc_irq_save();

  /* Abort any prior stuck command bits; drop page-erase config too. */

  FMC_REG_PUT(GD32_FMC_PECFG, 0);
  FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_SER | FMC_CTL_START | FMC_CTL_PG |
                 FMC_CTL_SN_MASK, 0);
  FMC_REG_PUT(GD32_FMC_STAT, FMC_STAT_END | FMC_STAT_OPERR | FMC_STAT_WPERR |
              FMC_STAT_PGMERR | FMC_STAT_PGSERR | FMC_STAT_RDDERR);

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Manual 2.3.5: SER, SN, START (SN encodes sector number). */

      FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_SN_MASK, FMC_CTL_SER | fmc_sector);
      FMC_REG_CLRSET(GD32_FMC_CTL, 0, FMC_CTL_START);
      __asm__ volatile ("dsb" ::: "memory");
      __asm__ volatile ("isb" ::: "memory");

      /* Wait for the FMC ready (must run from RAM). */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the SER / SN / START bits (always, including timeout). */

      FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_SER | FMC_CTL_START | FMC_CTL_SN_MASK,
                     0);
      FMC_REG_PUT(GD32_FMC_PECFG, 0);
      FMC_REG_PUT(GD32_FMC_STAT, FMC_STAT_END | FMC_STAT_OPERR | FMC_STAT_WPERR |
                  FMC_STAT_PGMERR | FMC_STAT_PGSERR | FMC_STAT_RDDERR);
    }

  fmc_irq_restore(flags);
  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_mass_erase
 *
 * Description:
 *   Erase whole chip
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_mass_erase(void)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start whole chip erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER0);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER1);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the MER bits */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER0, 0);
      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER1, 0);
    }

  /* Return the fmc state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_bank0_erase
 *
 * Description:
 *   Erase whole bank0
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_bank0_erase(void)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start FMC bank0 erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER0);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the MER0 bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER0, 0);
    }

  /* Return the fmc state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_bank1_erase
 *
 * Description:
 *   Erase whole bank1
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_bank1_erase(void)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start FMC bank1 erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER1);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the MER1 bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER1, 0);
    }

  /* Return the fmc state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_word_program
 *
 * Description:
 *   Program a word at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Word to program(0x00000000 - 0xFFFFFFFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

FMC_RAMFUNC gd32_fmc_state_enum gd32_fmc_word_program(uint32_t address,
                                                       uint32_t data)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;
  irqstate_t flags;

  /* IRQs off: flash-resident ISRs hang if they fetch while FMC BUSY. */

  flags = fmc_irq_save();

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Set the PG bit to start program */

      FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_PSZ_MASK, FMC_CTL_PSZ_WORD);
      FMC_REG_CLRSET(GD32_FMC_CTL, 0, FMC_CTL_PG);

      FMC_REG_PUT(address, data);
      __asm__ volatile ("dsb" ::: "memory");

      /* Wait for the FMC ready (must run from RAM; see FMC_RAMFUNC). */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PG bit */

      FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_PG, 0);

      /* Verify while still in RAM (same-bank flash fetch during BUSY hangs). */

      if (FMC_READY == fmc_state && FMC_REG_GET(address) != data)
        {
          fmc_state = FMC_PGMERR;
        }
    }

  fmc_irq_restore(flags);

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_halfword_program
 *
 * Description:
 *   Program a half word at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Word to program(0x0000 - 0xFFFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_halfword_program(uint32_t address,
                                              uint16_t data)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Set the PG bit to start program */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PSZ_MASK, FMC_CTL_PSZ_HALF_WORD);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_PG);

      putreg16(data, address);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PG bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PG, 0);
    }

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_byte_program
 *
 * Description:
 *   Program a byte at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Byte to program(0x00 - 0xFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

FMC_RAMFUNC gd32_fmc_state_enum gd32_fmc_byte_program(uint32_t address,
                                                       uint8_t data)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;
  irqstate_t flags;

  flags = fmc_irq_save();

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Set the PG bit to start program */

      FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_PSZ_MASK, FMC_CTL_PSZ_BYTE);
      FMC_REG_CLRSET(GD32_FMC_CTL, 0, FMC_CTL_PG);

      *(volatile uint8_t *)address = data;
      __asm__ volatile ("dsb" ::: "memory");

      /* Wait for the FMC ready (must run from RAM). */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PG bit */

      FMC_REG_CLRSET(GD32_FMC_CTL, FMC_CTL_PG, 0);

      if (FMC_READY == fmc_state &&
          *(volatile uint8_t *)address != data)
        {
          fmc_state = FMC_PGMERR;
        }
    }

  fmc_irq_restore(flags);

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_ob_unlock
 *
 * Description:
 *   Unlock the option byte operation
 *
 ****************************************************************************/

void gd32_ob_unlock(void)
{
  if (getreg32(GD32_FMC_OBCTL0) & FMC_OBCTL0_OB_LK)
    {
      /* Write the FMC key */

      putreg32(FMC_OB_UNLOCK_KEY0, GD32_FMC_OBKEY);
      putreg32(FMC_OB_UNLOCK_KEY1, GD32_FMC_OBKEY);
    }
}

/****************************************************************************
 * Name: gd32_ob_lock
 *
 * Description:
 *   Lock the option byte operation
 *
 ****************************************************************************/

void gd32_ob_lock(void)
{
  /* Reset the OB_LK bit */

  modifyreg32(GD32_FMC_OBCTL0, 0, FMC_OBCTL0_OB_LK);
}

/****************************************************************************
 * Name: gd32_ob_start
 *
 * Description:
 *   Send option byte change command
 *
 ****************************************************************************/

void gd32_ob_start(void)
{
  /* Set the OB_START bit in OBCTL0 register */

  modifyreg32(GD32_FMC_OBCTL0, 0, FMC_OBCTL0_OB_START);
}

/****************************************************************************
 * Name: gd32_ob_write_protection_enable
 *
 * Description:
 *   Enable write protection
 *
 * Parameters:
 *   ob_wp - Specify sector to be write protected
 *
 ****************************************************************************/

int gd32_ob_write_protection_enable(uint32_t ob_wp)
{
  uint32_t regval0;
  uint32_t regval1;

  regval0 = getreg32(GD32_FMC_OBCTL0);
  regval1 = getreg32(GD32_FMC_OBCTL1);

  gd32_fmc_state_enum fmc_state = FMC_READY;
  if (regval0 & FMC_OBCTL0_DRP)
    {
      return -1;
    }

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      regval0 &= (~(uint32_t)ob_wp << 16U);
      regval1 &= (~(ob_wp & 0xffff0000u));
      putreg32(regval0, GD32_FMC_OBCTL0);
      putreg32(regval1, GD32_FMC_OBCTL1);

      return 0;
    }
  else
    {
      return -1;
    }
}

/****************************************************************************
 * Name: gd32_ob_write_protection_disable
 *
 * Description:
 *   Disable write protection
 *
 * Parameters:
 *   ob_wp - Specify sector to be write protected
 *
 ****************************************************************************/

int gd32_ob_write_protection_disable(uint32_t ob_wp)
{
  uint32_t regval0;
  uint32_t regval1;

  regval0 = getreg32(GD32_FMC_OBCTL0);
  regval1 = getreg32(GD32_FMC_OBCTL1);

  gd32_fmc_state_enum fmc_state = FMC_READY;
  if (regval0 & FMC_OBCTL0_DRP)
    {
      return -1;
    }

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      regval0 |= ((uint32_t)(ob_wp << 16U));
      regval1 |= (ob_wp & 0xffff0000u);
      putreg32(regval0, GD32_FMC_OBCTL0);
      putreg32(regval1, GD32_FMC_OBCTL1);

      return 0;
    }
  else
    {
      return -1;
    }
}

/****************************************************************************
 * Name: gd32_fmc_flag_clear
 *
 * Description:
 *   Clear the FMC pending flag
 *
 * Parameters:
 *   fmc_flag - FMC flag
 *
 ****************************************************************************/

void gd32_fmc_flag_clear(uint32_t fmc_flag)
{
  /* Status flags are write-1-to-clear (see GD32F4xx FMC_STAT). */

  putreg32(fmc_flag, GD32_FMC_STAT);
}
