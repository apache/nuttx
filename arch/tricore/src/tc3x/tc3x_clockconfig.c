/****************************************************************************
 * arch/tricore/src/tc3x/tc3x_clockconfig.c
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
#include <nuttx/bits.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/arch.h>
#include "tricore_internal.h"

#include "hardware/tc3x_clock.h"

#ifndef DIV_ROUND_UP
#  define DIV_ROUND_UP(n, d)  (((n) + (d) - 1u) / (d))
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FIELD_PREP(mask, shift, val)   (((uint32_t)(val) << (shift)) & (mask))
#define REGADDR(off)   (TC3X_SCU_BASE + (off))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static inline void tc3x_busywait(unsigned int loops)
{
  volatile unsigned int i;
  for (i = 0; i < loops; i++)
    {
      __asm__ __volatile__("nop");
    }
}

static inline void tc3x_ccucon0_wait_unlocked(void)
{
  while (getreg32(REGADDR(TC3X_CLOCK_CCUCON0_OFFSET)) & TC3X_CCUCON0_LCK)
    {
      __asm__ __volatile__("nop");
    }
}

static inline void tc3x_ccucon_wait_unlocked(uint32_t offset)
{
  while (getreg32(REGADDR(offset)) & BIT(31))
    {
      __asm__ __volatile__("nop");
    }
}

static void tc3x_osc_init(void)
{
  uint32_t addr = REGADDR(TC3X_CLOCK_OSCCON_OFFSET);
  uint32_t val;
  uint32_t oscval;

  oscval = (TC3X_FOSC_HZ / 1000000u) - 15u;

  val  = getreg32(addr);
  val &= ~(TC3X_OSCCON_MODE_MASK | TC3X_OSCCON_OSCVAL_MASK |
           TC3X_OSCCON_GAINSEL_MASK);
  val |= TC3X_OSCCON_MODE_XTAL;
  val |= FIELD_PREP(TC3X_OSCCON_OSCVAL_MASK,
                    TC3X_OSCCON_OSCVAL_SHIFT,
                    oscval);
  val |= FIELD_PREP(TC3X_OSCCON_GAINSEL_MASK,
                    TC3X_OSCCON_GAINSEL_SHIFT,
                    3u);

  aurix_safety_endinit_enable(false);
  putreg32(val, addr);
  aurix_safety_endinit_enable(true);

  while (1)
    {
      val = getreg32(addr);
      if ((val & TC3X_OSCCON_PLLHV) && (val & TC3X_OSCCON_PLLLV))
        {
          break;
        }

      tc3x_busywait(100);
    }
}

static void tc3x_syspll_init(void)
{
  uint32_t con0 = REGADDR(TC3X_CLOCK_SYSPLLCON0_OFFSET);
  uint32_t con1 = REGADDR(TC3X_CLOCK_SYSPLLCON1_OFFSET);
  uint32_t stat = REGADDR(TC3X_CLOCK_SYSPLLSTAT_OFFSET);
  uint32_t val;

  val  = TC3X_SYSPLLCON0_INSEL_FOSC;
  val |= FIELD_PREP(TC3X_SYSPLLCON0_PDIV_MASK,
                    TC3X_SYSPLLCON0_PDIV_SHIFT,
                    TC3X_SYSPLL_PDIV - 1u);
  val |= FIELD_PREP(TC3X_SYSPLLCON0_NDIV_MASK,
                    TC3X_SYSPLLCON0_NDIV_SHIFT,
                    TC3X_SYSPLL_NDIV - 1u);
  val |= TC3X_SYSPLLCON0_PLLPWD;
  val |= TC3X_SYSPLLCON0_RESLD;

  aurix_safety_endinit_enable(false);
  putreg32(val, con0);
  aurix_safety_endinit_enable(true);

  val = FIELD_PREP(TC3X_SYSPLLCON1_K2DIV_MASK,
                   TC3X_SYSPLLCON1_K2DIV_SHIFT,
                   TC3X_SYSPLL_K2DIV_INIT - 1u);

  aurix_safety_endinit_enable(false);
  putreg32(val, con1);
  aurix_safety_endinit_enable(true);

  while (!(getreg32(stat) & TC3X_SYSPLLSTAT_LOCK))
    {
      tc3x_busywait(10);
    }
}

static void tc3x_perpll_init(void)
{
  uint32_t con0 = REGADDR(TC3X_CLOCK_PERPLLCON0_OFFSET);
  uint32_t con1 = REGADDR(TC3X_CLOCK_PERPLLCON1_OFFSET);
  uint32_t stat = REGADDR(TC3X_CLOCK_PERPLLSTAT_OFFSET);
  uint32_t val;

  val  = 0;
  if (TC3X_PERPLL_DIVBY)
    {
      val |= TC3X_PERPLLCON0_DIVBY;
    }

  val |= FIELD_PREP(TC3X_PERPLLCON0_PDIV_MASK,
                    TC3X_PERPLLCON0_PDIV_SHIFT,
                    TC3X_PERPLL_PDIV - 1u);
  val |= FIELD_PREP(TC3X_PERPLLCON0_NDIV_MASK,
                    TC3X_PERPLLCON0_NDIV_SHIFT,
                    TC3X_PERPLL_NDIV - 1u);
  val |= TC3X_PERPLLCON0_PLLPWD;
  val |= TC3X_PERPLLCON0_RESLD;

  aurix_safety_endinit_enable(false);
  putreg32(val, con0);
  aurix_safety_endinit_enable(true);

  val  = 0;
  val |= FIELD_PREP(TC3X_PERPLLCON1_K2DIV_MASK,
                    TC3X_PERPLLCON1_K2DIV_SHIFT,
                    TC3X_PERPLL_K2DIV - 1u);
  val |= FIELD_PREP(TC3X_PERPLLCON1_K3DIV_MASK,
                    TC3X_PERPLLCON1_K3DIV_SHIFT,
                    TC3X_PERPLL_K3DIV - 1u);

  aurix_safety_endinit_enable(false);
  putreg32(val, con1);
  aurix_safety_endinit_enable(true);

  while (!(getreg32(stat) & TC3X_PERPLLSTAT_LOCK))
    {
      tc3x_busywait(10);
    }
}

static void tc3x_ccu_set_dividers(void)
{
  uint32_t fsource0 = TC3X_FSOURCE0_HZ;
  uint32_t fsource1 = TC3X_FSOURCE1_HZ;
  uint32_t fsource2 = TC3X_FSOURCE2_HZ;

  uint32_t fsri_div;
  uint32_t fspb_div;
  uint32_t fbbb_div;
  uint32_t fstm_div;
  uint32_t fgeth_div;
  uint32_t fmcanh_div;
  uint32_t fadas_div;

  uint32_t fmcani_div;
  uint32_t fi2c_div;
  uint32_t fmsc_div;
  uint32_t fqspi_div;
  uint32_t fasclinf_div;
  uint32_t fasclinsi_div;

  uint32_t val;

  /* System domain from fsource0 (300 MHz) */

  fsri_div   = DIV_ROUND_UP(fsource0, TC3X_FSRI_TARGET_HZ) & 0xfu;
  fspb_div   = DIV_ROUND_UP(fsource0, TC3X_FSPB_TARGET_HZ) & 0xfu;
  fbbb_div   = DIV_ROUND_UP(fsource0, TC3X_FBBB_TARGET_HZ) & 0xfu;
  fstm_div   = DIV_ROUND_UP(fsource0, TC3X_FSTM_TARGET_HZ) & 0xfu;
  fgeth_div  = DIV_ROUND_UP(fsource0, TC3X_FGETH_TARGET_HZ) & 0xfu;
  fmcanh_div = DIV_ROUND_UP(fsource0, TC3X_FMCANH_TARGET_HZ) & 0xfu;
  fadas_div  = DIV_ROUND_UP(fsource0, TC3X_FADAS_TARGET_HZ) & 0xfu;

  /* Peripheral from fsource1 (160 MHz) */

  fmcani_div    = DIV_ROUND_UP(fsource1, TC3X_FMCANI_TARGET_HZ) & 0xfu;
  fasclinsi_div = DIV_ROUND_UP(fsource1, TC3X_FASCLINS_TARGET_HZ) & 0xfu;

  /* Peripheral from fsource2 (200 MHz) */

  fqspi_div    = DIV_ROUND_UP(fsource2, TC3X_FQSPI_TARGET_HZ) & 0xfu;
  fmsc_div     = DIV_ROUND_UP(fsource2, TC3X_FMSC_TARGET_HZ) & 0xfu;
  fi2c_div     = DIV_ROUND_UP(fsource2, TC3X_FI2C_TARGET_HZ) & 0xfu;
  fasclinf_div = DIV_ROUND_UP(fsource2, TC3X_FASCLINF_TARGET_HZ) & 0xfu;

  val  = 0;
  val |= FIELD_PREP(TC3X_CCUCON0_STMDIV_MASK,
                    TC3X_CCUCON0_STMDIV_SHIFT,
                    fstm_div);
  val |= FIELD_PREP(TC3X_CCUCON0_GTMDIV_MASK,
                    TC3X_CCUCON0_GTMDIV_SHIFT,
                    1u);
  val |= FIELD_PREP(TC3X_CCUCON0_SRIDIV_MASK,
                    TC3X_CCUCON0_SRIDIV_SHIFT,
                    fsri_div);
  val |= FIELD_PREP(TC3X_CCUCON0_SPBDIV_MASK,
                    TC3X_CCUCON0_SPBDIV_SHIFT,
                    fspb_div);
  val |= FIELD_PREP(TC3X_CCUCON0_BBBDIV_MASK,
                    TC3X_CCUCON0_BBBDIV_SHIFT,
                    fbbb_div);
  val |= FIELD_PREP(TC3X_CCUCON0_FSIDIV_MASK,
                    TC3X_CCUCON0_FSIDIV_SHIFT,
                    3u);
  val |= FIELD_PREP(TC3X_CCUCON0_FSI2DIV_MASK,
                    TC3X_CCUCON0_FSI2DIV_SHIFT,
                    1u);

  tc3x_ccucon0_wait_unlocked();
  aurix_safety_endinit_enable(false);
  putreg32(val, REGADDR(TC3X_CLOCK_CCUCON0_OFFSET));
  aurix_safety_endinit_enable(true);

  val  = 0;
  val |= FIELD_PREP(TC3X_CCUCON5_GETHDIV_MASK,
                    TC3X_CCUCON5_GETHDIV_SHIFT,
                    fgeth_div);
  val |= FIELD_PREP(TC3X_CCUCON5_MCANHDIV_MASK,
                    TC3X_CCUCON5_MCANHDIV_SHIFT,
                    fmcanh_div);
  val |= FIELD_PREP(TC3X_CCUCON5_ADASDIV_MASK,
                    TC3X_CCUCON5_ADASDIV_SHIFT,
                    fadas_div);
  val |= TC3X_CCUCON5_UP;

  aurix_safety_endinit_enable(false);
  putreg32(val, REGADDR(TC3X_CLOCK_CCUCON5_OFFSET));
  aurix_safety_endinit_enable(true);

  tc3x_ccucon0_wait_unlocked();

  val  = 0;
  val |= FIELD_PREP(TC3X_CCUCON1_MCANDIV_MASK,
                    TC3X_CCUCON1_MCANDIV_SHIFT,
                    fmcani_div);
  val |= FIELD_PREP(TC3X_CCUCON1_CLKSELMCAN_MASK,
                    TC3X_CCUCON1_CLKSELMCAN_SHIFT,
                    TC3X_CLKSEL_MCAN_FMCANI);
  val |= FIELD_PREP(TC3X_CCUCON1_I2CDIV_MASK,
                    TC3X_CCUCON1_I2CDIV_SHIFT,
                    fi2c_div);
  val |= FIELD_PREP(TC3X_CCUCON1_MSCDIV_MASK,
                    TC3X_CCUCON1_MSCDIV_SHIFT,
                    fmsc_div);
  val |= FIELD_PREP(TC3X_CCUCON1_CLKSELMSC_MASK,
                    TC3X_CCUCON1_CLKSELMSC_SHIFT,
                    TC3X_CLKSEL_SRC2);
  val |= FIELD_PREP(TC3X_CCUCON1_QSPIDIV_MASK,
                    TC3X_CCUCON1_QSPIDIV_SHIFT,
                    fqspi_div);
  val |= FIELD_PREP(TC3X_CCUCON1_CLKSELQSPI_MASK,
                    TC3X_CCUCON1_CLKSELQSPI_SHIFT,
                    TC3X_CLKSEL_SRC2);

  if (TC3X_PLL1DIVDIS)
    {
      val |= TC3X_CCUCON1_PLL1DIVDIS;
    }

  tc3x_ccucon_wait_unlocked(TC3X_CLOCK_CCUCON1_OFFSET);
  aurix_safety_endinit_enable(false);
  putreg32(val, REGADDR(TC3X_CLOCK_CCUCON1_OFFSET));
  aurix_safety_endinit_enable(true);
  tc3x_ccucon_wait_unlocked(TC3X_CLOCK_CCUCON1_OFFSET);

  val  = 0;
  val |= FIELD_PREP(TC3X_CCUCON2_ASCLINFDIV_MASK,
                    TC3X_CCUCON2_ASCLINFDIV_SHIFT,
                    fasclinf_div);
  val |= FIELD_PREP(TC3X_CCUCON2_ASCLINSDIV_MASK,
                    TC3X_CCUCON2_ASCLINSDIV_SHIFT,
                    fasclinsi_div);
  val |= FIELD_PREP(TC3X_CCUCON2_CLKSELASCLINS_MASK,
                    TC3X_CCUCON2_CLKSELASCLINS_SHIFT,
                    TC3X_CLKSEL_ASCLINS_FASCLINSI);
  val |= TC3X_CCUCON2_EBUPERON;
  val |= TC3X_CCUCON2_ERAYPERON;
  val |= TC3X_CCUCON2_HSPDMPERON;

  tc3x_ccucon_wait_unlocked(TC3X_CLOCK_CCUCON2_OFFSET);
  aurix_safety_endinit_enable(false);
  putreg32(val, REGADDR(TC3X_CLOCK_CCUCON2_OFFSET));
  aurix_safety_endinit_enable(true);
  tc3x_ccucon_wait_unlocked(TC3X_CLOCK_CCUCON2_OFFSET);
}

static void tc3x_ccu_select_clock(uint32_t clksel)
{
  uint32_t addr = REGADDR(TC3X_CLOCK_CCUCON0_OFFSET);
  uint32_t val;

  tc3x_ccucon0_wait_unlocked();

  val  = getreg32(addr);
  val &= ~(TC3X_CCUCON0_CLKSEL_MASK | TC3X_CCUCON0_UP);
  val |= FIELD_PREP(TC3X_CCUCON0_CLKSEL_MASK,
                    TC3X_CCUCON0_CLKSEL_SHIFT,
                    clksel);
  val |= TC3X_CCUCON0_UP;

  aurix_safety_endinit_enable(false);
  putreg32(val, addr);
  aurix_safety_endinit_enable(true);

  tc3x_ccucon0_wait_unlocked();
}

static void tc3x_syspll_k2_step(void)
{
  uint32_t con1 = REGADDR(TC3X_CLOCK_SYSPLLCON1_OFFSET);
  uint32_t stat = REGADDR(TC3X_CLOCK_SYSPLLSTAT_OFFSET);

  static const uint32_t k2_steps[] =
  {
    4u,  /* fPLL0 = 150 MHz */

    3u,  /* fPLL0 = 200 MHz */

    2u,  /* fPLL0 = 300 MHz */
  };

  unsigned int i;

  for (i = 0; i < sizeof(k2_steps) / sizeof(k2_steps[0]); i++)
    {
      uint32_t k2div_val = k2_steps[i] - 1u;

      while (!(getreg32(stat) & TC3X_SYSPLLSTAT_K2RDY))
        {
          __asm__ __volatile__("nop");
        }

      aurix_safety_endinit_enable(false);
      putreg32(k2div_val, con1);
      aurix_safety_endinit_enable(true);

      tc3x_busywait(50000);
    }

  while (!(getreg32(stat) & TC3X_SYSPLLSTAT_K2RDY))
    {
      __asm__ __volatile__("nop");
    }
}

void up_clockconfig(void)
{
  tc3x_osc_init();
  tc3x_syspll_init();
  tc3x_perpll_init();
  tc3x_ccu_set_dividers();
  tc3x_ccu_select_clock(1u);
  tc3x_syspll_k2_step();
}
