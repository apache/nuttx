/****************************************************************************
 * arch/tricore/src/tc4x/tc4x_clockconfig.c
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

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/bits.h>

#include "tricore_internal.h"
#include "hardware/tc4x_clock.h"

#ifndef DIV_ROUND_UP
#  define DIV_ROUND_UP(n, d)  (((n) + (d) - 1u) / (d))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void tc4x_busywait(unsigned int loops)
{
  volatile unsigned int i;
  for (i = 0; i < loops; i++)
    {
      __asm__ __volatile__("nop");
    }
}

/* Convenience: field prep without needing ffs() since we know SHIFT. */

#define FIELD_PREP(mask, shift, val)   (((uint32_t)(val) << (shift)) & (mask))
#define REGADDR(off)   (TC4X_CLOCK_BASE + (off))

static inline void tc4x_ccu_wait_unlocked(void)
{
  while (getreg32(REGADDR(TC4X_CLOCK_CCUSTAT_OFFSET)) & TC4X_CCUSTAT_LCK)
    {
      __asm__ __volatile__("nop");
    }
}

static void tc4x_osc_init(void)
{
  uint32_t addr;
  uint32_t val;

  /* OSCCON: MODE = external clock, INSEL = XTAL input */

  addr = REGADDR(TC4X_CLOCK_OSCCON_OFFSET);
  val  = getreg32(addr);

  val &= ~(TC4X_OSCCON_MODE_MASK | TC4X_OSCCON_INSEL_MASK);
  val |= TC4X_OSCCON_MODE_EXTCLK;
  val |= TC4X_OSCCON_INSEL_XTAL;

  tc4x_ccu_wait_unlocked();
  putreg32(val, addr);

  uint32_t mhz = TC4X_FOSC_HZ / 1000000u;
  uint32_t oscval = 0;

  if (mhz > 15u)
    {
      oscval = mhz - 15u;
    }

  val  = FIELD_PREP(TC4X_OSCMON1_OSCVAL_MASK,
                    TC4X_OSCMON1_OSCVAL_SHIFT,
                    oscval);

  tc4x_ccu_wait_unlocked();
  putreg32(val, REGADDR(TC4X_CLOCK_OSCMON1_OFFSET));

  /* Give oscillator some time to stabilise */

  tc4x_busywait(100000);
}

static void tc4x_syspll_init(void)
{
  uint32_t con0;
  uint32_t con1;
  uint32_t stat;
  uint32_t val;

  con0 = REGADDR(TC4X_CLOCK_SYSPLLCON0_OFFSET);
  con1 = REGADDR(TC4X_CLOCK_SYSPLLCON1_OFFSET);
  stat = REGADDR(TC4X_CLOCK_SYSPLLSTAT_OFFSET);

  val = getreg32(con0);
  val = ~TC4X_SYSPLLCON0_PLLPWR;

  tc4x_ccu_wait_unlocked();
  putreg32(val, con0);

  /* Wait for SYSPLL power status */

  while ((getreg32(stat) & TC4X_PLLSTAT_PWRSTAT) != 0u)
    {
    }

  val = 0;
  val |= FIELD_PREP(TC4X_SYSPLLCON1_K3PREDIV_MASK,
                    TC4X_SYSPLLCON1_K3PREDIV_SHIFT,
                    TC4X_SYSPLL_K3PREDIV_DEF);

  tc4x_ccu_wait_unlocked();
  modreg32(val, TC4X_SYSPLLCON1_K3PREDIV_MASK, con1);

  val = 0;
  val |= FIELD_PREP(TC4X_SYSPLLCON1_K2DIV_MASK,
                    TC4X_SYSPLLCON1_K2DIV_SHIFT,
                    TC4X_SYSPLL_K2DIV_DEF);
  val |= FIELD_PREP(TC4X_SYSPLLCON1_K3DIV_MASK,
                    TC4X_SYSPLLCON1_K3DIV_SHIFT,
                    TC4X_SYSPLL_K3DIV_DEF);

  tc4x_ccu_wait_unlocked();
  modreg32(val,
           (TC4X_SYSPLLCON1_K2DIV_MASK | TC4X_SYSPLLCON1_K3DIV_MASK), con1);

  /* SYSPLL init */

  val = 0;
  val |= FIELD_PREP(TC4X_SYSPLLCON0_NDIV_MASK,
                    TC4X_SYSPLLCON0_NDIV_SHIFT,
                    TC4X_SYSPLL_NDIV - 1u);
  val |= FIELD_PREP(TC4X_SYSPLLCON0_PDIV_MASK,
                    TC4X_SYSPLLCON0_PDIV_SHIFT,
                    TC4X_SYSPLL_PDIV - 1u);
  val |= TC4X_SYSPLLCON0_PLLPWR;
  val |= TC4X_SYSPLLCON0_RESLD;

  tc4x_ccu_wait_unlocked();
  putreg32(val, con0);

  /* Wait for SYSPLL power status */

  while ((getreg32(stat) & TC4X_PLLSTAT_PWRSTAT) != 1u)
    {
    }

  tc4x_busywait(100);
}

static void tc4x_perpll_init(void)
{
  uint32_t con0;
  uint32_t con1;
  uint32_t stat;
  uint32_t val;

  con0 = REGADDR(TC4X_CLOCK_PERPLLCON0_OFFSET);
  con1 = REGADDR(TC4X_CLOCK_PERPLLCON1_OFFSET);
  stat = REGADDR(TC4X_CLOCK_PERPLLSTAT_OFFSET);

  val = getreg32(con0);
  val = ~TC4X_PERPLLCON0_PLLPWR;

  tc4x_ccu_wait_unlocked();
  putreg32(val, con0);

  /* Wait for PERPLL power status */

  while ((getreg32(stat) & TC4X_PLLSTAT_PWRSTAT) != 0u)
    {
    }

  val = 0;
  val |= FIELD_PREP(TC4X_PERPLLCON1_K2PREDIV_MASK,
                    TC4X_PERPLLCON1_K2PREDIV_SHIFT,
                    TC4X_PERPLL_K2PREDIV_DEF);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K3PREDIV_MASK,
                    TC4X_PERPLLCON1_K3PREDIV_SHIFT,
                    TC4X_PERPLL_K3PREDIV_DEF);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K4PREDIV_MASK,
                    TC4X_PERPLLCON1_K4PREDIV_SHIFT,
                    TC4X_PERPLL_K4PREDIV_DEF);
  tc4x_ccu_wait_unlocked();
  modreg32(val,
           (TC4X_PERPLLCON1_K2PREDIV_MASK |
            TC4X_PERPLLCON1_K3PREDIV_MASK |
            TC4X_PERPLLCON1_K4PREDIV_MASK), con1);

  val = 0;
  val |= FIELD_PREP(TC4X_PERPLLCON1_K2DIV_MASK,
                    TC4X_PERPLLCON1_K2DIV_SHIFT,
                    TC4X_PERPLL_K2DIV_DEF);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K3DIV_MASK,
                    TC4X_PERPLLCON1_K3DIV_SHIFT,
                    TC4X_PERPLL_K3DIV_DEF);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K4DIV_MASK,
                    TC4X_PERPLLCON1_K4DIV_SHIFT,
                    TC4X_PERPLL_K4DIV_DEF);
  tc4x_ccu_wait_unlocked();
  modreg32(val,
           (TC4X_PERPLLCON1_K2DIV_MASK |
            TC4X_PERPLLCON1_K3DIV_MASK |
            TC4X_PERPLLCON1_K4DIV_MASK), con1);

  val = 0;
  val |= FIELD_PREP(TC4X_PERPLLCON0_NDIV_MASK,
                    TC4X_PERPLLCON0_NDIV_SHIFT,
                    TC4X_PERPLL_NDIV - 1u);
  val |= FIELD_PREP(TC4X_PERPLLCON0_PDIV_MASK,
                    TC4X_PERPLLCON0_PDIV_SHIFT,
                    TC4X_PERPLL_PDIV - 1u);
  val |= TC4X_PERPLLCON0_PLLPWR;
  val |= TC4X_PERPLLCON0_RESLD;

  tc4x_ccu_wait_unlocked();
  putreg32(val, con0);

  /* Wait for PERPLL power status */

  while ((getreg32(stat) & TC4X_PLLSTAT_PWRSTAT) != 1u)
    {
    }

  tc4x_busywait(100);
}

static void tc4x_ccu_set_dividers(void)
{
  uint64_t sys_vco;
  uint32_t fsource0;
  uint64_t per_vco;
  uint32_t fsource1;
  uint32_t fsource2;
  uint32_t fsourceppu;

  uint32_t fsri_div;
  uint32_t fspb_div;
  uint32_t ftpb_div;
  uint32_t fstm_div;
  uint32_t fleth_div;
  uint32_t ffsi_div;
  uint32_t fgeth_div;
  uint32_t fegtm_div;
  uint32_t fmcanh_div;

  uint32_t fmcani_div;
  uint32_t fasclinf_div;
  uint32_t fasclinsi_div;
  uint32_t fqspi_div;
  uint32_t fi2c_div;

  uint32_t fmcanxl_div;
  uint32_t fppu_div;
  uint32_t val;

  /* 500 Mhz */

  sys_vco = (uint64_t)TC4X_FOSC_HZ *
            (uint64_t)TC4X_SYSPLL_NDIV /
            (uint64_t)TC4X_SYSPLL_PDIV;

  /* 500 Mhz */

  fsource0 = (uint32_t)(sys_vco /
              ((uint64_t)TC4X_SYSPLL_K2DIV *
               (uint64_t)TC4X_SYSPLL_K2PREDIV));

  /* 500 Mhz */

  fsourceppu = (uint32_t)(sys_vco /
              ((uint64_t)TC4X_SYSPLL_K3DIV *
               (uint64_t)TC4X_SYSPLL_K3PREDIV));

  /* 800 Mhz */

  per_vco = (uint64_t)TC4X_FOSC_HZ *
            (uint64_t)TC4X_PERPLL_NDIV /
            (uint64_t)TC4X_PERPLL_PDIV;

  /* 160 Mhz */

  fsource1 = (uint32_t)(per_vco /
              ((uint64_t)TC4X_PERPLL_K2DIV *
               (uint64_t)TC4X_PERPLL_K2PREDIV));

  /* 200 Mhz */

  fsource2 = (uint32_t)(per_vco /
              ((uint64_t)TC4X_PERPLL_K3DIV *
               (uint64_t)TC4X_PERPLL_K3PREDIV));

  /* ---- System domain dividers from fsource0 (500 MHz) ---- */

  fspb_div  = DIV_ROUND_UP(fsource0, TC4X_FSPB_TARGET_HZ) & 0xfu;
  fsri_div  = DIV_ROUND_UP(fsource0, TC4X_FSRI_TARGET_HZ) & 0xfu;
  ffsi_div  = DIV_ROUND_UP(fsource0, TC4X_FFSI_TARGET_HZ) & 0xfu;
  fstm_div  = DIV_ROUND_UP(fsource0, TC4X_FSTM_TARGET_HZ) & 0xfu;
  fgeth_div = DIV_ROUND_UP(fsource0, TC4X_FGETH_TARGET_HZ) & 0xfu;
  fmcanh_div = DIV_ROUND_UP(fsource0, TC4X_MCANH_TARGET_HZ) & 0xfu;
  fmcanxl_div = DIV_ROUND_UP(fsource0, TC4X_CANXL_TARGET_HZ) & 0xfu;

  ftpb_div  = DIV_ROUND_UP(fsource0, TC4X_FTPB_TARGET_HZ) & 0xfu;
  fleth_div = DIV_ROUND_UP(fsource0, TC4X_FLETH_TARGET_HZ) & 0xfu;
  fegtm_div = DIV_ROUND_UP(fsource0, TC4X_EGTM_TARGET_HZ) & 0xfu;

  /* ---- Peripheral domain dividers from fsource1 (160 MHz) ---- */

  fmcani_div    = DIV_ROUND_UP(fsource1, TC4X_MCANI_TARGET_HZ) & 0xfu;
  fasclinsi_div = DIV_ROUND_UP(fsource1, TC4X_ASCLINSI_TARGET_HZ) & 0xfu;

  /* ---- Peripheral domain dividers from fsource2 (200 MHz) ---- */

  fi2c_div      = DIV_ROUND_UP(fsource2, TC4X_I2C_TARGET_HZ) & 0xfu;
  fasclinf_div  = DIV_ROUND_UP(fsource2, TC4X_ASCLINF_TARGET_HZ) & 0xfu;
  fqspi_div     = DIV_ROUND_UP(fsource2, TC4X_QSPI_TARGET_HZ) & 0xfu;

  /* ---- Peripheral domain dividers from fsourceppu (500 MHz) ---- */

  fppu_div     = DIV_ROUND_UP(fsourceppu, TC4X_PPU_TARGET_HZ) & 0xfu;

  /* ---- Program SYSCCUCON0 ---- */

  val = 0;
  val |= FIELD_PREP(TC4X_SYSCCUCON0_SPBDIV_MASK,
                    TC4X_SYSCCUCON0_SPBDIV_SHIFT,
                    fspb_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON0_TPBDIV_MASK,
                    TC4X_SYSCCUCON0_TPBDIV_SHIFT,
                    ftpb_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON0_SRIDIV_MASK,
                    TC4X_SYSCCUCON0_SRIDIV_SHIFT,
                    fsri_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON0_FSIDIV_MASK,
                    TC4X_SYSCCUCON0_FSIDIV_SHIFT,
                    ffsi_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON0_STMDIV_MASK,
                    TC4X_SYSCCUCON0_STMDIV_SHIFT,
                    fstm_div);

  val |= TC4X_SYSCCUCON0_FSI2DIV;

  /* LPDIV left at reset */

  val |= TC4X_SYSCCUCON0_UP;

  tc4x_ccu_wait_unlocked();
  putreg32(val, REGADDR(TC4X_CLOCK_SYSCCUCON0_OFFSET));

  /* ---- Program SYSCCUCON1 ---- */

  val  = getreg32(REGADDR(TC4X_CLOCK_SYSCCUCON1_OFFSET));
  val = 0;
  val |= FIELD_PREP(TC4X_SYSCCUCON1_GETHDIV_MASK,
                    TC4X_SYSCCUCON1_GETHDIV_SHIFT,
                    fgeth_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON1_EGTMDIV_MASK,
                    TC4X_SYSCCUCON1_EGTMDIV_SHIFT,
                    fegtm_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON1_MCANHDIV_MASK,
                    TC4X_SYSCCUCON1_MCANHDIV_SHIFT,
                    fmcanh_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON1_LETHDIV_MASK,
                    TC4X_SYSCCUCON1_LETHDIV_SHIFT,
                    fleth_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON1_CANXLHDIV_MASK,
                    TC4X_SYSCCUCON1_CANXLHDIV_SHIFT,
                    fmcanxl_div);
  val |= TC4X_SYSCCUCON1_UP;

  tc4x_ccu_wait_unlocked();
  putreg32(val, REGADDR(TC4X_CLOCK_SYSCCUCON1_OFFSET));

  /* ---- Program PERCCUCON0 ---- */

  tc4x_ccu_wait_unlocked();
  putreg32(0, REGADDR(TC4X_CLOCK_PERCCUCON0_OFFSET));
  val  = getreg32(REGADDR(TC4X_CLOCK_PERCCUCON0_OFFSET));
  val |= FIELD_PREP(TC4X_PERCCUCON0_MCANDIV_MASK,
                    TC4X_PERCCUCON0_MCANDIV_SHIFT,
                    fmcani_div);
  val |= FIELD_PREP(TC4X_PERCCUCON0_CLKSELMCAN_MASK,
                    TC4X_PERCCUCON0_CLKSELMCAN_SHIFT,
                    TC4X_CLKSEL_MCAN_FSYS);

  val |= FIELD_PREP(TC4X_PERCCUCON0_QSPIDIV_MASK,
                    TC4X_PERCCUCON0_QSPIDIV_SHIFT,
                    fqspi_div);
  val |= FIELD_PREP(TC4X_PERCCUCON0_CLKSELQSPI_MASK,
                    TC4X_PERCCUCON0_CLKSELQSPI_SHIFT,
                    TC4X_CLKSEL_QSPI_FSPB);

  val |= FIELD_PREP(TC4X_PERCCUCON0_I2CDIV_MASK,
                    TC4X_PERCCUCON0_I2CDIV_SHIFT,
                    fi2c_div);
  val |= FIELD_PREP(TC4X_PERCCUCON0_PPUDIV_MASK,
                    TC4X_PERCCUCON0_PPUDIV_SHIFT,
                    fppu_div);

  tc4x_ccu_wait_unlocked();
  putreg32(val, REGADDR(TC4X_CLOCK_PERCCUCON0_OFFSET));

  /* ---- Program PERCCUCON1 ---- */

  tc4x_ccu_wait_unlocked();
  putreg32(0, REGADDR(TC4X_CLOCK_PERCCUCON1_OFFSET));
  val  = getreg32(REGADDR(TC4X_CLOCK_PERCCUCON1_OFFSET));
  val |= FIELD_PREP(TC4X_PERCCUCON1_ASCLINFDIV_MASK,
                    TC4X_PERCCUCON1_ASCLINFDIV_SHIFT,
                    fasclinf_div);
  val |= FIELD_PREP(TC4X_PERCCUCON1_ASCLINSDIV_MASK,
                    TC4X_PERCCUCON1_ASCLINSDIV_SHIFT,
                    fasclinsi_div);
  val |= FIELD_PREP(TC4X_PERCCUCON1_CLKSELASCLINS_MASK,
                    TC4X_PERCCUCON1_CLKSELASCLINS_SHIFT,
                    TC4X_CLKSEL_ASCLINS_FOSC0);

  tc4x_ccu_wait_unlocked();
  putreg32(val, REGADDR(TC4X_CLOCK_PERCCUCON1_OFFSET));
}

static void tc4x_set_rootclk_source(enum tc4x_rootclk_domain dom,
                                    enum tc4x_clk_source src)
{
  uint32_t addr = REGADDR(TC4X_CLOCK_CCUCON_OFFSET);
  uint32_t val  = getreg32(addr);

  switch (dom)
    {
      case TC4X_ROOTCLK_SYS:
        val &= ~TC4X_CCUCON_CLKSELS_MASK;
        val |= FIELD_PREP(TC4X_CCUCON_CLKSELS_MASK,
                          TC4X_CCUCON_CLKSELS_SHIFT,
                          (uint32_t)src);
        break;

      case TC4X_ROOTCLK_PER:
        val &= ~TC4X_CCUCON_CLKSELP_MASK;
        val |= FIELD_PREP(TC4X_CCUCON_CLKSELP_MASK,
                          TC4X_CCUCON_CLKSELP_SHIFT,
                          (uint32_t)src);
        break;
    }

  tc4x_ccu_wait_unlocked();
  putreg32(val, addr);
}

static void tc4x_set_sysclk_source(enum tc4x_clk_source src)
{
  tc4x_set_rootclk_source(TC4X_ROOTCLK_SYS, src);
}

static void tc4x_set_perclk_source(enum tc4x_clk_source src)
{
  tc4x_set_rootclk_source(TC4X_ROOTCLK_PER, src);
}

static void tc4x_ramposc_init(void)
{
  uint32_t con;
  uint32_t stat;
  uint32_t val;

  con = REGADDR(TC4X_CLOCK_RAMPCON0_OFFSET);
  val  = getreg32(con);

  val  |= TC4X_RAMPCON0_PWR;

  tc4x_ccu_wait_unlocked();
  putreg32(val, con);

  stat = REGADDR(TC4X_CLOCK_RAMPSTAT_OFFSET);
  while (((getreg32(stat) & TC4X_RAMPSTAT_ACTIVE) >>
          TC4X_RAMPSTAT_ACTIVE_SHIFT) != 1)
    {
    }

  while (!((getreg32(stat) & TC4X_RAMPSTAT_FSTAT) >>
           TC4X_RAMPSTAT_FSTAT_SHIFT))
    {
    }

  if (((getreg32(stat) & TC4X_RAMPSTAT_FSTAT) >>
       TC4X_RAMPSTAT_FSTAT_SHIFT) != 1)
    {
      val = getreg32(con);
      val |= TC4X_RAMPCON0_CMD_BOTTOM;

      tc4x_ccu_wait_unlocked();
      putreg32(val, con);

      while (getreg32(stat) & TC4X_RAMPSTAT_FLLLOCK)
        {
        }
    }
}

static void tc4x_ramposc_move(void)
{
  uint32_t con;
  uint32_t stat;
  uint32_t val;

  con = REGADDR(TC4X_CLOCK_RAMPCON0_OFFSET);
  stat = REGADDR(TC4X_CLOCK_RAMPSTAT_OFFSET);

  val = getreg32(stat);
  if ((val & TC4X_RAMPSTAT_ACTIVE) &&
       (val & TC4X_RAMPSTAT_SSTAT) &&
       ((val & TC4X_RAMPSTAT_FSTAT) >> TC4X_RAMPSTAT_FSTAT_SHIFT) != 1)
    {
      return;
    }

  val  = 0;
  val |= TC4X_RAMPCON0_PWR;
  val |= FIELD_PREP(TC4X_RAMPCON0_UFL_MASK,
                    TC4X_RAMPCON0_UFL_SHIFT,
                    500);
  val |= FIELD_PREP(TC4X_RAMPCON0_CMD_MASK,
                    TC4X_RAMPCON0_CMD_SHIFT,
                    1);

  tc4x_ccu_wait_unlocked();
  putreg32(val, con);
  tc4x_busywait(100000);
}

static void tc4x_set_syspll_divider(void)
{
  uint32_t con1;
  uint32_t val;

  con1 = REGADDR(TC4X_CLOCK_SYSPLLCON1_OFFSET);

  /* SYSPLL default */

  val = 0;
  val |= FIELD_PREP(TC4X_SYSPLLCON1_K3PREDIV_MASK,
                    TC4X_SYSPLLCON1_K3PREDIV_SHIFT,
                    1);

  tc4x_ccu_wait_unlocked();
  modreg32(val, TC4X_SYSPLLCON1_K3PREDIV_MASK, con1);
  tc4x_busywait(100);

  val = 0;
  val |= FIELD_PREP(TC4X_SYSPLLCON1_K2DIV_MASK,
                    TC4X_SYSPLLCON1_K2DIV_SHIFT,
                    0);
  val |= FIELD_PREP(TC4X_SYSPLLCON1_K3DIV_MASK,
                    TC4X_SYSPLLCON1_K3DIV_SHIFT,
                    0);

  tc4x_ccu_wait_unlocked();
  modreg32(val,
           (TC4X_SYSPLLCON1_K2DIV_MASK | TC4X_SYSPLLCON1_K3DIV_MASK), con1);
}

static void tc4x_set_perpll_divider(void)
{
  uint32_t con1;
  uint32_t val;

  con1 = REGADDR(TC4X_CLOCK_PERPLLCON1_OFFSET);

  val = 0;
  val |= FIELD_PREP(TC4X_PERPLLCON1_K2PREDIV_MASK,
                    TC4X_PERPLLCON1_K2PREDIV_SHIFT,
                    0);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K3PREDIV_MASK,
                    TC4X_PERPLLCON1_K3PREDIV_SHIFT,
                    TC4X_PERPLL_K3PREDIV_DEF);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K4PREDIV_MASK,
                    TC4X_PERPLLCON1_K4PREDIV_SHIFT,
                    TC4X_PERPLL_K4PREDIV_DEF);
  tc4x_ccu_wait_unlocked();
  modreg32(val,
           (TC4X_PERPLLCON1_K2PREDIV_MASK |
            TC4X_PERPLLCON1_K3PREDIV_MASK |
            TC4X_PERPLLCON1_K4PREDIV_MASK), con1);

  val = 0;
  val |= FIELD_PREP(TC4X_PERPLLCON1_K2DIV_MASK,
                    TC4X_PERPLLCON1_K2DIV_SHIFT,
                    TC4X_PERPLL_K2DIV - 1u);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K3DIV_MASK,
                    TC4X_PERPLLCON1_K3DIV_SHIFT,
                    TC4X_PERPLL_K3DIV - 1u);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K4DIV_MASK,
                    TC4X_PERPLLCON1_K4DIV_SHIFT,
                    TC4X_PERPLL_K4DIV - 1u);
  tc4x_ccu_wait_unlocked();
  modreg32(val,
           (TC4X_PERPLLCON1_K2DIV_MASK |
            TC4X_PERPLLCON1_K3DIV_MASK |
            TC4X_PERPLLCON1_K4DIV_MASK), con1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clockconfig(void)
{
  uint32_t stat;
  uint32_t val;

  /* External oscillator */

  tc4x_osc_init();

  /* RAMP oscillator */

  tc4x_ramposc_init();

  /* SYSPLL */

  tc4x_syspll_init();

  /* PERPLL */

  tc4x_perpll_init();

  /* CCU dividers for CPU + bus + periph clocks */

  tc4x_ccu_set_dividers();

  /* RAMP oscillator move */

  tc4x_ramposc_move();

  stat = REGADDR(TC4X_CLOCK_RAMPSTAT_OFFSET);
  val = getreg32(stat);
  if (((val & TC4X_RAMPSTAT_FLLLOCK) >> TC4X_RAMPSTAT_FLLLOCK_SHIFT) != 1)
    {
      return;
    }

  stat = REGADDR(TC4X_CLOCK_SYSPLLSTAT_OFFSET);
  val = getreg32(stat);
  if (((val & TC4X_PLLSTAT_PLLLOCK) >> TC4X_PLLSTAT_PLLLOCK_SHIFT) != 1)
    {
      return;
    }

  stat = REGADDR(TC4X_CLOCK_PERPLLSTAT_OFFSET);
  val = getreg32(stat);
  if (((val & TC4X_PLLSTAT_PLLLOCK) >> TC4X_PLLSTAT_PLLLOCK_SHIFT) != 1)
    {
      return;
    }

  /* SYSPLL divider */

  tc4x_set_syspll_divider();

  /* Select PLLs as clock sources */

  tc4x_set_sysclk_source(TC4X_CLK_SOURCE_PLL);

  /* PERPLL divider */

  tc4x_set_perpll_divider();
  tc4x_set_perclk_source(TC4X_CLK_SOURCE_PLL);
}
