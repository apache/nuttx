/****************************************************************************
 * arch/arm/src/imxrt/imxrt_clockconfig_ver2.c
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

#include "arm_internal.h"
#include <arch/board/board.h>
#include "hardware/imxrt_ccm.h"
#include "hardware/imxrt_dcdc.h"
#include "imxrt_clockconfig_ver2.h"
#include "imxrt_lcd.h"
#include "imxrt_pmu.h"
#include "hardware/imxrt_memorymap.h"
#include "hardware/imxrt_iomuxc.h"
#include "hardware/rt117x/imxrt117x_osc.h"
#include "hardware/rt117x/imxrt117x_pll.h"
#include "hardware/rt117x/imxrt117x_anadig.h"
#include "hardware/rt117x/imxrt117x_ocotp.h"
#include "hardware/rt117x/imxrt117x_gpc.h"

#include "hardware/imxrt_pinmux.h"
#include "imxrt_iomuxc.h"
#include "imxrt_gpio.h"

#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_PLL_MIN_FREQ 650000000
#define OSC24_FREQ         24000000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_oscsetup
 ****************************************************************************/

static void imxrt_oscsetup(void)
{
  uint32_t reg;

#ifdef WE_WROTE_OSC_400M_AUDIO_DRIVER
  /* Config 1Mhz clock from 400M RC OSC */

  putreg32(OSC_RC_400M_CTRL3_EN_1M_CLK | OSC_RC_400M_CTRL3_MUX_1M_CLK,
           IMXRT_OSC_RC_400M_CTRL3_CLR);
#endif

  /* Init OSC RC 16M */

  reg = getreg32(IMXRT_ANADIG_OSC_OSC_16M_CTRL);
  reg |= ANADIG_OSC_OSC_16M_CTRL_EN_IRC4M16M;
  putreg32(reg, IMXRT_ANADIG_OSC_OSC_16M_CTRL);

  /* Init OSC RC 400M */

  reg = getreg32(IMXRT_ANADIG_OSC_OSC_400M_CTRL1);
  reg &= ~ANADIG_OSC_OSC_400M_CTRL1_PWD;
  putreg32(reg, IMXRT_ANADIG_OSC_OSC_400M_CTRL1);

  reg = getreg32(IMXRT_ANADIG_OSC_OSC_400M_CTRL2);
  reg |= ANADIG_OSC_OSC_400M_CTRL2_ENABLE_CLK;
  putreg32(reg, IMXRT_ANADIG_OSC_OSC_400M_CTRL2);

  reg = getreg32(IMXRT_ANADIG_OSC_OSC_400M_CTRL1);
  reg |= ANADIG_OSC_OSC_400M_CTRL1_CLKGATE_400MEG;
  putreg32(reg, IMXRT_ANADIG_OSC_OSC_400M_CTRL1);

  /* Init OSC RC 48M */

  reg = getreg32(IMXRT_ANADIG_OSC_OSC_48M_CTRL);
  reg |= ANADIG_OSC_OSC_48M_CTRL_TEN;
  reg |= ANADIG_OSC_OSC_48M_CTRL_RC_48M_DIV2_EN;
  putreg32(reg, IMXRT_ANADIG_OSC_OSC_48M_CTRL);

  /* Config OSC 24M */

  reg = getreg32(IMXRT_ANADIG_OSC_OSC_24M_CTRL);
  reg |= ANADIG_OSC_OSC_24M_CTRL_OSC_EN | ANADIG_OSC_OSC_24M_CTRL_LP_EN;
  putreg32(reg, IMXRT_ANADIG_OSC_OSC_24M_CTRL);

  /* Wait for 24M OSC to be stable. */

  while ((getreg32(IMXRT_ANADIG_OSC_OSC_24M_CTRL) &
         ANADIG_OSC_OSC_24M_CTRL_OSC_24M_STABLE) !=
         ANADIG_OSC_OSC_24M_CTRL_OSC_24M_STABLE);

  /* Switch M7 Core CLOCK_ROOT0 to OSC_RC_48M_DIV2 first */

  reg = getreg32(IMXRT_CCM_CR_CTRL(0));
  reg &= (~CCM_CR_CTRL_MUX_MASK | CCM_CR_CTRL_DIV_MASK | CCM_CR_CTRL_OFF);
  reg |= CCM_CR_CTRL_MUX_SRCSEL(M7_CLK_ROOT_OSC_RC_48M_DIV2);
  reg |= CCM_CR_CTRL_DIV(1);
  putreg32(reg, IMXRT_CCM_CR_CTRL(0));

  /* Switch M7 Systick CLOCK_ROOT8 to OSC_RC_48M_DIV2 first */

  reg = getreg32(IMXRT_CCM_CR_CTRL(8));
  reg &= (~CCM_CR_CTRL_MUX_MASK | CCM_CR_CTRL_DIV_MASK | CCM_CR_CTRL_OFF);
  reg |= CCM_CR_CTRL_MUX_SRCSEL(M7_CLK_ROOT_OSC_RC_48M_DIV2);
  reg |= CCM_CR_CTRL_DIV(1);
  putreg32(reg, IMXRT_CCM_CR_CTRL(8));

  /* FlexRAM AXI CLK ROOT */

  putreg32(CCM_CG_CTRL_RSTDIV(1) | CCM_CG_CTRL_DIV0(1),
  IMXRT_CCM_CG_CTRL(0));
}

/****************************************************************************
 * Name: imxrt_pllsetup
 ****************************************************************************/

static void imxrt_armpllsetup(void)
{
  uint32_t reg;

  reg = getreg32(IMXRT_ANADIG_PLL_ARM_PLL_CTRL);

  if ((reg & ANADIG_PLL_ARM_PLL_CTRL_POWERUP) != 0
      && ((reg & ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT_MASK)
       == g_initial_clkconfig.arm_pll.loop_div)
      && (((reg & ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_MASK) >>
      ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_SHIFT)
       == g_initial_clkconfig.arm_pll.post_div))
    {
      /* PLL Already configured no need to reconfigure */

      if ((reg & ANADIG_PLL_ARM_PLL_CTRL_ENABLE_CLK) == 0)
        {
          reg |= ANADIG_PLL_ARM_PLL_CTRL_ENABLE_CLK;
        }

      if ((reg & ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE) != 0)
        {
          reg &= ~ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE;
        }

      putreg32(reg, IMXRT_ANADIG_PLL_ARM_PLL_CTRL);
      return;
    }

  imxrt_pmu_enable_pll_ldo();

  reg &= ~ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_STABLE;

  if ((reg & (ANADIG_PLL_ARM_PLL_CTRL_POWERUP |
            ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_STABLE)) != 0)
    {
      /* Power down PLL */

      reg &= ~(ANADIG_PLL_ARM_PLL_CTRL_POWERUP |
            ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_STABLE);
      reg |= ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE;
      putreg32(reg, IMXRT_ANADIG_PLL_ARM_PLL_CTRL);
    }

  /* Set config */

  reg &= ~(ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT_MASK |
           ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_MASK);
  reg |= ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT(
          g_initial_clkconfig.arm_pll.loop_div) |
         ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL(
          g_initial_clkconfig.arm_pll.post_div) |
          ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE |
          ANADIG_PLL_ARM_PLL_CTRL_POWERUP;
  putreg32(reg, IMXRT_ANADIG_PLL_ARM_PLL_CTRL);

  /* Wait 30 usec */

  up_mdelay(1);

  while ((getreg32(IMXRT_ANADIG_PLL_ARM_PLL_CTRL) &
        ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_STABLE) == 0);

  /* Enable and ungate the clock */

  reg |= ANADIG_PLL_ARM_PLL_CTRL_ENABLE_CLK;
  reg &= ~ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE;
  putreg32(reg, IMXRT_ANADIG_PLL_ARM_PLL_CTRL);
}

static void imxrt_pll1_init(void)
{
  uint32_t reg;

  /* Bypass Sys Pll1. */

  imxrt_modify_pll1g(IMXRT_ANADIG_MISC_ADDR_PLL1G_CTRL0,
                     AI_PLL1G_CTRL0_BYPASS_MASK, 0);

  if (g_initial_clkconfig.sys_pll1.enable)
    {
      reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL1_CTRL);
      reg |= ANADIG_PLL_SYS_PLL1_CTRL_ENABLE_CLK;
      reg |= ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_DIV2;
      reg |= ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_DIV5;
      reg |= ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_GATE;
      putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL1_CTRL);

      imxrt_modify_pll1g(IMXRT_ANADIG_MISC_ADDR_PLL1G_CTRL3,
              g_initial_clkconfig.sys_pll1.denom,
              ~g_initial_clkconfig.sys_pll1.denom);
      imxrt_modify_pll1g(IMXRT_ANADIG_MISC_ADDR_PLL1G_CTRL2,
              g_initial_clkconfig.sys_pll1.num,
              ~g_initial_clkconfig.sys_pll1.num);
      imxrt_modify_pll1g(IMXRT_ANADIG_MISC_ADDR_PLL1G_CTRL0,
              g_initial_clkconfig.sys_pll1.div,
              ~g_initial_clkconfig.sys_pll1.div);

      imxrt_modify_pll1g(IMXRT_ANADIG_MISC_ADDR_PLL1G_CTRL0,
                         AI_PLL1G_CTRL0_ENABLE_MASK |
                         AI_PLL1G_CTRL0_POWER_UP_MASK |
                         AI_PLL1G_CTRL0_HOLD_RING_OFF_MASK |
                         AI_PLL1G_CTRL0_PLL_REG_EN_MASK, 0);

      /* Disable bypass Sys Pll1. */

      imxrt_modify_pll1g(IMXRT_ANADIG_MISC_ADDR_PLL1G_CTRL0,
                     0, AI_PLL1G_CTRL0_BYPASS_MASK);
    }
  else
    {
      reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL1_CTRL);
      reg &= ~ANADIG_PLL_SYS_PLL1_CTRL_ENABLE_CLK;
      reg &= ~ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_DIV2;
      reg &= ~ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_DIV5;
      reg &= ~ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_GATE;
      putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL1_CTRL);
      imxrt_modify_pll1g(IMXRT_ANADIG_MISC_ADDR_PLL1G_CTRL0, 0,
                         AI_PLL1G_CTRL0_ENABLE_MASK |
                         AI_PLL1G_CTRL0_POWER_UP_MASK |
                         AI_PLL1G_CTRL0_PLL_REG_EN_MASK);
    }
}

static void imxrt_pll2_init(void)
{
  uint32_t reg;

  reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);

  if ((reg & ANADIG_PLL_SYS_PLL2_CTRL_POWERUP) != 0)
    {
      reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_SS);

      if (((reg & ANADIG_PLL_SYS_PLL2_SS_ENABLE) == 0) &&
          (!g_initial_clkconfig.sys_pll2.ss_enable))
        {
          /* PLL Already configured no need to reconfigure */

          reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);

          if ((reg & ANADIG_PLL_SYS_PLL2_CTRL_ENABLE_CLK) == 0)
            {
              reg |= ANADIG_PLL_SYS_PLL2_CTRL_ENABLE_CLK;
            }

          if ((reg & ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_GATE) != 0)
            {
              reg &= ~ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_GATE;
            }

          putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);
          return;
        }
    }

  imxrt_pmu_enable_pll_ldo();

  reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_PFD);

  reg |= (ANADIG_PLL_SYS_PLL2_PFD_PFD0_DIV1_CLKGATE |
          ANADIG_PLL_SYS_PLL2_PFD_PFD1_DIV1_CLKGATE |
          ANADIG_PLL_SYS_PLL2_PFD_PFD2_DIV1_CLKGATE |
          ANADIG_PLL_SYS_PLL2_PFD_PFD3_DIV1_CLKGATE);

  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_PFD);

  reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);

  if ((reg & (ANADIG_PLL_SYS_PLL2_CTRL_POWERUP |
      ANADIG_PLL_SYS_PLL2_CTRL_ENABLE_CLK)) != 0)
    {
      /* Power down PLL */

      reg &= ~(ANADIG_PLL_SYS_PLL2_CTRL_POWERUP |
      ANADIG_PLL_SYS_PLL2_CTRL_ENABLE_CLK);
      reg |= ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_GATE;
      putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);
    }

  if (g_initial_clkconfig.sys_pll2.ss_enable)
    {
      putreg32(g_initial_clkconfig.sys_pll2.mfd,
               IMXRT_ANADIG_PLL_SYS_PLL2_MFD);
      putreg32(ANADIG_PLL_SYS_PLL2_SS_ENABLE |
               ANADIG_PLL_SYS_PLL2_SS_STOP(
                   g_initial_clkconfig.sys_pll2.ss_stop) |
               ANADIG_PLL_SYS_PLL2_SS_STEP(
                   g_initial_clkconfig.sys_pll2.ss_step),
               IMXRT_ANADIG_PLL_SYS_PLL2_SS);
    }

  putreg32(ANADIG_PLL_SYS_PLL2_CTRL_PLL_REG_EN |
           ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_GATE,
           IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);

  /* Wait 30 usec */

  up_mdelay(1);

  reg  = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);
  reg |= ANADIG_PLL_SYS_PLL2_CTRL_POWERUP |
         ANADIG_PLL_SYS_PLL2_CTRL_HOLD_RING_OFF;
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);

  /* Wait 250 usec */

  up_mdelay(1);

  reg &= ~ANADIG_PLL_SYS_PLL2_CTRL_HOLD_RING_OFF;
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);

  while ((getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_CTRL) &
        ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_STABLE) !=
        ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_STABLE);

  reg |= ANADIG_PLL_SYS_PLL2_CTRL_ENABLE_CLK;
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);

  reg &= ~ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_GATE;
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);
}

static void imxrt_pll2_pfd(void)
{
  uint32_t reg;

  reg  = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_PFD);
  reg &= ~(ANADIG_PLL_SYS_PLL2_PFD_PFD0_DIV1_CLKGATE |
           ANADIG_PLL_SYS_PLL2_PFD_PFD1_DIV1_CLKGATE |
           ANADIG_PLL_SYS_PLL2_PFD_PFD2_DIV1_CLKGATE |
           ANADIG_PLL_SYS_PLL2_PFD_PFD3_DIV1_CLKGATE);
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_PFD);

  /* Configure PFD dividers */

  reg  = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_PFD);
  reg &= (ANADIG_PLL_SYS_PLL2_PFD_PFD0_STABLE |
          ANADIG_PLL_SYS_PLL2_PFD_PFD1_STABLE |
          ANADIG_PLL_SYS_PLL2_PFD_PFD2_STABLE |
          ANADIG_PLL_SYS_PLL2_PFD_PFD3_STABLE);

  if (g_initial_clkconfig.sys_pll2.pfd0 > 0)
    {
      reg &= ~ANADIG_PLL_SYS_PLL2_PFD_PFD0_DIV1_CLKGATE;
      reg |= ANADIG_PLL_SYS_PLL2_PFD_PFD0_FRAC(
          g_initial_clkconfig.sys_pll2.pfd0);
    }

  if (g_initial_clkconfig.sys_pll2.pfd1 > 0)
    {
      reg &= ~ANADIG_PLL_SYS_PLL2_PFD_PFD1_DIV1_CLKGATE;
      reg |= ANADIG_PLL_SYS_PLL2_PFD_PFD1_FRAC(
          g_initial_clkconfig.sys_pll2.pfd1);
    }

  if (g_initial_clkconfig.sys_pll2.pfd2 > 0)
    {
      reg &= ~ANADIG_PLL_SYS_PLL2_PFD_PFD2_DIV1_CLKGATE;
      reg |= ANADIG_PLL_SYS_PLL2_PFD_PFD2_FRAC(
          g_initial_clkconfig.sys_pll2.pfd2);
    }

  if (g_initial_clkconfig.sys_pll2.pfd3 > 0)
    {
      reg &= ~ANADIG_PLL_SYS_PLL2_PFD_PFD3_DIV1_CLKGATE;
      reg |= ANADIG_PLL_SYS_PLL2_PFD_PFD3_FRAC(
          g_initial_clkconfig.sys_pll2.pfd3);
    }

  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_PFD);

  reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_UPDATE);

  if (g_initial_clkconfig.sys_pll2.pfd0 > 0)
    {
      reg ^= ANADIG_PLL_SYS_PLL2_UPDATE_PFD0_UPDATE;
    }

  if (g_initial_clkconfig.sys_pll2.pfd1 > 0)
    {
      reg ^= ANADIG_PLL_SYS_PLL2_UPDATE_PFD1_UPDATE;
    }

  if (g_initial_clkconfig.sys_pll2.pfd2 > 0)
    {
      reg ^= ANADIG_PLL_SYS_PLL2_UPDATE_PFD2_UPDATE;
    }

  if (g_initial_clkconfig.sys_pll2.pfd3 > 0)
    {
      reg ^= ANADIG_PLL_SYS_PLL2_UPDATE_PFD3_UPDATE;
    }

  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_UPDATE);

  /* Wait for stablizing */

  reg = 0;

  if (g_initial_clkconfig.sys_pll2.pfd0 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL2_PFD_PFD0_STABLE;
    }

  if (g_initial_clkconfig.sys_pll2.pfd1 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL2_PFD_PFD1_STABLE;
    }

  if (g_initial_clkconfig.sys_pll2.pfd2 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL2_PFD_PFD2_STABLE;
    }

  if (g_initial_clkconfig.sys_pll2.pfd3 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL2_PFD_PFD3_STABLE;
    }

  while ((getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_CTRL) &
        (ANADIG_PLL_SYS_PLL2_PFD_PFD0_STABLE |
         ANADIG_PLL_SYS_PLL2_PFD_PFD1_STABLE |
         ANADIG_PLL_SYS_PLL2_PFD_PFD2_STABLE |
         ANADIG_PLL_SYS_PLL2_PFD_PFD3_STABLE)) == reg);
}

static void imxrt_pll3_init(void)
{
  uint32_t reg;

  reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_CTRL);

  if ((reg & ANADIG_PLL_SYS_PLL3_CTRL_POWERUP) != 0)
    {
      /* PLL Already configured no need to reconfigure */

      if ((reg & ANADIG_PLL_SYS_PLL3_CTRL_ENABLE_CLK) == 0)
        {
          reg |= ANADIG_PLL_SYS_PLL3_CTRL_ENABLE_CLK;
        }

      if ((reg & ANADIG_PLL_SYS_PLL3_CTRL_SYS_PLL3_GATE) != 0)
        {
          reg &= ~ANADIG_PLL_SYS_PLL3_CTRL_SYS_PLL3_GATE;
        }

      putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL3_CTRL);
      return;
    }

  imxrt_pmu_enable_pll_ldo();

  reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_PFD);

  reg |= (ANADIG_PLL_SYS_PLL3_PFD_PFD0_DIV1_CLKGATE |
          ANADIG_PLL_SYS_PLL3_PFD_PFD1_DIV1_CLKGATE |
          ANADIG_PLL_SYS_PLL3_PFD_PFD2_DIV1_CLKGATE |
          ANADIG_PLL_SYS_PLL3_PFD_PFD3_DIV1_CLKGATE);

  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL3_PFD);

  putreg32(ANADIG_PLL_SYS_PLL3_CTRL_PLL_REG_EN |
           ANADIG_PLL_SYS_PLL3_CTRL_SYS_PLL3_GATE,
           IMXRT_ANADIG_PLL_SYS_PLL3_CTRL);

  /* Wait 30 usec */

  up_mdelay(1);

  reg  = getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_CTRL);
  reg |= ANADIG_PLL_SYS_PLL3_CTRL_POWERUP |
         ANADIG_PLL_SYS_PLL3_CTRL_HOLD_RING_OFF;
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL2_CTRL);

  /* Wait 30 usec */

  up_mdelay(1);

  reg &= ~ANADIG_PLL_SYS_PLL3_CTRL_HOLD_RING_OFF;
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL3_CTRL);

  while ((getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_CTRL) &
        ANADIG_PLL_SYS_PLL3_CTRL_SYS_PLL3_STABLE) !=
        ANADIG_PLL_SYS_PLL3_CTRL_SYS_PLL3_STABLE);

  reg |= ANADIG_PLL_SYS_PLL3_CTRL_ENABLE_CLK;
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL3_CTRL);

  reg &= ~ANADIG_PLL_SYS_PLL3_CTRL_SYS_PLL3_GATE;
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL3_CTRL);
}

static void imxrt_pll3_pfd(void)
{
  uint32_t reg;

  reg  = getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_PFD);
  reg &= ~(ANADIG_PLL_SYS_PLL3_PFD_PFD0_DIV1_CLKGATE |
           ANADIG_PLL_SYS_PLL3_PFD_PFD1_DIV1_CLKGATE |
           ANADIG_PLL_SYS_PLL3_PFD_PFD2_DIV1_CLKGATE |
           ANADIG_PLL_SYS_PLL3_PFD_PFD3_DIV1_CLKGATE);
  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL3_PFD);

  /* Configure PFD dividers */

  reg  = getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_PFD);
  reg &= (ANADIG_PLL_SYS_PLL3_PFD_PFD0_STABLE |
          ANADIG_PLL_SYS_PLL3_PFD_PFD1_STABLE |
          ANADIG_PLL_SYS_PLL3_PFD_PFD2_STABLE |
          ANADIG_PLL_SYS_PLL3_PFD_PFD3_STABLE);

  if (g_initial_clkconfig.sys_pll3.pfd0 > 0)
    {
      reg &= ~ANADIG_PLL_SYS_PLL3_PFD_PFD0_DIV1_CLKGATE;
      reg |= ANADIG_PLL_SYS_PLL3_PFD_PFD0_FRAC(
          g_initial_clkconfig.sys_pll3.pfd0);
    }

  if (g_initial_clkconfig.sys_pll3.pfd1 > 0)
    {
      reg &= ~ANADIG_PLL_SYS_PLL3_PFD_PFD1_DIV1_CLKGATE;
      reg |= ANADIG_PLL_SYS_PLL3_PFD_PFD1_FRAC(
          g_initial_clkconfig.sys_pll3.pfd1);
    }

  if (g_initial_clkconfig.sys_pll3.pfd2 > 0)
    {
      reg &= ~ANADIG_PLL_SYS_PLL3_PFD_PFD2_DIV1_CLKGATE;
      reg |= ANADIG_PLL_SYS_PLL3_PFD_PFD2_FRAC(
          g_initial_clkconfig.sys_pll3.pfd2);
    }

  if (g_initial_clkconfig.sys_pll3.pfd3 > 0)
    {
      reg &= ~ANADIG_PLL_SYS_PLL3_PFD_PFD3_DIV1_CLKGATE;
      reg |= ANADIG_PLL_SYS_PLL3_PFD_PFD3_FRAC(
          g_initial_clkconfig.sys_pll3.pfd3);
    }

  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL3_PFD);

  reg = getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_UPDATE);

  if (g_initial_clkconfig.sys_pll3.pfd0 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL3_UPDATE_PFD0_UPDATE;
    }

  if (g_initial_clkconfig.sys_pll3.pfd1 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL3_UPDATE_PFD1_UPDATE;
    }

  if (g_initial_clkconfig.sys_pll3.pfd2 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL3_UPDATE_PFD2_UPDATE;
    }

  if (g_initial_clkconfig.sys_pll3.pfd3 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL3_UPDATE_PFD3_UPDATE;
    }

  putreg32(reg, IMXRT_ANADIG_PLL_SYS_PLL3_UPDATE);

  /* Wait for stablizing */

  reg = 0;

  if (g_initial_clkconfig.sys_pll3.pfd0 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL3_PFD_PFD0_STABLE;
    }

  if (g_initial_clkconfig.sys_pll3.pfd1 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL3_PFD_PFD1_STABLE;
    }

  if (g_initial_clkconfig.sys_pll3.pfd2 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL3_PFD_PFD2_STABLE;
    }

  if (g_initial_clkconfig.sys_pll3.pfd3 > 0)
    {
      reg |= ANADIG_PLL_SYS_PLL3_PFD_PFD3_STABLE;
    }

  while ((getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_CTRL) &
        (ANADIG_PLL_SYS_PLL3_PFD_PFD0_STABLE |
         ANADIG_PLL_SYS_PLL3_PFD_PFD1_STABLE |
         ANADIG_PLL_SYS_PLL3_PFD_PFD2_STABLE |
         ANADIG_PLL_SYS_PLL3_PFD_PFD3_STABLE)) == reg);
}

static void imxrt_pllsetup(void)
{
  /* if DCD is used, please make sure the clock source of
   * SEMC is not changed in the following PLL/PFD configuration code.
   */

  /* Setup ARM PLL with dividers from g_initial_clkconfig.arm_pll */

  imxrt_armpllsetup();

  imxrt_pll1_init();  /* Enable/Disable PLL1 */
  imxrt_pll2_init();  /* Init PLL2 @ 518Mhz */
  imxrt_pll2_pfd();   /* Set PLL2 Clock dividers */
  imxrt_pll3_init();  /* Init Sys Pll3 @ 480 Mhz */
  imxrt_pll3_pfd();   /* Set PLL3 Clock dividers */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_clockconfig
 *
 * Description:
 *   Called to initialize the i.MXRT.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void imxrt_clockconfig()
{
  uint32_t reg;

  /* Set Soc VDD and wait for it to stablise */

  imxrt_pmu_vdd1p0_buckmode_targetvoltage(dcdc_1p0bucktarget1p15v);

  /* FUSE FBB bit so that FBB has to be enabled */

  if (((getreg32(IMXRT_OCOTP_FUSE(7)) & 0x10) >> 4) != 1)
    {
      imxrt_pmu_enable_body_bias_fbb_cm7(1);
    }
  else
    {
      imxrt_pmu_enable_body_bias_fbb_cm7(0);
    }

  imxrt_pmu_lpsr_ana_ldo_bypassmode(1);
  imxrt_pmu_lpsr_dig_ldo_bypassmode(1);

#ifdef GPIO_CLOCKOUT
  /* setup clkout pinmux */

  imxrt_config_gpio(GPIO_CCM_CLKO1_1);
  imxrt_config_gpio(GPIO_CCM_CLKO2_1);
#endif

  /* Config OSC */

  imxrt_oscsetup();

  /* OK, now nothing is depending on us, configure the PLLs */

  imxrt_pllsetup();

  /* Set all clock Roots based on boardconfig */

  for (int i = 0; i < IMXRT_CCM_CR_COUNT; i++)
    {
      reg = getreg32(IMXRT_CCM_CR_CTRL(i));

      reg &= ~(CCM_CR_CTRL_MUX_MASK |
               CCM_CR_CTRL_DIV_MASK | CCM_CR_CTRL_OFF);

      reg |= CCM_CR_CTRL_MUX_SRCSEL(
          g_initial_clkconfig.ccm.clock_root[i].mux);
      reg |= CCM_CR_CTRL_DIV(
          g_initial_clkconfig.ccm.clock_root[i].div);

      if (!g_initial_clkconfig.ccm.clock_root[i].enable)
        {
          reg |= CCM_CR_CTRL_OFF;
        }

      putreg32(reg, IMXRT_CCM_CR_CTRL(i));
    }

  /* Keep core clock ungated during WFI */

  putreg32(0x1, IMXRT_CCM_GPR_PR_SET(1));

  putreg32(0x1, IMXRT_CCM_GPR_PR_SET(28));

  /* Keep the system clock running so SYSTICK can wake up the system from
   * wfi.
   */

  modifyreg32(IMXRT_GPC_CPU_MODE_CTRL_0_CM_MODE_CTRL,
              GPC_CPU_MODE_CTRL_CM_MODE_CTRL_CPU_MODE_TARGET_MASK,
              GPC_CPU_MODE_CTRL_CM_MODE_CTRL_CPU_MODE_TARGET(
              GPC_CPU_MODE_RUN_MODE));

  modifyreg32(IMXRT_GPC_CPU_MODE_CTRL_0_CM_MISC,
              GPC_CPU_MODE_CTRL_CM_MISC_SLEEP_HOLD_EN,
              0);
}

int imxrt_get_pll(enum ccm_clock_name clkname, uint32_t *frequency)
{
  uint32_t loop_div;
  uint32_t post_div;

  switch (clkname)
    {
      case PLL_ARM_CLK:
          loop_div = ((getreg32(IMXRT_ANADIG_PLL_ARM_PLL_CTRL) &
                      ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT_MASK)
                      >> ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT_SHIFT);
          post_div = ((getreg32(IMXRT_ANADIG_PLL_ARM_PLL_CTRL) &
                      ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_MASK)
                      >> ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_SHIFT);
          post_div = (1 << (post_div + 1));
          *frequency = ((BOARD_XTAL_FREQUENCY / (2 * post_div)) * loop_div);
          break;

      case SYS_PLL1:
          *frequency = SYS_PLL1_FREQ;
          break;

      case SYS_PLL2:
          *frequency = SYS_PLL2_FREQ;
          break;

      case SYS_PLL3:
          *frequency = SYS_PLL3_FREQ;
          break;

      case AUDIO_PLL_CLK:
      case VIDEO_PLL_CLK:
          return -ENOSYS;

      default:

          /* Wrong input parameter pll or not implemented */

          DEBUGASSERT(false);
          return -ENODEV;
          break;
    }

    return OK;
}

int imxrt_get_pll_pfd(enum ccm_clock_name clkname, uint32_t pfd,
                      uint32_t *frequency)
{
  uint32_t pllfreq;
  uint32_t frac = 1;

  if (imxrt_get_pll(clkname, &pllfreq))
    {
      switch (clkname)
        {
          case SYS_PLL2:
            frac = (getreg32(IMXRT_ANADIG_PLL_SYS_PLL2_PFD) &
                    (ANADIG_PLL_SYS_PLL2_PFD_PFD0_FRAC_MASK
                     << (8 * pfd)));
            break;

          case SYS_PLL3:
            frac = (getreg32(IMXRT_ANADIG_PLL_SYS_PLL3_PFD) &
                    (ANADIG_PLL_SYS_PLL3_PFD_PFD0_FRAC_MASK
                     << (8 * pfd)));
            break;

          default:

            /* Wrong input parameter pll or not implemented */

            DEBUGASSERT(false);
            break;
        }

       frac = frac >> (8 * pfd);

       if (frac != 0)
         {
           *frequency = (pllfreq / frac * 18);
         }
       else
         {
           *frequency = 0;
         }

       return OK;
    }
  else
    {
      return -ENODEV;
    }
}

/****************************************************************************
 * Name: imxrt_get_clock
 *
 * Description:
 *   This function returns the clock frequency of the specified functional
 *   clock.
 *
 * Input Parameters:
 *   clkname   - Identifies the clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int imxrt_get_clock(enum ccm_clock_name clkname, uint32_t *frequency)
{
    switch (clkname)
      {
        case OSC_RC_16M:
            *frequency = 16000000U;
            break;

        case OSC_RC_48M:
            *frequency = 48000000U;
            break;

        case OSC_RC_48M_DIV2:
            *frequency = 24000000U;
            break;

        case OSC_RC_400M:
            *frequency = 400000000U;
            break;

        case OSC_24M:
            *frequency = 24000000U;
            break;

        case PLL_ARM_CLK:
            return imxrt_get_pll(PLL_ARM_CLK, frequency);
            break;

        case SYS_PLL2:
            return imxrt_get_pll(SYS_PLL2, frequency);
            break;

        case SYS_PLL2_PFD0:
            return imxrt_get_pll_pfd(SYS_PLL2, 0, frequency);
            break;

        case SYS_PLL2_PFD1:
            return imxrt_get_pll_pfd(SYS_PLL2, 1, frequency);
            break;

        case SYS_PLL2_PFD2:
            return imxrt_get_pll_pfd(SYS_PLL2, 2, frequency);
            break;

        case SYS_PLL2_PFD3:
            return imxrt_get_pll_pfd(SYS_PLL2, 3, frequency);
            break;

        case SYS_PLL3:
            return imxrt_get_pll(SYS_PLL3, frequency);
            break;

        case SYS_PLL3_DIV2:
            imxrt_get_pll(SYS_PLL3, frequency);
            *frequency = *frequency / 2;
            break;

        case SYS_PLL3_PFD0:
            return imxrt_get_pll_pfd(SYS_PLL3, 0, frequency);
            break;

        case SYS_PLL3_PFD1:
            return imxrt_get_pll_pfd(SYS_PLL3, 1, frequency);
            break;

        case SYS_PLL3_PFD2:
            return imxrt_get_pll_pfd(SYS_PLL3, 2, frequency);
            break;

        case SYS_PLL3_PFD3:
            return imxrt_get_pll_pfd(SYS_PLL3, 3, frequency);
            break;

        case SYS_PLL1:
            return imxrt_get_pll(SYS_PLL1, frequency);
            break;

        case SYS_PLL1_DIV2:
            imxrt_get_pll(SYS_PLL1, frequency);
            *frequency = *frequency / 2;
            break;

        case SYS_PLL1_DIV5:
            imxrt_get_pll(SYS_PLL1, frequency);
            *frequency = *frequency / 5;
            break;

        case AUDIO_PLL_CLK:
            return imxrt_get_pll(AUDIO_PLL_CLK, frequency);
            break;
        case VIDEO_PLL_CLK:
            return imxrt_get_pll(VIDEO_PLL_CLK, frequency);
            break;

        default:

            /* Wrong input parameter name. */

            return -ENODEV;
            break;
    }

    return OK;
}

/****************************************************************************
 * Name: imxrt_get_rootclock
 *
 * Description:
 *   This function returns the clock frequency of the specified root
 *   functional clock.
 *
 * Input Parameters:
 *   clkroot   - Identifies the peripheral clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int imxrt_get_rootclock(uint32_t clkroot, uint32_t *frequency)
{
  uint32_t reg;
  uint32_t div;
  ccm_clock_name clk_name;

  if (clkroot <= IMXRT_CCM_CR_COUNT)
    {
      clk_name = ((g_initial_clkconfig.ccm.clock_root[clkroot].mux
                 & CLOCK_NAME_MASK) >> CLOCK_NAME_SHIFT);
      reg = getreg32(IMXRT_CCM_CR_CTRL(clkroot));

      if ((reg & CCM_CR_CTRL_OFF) == CCM_CR_CTRL_OFF)
        {
          *frequency = 0;
        }
      else
        {
          imxrt_get_clock(clk_name, frequency);
          div = ((reg & CCM_CR_CTRL_DIV_MASK) >> CCM_CR_CTRL_DIV_SHIFT) + 1;
          *frequency = *frequency / div;
        }

      return OK;
    }

  return -ENODEV;
}
