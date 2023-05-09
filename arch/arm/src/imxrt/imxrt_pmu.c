/****************************************************************************
 * arch/arm/src/imxrt/imxrt_pmu.c
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
#include "imxrt_pmu.h"
#include "hardware/imxrt_memorymap.h"
#include "hardware/imxrt_iomuxc.h"
#include "hardware/rt117x/imxrt117x_pmu.h"
#include "hardware/rt117x/imxrt117x_anadig.h"
#include "hardware/rt117x/imxrt117x_dcdc.h"
#include "hardware/rt117x/imxrt117x_snvs.h"

#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void imxrt_vddsoc_ai_write(uint8_t addr, uint32_t wdata)
{
  uint32_t reg;

  putreg32(~ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AIRWB,
           IMXRT_ANADIG_MISC_VDDSOC_AI_CTRL);

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC_AI_CTRL);
  reg &= ~ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AI_ADDR_MASK;
  reg |= ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AI_ADDR(addr);

  /* Write wdata to addr */

  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC_AI_CTRL);
  putreg32(wdata, IMXRT_ANADIG_MISC_VDDSOC_AI_WDATA);

  /* Toggle */

  reg  = getreg32(IMXRT_ANADIG_PMU_PMU_LDO_PLL);
  reg ^= ANADIG_PMU_PMU_LDO_PLL_LDO_PLL_AI_TOGGLE;
  putreg32(reg, IMXRT_ANADIG_PMU_PMU_LDO_PLL);
}

uint32_t imxrt_vddsoc_ai_read(uint8_t addr)
{
  uint32_t reg;

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC_AI_CTRL);
  reg &= ~ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AIRWB;
  reg |= ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AIRWB;
  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC_AI_CTRL);

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC_AI_CTRL);
  reg &= ~ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AI_ADDR_MASK;
  reg |= ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AI_ADDR(addr);
  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC_AI_CTRL);

  /* Toggle */

  reg  = getreg32(IMXRT_ANADIG_PMU_PMU_LDO_PLL);
  reg ^= ANADIG_PMU_PMU_LDO_PLL_LDO_PLL_AI_TOGGLE;
  putreg32(reg, IMXRT_ANADIG_PMU_PMU_LDO_PLL);

  /* Read data */

  return getreg32(IMXRT_ANADIG_MISC_VDDSOC_AI_RDATA);
}

void imxrt_pll1g_ai_write(uint8_t addr, uint32_t wdata)
{
  uint32_t toggle_pre;
  uint32_t reg;

  toggle_pre = getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  toggle_pre &=
    ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_DONE_1G;

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  reg &= ~ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIRWB_1G;
  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  reg &= ~ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIADDR_1G_MASK;
  reg |= ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIADDR_1G(addr);

  /* Write wdata to addr */

  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  putreg32(wdata, IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_WDATA_1G);

  /* Toggle */

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  reg ^= ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_1G;
  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);

  while ((getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G) &
          ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_DONE_1G)
          == toggle_pre);
}

uint32_t imxrt_pll1g_ai_read(uint8_t addr)
{
  uint32_t toggle_pre;
  uint32_t reg;

  toggle_pre = getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  toggle_pre &=
    ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_DONE_1G;

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  reg &= ~ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIRWB_1G;
  reg |= ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIRWB_1G;
  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  reg &= ~ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIADDR_1G_MASK;
  reg |= ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIADDR_1G(addr);

  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);

  /* Toggle */

  reg  = getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);
  reg ^= ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_1G;
  putreg32(reg, IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G);

  while ((getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G) &
          ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_DONE_1G)
          == toggle_pre);

  return getreg32(IMXRT_ANADIG_MISC_VDDSOC2PLL_AI_RDATA_1G);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imxrt_modify_pll1g(uint8_t addr, uint32_t set, uint32_t clr)
{
  if (set > 0)
    {
      imxrt_pll1g_ai_write(addr + IMXRT_ANADIG_MISC_ADDR_SET_OFFSET, set);
    }

  if (clr > 0)
    {
      imxrt_pll1g_ai_write(addr + IMXRT_ANADIG_MISC_ADDR_CLR_OFFSET, clr);
    }
}

uint32_t imxrt_get_pll1g(uint8_t addr)
{
  return imxrt_pll1g_ai_read(addr);
}

void imxrt_pmu_enable_pll_ldo(void)
{
  uint32_t reg;

  reg = imxrt_vddsoc_ai_read(IMXRT_ANADIG_MISC_ADDR_PHY_LDO_CTRL0);

  if (reg !=
        (AI_PHY_LDO_CTRL0_OUTPUT_TRG(0x10) |
        AI_PHY_LDO_CTRL0_LINREG_EN_MASK |
        AI_PHY_LDO_CTRL0_LIMIT_EN_MASK))
    {
      imxrt_vddsoc_ai_write(IMXRT_ANADIG_MISC_ADDR_PHY_LDO_CTRL0,
            (AI_PHY_LDO_CTRL0_OUTPUT_TRG(0x10) |
            AI_PHY_LDO_CTRL0_LINREG_EN_MASK |
            AI_PHY_LDO_CTRL0_LIMIT_EN_MASK));

      /* Wait 1 usec */

      up_mdelay(1);

      /* Enable Voltage Reference for PLLs
       * before those PLLs were enabled.
       */

      reg  = getreg32(IMXRT_ANADIG_PMU_PMU_REF_CTRL);
      reg |= ANADIG_PMU_PMU_REF_CTRL_EN_PLL_VOL_REF_BUFFER;
      putreg32(reg, IMXRT_ANADIG_PMU_PMU_REF_CTRL);
    }
}

void imxrt_pmu_enable_body_bias_fbb_cm7(uint32_t enable)
{
  uint32_t reg;

  if (enable)
    {
      reg  = getreg32(IMXRT_ANADIG_PMU_PMU_BIAS_CTRL);
      reg &= ~ANADIG_PMU_PMU_BIAS_CTRL_WB_CFG_1P8_WELL_SELECT;
      reg |= ANADIG_PMU_PMU_BIAS_CTRL_WB_CFG_1P8_VOLTAGE_THRESHOLD_MASK;
      putreg32(reg, IMXRT_ANADIG_PMU_PMU_BIAS_CTRL);

      reg  = getreg32(IMXRT_ANADIG_PMU_PMU_BIAS_CTRL2);
      reg &= ANADIG_PMU_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8_MASK;
      reg |= ANADIG_PMU_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8(1) |
      ANADIG_PMU_PMU_BIAS_CTRL2_WB_EN;
      putreg32(reg, IMXRT_ANADIG_PMU_PMU_BIAS_CTRL2);

      while ((getreg32(IMXRT_ANADIG_PMU_PMU_BIAS_CTRL2) &
             ANADIG_PMU_PMU_BIAS_CTRL2_WB_OK)
            != ANADIG_PMU_PMU_BIAS_CTRL2_WB_OK);
    }
  else
    {
      reg  = getreg32(IMXRT_ANADIG_PMU_PMU_BIAS_CTRL2);
      reg &=  ~(ANADIG_PMU_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8_MASK |
      ANADIG_PMU_PMU_BIAS_CTRL2_WB_EN);
    }
}

void imxrt_pmu_vdd1p0_buckmode_targetvoltage(
    dcdc_1p0_buck_mode_target_voltage voltage)
{
  uint32_t reg;

  reg  = getreg32(IMXRT_DCDC_REG3);
  reg &= ~DCDC_REG3_VDD1P0CTRL_DISABLE_STEP;
  putreg32(reg, IMXRT_DCDC_REG3);

  reg  = getreg32(IMXRT_DCDC_CTRL1);
  reg &= ~DCDC_CTRL1_VDD1P0CTRL_TRG_MASK;
  reg |= DCDC_CTRL1_VDD1P0CTRL_TRG(voltage);
  putreg32(reg, IMXRT_DCDC_CTRL1);

  while ((getreg32(IMXRT_DCDC_REG0) & DCDC_REG0_STS_DC_OK)
      != DCDC_REG0_STS_DC_OK);
}

void imxrt_pmu_lpsr_ana_ldo_bypassmode(uint32_t enable)
{
  uint32_t reg;

  if (enable == 0)
    {
      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
      reg &= ~(ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_REG_LP_EN |
      ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_REG_DISABLE);
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
      reg &= ~(ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_BYPASS_MODE_EN);
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
      reg &= ~(ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_TRACK_MODE_EN);
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
    }
  else
    {
      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
      reg &= ~(ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_REG_LP_EN);
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
      reg |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_TRACK_MODE_EN;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
      reg |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_BYPASS_MODE_EN;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
      reg |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_REG_DISABLE;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA);
    }
}

void imxrt_pmu_lpsr_dig_ldo_bypassmode(uint32_t enable)
{
  uint32_t reg;

  if (enable)
    {
      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);
      reg |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_TRACKING_MODE;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);
      reg |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_BYPASS_MODE;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);
      reg &= ~ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_REG_EN;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);
    }
    else
    {
      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);
      reg |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_REG_EN;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);
      reg &= ~ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_BYPASS_MODE;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);

      up_udelay(1000);

      reg  = getreg32(IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);
      reg &= ~ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_TRACKING_MODE;
      putreg32(reg, IMXRT_ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG);
    }
}
