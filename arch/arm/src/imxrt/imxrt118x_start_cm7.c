/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_start_cm7.c
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
#include <inttypes.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "imxrt118x_ele.h"
#include "imxrt118x_start_cm7.h"
#include "hardware/imxrt_ccm.h"
#include "hardware/rt117x/imxrt117x_dcdc.h"
#include "hardware/rt118x/imxrt118x_anadig.h"
#include "hardware/rt118x/imxrt118x_blkctrl.h"
#include "hardware/rt118x/imxrt118x_gpc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CCM_M7_ROOT_CONFIG          (CCM_CR_CTRL_MUX_SRCSEL(2) | \
                                     CCM_CR_CTRL_DIV(1))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt118x_pmu_enable_body_bias_fbb_cm7
 *
 * Description:
 *   Enable forward body bias (FBB) on the LVT core logic.  FBB is required
 *   for the M7 to reach its 800 MHz HSRUN (OverDrive) frequency
 *   (IMXRT1180RM Rev. 10 sec. 2.1.3).
 *
 *   Follows the "PMU Well Bias Enable Sequence" of RM sec. 24.3.2.3.3.2.
 *   FBB is implemented in the Cortex-M7 platform only (RM sec. 19.3.3),
 *   and the second step turns on the CM7 well-bias power switch, so this
 *   belongs to the M7 release path rather than to generic clock setup.
 *   The RM requires VDD_SOC_IN and VDD_AON_ANA to be settled beforehand;
 *   imxrt_clockconfig() has already raised VDD1P0 to 1.125 V by this
 *   point.
 *
 ****************************************************************************/

static int imxrt118x_pmu_enable_body_bias_fbb_cm7(void)
{
  int timeout;
  uint32_t regval;

  /* Step 1/2: configure the well bias. */

  regval  = getreg32(IMXRT_ANADIG_PMU_BIAS_CTRL);
  regval &= ~(ANADIG_PMU_BIAS_CTRL_WB_NW_LVL_1P8_MASK |
              ANADIG_PMU_BIAS_CTRL_WB_PW_LVL_1P8_MASK |
              ANADIG_PMU_BIAS_CTRL_WB_CFG_1P8_MASK);
  regval |= ANADIG_PMU_BIAS_CTRL_WB_CFG_1P8(ANADIG_PMU_WB_CFG_NWELL_FBB |
                                    ANADIG_PMU_WB_CFG_PULLDOWN_EN |
                                    ANADIG_PMU_WB_CFG_AREA(
                                      ANADIG_PMU_WB_AREA_90UA) |
                                    ANADIG_PMU_WB_CFG_ADAPTIVE_DIS |
                                    ANADIG_PMU_WB_CFG_OSC(
                                      ANADIG_PMU_WB_OSC_FULL)) |
            ANADIG_PMU_BIAS_CTRL_WB_NW_LVL_1P8(1) |
            ANADIG_PMU_BIAS_CTRL_WB_PW_LVL_1P8(1);
  putreg32(regval, IMXRT_ANADIG_PMU_BIAS_CTRL);

  /* Step 3: turn on the CM7 FBB switch and the well-bias regulator. */

  regval  = getreg32(IMXRT_ANADIG_PMU_BIAS_CTRL2);
  regval |= ANADIG_PMU_BIAS_CTRL2_WB_EN |
            ANADIG_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8;
  putreg32(regval, IMXRT_ANADIG_PMU_BIAS_CTRL2);

  /* Step 4: wait for the well bias to stabilise. */

  for (timeout = 10000; timeout != 0; timeout--)
    {
      if ((getreg32(IMXRT_ANADIG_PMU_BIAS_CTRL2) &
           ANADIG_PMU_BIAS_CTRL2_WB_OK) != 0)
        {
          return 0;
        }

      up_udelay(1);
    }

  syslog(LOG_ERR, "bootloader: FBB not stable (BIAS_CTRL2=0x%08" PRIx32
         ")\n", getreg32(IMXRT_ANADIG_PMU_BIAS_CTRL2));
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: prepare_m7_power
 ****************************************************************************/

static int prepare_m7_power(void)
{
  uint32_t rootctrl;

  /* Forward body bias is required for HSRUN (IMXRT1180RM Rev. 10
   * Table 128).
   */

  if (imxrt118x_pmu_enable_body_bias_fbb_cm7() < 0)
    {
      return -ETIMEDOUT;
    }

  /* Stop root0 before selecting ARM_PLL / 1.  Keep it stopped until the
   * status register reflects the requested mux and divider, then restart
   * it and wait for the transition to complete.
   */

  putreg32(CCM_CR_CTRL_OFF, IMXRT_CCM_CR_CTRL_SET(CCM_CR_M7));

  while ((getreg32(IMXRT_CCM_CR_STAT0(CCM_CR_M7)) &
          (CCM_CR_STAT0_OFF | CCM_CR_STAT0_CHANGING)) != CCM_CR_STAT0_OFF);

  rootctrl = getreg32(IMXRT_CCM_CR_CTRL(CCM_CR_M7));
  rootctrl &= ~(CCM_CR_CTRL_MUX_MASK | CCM_CR_CTRL_DIV_MASK);
  rootctrl |= CCM_M7_ROOT_CONFIG;
  putreg32(rootctrl, IMXRT_CCM_CR_CTRL(CCM_CR_M7));

  while ((getreg32(IMXRT_CCM_CR_STAT0(CCM_CR_M7)) &
          (CCM_CR_STAT0_MUX_MASK | CCM_CR_STAT0_DIV_MASK |
           CCM_CR_STAT0_OFF | CCM_CR_STAT0_CHANGING)) !=
         (CCM_M7_ROOT_CONFIG | CCM_CR_STAT0_OFF));

  putreg32(CCM_CR_CTRL_OFF, IMXRT_CCM_CR_CTRL_CLR(CCM_CR_M7));

  while (getreg32(IMXRT_CCM_CR_STAT0(CCM_CR_M7)) &
         (CCM_CR_STAT0_OFF | CCM_CR_STAT0_CHANGING));

  putreg32(DCDC_REG3_REG_FBK_SEL(2) |
           DCDC_REG3_DISABLE_IDLE_SKIP |
           DCDC_REG3_DISABLE_PULSE_SKIP,
           IMXRT_DCDC_REG3);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt118x_release_cm7
 ****************************************************************************/

int imxrt118x_release_cm7(uintptr_t vtor)
{
  uint32_t cfg;
  int ret;

  /* Prepare the M7 power domain and core clock root.  ARM_PLL is configured
   * by the M33's imxrt_clockconfig().
   */

  if (prepare_m7_power() < 0)
    {
      return -ETIMEDOUT;
    }

  /* Keep the M7 platform in RUN during WFI so core exceptions such as
   * SysTick can wake it without requiring a GPC wakeup source.  This is
   * done before reset release because the bootloader owns the M7 power
   * setup.
   */

  modifyreg32(IMXRT_GPC_CM7_MODE_CTRL,
              GPC_CM_MODE_CTRL_TARGET_MASK,
              GPC_CM_MODE_CTRL_TARGET(GPC_CM_MODE_TARGET_RUN));
  modifyreg32(IMXRT_GPC_CM_MISC(1),
              GPC_CM_MISC_SLEEP_HOLD_EN, 0);

  /* Program the M7 initial VTOR and force its clocks on. */

  cfg = getreg32(IMXRT_AON_M7_CFG);
  cfg = (cfg & ~AON_M7_CFG_INITVTOR_MASK) |
        AON_M7_CFG_INITVTOR(vtor >> AON_M7_CFG_INITVTOR_SHIFT) |
        AON_M7_CFG_HCLK_FORCE_ON |
        AON_M7_CFG_CORECLK_FORCE_ON;
  putreg32(cfg, IMXRT_AON_M7_CFG);

  /* Release the M7 from reset (write-once). */

  putreg32(getreg32(IMXRT_SRC_SCR) | SRC_SCR_BT_RELEASE_M7, IMXRT_SRC_SCR);

  /* Ask ELE to release the M7. */

  ret = imxrt118x_ele_enable_apc();
  if (ret < 0)
    {
      return ret;
    }

  /* Kick the M7 by toggling its clock around WAIT deassertion.
   */

  putreg32(0, IMXRT_CCM_LPCG_DIR(0));

  putreg32(getreg32(IMXRT_AON_M7_CFG) & ~AON_M7_CFG_WAIT, IMXRT_AON_M7_CFG);

  putreg32(CCM_LPCG_DIR_ON, IMXRT_CCM_LPCG_DIR(0));

  return 0;
}
