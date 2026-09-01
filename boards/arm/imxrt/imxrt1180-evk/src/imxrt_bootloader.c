/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_bootloader.c
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

/* Minimal bootloader application for releasing the Cortex-M7. */

#include <nuttx/config.h>

#include <stdint.h>
#include <inttypes.h>
#include <debug.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "imxrt1180-evk.h"
#include "imxrt118x_ele.h"
#include "hardware/rt118x/imxrt118x_anadig.h"
#include "hardware/imxrt_lpuart.h"
#include "imxrt_clockconfig.h"
#include "hardware/imxrt_ccm.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG16(a)                    (*(volatile uint16_t *)(uintptr_t)(a))

#define BLK_CTRL_S_AONMIX_BASE      0x444f0000u
#define AON_M7_CFG                  (BLK_CTRL_S_AONMIX_BASE + 0x80)
#define AON_M7_CFG_WAIT             (1u << 4)
#define AON_M7_CFG_CORECLK_FORCE_ON (1u << 5)
#define AON_M7_CFG_HCLK_FORCE_ON    (1u << 6)
#define AON_M7_CFG_INITVTOR_MASK    0xffffff80u
#define AON_M7_CFG_INITVTOR_SHIFT   7u

#define AON_M7_CFG_INITVTOR(a)      ((((a) >> 7) << AON_M7_CFG_INITVTOR_SHIFT) \
                                     & AON_M7_CFG_INITVTOR_MASK)

#define SRC_BASE                    0x44460000u
#define SRC_SCR                     (SRC_BASE + 0x10)
#define SRC_SCR_BT_RELEASE_M7       (1u << 0)

/* Base of the Cortex-M7 XIP image. */

#define M7_ENTRY                    0x28080000u

#define M7_INITVTOR                 M7_ENTRY

#define DMA4_BASE                   0x42000000u
#define DMA4_CH0_CH_CSR             (DMA4_BASE + 0x10000)   /* 32b */
#define DMA4_CH0_SADDR              (DMA4_BASE + 0x10020)   /* 32b */
#define DMA4_CH0_SOFF               (DMA4_BASE + 0x10024)   /* 16b */
#define DMA4_CH0_ATTR               (DMA4_BASE + 0x10026)   /* 16b */
#define DMA4_CH0_NBYTES_MLOFFNO     (DMA4_BASE + 0x10028)   /* 32b */
#define DMA4_CH0_DADDR              (DMA4_BASE + 0x10030)   /* 32b */
#define DMA4_CH0_DOFF               (DMA4_BASE + 0x10034)   /* 16b */
#define DMA4_CH0_CITER_ELINKNO      (DMA4_BASE + 0x10036)   /* 16b */
#define DMA4_CH0_CSR                (DMA4_BASE + 0x1003c)   /* 16b */
#define DMA4_CH0_BITER_ELINKNO      (DMA4_BASE + 0x1003e)   /* 16b */

#define DMA4_CH_CSR_DONE            (1u << 30)

#define M7_ITCM_ALIAS               0x303c0000u

#define DMA_SCRATCH                 0x204fc000u

#define CCM_M7_ROOT_MUX_ARM_PLL     CCM_CR_CTRL_MUX_SRCSEL(2)
#define CCM_M7_ROOT_DIV             CCM_CR_CTRL_DIV(1)
#define CCM_M7_ROOT_CONFIG          (CCM_M7_ROOT_MUX_ARM_PLL | \
                                     CCM_M7_ROOT_DIV)

#define CCM_LPCG0_DIRECT            (0x44450000u + 0x8000u)
#define CCM_LPCG_DIRECT_ON          (1u << 0)

/* Well bias NWELL/PWELL voltage level for FBB.  IMXRT1180RM Rev. 10 does
 * not tabulate the WB_NW_LVL_1P8 / WB_PW_LVL_1P8 encodings; this is the
 * level NXP programs for the 800 MHz OverDrive operating point.
 */

#define PMU_BIAS_CTRL_WB_LVL_1P8_VAL 1u

#define DCDC_BASE                   0x44520000u
#define DCDC_REG3                   (DCDC_BASE + 0x0c)
#define DCDC_REG3_REG_FBK_SEL(x)    (((x) << 22) & 0x00c00000u)
#define DCDC_REG3_DISABLE_PULSE_SKIP (1u << 19)
#define DCDC_REG3_DISABLE_IDLE_SKIP (1u << 20)
#define DCDC_REG3_VAL               (DCDC_REG3_REG_FBK_SEL(2) | \
                                     DCDC_REG3_DISABLE_IDLE_SKIP | \
                                     DCDC_REG3_DISABLE_PULSE_SKIP)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enable_forward_body_bias
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

static int enable_forward_body_bias(void)
{
  volatile uint32_t timeout;

  /* Step 1/2: configure the well bias.
   *
   * The RM sequence names only the two bits that select what the bias
   * does (bit 1 = NWELL to supply / LVT CORE, i.e. FBB) and says all
   * other bits are 0.  Taken literally that also selects bits 8-6 = 000,
   * i.e. a charge pump running at osc_freq/128, with bit 5 = 0 leaving
   * the adaptive control free to run slower still.  The pump then cannot
   * charge the M7 well capacitance in reasonable time and WB_OK never
   * asserts.
   *
   * So the functional selection is taken from the RM and the charge pump
   * is additionally configured to run at the full oscillator frequency
   * with the adaptive control disabled, which is the configuration NXP
   * uses and which is confirmed working on this board.
   */

  putreg32(
    ANADIG_PMU_BIAS_CTRL_WB_CFG_1P8(ANADIG_PMU_WB_CFG_NWELL_FBB |
                                    ANADIG_PMU_WB_CFG_PULLDOWN_EN |
                                    ANADIG_PMU_WB_CFG_AREA(
                                      ANADIG_PMU_WB_AREA_90UA) |
                                    ANADIG_PMU_WB_CFG_ADAPTIVE_DIS |
                                    ANADIG_PMU_WB_CFG_OSC(
                                      ANADIG_PMU_WB_OSC_FULL)) |
    ANADIG_PMU_BIAS_CTRL_WB_NW_LVL_1P8(PMU_BIAS_CTRL_WB_LVL_1P8_VAL) |
    ANADIG_PMU_BIAS_CTRL_WB_PW_LVL_1P8(PMU_BIAS_CTRL_WB_LVL_1P8_VAL),
    IMXRT_ANADIG_PMU_BIAS_CTRL);

  /* Step 3: turn on the CM7 FBB switch and the well-bias regulator.  The
   * RM requires both in the same write.
   */

  putreg32(ANADIG_PMU_BIAS_CTRL2_WB_EN |
           ANADIG_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8,
           IMXRT_ANADIG_PMU_BIAS_CTRL2);

  /* Step 4: wait for the well bias to stabilise.  Report rather than spin
   * forever: without FBB the M7 must not be started at 800 MHz, and a
   * silent hang here is indistinguishable from the M7 itself failing.
   */

  timeout = 1000000;
  while (timeout-- &&
         (getreg32(IMXRT_ANADIG_PMU_BIAS_CTRL2) &
          ANADIG_PMU_BIAS_CTRL2_WB_OK) == 0);

  if (timeout == 0)
    {
      syslog(LOG_ERR, "bootloader: FBB not stable (BIAS_CTRL2=0x%08" PRIx32
             ")\n", getreg32(IMXRT_ANADIG_PMU_BIAS_CTRL2));
      return -ETIMEDOUT;
    }

  return 0;
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

  if (enable_forward_body_bias() < 0)
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

  putreg32(DCDC_REG3_VAL, DCDC_REG3);

  return 0;
}

/****************************************************************************
 * Name: dma_fill
 *
 * Description:
 *   Fill a 128 KB TCM window using eDMA4.
 *
 ****************************************************************************/

static int dma_fill(uint32_t target)
{
  volatile uint32_t timeout = 100000;

  putreg32(DMA_SCRATCH, DMA4_CH0_SADDR);
  putreg32(target, DMA4_CH0_DADDR);
  putreg32(0x20000, DMA4_CH0_NBYTES_MLOFFNO);
  REG16(DMA4_CH0_CITER_ELINKNO)  = 0x1;
  REG16(DMA4_CH0_BITER_ELINKNO)  = 0x1;
  REG16(DMA4_CH0_ATTR)           = 0x303;
  REG16(DMA4_CH0_SOFF)           = 0;
  REG16(DMA4_CH0_DOFF)           = 0x8;
  putreg32(0x7, DMA4_CH0_CH_CSR);
  REG16(DMA4_CH0_CSR)            = 0x8;
  REG16(DMA4_CH0_CSR)            = 0x9;
  putreg32(0x40000006, DMA4_CH0_CH_CSR);

  while (timeout-- &&
         (getreg32(DMA4_CH0_CH_CSR) & DMA4_CH_CSR_DONE) == 0);

  if (timeout == 0)
    {
      syslog(LOG_ERR, "bootloader: eDMA4 fill of 0x%08" PRIx32 " timed out "
             "(CH_CSR=0x%08" PRIx32 ")\n", target,
             getreg32(DMA4_CH0_CH_CSR));
      return -ETIMEDOUT;
    }

  putreg32(DMA4_CH_CSR_DONE, DMA4_CH0_CH_CSR);
  return 0;
}

/****************************************************************************
 * Name: prepare_edma4
 *
 * Description:
 *   Enable the eDMA4 clock.
 *
 ****************************************************************************/

static void prepare_edma4(void)
{
  imxrt_ccm_configure_root_clock(CCM_CR_WAKEUP_AXI, SYS_PLL3_OUT, 2);
  imxrt_ccm_gate_on(CCM_LPCG_EDMA4, true);
}

/****************************************************************************
 * Name: init_cm7_tcm
 *
 * Description:
 *   Clear the M7 TCM aliases.
 *
 ****************************************************************************/

static int init_cm7_tcm(void)
{
  putreg32(0, DMA_SCRATCH);
  putreg32(0, DMA_SCRATCH + 4);

  return dma_fill(0x303c0000u) | dma_fill(0x303e0000u) |
         dma_fill(0x30400000u) | dma_fill(0x30420000u);
}

/****************************************************************************
 * Name: stage_cm7_kickoff_vector
 ****************************************************************************/

static void stage_cm7_kickoff_vector(void)
{
  putreg32(getreg32(M7_ENTRY + 0), M7_ITCM_ALIAS + 0);
  putreg32(getreg32(M7_ENTRY + 4), M7_ITCM_ALIAS + 4);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bootloader_main
 *
 * Description:
 *   Release the Cortex-M7 and then exit.
 *
 ****************************************************************************/

int bootloader_main(int argc, char *argv[])
{
  uint32_t cfg;
  int ret;

  /* Prepare the M7 power domain and core clock root.  ARM_PLL is configured
   * by the M33's imxrt_clockconfig().
   */

  if (prepare_m7_power() < 0)
    {
      syslog(LOG_ERR, "bootloader: not releasing M7\n");
      return EXIT_FAILURE;
    }

  /* Program the M7 initial VTOR and force its clocks on. */

  cfg = getreg32(AON_M7_CFG);
  cfg = (cfg & ~AON_M7_CFG_INITVTOR_MASK) |
        AON_M7_CFG_INITVTOR(M7_INITVTOR) |
        AON_M7_CFG_HCLK_FORCE_ON |
        AON_M7_CFG_CORECLK_FORCE_ON;
  putreg32(cfg, AON_M7_CFG);

  /* Clock eDMA4 before clearing TCM. */

  prepare_edma4();

  /* Release the M7 from reset (write-once). */

  putreg32(getreg32(SRC_SCR) | SRC_SCR_BT_RELEASE_M7, SRC_SCR);

  /* Clear TCM and stage the kickoff vector before clearing WAIT. */

  if (init_cm7_tcm() < 0)
    {
      syslog(LOG_ERR, "bootloader: TCM scrub failed, not releasing M7\n");
      return EXIT_FAILURE;
    }

  stage_cm7_kickoff_vector();

  /* Ask ELE to release the M7. */

  ret = imxrt118x_ele_enable_apc();
  if (ret < 0)
    {
      syslog(LOG_ERR, "bootloader: ELE_ENABLE_APC failed (%d)\n", ret);
      return EXIT_FAILURE;
    }

  /* Kick the M7 by toggling its clock around WAIT deassertion.
   */

  putreg32(0, CCM_LPCG0_DIRECT);

  putreg32(getreg32(AON_M7_CFG) & ~AON_M7_CFG_WAIT, AON_M7_CFG);

  putreg32(CCM_LPCG_DIRECT_ON, CCM_LPCG0_DIRECT);

  return 0;
}
