/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_anadig.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ANADIG_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ANADIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ANADIG PLL registers (IMXRT1180RM Ch. 41).  Offsets are relative to
 * IMXRT_ANADIG_PLL_BASE (0x4448_0000).  The M7 core clock is fed from
 * ARM_PLL; the CM33 side and most peripherals are fed from SYS_PLL2/3.
 */

#define IMXRT_ANADIG_PLL_ARM_CTRL        (IMXRT_ANADIG_PLL_BASE + 0x4000)
#define IMXRT_ANADIG_PLL_SYS3_CTRL       (IMXRT_ANADIG_PLL_BASE + 0x4010)
#define IMXRT_ANADIG_PLL_SYS3_UPDATE     (IMXRT_ANADIG_PLL_BASE + 0x4020)
#define IMXRT_ANADIG_PLL_SYS3_PFD        (IMXRT_ANADIG_PLL_BASE + 0x4030)
#define IMXRT_ANADIG_PLL_SYS2_CTRL       (IMXRT_ANADIG_PLL_BASE + 0x4040)
#define IMXRT_ANADIG_PLL_SYS2_UPDATE     (IMXRT_ANADIG_PLL_BASE + 0x4050)
#define IMXRT_ANADIG_PLL_SYS2_SS         (IMXRT_ANADIG_PLL_BASE + 0x4060)
#define IMXRT_ANADIG_PLL_SYS2_PFD        (IMXRT_ANADIG_PLL_BASE + 0x4070)
#define IMXRT_ANADIG_PLL_SYS2_MFN        (IMXRT_ANADIG_PLL_BASE + 0x4080)
#define IMXRT_ANADIG_PLL_SYS2_MFI        (IMXRT_ANADIG_PLL_BASE + 0x4090)
#define IMXRT_ANADIG_PLL_SYS2_MFD        (IMXRT_ANADIG_PLL_BASE + 0x40a0)
#define IMXRT_ANADIG_PLL_SYS1_CTRL       (IMXRT_ANADIG_PLL_BASE + 0x4100)
#define IMXRT_ANADIG_PLL_AUDIO_CTRL      (IMXRT_ANADIG_PLL_BASE + 0x4200)
#define IMXRT_ANADIG_PLL_VIDEO_CTRL      (IMXRT_ANADIG_PLL_BASE + 0x4300)

/* ARM_PLL_CTRL bit fields (IMXRT1180RM Rev. 10 sec. 24.5.1.1, reset
 * value 4000_00A6h).
 *
 *   Fout = 24 MHz * DIV_SELECT / (2 * post_div)
 *
 * where post_div is the divider selected by POST_DIV_SEL and DIV_SELECT
 * has a valid range of 104-208.
 */

#define ANADIG_PLL_ARM_DIV_SELECT_SHIFT  (0)
#define ANADIG_PLL_ARM_DIV_SELECT_MASK   (0xffu << ANADIG_PLL_ARM_DIV_SELECT_SHIFT)
#define ANADIG_PLL_ARM_DIV_SELECT(x)     (((x) << ANADIG_PLL_ARM_DIV_SELECT_SHIFT) & \
                                          ANADIG_PLL_ARM_DIV_SELECT_MASK)
#define ANADIG_PLL_ARM_HOLD_RING_OFF     (1u << 12)
#define ANADIG_PLL_ARM_POWERUP           (1u << 13)
#define ANADIG_PLL_ARM_ENABLE_CLK        (1u << 14)
#define ANADIG_PLL_ARM_POST_DIV_SEL_SHIFT (15)
#define ANADIG_PLL_ARM_POST_DIV_SEL_MASK (0x3u << ANADIG_PLL_ARM_POST_DIV_SEL_SHIFT)
#define ANADIG_PLL_ARM_POST_DIV_SEL(x)   (((x) << ANADIG_PLL_ARM_POST_DIV_SEL_SHIFT) & \
                                          ANADIG_PLL_ARM_POST_DIV_SEL_MASK)
#  define ANADIG_PLL_ARM_POST_DIV_2      (0)  /* post_div = 2 */
#  define ANADIG_PLL_ARM_POST_DIV_4      (1)  /* post_div = 4 */
#  define ANADIG_PLL_ARM_POST_DIV_8      (2)  /* post_div = 8 */
#  define ANADIG_PLL_ARM_POST_DIV_1      (3)  /* post_div = 1 */
#define ANADIG_PLL_ARM_BYPASS            (1u << 17)
#define ANADIG_PLL_ARM_STABLE            (1u << 29)
#define ANADIG_PLL_ARM_GATE              (1u << 30)
#define ANADIG_PLL_ARM_CONTROL_MODE      (1u << 31)  /* 0: software, 1: GPC */

/* ANADIG PMU registers (IMXRT1180RM Rev. 10 sec. 24.5.1).  Offsets are
 * relative to the ANADIG base 0x4448_0000, same as the PLL block above.
 */

#define IMXRT_ANADIG_PMU_BIAS_CTRL       (IMXRT_ANADIG_PLL_BASE + 0x4600)
#define IMXRT_ANADIG_PMU_BIAS_CTRL2      (IMXRT_ANADIG_PLL_BASE + 0x4610)
#define IMXRT_ANADIG_PMU_LDO_PLL         (IMXRT_ANADIG_PLL_BASE + 0x4640)

/* PMU_BIAS_CTRL bit fields (reset value 0000_8000h) */

#define ANADIG_PMU_BIAS_CTRL_WB_CFG_1P8_SHIFT     (0)
#define ANADIG_PMU_BIAS_CTRL_WB_CFG_1P8_MASK      (0x1fffu << 0)
#define ANADIG_PMU_BIAS_CTRL_WB_CFG_1P8(x)        (((x) << 0) & \
                                        ANADIG_PMU_BIAS_CTRL_WB_CFG_1P8_MASK)

/* WB_CFG_1P8 sub-fields.  Each bit range below is described individually by
 * IMXRT1180RM Rev. 10 sec. 24.4.1.25 under the WB_CFG_1P8 field.
 */

#  define ANADIG_PMU_WB_CFG_PWELL_ONLY  (1u << 0)  /* 0: PWELL and NWELL on
                                                    * 1: PWELL only, NWELL
                                                    *    kept disabled */
#  define ANADIG_PMU_WB_CFG_NWELL_FBB   (1u << 1)  /* 0: NWELL to supply,
                                                    *    RVT CORE (RBB)
                                                    * 1: NWELL to supply,
                                                    *    LVT CORE (FBB) */

/* Bits 4-2: size of the bias area, i.e. the charge pump current budget. */

#  define ANADIG_PMU_WB_CFG_AREA_SHIFT  (2)
#  define ANADIG_PMU_WB_CFG_AREA_MASK   (0x7u << 2)
#  define ANADIG_PMU_WB_CFG_AREA(x)     (((x) << 2) & \
                                         ANADIG_PMU_WB_CFG_AREA_MASK)
#    define ANADIG_PMU_WB_AREA_180UA    (0)        /* 6.00 mm2 at 125C */
#    define ANADIG_PMU_WB_AREA_150UA    (1)        /* 5.00 mm2 at 125C */
#    define ANADIG_PMU_WB_AREA_120UA    (2)        /* 4.00 mm2 at 125C */
#    define ANADIG_PMU_WB_AREA_90UA     (3)        /* 3.00 mm2 at 125C */
#    define ANADIG_PMU_WB_AREA_60UA     (4)        /* 2.00 mm2 at 125C */
#    define ANADIG_PMU_WB_AREA_45UA     (5)        /* 0.15 mm2 at 125C */
#    define ANADIG_PMU_WB_AREA_30UA     (6)        /* 1.00 mm2 at 125C */
#    define ANADIG_PMU_WB_AREA_15UA     (7)        /* 0.50 mm2 at 125C */

/* Bit 5: when clear the charge pump frequency adapts each half cycle and
 * bits 8-6 give only the minimum.  When set the frequency is fixed at the
 * value selected by bits 8-6.
 */

#  define ANADIG_PMU_WB_CFG_ADAPTIVE_DIS (1u << 5)

/* Bits 8-6: charge pump oscillator frequency. */

#  define ANADIG_PMU_WB_CFG_OSC_SHIFT   (6)
#  define ANADIG_PMU_WB_CFG_OSC_MASK    (0x7u << 6)
#  define ANADIG_PMU_WB_CFG_OSC(x)      (((x) << 6) & \
                                         ANADIG_PMU_WB_CFG_OSC_MASK)
#    define ANADIG_PMU_WB_OSC_DIV128    (0)
#    define ANADIG_PMU_WB_OSC_DIV64     (1)
#    define ANADIG_PMU_WB_OSC_DIV32     (2)
#    define ANADIG_PMU_WB_OSC_DIV16     (3)
#    define ANADIG_PMU_WB_OSC_DIV8      (4)
#    define ANADIG_PMU_WB_OSC_DIV2      (6)
#    define ANADIG_PMU_WB_OSC_FULL      (7)        /* osc_freq */

/* Bit 9 and bits 11-10: adaptive clock source and frequency reduction. */

#  define ANADIG_PMU_WB_CFG_TRIM_SYNC   (1u << 9)
#  define ANADIG_PMU_WB_CFG_FREQ_RED_SHIFT (10)
#  define ANADIG_PMU_WB_CFG_FREQ_RED_MASK  (0x3u << 10)
#  define ANADIG_PMU_WB_CFG_PULLDOWN_EN (1u << 12) /* Pull-down option */
#define ANADIG_PMU_BIAS_CTRL_WB_VDD_SEL_1P8       (1u << 14)
#define ANADIG_PMU_BIAS_CTRL_FBB_M7_STBY_EN       (1u << 15)
#define ANADIG_PMU_BIAS_CTRL_WB_PW_LVL_1P8_SHIFT  (24)
#define ANADIG_PMU_BIAS_CTRL_WB_PW_LVL_1P8_MASK   (0xfu << 24)
#define ANADIG_PMU_BIAS_CTRL_WB_PW_LVL_1P8(x)     (((x) << 24) & \
                                     ANADIG_PMU_BIAS_CTRL_WB_PW_LVL_1P8_MASK)
#define ANADIG_PMU_BIAS_CTRL_WB_NW_LVL_1P8_SHIFT  (28)
#define ANADIG_PMU_BIAS_CTRL_WB_NW_LVL_1P8_MASK   (0xfu << 28)
#define ANADIG_PMU_BIAS_CTRL_WB_NW_LVL_1P8(x)     (((x) << 28) & \
                                     ANADIG_PMU_BIAS_CTRL_WB_NW_LVL_1P8_MASK)

/* PMU_BIAS_CTRL2 bit fields (reset value 0000_0000h) */

#define ANADIG_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8    (1u << 12)
#define ANADIG_PMU_BIAS_CTRL2_WB_ADJ_1P8_SHIFT    (13)
#define ANADIG_PMU_BIAS_CTRL2_WB_ADJ_1P8_MASK     (0xffu << 13)
#define ANADIG_PMU_BIAS_CTRL2_WB_EN               (1u << 24)
#define ANADIG_PMU_BIAS_CTRL2_WB_OK               (1u << 26)

/* ANADIG OSC registers (IMXRT1180RM Ch. 41) */

#define IMXRT_ANADIG_OSC_RC24M_CTRL      (IMXRT_ANADIG_OSC_BASE + 0x3310)
#define IMXRT_ANADIG_OSC_24M_CTRL        (IMXRT_ANADIG_OSC_BASE + 0x3320)
#define IMXRT_ANADIG_OSC_400M_CTRL1      (IMXRT_ANADIG_OSC_BASE + 0x3350)

/* PHY_LDO registers (IMXRT1180RM Ch. 41.  Physical: 0x4448_4680).  Each
 * has RW/SET/CLR/TOG aliases.
 */

#define IMXRT_PHY_LDO_CTRL0_BASE         (IMXRT_ANADIG_LDO_BASE + 0x680)
#define IMXRT_PHY_LDO_CTRL0_RW           (IMXRT_PHY_LDO_CTRL0_BASE + 0x0)
#define IMXRT_PHY_LDO_CTRL0_SET          (IMXRT_PHY_LDO_CTRL0_BASE + 0x4)
#define IMXRT_PHY_LDO_CTRL0_CLR          (IMXRT_PHY_LDO_CTRL0_BASE + 0x8)
#define IMXRT_PHY_LDO_CTRL0_TOG          (IMXRT_PHY_LDO_CTRL0_BASE + 0xc)

/* PHY_LDO_CTRL0 bit fields */

#define PHY_LDO_CTRL0_LINREG_EN          (1u << 0)   /* Enable linear regulator */
#define PHY_LDO_CTRL0_LINREG_PWRUPLOAD_DIS (1u << 1)
#define PHY_LDO_CTRL0_LINREG_ILIMIT_EN   (1u << 2)   /* Enable current-limit protection */
#define PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_SHIFT (4)    /* Output voltage trim */
#define PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_MASK (0x1fu << PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_SHIFT)
#define PHY_LDO_CTRL0_LINREG_OUTPUT_TRG(x) (((x) << PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_SHIFT) & \
                                            PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_MASK)
#define PHY_LDO_CTRL0_LINREG_PHY_ISO_B   (1u << 15)  /* Release PHY isolation */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ANADIG_H */
