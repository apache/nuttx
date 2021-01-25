/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_pds.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_PDS_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_PDS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_PDS_CTL_OFFSET               0x000000  /* PDS_CTL */
#define BL602_PDS_TIME1_OFFSET             0x000004  /* PDS_TIME1 */
#define BL602_PDS_INT_OFFSET               0x00000c  /* PDS_INT */
#define BL602_PDS_CTL2_OFFSET              0x000010  /* PDS_CTL2 */
#define BL602_PDS_CTL3_OFFSET              0x000014  /* PDS_CTL3 */
#define BL602_PDS_CTL4_OFFSET              0x000018  /* PDS_CTL4 */
#define BL602_PDS_STAT_OFFSET              0x00001c  /* pds_stat */
#define BL602_PDS_RAM1_OFFSET              0x000020  /* pds_ram1 */
#define BL602_PDS_RC32M_CTRL0_OFFSET       0x000300  /* rc32m_ctrl0 */
#define BL602_PDS_RC32M_CTRL1_OFFSET       0x000304  /* rc32m_ctrl1 */
#define BL602_PDS_PU_RST_CLKPLL_OFFSET     0x000400  /* pu_rst_clkpll */
#define BL602_PDS_CLKPLL_TOP_CTRL_OFFSET   0x000404  /* clkpll_top_ctrl */
#define BL602_PDS_CLKPLL_CP_OFFSET         0x000408  /* clkpll_cp */
#define BL602_PDS_CLKPLL_RZ_OFFSET         0x00040c  /* clkpll_rz */
#define BL602_PDS_CLKPLL_FBDV_OFFSET       0x000410  /* clkpll_fbdv */
#define BL602_PDS_CLKPLL_VCO_OFFSET        0x000414  /* clkpll_vco */
#define BL602_PDS_CLKPLL_SDM_OFFSET        0x000418  /* clkpll_sdm */
#define BL602_PDS_CLKPLL_OUTPUT_EN_OFFSET  0x00041c  /* clkpll_output_en */

/* Register definitions *****************************************************/

#define BL602_PDS_CTL               (BL602_PDS_BASE + BL602_PDS_CTL_OFFSET)
#define BL602_PDS_TIME1             (BL602_PDS_BASE + BL602_PDS_TIME1_OFFSET)
#define BL602_PDS_INT               (BL602_PDS_BASE + BL602_PDS_INT_OFFSET)
#define BL602_PDS_CTL2              (BL602_PDS_BASE + BL602_PDS_CTL2_OFFSET)
#define BL602_PDS_CTL3              (BL602_PDS_BASE + BL602_PDS_CTL3_OFFSET)
#define BL602_PDS_CTL4              (BL602_PDS_BASE + BL602_PDS_CTL4_OFFSET)
#define BL602_PDS_STAT              (BL602_PDS_BASE + BL602_PDS_STAT_OFFSET)
#define BL602_PDS_RAM1              (BL602_PDS_BASE + BL602_PDS_RAM1_OFFSET)
#define BL602_PDS_RC32M_CTRL0       (BL602_PDS_BASE + BL602_PDS_RC32M_CTRL0_OFFSET)
#define BL602_PDS_RC32M_CTRL1       (BL602_PDS_BASE + BL602_PDS_RC32M_CTRL1_OFFSET)
#define BL602_PDS_PU_RST_CLKPLL     (BL602_PDS_BASE + BL602_PDS_PU_RST_CLKPLL_OFFSET)
#define BL602_PDS_CLKPLL_TOP_CTRL   (BL602_PDS_BASE + BL602_PDS_CLKPLL_TOP_CTRL_OFFSET)
#define BL602_PDS_CLKPLL_CP         (BL602_PDS_BASE + BL602_PDS_CLKPLL_CP_OFFSET)
#define BL602_PDS_CLKPLL_RZ         (BL602_PDS_BASE + BL602_PDS_CLKPLL_RZ_OFFSET)
#define BL602_PDS_CLKPLL_FBDV       (BL602_PDS_BASE + BL602_PDS_CLKPLL_FBDV_OFFSET)
#define BL602_PDS_CLKPLL_VCO        (BL602_PDS_BASE + BL602_PDS_CLKPLL_VCO_OFFSET)
#define BL602_PDS_CLKPLL_SDM        (BL602_PDS_BASE + BL602_PDS_CLKPLL_SDM_OFFSET)
#define BL602_PDS_CLKPLL_OUTPUT_EN  (BL602_PDS_BASE + BL602_PDS_CLKPLL_OUTPUT_EN_OFFSET)

/* Register bit definitions *************************************************/

#define PDS_CTL_CR_CTRL_PLL_SHIFT              (30)
#define PDS_CTL_CR_CTRL_PLL_MASK               (0x03 << PDS_CTL_CR_CTRL_PLL_SHIFT)
#define PDS_CTL_CR_CTRL_RF_SHIFT               (28)
#define PDS_CTL_CR_CTRL_RF_MASK                (0x03 << PDS_CTL_CR_CTRL_RF_SHIFT)
#define PDS_CTL_CR_LDO_VOL_SHIFT               (24)
#define PDS_CTL_CR_LDO_VOL_MASK                (0x0f << PDS_CTL_CR_LDO_VOL_SHIFT)
#define PDS_CTL_CR_PD_LDO11                    (1 << 22)
#define PDS_CTL_CR_NP_WFI_MASK                 (1 << 21)
#define PDS_CTL_CR_LDO_VSEL_EN                 (1 << 18)
#define PDS_CTL_CR_RC32M_OFF_DIS               (1 << 17)
#define PDS_CTL_CR_RST_SOC_EN                  (1 << 16)
#define PDS_CTL_CR_SOC_ENB_FORCE_ON            (1 << 15)
#define PDS_CTL_CR_PD_XTAL                     (1 << 14)
#define PDS_CTL_CR_PWR_OFF                     (1 << 13)
#define PDS_CTL_CR_WAIT_XTAL_RDY               (1 << 12)
#define PDS_CTL_CR_ISO_EN                      (1 << 11)
#define PDS_CTL_CR_MEM_STBY                    (1 << 9)
#define PDS_CTL_CR_GATE_CLK                    (1 << 8)
#define PDS_CTL_CR_PD_BG_SYS                   (1 << 5)
#define PDS_CTL_CR_PD_DCDC18                   (1 << 4)
#define PDS_CTL_CR_WIFI_PDS_SAVE_STATE         (1 << 3)
#define PDS_CTL_CR_XTAL_FORCE_OFF              (1 << 2)
#define PDS_CTL_CR_SLEEP_FOREVER               (1 << 1)
#define PDS_CTL_PDS_START_PS                   (1 << 0)

#define PDS_INT_CR_INT_CLR                     (1 << 16)
#define PDS_INT_CR_PLL_DONE_INT_MASK           (1 << 11)
#define PDS_INT_CR_RF_DONE_INT_MASK            (1 << 10)
#define PDS_INT_CR_IRQ_IN_DIS                  (1 << 9)
#define PDS_INT_CR_WAKE_INT_MASK               (1 << 8)
#define PDS_INT_RO_PLL_DONE_INT                (1 << 3)
#define PDS_INT_RO_RF_DONE_INT                 (1 << 2)
#define PDS_INT_RO_IRQ_IN                      (1 << 1)
#define PDS_INT_RO_WAKE_INT                    (1 << 0)

#define PDS_CTL2_CR_FORCE_WB_GATE_CLK          (1 << 18)
#define PDS_CTL2_CR_FORCE_NP_GATE_CLK          (1 << 16)
#define PDS_CTL2_CR_FORCE_WB_MEM_STBY          (1 << 14)
#define PDS_CTL2_CR_FORCE_NP_MEM_STBY          (1 << 12)
#define PDS_CTL2_CR_FORCE_WB_PDS_RST           (1 << 10)
#define PDS_CTL2_CR_FORCE_NP_PDS_RST           (1 << 8)
#define PDS_CTL2_CR_FORCE_WB_ISO_EN            (1 << 6)
#define PDS_CTL2_CR_FORCE_NP_ISO_EN            (1 << 4)
#define PDS_CTL2_CR_FORCE_WB_PWR_OFF           (1 << 2)
#define PDS_CTL2_CR_FORCE_NP_PWR_OFF           (1 << 0)

#define PDS_CTL3_CR_MISC_ISO_EN                (1 << 30)
#define PDS_CTL3_CR_WB_ISO_EN                  (1 << 27)
#define PDS_CTL3_CR_NP_ISO_EN                  (1 << 24)
#define PDS_CTL3_CR_FORCE_MISC_GATE_CLK        (1 << 13)
#define PDS_CTL3_CR_FORCE_MISC_MEM_STBY        (1 << 10)
#define PDS_CTL3_CR_FORCE_MISC_PDS_RST         (1 << 7)
#define PDS_CTL3_CR_FORCE_MISC_ISO_EN          (1 << 4)
#define PDS_CTL3_CR_FORCE_MISC_PWR_OFF         (1 << 1)

#define PDS_CTL4_CR_MISC_GATE_CLK              (1 << 27)
#define PDS_CTL4_CR_MISC_MEM_STBY              (1 << 26)
#define PDS_CTL4_CR_MISC_RESET                 (1 << 25)
#define PDS_CTL4_CR_MISC_PWR_OFF               (1 << 24)
#define PDS_CTL4_CR_WB_GATE_CLK                (1 << 15)
#define PDS_CTL4_CR_WB_MEM_STBY                (1 << 14)
#define PDS_CTL4_CR_WB_RESET                   (1 << 13)
#define PDS_CTL4_CR_WB_PWR_OFF                 (1 << 12)
#define PDS_CTL4_CR_NP_GATE_CLK                (1 << 3)
#define PDS_CTL4_CR_NP_MEM_STBY                (1 << 2)
#define PDS_CTL4_CR_NP_RESET                   (1 << 1)
#define PDS_CTL4_CR_NP_PWR_OFF                 (1 << 0)

#define PDS_STAT_RO_PLL_STATE_SHIFT            (16)
#define PDS_STAT_RO_PLL_STATE_MASK             (0x03 << PDS_STAT_RO_PLL_STATE_SHIFT)
#define PDS_STAT_RO_RF_STATE_SHIFT             (8)
#define PDS_STAT_RO_RF_STATE_MASK              (0x0f << PDS_STAT_RO_RF_STATE_SHIFT)
#define PDS_STAT_RO_STATE_MASK                 (0x0f)

#define PDS_RAM1_CR_NP_SRAM_PWR_MASK               (0xff)

#define PDS_RC32M_CTRL0_CODE_FR_EXT_SHIFT        (22)
#define PDS_RC32M_CTRL0_CODE_FR_EXT_MASK         (0xff << PDS_RC32M_CTRL0_CODE_FR_EXT_SHIFT)
#define PDS_RC32M_CTRL0_PD                       (1 << 21)
#define PDS_RC32M_CTRL0_CAL_EN                   (1 << 20)
#define PDS_RC32M_CTRL0_EXT_CODE_EN              (1 << 19)
#define PDS_RC32M_CTRL0_REFCLK_HALF              (1 << 18)
#define PDS_RC32M_CTRL0_ALLOW_CAL                (1 << 17)
#define PDS_RC32M_CTRL0_DIG_CODE_FR_CAL_SHIFT    (6)
#define PDS_RC32M_CTRL0_DIG_CODE_FR_CAL_MASK     (0xff << PDS_RC32M_CTRL0_DIG_CODE_FR_CAL_SHIFT)
#define PDS_RC32M_CTRL0_CAL_PRECHARGE            (1 << 5)
#define PDS_RC32M_CTRL0_CAL_DIV_SHIFT            (3)
#define PDS_RC32M_CTRL0_CAL_DIV_MASK             (0x03 << PDS_RC32M_CTRL0_CAL_DIV_SHIFT)
#define PDS_RC32M_CTRL0_CAL_INPROGRESS           (1 << 2)
#define PDS_RC32M_CTRL0_RDY                      (1 << 1)
#define PDS_RC32M_CTRL0_CAL_DONE                 (1 << 0)

#define PDS_RC32M_CTRL1_RC32M_CLK_FORCE_ON             (1 << 4)
#define PDS_RC32M_CTRL1_RC32M_CLK_INV                  (1 << 3)
#define PDS_RC32M_CTRL1_RC32M_CLK_SOFT_RST             (1 << 2)
#define PDS_RC32M_CTRL1_RC32M_SOFT_RST                 (1 << 1)
#define PDS_RC32M_CTRL1_RC32M_TEST_EN                  (1 << 0)

#define PDS_PU_RST_CLKPLL_PU_CLKPLL                    (1 << 10)
#define PDS_PU_RST_CLKPLL_PU_CLKPLL_SFREG              (1 << 9)
#define PDS_PU_RST_CLKPLL_PU_CP                 (1 << 8)
#define PDS_PU_RST_CLKPLL_PU_PFD                (1 << 7)
#define PDS_PU_RST_CLKPLL_PU_CLAMP_OP           (1 << 6)
#define PDS_PU_RST_CLKPLL_PU_FBDV               (1 << 5)
#define PDS_PU_RST_CLKPLL_PU_POSTDIV            (1 << 4)
#define PDS_PU_RST_CLKPLL_RESET_REFDIV          (1 << 3)
#define PDS_PU_RST_CLKPLL_RESET_FBDV            (1 << 2)
#define PDS_PU_RST_CLKPLL_RESET_POSTDIV         (1 << 1)
#define PDS_PU_RST_CLKPLL_SDM_RESET             (1 << 0)

#define PDS_CLKPLL_TOP_CTRL_VG13_SEL_SHIFT      (24)
#define PDS_CLKPLL_TOP_CTRL_VG13_SEL_MASK       (0x03 << PDS_CLKPLL_TOP_CTRL_VG13_SEL_SHIFT)
#define PDS_CLKPLL_TOP_CTRL_VG11_SEL_SHIFT      (20)
#define PDS_CLKPLL_TOP_CTRL_VG11_SEL_MASK       (0x03 << PDS_CLKPLL_TOP_CTRL_VG11_SEL_SHIFT)
#define PDS_CLKPLL_TOP_CTRL_REFCLK_SEL          (1 << 16)
#define PDS_CLKPLL_TOP_CTRL_XTAL_RC32M_SEL      (1 << 12)
#define PDS_CLKPLL_TOP_CTRL_REFDIV_RATIO_SHIFT  (8)
#define PDS_CLKPLL_TOP_CTRL_REFDIV_RATIO_MASK   (0x0f << PDS_CLKPLL_TOP_CTRL_REFDIV_RATIO_SHIFT)
#define PDS_CLKPLL_TOP_CTRL_POSTDIV_MASK        (0x7f)

#define PDS_CLKPLL_CP_CP_OPAMP_EN               (1 << 10)
#define PDS_CLKPLL_CP_CP_STARTUP_EN             (1 << 9)
#define PDS_CLKPLL_CP_INT_FRAC_SW               (1 << 8)
#define PDS_CLKPLL_CP_ICP_1U_SHIFT              (6)
#define PDS_CLKPLL_CP_ICP_1U_MASK               (0x03 << PDS_CLKPLL_CP_ICP_1U_SHIFT)
#define PDS_CLKPLL_CP_ICP_5U_SHIFT              (4)
#define PDS_CLKPLL_CP_ICP_5U_MASK               (0x03 << PDS_CLKPLL_CP_ICP_5U_SHIFT)
#define PDS_CLKPLL_CP_SEL_CP_BIAS               (1 << 0)

#define PDS_CLKPLL_RZ_RZ_SHIFT                  (16)
#define PDS_CLKPLL_RZ_MASK                      (0x07 << PDS_CLKPLL_RZ_RZ_SHIFT)
#define PDS_CLKPLL_RZ_CZ_SHIFT                  (14)
#define PDS_CLKPLL_RZ_CZ_MASK                   (0x03 << PDS_CLKPLL_RZ_CZ_SHIFT)
#define PDS_CLKPLL_RZ_C3_SHIFT                  (12)
#define PDS_CLKPLL_RZ_C3_MASK                   (0x03 << PDS_CLKPLL_RZ_C3_SHIFT)
#define PDS_CLKPLL_RZ_R4_SHORT                  (1 << 8)
#define PDS_CLKPLL_RZ_R4_SHIFT                  (4)
#define PDS_CLKPLL_RZ_R4_MASK                   (0x03 << PDS_CLKPLL_RZ_R4_SHIFT)
#define PDS_CLKPLL_RZ_C4_EN                     (1 << 0)

#define PDS_CLKPLL_FBDV_SEL_FB_CLK_SHIFT        (2)
#define PDS_CLKPLL_FBDV_SEL_FB_CLK_MASK         (0x03 << PDS_CLKPLL_FBDV_SEL_FB_CLK_SHIFT)
#define PDS_CLKPLL_FBDV_SEL_SAMPLE_CLK_MASK     (0x03)

#define PDS_CLKPLL_VCO_SHRTR                    (1 << 3)
#define PDS_CLKPLL_VCO_VCO_SPEED_MASK           (0x07)

#define PDS_CLKPLL_SDM_SDM_BYPASS               (1 << 29)
#define PDS_CLKPLL_SDM_SDM_FLAG                 (1 << 28)
#define PDS_CLKPLL_SDM_DITHER_SEL_SHIFT         (24)
#define PDS_CLKPLL_SDM_DITHER_SEL_MASK          (0x03 << PDS_CLKPLL_SDM_DITHER_SEL_SHIFT)
#define PDS_CLKPLL_SDM_SDMIN_MASK               (0xffffff)

#define PDS_CLKPLL_OUTPUT_EN_EN_DIV2_480M       (1 << 9)
#define PDS_CLKPLL_OUTPUT_EN_EN_32M             (1 << 8)
#define PDS_CLKPLL_OUTPUT_EN_EN_48M             (1 << 7)
#define PDS_CLKPLL_OUTPUT_EN_EN_80M             (1 << 6)
#define PDS_CLKPLL_OUTPUT_EN_EN_96M             (1 << 5)
#define PDS_CLKPLL_OUTPUT_EN_EN_120M            (1 << 4)
#define PDS_CLKPLL_OUTPUT_EN_EN_160M            (1 << 3)
#define PDS_CLKPLL_OUTPUT_EN_EN_192M            (1 << 2)
#define PDS_CLKPLL_OUTPUT_EN_EN_240M            (1 << 1)
#define PDS_CLKPLL_OUTPUT_EN_EN_480M            (1 << 0)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_PDS_H */
