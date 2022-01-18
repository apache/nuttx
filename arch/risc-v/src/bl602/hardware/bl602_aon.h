/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_aon.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_AON_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_AON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_AON_OFFSET                   0x000800  /* aon */
#define BL602_AON_COMMON_OFFSET            0x000804  /* aon_common */
#define BL602_AON_MISC_OFFSET              0x000808  /* aon_misc */
#define BL602_BG_SYS_TOP_OFFSET            0x000810  /* bg_sys_top */
#define BL602_DCDC18_TOP_0_OFFSET          0x000814  /* dcdc18_top_0 */
#define BL602_DCDC18_TOP_1_OFFSET          0x000818  /* dcdc18_top_1 */
#define BL602_LDO11SOC_AND_DCTEST_OFFSET   0x00081c  /* ldo11soc_and_dctest */
#define BL602_PSW_IRRCV_OFFSET             0x000820  /* psw_irrcv */
#define BL602_RF_TOP_AON_OFFSET            0x000880  /* rf_top_aon */
#define BL602_XTAL_CFG_OFFSET              0x000884  /* xtal_cfg */
#define BL602_TSEN_OFFSET                  0x000888  /* tsen */
#define BL602_ACOMP0_CTRL_OFFSET           0x000900  /* acomp0_ctrl */
#define BL602_ACOMP1_CTRL_OFFSET           0x000904  /* acomp1_ctrl */
#define BL602_ACOMP_CTRL_OFFSET            0x000908  /* acomp_ctrl */
#define BL602_GPADC_REG_CMD_OFFSET         0x00090c  /* gpadc_reg_cmd */
#define BL602_GPADC_REG_CONFIG1_OFFSET     0x000910  /* gpadc_reg_config1 */
#define BL602_GPADC_REG_CONFIG2_OFFSET     0x000914  /* gpadc_reg_config2 */
#define BL602_GPADC_REG_SCN_POS1_OFFSET    0x000918  /* adc conversion sequence 1 */
#define BL602_GPADC_REG_SCN_POS2_OFFSET    0x00091c  /* adc conversion sequence 2 */
#define BL602_GPADC_REG_SCN_NEG1_OFFSET    0x000920  /* adc conversion sequence 3 */
#define BL602_GPADC_REG_SCN_NEG2_OFFSET    0x000924  /* adc conversion sequence 4 */
#define BL602_GPADC_REG_STATUS_OFFSET      0x000928  /* gpadc_reg_status */
#define BL602_GPADC_REG_ISR_OFFSET         0x00092c  /* gpadc_reg_isr */
#define BL602_GPADC_REG_RESULT_OFFSET      0x000930  /* gpadc_reg_result */
#define BL602_GPADC_REG_RAW_RESULT_OFFSET  0x000934  /* gpadc_reg_raw_result */
#define BL602_GPADC_REG_DEFINE_OFFSET      0x000938  /* gpadc_reg_define */
#define BL602_HBNCORE_RESV0_OFFSET         0x00093c  /* hbncore_resv0 */
#define BL602_HBNCORE_RESV1_OFFSET         0x000940  /* hbncore_resv1 */

/* Register definitions *****************************************************/

#define BL602_AON                   (BL602_AON_BASE + BL602_AON_OFFSET)
#define BL602_AON_COMMON            (BL602_AON_BASE + BL602_AON_COMMON_OFFSET)
#define BL602_AON_MISC              (BL602_AON_BASE + BL602_AON_MISC_OFFSET)
#define BL602_BG_SYS_TOP            (BL602_AON_BASE + BL602_BG_SYS_TOP_OFFSET)
#define BL602_DCDC18_TOP_0          (BL602_AON_BASE + BL602_DCDC18_TOP_0_OFFSET)
#define BL602_DCDC18_TOP_1          (BL602_AON_BASE + BL602_DCDC18_TOP_1_OFFSET)
#define BL602_LDO11SOC_AND_DCTEST   (BL602_AON_BASE + BL602_LDO11SOC_AND_DCTEST_OFFSET)
#define BL602_PSW_IRRCV             (BL602_AON_BASE + BL602_PSW_IRRCV_OFFSET)
#define BL602_RF_TOP_AON            (BL602_AON_BASE + BL602_RF_TOP_AON_OFFSET)
#define BL602_XTAL_CFG              (BL602_AON_BASE + BL602_XTAL_CFG_OFFSET)
#define BL602_TSEN                  (BL602_AON_BASE + BL602_TSEN_OFFSET)
#define BL602_ACOMP0_CTRL           (BL602_AON_BASE + BL602_ACOMP0_CTRL_OFFSET)
#define BL602_ACOMP1_CTRL           (BL602_AON_BASE + BL602_ACOMP1_CTRL_OFFSET)
#define BL602_ACOMP_CTRL            (BL602_AON_BASE + BL602_ACOMP_CTRL_OFFSET)
#define BL602_GPADC_REG_CMD         (BL602_AON_BASE + BL602_GPADC_REG_CMD_OFFSET)
#define BL602_GPADC_REG_CONFIG1     (BL602_AON_BASE + BL602_GPADC_REG_CONFIG1_OFFSET)
#define BL602_GPADC_REG_CONFIG2     (BL602_AON_BASE + BL602_GPADC_REG_CONFIG2_OFFSET)
#define BL602_GPADC_REG_SCN_POS1    (BL602_AON_BASE + BL602_GPADC_REG_SCN_POS1_OFFSET)
#define BL602_GPADC_REG_SCN_POS2    (BL602_AON_BASE + BL602_GPADC_REG_SCN_POS2_OFFSET)
#define BL602_GPADC_REG_SCN_NEG1    (BL602_AON_BASE + BL602_GPADC_REG_SCN_NEG1_OFFSET)
#define BL602_GPADC_REG_SCN_NEG2    (BL602_AON_BASE + BL602_GPADC_REG_SCN_NEG2_OFFSET)
#define BL602_GPADC_REG_STATUS      (BL602_AON_BASE + BL602_GPADC_REG_STATUS_OFFSET)
#define BL602_GPADC_REG_ISR         (BL602_AON_BASE + BL602_GPADC_REG_ISR_OFFSET)
#define BL602_GPADC_REG_RESULT      (BL602_AON_BASE + BL602_GPADC_REG_RESULT_OFFSET)
#define BL602_GPADC_REG_RAW_RESULT  (BL602_AON_BASE + BL602_GPADC_REG_RAW_RESULT_OFFSET)
#define BL602_GPADC_REG_DEFINE      (BL602_AON_BASE + BL602_GPADC_REG_DEFINE_OFFSET)
#define BL602_HBNCORE_RESV0         (BL602_AON_BASE + BL602_HBNCORE_RESV0_OFFSET)
#define BL602_HBNCORE_RESV1         (BL602_AON_BASE + BL602_HBNCORE_RESV1_OFFSET)

/* Register bit definitions *************************************************/

#define SW_PU_LDO11_RT                                   (1 << 22)
#define LDO11_RT_PULLDOWN_SEL                            (1 << 21)
#define LDO11_RT_PULLDOWN                                (1 << 20)
#define PU_AON_DC_TBUF                                   (1 << 12)
#define RESV_MASK                                        (0xff)

#define COMMON_TEN_CIP_MISC_AON                          (1 << 20)
#define COMMON_TEN_MBG_AON                               (1 << 19)
#define COMMON_DTEN_XTAL_AON                             (1 << 18)
#define COMMON_TEN_XTAL_AON                              (1 << 17)
#define COMMON_TEN_LDO15RF_AON                           (1 << 16)
#define COMMON_TEN_BG_SYS_AON                            (1 << 12)
#define COMMON_TEN_DCDC18_1_AON                          (1 << 11)
#define COMMON_TEN_DCDC18_0_AON                          (1 << 10)
#define COMMON_TEN_LDO11SOC_AON                          (1 << 9)
#define COMMON_TEN_VDDCORE_AON                           (1 << 8)
#define COMMON_TEN_XTAL32K                               (1 << 6)
#define COMMON_DTEN_XTAL32K                              (1 << 5)
#define COMMON_TEN_AON                                   (1 << 4)
#define COMMON_TMUX_AON_MASK                             (0x07)

#define MISC_SW_WB_EN_AON                                (1 << 1)
#define MISC_SW_SOC_EN_AON                               (1 << 0)

#define BG_SYS_TOP_BG_SYS_START_CTRL_AON                     (1 << 12)
#define BG_SYS_TOP_PU_BG_SYS_AON                             (1 << 8)
#define BG_SYS_TOP_PMIP_RESV_MASK                            (0xff)

#define DCDC18_TOP_0_DCDC18_RDY_AON                          (1 << 31)
#define DCDC18_TOP_0_DCDC18_SSTART_TIME_AON_SHIFT            (28)
#define DCDC18_TOP_0_DCDC18_SSTART_TIME_AON_MASK             (0x03 << DCDC18_TOP_0_DCDC18_SSTART_TIME_AON_SHIFT)
#define DCDC18_TOP_0_DCDC18_OSC_INHIBIT_T2_AON               (1 << 27)
#define DCDC18_TOP_0_DCDC18_SLOW_OSC_AON                     (1 << 26)
#define DCDC18_TOP_0_DCDC18_STOP_OSC_AON                     (1 << 25)
#define DCDC18_TOP_0_DCDC18_SLOPE_CURR_SEL_AON_SHIFT         (20)
#define DCDC18_TOP_0_DCDC18_SLOPE_CURR_SEL_AON_MASK          (0x1f << DCDC18_TOP_0_DCDC18_SLOPE_CURR_SEL_AON_SHIFT)
#define DCDC18_TOP_0_DCDC18_OSC_FREQ_TRIM_AON_SHIFT          (16)
#define DCDC18_TOP_0_DCDC18_OSC_FREQ_TRIM_AON_MASK           (0x0f << DCDC18_TOP_0_DCDC18_OSC_FREQ_TRIM_AON_SHIFT)
#define DCDC18_TOP_0_DCDC18_OSC_2M_MODE_AON                  (1 << 12)
#define DCDC18_TOP_0_DCDC18_VPFM_AON_SHIFT                   (8)
#define DCDC18_TOP_0_DCDC18_VPFM_AON_MASK                    (0x0f << DCDC18_TOP_0_DCDC18_VPFM_AON_SHIFT)
#define DCDC18_TOP_0_DCDC18_VOUT_SEL_AON_SHIFT               (1)
#define DCDC18_TOP_0_DCDC18_VOUT_SEL_AON_MASK                (0x1f << DCDC18_TOP_0_DCDC18_VOUT_SEL_AON_SHIFT)

#define DCDC18_TOP_1_DCDC18_PULLDOWN_AON                     (1 << 29)
#define DCDC18_TOP_1_DCDC18_EN_ANTIRING_AON                  (1 << 28)
#define DCDC18_TOP_1_DCDC18_CFB_SEL_AON_SHIFT                (24)
#define DCDC18_TOP_1_DCDC18_CFB_SEL_AON_MASK                 (0x0f << DCDC18_TOP_1_DCDC18_CFB_SEL_AON_SHIFT)
#define DCDC18_TOP_1_DCDC18_CHF_SEL_AON_SHIFT                (20)
#define DCDC18_TOP_1_DCDC18_CHF_SEL_AON_MASK                 (0x0f << DCDC18_TOP_1_DCDC18_CHF_SEL_AON_SHIFT)
#define DCDC18_TOP_1_DCDC18_RC_SEL_AON_SHIFT                 (16)
#define DCDC18_TOP_1_DCDC18_RC_SEL_AON_MASK                  (0x0f << DCDC18_TOP_1_DCDC18_RC_SEL_AON_SHIFT)
#define DCDC18_TOP_1_DCDC18_NONOVERLAP_TD_AON_SHIFT          (8)
#define DCDC18_TOP_1_DCDC18_NONOVERLAP_TD_AON_MASK           (0x1f << DCDC18_TOP_1_DCDC18_NONOVERLAP_TD_AON_SHIFT)
#define DCDC18_TOP_1_DCDC18_ZVS_TD_OPT_AON_SHIFT             (4)
#define DCDC18_TOP_1_DCDC18_ZVS_TD_OPT_AON_MASK              (0x07 << DCDC18_TOP_1_DCDC18_ZVS_TD_OPT_AON_SHIFT)
#define DCDC18_TOP_1_DCDC18_CS_DELAY_AON_SHIFT               (1)
#define DCDC18_TOP_1_DCDC18_CS_DELAY_AON_MASK                (0x07 << DCDC18_TOP_1_DCDC18_CS_DELAY_AON_SHIFT)
#define DCDC18_TOP_1_DCDC18_FORCE_CS_ZVS_AON                 (1 << 0)

#define LDO11SOC_AND_DCTEST_PMIP_DC_TP_OUT_EN_AON            (1 << 31)
#define LDO11SOC_AND_DCTEST_PU_VDDCORE_MISC_AON              (1 << 30)
#define LDO11SOC_AND_DCTEST_LDO11SOC_POWER_GOOD_AON          (1 << 29)
#define LDO11SOC_AND_DCTEST_LDO11SOC_RDY_AON                 (1 << 28)
#define LDO11SOC_AND_DCTEST_LDO11SOC_CC_AON_SHIFT            (24)
#define LDO11SOC_AND_DCTEST_LDO11SOC_CC_AON_MASK             (0x03 << LDO11SOC_AND_DCTEST_LDO11SOC_CC_AON_SHIFT)
#define LDO11SOC_AND_DCTEST_LDO11SOC_VTH_SEL_AON_SHIFT       (12)
#define LDO11SOC_AND_DCTEST_LDO11SOC_VTH_SEL_AON_MASK        (0x03 << LDO11SOC_AND_DCTEST_LDO11SOC_VTH_SEL_AON_SHIFT)
#define LDO11SOC_AND_DCTEST_LDO11SOC_PULLDOWN_SEL_AON        (1 << 11)
#define LDO11SOC_AND_DCTEST_LDO11SOC_PULLDOWN_AON            (1 << 10)
#define LDO11SOC_AND_DCTEST_LDO11SOC_SSTART_DELAY_AON_SHIFT  (8)
#define LDO11SOC_AND_DCTEST_LDO11SOC_SSTART_DELAY_AON_MASK   (0x03 << LDO11SOC_AND_DCTEST_LDO11SOC_SSTART_DELAY_AON_SHIFT)
#define LDO11SOC_AND_DCTEST_LDO11SOC_SSTART_SEL_AON          (1 << 4)
#define LDO11SOC_AND_DCTEST_PU_LDO11SOC_AON                  (1 << 0)

#define PSW_IRRCV_PU_IR_PSW_AON                              (1 << 0)

#define RF_TOP_AON_LDO15RF_BYPASS_AON                        (1 << 28)
#define RF_TOP_AON_LDO15RF_CC_AON_SHIFT                      (24)
#define RF_TOP_AON_LDO15RF_CC_AON_MASK                       (0x03 << RF_TOP_AON_LDO15RF_CC_AON_SHIFT)
#define RF_TOP_AON_LDO15RF_VOUT_SEL_AON_SHIFT                (16)
#define RF_TOP_AON_LDO15RF_VOUT_SEL_AON_MASK                 (0x07 << RF_TOP_AON_LDO15RF_VOUT_SEL_AON_SHIFT)
#define RF_TOP_AON_LDO15RF_PULLDOWN_SEL_AON                  (1 << 13)
#define RF_TOP_AON_LDO15RF_PULLDOWN_AON                      (1 << 12)
#define RF_TOP_AON_LDO15RF_SSTART_DELAY_AON_SHIFT            (9)
#define RF_TOP_AON_LDO15RF_SSTART_DELAY_AON_MASK             (0x03 << RF_TOP_AON_LDO15RF_SSTART_DELAY_AON_SHIFT)
#define RF_TOP_AON_LDO15RF_SSTART_SEL_AON                    (1 << 8)
#define RF_TOP_AON_PU_XTAL_AON                               (1 << 5)
#define RF_TOP_AON_PU_XTAL_BUF_AON                           (1 << 4)
#define RF_TOP_AON_PU_SFREG_AON                              (1 << 2)
#define RF_TOP_AON_PU_LDO15RF_AON                            (1 << 1)
#define RF_TOP_AON_PU_MBG_AON                                (1 << 0)

#define XTAL_CFG_XTAL_RDY_SEL_AON_SHIFT                      (30)
#define XTAL_CFG_XTAL_RDY_SEL_AON_MASK                       (0x03 << XTAL_CFG_XTAL_RDY_SEL_AON_SHIFT)
#define XTAL_CFG_XTAL_GM_BOOST_AON_SHIFT                     (28)
#define XTAL_CFG_XTAL_GM_BOOST_AON_MASK                      (0x03 << XTAL_CFG_XTAL_GM_BOOST_AON_SHIFT)
#define XTAL_CFG_XTAL_CAPCODE_IN_AON_SHIFT                   (22)
#define XTAL_CFG_XTAL_CAPCODE_IN_AON_MASK                    (0x3f << XTAL_CFG_XTAL_CAPCODE_IN_AON_SHIFT)
#define XTAL_CFG_XTAL_CAPCODE_OUT_AON_SHIFT                  (16)
#define XTAL_CFG_XTAL_CAPCODE_OUT_AON_MASK                   (0x3f << XTAL_CFG_XTAL_CAPCODE_OUT_AON_SHIFT)
#define XTAL_CFG_XTAL_AMP_CTRL_AON_SHIFT                     (14)
#define XTAL_CFG_XTAL_AMP_CTRL_AON_MASK                      (0x03 << XTAL_CFG_XTAL_AMP_CTRL_AON_SHIFT)
#define XTAL_CFG_XTAL_SLEEP_AON                              (1 << 13)
#define XTAL_CFG_XTAL_FAST_STARTUP_AON                       (1 << 12)
#define XTAL_CFG_XTAL_BUF_HP_AON_SHIFT                       (8)
#define XTAL_CFG_XTAL_BUF_HP_AON_MASK                        (0x0f << XTAL_CFG_XTAL_BUF_HP_AON_SHIFT)
#define XTAL_CFG_XTAL_BUF_EN_AON_SHIFT                       (4)
#define XTAL_CFG_XTAL_BUF_EN_AON_MASK                        (0x0f << XTAL_CFG_XTAL_BUF_EN_AON_SHIFT)
#define XTAL_CFG_XTAL_EXT_SEL_AON                            (1 << 3)
#define XTAL_CFG_XTAL_CAPCODE_EXTRA_AON                      (1 << 2)
#define XTAL_CFG_XTAL_BK_AON_MASK                            (0x03)

#define TSEN_XTAL_RDY_INT_SEL_AON_SHIFT                      (30)
#define TSEN_XTAL_RDY_INT_SEL_AON_MASK                       (0x03 << TSEN_XTAL_RDY_INT_SEL_AON_SHIFT)
#define TSEN_XTAL_INN_CFG_EN_AON                             (1 << 29)
#define TSEN_XTAL_RDY                                        (1 << 28)
#define TSEN_TSEN_REFCODE_RFCAL_SHIFT                        (16)
#define TSEN_TSEN_REFCODE_RFCAL_MASK                         (0xfff << TSEN_TSEN_REFCODE_RFCAL_SHIFT)
#define TSEN_TSEN_REFCODE_CORNER_MASK                        (0xfff)

#define ACOMP0_CTRL_ACOMP0_MUXEN                             (1 << 26)
#define ACOMP0_CTRL_ACOMP0_POS_SEL_SHIFT                     (22)
#define ACOMP0_CTRL_ACOMP0_POS_SEL_MASK                      (0x0f << ACOMP0_CTRL_ACOMP0_POS_SEL_SHIFT)
#define ACOMP0_CTRL_ACOMP0_NEG_SEL_SHIFT                     (18)
#define ACOMP0_CTRL_ACOMP0_NEG_SEL_MASK                      (0x0f << ACOMP0_CTRL_ACOMP0_NEG_SEL_SHIFT)
#define ACOMP0_CTRL_ACOMP0_LEVEL_SEL_SHIFT                   (12)
#define ACOMP0_CTRL_ACOMP0_LEVEL_SEL_MASK                    (0x3f << ACOMP0_CTRL_ACOMP0_LEVEL_SEL_SHIFT)
#define ACOMP0_CTRL_ACOMP0_BIAS_PROG_SHIFT                   (10)
#define ACOMP0_CTRL_ACOMP0_BIAS_PROG_MASK                    (0x03 << ACOMP0_CTRL_ACOMP0_BIAS_PROG_SHIFT)
#define ACOMP0_CTRL_ACOMP0_HYST_SELP_SHIFT                   (7)
#define ACOMP0_CTRL_ACOMP0_HYST_SELP_MASK                    (0x07 << ACOMP0_CTRL_ACOMP0_HYST_SELP_SHIFT)
#define ACOMP0_CTRL_ACOMP0_HYST_SELN_SHIFT                   (4)
#define ACOMP0_CTRL_ACOMP0_HYST_SELN_MASK                    (0x07 << ACOMP0_CTRL_ACOMP0_HYST_SELN_SHIFT)
#define ACOMP0_CTRL_ACOMP0_EN                                (1 << 0)

#define ACOMP1_CTRL_ACOMP1_MUXEN                             (1 << 26)
#define ACOMP1_CTRL_ACOMP1_POS_SEL_SHIFT                     (22)
#define ACOMP1_CTRL_ACOMP1_POS_SEL_MASK                      (0x0f << ACOMP1_CTRL_ACOMP1_POS_SEL_SHIFT)
#define ACOMP1_CTRL_ACOMP1_NEG_SEL_SHIFT                     (18)
#define ACOMP1_CTRL_ACOMP1_NEG_SEL_MASK                      (0x0f << ACOMP1_CTRL_ACOMP1_NEG_SEL_SHIFT)
#define ACOMP1_CTRL_ACOMP1_LEVEL_SEL_SHIFT                   (12)
#define ACOMP1_CTRL_ACOMP1_LEVEL_SEL_MASK                    (0x3f << ACOMP1_CTRL_ACOMP1_LEVEL_SEL_SHIFT)
#define ACOMP1_CTRL_ACOMP1_BIAS_PROG_SHIFT                   (10)
#define ACOMP1_CTRL_ACOMP1_BIAS_PROG_MASK                    (0x03 << ACOMP1_CTRL_ACOMP1_BIAS_PROG_SHIFT)
#define ACOMP1_CTRL_ACOMP1_HYST_SELP_SHIFT                   (7)
#define ACOMP1_CTRL_ACOMP1_HYST_SELP_MASK                    (0x07 << ACOMP1_CTRL_ACOMP1_HYST_SELP_SHIFT)
#define ACOMP1_CTRL_ACOMP1_HYST_SELN_SHIFT                   (4)
#define ACOMP1_CTRL_ACOMP1_HYST_SELN_MASK                    (0x07 << ACOMP1_CTRL_ACOMP1_HYST_SELN_SHIFT)
#define ACOMP1_CTRL_ACOMP1_EN                                (1 << 0)

#define ACOMP_CTRL_ACOMP0_OUT_RAW                            (1 << 19)
#define ACOMP_CTRL_ACOMP1_OUT_RAW                            (1 << 17)
#define ACOMP_CTRL_ACOMP0_TEST_SEL_SHIFT                     (12)
#define ACOMP_CTRL_ACOMP0_TEST_SEL_MASK                      (0x03 << ACOMP_CTRL_ACOMP0_TEST_SEL_SHIFT)
#define ACOMP_CTRL_ACOMP1_TEST_SEL_SHIFT                     (10)
#define ACOMP_CTRL_ACOMP1_TEST_SEL_MASK                      (0x03 << ACOMP_CTRL_ACOMP1_TEST_SEL_SHIFT)
#define ACOMP_CTRL_ACOMP0_TEST_EN                            (1 << 9)
#define ACOMP_CTRL_ACOMP1_TEST_EN                            (1 << 8)
#define ACOMP_CTRL_ACOMP0_RSTN_ANA                           (1 << 1)
#define ACOMP_CTRL_ACOMP1_RSTN_ANA                           (1 << 0)

#define GPADC_REG_CMD_GPADC_SEN_TEST_EN                      (1 << 30)
#define GPADC_REG_CMD_GPADC_SEN_SEL_SHIFT                    (28)
#define GPADC_REG_CMD_GPADC_SEN_SEL_MASK                     (0x03 << GPADC_REG_CMD_GPADC_SEN_SEL_SHIFT)
#define GPADC_REG_CMD_GPADC_CHIP_SEN_PU                      (1 << 27)
#define GPADC_REG_CMD_GPADC_MICBOOST_32DB_EN                 (1 << 23)
#define GPADC_REG_CMD_GPADC_MIC_PGA2_GAIN_SHIFT              (21)
#define GPADC_REG_CMD_GPADC_MIC_PGA2_GAIN_MASK               (0x03 << GPADC_REG_CMD_GPADC_MIC_PGA2_GAIN_SHIFT)
#define GPADC_REG_CMD_GPADC_MIC1_DIFF                        (1 << 20)
#define GPADC_REG_CMD_GPADC_MIC2_DIFF                        (1 << 19)
#define GPADC_REG_CMD_GPADC_DWA_EN                           (1 << 18)
#define GPADC_REG_CMD_GPADC_BYP_MICBOOST                     (1 << 16)
#define GPADC_REG_CMD_GPADC_MICPGA_EN                        (1 << 15)
#define GPADC_REG_CMD_GPADC_MICBIAS_EN                       (1 << 14)
#define GPADC_REG_CMD_GPADC_NEG_GND                          (1 << 13)
#define GPADC_REG_CMD_GPADC_POS_SEL_SHIFT                    (8)
#define GPADC_REG_CMD_GPADC_POS_SEL_MASK                     (0x1f << GPADC_REG_CMD_GPADC_POS_SEL_SHIFT)
#define GPADC_REG_CMD_GPADC_NEG_SEL_SHIFT                    (3)
#define GPADC_REG_CMD_GPADC_NEG_SEL_MASK                     (0x1f << GPADC_REG_CMD_GPADC_NEG_SEL_SHIFT)
#define GPADC_REG_CMD_GPADC_SOFT_RST                         (1 << 2)
#define GPADC_REG_CMD_GPADC_CONV_START                       (1 << 1)
#define GPADC_REG_CMD_GPADC_GLOBAL_EN                        (1 << 0)

#define GPADC_REG_CONFIG1_GPADC_V18_SEL_SHIFT                (29)
#define GPADC_REG_CONFIG1_GPADC_V18_SEL_MASK                 (0x03 << GPADC_REG_CONFIG1_GPADC_V18_SEL_SHIFT)
#define GPADC_REG_CONFIG1_GPADC_V11_SEL_SHIFT                (27)
#define GPADC_REG_CONFIG1_GPADC_V11_SEL_MASK                 (0x03 << GPADC_REG_CONFIG1_GPADC_V11_SEL_SHIFT)
#define GPADC_REG_CONFIG1_GPADC_DITHER_EN                    (1 << 26)
#define GPADC_REG_CONFIG1_GPADC_SCAN_EN                      (1 << 25)
#define GPADC_REG_CONFIG1_GPADC_SCAN_LENGTH_SHIFT            (21)
#define GPADC_REG_CONFIG1_GPADC_SCAN_LENGTH_MASK             (0x0f << GPADC_REG_CONFIG1_GPADC_SCAN_LENGTH_SHIFT)
#define GPADC_REG_CONFIG1_GPADC_CLK_DIV_RATIO_SHIFT          (18)
#define GPADC_REG_CONFIG1_GPADC_CLK_DIV_RATIO_MASK           (0x07 << GPADC_REG_CONFIG1_GPADC_CLK_DIV_RATIO_SHIFT)
#define GPADC_REG_CONFIG1_GPADC_CLK_ANA_INV                  (1 << 17)
#define GPADC_REG_CONFIG1_GPADC_RES_SEL_SHIFT                (2)
#define GPADC_REG_CONFIG1_GPADC_RES_SEL_MASK                 (0x07 << GPADC_REG_CONFIG1_GPADC_RES_SEL_SHIFT)
#define GPADC_REG_CONFIG1_GPADC_CONT_CONV_EN                 (1 << 1)
#define GPADC_REG_CONFIG1_GPADC_CAL_OS_EN                    (1 << 0)

#define GPADC_REG_CONFIG2_GPADC_TSVBE_LOW                    (1 << 31)
#define GPADC_REG_CONFIG2_GPADC_DLY_SEL_SHIFT                (28)
#define GPADC_REG_CONFIG2_GPADC_DLY_SEL_MASK                 (0x07 << GPADC_REG_CONFIG2_GPADC_DLY_SEL_SHIFT)
#define GPADC_REG_CONFIG2_GPADC_PGA1_GAIN_SHIFT              (25)
#define GPADC_REG_CONFIG2_GPADC_PGA1_GAIN_MASK               (0x07 << GPADC_REG_CONFIG2_GPADC_PGA1_GAIN_SHIFT)
#define GPADC_REG_CONFIG2_GPADC_PGA2_GAIN_SHIFT              (22)
#define GPADC_REG_CONFIG2_GPADC_PGA2_GAIN_MASK               (0x07 << GPADC_REG_CONFIG2_GPADC_PGA2_GAIN_SHIFT)
#define GPADC_REG_CONFIG2_GPADC_TEST_SEL_SHIFT               (19)
#define GPADC_REG_CONFIG2_GPADC_TEST_SEL_MASK                (0x07 << GPADC_REG_CONFIG2_GPADC_TEST_SEL_SHIFT)
#define GPADC_REG_CONFIG2_GPADC_TEST_EN                      (1 << 18)
#define GPADC_REG_CONFIG2_GPADC_BIAS_SEL                     (1 << 17)
#define GPADC_REG_CONFIG2_GPADC_CHOP_MODE_SHIFT              (15)
#define GPADC_REG_CONFIG2_GPADC_CHOP_MODE_MASK               (0x03 << GPADC_REG_CONFIG2_GPADC_CHOP_MODE_SHIFT)
#define GPADC_REG_CONFIG2_GPADC_PGA_VCMI_EN                  (1 << 14)
#define GPADC_REG_CONFIG2_GPADC_PGA_EN                       (1 << 13)
#define GPADC_REG_CONFIG2_GPADC_PGA_OS_CAL_SHIFT             (9)
#define GPADC_REG_CONFIG2_GPADC_PGA_OS_CAL_MASK              (0x0f << GPADC_REG_CONFIG2_GPADC_PGA_OS_CAL_SHIFT)
#define GPADC_REG_CONFIG2_GPADC_PGA_VCM_SHIFT                (7)
#define GPADC_REG_CONFIG2_GPADC_PGA_VCM_MASK                 (0x03 << GPADC_REG_CONFIG2_GPADC_PGA_VCM_SHIFT)
#define GPADC_REG_CONFIG2_GPADC_TS_EN                        (1 << 6)
#define GPADC_REG_CONFIG2_GPADC_TSEXT_SEL                    (1 << 5)
#define GPADC_REG_CONFIG2_GPADC_VBAT_EN                      (1 << 4)
#define GPADC_REG_CONFIG2_GPADC_VREF_SEL                     (1 << 3)
#define GPADC_REG_CONFIG2_GPADC_DIFF_MODE                    (1 << 2)

#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_5_SHIFT            (25)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_5_MASK             (0x1f << GPADC_REG_SCN_POS1_GPADC_SCAN_POS_5_SHIFT)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_4_SHIFT            (20)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_4_MASK             (0x1f << GPADC_REG_SCN_POS1_GPADC_SCAN_POS_4_SHIFT)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_3_SHIFT            (15)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_3_MASK             (0x1f << GPADC_REG_SCN_POS1_GPADC_SCAN_POS_3_SHIFT)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_2_SHIFT            (10)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_2_MASK             (0x1f << GPADC_REG_SCN_POS1_GPADC_SCAN_POS_2_SHIFT)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_1_SHIFT            (5)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_1_MASK             (0x1f << GPADC_REG_SCN_POS1_GPADC_SCAN_POS_1_SHIFT)
#define GPADC_REG_SCN_POS1_GPADC_SCAN_POS_0_MASK             (0x1f)

#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_11_SHIFT           (25)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_11_MASK            (0x1f << GPADC_REG_SCN_POS2_GPADC_SCAN_POS_11_SHIFT)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_10_SHIFT           (20)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_10_MASK            (0x1f << GPADC_REG_SCN_POS2_GPADC_SCAN_POS_10_SHIFT)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_9_SHIFT            (15)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_9_MASK             (0x1f << GPADC_REG_SCN_POS2_GPADC_SCAN_POS_9_SHIFT)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_8_SHIFT            (10)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_8_MASK             (0x1f << GPADC_REG_SCN_POS2_GPADC_SCAN_POS_8_SHIFT)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_7_SHIFT            (5)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_7_MASK             (0x1f << GPADC_REG_SCN_POS2_GPADC_SCAN_POS_7_SHIFT)
#define GPADC_REG_SCN_POS2_GPADC_SCAN_POS_6_MASK             (0x1f)

#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_5_SHIFT            (25)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_5_MASK             (0x1f << GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_5_SHIFT)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_4_SHIFT            (20)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_4_MASK             (0x1f << GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_4_SHIFT)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_3_SHIFT            (15)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_3_MASK             (0x1f << GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_3_SHIFT)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_2_SHIFT            (10)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_2_MASK             (0x1f << GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_2_SHIFT)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_1_SHIFT            (5)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_1_MASK             (0x1f << GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_1_SHIFT)
#define GPADC_REG_SCN_NEG1_GPADC_SCAN_NEG_0_MASK             (0x1f)

#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_11_SHIFT           (25)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_11_MASK            (0x1f << GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_11_SHIFT)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_10_SHIFT           (20)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_10_MASK            (0x1f << GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_10_SHIFT)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_9_SHIFT            (15)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_9_MASK             (0x1f << GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_9_SHIFT)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_8_SHIFT            (10)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_8_MASK             (0x1f << GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_8_SHIFT)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_7_SHIFT            (5)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_7_MASK             (0x1f << GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_7_SHIFT)
#define GPADC_REG_SCN_NEG2_GPADC_SCAN_NEG_6_MASK             (0x1f)

#define GPADC_REG_STATUS_GPADC_DATA_RDY                      (1 << 0)

#define GPADC_REG_ISR_GPADC_POS_SATUR_MASK                   (1 << 9)
#define GPADC_REG_ISR_GPADC_NEG_SATUR_MASK                   (1 << 8)
#define GPADC_REG_ISR_GPADC_POS_SATUR_CLR                    (1 << 5)
#define GPADC_REG_ISR_GPADC_NEG_SATUR_CLR                    (1 << 4)
#define GPADC_REG_ISR_GPADC_POS_SATUR                        (1 << 1)
#define GPADC_REG_ISR_GPADC_NEG_SATUR                        (1 << 0)

#define GPADC_REG_RESULT_GPADC_DATA_OUT_MASK                 (0x3ffffff)

#define GPADC_REG_RAW_RESULT_GPADC_RAW_DATA_MASK             (0xfff)

#define GPADC_REG_DEFINE_GPADC_OS_CAL_DATA_MASK              (0xffff)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_AON_H */
