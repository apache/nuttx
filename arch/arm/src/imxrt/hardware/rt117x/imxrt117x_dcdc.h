/*********************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_dcdc.h
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
 *********************************************************************************/

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_DCDC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_DCDC_H

/*********************************************************************************
 * Included Files
 *********************************************************************************/

#include <nuttx/config.h>

/*********************************************************************************
 * Preprocessor Definitions
 *********************************************************************************/

/* DCDC Register Offsets *********************************************************/
#define IMXRT_DCDC_CTRL0_OFFSET  (0x0000)
#define IMXRT_DCDC_CTRL1_OFFSET  (0x0004)
#define IMXRT_DCDC_REG0_OFFSET   (0x0008)
#define IMXRT_DCDC_REG1_OFFSET   (0x000c)
#define IMXRT_DCDC_REG2_OFFSET   (0x0010)
#define IMXRT_DCDC_REG3_OFFSET   (0x0014)
#define IMXRT_DCDC_REG4_OFFSET   (0x0018)
#define IMXRT_DCDC_REG5_OFFSET   (0x001c)
#define IMXRT_DCDC_REG6_OFFSET   (0x0020)
#define IMXRT_DCDC_REG7_OFFSET   (0x0024)
#define IMXRT_DCDC_REG7P_OFFSET  (0x0028)
#define IMXRT_DCDC_REG8_OFFSET   (0x002c)
#define IMXRT_DCDC_REG9_OFFSET   (0x0030)
#define IMXRT_DCDC_REG10_OFFSET  (0x0034)
#define IMXRT_DCDC_REG11_OFFSET  (0x0038)
#define IMXRT_DCDC_REG12_OFFSET  (0x003c)
#define IMXRT_DCDC_REG13_OFFSET  (0x0040)
#define IMXRT_DCDC_REG14_OFFSET  (0x0044)
#define IMXRT_DCDC_REG15_OFFSET  (0x0048)
#define IMXRT_DCDC_REG16_OFFSET  (0x004c)
#define IMXRT_DCDC_REG17_OFFSET  (0x0050)
#define IMXRT_DCDC_REG18_OFFSET  (0x0054)
#define IMXRT_DCDC_REG19_OFFSET  (0x0058)
#define IMXRT_DCDC_REG20_OFFSET  (0x005c)
#define IMXRT_DCDC_REG21_OFFSET  (0x0060)
#define IMXRT_DCDC_REG22_OFFSET  (0x0064)
#define IMXRT_DCDC_REG23_OFFSET  (0x0068)
#define IMXRT_DCDC_REG24_OFFSET  (0x006c)

/* DCDC Register Addresses *******************************************************/
#define IMXRT_DCDC_CTRL0  (IMXRT_DCDC_BASE + IMXRT_DCDC_CTRL0_OFFSET)
#define IMXRT_DCDC_CTRL1  (IMXRT_DCDC_BASE + IMXRT_DCDC_CTRL1_OFFSET)
#define IMXRT_DCDC_REG0   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG0_OFFSET)
#define IMXRT_DCDC_REG1   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG1_OFFSET)
#define IMXRT_DCDC_REG2   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG2_OFFSET)
#define IMXRT_DCDC_REG3   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG3_OFFSET)
#define IMXRT_DCDC_REG4   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG4_OFFSET)
#define IMXRT_DCDC_REG5   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG5_OFFSET)
#define IMXRT_DCDC_REG6   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG6_OFFSET)
#define IMXRT_DCDC_REG7   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG7_OFFSET)
#define IMXRT_DCDC_REG7P  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG7P_OFFSET)
#define IMXRT_DCDC_REG8   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG8_OFFSET)
#define IMXRT_DCDC_REG9   (IMXRT_DCDC_BASE + IMXRT_DCDC_REG9_OFFSET)
#define IMXRT_DCDC_REG10  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG10_OFFSET)
#define IMXRT_DCDC_REG11  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG11_OFFSET)
#define IMXRT_DCDC_REG12  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG12_OFFSET)
#define IMXRT_DCDC_REG13  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG13_OFFSET)
#define IMXRT_DCDC_REG14  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG14_OFFSET)
#define IMXRT_DCDC_REG15  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG15_OFFSET)
#define IMXRT_DCDC_REG16  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG16_OFFSET)
#define IMXRT_DCDC_REG17  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG17_OFFSET)
#define IMXRT_DCDC_REG18  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG18_OFFSET)
#define IMXRT_DCDC_REG19  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG19_OFFSET)
#define IMXRT_DCDC_REG20  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG20_OFFSET)
#define IMXRT_DCDC_REG21  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG21_OFFSET)
#define IMXRT_DCDC_REG22  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG22_OFFSET)
#define IMXRT_DCDC_REG23  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG23_OFFSET)
#define IMXRT_DCDC_REG24  (IMXRT_DCDC_BASE + IMXRT_DCDC_REG24_OFFSET)

/* DCDC Control Register 0 (CTRL0) */
#define DCDC_CTRL0_ENABLE            (1 << 0)   /* Bit 0: DCDC Enable */
#define DCDC_CTRL0_DIG_EN            (1 << 1)   /* Bit 1: Enable the DCDC_DIG switching converter output */
#define DCDC_CTRL0_STBY_EN           (1 << 2)   /* Bit 2: DCDC standby mode enable */
#define DCDC_CTRL0_LP_MODE_EN        (1 << 3)   /* Bit 3: DCDC low-power (LP) mode enable DCDC can't start up directly into LP mode */
#define DCDC_CTRL0_STBY_LP_MODE_EN   (1 << 4)   /* Bit 4: DCDC low-power mode enable by GPC standby request */
#define DCDC_CTRL0_ENABLE_DCDC_CNT   (1 << 5)   /* Bit 5: Enable internal count for DCDC_OK timeout */
#define DCDC_CTRL0_TRIM_HOLD         (1 << 6)   /* Bit 6: Hold trim input */
#define DCDC_CTRL0_DEBUG_BITS_SHIFT  (19)       /* Bits 19-31: DEBUG_BITS[11:0] */
#define DCDC_CTRL0_DEBUG_BITS_MASK   (0xFFF << DCDC_CTRL0_DEBUG_BITS_SHIFT)
#define DCDC_CTRL0_DEBUG_BITS(n)     (((n) << DCDC_CTRL0_DEBUG_BITS_SHIFT) & DCDC_CTRL0_DEBUG_BITS_MASK)
#define DCDC_CTRL0_CONTROL_MODE      (1 << 31)  /* Bit 31: Control mode */

/* DCDC Control Register 1 (CTRL1) */
#define DCDC_CTRL1_VDD1P8CTRL_TRG_SHIFT       (0)   /* Bits 0-5: Target value of VDD1P8 in buck mode, 25mV each step from 0x00 to 0x1F: */
#define DCDC_CTRL1_VDD1P8CTRL_TRG_MASK        (0x1F << DCDC_CTRL1_VDD1P8CTRL_TRG_SHIFT)
#define DCDC_CTRL1_VDD1P8CTRL_TRG(n)          (((n) << DCDC_CTRL1_VDD1P8CTRL_TRG_SHIFT) & DCDC_CTRL1_VDD1P8CTRL_TRG_MASK)
#define DCDC_CTRL1_VDD1P0CTRL_TRG_SHIFT       (8)   /* Bits 8-13: Target value of VDD1P0 in buck mode, 25mV each step from 0x00 to 0x1F: */
#define DCDC_CTRL1_VDD1P0CTRL_TRG_MASK        (0x1F << DCDC_CTRL1_VDD1P0CTRL_TRG_SHIFT)
#define DCDC_CTRL1_VDD1P0CTRL_TRG(n)          (((n) << DCDC_CTRL1_VDD1P0CTRL_TRG_SHIFT) & DCDC_CTRL1_VDD1P0CTRL_TRG_MASK)
#define DCDC_CTRL1_VDD1P8CTRL_STBY_TRG_SHIFT  (16)  /* Bits 16-21: Target value of VDD1P8 in standby mode, 25mV each step from 0x00 to 0x1F: */
#define DCDC_CTRL1_VDD1P8CTRL_STBY_TRG_MASK   (0x1F << DCDC_CTRL1_VDD1P8CTRL_STBY_TRG_SHIFT)
#define DCDC_CTRL1_VDD1P8CTRL_STBY_TRG(n)     (((n) << DCDC_CTRL1_VDD1P8CTRL_STBY_TRG_SHIFT) & DCDC_CTRL1_VDD1P8CTRL_STBY_TRG_MASK)
#define DCDC_CTRL1_VDD1P0CTRL_STBY_TRG_SHIFT  (24)  /* Bits 24-29: Target value of VDD1P0 in standby mode, 25mV each step from 0x00 to 0x1F: */
#define DCDC_CTRL1_VDD1P0CTRL_STBY_TRG_MASK   (0x1F << DCDC_CTRL1_VDD1P0CTRL_STBY_TRG_SHIFT)
#define DCDC_CTRL1_VDD1P0CTRL_STBY_TRG(n)     (((n) << DCDC_CTRL1_VDD1P0CTRL_STBY_TRG_SHIFT) & DCDC_CTRL1_VDD1P0CTRL_STBY_TRG_MASK)

/* DCDC Register 0 (REG0) */
#define DCDC_REG0_PWD_ZCD                  (1 << 0)   /* Bit 0: Power Down Zero Cross Detection */
#define DCDC_REG0_DISABLE_AUTO_CLK_SWITCH  (1 << 1)   /* Bit 1: Disable Auto Clock Switch */
#define DCDC_REG0_SEL_CLK                  (1 << 2)   /* Bit 2: Select Clock */
#define DCDC_REG0_PWD_OSC_INT              (1 << 3)   /* Bit 3: Power down internal ring oscillator */
#define DCDC_REG0_PWD_CUR_SNS_CMP          (1 << 4)   /* Bit 4: Power down signal of the current detector */
#define DCDC_REG0_CUR_SNS_THRSH_SHIFT      (5)        /* Bits 5-8: Current Sense (detector) Threshold */
#define DCDC_REG0_CUR_SNS_THRSH_MASK       (0x7 << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#define DCDC_REG0_CUR_SNS_THRSH(n)         (((n) << DCDC_REG0_CUR_SNS_THRSH_SHIFT) & DCDC_REG0_CUR_SNS_THRSH_MASK)
#define DCDC_REG0_PWD_OVERCUR_DET          (1 << 8)   /* Bit 8: Power down overcurrent detection comparator */
#define DCDC_REG0_PWD_CMP_DCDC_IN_DET      (1 << 11)  /* Bit 11: Set to "1" to power down the low voltage detection comparator */
#define DCDC_REG0_PWD_HIGH_VDD1P8_DET      (1 << 16)  /* Bit 16: Power Down High Voltage Detection for VDD1P8 */
#define DCDC_REG0_PWD_HIGH_VDD1P0_DET      (1 << 17)  /* Bit 17: Power Down High Voltage Detection for VDD1P0 */
#define DCDC_REG0_LP_HIGH_HYS              (1 << 21)  /* Bit 21: Low Power High Hysteric Value */
#define DCDC_REG0_PWD_CMP_OFFSET           (1 << 26)  /* Bit 26: power down the out-of-range detection comparator */
#define DCDC_REG0_XTALOK_DISABLE           (1 << 27)  /* Bit 27: Disable xtalok detection circuit */
#define DCDC_REG0_XTAL_24M_OK              (1 << 29)  /* Bit 29: 24M XTAL OK */
#define DCDC_REG0_STS_DC_OK                (1 << 31)  /* Bit 31: DCDC Output OK */

/* DCDC Register 1 (REG1) */
#define DCDC_REG1_DM_CTRL                 (1 << 3)   /* Bit 3: DM Control */
#define DCDC_REG1_RLOAD_REG_EN_LPSR       (1 << 4)   /* Bit 4: Load Resistor Enable */
#define DCDC_REG1_VBG_TRIM_SHIFT          (6)        /* Bits 6-11: Trim Bandgap Voltage */
#define DCDC_REG1_VBG_TRIM_MASK           (0x1F << DCDC_REG1_VBG_TRIM_SHIFT)
#define DCDC_REG1_VBG_TRIM(n)             (((n) << DCDC_REG1_VBG_TRIM_SHIFT) & DCDC_REG1_VBG_TRIM_MASK)
#define DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT   (11)       /* Bits 11-13: Low Power Comparator Current Bias */
#define DCDC_REG1_LP_CMP_ISRC_SEL_MASK    (0x3 << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#define DCDC_REG1_LP_CMP_ISRC_SEL(n)      (((n) << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT) & DCDC_REG1_LP_CMP_ISRC_SEL_MASK)
#define DCDC_REG1_LOOPCTRL_CM_HST_THRESH  (1 << 27)  /* Bit 27: Increase Threshold Detection */
#define DCDC_REG1_LOOPCTRL_DF_HST_THRESH  (1 << 28)  /* Bit 28: Increase Threshold Detection */
#define DCDC_REG1_LOOPCTRL_EN_CM_HYST     (1 << 29)  /* Bit 29: Enable hysteresis in switching converter common mode analog comparators */
#define DCDC_REG1_LOOPCTRL_EN_DF_HYST     (1 << 30)  /* Bit 30: Enable hysteresis in switching converter differential mode analog comparators */

/* DCDC Register 2 (REG2) */
#define DCDC_REG2_LOOPCTRL_DC_C_SHIFT         (0)        /* Bits 0-2: Ratio of integral control parameter to proportional control parameter in the switching DCDC converter, and can be used to optimize efficiency and loop response */
#define DCDC_REG2_LOOPCTRL_DC_C_MASK          (0x3 << DCDC_REG2_LOOPCTRL_DC_C_SHIFT)
#define DCDC_REG2_LOOPCTRL_DC_C(n)            (((n) << DCDC_REG2_LOOPCTRL_DC_C_SHIFT) & DCDC_REG2_LOOPCTRL_DC_C_MASK)
#define DCDC_REG2_LOOPCTRL_DC_R_SHIFT         (2)        /* Bits 2-6: Magnitude of proportional control parameter in the switching DCDC converter control loop. */
#define DCDC_REG2_LOOPCTRL_DC_R_MASK          (0xF << DCDC_REG2_LOOPCTRL_DC_R_SHIFT)
#define DCDC_REG2_LOOPCTRL_DC_R(n)            (((n) << DCDC_REG2_LOOPCTRL_DC_R_SHIFT) & DCDC_REG2_LOOPCTRL_DC_R_MASK)
#define DCDC_REG2_LOOPCTRL_DC_FF_SHIFT        (6)        /* Bits 6-9: Two's complement feed forward step in duty cycle in the switching DCDC converter */
#define DCDC_REG2_LOOPCTRL_DC_FF_MASK         (0x7 << DCDC_REG2_LOOPCTRL_DC_FF_SHIFT)
#define DCDC_REG2_LOOPCTRL_DC_FF(n)           (((n) << DCDC_REG2_LOOPCTRL_DC_FF_SHIFT) & DCDC_REG2_LOOPCTRL_DC_FF_MASK)
#define DCDC_REG2_LOOPCTRL_EN_RCSCALE_SHIFT   (9)        /* Bits 9-12: Enable RC Scale */
#define DCDC_REG2_LOOPCTRL_EN_RCSCALE_MASK    (0x7 << DCDC_REG2_LOOPCTRL_EN_RCSCALE_SHIFT)
#define DCDC_REG2_LOOPCTRL_EN_RCSCALE(n)      (((n) << DCDC_REG2_LOOPCTRL_EN_RCSCALE_SHIFT) & DCDC_REG2_LOOPCTRL_EN_RCSCALE_MASK)
#define DCDC_REG2_LOOPCTRL_RCSCALE_THRSH      (1 << 12)  /* Bit 12: Increase the threshold detection for RC scale circuit. */
#define DCDC_REG2_LOOPCTRL_HYST_SIGN          (1 << 13)  /* Bit 13: Invert the sign of the hysteresis in DCDC analog comparators. */
#define DCDC_REG2_BATTMONITOR_EN_BATADJ       (1 << 15)  /* Bit 15: This bit enables the DCDC to improve efficiency and minimize ripple using the information from the BATT_VAL field */
#define DCDC_REG2_BATTMONITOR_BATT_VAL_SHIFT  (16)       /* Bits 16-26: Software should be configured to place the battery voltage in this register measured with an 8-mV LSB resolution through the ADC */
#define DCDC_REG2_BATTMONITOR_BATT_VAL_MASK   (0x3FF << DCDC_REG2_BATTMONITOR_BATT_VAL_SHIFT)
#define DCDC_REG2_BATTMONITOR_BATT_VAL(n)     (((n) << DCDC_REG2_BATTMONITOR_BATT_VAL_SHIFT) & DCDC_REG2_BATTMONITOR_BATT_VAL_MASK)
#define DCDC_REG2_DCM_SET_CTRL                (1 << 28)  /* Bit 28: DCM Set Control */
#define DCDC_REG2_LOOPCTRL_TOGGLE_DIF         (1 << 30)  /* Bit 30: Set high to enable supply stepping to change only after the differential control loop has toggled as well */

/* DCDC Register 3 (REG3) */
#define DCDC_REG3_IN_BROWNOUT               (1 << 14)  /* Bit 14: signal "1" when the voltage on DCDC_IN is lower than 2.6V */
#define DCDC_REG3_OVERVOLT_VDD1P8_DET_OUT   (1 << 15)  /* Bit 15: signal "1" when overvoltage on the VDD1P8 output happens */
#define DCDC_REG3_OVERVOLT_VDD1P0_DET_OUT   (1 << 16)  /* Bit 16: signal "1" when overvoltage on the VDD1P0 output happens */
#define DCDC_REG3_OVERCUR_DETECT_OUT        (1 << 17)  /* Bit 17: signal "1" when overcurrent happens. */
#define DCDC_REG3_ENABLE_FF                 (1 << 18)  /* Bit 18: no description available */
#define DCDC_REG3_DISABLE_PULSE_SKIP        (1 << 19)  /* Bit 19: Disable Pulse Skip */
#define DCDC_REG3_DISABLE_IDLE_SKIP         (1 << 20)  /* Bit 20: no description available */
#define DCDC_REG3_DOUBLE_IBIAS_CMP_LP_LPSR  (1 << 21)  /* Bit 21: no description available */
#define DCDC_REG3_REG_FBK_SEL_SHIFT         (22)       /* Bits 22-24: Select the feedback point of the internal regulator */
#define DCDC_REG3_REG_FBK_SEL_MASK          (0x3 << DCDC_REG3_REG_FBK_SEL_SHIFT)
#define DCDC_REG3_REG_FBK_SEL(n)            (((n) << DCDC_REG3_REG_FBK_SEL_SHIFT) & DCDC_REG3_REG_FBK_SEL_MASK)
#define DCDC_REG3_MINPWR_DC_HALFCLK         (1 << 24)  /* Bit 24: Set DCDC clock to half freqeuncy for continuous mode. */
#define DCDC_REG3_MINPWR_HALF_FETS          (1 << 26)  /* Bit 26: Use half switch FET */
#define DCDC_REG3_MISC_DELAY_TIMING         (1 << 27)  /* Bit 27: Miscellaneous Delay Timing */
#define DCDC_REG3_VDD1P0CTRL_DISABLE_STEP   (1 << 29)  /* Bit 29: Disable Step for VDD1P0 */
#define DCDC_REG3_VDD1P8CTRL_DISABLE_STEP   (1 << 30)  /* Bit 30: Disable Step for VDD1P8 */

/* DCDC Register 4 (REG4) */
#define DCDC_REG4_ENABLE_SP_SHIFT  (0)  /* Bits 0-16: Configures CTRL0[ENABLE] (DCDC Enable) for Setpoints 0-15 */
#define DCDC_REG4_ENABLE_SP_MASK   (0xFFFF << DCDC_REG4_ENABLE_SP_SHIFT)
#define DCDC_REG4_ENABLE_SP(n)     (((n) << DCDC_REG4_ENABLE_SP_SHIFT) & DCDC_REG4_ENABLE_SP_MASK)

/* DCDC Register 5 (REG5) */
#define DCDC_REG5_DIG_EN_SP_SHIFT  (0)  /* Bits 0-16: Configures CTRL0[DIG_EN] (DCDC_DIG Enable) for Setpoints 0-15. Always set these bits to 1. */
#define DCDC_REG5_DIG_EN_SP_MASK   (0xFFFF << DCDC_REG5_DIG_EN_SP_SHIFT)
#define DCDC_REG5_DIG_EN_SP(n)     (((n) << DCDC_REG5_DIG_EN_SP_SHIFT) & DCDC_REG5_DIG_EN_SP_MASK)

/* DCDC Register 6 (REG6) */
#define DCDC_REG6_LP_MODE_SP_SHIFT  (0)  /* Bits 0-16: Configures CTRL0[LP_MODE_EN] (LP Mode Enable) for Setpoints 0-15 */
#define DCDC_REG6_LP_MODE_SP_MASK   (0xFFFF << DCDC_REG6_LP_MODE_SP_SHIFT)
#define DCDC_REG6_LP_MODE_SP(n)     (((n) << DCDC_REG6_LP_MODE_SP_SHIFT) & DCDC_REG6_LP_MODE_SP_MASK)

/* DCDC Register 7 (REG7) */
#define DCDC_REG7_STBY_EN_SP_SHIFT  (0)  /* Bits 0-16: Configures CTRL0[STBY_EN] (Standby Enable) for Setpoints 0-15 */
#define DCDC_REG7_STBY_EN_SP_MASK   (0xFFFF << DCDC_REG7_STBY_EN_SP_SHIFT)
#define DCDC_REG7_STBY_EN_SP(n)     (((n) << DCDC_REG7_STBY_EN_SP_SHIFT) & DCDC_REG7_STBY_EN_SP_MASK)

/* DCDC Register 7 plus (REG7P) */
#define DCDC_REG7P_STBY_LP_MODE_SP_SHIFT  (0)  /* Bits 0-16: Configures CTRL0[STBY_LP_MODE_EN] (LP Mode via GPC Enable) for Setpoints 0-15 */
#define DCDC_REG7P_STBY_LP_MODE_SP_MASK   (0xFFFF << DCDC_REG7P_STBY_LP_MODE_SP_SHIFT)
#define DCDC_REG7P_STBY_LP_MODE_SP(n)     (((n) << DCDC_REG7P_STBY_LP_MODE_SP_SHIFT) & DCDC_REG7P_STBY_LP_MODE_SP_MASK)

/* DCDC Register 8 (REG8) */
#define DCDC_REG8_ANA_TRG_SP0_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P8CTRL_TRG] FOR Setpoints 0-3 */
#define DCDC_REG8_ANA_TRG_SP0_MASK   (0xFFFFFFFF << DCDC_REG8_ANA_TRG_SP0_SHIFT)
#define DCDC_REG8_ANA_TRG_SP0(n)     (((n) << DCDC_REG8_ANA_TRG_SP0_SHIFT) & DCDC_REG8_ANA_TRG_SP0_MASK)

/* DCDC Register 9 (REG9) */
#define DCDC_REG9_ANA_TRG_SP1_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P8CTRL_TRG] FOR Setpoints 4-7 */
#define DCDC_REG9_ANA_TRG_SP1_MASK   (0xFFFFFFFF << DCDC_REG9_ANA_TRG_SP1_SHIFT)
#define DCDC_REG9_ANA_TRG_SP1(n)     (((n) << DCDC_REG9_ANA_TRG_SP1_SHIFT) & DCDC_REG9_ANA_TRG_SP1_MASK)

/* DCDC Register 10 (REG10) */
#define DCDC_REG10_ANA_TRG_SP2_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P8CTRL_TRG] FOR Setpoints 8-11 */
#define DCDC_REG10_ANA_TRG_SP2_MASK   (0xFFFFFFFF << DCDC_REG10_ANA_TRG_SP2_SHIFT)
#define DCDC_REG10_ANA_TRG_SP2(n)     (((n) << DCDC_REG10_ANA_TRG_SP2_SHIFT) & DCDC_REG10_ANA_TRG_SP2_MASK)

/* DCDC Register 11 (REG11) */
#define DCDC_REG11_ANA_TRG_SP3_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P8CTRL_TRG] FOR Setpoints 12-15 */
#define DCDC_REG11_ANA_TRG_SP3_MASK   (0xFFFFFFFF << DCDC_REG11_ANA_TRG_SP3_SHIFT)
#define DCDC_REG11_ANA_TRG_SP3(n)     (((n) << DCDC_REG11_ANA_TRG_SP3_SHIFT) & DCDC_REG11_ANA_TRG_SP3_MASK)

/* DCDC Register 12 (REG12) */
#define DCDC_REG12_DIG_TRG_SP0_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P0CTRL_TRG] FOR Setpoints 0-3 */
#define DCDC_REG12_DIG_TRG_SP0_MASK   (0xFFFFFFFF << DCDC_REG12_DIG_TRG_SP0_SHIFT)
#define DCDC_REG12_DIG_TRG_SP0(n)     (((n) << DCDC_REG12_DIG_TRG_SP0_SHIFT) & DCDC_REG12_DIG_TRG_SP0_MASK)

/* DCDC Register 13 (REG13) */
#define DCDC_REG13_DIG_TRG_SP1_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P0CTRL_TRG] FOR Setpoints 4-7 */
#define DCDC_REG13_DIG_TRG_SP1_MASK   (0xFFFFFFFF << DCDC_REG13_DIG_TRG_SP1_SHIFT)
#define DCDC_REG13_DIG_TRG_SP1(n)     (((n) << DCDC_REG13_DIG_TRG_SP1_SHIFT) & DCDC_REG13_DIG_TRG_SP1_MASK)

/* DCDC Register 14 (REG14) */
#define DCDC_REG14_DIG_TRG_SP2_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P0CTRL_TRG] FOR Setpoints 8-11 */
#define DCDC_REG14_DIG_TRG_SP2_MASK   (0xFFFFFFFF << DCDC_REG14_DIG_TRG_SP2_SHIFT)
#define DCDC_REG14_DIG_TRG_SP2(n)     (((n) << DCDC_REG14_DIG_TRG_SP2_SHIFT) & DCDC_REG14_DIG_TRG_SP2_MASK)

/* DCDC Register 15 (REG15) */
#define DCDC_REG15_DIG_TRG_SP3_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P0CTRL_TRG] FOR Setpoints 12-15 */
#define DCDC_REG15_DIG_TRG_SP3_MASK   (0xFFFFFFFF << DCDC_REG15_DIG_TRG_SP3_SHIFT)
#define DCDC_REG15_DIG_TRG_SP3(n)     (((n) << DCDC_REG15_DIG_TRG_SP3_SHIFT) & DCDC_REG15_DIG_TRG_SP3_MASK)

/* DCDC Register 16 (REG16) */
#define DCDC_REG16_ANA_STBY_TRG_SP0_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P8CTRL_STBY_TRG] FOR Setpoints 0-3 */
#define DCDC_REG16_ANA_STBY_TRG_SP0_MASK   (0xFFFFFFFF << DCDC_REG16_ANA_STBY_TRG_SP0_SHIFT)
#define DCDC_REG16_ANA_STBY_TRG_SP0(n)     (((n) << DCDC_REG16_ANA_STBY_TRG_SP0_SHIFT) & DCDC_REG16_ANA_STBY_TRG_SP0_MASK)

/* DCDC Register 17 (REG17) */
#define DCDC_REG17_ANA_STBY_TRG_SP1_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P8CTRL_STBY_TRG] FOR Setpoints 4-7 */
#define DCDC_REG17_ANA_STBY_TRG_SP1_MASK   (0xFFFFFFFF << DCDC_REG17_ANA_STBY_TRG_SP1_SHIFT)
#define DCDC_REG17_ANA_STBY_TRG_SP1(n)     (((n) << DCDC_REG17_ANA_STBY_TRG_SP1_SHIFT) & DCDC_REG17_ANA_STBY_TRG_SP1_MASK)

/* DCDC Register 18 (REG18) */
#define DCDC_REG18_ANA_STBY_TRG_SP2_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P8CTRL_STBY_TRG] FOR Setpoints 8-11 */
#define DCDC_REG18_ANA_STBY_TRG_SP2_MASK   (0xFFFFFFFF << DCDC_REG18_ANA_STBY_TRG_SP2_SHIFT)
#define DCDC_REG18_ANA_STBY_TRG_SP2(n)     (((n) << DCDC_REG18_ANA_STBY_TRG_SP2_SHIFT) & DCDC_REG18_ANA_STBY_TRG_SP2_MASK)

/* DCDC Register 19 (REG19) */
#define DCDC_REG19_ANA_STBY_TRG_SP3_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P8CTRL_STBY_TRG] FOR Setpoints 12-15 */
#define DCDC_REG19_ANA_STBY_TRG_SP3_MASK   (0xFFFFFFFF << DCDC_REG19_ANA_STBY_TRG_SP3_SHIFT)
#define DCDC_REG19_ANA_STBY_TRG_SP3(n)     (((n) << DCDC_REG19_ANA_STBY_TRG_SP3_SHIFT) & DCDC_REG19_ANA_STBY_TRG_SP3_MASK)

/* DCDC Register 20 (REG20) */
#define DCDC_REG20_DIG_STBY_TRG_SP0_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P0CTRL_STBY_TRG] FOR Setpoints 0-3 */
#define DCDC_REG20_DIG_STBY_TRG_SP0_MASK   (0xFFFFFFFF << DCDC_REG20_DIG_STBY_TRG_SP0_SHIFT)
#define DCDC_REG20_DIG_STBY_TRG_SP0(n)     (((n) << DCDC_REG20_DIG_STBY_TRG_SP0_SHIFT) & DCDC_REG20_DIG_STBY_TRG_SP0_MASK)

/* DCDC Register 21 (REG21) */
#define DCDC_REG21_DIG_STBY_TRG_SP1_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P0CTRL_STBY_TRG] FOR Setpoints 4-7 */
#define DCDC_REG21_DIG_STBY_TRG_SP1_MASK   (0xFFFFFFFF << DCDC_REG21_DIG_STBY_TRG_SP1_SHIFT)
#define DCDC_REG21_DIG_STBY_TRG_SP1(n)     (((n) << DCDC_REG21_DIG_STBY_TRG_SP1_SHIFT) & DCDC_REG21_DIG_STBY_TRG_SP1_MASK)

/* DCDC Register 22 (REG22) */
#define DCDC_REG22_DIG_STBY_TRG_SP2_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P0CTRL_STBY_TRG] FOR Setpoints 8-11 */
#define DCDC_REG22_DIG_STBY_TRG_SP2_MASK   (0xFFFFFFFF << DCDC_REG22_DIG_STBY_TRG_SP2_SHIFT)
#define DCDC_REG22_DIG_STBY_TRG_SP2(n)     (((n) << DCDC_REG22_DIG_STBY_TRG_SP2_SHIFT) & DCDC_REG22_DIG_STBY_TRG_SP2_MASK)

/* DCDC Register 23 (REG23) */
#define DCDC_REG23_DIG_STBY_TRG_SP3_SHIFT  (0)  /* Bits 0-32: Configures CTRL1[VDD1P0CTRL_STBY_TRG] FOR Setpoints 12-15 */
#define DCDC_REG23_DIG_STBY_TRG_SP3_MASK   (0xFFFFFFFF << DCDC_REG23_DIG_STBY_TRG_SP3_SHIFT)
#define DCDC_REG23_DIG_STBY_TRG_SP3(n)     (((n) << DCDC_REG23_DIG_STBY_TRG_SP3_SHIFT) & DCDC_REG23_DIG_STBY_TRG_SP3_MASK)

/* DCDC Register 24 (REG24) */
#define DCDC_REG24_OK_COUNT_SHIFT  (0)  /* Bits 0-32: Internal count for dcdc_ok timeout */
#define DCDC_REG24_OK_COUNT_MASK   (0xFFFFFFFF << DCDC_REG24_OK_COUNT_SHIFT)
#define DCDC_REG24_OK_COUNT(n)     (((n) << DCDC_REG24_OK_COUNT_SHIFT) & DCDC_REG24_OK_COUNT_MASK)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_DCDC_H */
