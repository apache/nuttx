/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx93/imx93_pll.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_PLL_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_PLL_H

/* All registers besides STATUS have SET, CLR, TGL and VAL shadow registers */

#define PLL_REG_VAL_OFFSET         (0x00)
#define PLL_REG_SET_OFFSET         (0x04)
#define PLL_REG_CLR_OFFSET         (0x08)
#define PLL_REG_TGL_OFFSET         (0x0c)

/* User can access the individual registers via these macros */

#define PLL_VAL(n)                 ((n) + PLL_REG_VAL_OFFSET) /* Same as the register itself */
#define PLL_SET(n)                 ((n) + PLL_REG_SET_OFFSET)
#define PLL_CLR(n)                 ((n) + PLL_REG_CLR_OFFSET)
#define PLL_TGL(n)                 ((n) + PLL_REG_TGL_OFFSET)

/* Common offsets for all PLL registers, existence depends on the register
 * itself
 */

#define PLL_CTRL_OFFSET            (0x00) /* PLL Control */
#define PLL_SPREAD_SPECTRUM_OFFSET (0x30) /* Spread Spectrum */
#define PLL_NUMERATOR_OFFSET       (0x40) /* Numerator */
#define PLL_DENOMINATOR_OFFSET     (0x50) /* Denominator */
#define PLL_DIV_OFFSET             (0x60) /* PLL Dividers */
#define PLL_DFS_CTRL_0_OFFSET      (0x70) /* DFS Control */
#define PLL_DFS_DIV_0_OFFSET       (0x80) /* DFS Division_0 */
#define PLL_DFS_CTRL_1_OFFSET      (0x90) /* DFS Control */
#define PLL_DFS_DIV_1_OFFSET       (0xa0) /* DFS Division_1 */
#define PLL_DFS_CTRL_2_OFFSET      (0xb0) /* DFS Control */
#define PLL_DFS_DIV_2_OFFSET       (0xc0) /* DFS Division_2 */
#define PLL_PLL_STATUS_OFFSET      (0xf0) /* PLL Status */
#define PLL_DFS_STATUS_OFFSET      (0xf4) /* DFS Status */

/* Register addresses */

#define PLL_CTRL(n)                ((n) + PLL_CTRL_OFFSET)
#define PLL_SPREAD_SPECTRUM(n)     ((n) + PLL_SPREAD_SPECTRUM_OFFSET)
#define PLL_NUMERATOR(n)           ((n) + PLL_NUMERATOR_OFFSET)
#define PLL_DENOMINATOR(n)         ((n) + PLL_DENOMINATOR_OFFSET)
#define PLL_DIV(n)                 ((n) + PLL_DIV_OFFSET)
#define PLL_DFS_CTRL_0(n)          ((n) + PLL_DFS_CTRL_0_OFFSET)
#define PLL_DFS_DIV_0(n)           ((n) + PLL_DFS_DIV_0_OFFSET)
#define PLL_DFS_CTRL_1(n)          ((n) + PLL_DFS_CTRL_1_OFFSET)
#define PLL_DFS_DIV_1(n)           ((n) + PLL_DFS_DIV_1_OFFSET)
#define PLL_DFS_CTRL_2(n)          ((n) + PLL_DFS_CTRL_2_OFFSET)
#define PLL_DFS_DIV_2(n)           ((n) + PLL_DFS_DIV_2_OFFSET)
#define PLL_PLL_STATUS(n)          ((n) + PLL_PLL_STATUS_OFFSET)
#define PLL_DFS_STATUS(n)          ((n) + PLL_DFS_STATUS_OFFSET)

/* SYSPLL registers */

#define SYSPLL_CTRL                (IMX9_SYSPLL_BASE + PLL_CTRL_OFFSET)
#define SYSPLL_SPREAD_SPECTRUM     (IMX9_SYSPLL_BASE + PLL_SPREAD_SPECTRUM_OFFSET)
#define SYSPLL_NUMERATOR           (IMX9_SYSPLL_BASE + PLL_NUMERATOR_OFFSET)
#define SYSPLL_DENOMINATOR         (IMX9_SYSPLL_BASE + PLL_DENOMINATOR_OFFSET)
#define SYSPLL_DIV                 (IMX9_SYSPLL_BASE + PLL_DIV_OFFSET)
#define SYSPLL_DFS_CTRL_0          (IMX9_SYSPLL_BASE + PLL_DFS_CTRL_0_OFFSET)
#define SYSPLL_DFS_DIV_0           (IMX9_SYSPLL_BASE + PLL_DFS_DIV_0_OFFSET)
#define SYSPLL_DFS_CTRL_1          (IMX9_SYSPLL_BASE + PLL_DFS_CTRL_1_OFFSET)
#define SYSPLL_DFS_DIV_1           (IMX9_SYSPLL_BASE + PLL_DFS_DIV_1_OFFSET)
#define SYSPLL_DFS_CTRL_2          (IMX9_SYSPLL_BASE + PLL_DFS_CTRL_2_OFFSET)
#define SYSPLL_DFS_DIV_2           (IMX9_SYSPLL_BASE + PLL_DFS_DIV_2_OFFSET)
#define SYSPLL_PLL_STATUS          (IMX9_SYSPLL_BASE + PLL_PLL_STATUS_OFFSET)
#define SYSPLL_DFS_STATUS          (IMX9_SYSPLL_BASE + PLL_DFS_STATUS_OFFSET)

/* ARMPLL registers */

#define ARMPLL_CTRL                (IMX9_ARMPLL_BASE + PLL_CTRL_OFFSET)
#define ARMPLL_DIV                 (IMX9_ARMPLL_BASE + PLL_DIV_OFFSET)
#define ARMPLL_PLL_STATUS          (IMX9_ARMPLL_BASE + PLL_PLL_STATUS_OFFSET)

/* AUDIOPLL registers */

#define AUDIOPLL_CTRL              (IMX9_AUDIOPLL_BASE + PLL_CTRL_OFFSET)
#define AUDIOPLL_SPREAD_SPECTRUM   (IMX9_AUDIOPLL_BASE + PLL_SPREAD_SPECTRUM_OFFSET)
#define AUDIOPLL_NUMERATOR         (IMX9_AUDIOPLL_BASE + PLL_NUMERATOR_OFFSET)
#define AUDIOPLL_DENOMINATOR       (IMX9_AUDIOPLL_BASE + PLL_DENOMINATOR_OFFSET)
#define AUDIOPLL_DIV               (IMX9_AUDIOPLL_BASE + PLL_DIV_OFFSET)
#define AUDIOPLL_PLL_STATUS        (IMX9_AUDIOPLL_BASE + PLL_PLL_STATUS_OFFSET)

/* DRAMPLL registers */

#define DRAMPLL_CTRL               (IMX9_AUDIOPLL_BASE + PLL_CTRL_OFFSET)
#define DRAMPLL_SPREAD_SPECTRUM    (IMX9_AUDIOPLL_BASE + PLL_SPREAD_SPECTRUM_OFFSET)
#define DRAMPLL_NUMERATOR          (IMX9_AUDIOPLL_BASE + PLL_NUMERATOR_OFFSET)
#define DRAMPLL_DENOMINATOR        (IMX9_AUDIOPLL_BASE + PLL_DENOMINATOR_OFFSET)
#define DRAMPLL_DIV                (IMX9_AUDIOPLL_BASE + PLL_DIV_OFFSET)
#define DRAMPLL_PLL_STATUS         (IMX9_AUDIOPLL_BASE + PLL_PLL_STATUS_OFFSET)

/* VIDEOPLL registers */

#define VIDEOPLL_CTRL              (IMX9_VIDEOPLL_BASE + PLL_CTRL_OFFSET)
#define VIDEOPLL_SPREAD_SPECTRUM   (IMX9_VIDEOPLL_BASE + PLL_SPREAD_SPECTRUM_OFFSET)
#define VIDEOPLL_NUMERATOR         (IMX9_VIDEOPLL_BASE + PLL_NUMERATOR_OFFSET)
#define VIDEOPLL_DENOMINATOR       (IMX9_VIDEOPLL_BASE + PLL_DENOMINATOR_OFFSET)
#define VIDEOPLL_DIV               (IMX9_VIDEOPLL_BASE + PLL_DIV_OFFSET)
#define VIDEOPLL_PLL_STATUS        (IMX9_VIDEOPLL_BASE + PLL_PLL_STATUS_OFFSET)

/* PLL Control (CTRL) */

#define PLL_CTRL_POWERUP        (1 << 0)  /* Bit 0: Power up PLL */
#define PLL_CTRL_CLKMUX_EN      (1 << 1)  /* Bit 1: Enable CLKMUX output */
#define PLL_CTRL_CLKMUX_BYPASS  (1 << 2)  /* Bit 2: Enable CLKMUX bypass */
#define PLL_CTRL_SPREADCTL      (1 << 8)  /* Bit 8: Modulation Type Select */
#define PLL_CTRL_HW_CTRL_SEL    (1 << 16) /* Bit 16: Hardware Control Select */
#define PLL_CTRL_LOCK_BYPASS    (1 << 31) /* Bit 31: Lock bypass */

/* Spread Spectrum (SPREAD_SPECTRUM) */

#define PLL_SPREAD_SPECTRUM_STEP_SHIFT (0)       /* Bits 14-0: Set spread spectrum step */
#define PLL_SPREAD_SPECTRUM_STEP_MASK  (0x7fff << PLL_SPREAD_SPECTRUM_STEP_SHIFT)
#define PLL_SPREAD_SPECTRUM_STEP(n)    (((n) << PLL_SPREAD_SPECTRUM_STEP_SHIFT) & PLL_SPREAD_SPECTRUM_STEP_MASK)
#define PLL_SPREAD_SPECTRUM_ENABLE     (1 << 15) /* Bit 15: Enable spread spectrum */
#define PLL_SPREAD_SPECTRUM_STOP_SHIFT (16)      /* Bits 16-31: Set spread spectrum stop */
#define PLL_SPREAD_SPECTRUM_STOP_MASK  (0xffff << PLL_SPREAD_SPECTRUM_STOP_SHIFT)
#define PLL_SPREAD_SPECTRUM_STOP(n)    (((n) << PLL_SPREAD_SPECTRUM_STOP_SHIFT) & PLL_SPREAD_SPECTRUM_STOP_MASK)

/* Numerator (NUMERATOR) */

#define PLL_NUMERATOR_MFN_SHIFT (2) /* Bits 2-31: Numerator MFN value */
#define PLL_NUMERATOR_MFN_MASK  (0x3fffffff << PLL_NUMERATOR_MFN_SHIFT)
#define PLL_NUMERATOR_MFN(n)    (((n) << PLL_NUMERATOR_MFN_SHIFT) & PLL_NUMERATOR_MFN_MASK)

/* Denominator (DENOMINATOR) */

#define PLL_DENOMINATOR_MFD_SHIFT (0) /* Bits 0-29: Denominator MFD value */
#define PLL_DENOMINATOR_MFD_MASK  (0x3fffffff << PLL_DENOMINATOR_MFD_SHIFT)
#define PLL_DENOMINATOR_MFD(n)    (((n) << PLL_DENOMINATOR_MFD_SHIFT) & PLL_DENOMINATOR_MFD_MASK)

/* PLL Dividers (DIV) */

#define PLL_DIV_ODIV_SHIFT (0)  /* Bits 0-7: Output Frequency Divider for Clock Output */
#define PLL_DIV_ODIV_MASK  (0xff << PLL_DIV_ODIV_SHIFT)
#define PLL_DIV_ODIV(n)    (((n) << PLL_DIV_ODIV_SHIFT) & PLL_DIV_ODIV_MASK)
#define PLL_DIV_RDIV_SHIFT (13) /* Bits 13-15: Input Clock Predivider */
#define PLL_DIV_RDIV_MASK  (0x7 << PLL_DIV_RDIV_SHIFT)
#define PLL_DIV_RDIV(n)    (((n) << PLL_DIV_RDIV_SHIFT) & PLL_DIV_RDIV_MASK)
#define PLL_DIV_MFI_SHIFT  (16) /* Bits 16-24: Integer Portion of Loop Divider */
#define PLL_DIV_MFI_MASK   (0x1ff << PLL_DIV_MFI_SHIFT)
#define PLL_DIV_MFI(n)     (((n) << PLL_DIV_MFI_SHIFT) & PLL_DIV_MFI_MASK)

/* DFS Control (DFS_CTRL_0 - DFS_CTRL_2) */

#define PLL_DFS_HW_CTRL_SEL      (1 << 16) /* Bit 16: Hardware Control Select */
#define PLL_DFS_BYPASS_EN        (1 << 23) /* Bit 23: Bypass Enable */
#define PLL_DFS_CLKOUT_DIVBY2_EN (1 << 29) /* Bit 29: DFS Clock Output Divide by 2 Enable */
#define PLL_DFS_CLKOUT_EN        (1 << 30) /* Bit 30: DFS Clock Output Enable */
#define PLL_DFS_ENABLE           (1 << 31) /* Bit 31: DFS Block Enable */

/* DFS Division_a (DFS_DIV_0 - DFS_DIV_2) */

#define PLL_DFS_MFN_SHIFT        (0) /* Bits 0-2: MFN */
#define PLL_DFS_MFN_MASK         (0x7 << PLL_DFS_MFN_SHIFT)
#define PLL_DFS_MFN(n)           (((n) << PLL_DFS_MFN_SHIFT) & PLL_DFS_MFN_MASK)
#define PLL_DFS_MFI_SHIFT        (8) /* Bits 8-15: MFI */
#define PLL_DFS_MFI_MASK         (0xff << PLL_DFS_MFI_SHIFT)
#define PLL_DFS_MFI(n)           (((n) << PLL_DFS_MFI_SHIFT) & PLL_DFS_MFI_MASK)

/* PLL Dividers (DIV) */

#define PLL_PLL_STATUS_PLL_LOCK      (1 << 0) /* Bit 0: PLL is locked */
#define PLL_PLL_STATUS_PLL_LOL       (1 << 1) /* Bit 1: PLL lock is lost */
#define PLL_PLL_STATUS_ANA_MFN_SHIFT (2)
#define PLL_PLL_STATUS_ANA_MFN_MASK  (0x3fffffff << PLL_PLL_STATUS_ANA_MFN_SHIFT)
#define PLL_PLL_STATUS_ANA_MFN(n)    (((n) << PLL_PLL_STATUS_ANA_MFN_SHIFT) & PLL_PLL_STATUS_ANA_MFN_MASK)

/* DFS Status (DFS_STATUS) */

#define PLL_DFS_STATUS_DFS_OK_SHIFT (0) /* Bits 0-2: DFS OK status */
#define PLL_DFS_STATUS_DFS_OK_MASK  (0x7 << PLL_DFS_STATUS_DFS_OK_SHIFT)
#define PLL_DFS_STATUS_DFS_OK(n)    (((n) << PLL_DFS_STATUS_DFS_OK_SHIFT) & PLL_DFS_STATUS_DFS_OK_MASK)

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_PLL_H_*/
