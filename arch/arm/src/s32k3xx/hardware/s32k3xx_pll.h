/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_pll.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PLL_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PLL Register Offsets *****************************************************/

#define S32K3XX_PLL_CR_OFFSET    (0x00) /* PLL Control Register (PLLCR) */
#define S32K3XX_PLL_SR_OFFSET    (0x04) /* PLL Status Register (PLLSR) */
#define S32K3XX_PLL_DV_OFFSET    (0x08) /* PLL Divider Register (PLLDV) */
#define S32K3XX_PLL_FM_OFFSET    (0x0c) /* PLL Frequency Modulation Register (PLLFM) */
#define S32K3XX_PLL_FD_OFFSET    (0x10) /* PLL Fractional Divider Register (PLLFD) */
#define S32K3XX_PLL_CAL2_OFFSET  (0x18) /* PLL Calibration Register 2 (PLLCAL2) */
#define S32K3XX_PLL_ODIV0_OFFSET (0x80) /* PLL Output Divider 0 Register (PLLODIV0) */
#define S32K3XX_PLL_ODIV1_OFFSET (0x84) /* PLL Output Divider 1 Register (PLLODIV1) */

/* PLL Register Addresses ***************************************************/

#define S32K3XX_PLL_CR           (S32K3XX_PLL_BASE + S32K3XX_PLL_CR_OFFSET)
#define S32K3XX_PLL_SR           (S32K3XX_PLL_BASE + S32K3XX_PLL_SR_OFFSET)
#define S32K3XX_PLL_DV           (S32K3XX_PLL_BASE + S32K3XX_PLL_DV_OFFSET)
#define S32K3XX_PLL_FM           (S32K3XX_PLL_BASE + S32K3XX_PLL_FM_OFFSET)
#define S32K3XX_PLL_FD           (S32K3XX_PLL_BASE + S32K3XX_PLL_FD_OFFSET)
#define S32K3XX_PLL_CAL2         (S32K3XX_PLL_BASE + S32K3XX_PLL_CAL2_OFFSET)
#define S32K3XX_PLL_ODIV0        (S32K3XX_PLL_BASE + S32K3XX_PLL_ODIV0_OFFSET)
#define S32K3XX_PLL_ODIV1        (S32K3XX_PLL_BASE + S32K3XX_PLL_ODIV1_OFFSET)

/* PLL Register Bitfield Definitions ****************************************/

/* PLL Control Register (PLLCR) */

                                           /* Bits 0-30: Reserved */
#define PLL_CR_PLLPD             (1 << 31) /* Bit 31: PLL Power Down (PLLPD) */
#  define PLL_CR_PLLPU           (0 << 31) /*         PLL Power Up */

/* PLL Status Register (PLLSR) */

                                           /* Bits 0-1: Reserved */
#define PLL_SR_LOCK              (1 << 2)  /* Bit 2: Lock Status (LOCK) */
#define PLL_SR_LOL               (1 << 3)  /* Bit 3: Loss-Of-Lock Flag (LOL) */
                                           /* Bits 4-31: Reserved */

/* PLL Divider Register (PLLDV) */

#define PLL_DV_MFI_SHIFT         (0)       /* Bits 0-7: PLL feedback loop divider (MFI) */
#define PLL_DV_MFI_MASK          (0xff << PLL_DV_MFI_SHIFT)
#define PLL_DV_MFI(n)            (((n) << PLL_DV_MFI_SHIFT) & PLL_DV_MFI_MASK)
                                           /* Bits 8-11: Reserved */
#define PLL_DV_RDIV_SHIFT        (12)      /* Bits 12-14: Input clock predivider (RDIV) */
#define PLL_DV_RDIV_MASK         (0x07 << PLL_DV_RDIV_SHIFT)
#  define PLL_DV_RDIV_DIV(n)     ((n) << PLL_DV_RDIV_SHIFT) /* Divide by n=1..7 */

                                           /* Bits 15-24: Reserved */
#define PLL_DV_ODIV2_SHIFT       (25)      /* Bits 25-30: Output frequency divider for raw PLL clock (ODIV2) */
#define PLL_DV_ODIV2_MASK        (0x3f << PLL_DV_ODIV2_SHIFT)
#  define PLL_DV_ODIV2_DIV(n)    ((n) << PLL_DV_ODIV2_SHIFT) /* Divide by n=1..63 */

                                           /* Bit 31: Reserved */

/* PLL Frequency Modulation Register (PLLFM) */

#define PLL_FM_STEPNO_SHIFT      (0)       /* Bits 0-10: Number of steps of modulation period or frequency modulation (STEPNO) */
#define PLL_FM_STEPNO_MASK       (0x07ff << PLL_FM_STEPNO_SHIFT)
                                           /* Bits 11-15: Reserved */
#define PLL_FM_STEPSIZE_SHIFT    (16)      /* Bits 16-25: Frequency modulation step size (STEPSIZE) */
#define PLL_FM_STEPSIZE_MASK     (0x03ff << PLL_FM_STEPSIZE_SHIFT)
                                           /* Bits 26-28: Reserved */
#define PLL_FM_SPREADCTL         (1 << 29) /* Bit 29: Modulation Type Selection (SPREADCTL) */
#define PLL_FM_SSCGBYP           (1 << 30) /* Bit 30: Frequency Modulation (Spread Spectrum Clock Generation) Bypass (SSCGBYP) */
                                           /* Bit 31: Reserved */

/* PLL Fractional Divider Register (PLLFD) */

#define PLL_FD_MFN_SHIFT         (0)       /* Bits 0-14: Numerator of fractional loop division factor (MFN) */
#define PLL_FD_MFN_MASK          (0x7fff << PLL_FD_MFN_SHIFT)
                                           /* Bits 15-27: Reserved */
#define PLL_FD_SDM3              (1 << 28) /* Bit 28: Fractional Mode Configuration (SDM3) */
#define PLL_FD_SDM2              (1 << 29) /* Bit 29: Fractional Mode Configuration (SDM2) */
#define PLL_FD_SDMEN             (1 << 30) /* Bit 30: Fractional Mode Enable (SDMEN) */
                                           /* Bit 31: Reserved */

/* PLL Calibration Register 2 (PLLCAL2) */

                                           /* Bits 0-6: Reserved */
#define PLL_CAL2_ULKCTL_SHIFT    (7)       /* Bits 7-8: Unlock Control Accuracy (ULKCTL) */
#define PLL_CAL2_ULKCTL_MASK     (0x03 << PLL_CAL2_ULKCTL_SHIFT)
#  define PLL_CAL2_ULKCTL_EV9    (0x00 << PLL_CAL2_ULKCTL_SHIFT) /* Expected value +- 9 */
#  define PLL_CAL2_ULKCTL_EV17   (0x01 << PLL_CAL2_ULKCTL_SHIFT) /* Expected value +- 17 */
#  define PLL_CAL2_ULKCTL_EV33   (0x02 << PLL_CAL2_ULKCTL_SHIFT) /* Expected value +- 33 */
#  define PLL_CAL2_ULKCTL_EV5    (0x03 << PLL_CAL2_ULKCTL_SHIFT) /* Expected value +- 5 */

                                           /* Bits 9-31: Reserved */

/* PLL Output Divider n=0..1 Register (PLLODIVn) */

                                           /* Bits 0-15: Reserved */
#define PLL_ODIV_DIV_SHIFT       (16)      /* Bits 16-19: Division Value (DIV) */
#define PLL_ODIV_DIV_MASK        (0x0f << PLL_ODIV_DIV_SHIFT)
#define PLL_ODIV_DIV(n)          (((n-1) << PLL_ODIV_DIV_SHIFT) & PLL_ODIV_DIV_MASK)
                                           /* DIV + 1 times the time period of the divider input clock */
                                           /* Bits 20-30: Reserved */
#define PLL_ODIV_DE              (1 << 31) /* Bit 31: Divider Enable (DE) */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PLL_H */
