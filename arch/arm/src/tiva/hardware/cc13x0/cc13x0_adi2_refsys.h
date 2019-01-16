/********************************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_adi2_refsys.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_ADI2_REFSYS_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_ADI2_REFSYS_H

/********************************************************************************************************************
 * Included Files
 ********************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"
#include "hardware/tiva_ddi.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************************/

/* ADI2 REFSYS Register Offsets *************************************************************************************/

#define TIVA_ADI2_REFSYS_REFSYSCTL0_OFFSET                   0x0000
#define TIVA_ADI2_REFSYS_SOCLDOCTL0_OFFSET                   0x0002
#define TIVA_ADI2_REFSYS_SOCLDOCTL1_OFFSET                   0x0003
#define TIVA_ADI2_REFSYS_SOCLDOCTL2_OFFSET                   0x0004
#define TIVA_ADI2_REFSYS_SOCLDOCTL3_OFFSET                   0x0005
#define TIVA_ADI2_REFSYS_SOCLDOCTL4_OFFSET                   0x0006
#define TIVA_ADI2_REFSYS_SOCLDOCTL5_OFFSET                   0x0007
#define TIVA_ADI2_REFSYS_HPOSCCTL0_OFFSET                    0x000a
#define TIVA_ADI2_REFSYS_HPOSCCTL1_OFFSET                    0x000b
#define TIVA_ADI2_REFSYS_HPOSCCTL2_OFFSET                    0x000c

/* ADI2 REFSYS Register Addresses ***********************************************************************************/

#define TIVA_ADI2_REFSYS_REFSYSCTL0                          (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_REFSYSCTL0_OFFSET)
#define TIVA_ADI2_REFSYS_SOCLDOCTL0                          (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_SOCLDOCTL0_OFFSET)
#define TIVA_ADI2_REFSYS_SOCLDOCTL1                          (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_SOCLDOCTL1_OFFSET)
#define TIVA_ADI2_REFSYS_SOCLDOCTL2                          (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_SOCLDOCTL2_OFFSET)
#define TIVA_ADI2_REFSYS_SOCLDOCTL3                          (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_SOCLDOCTL3_OFFSET)
#define TIVA_ADI2_REFSYS_SOCLDOCTL4                          (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_SOCLDOCTL4_OFFSET)
#define TIVA_ADI2_REFSYS_SOCLDOCTL5                          (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_SOCLDOCTL5_OFFSET)
#define TIVA_ADI2_REFSYS_HPOSCCTL0                           (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_HPOSCCTL0_OFFSET)
#define TIVA_ADI2_REFSYS_HPOSCCTL1                           (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_HPOSCCTL1_OFFSET)
#define TIVA_ADI2_REFSYS_HPOSCCTL2                           (TIVA_ADI2_BASE + TIVA_ADI2_REFSYS_HPOSCCTL2_OFFSET)

/* Offsets may also be used in conjunction with access as described in cc13x0_ddi.h */

#define TIVA_ADI2_REFSYS_DIR                                 (TIVA_ADI2_BASE + TIVA_DDI_DIR_OFFSET)
#define TIVA_ADI2_REFSYS_SET                                 (TIVA_ADI2_BASE + TIVA_DDI_SET_OFFSET)
#define TIVA_ADI2_REFSYS_CLR                                 (TIVA_ADI2_BASE + TIVA_DDI_CLR_OFFSET)
#define TIVA_ADI2_REFSYS_MASK4B                              (TIVA_ADI2_BASE + TIVA_DDI_MASK4B_OFFSET)
#define TIVA_ADI2_REFSYS_MASK8B                              (TIVA_ADI2_BASE + TIVA_DDI_MASK8B_OFFSET)
#define TIVA_ADI2_REFSYS_MASK16B                             (TIVA_ADI2_BASE + TIVA_DDI_MASK16B_OFFSET)

/* ADI2 REFSYS Bitfield Definitions *********************************************************************************/

/* TIVA_ADI2_REFSYS_REFSYSCTL0 */

#define ADI2_REFSYS_REFSYSCTL0_TRIM_IREF_SHIFT                 (0)       /* Bit 0-4 */
#define ADI2_REFSYS_REFSYSCTL0_TRIM_IREF_MASK                  (15 << ADI2_REFSYS_REFSYSCTL0_TRIM_IREF_SHIFT)
#  define ADI2_REFSYS_REFSYSCTL0_TRIM_IREF(n)                  ((uint32_t)(n) << ADI2_REFSYS_REFSYSCTL0_TRIM_IREF_SHIFT)

/* TIVA_ADI2_REFSYS_SOCLDOCTL0 */

#define ADI2_REFSYS_SOCLDOCTL0_VTRIM_BOD_SHIFT                 (0)       /* Bits 0-3 */
#define ADI2_REFSYS_SOCLDOCTL0_VTRIM_BOD_MASK                  (15 << ADI2_REFSYS_SOCLDOCTL0_VTRIM_BOD_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL0_VTRIM_BOD(n)                  ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL0_VTRIM_BOD_SHIFT)
#define ADI2_REFSYS_SOCLDOCTL0_VTRIM_UDIG_SHIFT                (4)       /* Bits 4-7 */
#define ADI2_REFSYS_SOCLDOCTL0_VTRIM_UDIG_MASK                 (15 << ADI2_REFSYS_SOCLDOCTL0_VTRIM_UDIG_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL0_VTRIM_UDIG(n)                 ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL0_VTRIM_UDIG_SHIFT)

/* TIVA_ADI2_REFSYS_SOCLDOCTL1 */

#define ADI2_REFSYS_SOCLDOCTL1_VTRIM_DIG_SHIFT                 (0)       /* Bits 0-3 */
#define ADI2_REFSYS_SOCLDOCTL1_VTRIM_DIG_MASK                  (15 << ADI2_REFSYS_SOCLDOCTL1_VTRIM_DIG_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL1_VTRIM_DIG(n)                  ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL1_VTRIM_DIG_SHIFT)
#define ADI2_REFSYS_SOCLDOCTL1_VTRIM_COARSE_SHIFT              (4)       /* Bit 4-7 */
#define ADI2_REFSYS_SOCLDOCTL1_VTRIM_COARSE_MASK               (15 << ADI2_REFSYS_SOCLDOCTL1_VTRIM_COARSE_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL1_VTRIM_COARSE(n)               ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL1_VTRIM_COARSE_SHIFT)

/* TIVA_ADI2_REFSYS_SOCLDOCTL2 */

#define ADI2_REFSYS_SOCLDOCTL2_VTRIM_DELTA_SHIFT               (0)       /* Bits 0-2 */
#define ADI2_REFSYS_SOCLDOCTL2_VTRIM_DELTA_MASK                (7 << ADI2_REFSYS_SOCLDOCTL2_VTRIM_DELTA_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL2_VTRIM_DELTA(n)                ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL2_VTRIM_DELTA_SHIFT)

/* TIVA_ADI2_REFSYS_SOCLDOCTL3 */

#define ADI2_REFSYS_SOCLDOCTL3_ITRIM_UDIGLDO_SHIFT             (0)       /* Bits 0-2 */
#define ADI2_REFSYS_SOCLDOCTL3_ITRIM_UDIGLDO_MASK              (7 << ADI2_REFSYS_SOCLDOCTL3_ITRIM_UDIGLDO_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL3_ITRIM_UDIGLDO(n)              ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL3_ITRIM_UDIGLDO_SHIFT)
#define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_SHIFT              (3)       /* Bits 3-5 */
#define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_MASK               (7 << ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO(n)               ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_BIAS_60P         (0 << ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_BIAS_80P         (3 << ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_BIAS_100P        (5 << ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_BIAS_120P        (7 << ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_SHIFT)
#define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_LOAD_SHIFT         (6)       /* Bits 6-7 */
#define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_LOAD_MASK          (3 << ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_LOAD_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_LOAD(n)          ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_LOAD_SHIFT)

/* TIVA_ADI2_REFSYS_SOCLDOCTL4 */

#define ADI2_REFSYS_SOCLDOCTL4_UDIG_LDO_EN                     (1 << 0)  /* Bit 0 */
#define ADI2_REFSYS_SOCLDOCTL4_BIAS_DIS                        (1 << 1)  /* Bit 1 */
#define ADI2_REFSYS_SOCLDOCTL4_DIG_ITEST_EN_SHIFT              (2)       /* Bits 2-4 */
#define ADI2_REFSYS_SOCLDOCTL4_DIG_ITEST_EN_MASK               (7 << ADI2_REFSYS_SOCLDOCTL4_DIG_ITEST_EN_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL4_DIG_ITEST_EN(n)               ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL4_DIG_ITEST_EN_SHIFT)
#define ADI2_REFSYS_SOCLDOCTL4_UDIG_ITEST_EN_SHIFT             (5)       /* Bits 5-6 */
#define ADI2_REFSYS_SOCLDOCTL4_UDIG_ITEST_EN_MASK              (3 << ADI2_REFSYS_SOCLDOCTL4_UDIG_ITEST_EN_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL4_UDIG_ITEST_EN(n)              ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL4_UDIG_ITEST_EN_SHIFT)

/* TIVA_ADI2_REFSYS_SOCLDOCTL5 */

#define ADI2_REFSYS_SOCLDOCTL5_TESTSEL_SHIFT                   (0)       /* Bits 0-2 */
#define ADI2_REFSYS_SOCLDOCTL5_TESTSEL_MASK                    (7 << ADI2_REFSYS_SOCLDOCTL5_TESTSEL_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL5_TESTSEL(n)                    ((uint32_t)(n) << ADI2_REFSYS_SOCLDOCTL5_TESTSEL_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL5_TESTSEL_NC                    (0 << ADI2_REFSYS_SOCLDOCTL5_TESTSEL_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL5_TESTSEL_ITEST                 (1 << ADI2_REFSYS_SOCLDOCTL5_TESTSEL_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL5_TESTSEL_VREF_AMP              (2 << ADI2_REFSYS_SOCLDOCTL5_TESTSEL_SHIFT)
#  define ADI2_REFSYS_SOCLDOCTL5_TESTSEL_VDD_AON               (4 << ADI2_REFSYS_SOCLDOCTL5_TESTSEL_SHIFT)
#define ADI2_REFSYS_SOCLDOCTL5_IMON_ITEST_EN                   (1 << 3)  /* Bit 3 */

/* TIVA_ADI2_REFSYS_HPOSCCTL0 */

#define ADI2_REFSYS_HPOSCCTL0_DIV3_BYPASS                      (1 << 0)  /* Bit 0 */
#  define ADI2_REFSYS_HPOSCCTL0_DIV3_BYPASS_HPOSC_840MHZ       (0)
#  define ADI2_REFSYS_HPOSCCTL0_DIV3_BYPASS_HPOSC_2520MHZ      ADI2_REFSYS_HPOSCCTL0_DIV3_BYPASS
#define ADI2_REFSYS_HPOSCCTL0_SERIES_CAP_SHIFT                 (1)       /* Bits 1-2 */
#define ADI2_REFSYS_HPOSCCTL0_SERIES_CAP_MASK                  (3 << ADI2_REFSYS_HPOSCCTL0_SERIES_CAP_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_SERIES_CAP(n)                  ((uint32_t)(n) << ADI2_REFSYS_HPOSCCTL0_SERIES_CAP_SHIFT)
#define ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT                   (3)       /* Bits 3-4 */
#define ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_MASK                    (3 << ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_TUNE_CAP(n)                    ((uint32_t)(n) << ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT_0               (0 << ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT_M35             (1 << ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT_M70             (2 << ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT_M108            (3 << ADI2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT)
#define ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_SHIFT          (5)       /* Bits 5-6 */
#define ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MASK           (3 << ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY(n)           ((uint32_t)(n) << ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MIN_DLY_X1   (0 << ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MIN_DLY_X2   (1 << ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MIN_DLY_X4   (2 << ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MIN_DLY_X8   (3 << ADI2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_SHIFT)
#define ADI2_REFSYS_HPOSCCTL0_FILTER_EN                        (1 << 7)  /* Bit 7 */

/* TIVA_ADI2_REFSYS_HPOSCCTL1 */

#define ADI2_REFSYS_HPOSCCTL1_BIAS_RES_SET_SHIFT               (0)       /* Bits 0-3 */
#define ADI2_REFSYS_HPOSCCTL1_BIAS_RES_SET_MASK                (15 << ADI2_REFSYS_HPOSCCTL1_BIAS_RES_SET_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL1_BIAS_RES_SET(n)                ((uint32_t)(n) << ADI2_REFSYS_HPOSCCTL1_BIAS_RES_SET_SHIFT)
#define ADI2_REFSYS_HPOSCCTL1_PWRDET_EN                        (1 << 4)  /* Bit 4 */
#define ADI2_REFSYS_HPOSCCTL1_BIAS_DIS                         (1 << 5)  /* Bit 5 */

/* TIVA_ADI2_REFSYS_HPOSCCTL2 */

#define ADI2_REFSYS_HPOSCCTL2_CURRMIRR_RATIO_SHIFT             (0)       /* Bits 0-3 */
#define ADI2_REFSYS_HPOSCCTL2_CURRMIRR_RATIO_MASK              (15 << ADI2_REFSYS_HPOSCCTL2_CURRMIRR_RATIO_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL2_CURRMIRR_RATIO(n)              ((uint32_t)(n) << ADI2_REFSYS_HPOSCCTL2_CURRMIRR_RATIO_SHIFT)
#define ADI2_REFSYS_HPOSCCTL2_ATEST_SEL_SHIFT                  (4)       /* Bits 4-5 */
#define ADI2_REFSYS_HPOSCCTL2_ATEST_SEL_MASK                   (3 << ADI2_REFSYS_HPOSCCTL2_ATEST_SEL_SHIFT)
#  define ADI2_REFSYS_HPOSCCTL2_ATEST_SEL(n)                   ((uint32_t)(n) << ADI2_REFSYS_HPOSCCTL2_ATEST_SEL_SHIFT)
#define ADI2_REFSYS_HPOSCCTL2_TESTMUX_EN                       (1 << 6)  /* Bit 6 */
#define ADI2_REFSYS_HPOSCCTL2_BIAS_HOLD_MODE_EN                (1 << 7)  /* Bit 7 */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_ADI2_REFSYS_H */
