/********************************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_aon_batmon.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_BATMON_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_BATMON_H

/********************************************************************************************************************
 * Included Files
 ********************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************************/

/* AON BATMON Register Offsets **************************************************************************************/

#define TIVA_AON_BATMON_CTL_OFFSET                      0x0000
#define TIVA_AON_BATMON_MEASCFG_OFFSET                  0x0004
#define TIVA_AON_BATMON_TEMPP0_OFFSET                   0x000c
#define TIVA_AON_BATMON_TEMPP1_OFFSET                   0x0010
#define TIVA_AON_BATMON_TEMPP2_OFFSET                   0x0014
#define TIVA_AON_BATMON_BATMONP0_OFFSET                 0x0018
#define TIVA_AON_BATMON_BATMONP1_OFFSET                 0x001c
#define TIVA_AON_BATMON_IOSTRP0_OFFSET                  0x0020
#define TIVA_AON_BATMON_FLASHPUMPP0_OFFSET              0x0024
#define TIVA_AON_BATMON_BAT_OFFSET                      0x0028  /* Last Measured Battery Voltage */
#define TIVA_AON_BATMON_BATUPD_OFFSET                   0x002c  /* Battery Update */
#define TIVA_AON_BATMON_TEMP_OFFSET                     0x0030  /* Temperature */
#define TIVA_AON_BATMON_TEMPUPD_OFFSET                  0x0034  /* Temperature Update */
#define TIVA_AON_BATMON_EVENTMASK_OFFSET                0x0048  /* Event Mask */
#define TIVA_AON_BATMON_EVENT_OFFSET                    0x004c  /* Event */
#define TIVA_AON_BATMON_BATUL_OFFSETT                   0x0050  /* Battery Upper Limit */
#define TIVA_AON_BATMON_BATLL_OFFSETT                   0x0054  /* Battery Lower Limit */
#define TIVA_AON_BATMON_TEMPUL_OFFSET                   0x0058  /* Temperature Upper Limit */
#define TIVA_AON_BATMON_TEMPLL_OFFSET                   0x005c  /* Temperature Lower Limit */

/* AON BATMON Register Addresses ************************************************************************************/

#define TIVA_AON_BATMON_CTL                             (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_CTL_OFFSET)
#define TIVA_AON_BATMON_MEASCFG                         (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_MEASCFG_OFFSET)
#define TIVA_AON_BATMON_TEMPP0                          (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_TEMPP0_OFFSET)
#define TIVA_AON_BATMON_TEMPP1                          (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_TEMPP1_OFFSET)
#define TIVA_AON_BATMON_TEMPP2                          (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_TEMPP2_OFFSET)
#define TIVA_AON_BATMON_BATMONP0                        (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_BATMONP0_OFFSET)
#define TIVA_AON_BATMON_BATMONP1                        (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_BATMONP1_OFFSET)
#define TIVA_AON_BATMON_IOSTRP0                         (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_IOSTRP0_OFFSET)
#define TIVA_AON_BATMON_FLASHPUMPP0                     (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_FLASHPUMPP0_OFFSET)
#define TIVA_AON_BATMON_BAT                             (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_BAT_OFFSET)
#define TIVA_AON_BATMON_BATUPD                          (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_BATUPD_OFFSET)
#define TIVA_AON_BATMON_TEMP                            (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_TEMP_OFFSET)
#define TIVA_AON_BATMON_TEMPUPD                         (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_TEMPUPD_OFFSET)
#define TIVA_AON_BATMON_EVENTMASK                       (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_EVENTMASK_OFFSET)
#define TIVA_AON_BATMON_EVENT                           (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_EVENT_OFFSET)
#define TIVA_AON_BATMON_BATUL                           (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_BATUL_OFFSETT)
#define TIVA_AON_BATMON_BATLL                           (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_BATLL_OFFSETT)
#define TIVA_AON_BATMON_TEMPUL                          (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_TEMPUL_OFFSET)
#define TIVA_AON_BATMON_TEMPLL                          (TIVA_AON_BATMON_BASE + TIVA_AON_BATMON_TEMPLL_OFFSET)

/* AON BATMON Register Bitfield Definitions *************************************************************************/

/* AON_BATMON_CTL */

#define AON_BATMON_CTL_MEAS_EN                          (1 << 0)  /* Bit 0 */
#define AON_BATMON_CTL_CALC_EN                          (1 << 1)  /* Bit 1 */

/* AON_BATMON_MEASCFG */

#define AON_BATMON_MEASCFG_PER_SHIFT                    (0)       /* Bits 0-1 */
#define AON_BATMON_MEASCFG_PER_MASK                     (3 << AON_BATMON_MEASCFG_PER_SHIFT)
#  define AON_BATMON_MEASCFG_PER(n)                     ((uint32_t)(n) << AON_BATMON_MEASCFG_PER_SHIFT)
#  define AON_BATMON_MEASCFG_PER_CONT                   (0 << AON_BATMON_MEASCFG_PER_SHIFT)
#  define AON_BATMON_MEASCFG_PER_8CYC                   (1 << AON_BATMON_MEASCFG_PER_SHIFT)
#  define AON_BATMON_MEASCFG_PER_16CYC                  (2 << AON_BATMON_MEASCFG_PER_SHIFT)
#  define AON_BATMON_MEASCFG_PER_32CYC                  (3 << AON_BATMON_MEASCFG_PER_SHIFT)

/* AON_BATMON_TEMPP0 */

#define AON_BATMON_TEMPP0_CFG_SHIFT                     (0)       /* Bits 0-7 */
#define AON_BATMON_TEMPP0_CFG_MASK                      (0xff << AON_BATMON_TEMPP0_CFG_SHIFT)
#  define AON_BATMON_TEMPP0_CFG(n)                      ((uint32_t)(n) << AON_BATMON_TEMPP0_CFG_SHIFT)

/* AON_BATMON_TEMPP1 */

#define AON_BATMON_TEMPP1_CFG_SHIFT                     (0)       /* Bits 0-5 */
#define AON_BATMON_TEMPP1_CFG_MASK                      (0x3f << AON_BATMON_TEMPP1_CFG_SHIFT)
#  define AON_BATMON_TEMPP1_CFG(n)                      ((uint32_t)(n) << AON_BATMON_TEMPP1_CFG_SHIFT)

/* AON_BATMON_TEMPP2 */

#define AON_BATMON_TEMPP2_CFG_SHIFT                     (0)       /* Bits 0-4 */
#define AON_BATMON_TEMPP2_CFG_MASK                      (0x2f << AON_BATMON_TEMPP2_CFG_SHIFT)
#  define AON_BATMON_TEMPP2_CFG(n)                      ((uint32_t)(n) << AON_BATMON_TEMPP2_CFG_SHIFT)

/* AON_BATMON_BATMONP0 */

#define AON_BATMON_BATMONP0_CFG_SHIFT                   (0)       /* Bits 0-6 */
#define AON_BATMON_BATMONP0_CFG_MASK                    (0x7f << AON_BATMON_BATMONP0_CFG_SHIFT)
#  define AON_BATMON_BATMONP0_CFG(n)                    ((uint32_t)(n) << AON_BATMON_BATMONP0_CFG_SHIFT)

/* AON_BATMON_BATMONP1 */

#define AON_BATMON_BATMONP1_CFG_SHIFT                   (0)       /* Bits 0-5 */
#define AON_BATMON_BATMONP1_CFG_MASK                    (0x3f << AON_BATMON_BATMONP1_CFG_SHIFT)
#  define AON_BATMON_BATMONP1_CFG(n)                    ((uint32_t)(n) << AON_BATMON_BATMONP1_CFG_SHIFT)

/* AON_BATMON_IOSTRP0 */

#define AON_BATMON_IOSTRP0_CFG1_SHIFT                   (0)       /* Bits 0-3 */
#define AON_BATMON_IOSTRP0_CFG1_MASK                    (15 << AON_BATMON_IOSTRP0_CFG1_SHIFT)
#  define AON_BATMON_IOSTRP0_CFG1(n)                    ((uint32_t)(n) << AON_BATMON_IOSTRP0_CFG1_SHIFT)
#define AON_BATMON_IOSTRP0_CFG2_SHIFT                   (4)       /* Bits 4-6 */
#define AON_BATMON_IOSTRP0_CFG2_MASK                    (3 << AON_BATMON_IOSTRP0_CFG2_SHIFT)
#  define AON_BATMON_IOSTRP0_CFG2(n)                    ((uint32_t)(n) << AON_BATMON_IOSTRP0_CFG2_SHIFT)

/* AON_BATMON_FLASHPUMPP0 */

#define AON_BATMON_FLASHPUMPP0_CFG_SHIFT                (0)       /* Bits 0-3 */
#define AON_BATMON_FLASHPUMPP0_CFG_MASK                 (15 << AON_BATMON_FLASHPUMPP0_CFG_SHIFT)
#  define AON_BATMON_FLASHPUMPP0_CFG(n)                 ((uint32_t)(n) << AON_BATMON_FLASHPUMPP0_CFG_SHIFT)
#define AON_BATMON_FLASHPUMPP0_OVR                      (1 << 4)  /* Bit 4 */
#define AON_BATMON_FLASHPUMPP0_LOWLIM                   (1 << 5)  /* Bit 5 */
#define AON_BATMON_FLASHPUMPP0_HIGHLIM_SHIFT            (6)       /* Bits 6-7 */
#define AON_BATMON_FLASHPUMPP0_HIGHLIM_MASK             (3 << AON_BATMON_FLASHPUMPP0_HIGHLIM_SHIFT)
#  define AON_BATMON_FLASHPUMPP0_HIGHLIM(n)             ((uint32_t)(n) << AON_BATMON_FLASHPUMPP0_HIGHLIM_SHIFT)
#define AON_BATMON_FLASHPUMPP0_FALLB                    (1 << 8)  /* Bit 8 */
#define AON_BATMON_FLASHPUMPP0_DIS_NOISE_FILTER         (1 << 9)  /* Bit 9 */

/* AON_BATMON_BAT */

#define AON_BATMON_BAT_FRAC_SHIFT                       (0)       /* Bits 0-7:  Fractional part */
#define AON_BATMON_BAT_FRAC_MASK                        (0xff << AON_BATMON_BAT_FRAC_SHIFT)
#  define AON_BATMON_BAT_FRAC(n)                        ((uint32_t)(n) << AON_BATMON_BAT_FRAC_SHIFT)
#define AON_BATMON_BAT_INT_SHIFT                        (8)       /* Bits 8-10:  Integer part */
#define AON_BATMON_BAT_INT_MASK                         (7 << AON_BATMON_BAT_INT_SHIFT)
#  define AON_BATMON_BAT_INT(n)                         ((uint32_t)(n) << AON_BATMON_BAT_INT_SHIFT)

/* AON_BATMON_BATUPD */

#define AON_BATMON_BATUPD_STAT                          (1 << 0)  /* Bit 0:  New battery voltage is present */

/* AON_BATMON_TEMP */

#define AON_BATMON_TEMP_INT_SHIFT                       (8)       /* Bits 8-16:  Signed integer part of temperature */
#define AON_BATMON_TEMP_INT_MASK                        (0x1ff << AON_BATMON_TEMP_INT_SHIFT)
#  define AON_BATMON_TEMP_INT(n)                        ((uint32_t)(n) << AON_BATMON_TEMP_INT_SHIFT)
#  define AON_BATMON_TEMP_INT_MIN                       (0x100 << AON_BATMON_TEMP_INT_SHIFT)
#  define AON_BATMON_TEMP_INT_MAX                       (0x0ff << AON_BATMON_TEMP_INT_SHIFT)
#  define AON_BATMON_TEMP_INT_ZERO                      (0x000 << AON_BATMON_TEMP_INT_SHIFT)

/* AON_BATMON_TEMPUPD */

#define AON_BATMON_TEMPUPD_STAT                         (1 << 0)  /* Bit 0:  New temperature is present */

/* AON_BATMON_EVENTMASK */

#define AON_BATMON_EVENTMASK_BATT_OVER_UL               (1 << 0)  /* Bit 0:  EVENT.BATT_OVER_UL does not
                                                                   * contribute to combined event from
                                                                   * BATMON */
#define AON_BATMON_EVENTMASK_BATT_BELOW_LL              (1 << 1)  /* Bit 1:  EVENT.BATT_BELOW_LL does not
                                                                   * contribute to combined event from
                                                                   * BATMON */
#define AON_BATMON_EVENTMASK_TEMP_OVER_UL               (1 << 2)  /* Bit 2:  EVENT.TEMP_OVER_UL does not
                                                                   * contribute to combined event from
                                                                   * BATMON */
#define AON_BATMON_EVENTMASK_TEMP_BELOW_LL              (1 << 3)  /* Bit 3:  EVENT.TEMP_BELOW_LL does not
                                                                   * contribute to combined event from
                                                                   * BATMON */
#define AON_BATMON_EVENTMASK_BATT_UPDATE                (1 << 4)  /* Bit 4:  EVENT.BATT_UPDATE does not
                                                                   * contribute to combined event from
                                                                   * BATMON */
#define AON_BATMON_EVENTMASK_TEMP_UPDATE                (1 << 5)  /* Bit 5: EVENT.TEMP_UPDATE does not
                                                                   * contribute to combined event from
                                                                   * BATMON */

/* AON_BATMON_EVENT */

#define AON_BATMON_EVENT_BATT_OVER_UL                   (1 << 0)  /* Bit 0:  R: Batter level above upper limit
                                                                   *         W: Clears the flag */
#define AON_BATMON_EVENT_BATT_BELOW_LL                  (1 << 1)  /* Bit 1:  Battery level below lower limit
                                                                   *         W: Clears the flag */
#define AON_BATMON_EVENT_TEMP_OVER_UL                   (1 << 2)  /* Bit 2:  R: Temperature level above upper limit
                                                                   *         W: Clears the flag */
#define AON_BATMON_EVENT_TEMP_BELOW_LL                  (1 << 3)  /* Bit 3:  R: Temperature level below lower limit
                                                                   *         W: Clears the flag */
#define AON_BATMON_EVENT_BATT_UPDATE                    (1 << 4)  /* Bit 4:  Alias to BATUPD.STAT */
#define AON_BATMON_EVENT_TEMP_UPDATE                    (1 << 5)  /* Bit 5:  Alias to TEMPUPD.STAT */

/* AON_BATMON_BATTUL */

#define AON_BATMON_BATTUL_FRAC_SHIFT                    (0)       /* Bits 0-7: Fractional part */
#define AON_BATMON_BATTUL_FRAC_MASK                     (0xff << AON_BATMON_BATTUL_FRAC_SHIFT)
#  define AON_BATMON_BATTUL_FRAC(n)                     ((uint32_t)(n) << AON_BATMON_BATTUL_FRAC_SHIFT)
#define AON_BATMON_BATTUL_INT_SHIFT                     (8)       /* Bits: 8-10:  Integer part */
#define AON_BATMON_BATTUL_INT_MASK                      (7 << AON_BATMON_BATTUL_INT_SHIFT)
#  define AON_BATMON_BATTUL_INT(n)                      ((uint32_t)(n) << AON_BATMON_BATTUL_INT_SHIFT)

/* AON_BATMON_BATTLL */

#define AON_BATMON_BATTLL_INT_SHIFT                     (8)       /* Bits: 8-10:  Integer part */
#define AON_BATMON_BATTLL_INT_MASK                      (7 << AON_BATMON_BATTLL_INT_SHIFT)
#  define AON_BATMON_BATTLL_INT(n)                      ((uint32_t)(n) << AON_BATMON_BATTLL_INT_SHIFT)

/* AON_BATMON_TEMPUL */

#define AON_BATMON_TEMPUL_FRAC_SHIFT                    (6)       /* Bits 6-7: Fractional part of
                                                                   * temperature upper limit */
#define AON_BATMON_TEMPUL_FRAC_MASK                     (3 << AON_BATMON_TEMPUL_FRAC_SHIFT)
#  define AON_BATMON_TEMPUL_FRAC(n)                     ((uint32_t)(n) << AON_BATMON_TEMPUL_FRAC_SHIFT)
#define AON_BATMON_TEMPUL_INT_SHIFT                     (8)       /* Bits 8-16:  Signed integer part
                                                                   * of temperature upper limit */
#define AON_BATMON_TEMPUL_INT_MASK                      (0x1ff << AON_BATMON_TEMPUL_INT_SHIFT)
#  define AON_BATMON_TEMPUL_INT(n)                      ((uint32_t)(n) << AON_BATMON_TEMPUL_INT_SHIFT)

/* AON_BATMON_TEMPLL */

#define AON_BATMON_TEMPLL_FRAC_SHIFT                    (6)       /* Bits 6-7: Fractional part of
                                                                   * temperature lower limit */
#define AON_BATMON_TEMPLL_FRAC_MASK                     (3 << AON_BATMON_TEMPLL_FRAC_SHIFT)
#  define AON_BATMON_TEMPLL_FRAC(n)                     ((uint32_t)(n) << AON_BATMON_TEMPLL_FRAC_SHIFT)
#define AON_BATMON_TEMPLL_INT_SHIFT                     (8)       /* Bits 8-16:  Signed integer part
                                                                   * of temperature lower limit */
#define AON_BATMON_TEMPLL_INT_MASK                      (0x1ff << AON_BATMON_TEMPLL_INT_SHIFT)
#  define AON_BATMON_TEMPLL_INT(n)                      ((uint32_t)(n) << AON_BATMON_TEMPLL_INT_SHIFT)

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_BATMON_H */
