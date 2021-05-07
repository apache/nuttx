/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_aon_batmon.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_BATMON_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_BATMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AON BATMON Register Offsets **********************************************/

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

/* AON BATMON Register Addresses ********************************************/

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

/* AON BATMON Register Bitfield Definitions *********************************/

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

#define AON_BATMON_BATMONP0_CFG_SHIFT                   (0)       /* Bits 0-5 */
#define AON_BATMON_BATMONP0_CFG_MASK                    (0x2f << AON_BATMON_BATMONP0_CFG_SHIFT)
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

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_BATMON_H */
