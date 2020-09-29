/********************************************************************************************
 * arch/arm/src/tiva/hardware/tiva_wdt.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_WDT_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_SSI_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <hardware/tiva_memorymap.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* WDT register offsets *********************************************************************/

#define TIVA_WDT_LOAD_OFFSET                    0x0000  /* Configuration */
#define TIVA_WDT_VALUE_OFFSET                   0x0004  /* Current Count Value */
#define TIVA_WDT_CTL_OFFSET                     0x0008  /* Control */
#define TIVA_WDT_ICR_OFFSET                     0x000c  /* Interrupt Clear */
#define TIVA_WDT_RIS_OFFSET                     0x0010  /* Raw Interrupt Status */
#define TIVA_WDT_MIS_OFFSET                     0x0014  /* Masked Interrupt Status */
#define TIVA_WDT_TEST_OFFSET                    0x0418  /* Test Mode */
#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define TIVA_WDT_INT_CAUSE_OFFSET             0x041c  /* Interrupt Cause Test Mode */
#endif
#define TIVA_WDT_LOCK_OFFSET                    0x0c00  /* Lock */

#if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_WDT_PERIPHID4_OFFSET             0x0fd0  /* Watchdog Peripheral Identification 4 */
#  define TIVA_WDT_PERIPHID5_OFFSET             0x0fd4  /* Watchdog Peripheral Identification 5 */
#  define TIVA_WDT_PERIPHID6_OFFSET             0x0fd5  /* Watchdog Peripheral Identification 6 */
#  define TIVA_WDT_PERIPHID7_OFFSET             0x0fdc  /* Watchdog Peripheral Identification 7 */
#  define TIVA_WDT_PERIPHID0_OFFSET             0x0fe0  /* Watchdog Peripheral Identification 0 */
#  define TIVA_WDT_PERIPHID1_OFFSET             0x0fe4  /* Watchdog Peripheral Identification 1 */
#  define TIVA_WDT_PERIPHID2_OFFSET             0x0fe8  /* Watchdog Peripheral Identification 2 */
#  define TIVA_WDT_PERIPHID3_OFFSET             0x0fec  /* Watchdog Peripheral Identification 3 */
#  define TIVA_WDT_CELLID0_OFFSET               0x0ff0  /* Watchdog PrimeCell Identification 0 */
#  define TIVA_WDT_CELLID1_OFFSET               0x0ff4  /* Watchdog PrimeCell Identification 1 */
#  define TIVA_WDT_CELLID2_OFFSET               0x0ff8  /* Watchdog PrimeCell Identification 2 */
#  define TIVA_WDT_CELLID3_OFFSET               0x0ffc  /* Watchdog PrimeCell Identification 3 */
#endif

/* WDT register addresses *******************************************************************/

#define TIVA_WDT_LOAD                           (TIVA_WDT_BASE + TIVA_WDT_LOAD_OFFSET)
#define TIVA_WDT_VALUE                          (TIVA_WDT_BASE + TIVA_WDT_VALUE_OFFSET)
#define TIVA_WDT_CTL                            (TIVA_WDT_BASE + TIVA_WDT_CTL_OFFSET)
#define TIVA_WDT_ICR                            (TIVA_WDT_BASE + TIVA_WDT_ICR_OFFSET)
#define TIVA_WDT_RIS                            (TIVA_WDT_BASE + TIVA_WDT_RIS_OFFSET)
#define TIVA_WDT_MIS                            (TIVA_WDT_BASE + TIVA_WDT_MIS_OFFSET)
#define TIVA_WDT_TEST                           (TIVA_WDT_BASE + TIVA_WDT_TEST_OFFSET)
#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define TIVA_WDT_INT_CAUSE                    (TIVA_WDT_BASE + TIVA_WDT_INT_CAUSE_OFFSET)
#endif
#define TIVA_WDT_LOCK                           (TIVA_WDT_BASE + TIVA_WDT_LOCK_OFFSET)
#if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_WDT_PERIPHID4                    (TIVA_WDT_BASE + TIVA_WDT_PERIPHID4_OFFSET)
#  define TIVA_WDT_PERIPHID5                    (TIVA_WDT_BASE + TIVA_WDT_PERIPHID5_OFFSET)
#  define TIVA_WDT_PERIPHID6                    (TIVA_WDT_BASE + TIVA_WDT_PERIPHID6_OFFSET)
#  define TIVA_WDT_PERIPHID7                    (TIVA_WDT_BASE + TIVA_WDT_PERIPHID7_OFFSET)
#  define TIVA_WDT_PERIPHID0                    (TIVA_WDT_BASE + TIVA_WDT_PERIPHID0_OFFSET)
#  define TIVA_WDT_PERIPHID1                    (TIVA_WDT_BASE + TIVA_WDT_PERIPHID1_OFFSET)
#  define TIVA_WDT_PERIPHID2                    (TIVA_WDT_BASE + TIVA_WDT_PERIPHID2_OFFSET)
#  define TIVA_WDT_PERIPHID3                    (TIVA_WDT_BASE + TIVA_WDT_PERIPHID3_OFFSET)
#  define TIVA_WDT_CELLID0                      (TIVA_WDT_BASE + TIVA_WDT_CELLID0_OFFSET)
#  define TIVA_WDT_CELLID1                      (TIVA_WDT_BASE + TIVA_WDT_CELLID1_OFFSET)
#  define TIVA_WDT_CELLID2                      (TIVA_WDT_BASE + TIVA_WDT_CELLID2_OFFSET)
#  define TIVA_WDT_CELLID3                      (TIVA_WDT_BASE + TIVA_WDT_CELLID3_OFFSET)
#endif

/* WDT register bitfield definitions ********************************************************/

/* Configuration (32-bit interval value) */

/* Current Count Value (32-bit timer counter value) */

/* Control */

#define WDT_CTL_INTEN                           (1 << 0)  /* Bit 0:  WDT Interrupt Enable */
#define WDT_CTL_RESEN                           (1 << 1)  /* Bit 1:  WDT Reset Enable */

#if defined(CONFIG_ARCH_CHIP_LM4F) ||  defined(CONFIG_ARCH_CHIP_TM4C) || \
    defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define WDT_CTL_INTTYPE                       (1 << 2)  /* Bit 2:  WDT Interrupt Type */
#    define WDT_CTL_INTTYPE_MASKABLE            0
#    define WDT_CTL_INTTYPE_NONMASKABLE         WDT_CTL_INTTYPE
#endif

#if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define WDT_CTL_WRC                           (1 << 31) /* Bit 31: Write Complete */
#endif

/* Interrupt Clear.
 *
 * A write of any value to this register clears the WDT interrupt and reloads the 32-bit
 * counter from the LOAD register.
 */

/* Raw Interrupt Status */

#define WDT_RIS_WDTRIS                          (1 << 0)  /* Bit 0:  Raw interrupt status */

/* Masked Interrupt Status */

#define WDT_MIS_WDTMIS                          (1 << 0)  /* Bit 0:  Masked interrupt status */

/* Test Mode */

#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define WDT_TEST_TEST_EN                      (1 << 0)  /* Bit 0:  Test enable */
#endif
#define WDT_TEST_STALL                          (1 << 8)  /* Bit 8:  WDT Stall Enable */

/* Interrupt Cause Test Mode */

#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define WDT_INT_CAUSE_CAUSE_INTR              (1 << 0)  /* Bit 0:  Replica of RIS.WDTRIS */
#  define WDT_INT_CAUSE_CAUSE_RESET             (1 << 1)  /* Bit 1:  Csuse of interrupt was a reset */
#endif

/* Lock (32-bit register unlock value) */

#define WDT_LOCK_WDTLOCK_UNLOCK                 0x1acce551

#if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TM4C)
/* Watchdog Peripheral Identification 0-7 */

#  define WDT_PERIPHID_PID_MASK                 (0xff)    /* Bits 0-7:  Watchdog peripheral ID n */

/* Watchdog PrimeCell Identification 0-3 */

#  define WDT_CELLID0_CID_MASK                  (0xff)    /* Bits 0-7:  Watchdog PrimeCell ID n */
#endif

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_WDT_H */
