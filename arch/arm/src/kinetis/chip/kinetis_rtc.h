/************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_rtc.h
 *
 *   Copyright (C) 2011, 2016-2017 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_RTC_H
#define __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(KINETIS_NRTC) && KINETIS_NRTC > 0

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

/* NXP/Freescale has familes and technology generations (sometimes seen as processor
 * speed).  These are organized into feature families, and faster speeds sometimes
 * have extended features.  Families are K02 K10 K20 K22  K24 K30 K40 K50 K60 K64 K65
 * K66 K70 K80
 *
 * So far only two variations/generations on the RTC have been discovered.
 * GEN1    RTC_TSR TPR TAR TCR CR SR LR IER                      WAR RAR
 * GEN2    RTC_TSR TPR TAR TCR CR SR LR IER  TTSR MER MCLR MCHR  WAR RAR
 *
 * KINETIS RTC_GEN1  K20P32-->K20P81M K22 K40 K50 K60@100Mhz K64 120MHz
 * Assumed K10 K11
 *
 * KINETIS_RTC_GEN2 K02 K20P144M K26P169 K60@120Mhz  K65x K66x
 *
 * Note current naming doesn't allow GEN1:MK60FN...Q10  & GEN2:MK60FN...Q12
 */

#if defined(KINETIS_K26) || defined(KINETIS_K65) || defined(KINETIS_K66)
#  define KINETIS_RTC_GEN2
#endif

#define KINETIS_RTC_TSR_OFFSET    0x0000 /* RTC Time Seconds Register */
#define KINETIS_RTC_TPR_OFFSET    0x0004 /* RTC Time Prescaler Register */
#define KINETIS_RTC_TAR_OFFSET    0x0008 /* RTC Time Alarm Register */
#define KINETIS_RTC_TCR_OFFSET    0x000c /* RTC Time Compensation Register */
#define KINETIS_RTC_CR_OFFSET     0x0010 /* RTC Control Register */
#define KINETIS_RTC_SR_OFFSET     0x0014 /* RTC Status Register */
#define KINETIS_RTC_LR_OFFSET     0x0018 /* RTC Lock Register */
#define KINETIS_RTC_IER_OFFSET    0x001c /* RTC Interrupt Enable Register (K40) */

#ifdef KINETIS_K60
 /* Haven't found a processor or nuttx file where KINETIS_RTC_CCR is in it
  * from K60P100M100SF2V2RM this would be called  KINETIS_RTC_IER_OFFSET.
  */

#  define KINETIS_RTC_CCR_OFFSET  0x001c /* RTC Chip Configuration Register (K60) */
#endif

#ifdef KINETIS_RTC_GEN2
#  define KINETIS_RTC_TTSR_OFFSET 0x0020 /* RTC Tamper Times Seconds Register */
#  define KINETIS_RTC_MR_OFFSET   0x0024 /* RTC Monotonic Enable Register */
#  define KINETIS_RTC_MCLR_OFFSET 0x0028 /* RTC Monotonic Counter Low  Register */
#  define KINETIS_RTC_MCHR_OFFSET 0x002c /* RTC Monotonic Counter High Register */
#endif

#define KINETIS_RTC_WAR_OFFSET    0x0800 /* RTC Write Access Register */
#define KINETIS_RTC_RAR_OFFSET    0x0804 /* RTC Read Access Register */

/* Register Addresses ***************************************************************/

#define KINETIS_RTC_TSR           (KINETIS_RTC_BASE+KINETIS_RTC_TSR_OFFSET)
#define KINETIS_RTC_TPR           (KINETIS_RTC_BASE+KINETIS_RTC_TPR_OFFSET)
#define KINETIS_RTC_TAR           (KINETIS_RTC_BASE+KINETIS_RTC_TAR_OFFSET)
#define KINETIS_RTC_TCR           (KINETIS_RTC_BASE+KINETIS_RTC_TCR_OFFSET)
#define KINETIS_RTC_CR            (KINETIS_RTC_BASE+KINETIS_RTC_CR_OFFSET)
#define KINETIS_RTC_SR            (KINETIS_RTC_BASE+KINETIS_RTC_SR_OFFSET)
#define KINETIS_RTC_LR            (KINETIS_RTC_BASE+KINETIS_RTC_LR_OFFSET)
#define KINETIS_RTC_IER           (KINETIS_RTC_BASE+KINETIS_RTC_IER_OFFSET)

#ifdef KINETIS_K60
/* From K60P100M100SF2V2RM this would be called  KINETIS_RTC_IER */

#  define KINETIS_CCR_IER         (KINETIS_RTC_BASE+KINETIS_RTC_CCR_OFFSET)
#endif

#ifdef KINETIS_RTC_GEN2
#  define KINETIS_RTC_TTSR        (KINETIS_RTC_BASE+KINETIS_RTC_TTSR_OFFSET)
#  define KINETIS_RTC_MER         (KINETIS_RTC_BASE+KINETIS_RTC_MER_OFFSET)
#  define KINETIS_RTC_MCLR        (KINETIS_RTC_BASE+KINETIS_RTC_MCLR_OFFSET)
#  define KINETIS_RTC_MCHR        (KINETIS_RTC_BASE+KINETIS_RTC_MCHR_OFFSET)
#endif

#define KINETIS_RTC_WAR           (KINETIS_RTC_BASE+KINETIS_RTC_WAR_OFFSET)
#define KINETIS_RTC_RAR           (KINETIS_RTC_BASE+KINETIS_RTC_RAR_OFFSET)

/* Register Bit Definitions *********************************************************/

/* RTC Time Seconds Register (32-bits of time in seconds) */

/* RTC Time Prescaler Register */

#define RTC_TPR_SHIFT             (0)       /* Bits 0-15: Time Prescaler Register */
#define RTC_TPR_MASK              (0xffff << RTC_TPR_SHIFT)
                                           /* Bits 16-31: Reserved */
/* RTC Time Alarm Register (32-bits of time alarm) */

/* RTC Time Compensation Register (32-bits) */

#define RTC_TCR_TCR_SHIFT         (0)       /* Bits 0-7: Time Compensation Register */
#define RTC_TCR_TCR_MASK          (0xff << RTC_TCR_CIR_MASK)
#define RTC_TCR_CIR_SHIFT         (8)       /* Bits 8-15: Compensation Interval Register */
#define RTC_TCR_CIR_MASK          (0xff << RTC_TCR_CIR_SHIFT)
#define RTC_TCR_TCV_SHIFT         (16)      /* Bits 16-23: Time Compensation Value */
#define RTC_TCR_TCV_MASK          (0xff << RTC_TCR_TCV_SHIFT)
#define RTC_TCR_CIC_SHIFT         (24)      /* Bits 24-31: Compensation Interval Counter */
#define RTC_TCR_CIC_MASK          (0xff << RTC_TCR_CIC_SHIFT)

/* RTC Control Register (32-bits) */

#define RTC_CR_SWR                (1 << 0)  /* Bit 0:  Software Reset */
#define RTC_CR_WPE                (1 << 1)  /* Bit 1:  Wakeup Pin Enable */
#define RTC_CR_SUP                (1 << 2)  /* Bit 2:  Supervisor Access */
#define RTC_CR_UM                 (1 << 3)  /* Bit 3:  Update Mode */
                                            /* Bits 4-7: Reserved */
#define RTC_CR_OSCE               (1 << 8)  /* Bit 8:  Oscillator Enable */
#define RTC_CR_CLKO               (1 << 9)  /* Bit 9:  Clock Output */
#define RTC_CR_SC16P              (1 << 10) /* Bit 10: Oscillator 16pF load configure */
#define RTC_CR_SC8P               (1 << 11) /* Bit 11: Oscillator 8pF load configure */
#define RTC_CR_SC4P               (1 << 12) /* Bit 12: Oscillator 4pF load configure */
#define RTC_CR_SC2P               (1 << 13) /* Bit 13: Oscillator 2pF load configure */
                                            /* Bits 14-31: Reserved */
/* RTC Status Register (32-bits) */

#define RTC_SR_TIF                (1 << 0)  /* Bit 0:  Time Invalid Flag */
#define RTC_SR_TOF                (1 << 1)  /* Bit 1:  Time Overflow Flag */
#define RTC_SR_TAF                (1 << 2)  /* Bit 2:  Time Alarm Flag */

#ifdef KINETIS_RTC_GEN2
#  define RTC_SR_MOF              (1 << 3)  /* Bit 3:  Time Monotonic overflow Flag */
#endif
                                            /* Bit 3: Reserved RTC_GEN1 */
#define RTC_SR_TCE                (1 << 4)  /* Bit 4:  Time Counter Enable */
                                            /* Bits 5-31: Reserved */
/* RTC Lock Register (32-bits) */
                                            /* Bits 0-2: Reserved */
#define RTC_LR_TCL                (1 << 3)  /* Bit 3:  Time Compensation Lock */
#define RTC_LR_CRL                (1 << 4)  /* Bit 4:  Control Register Lock */
#define RTC_LR_SRL                (1 << 5)  /* Bit 5:  Status Register Lock */
#define RTC_LR_LRL                (1 << 6)  /* Bit 6:  Lock Register Lock  */
                                            /* Bit 7:  Reserved */
#ifdef KINETIS_RTC_GEN2
#  define RTC_LR_TTSL             (1 << 8)  /* Bit 8:  Tamper Time Seconds Lock */
#  define RTC_LR_MEL              (1 << 9)  /* Bit 9:  Monotonic Enable lock */
#  define RTC_LR_MCLL             (1 << 10) /* Bit 10: Monotoic Counter Low Lock */
#  define RTC_LR_MCHL             (1 << 11) /* Bit 10: Monotoic Counter High Lock */
#endif
                                            /* Bits 12-31: Reserved */
/* RTC Interrupt Enable Register (32-bits, K40) */

#  define RTC_IER_TIIE            (1 << 0)  /* Bit 0:  Time Invalid Interrupt Enable */
#  define RTC_IER_TOIE            (1 << 1)  /* Bit 1:  Time Overflow Interrupt Enable */
#  define RTC_IER_TAIE            (1 << 2)  /* Bit 2:  Time Alarm Interrupt Enable */

#ifdef KINETIS_RTC_GEN2
#  define RTC_IER_MOIE            (1 << 3)  /* Bit 3:  Monotonic Overflow Interrupt Enable */
#endif

#  define RTC_IER_TSIE            (1 << 4)  /* Bit 4:  Time Seconds Interrupt Enable */
                                            /* Bits 5-6: Reserved */
#  define RTC_IER_WPON            (1 << 7)  /* Bit 7:  Wakeup Pin On */

#ifdef KINETIS_K60
/* RTC Chip Configuration Register (32-bits,K60) */
/* Haven't found this in K60P100M100SF2V2RM */

#  define RTC_CCR_CONFIG_SHIFT    (0)       /* Bits 0-7: Chip Configuration */
#  define RTC_CCR_CONFIG_MASK     (0xff << RTC_CCR_CONFIG_SHIFT)
                                            /* Bits 8-31: Reserved */
#endif

/* RTC Write Access Register (32-bits) */

#define RTC_WAR_TSRW              (1 << 0)  /* Bit 0:  Time Seconds Register Write */
#define RTC_WAR_TPRW              (1 << 1)  /* Bit 1:  Time Prescaler Register Write */
#define RTC_WAR_TARW              (1 << 2)  /* Bit 2:  Time Alarm Register Write */
#define RTC_WAR_TCRW              (1 << 3)  /* Bit 3:  Time Compensation Register Write */
#define RTC_WAR_CRW               (1 << 4)  /* Bit 4:  Control Register Write */
#define RTC_WAR_SRW               (1 << 5)  /* Bit 5:  Status Register Write */
#define RTC_WAR_LRW               (1 << 6)  /* Bit 6:  Lock Register Write */
#define RTC_WAR_IERW              (1 << 7)  /* Bit 7:  Interrupt Enable Register Write */

#ifdef KINETIS_K60
/* This looks like old name, from K60P100M100SF2V2RM bit 7 would be called RTC_RAR_IERW */

#  define RTC_WAR_CCRW            (1 << 7)  /* Bit 7:  Chip Config Register Write */
#endif
                                            /* Bits 8-31: Reserved */
/* RTC Read Access Register */

#define RTC_RAR_TSRR              (1 << 0)  /* Bit 0:  Time Seconds Register Read */
#define RTC_RAR_TPRR              (1 << 1)  /* Bit 1:  Time Prescaler Register Read */
#define RTC_RAR_TARR              (1 << 2)  /* Bit 2:  Time Alarm Register Read */
#define RTC_RAR_TCRR              (1 << 3)  /* Bit 3:  Time Compensation Register Read */
#define RTC_RAR_CRR               (1 << 4)  /* Bit 4:  Control Register Read */
#define RTC_RAR_SRR               (1 << 5)  /* Bit 5:  Status Register Read */
#define RTC_RAR_LRR               (1 << 6)  /* Bit 6:  Lock Register Read */
#define RTC_RAR_IERR              (1 << 7)  /* Bit 7:  Interrupt Enable Register Read */

#ifdef KINETIS_K60
/* This is possibly an old name, from K60P100M100SF2V2RM bit 7 would be called
 * RTC_RAR_IERR.
 */

#  define RTC_RAR_CCRR            (1 << 7)  /* Bit 7:  Chip Config Register Read */
#endif

#ifdef KINETIS_RTC_GEN2
#  define RTC_RAR_TTSR            (1 << 8)  /* Bit 8:  Tamper Time Seconds Read */
#  define RTC_RAR_MERR            (1 << 9)  /* Bit 9:  Monotonic Enable Read */
#  define RTC_RAR_MCLR            (1 << 10) /* Bit 10: Monotoic Counter Low Register Read */
#  define RTC_RAR_MCHR            (1 << 11) /* Bit 10: Monotoic Counter High Register Read */
#endif
                                            /* Bits 11-31: Reserved */

#if defined(KINETIS_RTC_GEN2)/* && defined(CONFIG_RTC_MAGIC) */
#  define CONFIG_RTC_MAGICL   0xfacefee0
#  define CONFIG_RTC_MAGICH   0xef32a141
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* KINETIS_NRTC && KINETIS_NRTC > 0 */
#endif /* __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_RTC_H */
