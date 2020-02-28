/************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_crs.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRS_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRS_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_CRS_CR_OFFSET        0x0000  /* CRS control register */
#define STM32_CRS_CFGR_OFFSET      0x0004  /* CRS configuration register */
#define STM32_CRS_ISR_OFFSET       0x0008  /* CRS interrupt and status register */
#define STM32_CRS_ICR_OFFSET       0x000c  /* CRS interrupt flag clear register */

/* Register Addresses ***************************************************************/

#define STM32_CRS_CR               (STM32_CRS_BASE + STM32_CRS_CR_OFFSET)
#define STM32_CRS_CFGR             (STM32_CRS_BASE + STM32_CRS_CFGR_OFFSET)
#define STM32_CRS_ISR              (STM32_CRS_BASE + STM32_CRS_ISR_OFFSET)
#define STM32_CRS_ICR              (STM32_CRS_BASE + STM32_CRS_ICR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* CRS control register */

#define CRS_CR_SYNCOKIE            (1 << 0)  /* Bit 0: SYNC event OK interrupt enable */
#define CRS_CR_SYNCWARNIE          (1 << 1)  /* Bit 1: SYNC warning interrupt enable */
#define CRS_CR_ERRIE               (1 << 2)  /* Bit 2: Synchronization or Trimming error interrupt enabled */
#define CRS_CR_ESYNCIE             (1 << 3)  /* Bit 3: Expected SYNC interrupt enable */
#define CRS_CR_CEN                 (1 << 5)  /* Bit 5: Frequency error counter enable */
#define CRS_CR_AUTOTRIMEN          (1 << 6)  /* Bit 6: Automatic trimming enabled */
#define CRS_CR_SWSYNC              (1 << 7)  /* Bit 7: Generate software SYNC event */
#define CRS_CR_TRIM_SHIFT          8         /* Bits 8-13: HSI48 oscillator smooth trimming */
#define CRS_CR_TRIM_MASK           (0x3f << CRS_CR_TRIM_SHIFT)

/* CRS configuration register */

#define CRS_CFGR_RELOAD_SHIFT      0         /* Bits 0-15: Counter reload value */
#define CRS_CFGR_RELOAD_MASK       (0xffff << CRS_CFGR_RELOAD_SHIFT)
#define CRS_CFGR_FELIM_SHIFT       16        /* Bits 16-23: Frequency error limit */
#define CRS_CFGR_FELIM_MASK        (0xff << CRS_CFGR_FELIM_SHIFT)
#define CRS_CFGR_SYNCDIV_SHIFT     24        /* Bits 24-26: SYNC divider */
#define CRS_CFGR_SYNCDIV_MASK      (7 << CRS_CFGR_SYNCDIV_SHIFT)
#  define CRS_CFGR_SYNCDIV_d1      (0 << CRS_CFGR_SYNCDIV_SHIFT) /* Not divided */
#  define CRS_CFGR_SYNCDIV_d2      (1 << CRS_CFGR_SYNCDIV_SHIFT) /* divided by 2 */
#  define CRS_CFGR_SYNCDIV_d4      (2 << CRS_CFGR_SYNCDIV_SHIFT) /* divided by 4 */
#  define CRS_CFGR_SYNCDIV_d8      (3 << CRS_CFGR_SYNCDIV_SHIFT) /* divided by 8 */
#  define CRS_CFGR_SYNCDIV_d16     (4 << CRS_CFGR_SYNCDIV_SHIFT) /* divided by 16 */
#  define CRS_CFGR_SYNCDIV_d32     (5 << CRS_CFGR_SYNCDIV_SHIFT) /* divided by 32 */
#  define CRS_CFGR_SYNCDIV_d64     (6 << CRS_CFGR_SYNCDIV_SHIFT) /* divided by 64 */
#  define CRS_CFGR_SYNCDIV_d128    (7 << CRS_CFGR_SYNCDIV_SHIFT) /* divided by 128 */
#define CRS_CFGR_SYNCSRC_SHIFT     28        /* Bits 28-29: SYNC signal source selection */
#define CRS_CFGR_SYNCSRC_MASK      (3 << CRS_CFGR_SYNCSRC_SHIFT)
#  define CRS_CFGR_SYNCSRC_GPIO    (0 << CRS_CFGR_SYNCSRC_SHIFT) /* GPIO as SYNC signal source */
#  define CRS_CFGR_SYNCSRC_LSE     (1 << CRS_CFGR_SYNCSRC_SHIFT) /* LSE as SYNC signal source */
#  define CRS_CFGR_SYNCSRC_USBSOF  (2 << CRS_CFGR_SYNCSRC_SHIFT) /* USB SOF as SYNC signal source */
#define CRS_CFGR_SYNCPOL           (1 << 31) /* SYNC polarity selection */

/* CRS interrupt and status register */

#define CRS_ISR_SYNCOKF            (1 << 0)  /* Bit 0: SYNC event OK flag */
#define CRS_ISR_SYNCWARNF          (1 << 1)  /* Bit 1: SYNC warning flag */
#define CRS_ISR_ERRF               (1 << 2)  /* Bit 2: Errot flag */
#define CRS_ISR_ESYNCF             (1 << 3)  /* Bit 3: Expected SYNC flag */
#define CRS_ISR_SYNCERR            (1 << 8)  /* Bit 8: SYNC error */
#define CRS_ISR_SYNCMISS           (1 << 9)  /* Bit 9: SYNC missed */
#define CRS_ISR_TRIMOVF            (1 << 10) /* Bit 10: Trimming overflow or underflow */
#define CRS_ISR_FEDIR              (1 << 15) /* Bit 15: Frequency error direction */
#define CRS_ISR_FECAP_SHIFT        16        /* Bits 16-31: Frequency error capture */
#define CRS_ISR_FECAP_MASK         (0xffff << CRS_ISR_FECAP_SHIFT)

/* CRS interrupt flag clear register */

#define CRS_ICR_SYNCOKC            (1 << 0)  /* Bit 0: SYNC event OK clear flag */
#define CRS_ICR_SYNCWARNC          (1 << 1)  /* Bit 1: SYNC warning clear flag */
#define CRS_ICR_ERRC               (1 << 2)  /* Bit 2: Error clear flag */
#define CRS_ICR_ESYNCC             (1 << 3)  /* Bit 3: Expected SYNC clear flag */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRS_H */
