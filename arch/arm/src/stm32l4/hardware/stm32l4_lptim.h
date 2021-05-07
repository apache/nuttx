/****************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4_lptim.h
 *
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2009, 2011-2012, 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_LPTIM_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_LPTIM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Basic Timers - TIM6 and TIM7 */

#define STM32L4_LPTIM_ISR_OFFSET  0x0000  /* Interrupt and Status Register */
#define STM32L4_LPTIM_ICR_OFFSET  0x0004  /* Interrupt Clear Register */
#define STM32L4_LPTIM_IER_OFFSET  0x0008  /* Interrupt Enable Register */
#define STM32L4_LPTIM_CFGR_OFFSET 0x000c  /* Configuration Register */
#define STM32L4_LPTIM_CR_OFFSET   0x0010  /* Control Register */
#define STM32L4_LPTIM_CMP_OFFSET  0x0014  /* Compare Register */
#define STM32L4_LPTIM_ARR_OFFSET  0x0018  /* Autoreload Register */
#define STM32L4_LPTIM_CNT_OFFSET  0x001c  /* Counter Register */

/* Register Addresses *******************************************************/

/* Low-Power Timers - LPTIM1 and LPTIM2 */

#define STM32L4_LPTIM1_ISR        (STM32L4_LPTIM1_BASE+STM32L4_LPTIM_ISR_OFFSET)
#define STM32L4_LPTIM1_ICR        (STM32L4_LPTIM1_BASE+STM32L4_LPTIM_ICR_OFFSET)
#define STM32L4_LPTIM1_IER        (STM32L4_LPTIM1_BASE+STM32L4_LPTIM_IER_OFFSET)
#define STM32L4_LPTIM1_CFGR       (STM32L4_LPTIM1_BASE+STM32L4_LPTIM_CFGR_OFFSET)
#define STM32L4_LPTIM1_CR         (STM32L4_LPTIM1_BASE+STM32L4_LPTIM_CR_OFFSET)
#define STM32L4_LPTIM1_CMP        (STM32L4_LPTIM1_BASE+STM32L4_LPTIM_CMP_OFFSET)
#define STM32L4_LPTIM1_ARR        (STM32L4_LPTIM1_BASE+STM32L4_LPTIM_ARR_OFFSET)
#define STM32L4_LPTIM1_CNT        (STM32L4_LPTIM1_BASE+STM32L4_LPTIM_CNT_OFFSET)

#define STM32L4_LPTIM2_ISR        (STM32L4_LPTIM2_BASE+STM32L4_LPTIM_ISR_OFFSET)
#define STM32L4_LPTIM2_ICR        (STM32L4_LPTIM2_BASE+STM32L4_LPTIM_ICR_OFFSET)
#define STM32L4_LPTIM2_IER        (STM32L4_LPTIM2_BASE+STM32L4_LPTIM_IER_OFFSET)
#define STM32L4_LPTIM2_CFGR       (STM32L4_LPTIM2_BASE+STM32L4_LPTIM_CFGR_OFFSET)
#define STM32L4_LPTIM2_CR         (STM32L4_LPTIM2_BASE+STM32L4_LPTIM_CR_OFFSET)
#define STM32L4_LPTIM2_CMP        (STM32L4_LPTIM2_BASE+STM32L4_LPTIM_CMP_OFFSET)
#define STM32L4_LPTIM2_ARR        (STM32L4_LPTIM2_BASE+STM32L4_LPTIM_ARR_OFFSET)
#define STM32L4_LPTIM2_CNT        (STM32L4_LPTIM2_BASE+STM32L4_LPTIM_CNT_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define LPTIM_CFGR_CKSEL          (1 << 0)   /* Bit   0: Clock selector */
#define LPTIM_CFGR_CKSEL_SHIFT    (0)
#define LPTIM_CFGR_CKSEL_MASK     (1)
#  define LPTIM_CFGR_CKSEL_INTCLK (0)        /* 0: Internal clock */
#  define LPTIM_CFGR_CKSEL_EXTCLK (1)        /* 1: External clock */

#define LPTIM_CFGR_CKPOL_SHIFT     (1)        /* Bits 2-1: Clock Polarity */
#define LPTIM_CFGR_CKPOL_MASK      (3 << LPTIM_CFGR_CKPOL_SHIFT)
#  define LPTIM_CFGR_CKPOL_RISING  (0 << LPTIM_CFGR_CKPOL_SHIFT) /* 00: Rising Edge */
#  define LPTIM_CFGR_CKPOL_FALLING (1 << LPTIM_CFGR_CKPOL_SHIFT) /* 01: Falling Edge */
#  define LPTIM_CFGR_CKPOL_BOTH    (2 << LPTIM_CFGR_CKPOL_SHIFT) /* 00: Both Edges */

#define LPTIM_CFGR_CKFLT_SHIFT    (3)        /* Bits 4-3: Digital filter for external clock */
#define LPTIM_CFGR_CKFLTN_MASK    (3 << LPTIM_CFGR_CKFLT_SHIFT)
                                             /* Bit  5: reserved */
#define LPTIM_CFGR_TRGFLT_SHIFT   (6)        /* Bits 7-6: Digital filter for trigger */
#define LPTIM_CFGR_TRGFLT_MASK    (3 << LPTIM_CFGR_TRGFLT_SHIFT)
                                             /* Bit  8: reserved */
#define LPTIM_CFGR_PRESC_SHIFT    (9)        /* Bits 11-9: clock pre-scaler */
#define LPTIM_CFGR_PRESC_MASK     (7 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESCd1      (0 << LPTIM_CFGR_PRESC_SHIFT) /* 000: divide by 1 */
#  define LPTIM_CFGR_PRESCd2      (1 << LPTIM_CFGR_PRESC_SHIFT) /* 001: divide by 2 */
#  define LPTIM_CFGR_PRESCd4      (2 << LPTIM_CFGR_PRESC_SHIFT) /* 010: divide by 4 */
#  define LPTIM_CFGR_PRESCd8      (3 << LPTIM_CFGR_PRESC_SHIFT) /* 011: divide by 8 */
#  define LPTIM_CFGR_PRESCd16     (4 << LPTIM_CFGR_PRESC_SHIFT) /* 100: divide by 16 */
#  define LPTIM_CFGR_PRESCd32     (5 << LPTIM_CFGR_PRESC_SHIFT) /* 101: divide by 32 */
#  define LPTIM_CFGR_PRESCd64     (6 << LPTIM_CFGR_PRESC_SHIFT) /* 110: divide by 64 */
#  define LPTIM_CFGR_PRESCd128    (7 << LPTIM_CFGR_PRESC_SHIFT) /* 111: divide by 128 */

                                             /* Bit  12: reserved */
#define LPTIM_CFGR_TRIGSEL_SHIFT  (13)       /* Bits 15-13: Trigger selector */
#define LPTIM_CFGR_TRIGSEL_MASK   (7 << LPTIM_CFGR_TRIGSEL_SHIFT)
                                             /* Bit  16: reserved */
#define LPTIM_CFGR_TRIGEN_SHIFT   (17)       /* Bits 18-17: Trigger enable and polarity */
#define LPTIM_CFGR_TRIGEN_MASK    (3 << LPTIM_CFGR_TRIGEN_SHIFT)
#define LPTIM_CFGR_TIMOUT         (1 << 19)  /* Bit  19: Timeout enable */
#define LPTIM_CFGR_WAVE           (1 << 20)  /* Bit  20: Waveform shape */
#define LPTIM_CFGR_WAVPOL         (1 << 21)  /* Bit  21: Waveform polarity */
#define LPTIM_CFGR_PRELOAD        (1 << 22)  /* Bit  22: Update mode enable */
#define LPTIM_CFGR_COUNTMODE      (1 << 23)  /* Bit  23: Count mode enable */
#define LPTIM_CFGR_ENC            (1 << 24)  /* Bit  24: Encoder mode enable (LPTIM1 only) */

#define LPTIM_CR_ENABLE           (1 << 0)   /* Bit 0: Enable */
#define LPTIM_CR_SNGSTRT          (1 << 1)   /* Bit 1: Single Mode */
#define LPTIM_CR_CNTSTRT          (1 << 2)   /* Bit 2: Continuous Mode */

#define LPTIM_ISR_CMPM            (1 << 0)   /* Bit 0: Compare match */
#define LPTIM_ISR_ARRM            (1 << 1)   /* Bit 1: Autoreload match */
#define LPTIM_ISR_EXTTRIG         (1 << 2)   /* Bit 2: External trigger edge event */
#define LPTIM_ISR_CMPOK           (1 << 3)   /* Bit 3: Compare register update OK */
#define LPTIM_ISR_ARROK           (1 << 4)   /* Bit 4: Autoreload register update OK */
#define LPTIM_ISR_UP              (1 << 5)   /* Bit 5: Counter direction change down to up */
#define LPTIM_ISR_DOWN            (1 << 6)   /* Bit 6: Counter direction change up to down */

#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_LPTIM_H */
