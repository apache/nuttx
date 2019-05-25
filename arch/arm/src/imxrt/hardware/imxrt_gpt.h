/************************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_gpt.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_GPT_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_GPT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define IMXRT_GPT_CR_OFFSET                 0x0000  /* GPT Control Register */
#define IMXRT_GPT_PR_OFFSET                 0x0004  /* GPT Prescaler Register */
#define IMXRT_GPT_SR_OFFSET                 0x0008  /* GPT Status Register */
#define IMXRT_GPT_IR_OFFSET                 0x000c  /* GPT Interrupt Register */
#define IMXRT_GPT_OCR1_OFFSET               0x0010  /* GPT Output Compare Register 1 */
#define IMXRT_GPT_OCR2_OFFSET               0x0014  /* GPT Output Compare Register 2 */
#define IMXRT_GPT_OCR3_OFFSET               0x0018  /* GPT Output Compare Register 3 */
#define IMXRT_GPT_ICR1_OFFSET               0x001c  /* GPT Input Capture Register 1 */
#define IMXRT_GPT_ICR2_OFFSET               0x0020  /* GPT Input Capture Register 2 */
#define IMXRT_GPT_CNT_OFFSET                0x0024  /* GPT Counter Register */

/* Register addresses ***********************************************************************/

#define IMXRT_GPT1_CR                       (IMXRT_GPT1_BASE + IMXRT_GPT1_CR_OFFSET)    /* GPT 1 Control Register */
#define IMXRT_GPT1_PR                       (IMXRT_GPT1_BASE + IMXRT_GPT1_PR_OFFSET)    /* GPT 1 Prescaler Register */
#define IMXRT_GPT1_SR                       (IMXRT_GPT1_BASE + IMXRT_GPT1_SR_OFFSET)    /* GPT 1 Status Register */
#define IMXRT_GPT1_IR                       (IMXRT_GPT1_BASE + IMXRT_GPT1_IR_OFFSET)    /* GPT 1 Interrupt Register */
#define IMXRT_GPT1_OCR1                     (IMXRT_GPT1_BASE + IMXRT_GPT1_OCR1_OFFSET)  /* GPT 1 Output Compare Register 1 */
#define IMXRT_GPT1_OCR2                     (IMXRT_GPT1_BASE + IMXRT_GPT1_OCR2_OFFSET)  /* GPT 1 Output Compare Register 2 */
#define IMXRT_GPT1_OCR3                     (IMXRT_GPT1_BASE + IMXRT_GPT1_OCR3_OFFSET)  /* GPT 1 Output Compare Register 3 */
#define IMXRT_GPT1_ICR1                     (IMXRT_GPT1_BASE + IMXRT_GPT1_ICR1_OFFSET)  /* GPT 1 Input Capture Register 1 */
#define IMXRT_GPT1_ICR2                     (IMXRT_GPT1_BASE + IMXRT_GPT1_ICR2_OFFSET)  /* GPT 1 Input Capture Register 2 */
#define IMXRT_GPT1_CNT                      (IMXRT_GPT1_BASE + IMXRT_GPT1_CNT_OFFSET)   /* GPT 1 Counter Register */
#define IMXRT_GPT2_CR                       (IMXRT_GPT2_BASE + IMXRT_GPT2_CR_OFFSET)    /* GPT 2 Control Register */
#define IMXRT_GPT2_PR                       (IMXRT_GPT2_BASE + IMXRT_GPT2_PR_OFFSET)    /* GPT 2 Prescaler Register */
#define IMXRT_GPT2_SR                       (IMXRT_GPT2_BASE + IMXRT_GPT2_SR_OFFSET)    /* GPT 2 Status Register */
#define IMXRT_GPT2_IR                       (IMXRT_GPT2_BASE + IMXRT_GPT2_IR_OFFSET)    /* GPT 2 Interrupt Register */
#define IMXRT_GPT2_OCR1                     (IMXRT_GPT2_BASE + IMXRT_GPT2_OCR1_OFFSET)  /* GPT 2 Output Compare Register 1 */
#define IMXRT_GPT2_OCR2                     (IMXRT_GPT2_BASE + IMXRT_GPT2_OCR2_OFFSET)  /* GPT 2 Output Compare Register 2 */
#define IMXRT_GPT2_OCR3                     (IMXRT_GPT2_BASE + IMXRT_GPT2_OCR3_OFFSET)  /* GPT 2 Output Compare Register 3 */
#define IMXRT_GPT2_ICR1                     (IMXRT_GPT2_BASE + IMXRT_GPT2_ICR1_OFFSET)  /* GPT 2 Input Capture Register 1 */
#define IMXRT_GPT2_ICR2                     (IMXRT_GPT2_BASE + IMXRT_GPT2_ICR2_OFFSET)  /* GPT 2 Input Capture Register 2 */
#define IMXRT_GPT2_CNT                      (IMXRT_GPT2_BASE + IMXRT_GPT2_CNT_OFFSET)   /* GPT 2 Counter Register */

/* GPT Control Register */

/* Register Bit Definitions *********************************************************/

#define GPT_CR_EN                  (1 << 0)   /* Bit: 0  GPT Enable. */
#define GPT_CR_ENMOD               (1 << 1)   /* Bit: 1  GPT Enable mode. */
#define GPT_CR_DBGEN               (1 << 2)   /* Bit: 2  GPT debug mode enable. */
#define GPT_CR_WAITEN              (1 << 3)   /* Bit: 3  GPT Wait Mode enable. */
#define GPT_CR_DOZEEN              (1 << 4)   /* Bit: 4  GPT Doze Mode Enable. */
#define GPT_CR_STOPEN              (1 << 5)   /* Bit: 5  GPT Stop Mode enable. */
#define GPT_CR_CLKSRC_SHIFT        (6)        /* Bits: 6-8  Clock Source select. */
#define GPT_CR_CLKSRC_MASK         (7 << GPT_CR_CLKSRC_SHIFT)
#  define GPT_CR_CLKSRC(n)         ((uint32_t)(n) << GPT_CR_CLKSRC_SHIFT)
#  define GPT_CR_CLKSRC_NONE       (0 << GPT_CR_CLKSRC_SHIFT)  /* No clock */
#  define GPT_CR_CLKSRC_IPG        (1 << GPT_CR_CLKSRC_SHIFT)  /* Peripheral Clock (ipg_clk) */
#  define GPT_CR_CLKSRC_IPG_HFR    (2 << GPT_CR_CLKSRC_SHIFT)  /* High Frequency Reference Clock (ipg_clk_highfreq) */
#  define GPT_CR_CLKSRC_EXT        (3 << GPT_CR_CLKSRC_SHIFT)  /* External Clock */
#  define GPT_CR_CLKSRC_IPG_LFR    (4 << GPT_CR_CLKSRC_SHIFT)  /* Low Frequency Reference Clock (ipg_clk_32k) */
#  define GPT_CR_CLKSRC_IPG_24M    (5 << GPT_CR_CLKSRC_SHIFT)  /* Crystal oscillator as Reference Clock (ipg_clk_24M) */
#define GPT_CR_FRR                 (1 << 9)   /* Bit: 9  Free-Run or Restart mode. */
#define GPT_CR_EN_24M              (1 << 10)  /* Bit: 10 Enable 24 MHz clock input from crystal. */
                                              /* Bits: 11-14  Reserved */
#define GPT_CR_SWR                 (1 << 15)  /* Bit: 15 Software reset. */
#define GPT_CR_IM1_SHIFT           (16)       /* Bits: 16-17  See IM2 */
#define GPT_CR_IM1_MASK            (3 << GPT_CR_IM1_SHIFT)
#  define GPT_CR_IM1(n)            ((uint32_t)(n) << GPT_CR_IM1_SHIFT)
#  define GPT_CR_IM1_DIS           (0 << GPT_CR_IM1_SHIFT)  /* Capture disabled */
#  define GPT_CR_IM1_RISING        (1 << GPT_CR_IM1_SHIFT)  /* Capture on rising edge */
#  define GPT_CR_IM1_FALLING       (2 << GPT_CR_IM1_SHIFT)  /* Capture on falling edge */
#  define GPT_CR_IM1_BOTH          (3 << GPT_CR_IM1_SHIFT)  /* Capture on both edges */
#define GPT_CR_IM2_SHIFT           (18)       /* Bits: 18-19  IM2 (bits 19-18, Input Capture Channel 2 operating mode) */
#define GPT_CR_IM2_MASK            (3 << GPT_CR_IM2_SHIFT)
#  define GPT_CR_IM2(n)            ((uint32_t)(n) << GPT_CR_IM2_SHIFT)
#  define GPT_CR_IM2_DIS           (0 << GPT_CR_IM2_SHIFT)  /* Capture disabled */
#  define GPT_CR_IM2_RISING        (1 << GPT_CR_IM2_SHIFT)  /* Capture on rising edge */
#  define GPT_CR_IM2_FALLING       (2 << GPT_CR_IM2_SHIFT)  /* Capture on falling edge */
#  define GPT_CR_IM2_BOTH          (3 << GPT_CR_IM2_SHIFT)  /* Capture on both edges */
#define GPT_CR_OM1_SHIFT           (20)       /* Bits: 20-22  See OM3 */
#define GPT_CR_OM1_MASK            (7 << GPT_CR_OM1_SHIFT)
#  define GPT_CR_OM1(n)            ((uint32_t)(n) << GPT_CR_OM1_SHIFT)
#  define GPT_CR_OM1_DIS           (0 << GPT_CR_OM1_SHIFT)  /* Output disconnected. No response on pin. */
#  define GPT_CR_OM1_TOGGLE        (1 << GPT_CR_OM1_SHIFT)  /* Toggle output pin */
#  define GPT_CR_OM1_CLEAR         (2 << GPT_CR_OM1_SHIFT)  /* Clear output pin */
#  define GPT_CR_OM1_SET           (3 << GPT_CR_OM1_SHIFT)  /* Set output pin */
#  define GPT_CR_OM1_PULSE         (4 << GPT_CR_OM1_SHIFT)  /* Generate an active low pulse */
#define GPT_CR_OM2_SHIFT           (23)       /* Bits: 23-25  See OM3 */
#define GPT_CR_OM2_MASK            (7 << GPT_CR_OM2_SHIFT)
#  define GPT_CR_OM2(n)            ((uint32_t)(n) << GPT_CR_OM2_SHIFT)
#  define GPT_CR_OM2_DIS           (0 << GPT_CR_OM2_SHIFT)  /* Output disconnected. No response on pin. */
#  define GPT_CR_OM2_TOGGLE        (1 << GPT_CR_OM2_SHIFT)  /* Toggle output pin */
#  define GPT_CR_OM2_CLEAR         (2 << GPT_CR_OM2_SHIFT)  /* Clear output pin */
#  define GPT_CR_OM2_SET           (3 << GPT_CR_OM2_SHIFT)  /* Set output pin */
#  define GPT_CR_OM2_PULSE         (4 << GPT_CR_OM2_SHIFT)  /* Generate an active low pulse */
#define GPT_CR_OM3_SHIFT           (26)       /* Bits: 26-28  OM3 (bits 28-26) controls the Output Compare Channel 3 operating mode. */
#define GPT_CR_OM3_MASK            (7 << GPT_CR_OM3_SHIFT)
#  define GPT_CR_OM3(n)            ((uint32_t)(n) << GPT_CR_OM3_SHIFT)
#  define GPT_CR_OM3_DIS           (0 << GPT_CR_OM3_SHIFT)  /* Output disconnected. No response on pin. */
#  define GPT_CR_OM3_TOGGLE        (1 << GPT_CR_OM3_SHIFT)  /* Toggle output pin */
#  define GPT_CR_OM3_CLEAR         (2 << GPT_CR_OM3_SHIFT)  /* Clear output pin */
#  define GPT_CR_OM3_SET           (3 << GPT_CR_OM3_SHIFT)  /* Set output pin */
#  define GPT_CR_OM3_PULSE         (4 << GPT_CR_OM3_SHIFT)  /* Generate an active low pulse */
#define GPT_CR_FO1                 (1 << 29)  /* Bit: 29 See F03 */
#define GPT_CR_FO2                 (1 << 30)  /* Bit: 30 See F03 */
#define GPT_CR_FO3                 (1 << 31)  /* Bit: 31 FO3 Force Output Compare Channel 3 */

/* GPT Prescaler Register */

#define GPT_PR_PRESCALER_SHIFT     (0)        /* Bits: 0-11  Prescaler bits. */
#define GPT_PR_PRESCALER_MASK      (0xfff << GPT_PR_PRESCALER_SHIFT)
#  define GPT_PR_PRESCALER(n)      ((uint32_t)(n) << GPT_PR_PRESCALER_SHIFT)
#define GPT_PR_PRESCALER24M_SHIFT  (12)       /* Bits: 12-15  Prescaler bits. */
#define GPT_PR_PRESCALER24M_MASK   (0xf << GPT_PR_PRESCALER24M_SHIFT)
#  define GPT_PR_PRESCALER24M(n)   ((uint32_t)(n) << GPT_PR_PRESCALER24M_SHIFT)
                                              /* Bits: 16-31  Reserved */

/* GPT Status Register */

#define GPT_SR_OF1                 (1 << 0)   /* Bit: 0  Output Compare 1 Flag*/
#define GPT_SR_OF2                 (1 << 1)   /* Bit: 1  Output Compare 2 Flag*/
#define GPT_SR_OF3                 (1 << 2)   /* Bit: 2  Output Compare 3 Flag */
#define GPT_SR_IF1                 (1 << 3)   /* Bit: 3  Input capture 1 Flag */
#define GPT_SR_IF2                 (1 << 4)   /* Bit: 4  Input capture 2 Flag */
#define GPT_SR_ROV                 (1 << 5)   /* Bit: 5  Rollover Flag. */
                                              /* Bits: 6-31  Reserved */

/* GPT Interrupt Register */

#define GPT_IR_OF1IE               (1 << 0)   /* Bit: 0  Output Compare 1 Interrupt Enable  */
#define GPT_IR_OF2IE               (1 << 1)   /* Bit: 1  Output Compare 2 Interrupt Enable  */
#define GPT_IR_OF3IE               (1 << 2)   /* Bit: 2  Output Compare 3 Interrupt Enable */
#define GPT_IR_IF1IE               (1 << 3)   /* Bit: 3  Input capture 1 Interrupt Enable */
#define GPT_IR_IF2IE               (1 << 4)   /* Bit: 4  Input capture 2 Interrupt Enable */
#define GPT_IR_ROVIE               (1 << 5)   /* Bit: 5  Rollover Interrupt Enable. */
                                              /* Bits: 6-31  Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_GPT_H */
