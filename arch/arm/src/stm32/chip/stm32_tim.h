/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32_tim.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_TIM_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_TIM_H

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

/* Basic Timers - TIM6 and TIM7 */

#define STM32_BTIM_CR1_OFFSET     0x0000  /* Control register 1 (16-bit) */
#define STM32_BTIM_CR2_OFFSET     0x0004  /* Control register 2 (16-bit) */
#define STM32_BTIM_DIER_OFFSET    0x000c  /* DMA/Interrupt enable register (16-bit) */
#define STM32_BTIM_SR_OFFSET      0x0010  /* Status register (16-bit) */
#define STM32_BTIM_EGR_OFFSET     0x0014  /* Event generation register (16-bit) */
#define STM32_BTIM_CNT_OFFSET     0x0024  /* Counter (16-bit) */
#define STM32_BTIM_PSC_OFFSET     0x0028  /* Prescaler (16-bit) */
#define STM32_BTIM_ARR_OFFSET     0x002c  /* Auto-reload register (16-bit) */

/* 16-/32-bit General Timers with DMA: TIM2, TM3, TIM4, and TIM5
 * 16-bit General Timers without DMA: TIM9, TIM10, TIM11, TIM12, TIM13, and TIM14
 * For the STM32F10xx all timers are 16-bit.
 * For the STM32F20xx and STM32F40xx, TIM2 and 5 are 32-bit
 * The STM32 F1 Value Line and the STM32 F3 have variant general purpose registers
 *   that are not yet fully covered in this header file.
 */

#define STM32_GTIM_CR1_OFFSET     0x0000  /* Control register 1 (16-bit) */
#define STM32_GTIM_CR2_OFFSET     0x0004  /* Control register 2 (16-bit, TIM2-5 only) */
#define STM32_GTIM_SMCR_OFFSET    0x0008  /* Slave mode control register (16-bit, TIM2-5 only) */
#define STM32_GTIM_DIER_OFFSET    0x000c  /* DMA/Interrupt enable register (16-bit) */
#define STM32_GTIM_SR_OFFSET      0x0010  /* Status register (16-bit) */
#define STM32_GTIM_EGR_OFFSET     0x0014  /* Event generation register (16-bit) */
#define STM32_GTIM_CCMR1_OFFSET   0x0018  /* Capture/compare mode register 1 (16-bit) */
#define STM32_GTIM_CCMR2_OFFSET   0x001c  /* Capture/compare mode register 2 (16-bit, TIM2-5 only) */
#define STM32_GTIM_CCER_OFFSET    0x0020  /* Capture/compare enable register (16-bit) */
#define STM32_GTIM_CNT_OFFSET     0x0024  /* Counter (16-bit* or 32-bit STM3240 TIM2 and 5 only) */
#define STM32_GTIM_PSC_OFFSET     0x0028  /* Prescaler (16-bit) */
#define STM32_GTIM_ARR_OFFSET     0x002c  /* Auto-reload register (16-bit) */
#define STM32_GTIM_CCR1_OFFSET    0x0034  /* Capture/compare register 1 (16-bit or 32-bit STM3240 TIM2/5 only) */
#define STM32_GTIM_CCR2_OFFSET    0x0038  /* Capture/compare register 2 (16-bit TIM2-5 only or 32-bit STM32 F4 TIM2/5 or STM2 F3 TIM15 only) */
#define STM32_GTIM_CCR3_OFFSET    0x003c  /* Capture/compare register 3 (16-bit TIM2-5 only or 32-bit STM32 F4 TIM2/5 only) */
#define STM32_GTIM_CCR4_OFFSET    0x0040  /* Capture/compare register 4 (16-bit TIM2-5 only or 32-bit STM32 F4 TIM2/5 only) */
#define STM32_GTIM_DCR_OFFSET     0x0048  /* DMA control register (16-bit, TIM2-5 only) */
#define STM32_GTIM_DMAR_OFFSET    0x004c  /* DMA address for burst mode (16-bit, TIM2-5 only) */

/* The Option register is available on in the
 *
 *   STM32 F1 value line, F2 and F4:  TIM2, TIM5, and TIM11
 *   STM32 F3 (and possibly the F1 value line): TIM16
 */

#define STM32_GTIM_OR_OFFSET      0x0050  /* Timer 2/5/11/16 option register */

/* TIM16, and 17 only.
 * Only available in the STM32 F1 Value Line and the STM32 F3 family.
 */

#define STM32_GTIM_RCR_OFFSET     0x002c  /* Repetition counter register (TIM16/TIM17) */
#define STM32_GTIM_BDTR_OFFSET    0x0044  /* Break and dead-time register (TIM16/TIM17) */

/* Advanced Timers - TIM1 and TIM8 */

#define STM32_ATIM_CR1_OFFSET     0x0000  /* Control register 1 (16-bit) */
#define STM32_ATIM_CR2_OFFSET     0x0004  /* Control register 2 (16-bit*) */
#define STM32_ATIM_SMCR_OFFSET    0x0008  /* Slave mode control register (16-bit) */
#define STM32_ATIM_DIER_OFFSET    0x000c  /* DMA/Interrupt enable register (16-bit) */
#define STM32_ATIM_SR_OFFSET      0x0010  /* Status register (16-bit*) */
#define STM32_ATIM_EGR_OFFSET     0x0014  /* Event generation register (16-bit) */
#define STM32_ATIM_CCMR1_OFFSET   0x0018  /* Capture/compare mode register 1 (16-bit*) */
#define STM32_ATIM_CCMR2_OFFSET   0x001c  /* Capture/compare mode register 2 (16-bit*) */
#define STM32_ATIM_CCER_OFFSET    0x0020  /* Capture/compare enable register (16-bit*) */
#define STM32_ATIM_CNT_OFFSET     0x0024  /* Counter (16-bit) */
#define STM32_ATIM_PSC_OFFSET     0x0028  /* Prescaler (16-bit) */
#define STM32_ATIM_ARR_OFFSET     0x002c  /* Auto-reload register (16-bit) */
#define STM32_ATIM_RCR_OFFSET     0x0030  /* Repetition counter register (16-bit) */
#define STM32_ATIM_CCR1_OFFSET    0x0034  /* Capture/compare register 1 (16-bit) */
#define STM32_ATIM_CCR2_OFFSET    0x0038  /* Capture/compare register 2 (16-bit) */
#define STM32_ATIM_CCR3_OFFSET    0x003c  /* Capture/compare register 3 (16-bit) */
#define STM32_ATIM_CCR4_OFFSET    0x0040  /* Capture/compare register 4 (16-bit) */
#define STM32_ATIM_BDTR_OFFSET    0x0044  /* Break and dead-time register (16-bit*) */
#define STM32_ATIM_DCR_OFFSET     0x0048  /* DMA control register (16-bit) */
#define STM32_ATIM_DMAR_OFFSET    0x004c  /* DMA address for burst mode (16-bit) */

/* *Note that many of the above registers are 32-bits wide on the F3 */

#ifdef CONFIG_STM32_STM32F30XX
#  define STM32_ATIM_CCMR3_OFFSET 0x0054  /* Capture/compare mode register 3 (32-bit) */
#  define STM32_ATIM_CCR5_OFFSET  0x0058  /* Capture/compare register 4 (16-bit) */
#  define STM32_ATIM_CCR6_OFFSET  0x005c  /* Capture/compare register 4 (32-bit) */
#endif

/* Register Addresses *******************************************************************************/

/* Advanced Timers - TIM1 and TIM8 */

#if STM32_NATIM > 0
#  define STM32_TIM1_CR1         (STM32_TIM1_BASE+STM32_ATIM_CR1_OFFSET)
#  define STM32_TIM1_CR2         (STM32_TIM1_BASE+STM32_ATIM_CR2_OFFSET)
#  define STM32_TIM1_SMCR        (STM32_TIM1_BASE+STM32_ATIM_SMCR_OFFSET)
#  define STM32_TIM1_DIER        (STM32_TIM1_BASE+STM32_ATIM_DIER_OFFSET)
#  define STM32_TIM1_SR          (STM32_TIM1_BASE+STM32_ATIM_SR_OFFSET)
#  define STM32_TIM1_EGR         (STM32_TIM1_BASE+STM32_ATIM_EGR_OFFSET)
#  define STM32_TIM1_CCMR1       (STM32_TIM1_BASE+STM32_ATIM_CCMR1_OFFSET)
#  define STM32_TIM1_CCMR2       (STM32_TIM1_BASE+STM32_ATIM_CCMR2_OFFSET)
#  define STM32_TIM1_CCER        (STM32_TIM1_BASE+STM32_ATIM_CCER_OFFSET)
#  define STM32_TIM1_CNT         (STM32_TIM1_BASE+STM32_ATIM_CNT_OFFSET)
#  define STM32_TIM1_PSC         (STM32_TIM1_BASE+STM32_ATIM_PSC_OFFSET)
#  define STM32_TIM1_ARR         (STM32_TIM1_BASE+STM32_ATIM_ARR_OFFSET)
#  define STM32_TIM1_RCR         (STM32_TIM1_BASE+STM32_ATIM_RCR_OFFSET)
#  define STM32_TIM1_CCR1        (STM32_TIM1_BASE+STM32_ATIM_CCR1_OFFSET)
#  define STM32_TIM1_CCR2        (STM32_TIM1_BASE+STM32_ATIM_CCR2_OFFSET)
#  define STM32_TIM1_CCR3        (STM32_TIM1_BASE+STM32_ATIM_CCR3_OFFSET)
#  define STM32_TIM1_CCR4        (STM32_TIM1_BASE+STM32_ATIM_CCR4_OFFSET)
#  define STM32_TIM1_BDTR        (STM32_TIM1_BASE+STM32_ATIM_BDTR_OFFSET)
#  define STM32_TIM1_DCR         (STM32_TIM1_BASE+STM32_ATIM_DCR_OFFSET)
#  define STM32_TIM1_DMAR        (STM32_TIM1_BASE+STM32_ATIM_DMAR_OFFSET)
#  ifdef CONFIG_STM32_STM32F30XX
#    define STM32_TIM1_CCMR3     (STM32_TIM1_BASE+STM32_ATIM_CCMR3_OFFSET)
#    define STM32_TIM1_CCR5      (STM32_TIM1_BASE+STM32_ATIM_CCR5_OFFSET)
#    define STM32_TIM1_CCR6      (STM32_TIM1_BASE+STM32_ATIM_CCR6_OFFSET)
#  endif
#endif

#if STM32_NATIM > 1
#  define STM32_TIM8_CR1         (STM32_TIM8_BASE+STM32_ATIM_CR1_OFFSET)
#  define STM32_TIM8_CR2         (STM32_TIM8_BASE+STM32_ATIM_CR2_OFFSET)
#  define STM32_TIM8_SMCR        (STM32_TIM8_BASE+STM32_ATIM_SMCR_OFFSET)
#  define STM32_TIM8_DIER        (STM32_TIM8_BASE+STM32_ATIM_DIER_OFFSET)
#  define STM32_TIM8_SR          (STM32_TIM8_BASE+STM32_ATIM_SR_OFFSET)
#  define STM32_TIM8_EGR         (STM32_TIM8_BASE+STM32_ATIM_EGR_OFFSET)
#  define STM32_TIM8_CCMR1       (STM32_TIM8_BASE+STM32_ATIM_CCMR1_OFFSET)
#  define STM32_TIM8_CCMR2       (STM32_TIM8_BASE+STM32_ATIM_CCMR2_OFFSET)
#  define STM32_TIM8_CCER        (STM32_TIM8_BASE+STM32_ATIM_CCER_OFFSET)
#  define STM32_TIM8_CNT         (STM32_TIM8_BASE+STM32_ATIM_CNT_OFFSET)
#  define STM32_TIM8_PSC         (STM32_TIM8_BASE+STM32_ATIM_PSC_OFFSET)
#  define STM32_TIM8_ARR         (STM32_TIM8_BASE+STM32_ATIM_ARR_OFFSET)
#  define STM32_TIM8_RCR         (STM32_TIM8_BASE+STM32_ATIM_RCR_OFFSET)
#  define STM32_TIM8_CCR1        (STM32_TIM8_BASE+STM32_ATIM_CCR1_OFFSET)
#  define STM32_TIM8_CCR2        (STM32_TIM8_BASE+STM32_ATIM_CCR2_OFFSET)
#  define STM32_TIM8_CCR3        (STM32_TIM8_BASE+STM32_ATIM_CCR3_OFFSET)
#  define STM32_TIM8_CCR4        (STM32_TIM8_BASE+STM32_ATIM_CCR4_OFFSET)
#  define STM32_TIM8_BDTR        (STM32_TIM8_BASE+STM32_ATIM_BDTR_OFFSET)
#  define STM32_TIM8_DCR         (STM32_TIM8_BASE+STM32_ATIM_DCR_OFFSET)
#  define STM32_TIM8_DMAR        (STM32_TIM8_BASE+STM32_ATIM_DMAR_OFFSET)
#  ifdef CONFIG_STM32_STM32F30XX
#    define STM32_TIM8_CCMR3     (STM32_TIM8_BASE+STM32_ATIM_CCMR3_OFFSET)
#    define STM32_TIM8_CCR5      (STM32_TIM8_BASE+STM32_ATIM_CCR5_OFFSET)
#    define STM32_TIM8_CCR6      (STM32_TIM8_BASE+STM32_ATIM_CCR6_OFFSET)
#  endif
#endif

/* 16-/32-bit General Timers - TIM2, TIM3, TIM4, and TIM5 with DMA.
 * For the STM32F10xx all timers are 16-bit.
 * For the STM32F2xx and STM32F40xx, TIM2 and 5 are 32-bit
 */

#if STM32_NGTIM > 0
#  define STM32_TIM2_CR1         (STM32_TIM2_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM2_CR2         (STM32_TIM2_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM2_SMCR        (STM32_TIM2_BASE+STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM2_DIER        (STM32_TIM2_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM2_SR          (STM32_TIM2_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM2_EGR         (STM32_TIM2_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM2_CCMR1       (STM32_TIM2_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM2_CCMR2       (STM32_TIM2_BASE+STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM2_CCER        (STM32_TIM2_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM2_CNT         (STM32_TIM2_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM2_PSC         (STM32_TIM2_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM2_ARR         (STM32_TIM2_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM2_CCR1        (STM32_TIM2_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM2_CCR2        (STM32_TIM2_BASE+STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM2_CCR3        (STM32_TIM2_BASE+STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM2_CCR4        (STM32_TIM2_BASE+STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM2_DCR         (STM32_TIM2_BASE+STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM2_DMAR        (STM32_TIM2_BASE+STM32_GTIM_DMAR_OFFSET)
#  if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    define STM32_TIM2_OR        (STM32_TIM2_BASE+STM32_GTIM_OR_OFFSET)
#  endif
#endif

#if STM32_NGTIM > 1
#  define STM32_TIM3_CR1         (STM32_TIM3_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM3_CR2         (STM32_TIM3_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM3_SMCR        (STM32_TIM3_BASE+STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM3_DIER        (STM32_TIM3_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM3_SR          (STM32_TIM3_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM3_EGR         (STM32_TIM3_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM3_CCMR1       (STM32_TIM3_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM3_CCMR2       (STM32_TIM3_BASE+STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM3_CCER        (STM32_TIM3_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM3_CNT         (STM32_TIM3_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM3_PSC         (STM32_TIM3_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM3_ARR         (STM32_TIM3_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM3_CCR1        (STM32_TIM3_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM3_CCR2        (STM32_TIM3_BASE+STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM3_CCR3        (STM32_TIM3_BASE+STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM3_CCR4        (STM32_TIM3_BASE+STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM3_DCR         (STM32_TIM3_BASE+STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM3_DMAR        (STM32_TIM3_BASE+STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 2
#  define STM32_TIM4_CR1         (STM32_TIM4_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM4_CR2         (STM32_TIM4_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM4_SMCR        (STM32_TIM4_BASE+STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM4_DIER        (STM32_TIM4_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM4_SR          (STM32_TIM4_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM4_EGR         (STM32_TIM4_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM4_CCMR1       (STM32_TIM4_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM4_CCMR2       (STM32_TIM4_BASE+STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM4_CCER        (STM32_TIM4_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM4_CNT         (STM32_TIM4_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM4_PSC         (STM32_TIM4_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM4_ARR         (STM32_TIM4_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM4_CCR1        (STM32_TIM4_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM4_CCR2        (STM32_TIM4_BASE+STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM4_CCR3        (STM32_TIM4_BASE+STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM4_CCR4        (STM32_TIM4_BASE+STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM4_DCR         (STM32_TIM4_BASE+STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM4_DMAR        (STM32_TIM4_BASE+STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 3
#  define STM32_TIM5_CR1         (STM32_TIM5_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM5_CR2         (STM32_TIM5_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM5_SMCR        (STM32_TIM5_BASE+STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM5_DIER        (STM32_TIM5_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM5_SR          (STM32_TIM5_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM5_EGR         (STM32_TIM5_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM5_CCMR1       (STM32_TIM5_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM5_CCMR2       (STM32_TIM5_BASE+STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM5_CCER        (STM32_TIM5_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM5_CNT         (STM32_TIM5_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM5_PSC         (STM32_TIM5_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM5_ARR         (STM32_TIM5_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM5_CCR1        (STM32_TIM5_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM5_CCR2        (STM32_TIM5_BASE+STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM5_CCR3        (STM32_TIM5_BASE+STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM5_CCR4        (STM32_TIM5_BASE+STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM5_DCR         (STM32_TIM5_BASE+STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM5_DMAR        (STM32_TIM5_BASE+STM32_GTIM_DMAR_OFFSET)
#  if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    define STM32_TIM5_OR        (STM32_TIM5_BASE+STM32_GTIM_OR_OFFSET)
#  endif
#endif

#define STM32_TIM15_CR1          (STM32_TIM15_BASE+STM32_GTIM_CR1_OFFSET)
#define STM32_TIM15_CR2          (STM32_TIM15_BASE+STM32_GTIM_CR2_OFFSET)
#define STM32_TIM15_SMCR         (STM32_TIM15_BASE+STM32_GTIM_SMCR_OFFSET)
#define STM32_TIM15_DIER         (STM32_TIM15_BASE+STM32_GTIM_DIER_OFFSET)
#define STM32_TIM15_SR           (STM32_TIM15_BASE+STM32_GTIM_SR_OFFSET)
#define STM32_TIM15_EGR          (STM32_TIM15_BASE+STM32_GTIM_EGR_OFFSET)
#define STM32_TIM15_CCMR1        (STM32_TIM15_BASE+STM32_GTIM_CCMR1_OFFSET)
#define STM32_TIM15_CCER         (STM32_TIM15_BASE+STM32_GTIM_CCER_OFFSET)
#define STM32_TIM15_CNT          (STM32_TIM15_BASE+STM32_GTIM_CNT_OFFSET)
#define STM32_TIM15_PSC          (STM32_TIM15_BASE+STM32_GTIM_PSC_OFFSET)
#define STM32_TIM15_ARR          (STM32_TIM15_BASE+STM32_GTIM_ARR_OFFSET)
#define STM32_TIM15_RCR          (STM32_TIM15_BASE+STM32_GTIM_RCR_OFFSET)
#define STM32_TIM15_CCR1         (STM32_TIM15_BASE+STM32_GTIM_CCR1_OFFSET)
#define STM32_TIM15_CCR2         (STM32_TIM15_BASE+STM32_GTIM_CCR2_OFFSET)
#define STM32_TIM15_BDTR         (STM32_TIM15_BASE+STM32_GTIM_BDTR_OFFSET)
#define STM32_TIM15_DCR          (STM32_TIM15_BASE+STM32_GTIM_DCR_OFFSET)
#define STM32_TIM15_DMAR         (STM32_TIM15_BASE+STM32_GTIM_DMAR_OFFSET)

#define STM32_TIM16_CR1          (STM32_TIM16_BASE+STM32_GTIM_CR1_OFFSET)
#define STM32_TIM16_CR2          (STM32_TIM16_BASE+STM32_GTIM_CR2_OFFSET)
#define STM32_TIM16_DIER         (STM32_TIM16_BASE+STM32_GTIM_DIER_OFFSET)
#define STM32_TIM16_SR           (STM32_TIM16_BASE+STM32_GTIM_SR_OFFSET)
#define STM32_TIM16_EGR          (STM32_TIM16_BASE+STM32_GTIM_EGR_OFFSET)
#define STM32_TIM16_CCMR1        (STM32_TIM16_BASE+STM32_GTIM_CCMR1_OFFSET)
#define STM32_TIM16_CCMR2        (STM32_TIM16_BASE+STM32_GTIM_CCMR2_OFFSET)
#define STM32_TIM16_CCER         (STM32_TIM16_BASE+STM32_GTIM_CCER_OFFSET)
#define STM32_TIM16_CNT          (STM32_TIM16_BASE+STM32_GTIM_CNT_OFFSET)
#define STM32_TIM16_PSC          (STM32_TIM16_BASE+STM32_GTIM_PSC_OFFSET)
#define STM32_TIM16_ARR          (STM32_TIM16_BASE+STM32_GTIM_ARR_OFFSET)
#define STM32_TIM16_RCR          (STM32_TIM16_BASE+STM32_GTIM_RCR_OFFSET)
#define STM32_TIM16_CCR1         (STM32_TIM16_BASE+STM32_GTIM_CCR1_OFFSET)
#define STM32_TIM16_BDTR         (STM32_TIM16_BASE+STM32_GTIM_BDTR_OFFSET)
#define STM32_TIM16_DCR          (STM32_TIM16_BASE+STM32_GTIM_DCR_OFFSET)
#define STM32_TIM16_DMAR         (STM32_TIM16_BASE+STM32_GTIM_DMAR_OFFSET)
#define STM32_TIM16_OR           (STM32_TIM16_BASE+STM32_GTIM_OR_OFFSET)

#define STM32_TIM17_CR1          (STM32_TIM17_BASE+STM32_GTIM_CR1_OFFSET)
#define STM32_TIM17_CR2          (STM32_TIM17_BASE+STM32_GTIM_CR2_OFFSET)
#define STM32_TIM17_DIER         (STM32_TIM17_BASE+STM32_GTIM_DIER_OFFSET)
#define STM32_TIM17_SR           (STM32_TIM17_BASE+STM32_GTIM_SR_OFFSET)
#define STM32_TIM17_EGR          (STM32_TIM17_BASE+STM32_GTIM_EGR_OFFSET)
#define STM32_TIM17_CCMR1        (STM32_TIM17_BASE+STM32_GTIM_CCMR1_OFFSET)
#define STM32_TIM17_CCMR2        (STM32_TIM17_BASE+STM32_GTIM_CCMR2_OFFSET)
#define STM32_TIM17_CCER         (STM32_TIM17_BASE+STM32_GTIM_CCER_OFFSET)
#define STM32_TIM17_CNT          (STM32_TIM17_BASE+STM32_GTIM_CNT_OFFSET)
#define STM32_TIM17_PSC          (STM32_TIM17_BASE+STM32_GTIM_PSC_OFFSET)
#define STM32_TIM17_ARR          (STM32_TIM17_BASE+STM32_GTIM_ARR_OFFSET)
#define STM32_TIM17_RCR          (STM32_TIM17_BASE+STM32_GTIM_RCR_OFFSET)
#define STM32_TIM17_CCR1         (STM32_TIM17_BASE+STM32_GTIM_CCR1_OFFSET)
#define STM32_TIM17_BDTR         (STM32_TIM17_BASE+STM32_GTIM_BDTR_OFFSET)
#define STM32_TIM17_DCR          (STM32_TIM17_BASE+STM32_GTIM_DCR_OFFSET)
#define STM32_TIM17_DMAR         (STM32_TIM17_BASE+STM32_GTIM_DMAR_OFFSET)

/* 16-bit General Timers - TIM9-14 without DMA.  Note that (1) these timers
 * support only a subset of the general timer registers are supported, and
 * (2) TIM9 and TIM12 differ from the others.
 */

#if STM32_NGTIMNDMA > 0
#  define STM32_TIM9_CR1         (STM32_TIM9_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM9_CR2         (STM32_TIM9_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM9_DIER        (STM32_TIM9_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM9_SR          (STM32_TIM9_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM9_EGR         (STM32_TIM9_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM9_CCMR1       (STM32_TIM9_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM9_CCER        (STM32_TIM9_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM9_CNT         (STM32_TIM9_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM9_PSC         (STM32_TIM9_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM9_ARR         (STM32_TIM9_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM9_CCR1        (STM32_TIM9_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM9_CCR2        (STM32_TIM9_BASE+STM32_GTIM_CCR2_OFFSET)
#endif

#if STM32_NGTIMNDMA > 1
#  define STM32_TIM10_CR1         (STM32_TIM10_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM10_DIER        (STM32_TIM10_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM10_SR          (STM32_TIM10_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM10_EGR         (STM32_TIM10_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM10_CCMR1       (STM32_TIM10_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM10_CCER        (STM32_TIM10_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM10_CNT         (STM32_TIM10_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM10_PSC         (STM32_TIM10_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM10_ARR         (STM32_TIM10_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM10_CCR1        (STM32_TIM10_BASE+STM32_GTIM_CCR1_OFFSET)
#endif

#if STM32_NGTIMNDMA > 2
#  define STM32_TIM11_CR1         (STM32_TIM11_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM11_DIER        (STM32_TIM11_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM11_SR          (STM32_TIM11_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM11_EGR         (STM32_TIM11_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM11_CCMR1       (STM32_TIM11_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM11_CCER        (STM32_TIM11_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM11_CNT         (STM32_TIM11_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM11_PSC         (STM32_TIM11_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM11_ARR         (STM32_TIM11_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM11_CCR1        (STM32_TIM11_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM11_OR        (STM32_TIM11_BASE+STM32_GTIM_OR_OFFSET)
#endif

#if STM32_NGTIMNDMA > 3
#  define STM32_TIM12_CR1         (STM32_TIM12_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM12_CR2         (STM32_TIM9_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM12_DIER        (STM32_TIM12_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM12_SR          (STM32_TIM12_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM12_EGR         (STM32_TIM12_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM12_CCMR1       (STM32_TIM12_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM12_CCER        (STM32_TIM12_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM12_CNT         (STM32_TIM12_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM12_PSC         (STM32_TIM12_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM12_ARR         (STM32_TIM12_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM12_CCR1        (STM32_TIM12_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM12_CCR2        (STM32_TIM12_BASE+STM32_GTIM_CCR2_OFFSET)
#endif

#if STM32_NGTIMNDMA > 4
#  define STM32_TIM13_CR1         (STM32_TIM13_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM13_DIER        (STM32_TIM13_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM13_SR          (STM32_TIM13_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM13_EGR         (STM32_TIM13_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM13_CCMR1       (STM32_TIM13_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM13_CCER        (STM32_TIM13_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM13_CNT         (STM32_TIM13_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM13_PSC         (STM32_TIM13_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM13_ARR         (STM32_TIM13_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM13_CCR1        (STM32_TIM13_BASE+STM32_GTIM_CCR1_OFFSET)
#endif

#if STM32_NGTIMNDMA > 5
#  define STM32_TIM14_CR1         (STM32_TIM14_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM14_DIER        (STM32_TIM14_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM14_SR          (STM32_TIM14_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM14_EGR         (STM32_TIM14_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM14_CCMR1       (STM32_TIM14_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM14_CCER        (STM32_TIM14_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM14_CNT         (STM32_TIM14_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM14_PSC         (STM32_TIM14_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM14_ARR         (STM32_TIM14_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM14_CCR1        (STM32_TIM14_BASE+STM32_GTIM_CCR1_OFFSET)
#endif

/* Basic Timers - TIM6 and TIM7 */

#if STM32_NBTIM > 0
#  define STM32_TIM6_CR1         (STM32_TIM6_BASE+STM32_BTIM_CR1_OFFSET)
#  define STM32_TIM6_CR2         (STM32_TIM6_BASE+STM32_BTIM_CR2_OFFSET)
#  define STM32_TIM6_DIER        (STM32_TIM6_BASE+STM32_BTIM_DIER_OFFSET)
#  define STM32_TIM6_SR          (STM32_TIM6_BASE+STM32_BTIM_SR_OFFSET)
#  define STM32_TIM6_EGR         (STM32_TIM6_BASE+STM32_BTIM_EGR_OFFSET)
#  define STM32_TIM6_CNT         (STM32_TIM6_BASE+STM32_BTIM_CNT_OFFSET)
#  define STM32_TIM6_PSC         (STM32_TIM6_BASE+STM32_BTIM_PSC_OFFSET)
#  define STM32_TIM6_ARR         (STM32_TIM6_BASE+STM32_BTIM_ARR_OFFSET)
#endif

#if STM32_NBTIM > 1
#  define STM32_TIM7_CR1         (STM32_TIM7_BASE+STM32_BTIM_CR1_OFFSET)
#  define STM32_TIM7_CR2         (STM32_TIM7_BASE+STM32_BTIM_CR2_OFFSET)
#  define STM32_TIM7_DIER        (STM32_TIM7_BASE+STM32_BTIM_DIER_OFFSET)
#  define STM32_TIM7_SR          (STM32_TIM7_BASE+STM32_BTIM_SR_OFFSET)
#  define STM32_TIM7_EGR         (STM32_TIM7_BASE+STM32_BTIM_EGR_OFFSET)
#  define STM32_TIM7_CNT         (STM32_TIM7_BASE+STM32_BTIM_CNT_OFFSET)
#  define STM32_TIM7_PSC         (STM32_TIM7_BASE+STM32_BTIM_PSC_OFFSET)
#  define STM32_TIM7_ARR         (STM32_TIM7_BASE+STM32_BTIM_ARR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************************************/

/* Control register 1 */

#define ATIM_CR1_CEN               (1 << 0)  /* Bit 0: Counter enable */
#define ATIM_CR1_UDIS              (1 << 1)  /* Bit 1: Update disable */
#define ATIM_CR1_URS               (1 << 2)  /* Bit 2: Update request source */
#define ATIM_CR1_OPM               (1 << 3)  /* Bit 3: One pulse mode */
#define ATIM_CR1_DIR               (1 << 4)  /* Bit 4: Direction */
#define ATIM_CR1_CMS_SHIFT         (5)       /* Bits 6-5: Center-aligned mode selection */
#define ATIM_CR1_CMS_MASK          (3 << ATIM_CR1_CMS_SHIFT)
#  define ATIM_CR1_EDGE            (0 << ATIM_CR1_CMS_SHIFT) /* 00: Edge-aligned mode */
#  define ATIM_CR1_CENTER1         (1 << ATIM_CR1_CMS_SHIFT) /* 01: Center-aligned mode 1 */
#  define ATIM_CR1_CENTER2         (2 << ATIM_CR1_CMS_SHIFT) /* 10: Center-aligned mode 2 */
#  define ATIM_CR1_CENTER3         (3 << ATIM_CR1_CMS_SHIFT) /* 11: Center-aligned mode 3 */
#define ATIM_CR1_ARPE              (1 << 7)  /* Bit 7: Auto-reload preload enable */
#define ATIM_CR1_CKD_SHIFT         (8)       /* Bits 9-8: Clock division */
#define ATIM_CR1_CKD_MASK          (3 << ATIM_CR1_CKD_SHIFT)
#  define ATIM_CR1_TCKINT          (0 << ATIM_CR1_CKD_SHIFT) /* 00: tDTS=tCK_INT */
#  define ATIM_CR1_2TCKINT         (1 << ATIM_CR1_CKD_SHIFT) /* 01: tDTS=2*tCK_INT */
#  define ATIM_CR1_4TCKINT         (2 << ATIM_CR1_CKD_SHIFT) /* 10: tDTS=4*tCK_INT */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_CR1_UIFREMAP        (1 << 11) /* Bit 11: UIF status bit remapping */
#endif

/* Control register 2 */

#define ATIM_CR2_CCPC             (1 << 0)  /* Bit 0:  Capture/Compare Preloaded Control */
#define ATIM_CR2_CCUS             (1 << 2)  /* Bit 2:  Capture/Compare Control Update Selection */
#define ATIM_CR2_CCDS             (1 << 3)  /* Bit 3:  Capture/Compare DMA Selection */
#define ATIM_CR2_MMS_SHIFT        (4)       /* Bits 6-4: Master Mode Selection */
#define ATIM_CR2_MMS_MASK         (7 << ATIM_CR2_MMS_SHIFT)
#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_CR2_MMS_RESET      (0 << ATIM_CR2_MMS_SHIFT) /* 000: Reset - TIMx_EGR UG bit is TRG9 */
#  define ATIM_CR2_MMS_ENABLE     (1 << ATIM_CR2_MMS_SHIFT) /* 001: Enable - CNT_EN is TRGO */
#  define ATIM_CR2_MMS_UPDATE     (2 << ATIM_CR2_MMS_SHIFT) /* 010: Update event is TRGH0*/
#  define ATIM_CR2_MMS_COMPP      (3 << ATIM_CR2_MMS_SHIFT) /* 010: Compare Pulse - CC1IF flag */
#endif
#  define ATIM_CR2_MMS_OC1REF     (4 << ATIM_CR2_MMS_SHIFT) /* 100: Compare OC1REF is TRGO */
#  define ATIM_CR2_MMS_OC2REF     (5 << ATIM_CR2_MMS_SHIFT) /* 101: Compare OC2REF is TRGO */
#  define ATIM_CR2_MMS_OC3REF     (6 << ATIM_CR2_MMS_SHIFT) /* 110: Compare OC3REF is TRGO */
#  define ATIM_CR2_MMS_OC4REF     (7 << ATIM_CR2_MMS_SHIFT) /* 111: Compare OC4REF is TRGO */
#define ATIM_CR2_TI1S             (1 << 7)  /* Bit 7: TI1 Selection */
#define ATIM_CR2_OIS1             (1 << 8)  /* Bit 8:  Output Idle state 1 (OC1 output) */
#define ATIM_CR2_OIS1N            (1 << 9)  /* Bit 9:  Output Idle state 1 (OC1N output) */
#define ATIM_CR2_OIS2             (1 << 10) /* Bit 10: Output Idle state 2 (OC2 output) */
#define ATIM_CR2_OIS2N            (1 << 11) /* Bit 11: Output Idle state 2 (OC2N output) */
#define ATIM_CR2_OIS3             (1 << 12) /* Bit 12: Output Idle state 3 (OC3 output) */
#define ATIM_CR2_OIS3N            (1 << 13) /* Bit 13: Output Idle state 3 (OC3N output) */
#define ATIM_CR2_OIS4             (1 << 14) /* Bit 14: Output Idle state 4 (OC4 output) */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_CR2_OIS5             (1 << 16) /* Bit 16: OOutput Idle state 5 (OC5 output) */
#  define ATIM_CR2_OIS6             (1 << 18) /* Bit 18: Output Idle state 6 (OC6 output) */
#  define ATIM_CR2_MMS2_SHIFT       (20)      /* Bits 20-23: Master Mode Selection 2 */
#  define ATIM_CR2_MMS2_MASK        (15 << ATIM_CR2_MMS2_SHIFT)
#    define ATIM_CR2_MMS2_RESET     (0 << ATIM_CR2_MMS2_SHIFT)  /* 0000: Reset - TIMx_EGR UG bit is TRG9 */
#    define ATIM_CR2_MMS2_ENABLE    (1 << ATIM_CR2_MMS2_SHIFT)  /* 0001: Enable - CNT_EN is TRGO2 */
#    define ATIM_CR2_MMS2_UPDATE    (2 << ATIM_CR2_MMS2_SHIFT)  /* 0010: Update event is TRGH0*/
#    define ATIM_CR2_MMS2_COMPP     (3 << ATIM_CR2_MMS2_SHIFT)  /* 0010: Compare Pulse - CC1IF flag */
#    define ATIM_CR2_MMS2_OC1REF    (4 << ATIM_CR2_MMS2_SHIFT)  /* 0100: Compare OC1REF is TRGO2 */
#    define ATIM_CR2_MMS2_OC2REF    (5 << ATIM_CR2_MMS2_SHIFT)  /* 0101: Compare OC2REF is TRGO2 */
#    define ATIM_CR2_MMS2_OC3REF    (6 << ATIM_CR2_MMS2_SHIFT)  /* 0110: Compare OC3REF is TRGO2 */
#    define ATIM_CR2_MMS2_OC4REF    (7 << ATIM_CR2_MMS2_SHIFT)  /* 0111: Compare OC4REF is TRGO2 */
#    define ATIM_CR2_MMS2_OC5REF    (8 << ATIM_CR2_MMS2_SHIFT)  /* 1000: Compare OC5REF is TRGO2 */
#    define ATIM_CR2_MMS2_OC6REF    (9 << ATIM_CR2_MMS2_SHIFT)  /* 1001: Compare OC6REF is TRGO2 */
#    define ATIM_CR2_MMS2_CMPOC4    (10 << ATIM_CR2_MMS2_SHIFT) /* 1010: Compare pulse - OC4REF edge is TRGO2 */
#    define ATIM_CR2_MMS2_CMPOC6    (11 << ATIM_CR2_MMS2_SHIFT) /* 1011: Compare pulse - OC6REF edge is TRGO2 */
#    define ATIM_CR2_MMS2_CMPOC4R6R (12 << ATIM_CR2_MMS2_SHIFT) /* 1100: Compare pulse - OC4REF/OC6REF rising */
#    define ATIM_CR2_MMS2_CMPOC4R6F (13 << ATIM_CR2_MMS2_SHIFT) /* 1101: Compare pulse - OC4REF rising/OC6REF falling */
#    define ATIM_CR2_MMS2_CMPOC5R6R (14 << ATIM_CR2_MMS2_SHIFT) /* 1110: Compare pulse - OC5REF/OC6REF rising */
#    define ATIM_CR2_MMS2_CMPOC5R6F (15 << ATIM_CR2_MMS2_SHIFT) /* 1111: Compare pulse - OC5REF rising/OC6REF falling */
#endif

/* Slave mode control register */

#define ATIM_SMCR_SMS_SHIFT       (0)       /* Bits 0-2: Slave mode selection */
#define ATIM_SMCR_SMS_MASK        (7 << ATIM_SMCR_SMS_SHIFT)
#  define ATIM_SMCR_DISAB         (0 << ATIM_SMCR_SMS_SHIFT) /* 000: Slave mode disabled */
#  define ATIM_SMCR_ENCMD1        (1 << ATIM_SMCR_SMS_SHIFT) /* 001: Encoder mode 1 */
#  define ATIM_SMCR_ENCMD2        (2 << ATIM_SMCR_SMS_SHIFT) /* 010: Encoder mode 2 */
#  define ATIM_SMCR_ENCMD3        (3 << ATIM_SMCR_SMS_SHIFT) /* 011: Encoder mode 3 */
#  define ATIM_SMCR_RESET         (4 << ATIM_SMCR_SMS_SHIFT) /* 100: Reset Mode */
#  define ATIM_SMCR_GATED         (5 << ATIM_SMCR_SMS_SHIFT) /* 101: Gated Mode */
#  define ATIM_SMCR_TRIGGER       (6 << ATIM_SMCR_SMS_SHIFT) /* 110: Trigger Mode */
#  define ATIM_SMCR_EXTCLK1       (7 << ATIM_SMCR_SMS_SHIFT) /* 111: External Clock Mode 1 */
#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_SMCR_OCCS          (1 << 3)  /* Bit 3: OCREF clear selection */
#endif
#define ATIM_SMCR_TS_SHIFT        (4)       /* Bits 4-6: Trigger selection */
#define ATIM_SMCR_TS_MASK         (7 << ATIM_SMCR_TS_SHIFT)
#  define ATIM_SMCR_ITR0          (0 << ATIM_SMCR_TS_SHIFT) /* 000: Internal trigger 0 (ITR0) */
#  define ATIM_SMCR_ITR1          (1 << ATIM_SMCR_TS_SHIFT) /* 001: Internal trigger 1 (ITR1) */
#  define ATIM_SMCR_ITR2          (2 << ATIM_SMCR_TS_SHIFT) /* 010: Internal trigger 2 (ITR2) */
#  define ATIM_SMCR_ITR3          (3 << ATIM_SMCR_TS_SHIFT) /* 011: Internal trigger 3 (ITR3) */
#  define ATIM_SMCR_T1FED         (4 << ATIM_SMCR_TS_SHIFT) /* 100: TI1 Edge Detector (TI1F_ED) */
#  define ATIM_SMCR_TI1FP1        (5 << ATIM_SMCR_TS_SHIFT) /* 101: Filtered Timer Input 1 (TI1FP1) */
#  define ATIM_SMCR_T12FP2        (6 << ATIM_SMCR_TS_SHIFT) /* 110: Filtered Timer Input 2 (TI2FP2) */
#  define ATIM_SMCR_ETRF          (7 << ATIM_SMCR_TS_SHIFT) /* 111: External Trigger input (ETRF) */
#define ATIM_SMCR_MSM             (1 << 7)  /* Bit 7: Master/slave mode */
#define ATIM_SMCR_ETF_SHIFT       (8)       /* Bits 8-11: External trigger filter */
#define ATIM_SMCR_ETF_MASK        (0x0f << ATIM_SMCR_ETF_SHIFT)
#  define ATIM_SMCR_NOFILT        (0 << ATIM_SMCR_ETF_SHIFT) /* 0000: No filter, sampling is done at fDTS */
#  define ATIM_SMCR_FCKINT2       (1 << ATIM_SMCR_ETF_SHIFT) /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define ATIM_SMCR_FCKINT4       (2 << ATIM_SMCR_ETF_SHIFT) /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define ATIM_SMCR_FCKINT8       (3 << ATIM_SMCR_ETF_SHIFT) /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define ATIM_SMCR_FDTSd26       (4 << ATIM_SMCR_ETF_SHIFT) /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define ATIM_SMCR_FDTSd28       (5 << ATIM_SMCR_ETF_SHIFT) /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define ATIM_SMCR_FDTSd46       (6 << ATIM_SMCR_ETF_SHIFT) /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define ATIM_SMCR_FDTSd48       (7 << ATIM_SMCR_ETF_SHIFT) /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define ATIM_SMCR_FDTSd86       (8 << ATIM_SMCR_ETF_SHIFT) /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define ATIM_SMCR_FDTSd88       (9 << ATIM_SMCR_ETF_SHIFT) /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define ATIM_SMCR_FDTSd165      (10 << ATIM_SMCR_ETF_SHIFT) /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define ATIM_SMCR_FDTSd166      (11 << ATIM_SMCR_ETF_SHIFT) /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define ATIM_SMCR_FDTSd168      (12 << ATIM_SMCR_ETF_SHIFT) /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define ATIM_SMCR_FDTSd325      (13 << ATIM_SMCR_ETF_SHIFT) /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define ATIM_SMCR_FDTSd326      (14 << ATIM_SMCR_ETF_SHIFT) /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define ATIM_SMCR_FDTSd328      (15 << ATIM_SMCR_ETF_SHIFT) /* 1111: fSAMPLING=fDTS/32, N=8 */
#define ATIM_SMCR_ETPS_SHIFT      (12)      /* Bits 12-13: External trigger prescaler */
#define ATIM_SMCR_ETPS_MASK       (3 << ATIM_SMCR_ETPS_SHIFT)
#  define ATIM_SMCR_PSCOFF        (0 << ATIM_SMCR_ETPS_SHIFT) /* 00: Prescaler OFF */
#  define ATIM_SMCR_ETRPd2        (1 << ATIM_SMCR_ETPS_SHIFT) /* 01: ETRP frequency divided by 2 */
#  define ATIM_SMCR_ETRPd4        (2 << ATIM_SMCR_ETPS_SHIFT) /* 10: ETRP frequency divided by 4 */
#  define ATIM_SMCR_ETRPd8        (3 << ATIM_SMCR_ETPS_SHIFT) /* 11: ETRP frequency divided by 8 */
#define ATIM_SMCR_ECE             (1 << 14) /* Bit 14: External clock enable */
#define ATIM_SMCR_ETP             (1 << 15) /* Bit 15: External trigger polarity */
#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_SMCR_SMS           (1 << 16) /* Bit 16: Slave mode selection - bit 3 */
#endif

/* DMA/Interrupt enable register */

#define ATIM_DIER_UIE             (1 << 0)  /* Bit 0: Update interrupt enable */
#define ATIM_DIER_CC1IE           (1 << 1)  /* Bit 1: Capture/Compare 1 interrupt enable */
#define ATIM_DIER_CC2IE           (1 << 2)  /* Bit 2: Capture/Compare 2 interrupt enable */
#define ATIM_DIER_CC3IE           (1 << 3)  /* Bit 3: Capture/Compare 3 interrupt enable */
#define ATIM_DIER_CC4IE           (1 << 4)  /* Bit 4: Capture/Compare 4 interrupt enable */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#  define ATIM_DIER_COMIE         (1 << 5)  /* Bit 5: COM interrupt enable */
#endif

#define ATIM_DIER_TIE             (1 << 6)  /* Bit 6: Trigger interrupt enable */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#  define ATIM_DIER_BIE           (1 << 7)  /* Bit 7: Break interrupt enable */
#endif

#define ATIM_DIER_UDE             (1 << 8)  /* Bit 8: Update DMA request enable */
#define ATIM_DIER_CC1DE           (1 << 9)  /* Bit 9: Capture/Compare 1 DMA request enable */
#define ATIM_DIER_CC2DE           (1 << 10) /* Bit 10: Capture/Compare 2 DMA request enable */
#define ATIM_DIER_CC3DE           (1 << 11) /* Bit 11: Capture/Compare 3 DMA request enable */
#define ATIM_DIER_CC4DE           (1 << 12) /* Bit 12: Capture/Compare 4 DMA request enable */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#  define ATIM_DIER_COMDE         (1 << 13) /* Bit 13: COM DMA request enable */
#endif

#define ATIM_DIER_TDE             (1 << 14) /* Bit 14: Trigger DMA request enable */

/* Status register */

#define ATIM_SR_UIF               (1 << 0)  /* Bit 0:  Update interrupt Flag */
#define ATIM_SR_CC1IF             (1 << 1)  /* Bit 1:  Capture/Compare 1 interrupt Flag */
#define ATIM_SR_CC2IF             (1 << 2)  /* Bit 2:  Capture/Compare 2 interrupt Flag */
#define ATIM_SR_CC3IF             (1 << 3)  /* Bit 3:  Capture/Compare 3 interrupt Flag */
#define ATIM_SR_CC4IF             (1 << 4)  /* Bit 4:  Capture/Compare 4 interrupt Flag */
#define ATIM_SR_COMIF             (1 << 5)  /* Bit 5:  COM interrupt Flag */
#define ATIM_SR_TIF               (1 << 6)  /* Bit 6:  Trigger interrupt Flag */

#if defined(CONFIG_STM32_STM32F10XX)
#  define ATIM_SR_BIF             (1 << 7)  /* Bit 7:  Break interrupt Flag */
#elif defined(CONFIG_STM32_STM32F30XX)
#  define ATIM_SR_BIF             (1 << 7)  /* Bit 7:  Break interrupt Flag */
#  define ATIM_SR_B2IF            (1 << 8)  /* Bit 8:  Break 2 interrupt Flag */
#endif

#define ATIM_SR_CC1OF             (1 << 9)  /* Bit 9:  Capture/Compare 1 Overcapture Flag */
#define ATIM_SR_CC2OF             (1 << 10) /* Bit 10: Capture/Compare 2 Overcapture Flag */
#define ATIM_SR_CC3OF             (1 << 11) /* Bit 11: Capture/Compare 3 Overcapture Flag */
#define ATIM_SR_CC4OF             (1 << 12) /* Bit 12: Capture/Compare 4 Overcapture Flag */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_SR_CC5IF           (1 << 16) /* Bit 16: Compare 5 interrupt flag */
#  define ATIM_SR_CC6IF           (1 << 17) /* Bit 17: Compare 6 interrupt flag */
#endif

/* Event generation register */

#define ATIM_EGR_UG               (1 << 0)  /* Bit 0: Update Generation */
#define ATIM_EGR_CC1G             (1 << 1)  /* Bit 1: Capture/Compare 1 Generation */
#define ATIM_EGR_CC2G             (1 << 2)  /* Bit 2: Capture/Compare 2 Generation */
#define ATIM_EGR_CC3G             (1 << 3)  /* Bit 3: Capture/Compare 3 Generation */
#define ATIM_EGR_CC4G             (1 << 4)  /* Bit 4: Capture/Compare 4 Generation */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#  define ATIM_EGR_COMG           (1 << 5)  /* Bit 5: Capture/Compare Control Update Generation */
#endif

#define ATIM_EGR_TG               (1 << 6)  /* Bit 6: Trigger Generation */

#if defined(CONFIG_STM32_STM32F10XX)
#  define ATIM_EGR_BG             (1 << 7)  /* Bit 7: Break Generation */
#elif defined(CONFIG_STM32_STM32F30XX)
#  define ATIM_EGR_BG             (1 << 7)  /* Bit 7: Break Generation */
#  define ATIM_EGR_B2G            (1 << 8)  /* Bit 8: Break 2 Generation */
#endif

/* Capture/compare mode register 1 -- Output compare mode */

#define ATIM_CCMR1_CC1S_SHIFT     (0)       /* Bits 1-0: Capture/Compare 1 Selection */
#define ATIM_CCMR1_CC1S_MASK      (3 << ATIM_CCMR1_CC1S_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC1FE          (1 << 2)  /* Bit 2: Output Compare 1 Fast enable */
#define ATIM_CCMR1_OC1PE          (1 << 3)  /* Bit 3: Output Compare 1 Preload enable */
#define ATIM_CCMR1_OC1M_SHIFT     (4)       /* Bits 6-4: Output Compare 1 Mode */
#define ATIM_CCMR1_OC1M_MASK      (7 << ATIM_CCMR1_OC1M_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC1CE          (1 << 7)  /* Bit 7: Output Compare 1Clear Enable */
#define ATIM_CCMR1_CC2S_SHIFT     (8)       /* Bits 8-9: Capture/Compare 2 Selection */
#define ATIM_CCMR1_CC2S_MASK      (3 << ATIM_CCMR1_CC2S_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC2FE          (1 << 10) /* Bit 10: Output Compare 2 Fast enable */
#define ATIM_CCMR1_OC2PE          (1 << 11) /* Bit 11: Output Compare 2 Preload enable */
#define ATIM_CCMR1_OC2M_SHIFT     (12)      /* Bits 14-12: Output Compare 2 Mode */
#define ATIM_CCMR1_OC2M_MASK      (7 << ATIM_CCMR1_OC2M_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC2CE          (1 << 15) /* Bit 15: Output Compare 2 Clear Enable */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_CCMR1_OC1M          (1 << 16) /* Bit 16: Output Compare 1 mode - bit 3 */
#  define ATIM_CCMR1_OC2M          (1 << 24) /* Bit 24: Output Compare 2 mode - bit 3 */
#endif

/* Common CCMR (unshifted) Capture/Compare Selection bit-field definitions */

#define ATIM_CCMR_CCS_CCOUT       (0)       /* 00: CCx channel  output */
#define ATIM_CCMR_CCS_CCIN1       (1)       /* 01: CCx channel input, ICx is TIx */
#define ATIM_CCMR_CCS_CCIN2       (2)       /* 10: CCx channel input, ICx is TIy */
#define ATIM_CCMR_CCS_CCINTRC     (3)       /* 11: CCx channel input, ICx is TRC */

/* Common CCMR (unshifted) Compare Mode bit field definitions */

#define ATIM_CCMR_MODE_FRZN       (0)       /* 000: Frozen */
#define ATIM_CCMR_MODE_CHACT      (1)       /* 001: Channel x active on match */
#define ATIM_CCMR_MODE_CHINACT    (2)       /* 010: Channel x inactive on match */
#define ATIM_CCMR_MODE_OCREFTOG   (3)       /* 011: OCxREF toggle ATIM_CNT=ATIM_CCRx */
#define ATIM_CCMR_MODE_OCREFLO    (4)       /* 100: OCxREF forced low */
#define ATIM_CCMR_MODE_OCREFHI    (5)       /* 101: OCxREF forced high */
#define ATIM_CCMR_MODE_PWM1       (6)       /* 110: PWM mode 1 */
#define ATIM_CCMR_MODE_PWM2       (7)       /* 111: PWM mode 2 */

/* Capture/compare mode register 1 -- Input capture mode */

                                            /* Bits 1-0:(same as output compare mode) */
#define ATIM_CCMR1_IC1PSC_SHIFT   (2)       /* Bits 3-2: Input Capture 1 Prescaler */
#define ATIM_CCMR1_IC1PSC_MASK    (3 << ATIM_CCMR1_IC1PSC_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_IC1F_SHIFT     (4)       /* Bits 7-4: Input Capture 1 Filter */
#define ATIM_CCMR1_IC1F_MASK      (0x0f << ATIM_CCMR1_IC1F_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
                                            /* Bits 9:8 (same as output compare mode) */
#define ATIM_CCMR1_IC2PSC_SHIFT   (10)      /* Bits 11:10: Input Capture 2 Prescaler */
#define ATIM_CCMR1_IC2PSC_MASK    (3 << ATIM_CCMR1_IC2PSC_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_IC2F_SHIFT     (12)      /* Bits 15-12: Input Capture 2 Filter */
#define ATIM_CCMR1_IC2F_MASK      (0x0f << ATIM_CCMR1_IC2F_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */

/* Common CCMR (unshifted) Input Capture Prescaler bit-field definitions */

#define ATIM_CCMR_ICPSC_NOPSC     (0)       /* 00: no prescaler, capture each edge */
#define ATIM_CCMR_ICPSC_EVENTS2   (1)       /* 01: capture once every 2 events */
#define ATIM_CCMR_ICPSC_EVENTS4   (2)       /* 10: capture once every 4 events */
#define ATIM_CCMR_ICPSC_EVENTS8   (3)       /* 11: capture once every 8 events */

/* Common CCMR (unshifted) Input Capture Filter bit-field definitions */

#define ATIM_CCMR_ICF_NOFILT      (0)       /* 0000: No filter, sampling at fDTS */
#define ATIM_CCMR_ICF_FCKINT2     (1)       /* 0001: fSAMPLING=fCK_INT, N=2 */
#define ATIM_CCMR_ICF_FCKINT4     (2)       /* 0010: fSAMPLING=fCK_INT, N=4 */
#define ATIM_CCMR_ICF_FCKINT8     (3)       /* 0011: fSAMPLING=fCK_INT, N=8 */
#define ATIM_CCMR_ICF_FDTSd26     (4)       /* 0100: fSAMPLING=fDTS/2, N=6 */
#define ATIM_CCMR_ICF_FDTSd28     (5)       /* 0101: fSAMPLING=fDTS/2, N=8 */
#define ATIM_CCMR_ICF_FDTSd46     (6)       /* 0110: fSAMPLING=fDTS/4, N=6 */
#define ATIM_CCMR_ICF_FDTSd48     (7)       /* 0111: fSAMPLING=fDTS/4, N=8 */
#define ATIM_CCMR_ICF_FDTSd86     (8)       /* 1000: fSAMPLING=fDTS/8, N=6 */
#define ATIM_CCMR_ICF_FDTSd88     (9)       /* 1001: fSAMPLING=fDTS/8, N=8 */
#define ATIM_CCMR_ICF_FDTSd165    (10)      /* 1010: fSAMPLING=fDTS/16, N=5 */
#define ATIM_CCMR_ICF_FDTSd166    (11)      /* 1011: fSAMPLING=fDTS/16, N=6 */
#define ATIM_CCMR_ICF_FDTSd168    (12)      /* 1100: fSAMPLING=fDTS/16, N=8 */
#define ATIM_CCMR_ICF_FDTSd325    (13)      /* 1101: fSAMPLING=fDTS/32, N=5 */
#define ATIM_CCMR_ICF_FDTSd326    (14)      /* 1110: fSAMPLING=fDTS/32, N=6 */
#define ATIM_CCMR_ICF_FDTSd328    (15)      /* 1111: fSAMPLING=fDTS/32, N=8 */

/* Capture/compare mode register 2 - Output Compare mode */

#define ATIM_CCMR2_CC3S_SHIFT     (0)       /* Bits 1-0: Capture/Compare 3 Selection */
#define ATIM_CCMR2_CC3S_MASK      (3 << ATIM_CCMR2_CC3S_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC3FE          (1 << 2)  /* Bit 2: Output Compare 3 Fast enable */
#define ATIM_CCMR2_OC3PE          (1 << 3)  /* Bit 3: Output Compare 3 Preload enable */
#define ATIM_CCMR2_OC3M_SHIFT     (4)       /* Bits 6-4: Output Compare 3 Mode */
#define ATIM_CCMR2_OC3M_MASK      (7 << ATIM_CCMR2_OC3M_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC3CE          (1 << 7)  /* Bit 7: Output Compare 3 Clear Enable */
#define ATIM_CCMR2_CC4S_SHIFT     (8)       /* Bits 9-8: Capture/Compare 4 Selection */
#define ATIM_CCMR2_CC4S_MASK      (3 << ATIM_CCMR2_CC4S_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC4FE          (1 << 10) /* Bit 10: Output Compare 4 Fast enable */
#define ATIM_CCMR2_OC4PE          (1 << 11) /* Bit 11: Output Compare 4 Preload enable */
#define ATIM_CCMR2_OC4M_SHIFT     (12)      /* Bits 14-12: Output Compare 4 Mode */
#define ATIM_CCMR2_OC4M_MASK      (7 << ATIM_CCMR2_OC4M_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC4CE          (1 << 15) /* Bit 15: Output Compare 4 Clear Enable */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_CCMR1_OC3M          (1 << 16) /* Bit 16: Output Compare 3 mode - bit 3 */
#  define ATIM_CCMR1_OC4M          (1 << 24) /* Bit 24: Output Compare 4 mode - bit 3 */
#endif

/* Capture/compare mode register 2 - Input Capture Mode */

                                            /* Bits 1-0:(same as output compare mode) */
#define ATIM_CCMR2_IC3PSC_SHIFT   (2)       /* Bits 3-2: Input Capture 3 Prescaler */
#define ATIM_CCMR1_IC3PSC_MASK    (3 << ATIM_CCMR2_IC3PSC_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_IC3F_SHIFT     (4)       /* Bits 7-4: Input Capture 3 Filter */
#define ATIM_CCMR2_IC3F_MASK      (0x0f << ATIM_CCMR2_IC3F_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
                                            /* Bits 9:8 (same as output compare mode) */
#define ATIM_CCMR2_IC4PSC_SHIFT   (10)      /* Bits 11:10: Input Capture 4 Prescaler */
#define ATIM_CCMR2_IC4PSC_MASK    (3 << ATIM_CCMR2_IC4PSC_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_IC4F_SHIFT     (12)      /* Bits 15-12: Input Capture 4 Filter */
#define ATIM_CCMR2_IC4F_MASK      (0x0f << ATIM_CCMR2_IC4F_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */

/* Capture/compare mode register 3 -- Output compare mode */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_CCMR3_OC5FE          (1 << 2)  /* Bit 2: Output Compare 5 Fast enable */
#  define ATIM_CCMR3_OC5PE          (1 << 3)  /* Bit 3: Output Compare 5 Preload enable */
#  define ATIM_CCMR3_OC5M_SHIFT     (4)       /* Bits 6-4: Output Compare 5 Mode */
#  define ATIM_CCMR3_OC5M_MASK      (7 << ATIM_CCMR3_OC5M_SHIFT)
                                              /* (See common (unshifted) bit field definitions below) */
#  define ATIM_CCMR3_OC5CE          (1 << 7)  /* Bit 7: Output Compare 5 Clear Enable */
#  define ATIM_CCMR3_OC6FE          (1 << 10) /* Bit 10: Output Compare 6 Fast enable */
#  define ATIM_CCMR3_OC6PE          (1 << 11) /* Bit 11: Output Compare 6 Preload enable */
#  define ATIM_CCMR3_OC6M_SHIFT     (12)      /* Bits 14-12: Output Compare 7 Mode */
#  define ATIM_CCMR3_OC6M_MASK      (7 << ATIM_CCMR3_OC6M_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#  define ATIM_CCMR3_OC6CE          (1 << 15) /* Bit 15: Output Compare 7 Clear Enable */

#  define ATIM_CCMR3_OC5M          (1 << 16) /* Bit 16: Output Compare 5 mode - bit 3 */
#  define ATIM_CCMR3_OC6M          (1 << 24) /* Bit 24: Output Compare 6 mode - bit 3 */
#endif

/* Capture/compare enable register */

#define ATIM_CCER_CC1E            (1 << 0)  /* Bit 0: Capture/Compare 1 output enable */
#define ATIM_CCER_CC1P            (1 << 1)  /* Bit 1: Capture/Compare 1 output Polarity */
#define ATIM_CCER_CC1NE           (1 << 2)  /* Bit 2: Capture/Compare 1 Complementary output enable */
#define ATIM_CCER_CC1NP           (1 << 3)  /* Bit 3: Capture/Compare 1 Complementary output polarity */
#define ATIM_CCER_CC2E            (1 << 4)  /* Bit 4: Capture/Compare 2 output enable */
#define ATIM_CCER_CC2P            (1 << 5)  /* Bit 5: Capture/Compare 2 output Polarity */
#define ATIM_CCER_CC2NE           (1 << 6)  /* Bit 6: Capture/Compare 2 Complementary output enable */
#define ATIM_CCER_CC2NP           (1 << 7)  /* Bit 7: Capture/Compare 2 Complementary output polarity */
#define ATIM_CCER_CC3E            (1 << 8)  /* Bit 8: Capture/Compare 3 output enable */
#define ATIM_CCER_CC3P            (1 << 9)  /* Bit 9: Capture/Compare 3 output Polarity */
#define ATIM_CCER_CC3NE           (1 << 10) /* Bit 10: Capture/Compare 3 Complementary output enable */
#define ATIM_CCER_CC3NP           (1 << 11) /* Bit 11: Capture/Compare 3 Complementary output polarity */
#define ATIM_CCER_CC4E            (1 << 12) /* Bit 12: Capture/Compare 4 output enable */
#define ATIM_CCER_CC4P            (1 << 13) /* Bit 13: Capture/Compare 4 output Polarity */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define ATIM_CCER_CC4NP         (1 << 15) /* Bit 15: Capture/Compare 4 Complementary output polarity */
#elif defined(CONFIG_STM32_STM32F30XX)
#  define ATIM_CCER_CC4NP         (1 << 15) /* Bit 15: Capture/Compare 4 Complementary output polarity */
#  define ATIM_CCER_CC5E          (1 << 16) /* Bit 16: Capture/Compare 5 output enable */
#  define ATIM_CCER_CC5P          (1 << 17) /* Bit 17: Capture/Compare 5 output Polarity */
#  define ATIM_CCER_CC6E          (1 << 20) /* Bit 20: Capture/Compare 6 output enable */
#  define ATIM_CCER_CC7P          (1 << 21) /* Bit 21: Capture/Compare 6 output Polarity */
#endif

/* 16-bit counter register */

#define ATIM_CNT_SHIFT            (0)       /* Bits 0-15: Timer counter value */
#define ATIM_CNT_MASK             (0xffff << ATIM_CNT_SHIFT)

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_CCER_UIFCPY        (1 << 31) /* Bit 31: UIF copy */
#endif

/* Repetition counter register */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_RCR_REP_SHIFT      (0)       /* Bits 0-15: Repetition Counter Value */
#  define ATIM_RCR_REP_MASK       (0xffff << ATIM_RCR_REP_SHIFT)

#  define ATIM_RCR_REP_MAX         32768    /* REVISIT */
#else
#  define ATIM_RCR_REP_SHIFT      (0)       /* Bits 0-7: Repetition Counter Value */
#  define ATIM_RCR_REP_MASK       (0xff << ATIM_RCR_REP_SHIFT)

#  define ATIM_RCR_REP_MAX        128
#endif

/* Capture/compare registers (CCR) */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_CCR5_GC5C1         (1 << 29)  /* Bit 29: Group Channel 5 and Channel 1 */
#  define ATIM_CCR5_GC5C2         (1 << 30)  /* Bit 30: Group Channel 5 and Channel 2 */
#  define ATIM_CCR5_GC5C3         (1 << 31)  /* Bit 31: Group Channel 5 and Channel 3 */
#endif

#define ATIM_CCR_MASK             (0xffff)

/* Break and dead-time register */

#define ATIM_BDTR_DTG_SHIFT       (0)       /* Bits 7:0 [7:0]: Dead-Time Generator set-up */
#define ATIM_BDTR_DTG_MASK        (0xff << ATIM_BDTR_DTG_SHIFT)
#define ATIM_BDTR_LOCK_SHIFT      (8)       /* Bits 9:8 [1:0]: Lock Configuration */
#define ATIM_BDTR_LOCK_MASK       (3 << ATIM_BDTR_LOCK_SHIFT)
#  define ATIM_BDTR_LOCKOFF       (0 << ATIM_BDTR_LOCK_SHIFT) /* 00: LOCK OFF - No bit is write protected */
#  define ATIM_BDTR_LOCK1         (1 << ATIM_BDTR_LOCK_SHIFT) /* 01: LOCK Level 1 protection */
#  define ATIM_BDTR_LOCK2         (2 << ATIM_BDTR_LOCK_SHIFT) /* 10: LOCK Level 2 protection */
#  define ATIM_BDTR_LOCK3         (3 << ATIM_BDTR_LOCK_SHIFT) /* 11: LOCK Level 3 protection */ */
#define ATIM_BDTR_OSSI            (1 << 10) /* Bit 10: Off-State Selection for Idle mode */
#define ATIM_BDTR_OSSR            (1 << 11) /* Bit 11: Off-State Selection for Run mode */
#define ATIM_BDTR_BKE             (1 << 12) /* Bit 12: Break enable */
#define ATIM_BDTR_BKP             (1 << 13) /* Bit 13: Break Polarity */
#define ATIM_BDTR_AOE             (1 << 14) /* Bit 14: Automatic Output enable */
#define ATIM_BDTR_MOE             (1 << 15) /* Bit 15: Main Output enable */

#ifdef CONFIG_STM32_STM32F30XX
#  define ATIM_BDTR_BKF_SHIFT     (16)      /* Bits 16-19: Break filter */
#  define ATIM_BDTR_BKF_MASK      (15 << ATIM_BDTR_BKF_SHIFT)
#    define ATIM_BDTR_BKF_NOFILT    (0 << ATIM_BDTR_BKF_SHIFT)   /* 0000: No filter, BRK acts asynchronously */
#    define ATIM_BDTR_BKF_FCKINT2   (1 << ATIM_BDTR_BKF_SHIFT)   /* 0001: fSAMPLING=fCK_INT, N=2 */
#    define ATIM_BDTR_BKF_FCKINT4   (2 << ATIM_BDTR_BKF_SHIFT)   /* 0010: fSAMPLING=fCK_INT, N=4 */
#    define ATIM_BDTR_BKF_FCKINT8   (3 << ATIM_BDTR_BKF_SHIFT)   /* 0011: fSAMPLING=fCK_INT, N=8 */
#    define ATIM_BDTR_BKF_FDTSd26   (4 << ATIM_BDTR_BKF_SHIFT)   /* 0100: fSAMPLING=fDTS/2, N=6 */
#    define ATIM_BDTR_BKF_FDTSd28   (5 << ATIM_BDTR_BKF_SHIFT)   /* 0101: fSAMPLING=fDTS/2, N=8 */
#    define ATIM_BDTR_BKF_FDTSd36   (6 << ATIM_BDTR_BKF_SHIFT)   /* 0110: fSAMPLING=fDTS/4, N=6 */
#    define ATIM_BDTR_BKF_FDTSd38   (7 << ATIM_BDTR_BKF_SHIFT)   /* 0111: fSAMPLING=fDTS/4, N=8 */
#    define ATIM_BDTR_BKF_FDTSd86   (8 << ATIM_BDTR_BKF_SHIFT)   /* 1000: fSAMPLING=fDTS/8, N=6 */
#    define ATIM_BDTR_BKF_FDTSd88   (9 << ATIM_BDTR_BKF_SHIFT)   /* 1001: fSAMPLING=fDTS/8, N=8 */
#    define ATIM_BDTR_BKF_FDTSd165  (10 << ATIM_BDTR_BKF_SHIFT)  /* 1010: fSAMPLING=fDTS/16, N=5 */
#    define ATIM_BDTR_BKF_FDTSd166  (11 << ATIM_BDTR_BKF_SHIFT)  /* 1011: fSAMPLING=fDTS/16, N=6 */
#    define ATIM_BDTR_BKF_FDTSd168  (12 << ATIM_BDTR_BKF_SHIFT)  /* 1100: fSAMPLING=fDTS/16, N=8 */
#    define ATIM_BDTR_BKF_FDTSd325  (13 << ATIM_BDTR_BKF_SHIFT)  /* 1101: fSAMPLING=fDTS/32, N=5 */
#    define ATIM_BDTR_BKF_FDTSd326  (14 << ATIM_BDTR_BKF_SHIFT)  /* 1110: fSAMPLING=fDTS/32, N=6 */
#    define ATIM_BDTR_BKF_FDTSd328  (15 << ATIM_BDTR_BKF_SHIFT)  /* 1111: fSAMPLING=fDTS/32, N=8 */
#  define ATIM_BDTR_BK2F_SHIFT    (20)      /* Bits 20-23: Break 2 filter */
#  define ATIM_BDTR_BK2F_MASK     (15 << ATIM_BDTR_BK2F_SHIFT)
#    define ATIM_BDTR_BK2F_NOFILT   (0 << ATIM_BDTR_BK2F_SHIFT)  /* 0000: No filter, BRK 2 acts asynchronously */
#    define ATIM_BDTR_BK2F_FCKINT2  (1 << ATIM_BDTR_BK2F_SHIFT)  /* 0001: fSAMPLING=fCK_INT, N=2 */
#    define ATIM_BDTR_BK2F_FCKINT4  (2 << ATIM_BDTR_BK2F_SHIFT)  /* 0010: fSAMPLING=fCK_INT, N=4 */
#    define ATIM_BDTR_BK2F_FCKINT8  (3 << ATIM_BDTR_BK2F_SHIFT)  /* 0011: fSAMPLING=fCK_INT, N=8 */
#    define ATIM_BDTR_BK2F_FDTSd26  (4 << ATIM_BDTR_BK2F_SHIFT)  /* 0100: fSAMPLING=fDTS/2, N=6 */
#    define ATIM_BDTR_BK2F_FDTSd28  (5 << ATIM_BDTR_BK2F_SHIFT)  /* 0101: fSAMPLING=fDTS/2, N=8 */
#    define ATIM_BDTR_BK2F_FDTSd36  (6 << ATIM_BDTR_BK2F_SHIFT)  /* 0110: fSAMPLING=fDTS/4, N=6 */
#    define ATIM_BDTR_BK2F_FDTSd38  (7 << ATIM_BDTR_BK2F_SHIFT)  /* 0111: fSAMPLING=fDTS/4, N=8 */
#    define ATIM_BDTR_BK2F_FDTSd86  (8 << ATIM_BDTR_BK2F_SHIFT)  /* 1000: fSAMPLING=fDTS/8, N=6 */
#    define ATIM_BDTR_BK2F_FDTSd88  (9 << ATIM_BDTR_BK2F_SHIFT)  /* 1001: fSAMPLING=fDTS/8, N=8 */
#    define ATIM_BDTR_BK2F_FDTSd165 (10 << ATIM_BDTR_BK2F_SHIFT) /* 1010: fSAMPLING=fDTS/16, N=5 */
#    define ATIM_BDTR_BK2F_FDTSd166 (11 << ATIM_BDTR_BK2F_SHIFT) /* 1011: fSAMPLING=fDTS/16, N=6 */
#    define ATIM_BDTR_BK2F_FDTSd168 (12 << ATIM_BDTR_BK2F_SHIFT) /* 1100: fSAMPLING=fDTS/16, N=8 */
#    define ATIM_BDTR_BK2F_FDTSd325 (13 << ATIM_BDTR_BK2F_SHIFT) /* 1101: fSAMPLING=fDTS/32, N=5 */
#    define ATIM_BDTR_BK2F_FDTSd326 (14 << ATIM_BDTR_BK2F_SHIFT) /* 1110: fSAMPLING=fDTS/32, N=6 */
#    define ATIM_BDTR_BK2F_FDTSd328 (15 << ATIM_BDTR_BK2F_SHIFT) /* 1111: fSAMPLING=fDTS/32, N=8 */
#  define ATIM_BDTR_BK2E          (1 << 24) /* Bit 24: Break 2 enable */
#  define ATIM_BDTR_BK2P          (1 << 1525 /* Bit 25:Break 2 polarity */
#endif

/* DMA control register */

#define ATIM_DCR_DBA_SHIFT        (0)       /* Bits 4-0: DMA Base Address */
#define ATIM_DCR_DBA_MASK         (0x1f << ATIM_DCR_DBA_SHIFT)
#define ATIM_DCR_DBL_SHIFT        (8)       /* Bits 12-8: DMA Burst Length */
#define ATIM_DCR_DBL_MASK         (0x1f << ATIM_DCR_DBL_SHIFT)
#  define ATIM_DCR_DBL(n)         (((n)-1) << ATIM_DCR_DBL_SHIFT) /* n transfers, n = 1..18 */

/* Control register 1 (TIM2-5 and TIM9-14) */

#define GTIM_CR1_CEN              (1 << 0)  /* Bit 0: Counter enable */
#define GTIM_CR1_UDIS             (1 << 1)  /* Bit 1: Update Disable */
#define GTIM_CR1_URS              (1 << 2)  /* Bit 2: Update Request Source */
#define GTIM_CR1_OPM              (1 << 3)  /* Bit 3: One Pulse Mode (TIM2-5, 9, and 12 only) */
#define GTIM_CR1_DIR              (1 << 4)  /* Bit 4: Direction (TIM2-5 only) */
#define GTIM_CR1_CMS_SHIFT        (5)       /* Bits 6-5: Center-aligned Mode Selection (TIM2-5 only) */
#define GTIM_CR1_CMS_MASK         (3 << GTIM_CR1_CMS_SHIFT)
#  define GTIM_CR1_EDGE           (0 << GTIM_CR1_CMS_SHIFT) /* 00: Edge-aligned mode.  */
#  define GTIM_CR1_CENTER1        (1 << GTIM_CR1_CMS_SHIFT) /* 01: Center-aligned mode 1 */
#  define GTIM_CR1_CENTER2        (2 << GTIM_CR1_CMS_SHIFT) /* 10: Center-aligned mode 2 */
#  define GTIM_CR1_CENTER3        (3 << GTIM_CR1_CMS_SHIFT) /* 11: Center-aligned mode 3 */
#define GTIM_CR1_ARPE             (1 << 7)  /* Bit 7: Auto-Reload Preload enable */
#define GTIM_CR1_CKD_SHIFT        (8)       /* Bits 9-8: Clock Division */
#define GTIM_CR1_CKD_MASK         (3 << GTIM_CR1_CKD_SHIFT)
#  define GTIM_CR1_TCKINT         (0 << GTIM_CR1_CKD_SHIFT) /* 00: tDTS = tCK_INT */
#  define GTIM_CR1_2TCKINT        (1 << GTIM_CR1_CKD_SHIFT) /* 01: tDTS = 2 x tCK_INT */
#  define GTIM_CR1_4TCKINT        (2 << GTIM_CR1_CKD_SHIFT) /* 10: tDTS = 4 x tCK_INT */

#ifdef CONFIG_STM32_STM32F30XX
#  define GTIM_CR1_UIFREMAP       (1 << 11) /* Bit 11: UIF status bit remapping */
#endif

/* Control register 2 (TIM2-5, TIM9-12, and TIM15-17 only) */

#define GTIM_CR2_CCPC             (1 << 0)  /* Bit 0: Capture/compare preloaded control (TIM15-17 only) */
#define GTIM_CR2_CCUS             (1 << 2)  /* Bit 2: Capture/compare control update selection (TIM15-17 only) */
#define GTIM_CR2_CCDS             (1 << 3)  /* Bit 3: Capture/Compare DMA Selection (TIM2-5,1,&16 only) */
#define GTIM_CR2_MMS_SHIFT        (4)       /* Bits 6-4: Master Mode Selection (not TIM16) */
#define GTIM_CR2_MMS_MASK         (7 << GTIM_CR2_MMS_SHIFT)
#  define GTIM_CR2_RESET          (0 << GTIM_CR2_MMS_SHIFT) /* 000: Reset */
#  define GTIM_CR2_ENAB           (1 << GTIM_CR2_MMS_SHIFT) /* 001: Enable */
#  define GTIM_CR2_UPDT           (2 << GTIM_CR2_MMS_SHIFT) /* 010: Update */
#  define GTIM_CR2_CMPP           (3 << GTIM_CR2_MMS_SHIFT) /* 011: Compare Pulse */
#  define GTIM_CR2_CMP1           (4 << GTIM_CR2_MMS_SHIFT) /* 100: Compare - OC1REF signal is used as trigger output (TRGO) */
#  define GTIM_CR2_CMP2           (5 << GTIM_CR2_MMS_SHIFT) /* 101: Compare - OC2REF signal is used as trigger output (TRGO) */
#  define GTIM_CR2_CMP3           (6 << GTIM_CR2_MMS_SHIFT) /* 110: Compare - OC3REF signal is used as trigger output (TRGO, TIM2-5 and TIM15 only) */
#  define GTIM_CR2_CMP4           (7 << GTIM_CR2_MMS_SHIFT) /* 111: Compare - OC4REF signal is used as trigger output (TRGO, TIM2-5 and TIM15 only) */
#define GTIM_CR2_TI1S             (1 << 7)  /* Bit 7: TI1 Selection (not TIM16) */
#define GTIM_CR2_OIS1             (1 << 8)  /* Bit 8: COutput Idle state 1 (OC1 output) (TIM15-17 only) */
#define GTIM_CR2_OIS1N            (1 << 9)  /* Bit 9: Output Idle state 1 (OC1N output) (TIM15-17 only) */
#define GTIM_CR2_OIS2             (1 << 10) /* Bit 10: Output idle state 2 (OC2 output) (TIM15 only) */

/* Slave mode control register (TIM2-5 and TIM15 only) */

#define GTIM_SMCR_SMS_SHIFT       (0)       /* Bits 2-0: Slave Mode Selection */
#define GTIM_SMCR_SMS_MASK        (7 << GTIM_SMCR_SMS_SHIFT)
#  define GTIM_SMCR_DISAB         (0 << GTIM_SMCR_SMS_SHIFT) /* 000: Slave mode disabled */
#  define GTIM_SMCR_ENCMD1        (1 << GTIM_SMCR_SMS_SHIFT) /* 001: Encoder mode 1 */
#  define GTIM_SMCR_ENCMD2        (2 << GTIM_SMCR_SMS_SHIFT) /* 010: Encoder mode 2 */
#  define GTIM_SMCR_ENCMD3        (3 << GTIM_SMCR_SMS_SHIFT) /* 011: Encoder mode 3 */
#  define GTIM_SMCR_RESET         (4 << GTIM_SMCR_SMS_SHIFT) /* 100: Reset Mode  */
#  define GTIM_SMCR_GATED         (5 << GTIM_SMCR_SMS_SHIFT) /* 101: Gated Mode  */
#  define GTIM_SMCR_TRIGGER       (6 << GTIM_SMCR_SMS_SHIFT) /* 110: Trigger Mode */
#  define GTIM_SMCR_EXTCLK1       (7 << GTIM_SMCR_SMS_SHIFT) /* 111: External Clock Mode 1 */
#define GTIM_SMCR_TS_SHIFT        (4)       /* Bits 6-4: Trigger Selection */
#define GTIM_SMCR_TS_MASK         (7 << GTIM_SMCR_TS_SHIFT)
#  define GTIM_SMCR_ITR0          (0 << GTIM_SMCR_TS_SHIFT) /* 000: Internal Trigger 0 (ITR0). TIM1 */
#  define GTIM_SMCR_ITR1          (1 << GTIM_SMCR_TS_SHIFT) /* 001: Internal Trigger 1 (ITR1). TIM2 */
#  define GTIM_SMCR_ITR2          (2 << GTIM_SMCR_TS_SHIFT) /* 010: Internal Trigger 2 (ITR2). TIM3 */
#  define GTIM_SMCR_ITR3          (3 << GTIM_SMCR_TS_SHIFT) /* 011: Internal Trigger 3 (ITR3). TIM4 */
#  define GTIM_SMCR_TI1FED        (4 << GTIM_SMCR_TS_SHIFT) /* 100: TI1 Edge Detector (TI1F_ED) */
#  define GTIM_SMCR_TI1FP1        (5 << GTIM_SMCR_TS_SHIFT) /* 101: Filtered Timer Input 1 (TI1FP1) */
#  define GTIM_SMCR_TI2FP2        (6 << GTIM_SMCR_TS_SHIFT) /* 110: Filtered Timer Input 2 (TI2FP2) */
#  define GTIM_SMCR_ETRF          (7 << GTIM_SMCR_TS_SHIFT) /* 111: External Trigger input (ETRF) */
#define GTIM_SMCR_MSM             (1 << 7)  /* Bit 7: Master/Slave mode */
#define GTIM_SMCR_ETF_SHIFT       (8)       /* Bits 11-8: External Trigger Filter (not TIM15) */
#define GTIM_SMCR_ETF_MASK        (0x0f << GTIM_SMCR_ETF_SHIFT)
#  define GTIM_SMCR_NOFILT        (0 << GTIM_SMCR_ETF_SHIFT)  /* 0000: No filter, sampling is done at fDTS */
#  define GTIM_SMCR_FCKINT2       (1 << GTIM_SMCR_ETF_SHIFT)  /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define GTIM_SMCR_FCKINT4       (2 << GTIM_SMCR_ETF_SHIFT)  /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define GTIM_SMCR_FCKINT8       (3 << GTIM_SMCR_ETF_SHIFT)  /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define GTIM_SMCR_FDTSd26       (4 << GTIM_SMCR_ETF_SHIFT)  /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define GTIM_SMCR_FDTSd28       (5 << GTIM_SMCR_ETF_SHIFT)  /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define GTIM_SMCR_FDTSd36       (6 << GTIM_SMCR_ETF_SHIFT)  /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define GTIM_SMCR_FDTSd38       (7 << GTIM_SMCR_ETF_SHIFT)  /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define GTIM_SMCR_FDTSd86       (8 << GTIM_SMCR_ETF_SHIFT)  /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define GTIM_SMCR_FDTSd88       (9 << GTIM_SMCR_ETF_SHIFT)  /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define GTIM_SMCR_FDTSd165      (10 << GTIM_SMCR_ETF_SHIFT) /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define GTIM_SMCR_FDTSd166      (11 << GTIM_SMCR_ETF_SHIFT) /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define GTIM_SMCR_FDTSd168      (12 << GTIM_SMCR_ETF_SHIFT) /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define GTIM_SMCR_FDTSd325      (13 << GTIM_SMCR_ETF_SHIFT) /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define GTIM_SMCR_FDTSd326      (14 << GTIM_SMCR_ETF_SHIFT) /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define GTIM_SMCR_FDTSd328      (15 << GTIM_SMCR_ETF_SHIFT) /* 1111: fSAMPLING=fDTS/32, N=8 */
#define GTIM_SMCR_ETPS_SHIFT      (12)      /* Bits 13-12: External Trigger Prescaler (not TIM15) */
#define GTIM_SMCR_ETPS_MASK       (3 << GTIM_SMCR_ETPS_SHIFT)
#  define GTIM_SMCR_PSCOFF        (0 << GTIM_SMCR_ETPS_SHIFT) /* 00: Prescaler OFF */
#  define GTIM_SMCR_ETRPd2        (1 << GTIM_SMCR_ETPS_SHIFT) /* 01: ETRP frequency divided by 2 */
#  define GTIM_SMCR_ETRPd4        (2 << GTIM_SMCR_ETPS_SHIFT) /* 10: ETRP frequency divided by 4 */
#  define GTIM_SMCR_ETRPd8        (3 << GTIM_SMCR_ETPS_SHIFT) /* 11: ETRP frequency divided by 8 */
#define GTIM_SMCR_ECE             (1 << 14) /* Bit 14: External Clock enable */
#define GTIM_SMCR_ETP             (1 << 15) /* Bit 15: External Trigger Polarity */

#ifdef CONFIG_STM32_STM32F30XX
#  define GTIM_CR1_SMS            (1 << 16) /* Bit 16: Slave mode selection - bit 3 */
#endif

/* DMA/Interrupt enable register (TIM2-5 and TIM9-14) */

#define GTIM_DIER_UIE             (1 << 0)  /* Bit 0: Update interrupt enable */
#define GTIM_DIER_CC1IE           (1 << 1)  /* Bit 1: Capture/Compare 1 interrupt enable */
#define GTIM_DIER_CC2IE           (1 << 2)  /* Bit 2: Capture/Compare 2 interrupt enable (TIM2-5,9,12,&15 only) */
#define GTIM_DIER_CC3IE           (1 << 3)  /* Bit 3: Capture/Compare 3 interrupt enable (TIM2-5 only) */
#define GTIM_DIER_CC4IE           (1 << 4)  /* Bit 4: Capture/Compare 4 interrupt enable (TIM2-5 only) */
#define GTIM_DIER_COMIE           (1 << 5)  /* Bit 5: COM interrupt enable (TIM15-17 only) */
#define GTIM_DIER_TIE             (1 << 6)  /* Bit 6: Trigger interrupt enable (TIM2-5,9,&12 only) */
#define GTIM_DIER_BIE             (1 << 7)  /* Bit 7: Break interrupt enable (TIM15-17 only) */
#define GTIM_DIER_UDE             (1 << 8)  /* Bit 8: Update DMA request enable (TIM2-5&15-17 only) */
#define GTIM_DIER_CC1DE           (1 << 9)  /* Bit 9: Capture/Compare 1 DMA request enable (TIM2-5&15-17 only) */
#define GTIM_DIER_CC2DE           (1 << 10) /* Bit 10: Capture/Compare 2 DMA request enable (TIM2-5&15 only) */
#define GTIM_DIER_CC3DE           (1 << 11) /* Bit 11: Capture/Compare 3 DMA request enable (TIM2-5 only) */
#define GTIM_DIER_CC4DE           (1 << 12) /* Bit 12: Capture/Compare 4 DMA request enable (TIM2-5 only) */
#define GTIM_DIER_COMDE           (1 << 13) /* Bit 13: COM DMA request enable (TIM15-17 only) */
#define GTIM_DIER_TDE             (1 << 14) /* Bit 14: Trigger DMA request enable (TIM2-5&15-17 only) */

/* Status register */

#define GTIM_SR_UIF               (1 << 0)  /* Bit 0: Update interrupt flag */
#define GTIM_SR_CC1IF             (1 << 1)  /* Bit 1: Capture/compare 1 interrupt flag */
#define GTIM_SR_CC2IF             (1 << 2)  /* Bit 2: Capture/Compare 2 interrupt flag (TIM2-5,9,12,&15 only) */
#define GTIM_SR_CC3IF             (1 << 3)  /* Bit 3: Capture/Compare 3 interrupt flag (TIM2-5 only) */
#define GTIM_SR_CC4IF             (1 << 4)  /* Bit 4: Capture/Compare 4 interrupt flag (TIM2-5 only) */
#define GTIM_SR_COMIF             (1 << 5)  /* Bit 5: COM interrupt flag (TIM15-17 only) */
#define GTIM_SR_TIF               (1 << 6)  /* Bit 6: Trigger interrupt Flag (TIM2-5,9,12&15-17 only) */
#define GTIM_SR_BIF               (1 << 7)  /* Bit 7: Break interrupt flag (TIM15-17 only) */
#define GTIM_SR_CC1OF             (1 << 9)  /* Bit 9: Capture/Compare 1 Overcapture flag */
#define GTIM_SR_CC2OF             (1 << 10) /* Bit 10: Capture/Compare 2 Overcapture flag (TIM2-5,9,12&15 only) */
#define GTIM_SR_CC3OF             (1 << 11) /* Bit 11: Capture/Compare 3 Overcapture flag (TIM2-5 only) */
#define GTIM_SR_CC4OF             (1 << 12) /* Bit 12: Capture/Compare 4 Overcapture flag (TIM2-5 only) */

/* Event generation register (TIM2-5 and TIM9-14) */

#define GTIM_EGR_UG               (1 << 0)  /* Bit 0: Update generation */
#define GTIM_EGR_CC1G             (1 << 1)  /* Bit 1: Capture/compare 1 generation */
#define GTIM_EGR_CC2G             (1 << 2)  /* Bit 2: Capture/compare 2 generation (TIM2-5,9,12,&15 only) */
#define GTIM_EGR_CC3G             (1 << 3)  /* Bit 3: Capture/compare 3 generation (TIM2-5 only) */
#define GTIM_EGR_CC4G             (1 << 4)  /* Bit 4: Capture/compare 4 generation (TIM2-5 only) */
#define GTIM_EGR_COMIG            (1 << 5)  /* Bit 5: Capture/Compare control update generation (TIM15-17 only) */
#define GTIM_EGR_TG               (1 << 6)  /* Bit 6: Trigger generation (TIM2-5,9,12&16-17 only) */
#define GTIM_EGR_BG               (1 << 7)  /* Bit 7: Break generation (TIM15-17 only) */

/* Capture/compare mode register 1 - Output compare mode (TIM2-5 and TIM9-14) */

#define GTIM_CCMR1_CC1S_SHIFT     (0)       /* Bits 1-0: Capture/Compare 1 Selection */
#define GTIM_CCMR1_CC1S_MASK      (3 << GTIM_CCMR1_CC1S_SHIFT)
                                            /* (See common CCMR Capture/Compare Selection definitions below) */
#define GTIM_CCMR1_OC1FE          (1 << 2)  /* Bit 2: Output Compare 1 Fast enable */
#define GTIM_CCMR1_OC1PE          (1 << 3)  /* Bit 3: Output Compare 1 Preload enable */
#define GTIM_CCMR1_OC1M_SHIFT     (4)       /* Bits 6-4: Output Compare 1 Mode */
#define GTIM_CCMR1_OC1M_MASK      (7 << GTIM_CCMR1_OC1M_SHIFT)
                                            /* (See common CCMR Output Compare Mode definitions below) */
#define GTIM_CCMR1_OC1CE          (1 << 7)  /* Bit 7: Output Compare 1Clear Enable */
#define GTIM_CCMR1_CC2S_SHIFT     (8)       /* Bits 9-8: Capture/Compare 2 Selection */
#define GTIM_CCMR1_CC2S_MASK      (3 << GTIM_CCMR1_CC2S_SHIFT)
                                            /* (See common CCMR Capture/Compare Selection definitions below) */
#define GTIM_CCMR1_OC2FE          (1 << 10) /* Bit 10: Output Compare 2 Fast enable */
#define GTIM_CCMR1_OC2PE          (1 << 11) /* Bit 11: Output Compare 2 Preload enable */
#define GTIM_CCMR1_OC2M_SHIFT     (12)      /* Bits 14-12: Output Compare 2 Mode */
#define GTIM_CCMR1_OC2M_MASK      (7 << GTIM_CCMR1_OC2M_SHIFT)
                                            /* (See common CCMR Output Compare Mode definitions below) */
#define GTIM_CCMR1_OC2CE          (1 << 15) /* Bit 15: Output Compare 2 Clear Enable */

#ifdef CONFIG_STM32_STM32F30XX
#  define GTIM_CCMR1_OC1M          (1 << 16) /* Bit 16: Output Compare 1 mode - bit 3 */
#  define GTIM_CCMR1_OC2M          (1 << 24) /* Bit 24: Output Compare 2 mode - bit 3 */
#endif

/* Common CCMR (unshifted) Capture/Compare Selection bit-field definitions */

#define GTIM_CCMR_CCS_CCOUT       (0)       /* 00: CCx channel output */
#define GTIM_CCMR_CCS_CCIN1       (1)       /* 01: CCx channel input, ICx is TIx */
#define GTIM_CCMR_CCS_CCIN2       (2)       /* 10: CCx channel input, ICx is TIy */
#define GTIM_CCMR_CCS_CCINTRC     (3)       /* 11: CCx channel input, ICx is TRC */

/* Common CCMR (unshifted) Compare Mode bit field definitions */

#define GTIM_CCMR_MODE_FRZN       (0)       /* 000: Frozen */
#define GTIM_CCMR_MODE_CHACT      (1)       /* 001: Channel x active on match */
#define GTIM_CCMR_MODE_CHINACT    (2)       /* 010: Channel x inactive on match */
#define GTIM_CCMR_MODE_OCREFTOG   (3)       /* 011: OCxREF toggle ATIM_CNT=ATIM_CCRx */
#define GTIM_CCMR_MODE_OCREFLO    (4)       /* 100: OCxREF forced low */
#define GTIM_CCMR_MODE_OCREFHI    (5)       /* 101: OCxREF forced high */
#define GTIM_CCMR_MODE_PWM1       (6)       /* 110: PWM mode 1 */
#define GTIM_CCMR_MODE_PWM2       (7)       /* 111: PWM mode 2 */

/* Capture/compare mode register 1 - Input capture mode (TIM2-5 and TIM9-14) */

                                            /* Bits 1-0 (Same as Output Compare Mode) */
#define GTIM_CCMR1_IC1PSC_SHIFT   (2)       /* Bits 3-2: Input Capture 1 Prescaler */
#define GTIM_CCMR1_IC1PSC_MASK    (3 << GTIM_CCMR1_IC1PSC_SHIFT)
                                            /* (See common CCMR Input Capture Prescaler definitions below) */
#define GTIM_CCMR1_IC1F_SHIFT     (4)       /* Bits 7-4: Input Capture 1 Filter */
#define GTIM_CCMR1_IC1F_MASK      (0x0f << GTIM_CCMR1_IC1F_SHIFT)
                                            /* (See common CCMR Input Capture Filter definitions below) */
                                            /* Bits 9-8: (Same as Output Compare Mode) */
#define GTIM_CCMR1_IC2PSC_SHIFT   (10)      /* Bits 11-10: Input Capture 2 Prescaler */
#define GTIM_CCMR1_IC2PSC_MASK    (3 << GTIM_CCMR1_IC2PSC_SHIFT)
                                            /* (See common CCMR Input Capture Prescaler definitions below) */
#define GTIM_CCMR1_IC2F_SHIFT     (12)      /* Bits 15-12: Input Capture 2 Filter */
#define GTIM_CCMR1_IC2F_MASK      (0x0f << GTIM_CCMR1_IC2F_SHIFT)
                                            /* (See common CCMR Input Capture Filter definitions below) */

/* Common CCMR (unshifted) Input Capture Prescaler bit-field definitions */

#define GTIM_CCMR_ICPSC_NOPSC     (0)       /* 00: no prescaler, capture each edge */
#define GTIM_CCMR_ICPSC_EVENTS2   (1)       /* 01: capture once every 2 events */
#define GTIM_CCMR_ICPSC_EVENTS4   (2)       /* 10: capture once every 4 events */
#define GTIM_CCMR_ICPSC_EVENTS8   (3)       /* 11: capture once every 8 events */

/* Common CCMR (unshifted) Input Capture Filter bit-field definitions */

#define GTIM_CCMR_ICF_NOFILT      (0)       /* 0000: No filter, sampling at fDTS */
#define GTIM_CCMR_ICF_FCKINT2     (1)       /* 0001: fSAMPLING=fCK_INT, N=2 */
#define GTIM_CCMR_ICF_FCKINT4     (2)       /* 0010: fSAMPLING=fCK_INT, N=4 */
#define GTIM_CCMR_ICF_FCKINT8     (3)       /* 0011: fSAMPLING=fCK_INT, N=8 */
#define GTIM_CCMR_ICF_FDTSd26     (4)       /* 0100: fSAMPLING=fDTS/2, N=6 */
#define GTIM_CCMR_ICF_FDTSd28     (5)       /* 0101: fSAMPLING=fDTS/2, N=8 */
#define GTIM_CCMR_ICF_FDTSd46     (6)       /* 0110: fSAMPLING=fDTS/4, N=6 */
#define GTIM_CCMR_ICF_FDTSd48     (7)       /* 0111: fSAMPLING=fDTS/4, N=8 */
#define GTIM_CCMR_ICF_FDTSd86     (8)       /* 1000: fSAMPLING=fDTS/8, N=6 */
#define GTIM_CCMR_ICF_FDTSd88     (9)       /* 1001: fSAMPLING=fDTS/8, N=8 */
#define GTIM_CCMR_ICF_FDTSd165    (10)      /* 1010: fSAMPLING=fDTS/16, N=5 */
#define GTIM_CCMR_ICF_FDTSd166    (11)      /* 1011: fSAMPLING=fDTS/16, N=6 */
#define GTIM_CCMR_ICF_FDTSd168    (12)      /* 1100: fSAMPLING=fDTS/16, N=8 */
#define GTIM_CCMR_ICF_FDTSd325    (13)      /* 1101: fSAMPLING=fDTS/32, N=5 */
#define GTIM_CCMR_ICF_FDTSd326    (14)      /* 1110: fSAMPLING=fDTS/32, N=6 */
#define GTIM_CCMR_ICF_FDTSd328    (15)      /* 1111: fSAMPLING=fDTS/32, N=8 */

/* Capture/compare mode register 2 - Output Compare mode (TIM2-5 only) */

#define GTIM_CCMR2_CC3S_SHIFT     (0)       /* Bits 1-0: Capture/Compare 3 Selection */
#define GTIM_CCMR2_CC3S_MASK      (3 << GTIM_CCMR2_CC3S_SHIFT)
                                            /* (See common CCMR Capture/Compare Selection definitions above) */
#define GTIM_CCMR2_OC3FE          (1 << 2)  /* Bit 2: Output Compare 3 Fast enable */
#define GTIM_CCMR2_OC3PE          (1 << 3)  /* Bit 3: Output Compare 3 Preload enable */
#define GTIM_CCMR2_OC3M_SHIFT     (4)       /* Bits 6-4: Output Compare 3 Mode */
#define GTIM_CCMR2_OC3M_MASK      (7 << GTIM_CCMR2_OC3M_SHIFT)
                                            /* (See common CCMR Output Compare Mode definitions above) */
#define GTIM_CCMR2_OC3CE          (1 << 7)  /* Bit 7: Output Compare 3 Clear Enable */
#define GTIM_CCMR2_CC4S_SHIFT     (8)       /* Bits 9-8: Capture/Compare 4 Selection */
#define GTIM_CCMR2_CC4S_MASK      (3 << GTIM_CCMR2_CC4S_SHIFT)
                                            /* (See common CCMR Capture/Compare Selection definitions above) */
#define GTIM_CCMR2_OC4FE          (1 << 10) /* Bit 10: Output Compare 4 Fast enable */
#define GTIM_CCMR2_OC4PE          (1 << 11) /* Bit 11: Output Compare 4 Preload enable */
#define GTIM_CCMR2_OC4M_SHIFT     (12)      /* Bits 14-12: Output Compare 4 Mode */
#define GTIM_CCMR2_OC4M_MASK      (7 << GTIM_CCMR2_OC4M_SHIFT)
                                            /* (See common CCMR Output Compare Mode definitions above) */
#define GTIM_CCMR2_OC4CE          (1 << 15) /* Bit 15: Output Compare 4 Clear Enable */

/* Capture/compare mode register 2 - Input capture mode (TIM2-5 only) */

                                            /* Bits 1-0 (Same as Output Compare Mode) */
#define GTIM_CCMR2_IC3PSC_SHIFT   (2)       /* Bits 3-2: Input Capture 3 Prescaler */
#define GTIM_CCMR2_IC3PSC_MASK    (3 << GTIM_CCMR2_IC3PSC_SHIFT)
                                            /* (See common CCMR Input Capture Prescaler definitions below) */
#define GTIM_CCMR2_IC3F_SHIFT     (4)       /* Bits 7-4: Input Capture 3 Filter */
#define GTIM_CCMR2_IC3F_MASK      (0x0f << GTIM_CCMR2_IC3F_SHIFT)
                                            /* (See common CCMR Input Capture Filter definitions below) */
                                            /* Bits 9-8: (Same as Output Compare Mode) */
#define GTIM_CCMR2_IC4PSC_SHIFT   (10)      /* Bits 11-10: Input Capture 4 Prescaler */
#define GTIM_CCMR2_IC4PSC_MASK    (3 << GTIM_CCMR2_IC4PSC_SHIFT)
                                            /* (See common CCMR Input Capture Prescaler definitions below) */
#define GTIM_CCMR2_IC4F_SHIFT     (12)      /* Bits 15-12: Input Capture 4 Filter */
#define GTIM_CCMR2_IC4F_MASK      (0x0f << GTIM_CCMR2_IC4F_SHIFT)
                                            /* (See common CCMR Input Capture Filter definitions below) */

/* Capture/compare enable register (TIM1 and TIM8, TIM2-5 and TIM9-14) */

#define GTIM_CCER_CC1E            (1 << 0)  /* Bit 0: Capture/Compare 1 output enable */
#define GTIM_CCER_CC1P            (1 << 1)  /* Bit 1: Capture/Compare 1 output polarity */
#define GTIM_CCER_CC1NE           (1 << 2)  /* Bit 2: Capture/Compare 1 complementary output enable (TIM1 and TIM8 only) */
#define GTIM_CCER_CC1NP           (1 << 3)  /* Bit 3: Capture/Compare 1 output Polarity (F2,F3,F4 and TIM15-17) */
#define GTIM_CCER_CC2E            (1 << 4)  /* Bit 4: Capture/Compare 2 output enable (TIM2-5,9&12 only) */
#define GTIM_CCER_CC2P            (1 << 5)  /* Bit 5: Capture/Compare 2 output polarity (TIM2-5,9&12 only) */
#define GTIM_CCER_CC2NE           (1 << 6)  /* Bit 6: Capture/Compare 2 complementary output enable (TIM1 and TIM8 only) */
#define GTIM_CCER_CC2NP           (1 << 7)  /* Bit 7: Capture/Compare 2 output Polarity (F2,F3,F4 and TIM2-5,9,12&15 only) */
#define GTIM_CCER_CC3E            (1 << 8)  /* Bit 8: Capture/Compare 3 output enable (TIM2-5 only) */
#define GTIM_CCER_CC3P            (1 << 9)  /* Bit 9: Capture/Compare 3 output Polarity (TIM2-5 only) */
#define GTIM_CCER_CC3NE           (1 << 10) /* Bit 10: Capture/Compare 3 complementary output enable (TIM1 and TIM8 only) */
#define GTIM_CCER_CC3NP           (1 << 11) /* Bit 11: Capture/Compare 3 output Polarity (F2,F4 and TIM2-5 only) */
#define GTIM_CCER_CC4E            (1 << 12) /* Bit 12: Capture/Compare 4 output enable (TIM2-5 only) */
#define GTIM_CCER_CC4P            (1 << 13) /* Bit 13: Capture/Compare 4 output Polarity (TIM2-5 only) */

/* 16-bit counter register */

#define GTIM_CNT_SHIFT            (0)       /* Bits 0-15: Timer counter value */
#define GTIM_CNT_MASK             (0xffff << ATIM_CNT_SHIFT)

#ifdef CONFIG_STM32_STM32F30XX
#  define GTIM_CCER_UIFCPY        (1 << 31) /* Bit 31: UIF copy */
#endif

/* Repitition counter (TIM15-17 only) */

#define GTIM_RCR_REP_SHIFT      (0)       /* Bits 0-7: Repetition Counter Value */
#define GTIM_RCR_REP_MASK       (0xff << GTIM_RCR_REP_SHIFT)

#define GTIM_RCR_REP_MAX        128

/* Break and dead-time register (TIM15-17 only */

#define GTIM_BDTR_DTG_SHIFT       (0)       /* Bits 7:0 [7:0]: Dead-Time Generator set-up */
#define GTIM_BDTR_DTG_MASK        (0xff << GTIM_BDTR_DTG_SHIFT)
#define GTIM_BDTR_LOCK_SHIFT      (8)       /* Bits 9:8 [1:0]: Lock Configuration */
#define GTIM_BDTR_LOCK_MASK       (3 << GTIM_BDTR_LOCK_SHIFT)
#  define GTIM_BDTR_LOCKOFF       (0 << GTIM_BDTR_LOCK_SHIFT) /* 00: LOCK OFF - No bit is write protected */
#  define GTIM_BDTR_LOCK1         (1 << GTIM_BDTR_LOCK_SHIFT) /* 01: LOCK Level 1 protection */
#  define GTIM_BDTR_LOCK2         (2 << GTIM_BDTR_LOCK_SHIFT) /* 10: LOCK Level 2 protection */
#  define GTIM_BDTR_LOCK3         (3 << GTIM_BDTR_LOCK_SHIFT) /* 11: LOCK Level 3 protection */ */
#define GTIM_BDTR_OSSI            (1 << 10) /* Bit 10: Off-State Selection for Idle mode */
#define GTIM_BDTR_OSSR            (1 << 11) /* Bit 11: Off-State Selection for Run mode */
#define GTIM_BDTR_BKE             (1 << 12) /* Bit 12: Break enable */
#define GTIM_BDTR_BKP             (1 << 13) /* Bit 13: Break Polarity */
#define GTIM_BDTR_AOE             (1 << 14) /* Bit 14: Automatic Output enable */
#define GTIM_BDTR_MOE             (1 << 15) /* Bit 15: Main Output enable */
#define GTIM_BDTR_BKF_SHIFT     (16)      /* Bits 16-19: Break filter */
#define GTIM_BDTR_BKF_MASK      (15 << GTIM_BDTR_BKF_SHIFT)
#  define GTIM_BDTR_BKF_NOFILT    (0 << GTIM_BDTR_BKF_SHIFT)   /* 0000: No filter, BRK acts asynchronously */
#  define GTIM_BDTR_BKF_FCKINT2   (1 << GTIM_BDTR_BKF_SHIFT)   /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define GTIM_BDTR_BKF_FCKINT4   (2 << GTIM_BDTR_BKF_SHIFT)   /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define GTIM_BDTR_BKF_FCKINT8   (3 << GTIM_BDTR_BKF_SHIFT)   /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define GTIM_BDTR_BKF_FDTSd26   (4 << GTIM_BDTR_BKF_SHIFT)   /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define GTIM_BDTR_BKF_FDTSd28   (5 << GTIM_BDTR_BKF_SHIFT)   /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define GTIM_BDTR_BKF_FDTSd36   (6 << GTIM_BDTR_BKF_SHIFT)   /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define GTIM_BDTR_BKF_FDTSd38   (7 << GTIM_BDTR_BKF_SHIFT)   /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define GTIM_BDTR_BKF_FDTSd86   (8 << GTIM_BDTR_BKF_SHIFT)   /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define GTIM_BDTR_BKF_FDTSd88   (9 << GTIM_BDTR_BKF_SHIFT)   /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define GTIM_BDTR_BKF_FDTSd165  (10 << GTIM_BDTR_BKF_SHIFT)  /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define GTIM_BDTR_BKF_FDTSd166  (11 << GTIM_BDTR_BKF_SHIFT)  /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define GTIM_BDTR_BKF_FDTSd168  (12 << GTIM_BDTR_BKF_SHIFT)  /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define GTIM_BDTR_BKF_FDTSd325  (13 << GTIM_BDTR_BKF_SHIFT)  /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define GTIM_BDTR_BKF_FDTSd326  (14 << GTIM_BDTR_BKF_SHIFT)  /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define GTIM_BDTR_BKF_FDTSd328  (15 << GTIM_BDTR_BKF_SHIFT)  /* 1111: fSAMPLING=fDTS/32, N=8 */

/* DMA control register */

#define GTIM_DCR_DBA_SHIFT        (0)       /* Bits 4-0: DMA Base Address */
#define GTIM_DCR_DBA_MASK         (0x1f << GTIM_DCR_DBA_SHIFT)
#define GTIM_DCR_DBL_SHIFT        (8)       /* Bits 12-8: DMA Burst Length */
#define GTIM_DCR_DBL_MASK         (0x1f << GTIM_DCR_DBL_SHIFT)

/* Timer 2/5 option register */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define TIM2_OR_ITR1_RMP_SHIFT  (10)     /* Bits 10-11: Internal trigger 1 remap */
#  define TIM2_OR_ITR1_RMP_MASK   (3 << TIM2_OR_ITR1_RMP_SHIFT)
#    define TIM2_OR_ITR1_TIM8_TRGOUT (0 << TIM2_OR_ITR1_RMP_SHIFT) /* 00: TIM2_ITR1 input connected to TIM8_TRGOUT */
#    define TIM2_OR_ITR1_PTP      (1 << TIM2_OR_ITR1_RMP_SHIFT) /* 01: TIM2_ITR1 input connected to PTP trigger output */
#    define TIM2_OR_ITR1_OTGFSSOF (2 << TIM2_OR_ITR1_RMP_SHIFT) /* 10: TIM2_ITR1 input connected to OTG FS SOF */
#    define TIM2_OR_ITR1_OTGHSSOF (3 << TIM2_OR_ITR1_RMP_SHIFT) /* 11: TIM2_ITR1 input connected to OTG HS SOF */

#  define TIM5_OR_TI4_RMP_SHIFT   (6)     /* Bits 6-7: Internal trigger 4 remap */
#  define TIM5_OR_TI4_RMP_MASK    (3 << TIM5_OR_TI4_RMP_SHIFT)
#    define TIM5_OR_TI4_GPIO      (0 << TIM5_OR_TI4_RMP_SHIFT) /* 00: TIM5_CH4 input connected to GPIO */
#    define TIM5_OR_TI4_LSI       (1 << TIM5_OR_TI4_RMP_SHIFT) /* 01: TIM5_CH4 input connected to LSI internal clock */
#    define TIM5_OR_TI4_LSE       (2 << TIM5_OR_TI4_RMP_SHIFT) /* 10: TIM5_CH4 input connected to LSE internal clock */
#    define TIM5_OR_TI4_RTC       (3 << TIM5_OR_TI4_RMP_SHIFT) /* 11: TIM5_CH4 input connected to RTC output event */

#  define TIM11_OR_TI1_RMP_SHIFT  (6)     /* Bits 6-7: Internal trigger 4 remap */
#  define TIM11_OR_TI1_RMP_MASK   (3 << TIM11_OR_TI1_RMP_SHIFT)
#    define TIM11_OR_TI1_GPIO     (0 << TIM11_OR_TI1_RMP_SHIFT) /* 00-11: TIM11_CH1 input connected to GPIO */
#    define TIM11_OR_TI1_HSERTC   (3 << TIM11_OR_TI1_RMP_SHIFT) /* 11: TIM11_CH1 input connected to HSE_RTC clock */
#endif

/* Timer 16 Option Register */

#ifdef CONFIG_STM32_STM32F30XX
#  define TIM16_OR_RMP_SHIFT      (0)     /* Bits 0-1: Timer 16 input 1 connection */
#  define TIM16_OR_RMP_MASK       (3 << TIM16_OR_RMP_SHIFT)
#    define TIM16_OR_RMP_GPRIO    (3 << TIM16_OR_RMP_SHIFT) /* TIM16 TI1 is connected to GPIO */
#    define TIM16_OR_RMP_RTC      (3 << TIM16_OR_RMP_SHIFT) /* TIM16 TI1 is connected to RTC_clock */
#    define TIM16_OR_RMP_HSEd32   (3 << TIM16_OR_RMP_SHIFT) /* TIM16 TI1 is connected to HSE/32 */
#    define TIM16_OR_RMP_MCO      (3 << TIM16_OR_RMP_SHIFT) /* TIM16 TI1 is connected to MCO */
#endif

/* Control register 1 */

#define BTIM_CR1_CEN              (1 << 0)  /* Bit 0: Counter enable */
#define BTIM_CR1_UDIS             (1 << 1)  /* Bit 1: Update Disable */
#define BTIM_CR1_URS              (1 << 2)  /* Bit 2: Update Request Source */
#define BTIM_CR1_OPM              (1 << 3)  /* Bit 3: One Pulse Mode */
#define BTIM_CR1_ARPE             (1 << 7)  /* Bit 7: Auto-Reload Preload enable */

/* Control register 2 */

#define BTIM_CR2_MMS_SHIFT        (4)       /* Bits 6-4: Master Mode Selection */
#define BTIM_CR2_MMS_MASK         (7 << BTIM_CR2_MMS_SHIFT)
#  define BTIM_CR2_RESET          (0 << BTIM_CR2_MMS_SHIFT) /* 000: Reset */
#  define BTIM_CR2_ENAB           (1 << BTIM_CR2_MMS_SHIFT) /* 001: Enable */
#  define BTIM_CR2_UPDT           (2 << BTIM_CR2_MMS_SHIFT) /* 010: Update */

/* DMA/Interrupt enable register */

#define BTIM_DIER_UIE             (1 << 0)  /* Bit 0: Update interrupt enable */
#define BTIM_DIER_UDE             (1 << 8)  /* Bit 8: Update DMA request enable */

/* Status register */

#define BTIM_SR_UIF               (1 << 0)  /* Bit 0: Update interrupt flag */

/* Event generation register */

#define BTIM_EGR_UG               (1 << 0)  /* Bit 0: Update generation */

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_TIM_H */

