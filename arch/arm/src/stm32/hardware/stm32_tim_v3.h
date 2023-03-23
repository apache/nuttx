/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_tim_v3.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_TIM_V3_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_TIM_V3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* BTIM: Basic Timers - TIM6 and TIM7 */

#define STM32_BTIM_CR1_OFFSET      0x0000  /* Control register 1 (16-bit) */
#define STM32_BTIM_CR2_OFFSET      0x0004  /* Control register 2 (16-bit) */
#define STM32_BTIM_DIER_OFFSET     0x000c  /* DMA/Interrupt enable register (16-bit) */
#define STM32_BTIM_SR_OFFSET       0x0010  /* Status register (16-bit) */
#define STM32_BTIM_EGR_OFFSET      0x0014  /* Event generation register (16-bit) */
#define STM32_BTIM_CNT_OFFSET      0x0024  /* Counter (32-bit register, 16-bit counter) */
#define STM32_BTIM_PSC_OFFSET      0x0028  /* Prescaler (16-bit) */
#define STM32_BTIM_ARR_OFFSET      0x002c  /* Auto-reload register (32-bit) */

/* GTIM: General Timers
 * 16-/32-bit General Timers with DMA: TIM2, TM3, TIM4, and TIM5
 * 16-bit General Timers with DMA: TIM15, TIM16, and TIM17
 *
 * Caution! TIM2/5, TIM3/4, TIM15, and TIM16/17 are slightly different and
 * have different registers, register sizes, and register bitfields!
 *
 * Some registers are marked with the following notes:
 *
 * Note 1: The register is 32-bit but its contents have different variants
 *         for TIM3/4 and TIM2/5, TIM15
 * Note 2: This register not available on TIM15
 * Note 3: This register not available on TIM16/17
 */

#define STM32_GTIM_CR1_OFFSET      0x0000  /* Control register 1 (16-bit) */
#define STM32_GTIM_CR2_OFFSET      0x0004  /* Control register 2 (32-bit TIM2-5; 16-bit TIM15-17) */
#define STM32_GTIM_SMCR_OFFSET     0x0008  /* Slave mode control register (32-bit TIM2-5, TIM15, See Note 3) */
#define STM32_GTIM_DIER_OFFSET     0x000c  /* DMA/Interrupt enable register (32-bit TIM2-5; 16-bit TIM15) */
#define STM32_GTIM_SR_OFFSET       0x0010  /* Status register (32-bit TIM2-5; 16-bit TIM15-17) */
#define STM32_GTIM_EGR_OFFSET      0x0014  /* Event generation register (16-bit) */
#define STM32_GTIM_CCMR1_OFFSET    0x0018  /* Capture/compare mode register 1 (32-bit) */
#define STM32_GTIM_CCMR2_OFFSET    0x001c  /* Capture/compare mode register 2 (32-bit) */
#define STM32_GTIM_CCER_OFFSET     0x0020  /* Capture/compare enable register (16-bit) */
#define STM32_GTIM_CNT_OFFSET      0x0024  /* Counter (32-bit, See Note 1) */
#define STM32_GTIM_PSC_OFFSET      0x0028  /* Prescaler (16-bit) */
#define STM32_GTIM_ARR_OFFSET      0x002c  /* Auto-reload register (32-bit, See Note 1) */
#define STM32_GTIM_RCR_OFFSET      0x0030  /* Repetition counter register (16-bit, TIM15 only) */
#define STM32_GTIM_CCR1_OFFSET     0x0034  /* Capture/compare register 1 (32-bit, See Note 1) */
#define STM32_GTIM_CCR2_OFFSET     0x0038  /* Capture/compare register 2 (32-bit, See Notes 1, 3) */
#define STM32_GTIM_CCR3_OFFSET     0x003c  /* Capture/compare register 3 (32-bit, See Notes 1, 3) */
#define STM32_GTIM_CCR4_OFFSET     0x0040  /* Capture/compare register 4 (32-bit, See Notes 1, 3) */
#define STM32_GTIM_BDTR_OFFSET     0x0044  /* Break and dead-time register (32-bit, TIM15-17 only) */
#define STM32_GTIM_OR1_OFFSET      0x0050  /* Option register 1 (TIM16/17 only) */
#define STM32_GTIM_DTR2_OFFSET     0x0054  /* Dead-time register 2 (32-bit, TIM15-17 only) */
#define STM32_GTIM_ECR_OFFSET      0x0058  /* Encoder control register (32-bit, See Notes 2, 3) */
#define STM32_GTIM_TISEL_OFFSET    0x005c  /* Timer input selection register (32-bit) */
#define STM32_GTIM_AF1_OFFSET      0x0060  /* Alternate function option register 1 (32-bit) */
#define STM32_GTIM_AF2_OFFSET      0x0064  /* Alternate function option register 2 (32-bit) */
#define STM32_GTIM_DCR_OFFSET      0x03dc  /* DMA control register (16-bit, TIM2-5 only) */
#define STM32_GTIM_DMAR_OFFSET     0x03e0  /* DMA address for burst mode (16-bit, TIM2-5 only) */

/* ATIM: Advanced Timers - TIM1, TIM8, and TIM20 */

#define STM32_ATIM_CR1_OFFSET      0x0000  /* Control register 1 (16-bit) */
#define STM32_ATIM_CR2_OFFSET      0x0004  /* Control register 2 (32-bit) */
#define STM32_ATIM_SMCR_OFFSET     0x0008  /* Slave mode control register (32-bit) */
#define STM32_ATIM_DIER_OFFSET     0x000c  /* DMA/Interrupt enable register (32-bit) */
#define STM32_ATIM_SR_OFFSET       0x0010  /* Status register (32-bit) */
#define STM32_ATIM_EGR_OFFSET      0x0014  /* Event generation register (16-bit) */
#define STM32_ATIM_CCMR1_OFFSET    0x0018  /* Capture/compare mode register 1 (32-bit) */
#define STM32_ATIM_CCMR2_OFFSET    0x001c  /* Capture/compare mode register 2 (32-bit) */
#define STM32_ATIM_CCER_OFFSET     0x0020  /* Capture/compare enable register (32-bit) */
#define STM32_ATIM_CNT_OFFSET      0x0024  /* Counter (32-bit register, 16-bit counter) */
#define STM32_ATIM_PSC_OFFSET      0x0028  /* Prescaler (16-bit) */
#define STM32_ATIM_ARR_OFFSET      0x002c  /* Auto-reload register (32-bit) */
#define STM32_ATIM_RCR_OFFSET      0x0030  /* Repetition counter register (16-bit) */
#define STM32_ATIM_CCR1_OFFSET     0x0034  /* Capture/compare register 1 (32-bit) */
#define STM32_ATIM_CCR2_OFFSET     0x0038  /* Capture/compare register 2 (32-bit) */
#define STM32_ATIM_CCR3_OFFSET     0x003c  /* Capture/compare register 3 (32-bit) */
#define STM32_ATIM_CCR4_OFFSET     0x0040  /* Capture/compare register 4 (32-bit) */
#define STM32_ATIM_BDTR_OFFSET     0x0044  /* Break and dead-time register (32-bit) */
#define STM32_ATIM_CCR5_OFFSET     0x0048  /* Capture/compare register 5 (32-bit) */
#define STM32_ATIM_CCR6_OFFSET     0x004c  /* Capture/compare register 6 (32-bit) */
#define STM32_ATIM_CCMR3_OFFSET    0x0050  /* Capture/compare mode register 3 (32-bit) */
#define STM32_ATIM_DTR2_OFFSET     0x0054  /* Dead-time register 2 (32-bit) */
#define STM32_ATIM_ECR_OFFSET      0x0058  /* Encoder control register (32-bit) */
#define STM32_ATIM_TISEL_OFFSET    0x005c  /* Timer input selection register (32-bit) */
#define STM32_ATIM_AF1_OFFSET      0x0060  /* Alternate function option register 1 (32-bit) */
#define STM32_ATIM_AF2_OFFSET      0x0064  /* Alternate function option register 2 (32-bit) */
#define STM32_ATIM_DCR_OFFSET      0x03dc  /* DMA control register (32-bit) */
#define STM32_ATIM_DMAR_OFFSET     0x03e0  /* DMA address for full transfer (32-bit) */

/* Register Addresses *******************************************************/

/* ATIM: Advanced Timers - TIM1, TIM8, and TIM20 */

#if STM32_NATIM > 0
#  define STM32_TIM1_CR1               (STM32_TIM1_BASE + STM32_ATIM_CR1_OFFSET)
#  define STM32_TIM1_CR2               (STM32_TIM1_BASE + STM32_ATIM_CR2_OFFSET)
#  define STM32_TIM1_SMCR              (STM32_TIM1_BASE + STM32_ATIM_SMCR_OFFSET)
#  define STM32_TIM1_DIER              (STM32_TIM1_BASE + STM32_ATIM_DIER_OFFSET)
#  define STM32_TIM1_SR                (STM32_TIM1_BASE + STM32_ATIM_SR_OFFSET)
#  define STM32_TIM1_EGR               (STM32_TIM1_BASE + STM32_ATIM_EGR_OFFSET)
#  define STM32_TIM1_CCMR1             (STM32_TIM1_BASE + STM32_ATIM_CCMR1_OFFSET)
#  define STM32_TIM1_CCMR2             (STM32_TIM1_BASE + STM32_ATIM_CCMR2_OFFSET)
#  define STM32_TIM1_CCER              (STM32_TIM1_BASE + STM32_ATIM_CCER_OFFSET)
#  define STM32_TIM1_CNT               (STM32_TIM1_BASE + STM32_ATIM_CNT_OFFSET)
#  define STM32_TIM1_PSC               (STM32_TIM1_BASE + STM32_ATIM_PSC_OFFSET)
#  define STM32_TIM1_ARR               (STM32_TIM1_BASE + STM32_ATIM_ARR_OFFSET)
#  define STM32_TIM1_RCR               (STM32_TIM1_BASE + STM32_ATIM_RCR_OFFSET)
#  define STM32_TIM1_CCR1              (STM32_TIM1_BASE + STM32_ATIM_CCR1_OFFSET)
#  define STM32_TIM1_CCR2              (STM32_TIM1_BASE + STM32_ATIM_CCR2_OFFSET)
#  define STM32_TIM1_CCR3              (STM32_TIM1_BASE + STM32_ATIM_CCR3_OFFSET)
#  define STM32_TIM1_CCR4              (STM32_TIM1_BASE + STM32_ATIM_CCR4_OFFSET)
#  define STM32_TIM1_BDTR              (STM32_TIM1_BASE + STM32_ATIM_BDTR_OFFSET)
#  define STM32_TIM1_CCR5              (STM32_TIM1_BASE + STM32_ATIM_CCR5_OFFSET)
#  define STM32_TIM1_CCR6              (STM32_TIM1_BASE + STM32_ATIM_CCR6_OFFSET)
#  define STM32_TIM1_CCMR3             (STM32_TIM1_BASE + STM32_ATIM_CCMR3_OFFSET)
#  define STM32_TIM1_DTR2              (STM32_TIM1_BASE + STM32_ATIM_DTR2_OFFSET)
#  define STM32_TIM1_ECR               (STM32_TIM1_BASE + STM32_ATIM_ECR_OFFSET)
#  define STM32_TIM1_TISEL             (STM32_TIM1_BASE + STM32_ATIM_TISEL_OFFSET)
#  define STM32_TIM1_AF1               (STM32_TIM1_BASE + STM32_ATIM_AF1_OFFSET)
#  define STM32_TIM1_AF2               (STM32_TIM1_BASE + STM32_ATIM_AF2_OFFSET)
#  define STM32_TIM1_DCR               (STM32_TIM1_BASE + STM32_ATIM_DCR_OFFSET)
#  define STM32_TIM1_DMAR              (STM32_TIM1_BASE + STM32_ATIM_DMAR_OFFSET)
#endif

#if STM32_NATIM > 1
#  define STM32_TIM8_CR1               (STM32_TIM8_BASE + STM32_ATIM_CR1_OFFSET)
#  define STM32_TIM8_CR2               (STM32_TIM8_BASE + STM32_ATIM_CR2_OFFSET)
#  define STM32_TIM8_SMCR              (STM32_TIM8_BASE + STM32_ATIM_SMCR_OFFSET)
#  define STM32_TIM8_DIER              (STM32_TIM8_BASE + STM32_ATIM_DIER_OFFSET)
#  define STM32_TIM8_SR                (STM32_TIM8_BASE + STM32_ATIM_SR_OFFSET)
#  define STM32_TIM8_EGR               (STM32_TIM8_BASE + STM32_ATIM_EGR_OFFSET)
#  define STM32_TIM8_CCMR1             (STM32_TIM8_BASE + STM32_ATIM_CCMR1_OFFSET)
#  define STM32_TIM8_CCMR2             (STM32_TIM8_BASE + STM32_ATIM_CCMR2_OFFSET)
#  define STM32_TIM8_CCER              (STM32_TIM8_BASE + STM32_ATIM_CCER_OFFSET)
#  define STM32_TIM8_CNT               (STM32_TIM8_BASE + STM32_ATIM_CNT_OFFSET)
#  define STM32_TIM8_PSC               (STM32_TIM8_BASE + STM32_ATIM_PSC_OFFSET)
#  define STM32_TIM8_ARR               (STM32_TIM8_BASE + STM32_ATIM_ARR_OFFSET)
#  define STM32_TIM8_RCR               (STM32_TIM8_BASE + STM32_ATIM_RCR_OFFSET)
#  define STM32_TIM8_CCR1              (STM32_TIM8_BASE + STM32_ATIM_CCR1_OFFSET)
#  define STM32_TIM8_CCR2              (STM32_TIM8_BASE + STM32_ATIM_CCR2_OFFSET)
#  define STM32_TIM8_CCR3              (STM32_TIM8_BASE + STM32_ATIM_CCR3_OFFSET)
#  define STM32_TIM8_CCR4              (STM32_TIM8_BASE + STM32_ATIM_CCR4_OFFSET)
#  define STM32_TIM8_BDTR              (STM32_TIM8_BASE + STM32_ATIM_BDTR_OFFSET)
#  define STM32_TIM8_CCR5              (STM32_TIM8_BASE + STM32_ATIM_CCR5_OFFSET)
#  define STM32_TIM8_CCR6              (STM32_TIM8_BASE + STM32_ATIM_CCR6_OFFSET)
#  define STM32_TIM8_CCMR3             (STM32_TIM8_BASE + STM32_ATIM_CCMR3_OFFSET)
#  define STM32_TIM8_DTR2              (STM32_TIM8_BASE + STM32_ATIM_DTR2_OFFSET)
#  define STM32_TIM8_ECR               (STM32_TIM8_BASE + STM32_ATIM_ECR_OFFSET)
#  define STM32_TIM8_TISEL             (STM32_TIM8_BASE + STM32_ATIM_TISEL_OFFSET)
#  define STM32_TIM8_AF1               (STM32_TIM8_BASE + STM32_ATIM_AF1_OFFSET)
#  define STM32_TIM8_AF2               (STM32_TIM8_BASE + STM32_ATIM_AF2_OFFSET)
#  define STM32_TIM8_DCR               (STM32_TIM8_BASE + STM32_ATIM_DCR_OFFSET)
#  define STM32_TIM8_DMAR              (STM32_TIM8_BASE + STM32_ATIM_DMAR_OFFSET)
#endif

#if STM32_NATIM > 2
#  define STM32_TIM20_CR1              (STM32_TIM20_BASE + STM32_ATIM_CR1_OFFSET)
#  define STM32_TIM20_CR2              (STM32_TIM20_BASE + STM32_ATIM_CR2_OFFSET)
#  define STM32_TIM20_SMCR             (STM32_TIM20_BASE + STM32_ATIM_SMCR_OFFSET)
#  define STM32_TIM20_DIER             (STM32_TIM20_BASE + STM32_ATIM_DIER_OFFSET)
#  define STM32_TIM20_SR               (STM32_TIM20_BASE + STM32_ATIM_SR_OFFSET)
#  define STM32_TIM20_EGR              (STM32_TIM20_BASE + STM32_ATIM_EGR_OFFSET)
#  define STM32_TIM20_CCMR1            (STM32_TIM20_BASE + STM32_ATIM_CCMR1_OFFSET)
#  define STM32_TIM20_CCMR2            (STM32_TIM20_BASE + STM32_ATIM_CCMR2_OFFSET)
#  define STM32_TIM20_CCER             (STM32_TIM20_BASE + STM32_ATIM_CCER_OFFSET)
#  define STM32_TIM20_CNT              (STM32_TIM20_BASE + STM32_ATIM_CNT_OFFSET)
#  define STM32_TIM20_PSC              (STM32_TIM20_BASE + STM32_ATIM_PSC_OFFSET)
#  define STM32_TIM20_ARR              (STM32_TIM20_BASE + STM32_ATIM_ARR_OFFSET)
#  define STM32_TIM20_RCR              (STM32_TIM20_BASE + STM32_ATIM_RCR_OFFSET)
#  define STM32_TIM20_CCR1             (STM32_TIM20_BASE + STM32_ATIM_CCR1_OFFSET)
#  define STM32_TIM20_CCR2             (STM32_TIM20_BASE + STM32_ATIM_CCR2_OFFSET)
#  define STM32_TIM20_CCR3             (STM32_TIM20_BASE + STM32_ATIM_CCR3_OFFSET)
#  define STM32_TIM20_CCR4             (STM32_TIM20_BASE + STM32_ATIM_CCR4_OFFSET)
#  define STM32_TIM20_BDTR             (STM32_TIM20_BASE + STM32_ATIM_BDTR_OFFSET)
#  define STM32_TIM20_CCR5             (STM32_TIM20_BASE + STM32_ATIM_CCR5_OFFSET)
#  define STM32_TIM20_CCR6             (STM32_TIM20_BASE + STM32_ATIM_CCR6_OFFSET)
#  define STM32_TIM20_CCMR3            (STM32_TIM20_BASE + STM32_ATIM_CCMR3_OFFSET)
#  define STM32_TIM20_DTR2             (STM32_TIM20_BASE + STM32_ATIM_DTR2_OFFSET)
#  define STM32_TIM20_ECR              (STM32_TIM20_BASE + STM32_ATIM_ECR_OFFSET)
#  define STM32_TIM20_TISEL            (STM32_TIM20_BASE + STM32_ATIM_TISEL_OFFSET)
#  define STM32_TIM20_AF1              (STM32_TIM20_BASE + STM32_ATIM_AF1_OFFSET)
#  define STM32_TIM20_AF2              (STM32_TIM20_BASE + STM32_ATIM_AF2_OFFSET)
#  define STM32_TIM20_DCR              (STM32_TIM20_BASE + STM32_ATIM_DCR_OFFSET)
#  define STM32_TIM20_DMAR             (STM32_TIM20_BASE + STM32_ATIM_DMAR_OFFSET)
#endif

/* GTIM: General Timers
 * 16-/32-bit General Timers with DMA: TIM2, TM3, TIM4, and TIM5
 * 16-bit General Timers with DMA: TIM15, TIM16, and TIM17
 */

#if STM32_NGTIM > 0
#  define STM32_TIM2_CR1               (STM32_TIM2_BASE + STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM2_CR2               (STM32_TIM2_BASE + STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM2_SMCR              (STM32_TIM2_BASE + STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM2_DIER              (STM32_TIM2_BASE + STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM2_SR                (STM32_TIM2_BASE + STM32_GTIM_SR_OFFSET)
#  define STM32_TIM2_EGR               (STM32_TIM2_BASE + STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM2_CCMR1             (STM32_TIM2_BASE + STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM2_CCMR2             (STM32_TIM2_BASE + STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM2_CCER              (STM32_TIM2_BASE + STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM2_CNT               (STM32_TIM2_BASE + STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM2_PSC               (STM32_TIM2_BASE + STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM2_ARR               (STM32_TIM2_BASE + STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM2_CCR1              (STM32_TIM2_BASE + STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM2_CCR2              (STM32_TIM2_BASE + STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM2_CCR3              (STM32_TIM2_BASE + STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM2_CCR4              (STM32_TIM2_BASE + STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM2_ECR               (STM32_TIM2_BASE + STM32_GTIM_ECR_OFFSET)
#  define STM32_TIM2_TISEL             (STM32_TIM2_BASE + STM32_GTIM_TISEL_OFFSET)
#  define STM32_TIM2_AF1               (STM32_TIM2_BASE + STM32_GTIM_AF1_OFFSET)
#  define STM32_TIM2_AF2               (STM32_TIM2_BASE + STM32_GTIM_AF2_OFFSET)
#  define STM32_TIM2_DCR               (STM32_TIM2_BASE + STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM2_DMAR              (STM32_TIM2_BASE + STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 1
#  define STM32_TIM3_CR1               (STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM3_CR2               (STM32_TIM3_BASE + STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM3_SMCR              (STM32_TIM3_BASE + STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM3_DIER              (STM32_TIM3_BASE + STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM3_SR                (STM32_TIM3_BASE + STM32_GTIM_SR_OFFSET)
#  define STM32_TIM3_EGR               (STM32_TIM3_BASE + STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM3_CCMR1             (STM32_TIM3_BASE + STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM3_CCMR2             (STM32_TIM3_BASE + STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM3_CCER              (STM32_TIM3_BASE + STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM3_CNT               (STM32_TIM3_BASE + STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM3_PSC               (STM32_TIM3_BASE + STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM3_ARR               (STM32_TIM3_BASE + STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM3_CCR1              (STM32_TIM3_BASE + STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM3_CCR2              (STM32_TIM3_BASE + STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM3_CCR3              (STM32_TIM3_BASE + STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM3_CCR4              (STM32_TIM3_BASE + STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM3_ECR               (STM32_TIM3_BASE + STM32_GTIM_ECR_OFFSET)
#  define STM32_TIM3_TISEL             (STM32_TIM3_BASE + STM32_GTIM_TISEL_OFFSET)
#  define STM32_TIM3_AF1               (STM32_TIM3_BASE + STM32_GTIM_AF1_OFFSET)
#  define STM32_TIM3_AF2               (STM32_TIM3_BASE + STM32_GTIM_AF2_OFFSET)
#  define STM32_TIM3_DCR               (STM32_TIM3_BASE + STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM3_DMAR              (STM32_TIM3_BASE + STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 2
#  define STM32_TIM4_CR1               (STM32_TIM4_BASE + STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM4_CR2               (STM32_TIM4_BASE + STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM4_SMCR              (STM32_TIM4_BASE + STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM4_DIER              (STM32_TIM4_BASE + STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM4_SR                (STM32_TIM4_BASE + STM32_GTIM_SR_OFFSET)
#  define STM32_TIM4_EGR               (STM32_TIM4_BASE + STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM4_CCMR1             (STM32_TIM4_BASE + STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM4_CCMR2             (STM32_TIM4_BASE + STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM4_CCER              (STM32_TIM4_BASE + STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM4_CNT               (STM32_TIM4_BASE + STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM4_PSC               (STM32_TIM4_BASE + STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM4_ARR               (STM32_TIM4_BASE + STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM4_CCR1              (STM32_TIM4_BASE + STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM4_CCR2              (STM32_TIM4_BASE + STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM4_CCR3              (STM32_TIM4_BASE + STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM4_CCR4              (STM32_TIM4_BASE + STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM4_ECR               (STM32_TIM4_BASE + STM32_GTIM_ECR_OFFSET)
#  define STM32_TIM4_TISEL             (STM32_TIM4_BASE + STM32_GTIM_TISEL_OFFSET)
#  define STM32_TIM4_AF1               (STM32_TIM4_BASE + STM32_GTIM_AF1_OFFSET)
#  define STM32_TIM4_AF2               (STM32_TIM4_BASE + STM32_GTIM_AF2_OFFSET)
#  define STM32_TIM4_DCR               (STM32_TIM4_BASE + STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM4_DMAR              (STM32_TIM4_BASE + STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 3
#  define STM32_TIM5_CR1               (STM32_TIM5_BASE + STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM5_CR2               (STM32_TIM5_BASE + STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM5_SMCR              (STM32_TIM5_BASE + STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM5_DIER              (STM32_TIM5_BASE + STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM5_SR                (STM32_TIM5_BASE + STM32_GTIM_SR_OFFSET)
#  define STM32_TIM5_EGR               (STM32_TIM5_BASE + STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM5_CCMR1             (STM32_TIM5_BASE + STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM5_CCMR2             (STM32_TIM5_BASE + STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM5_CCER              (STM32_TIM5_BASE + STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM5_CNT               (STM32_TIM5_BASE + STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM5_PSC               (STM32_TIM5_BASE + STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM5_ARR               (STM32_TIM5_BASE + STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM5_CCR1              (STM32_TIM5_BASE + STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM5_CCR2              (STM32_TIM5_BASE + STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM5_CCR3              (STM32_TIM5_BASE + STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM5_CCR4              (STM32_TIM5_BASE + STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM5_ECR               (STM32_TIM5_BASE + STM32_GTIM_ECR_OFFSET)
#  define STM32_TIM5_TISEL             (STM32_TIM5_BASE + STM32_GTIM_TISEL_OFFSET)
#  define STM32_TIM5_AF1               (STM32_TIM5_BASE + STM32_GTIM_AF1_OFFSET)
#  define STM32_TIM5_AF2               (STM32_TIM5_BASE + STM32_GTIM_AF2_OFFSET)
#  define STM32_TIM5_DCR               (STM32_TIM5_BASE + STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM5_DMAR              (STM32_TIM5_BASE + STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 4
#  define STM32_TIM15_CR1              (STM32_TIM15_BASE + STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM15_CR2              (STM32_TIM15_BASE + STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM15_SMCR             (STM32_TIM15_BASE + STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM15_DIER             (STM32_TIM15_BASE + STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM15_SR               (STM32_TIM15_BASE + STM32_GTIM_SR_OFFSET)
#  define STM32_TIM15_EGR              (STM32_TIM15_BASE + STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM15_CCMR1            (STM32_TIM15_BASE + STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM15_CCMR2            (STM32_TIM15_BASE + STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM15_CCER             (STM32_TIM15_BASE + STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM15_CNT              (STM32_TIM15_BASE + STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM15_PSC              (STM32_TIM15_BASE + STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM15_ARR              (STM32_TIM15_BASE + STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM15_CCR1             (STM32_TIM15_BASE + STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM15_CCR2             (STM32_TIM15_BASE + STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM15_CCR3             (STM32_TIM15_BASE + STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM15_CCR4             (STM32_TIM15_BASE + STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM15_ECR              (STM32_TIM15_BASE + STM32_GTIM_ECR_OFFSET)
#  define STM32_TIM15_TISEL            (STM32_TIM15_BASE + STM32_GTIM_TISEL_OFFSET)
#  define STM32_TIM15_AF1              (STM32_TIM15_BASE + STM32_GTIM_AF1_OFFSET)
#  define STM32_TIM15_AF2              (STM32_TIM15_BASE + STM32_GTIM_AF2_OFFSET)
#  define STM32_TIM15_DCR              (STM32_TIM15_BASE + STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM15_DMAR             (STM32_TIM15_BASE + STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 5
#  define STM32_TIM16_CR1              (STM32_TIM16_BASE + STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM16_CR2              (STM32_TIM16_BASE + STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM16_SMCR             (STM32_TIM16_BASE + STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM16_DIER             (STM32_TIM16_BASE + STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM16_SR               (STM32_TIM16_BASE + STM32_GTIM_SR_OFFSET)
#  define STM32_TIM16_EGR              (STM32_TIM16_BASE + STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM16_CCMR1            (STM32_TIM16_BASE + STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM16_CCMR2            (STM32_TIM16_BASE + STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM16_CCER             (STM32_TIM16_BASE + STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM16_CNT              (STM32_TIM16_BASE + STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM16_PSC              (STM32_TIM16_BASE + STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM16_ARR              (STM32_TIM16_BASE + STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM16_CCR1             (STM32_TIM16_BASE + STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM16_CCR2             (STM32_TIM16_BASE + STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM16_CCR3             (STM32_TIM16_BASE + STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM16_CCR4             (STM32_TIM16_BASE + STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM16_ECR              (STM32_TIM16_BASE + STM32_GTIM_ECR_OFFSET)
#  define STM32_TIM16_TISEL            (STM32_TIM16_BASE + STM32_GTIM_TISEL_OFFSET)
#  define STM32_TIM16_AF1              (STM32_TIM16_BASE + STM32_GTIM_AF1_OFFSET)
#  define STM32_TIM16_AF2              (STM32_TIM16_BASE + STM32_GTIM_AF2_OFFSET)
#  define STM32_TIM16_DCR              (STM32_TIM16_BASE + STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM16_DMAR             (STM32_TIM16_BASE + STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 6
#  define STM32_TIM17_CR1              (STM32_TIM17_BASE + STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM17_CR2              (STM32_TIM17_BASE + STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM17_SMCR             (STM32_TIM17_BASE + STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM17_DIER             (STM32_TIM17_BASE + STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM17_SR               (STM32_TIM17_BASE + STM32_GTIM_SR_OFFSET)
#  define STM32_TIM17_EGR              (STM32_TIM17_BASE + STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM17_CCMR1            (STM32_TIM17_BASE + STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM17_CCMR2            (STM32_TIM17_BASE + STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM17_CCER             (STM32_TIM17_BASE + STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM17_CNT              (STM32_TIM17_BASE + STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM17_PSC              (STM32_TIM17_BASE + STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM17_ARR              (STM32_TIM17_BASE + STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM17_CCR1             (STM32_TIM17_BASE + STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM17_CCR2             (STM32_TIM17_BASE + STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM17_CCR3             (STM32_TIM17_BASE + STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM17_CCR4             (STM32_TIM17_BASE + STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM17_ECR              (STM32_TIM17_BASE + STM32_GTIM_ECR_OFFSET)
#  define STM32_TIM17_TISEL            (STM32_TIM17_BASE + STM32_GTIM_TISEL_OFFSET)
#  define STM32_TIM17_AF1              (STM32_TIM17_BASE + STM32_GTIM_AF1_OFFSET)
#  define STM32_TIM17_AF2              (STM32_TIM17_BASE + STM32_GTIM_AF2_OFFSET)
#  define STM32_TIM17_DCR              (STM32_TIM17_BASE + STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM17_DMAR             (STM32_TIM17_BASE + STM32_GTIM_DMAR_OFFSET)
#endif

/* BTIM: Basic Timers - TIM6 and TIM7 */

#if STM32_NBTIM > 0
#  define STM32_TIM6_CR1               (STM32_TIM6_BASE + STM32_BTIM_CR1_OFFSET)
#  define STM32_TIM6_CR2               (STM32_TIM6_BASE + STM32_BTIM_CR2_OFFSET)
#  define STM32_TIM6_DIER              (STM32_TIM6_BASE + STM32_BTIM_DIER_OFFSET)
#  define STM32_TIM6_SR                (STM32_TIM6_BASE + STM32_BTIM_SR_OFFSET)
#  define STM32_TIM6_EGR               (STM32_TIM6_BASE + STM32_BTIM_EGR_OFFSET)
#  define STM32_TIM6_CNT               (STM32_TIM6_BASE + STM32_BTIM_CNT_OFFSET)
#  define STM32_TIM6_PSC               (STM32_TIM6_BASE + STM32_BTIM_PSC_OFFSET)
#  define STM32_TIM6_ARR               (STM32_TIM6_BASE + STM32_BTIM_ARR_OFFSET)
#endif

#if STM32_NBTIM > 1
#  define STM32_TIM7_CR1               (STM32_TIM7_BASE + STM32_BTIM_CR1_OFFSET)
#  define STM32_TIM7_CR2               (STM32_TIM7_BASE + STM32_BTIM_CR2_OFFSET)
#  define STM32_TIM7_DIER              (STM32_TIM7_BASE + STM32_BTIM_DIER_OFFSET)
#  define STM32_TIM7_SR                (STM32_TIM7_BASE + STM32_BTIM_SR_OFFSET)
#  define STM32_TIM7_EGR               (STM32_TIM7_BASE + STM32_BTIM_EGR_OFFSET)
#  define STM32_TIM7_CNT               (STM32_TIM7_BASE + STM32_BTIM_CNT_OFFSET)
#  define STM32_TIM7_PSC               (STM32_TIM7_BASE + STM32_BTIM_PSC_OFFSET)
#  define STM32_TIM7_ARR               (STM32_TIM7_BASE + STM32_BTIM_ARR_OFFSET)
#endif

/* Register Bitfield Definitions - ATIM *************************************/

/* ATIM: Advanced Timers - TIM1, TIM8, and TIM20 */

/* ATIM Control register 1 (CR1) */

#define ATIM_CR1_CEN                   (1 << 0)                           /* Bit 0: Counter enable */
#define ATIM_CR1_UDIS                  (1 << 1)                           /* Bit 1: Update disable */
#define ATIM_CR1_URS                   (1 << 2)                           /* Bit 2: Update request source */
#define ATIM_CR1_OPM                   (1 << 3)                           /* Bit 3: One pulse mode */
#define ATIM_CR1_DIR                   (1 << 4)                           /* Bit 4: Direction */
#define ATIM_CR1_CMS_SHIFT             (5)                                /* Bits 5-6: Center-aligned mode selection */
#define ATIM_CR1_CMS_MASK              (0x3 << ATIM_CR1_CMS_SHIFT)
#  define ATIM_CR1_EDGE                (0x0 << ATIM_CR1_CMS_SHIFT)        /* 00: Edge-aligned mode */
#  define ATIM_CR1_CENTER1             (0x1 << ATIM_CR1_CMS_SHIFT)        /* 01: Center-aligned mode 1 */
#  define ATIM_CR1_CENTER2             (0x2 << ATIM_CR1_CMS_SHIFT)        /* 10: Center-aligned mode 2 */
#  define ATIM_CR1_CENTER3             (0x3 << ATIM_CR1_CMS_SHIFT)        /* 11: Center-aligned mode 3 */
#define ATIM_CR1_ARPE                  (1 << 7)                           /* Bit 7: Auto-reload preload enable */
#define ATIM_CR1_CKD_SHIFT             (8)                                /* Bits 8-9: Clock division */
#define ATIM_CR1_CKD_MASK              (0x3 << ATIM_CR1_CKD_SHIFT)
#  define ATIM_CR1_TCKINT              (0x0 << ATIM_CR1_CKD_SHIFT)        /* 00: tDTS=1*tCK_INT */
#  define ATIM_CR1_2TCKINT             (0x1 << ATIM_CR1_CKD_SHIFT)        /* 01: tDTS=2*tCK_INT */
#  define ATIM_CR1_4TCKINT             (0x2 << ATIM_CR1_CKD_SHIFT)        /* 10: tDTS=4*tCK_INT */
#define ATIM_CR1_UIFREMAP              (1 << 11)                          /* Bit 11: UIF status bit remapping */
#define ATIM_CR1_DITHEN                (1 << 12)                          /* Bit 12: Dithering enable */

/* ATIM Control register 2 (CR2) */

#define ATIM_CR2_CCPC                  (1 << 0)                           /* Bit 0: Capture/Compare Preloaded Control */
#define ATIM_CR2_CCUS                  (1 << 2)                           /* Bit 2: Capture/Compare Control Update Selection */
#define ATIM_CR2_CCDS                  (1 << 3)                           /* Bit 3: Capture/Compare DMA Selection */
#define ATIM_CR2_MMS_SHIFT             (4)                                /* Bits 4-6: Master Mode Selection */
#define ATIM_CR2_MMS_MASK              (0x200007 << ATIM_CR2_MMS_SHIFT)
#  define ATIM_CR2_MMS_RESET           (0x0 << ATIM_CR2_MMS_SHIFT)        /* 0000: Reset - TIMx_EGR UG bit is TRGO */
#  define ATIM_CR2_MMS_ENABLE          (0x1 << ATIM_CR2_MMS_SHIFT)        /* 0001: Enable - CNT_EN is TRGO */
#  define ATIM_CR2_MMS_UPDATE          (0x2 << ATIM_CR2_MMS_SHIFT)        /* 0010: Update event is TRGO */
#  define ATIM_CR2_MMS_COMPP           (0x3 << ATIM_CR2_MMS_SHIFT)        /* 0010: Compare Pulse - CC1IF flag */
#  define ATIM_CR2_MMS_OC1REF          (0x4 << ATIM_CR2_MMS_SHIFT)        /* 0100: Compare OC1REF is TRGO */
#  define ATIM_CR2_MMS_OC2REF          (0x5 << ATIM_CR2_MMS_SHIFT)        /* 0101: Compare OC2REF is TRGO */
#  define ATIM_CR2_MMS_OC3REF          (0x6 << ATIM_CR2_MMS_SHIFT)        /* 0110: Compare OC3REF is TRGO */
#  define ATIM_CR2_MMS_OC4REF          (0x7 << ATIM_CR2_MMS_SHIFT)        /* 0111: Compare OC4REF is TRGO */
#  define ATIM_CR2_MMS_ENCODER         (0x200000 << ATIM_CR2_MMS_SHIFT)   /* 1000: Encoder clock is TRGO */
#define ATIM_CR2_TI1S                  (1 << 7)                           /* Bit 7: TI1 Selection */
#define ATIM_CR2_OIS1                  (1 << 8)                           /* Bit 8: Output Idle state 1 (OC1 output) */
#define ATIM_CR2_OIS1N                 (1 << 9)                           /* Bit 9: Output Idle state 1 (OC1N output) */
#define ATIM_CR2_OIS2                  (1 << 10)                          /* Bit 10: Output Idle state 2 (OC2 output) */
#define ATIM_CR2_OIS2N                 (1 << 11)                          /* Bit 11: Output Idle state 2 (OC2N output) */
#define ATIM_CR2_OIS3                  (1 << 12)                          /* Bit 12: Output Idle state 3 (OC3 output) */
#define ATIM_CR2_OIS3N                 (1 << 13)                          /* Bit 13: Output Idle state 3 (OC3N output) */
#define ATIM_CR2_OIS4                  (1 << 14)                          /* Bit 14: Output Idle state 4 (OC4 output) */
#define ATIM_CR2_OIS4N                 (1 << 15)                          /* Bit 15: Output Idle state 4 (OC4N output) */
#define ATIM_CR2_OIS5                  (1 << 16)                          /* Bit 16: OOutput Idle state 5 (OC5 output) */
#define ATIM_CR2_OIS6                  (1 << 18)                          /* Bit 18: Output Idle state 6 (OC6 output) */
#define ATIM_CR2_MMS2_SHIFT            (20)                               /* Bits 20-23: Master Mode Selection 2 */
#define ATIM_CR2_MMS2_MASK             (0xf << ATIM_CR2_MMS2_SHIFT)
#  define ATIM_CR2_MMS2_RESET          (0x0 << ATIM_CR2_MMS2_SHIFT)       /* 0000: Reset - TIMx_EGR UG bit is TRG9 */
#  define ATIM_CR2_MMS2_ENABLE         (0x1 << ATIM_CR2_MMS2_SHIFT)       /* 0001: Enable - CNT_EN is TRGO2 */
#  define ATIM_CR2_MMS2_UPDATE         (0x2 << ATIM_CR2_MMS2_SHIFT)       /* 0010: Update event is TRGH0 */
#  define ATIM_CR2_MMS2_COMPP          (0x3 << ATIM_CR2_MMS2_SHIFT)       /* 0010: Compare Pulse - CC1IF flag */
#  define ATIM_CR2_MMS2_OC1REF         (0x4 << ATIM_CR2_MMS2_SHIFT)       /* 0100: Compare OC1REF is TRGO2 */
#  define ATIM_CR2_MMS2_OC2REF         (0x5 << ATIM_CR2_MMS2_SHIFT)       /* 0101: Compare OC2REF is TRGO2 */
#  define ATIM_CR2_MMS2_OC3REF         (0x6 << ATIM_CR2_MMS2_SHIFT)       /* 0110: Compare OC3REF is TRGO2 */
#  define ATIM_CR2_MMS2_OC4REF         (0x7 << ATIM_CR2_MMS2_SHIFT)       /* 0111: Compare OC4REF is TRGO2 */
#  define ATIM_CR2_MMS2_OC5REF         (0x8 << ATIM_CR2_MMS2_SHIFT)       /* 1000: Compare OC5REF is TRGO2 */
#  define ATIM_CR2_MMS2_OC6REF         (0x9 << ATIM_CR2_MMS2_SHIFT)       /* 1001: Compare OC6REF is TRGO2 */
#  define ATIM_CR2_MMS2_CMPOC4         (0xa << ATIM_CR2_MMS2_SHIFT)       /* 1010: Compare pulse - OC4REF edge is TRGO2 */
#  define ATIM_CR2_MMS2_CMPOC6         (0xb << ATIM_CR2_MMS2_SHIFT)       /* 1011: Compare pulse - OC6REF edge is TRGO2 */
#  define ATIM_CR2_MMS2_CMPOC4R6R      (0xc << ATIM_CR2_MMS2_SHIFT)       /* 1100: Compare pulse - OC4REF/OC6REF rising */
#  define ATIM_CR2_MMS2_CMPOC4R6F      (0xd << ATIM_CR2_MMS2_SHIFT)       /* 1101: Compare pulse - OC4REF rising/OC6REF falling */
#  define ATIM_CR2_MMS2_CMPOC5R6R      (0xe << ATIM_CR2_MMS2_SHIFT)       /* 1110: Compare pulse - OC5REF/OC6REF rising */
#  define ATIM_CR2_MMS2_CMPOC5R6F      (0xf << ATIM_CR2_MMS2_SHIFT)       /* 1111: Compare pulse - OC5REF rising/OC6REF falling */

/* ATIM Slave mode control register (SMCR) */

#define ATIM_SMCR_SMS_SHIFT            (0)                                /* Bits 0-2: Slave mode selection */
#define ATIM_SMCR_SMS_MASK             (0x10007 << ATIM_SMCR_SMS_SHIFT)
#  define ATIM_SMCR_DISAB              (0x0 << ATIM_SMCR_SMS_SHIFT)       /* 0000: Slave mode disabled */
#  define ATIM_SMCR_ENCMD1             (0x1 << ATIM_SMCR_SMS_SHIFT)       /* 0001: Encoder mode 1 */
#  define ATIM_SMCR_ENCMD2             (0x2 << ATIM_SMCR_SMS_SHIFT)       /* 0010: Encoder mode 2 */
#  define ATIM_SMCR_ENCMD3             (0x3 << ATIM_SMCR_SMS_SHIFT)       /* 0011: Encoder mode 3 */
#  define ATIM_SMCR_RESET              (0x4 << ATIM_SMCR_SMS_SHIFT)       /* 0100: Reset Mode */
#  define ATIM_SMCR_GATED              (0x5 << ATIM_SMCR_SMS_SHIFT)       /* 0101: Gated Mode */
#  define ATIM_SMCR_TRIGGER            (0x6 << ATIM_SMCR_SMS_SHIFT)       /* 0110: Trigger Mode */
#  define ATIM_SMCR_EXTCLK1            (0x7 << ATIM_SMCR_SMS_SHIFT)       /* 0111: External Clock Mode 1 */
#  define ATIM_SMCR_RESET_TRIGGER      (0x10000 << ATIM_SMCR_SMS_SHIFT)   /* 1000: Combined reset + trigger */
#  define ATIM_SMCR_GATED_RESET        (0x10001 << ATIM_SMCR_SMS_SHIFT)   /* 1001: Combined gated + reset */
#  define ATIM_SMCR_ENCMD4             (0x10002 << ATIM_SMCR_SMS_SHIFT)   /* 1010: Encoder mode 4 */
#  define ATIM_SMCR_ENCMD5             (0x10003 << ATIM_SMCR_SMS_SHIFT)   /* 1011: Encoder mode 5 */
#  define ATIM_SMCR_ENCMD6             (0x10004 << ATIM_SMCR_SMS_SHIFT)   /* 1100: Encoder mode 6 */
#  define ATIM_SMCR_ENCMD7             (0x10005 << ATIM_SMCR_SMS_SHIFT)   /* 1101: Encoder mode 7 */
#  define ATIM_SMCR_ENCMD8             (0x10006 << ATIM_SMCR_SMS_SHIFT)   /* 1110: Encoder mode 8 */
#  define ATIM_SMCR_ENCMD9             (0x10007 << ATIM_SMCR_SMS_SHIFT)   /* 1111: Encoder mode 9 */
#define ATIM_SMCR_OCCS                 (1 << 3)                           /* Bit 3: OCREF clear selection */
#define ATIM_SMCR_TS_SHIFT             (4)                                /* Bits 4-6: Trigger selection */
#define ATIM_SMCR_TS_MASK              (0x30007 << ATIM_SMCR_TS_SHIFT)
#  define ATIM_SMCR_ITR0               (0x0 << ATIM_SMCR_TS_SHIFT)        /* 00 000: Internal trigger 0 (ITR0) */
#  define ATIM_SMCR_ITR1               (0x1 << ATIM_SMCR_TS_SHIFT)        /* 00 001: Internal trigger 1 (ITR1) */
#  define ATIM_SMCR_ITR2               (0x2 << ATIM_SMCR_TS_SHIFT)        /* 00 010: Internal trigger 2 (ITR2) */
#  define ATIM_SMCR_ITR3               (0x3 << ATIM_SMCR_TS_SHIFT)        /* 00 011: Internal trigger 3 (ITR3) */
#  define ATIM_SMCR_TI1FED             (0x4 << ATIM_SMCR_TS_SHIFT)        /* 00 100: TI1 Edge Detector (TI1F_ED) */
#  define ATIM_SMCR_TI1FP1             (0x5 << ATIM_SMCR_TS_SHIFT)        /* 00 101: Filtered Timer Input 1 (TI1FP1) */
#  define ATIM_SMCR_TI2FP2             (0x6 << ATIM_SMCR_TS_SHIFT)        /* 00 110: Filtered Timer Input 2 (TI2FP2) */
#  define ATIM_SMCR_ETRF               (0x7 << ATIM_SMCR_TS_SHIFT)        /* 00 111: External Trigger Input (ETRF) */
#  define ATIM_SMCR_ITR4               (0x1000 << ATIM_SMCR_TS_SHIFT)     /* 01 000: Internal Trigger 0 ITR4 */
#  define ATIM_SMCR_ITR5               (0x1001 << ATIM_SMCR_TS_SHIFT)     /* 01 001: Internal Trigger 1 ITR5 */
#  define ATIM_SMCR_ITR6               (0x1002 << ATIM_SMCR_TS_SHIFT)     /* 01 010: Internal Trigger 1 ITR6 */
#  define ATIM_SMCR_ITR7               (0x1003 << ATIM_SMCR_TS_SHIFT)     /* 01 011: Internal Trigger 1 ITR7 */
#  define ATIM_SMCR_ITR8               (0x1004 << ATIM_SMCR_TS_SHIFT)     /* 01 100: Internal Trigger 1 ITR8 */
#  define ATIM_SMCR_ITR9               (0x1005 << ATIM_SMCR_TS_SHIFT)     /* 01 101: Internal Trigger 1 ITR9 */
#  define ATIM_SMCR_ITR10              (0x1006 << ATIM_SMCR_TS_SHIFT)     /* 01 110: Internal Trigger 1 ITR10 */
#define ATIM_SMCR_MSM                  (1 << 7)                           /* Bit 7: Master/slave mode */
#define ATIM_SMCR_ETF_SHIFT            (8)                                /* Bits 8-11: External trigger filter */
#define ATIM_SMCR_ETF_MASK             (0xf << ATIM_SMCR_ETF_SHIFT)
#  define ATIM_SMCR_NOFILT             (0x0 << ATIM_SMCR_ETF_SHIFT)       /* 0000: No filter, sampling is done at fDTS */
#  define ATIM_SMCR_FCKINT2            (0x1 << ATIM_SMCR_ETF_SHIFT)       /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define ATIM_SMCR_FCKINT4            (0x2 << ATIM_SMCR_ETF_SHIFT)       /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define ATIM_SMCR_FCKINT8            (0x3 << ATIM_SMCR_ETF_SHIFT)       /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define ATIM_SMCR_FDTSd26            (0x4 << ATIM_SMCR_ETF_SHIFT)       /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define ATIM_SMCR_FDTSd28            (0x5 << ATIM_SMCR_ETF_SHIFT)       /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define ATIM_SMCR_FDTSd46            (0x6 << ATIM_SMCR_ETF_SHIFT)       /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define ATIM_SMCR_FDTSd48            (0x7 << ATIM_SMCR_ETF_SHIFT)       /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define ATIM_SMCR_FDTSd86            (0x8 << ATIM_SMCR_ETF_SHIFT)       /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define ATIM_SMCR_FDTSd88            (0x9 << ATIM_SMCR_ETF_SHIFT)       /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define ATIM_SMCR_FDTSd165           (0xa << ATIM_SMCR_ETF_SHIFT)       /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define ATIM_SMCR_FDTSd166           (0xb << ATIM_SMCR_ETF_SHIFT)       /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define ATIM_SMCR_FDTSd168           (0xc << ATIM_SMCR_ETF_SHIFT)       /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define ATIM_SMCR_FDTSd325           (0xd << ATIM_SMCR_ETF_SHIFT)       /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define ATIM_SMCR_FDTSd326           (0xe << ATIM_SMCR_ETF_SHIFT)       /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define ATIM_SMCR_FDTSd328           (0xf << ATIM_SMCR_ETF_SHIFT)       /* 1111: fSAMPLING=fDTS/32, N=8 */
#define ATIM_SMCR_ETPS_SHIFT           (12)                               /* Bits 12-13: External trigger prescaler */
#define ATIM_SMCR_ETPS_MASK            (0x3 << ATIM_SMCR_ETPS_SHIFT)
#  define ATIM_SMCR_PSCOFF             (0x0 << ATIM_SMCR_ETPS_SHIFT)      /* 00: Prescaler OFF */
#  define ATIM_SMCR_ETRPd2             (0x1 << ATIM_SMCR_ETPS_SHIFT)      /* 01: ETRP frequency divided by 2 */
#  define ATIM_SMCR_ETRPd4             (0x2 << ATIM_SMCR_ETPS_SHIFT)      /* 10: ETRP frequency divided by 4 */
#  define ATIM_SMCR_ETRPd8             (0x3 << ATIM_SMCR_ETPS_SHIFT)      /* 11: ETRP frequency divided by 8 */
#define ATIM_SMCR_ECE                  (1 << 14)                          /* Bit 14: External clock enable */
#define ATIM_SMCR_ETP                  (1 << 15)                          /* Bit 15: External trigger polarity */
#define ATIM_SMCR_SMS                  (1 << 16)                          /* Bit 16: Slave mode selection - bit 3 */
#define ATIM_SMCR_SMSPE                (1 << 24)                          /* Bit 24: SMS preload enable */
#define ATIM_SMCR_SMSPS                (1 << 25)                          /* Bit 25: SMS preload source */

/* ATIM DMA/Interrupt enable register (DIER) */

#define ATIM_DIER_UIE                  (1 << 0)                           /* Bit 0: Update interrupt enable */
#define ATIM_DIER_CC1IE                (1 << 1)                           /* Bit 1: Capture/Compare 1 interrupt enable */
#define ATIM_DIER_CC2IE                (1 << 2)                           /* Bit 2: Capture/Compare 2 interrupt enable */
#define ATIM_DIER_CC3IE                (1 << 3)                           /* Bit 3: Capture/Compare 3 interrupt enable */
#define ATIM_DIER_CC4IE                (1 << 4)                           /* Bit 4: Capture/Compare 4 interrupt enable */
#define ATIM_DIER_COMIE                (1 << 5)                           /* Bit 5: COM interrupt enable */
#define ATIM_DIER_TIE                  (1 << 6)                           /* Bit 6: Trigger interrupt enable */
#define ATIM_DIER_BIE                  (1 << 7)                           /* Bit 7: Break interrupt enable */
#define ATIM_DIER_UDE                  (1 << 8)                           /* Bit 8: Update DMA request enable */
#define ATIM_DIER_CC1DE                (1 << 9)                           /* Bit 9: Capture/Compare 1 DMA request enable */
#define ATIM_DIER_CC2DE                (1 << 10)                          /* Bit 10: Capture/Compare 2 DMA request enable */
#define ATIM_DIER_CC3DE                (1 << 11)                          /* Bit 11: Capture/Compare 3 DMA request enable */
#define ATIM_DIER_CC4DE                (1 << 12)                          /* Bit 12: Capture/Compare 4 DMA request enable */
#define ATIM_DIER_COMDE                (1 << 13)                          /* Bit 13: COM DMA request enable */
#define ATIM_DIER_TDE                  (1 << 14)                          /* Bit 14: Trigger DMA request enable */
#define ATIM_DIER_IDXIE                (1 << 20)                          /* Bit 20: Index interrupt enable */
#define ATIM_DIER_DIRIE                (1 << 21)                          /* Bit 21: Direction change interrupt enable */
#define ATIM_DIER_IERRIE               (1 << 22)                          /* Bit 22: Index error interrupt enable */
#define ATIM_DIER_TERRIE               (1 << 23)                          /* Bit 23: Transition error interrupt enable */

/* ATIM Status register (SR) */

#define ATIM_SR_UIF                    (1 << 0)                           /* Bit 0: Update Interrupt Flag */
#define ATIM_SR_CC1IF                  (1 << 1)                           /* Bit 1: Capture/Compare 1 Interrupt Flag */
#define ATIM_SR_CC2IF                  (1 << 2)                           /* Bit 2: Capture/Compare 2 Interrupt Flag */
#define ATIM_SR_CC3IF                  (1 << 3)                           /* Bit 3: Capture/Compare 3 Interrupt Flag */
#define ATIM_SR_CC4IF                  (1 << 4)                           /* Bit 4: Capture/Compare 4 Interrupt Flag */
#define ATIM_SR_COMIF                  (1 << 5)                           /* Bit 5: Com Interrupt Flag */
#define ATIM_SR_TIF                    (1 << 6)                           /* Bit 6: Trigger Interrupt Flag */
#define ATIM_SR_BIF                    (1 << 7)                           /* Bit 7: Break Interrupt Flag */
#define ATIM_SR_B2IF                   (1 << 8)                           /* Bit 8: Break 2 Interrupt Flag */
#define ATIM_SR_CC1OF                  (1 << 9)                           /* Bit 9: Capture/Compare 1 Overcapture Flag */
#define ATIM_SR_CC2OF                  (1 << 10)                          /* Bit 10: Capture/Compare 2 Overcapture Flag */
#define ATIM_SR_CC3OF                  (1 << 11)                          /* Bit 11: Capture/Compare 3 Overcapture Flag */
#define ATIM_SR_CC4OF                  (1 << 12)                          /* Bit 12: Capture/Compare 4 Overcapture Flag */
#define ATIM_SR_SBIF                   (1 << 13)                          /* Bit 13: System Break Interrupt Flag */
#define ATIM_SR_CC5IF                  (1 << 16)                          /* Bit 16: Compare 5 Interrupt Flag */
#define ATIM_SR_CC6IF                  (1 << 17)                          /* Bit 17: Compare 6 Interrupt Flag */
#define ATIM_SR_IDXF                   (1 << 20)                          /* Bit 20: Index Interrupt Flag */
#define ATIM_SR_DIRF                   (1 << 21)                          /* Bit 21: Direction Change Interrupt Flag */
#define ATIM_SR_IERRF                  (1 << 22)                          /* Bit 22: Index Error Interrupt Flag */
#define ATIM_SR_TERRF                  (1 << 23)                          /* Bit 23: Transition Error Interrupt Flag */

/* ATIM Event generation register (EGR) */

#define ATIM_EGR_UG                    (1 << 0)                           /* Bit 0: Update Generation */
#define ATIM_EGR_CC1G                  (1 << 1)                           /* Bit 1: Capture/Compare 1 Generation */
#define ATIM_EGR_CC2G                  (1 << 2)                           /* Bit 2: Capture/Compare 2 Generation */
#define ATIM_EGR_CC3G                  (1 << 3)                           /* Bit 3: Capture/Compare 3 Generation */
#define ATIM_EGR_CC4G                  (1 << 4)                           /* Bit 4: Capture/Compare 4 Generation */
#define ATIM_EGR_COMG                  (1 << 5)                           /* Bit 5: Capture/Compare Control Update Generation */
#define ATIM_EGR_TG                    (1 << 6)                           /* Bit 6: Trigger Generation */
#define ATIM_EGR_BG                    (1 << 7)                           /* Bit 7: Break Generation */
#define ATIM_EGR_B2G                   (1 << 8)                           /* Bit 8: Break 2 Generation */

/* ATIM Capture/compare mode register 1 (CCMR1) - Output Compare Mode */

#define ATIM_CCMR1_CC1S_SHIFT          (0)                                /* Bits 0-1: Capture/Compare 1 Selection */
#define ATIM_CCMR1_CC1S_MASK           (3 << ATIM_CCMR1_CC1S_SHIFT)       /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC1FE               (1 << 2)                           /* Bit 2: Output Compare 1 Fast enable */
#define ATIM_CCMR1_OC1PE               (1 << 3)                           /* Bit 3: Output Compare 1 Preload enable */
#define ATIM_CCMR1_OC1M_SHIFT          (4)                                /* Bits 4-6: Output Compare 1 Mode */
#define ATIM_CCMR1_OC1M_MASK           (7 << ATIM_CCMR1_OC1M_SHIFT)       /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC1CE               (1 << 7)                           /* Bit 7: Output Compare 1Clear Enable */
#define ATIM_CCMR1_CC2S_SHIFT          (8)                                /* Bits 8-9: Capture/Compare 2 Selection */
#define ATIM_CCMR1_CC2S_MASK           (3 << ATIM_CCMR1_CC2S_SHIFT)       /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC2FE               (1 << 10)                          /* Bit 10: Output Compare 2 Fast enable */
#define ATIM_CCMR1_OC2PE               (1 << 11)                          /* Bit 11: Output Compare 2 Preload enable */
#define ATIM_CCMR1_OC2M_SHIFT          (12)                               /* Bits 12-14: Output Compare 2 Mode */
#define ATIM_CCMR1_OC2M_MASK           (7 << ATIM_CCMR1_OC2M_SHIFT)       /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC2CE               (1 << 15)                          /* Bit 15: Output Compare 2 Clear Enable */
#define ATIM_CCMR1_OC1M                (1 << 16)                          /* Bit 16: Output Compare 1 mode - bit 3 */
#define ATIM_CCMR1_OC2M                (1 << 24)                          /* Bit 24: Output Compare 2 mode - bit 3 */

/* ATIM Common CCMR (unshifted) Capture/Compare Selection
 * bit-field definitions
 */

#define ATIM_CCMR_CCS_CCOUT            (0x0)                              /* 00: CCx channel  output */
#define ATIM_CCMR_CCS_CCIN1            (0x1)                              /* 01: CCx channel input, ICx is TIx */
#define ATIM_CCMR_CCS_CCIN2            (0x2)                              /* 10: CCx channel input, ICx is TIy */
#define ATIM_CCMR_CCS_CCINTRC          (0x3)                              /* 11: CCx channel input, ICx is TRC */

/* ATIM Common CCMR (unshifted) Compare Mode bit field definitions */

#define ATIM_CCMR_MODE_FRZN            (0x0)                              /* 0000: Frozen */
#define ATIM_CCMR_MODE_CHACT           (0x1)                              /* 0001: Channel x active on match */
#define ATIM_CCMR_MODE_CHINACT         (0x2)                              /* 0010: Channel x inactive on match */
#define ATIM_CCMR_MODE_OCREFTOG        (0x3)                              /* 0011: OCxREF toggle ATIM_CNT=ATIM_CCRx */
#define ATIM_CCMR_MODE_OCREFLO         (0x4)                              /* 0100: OCxREF forced low */
#define ATIM_CCMR_MODE_OCREFHI         (0x5)                              /* 0101: OCxREF forced high */
#define ATIM_CCMR_MODE_PWM1            (0x6)                              /* 0110: PWM mode 1 */
#define ATIM_CCMR_MODE_PWM2            (0x7)                              /* 0111: PWM mode 2 */
#define ATIM_CCMR_MODE_OPM1            (0x8)                              /* 1000: Retriggerable OPM mode 1 */
#define ATIM_CCMR_MODE_OPM2            (0x9)                              /* 1001: Retriggerable OPM mode 2 */
#define ATIM_CCMR_MODE_COMBINED1       (0xc)                              /* 1100: Combined PWM mode 1 */
#define ATIM_CCMR_MODE_COMBINED2       (0xd)                              /* 1101: Combined PWM mode 2 */
#define ATIM_CCMR_MODE_ASYMMETRIC1     (0xe)                              /* 1110: Asymmetric PWM mode 1 */
#define ATIM_CCMR_MODE_ASYMMETRIC2     (0xf)                              /* 1111: Asymmetric PWM mode 2 */

/* ATIM Capture/compare mode register 1 (CCMR1) - Input Capture Mode
 * Bits 0-1 and Bits 8-9 are same as Output Compare Mode
 */

#define ATIM_CCMR1_IC1PSC_SHIFT        (2)                                /* Bits 2-3: Input Capture 1 Prescaler */
#define ATIM_CCMR1_IC1PSC_MASK         (0x3 << ATIM_CCMR1_IC1PSC_SHIFT)   /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_IC1F_SHIFT          (4)                                /* Bits 4-7: Input Capture 1 Filter */
#define ATIM_CCMR1_IC1F_MASK           (0x0f << ATIM_CCMR1_IC1F_SHIFT)    /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_IC2PSC_SHIFT        (10)                               /* Bits 10-11: Input Capture 2 Prescaler */
#define ATIM_CCMR1_IC2PSC_MASK         (0x3 << ATIM_CCMR1_IC2PSC_SHIFT)   /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_IC2F_SHIFT          (12)                               /* Bits 12-15: Input Capture 2 Filter */
#define ATIM_CCMR1_IC2F_MASK           (0x0f << ATIM_CCMR1_IC2F_SHIFT)    /* (See common (unshifted) bit field definitions below) */

/* Common CCMR (unshifted) Input Capture Prescaler bit-field definitions */

#define ATIM_CCMR_ICPSC_NOPSC          (0x0)                              /* 00: no prescaler, capture each edge */
#define ATIM_CCMR_ICPSC_EVENTS2        (0x1)                              /* 01: capture once every 2 events */
#define ATIM_CCMR_ICPSC_EVENTS4        (0x2)                              /* 10: capture once every 4 events */
#define ATIM_CCMR_ICPSC_EVENTS8        (0x3)                              /* 11: capture once every 8 events */

/* Common CCMR (unshifted) Input Capture Filter bit-field definitions */

#define ATIM_CCMR_ICF_NOFILT           (0x0)                              /* 0000: No filter, sampling at fDTS */
#define ATIM_CCMR_ICF_FCKINT2          (0x1)                              /* 0001: fSAMPLING=fCK_INT, N=2 */
#define ATIM_CCMR_ICF_FCKINT4          (0x2)                              /* 0010: fSAMPLING=fCK_INT, N=4 */
#define ATIM_CCMR_ICF_FCKINT8          (0x3)                              /* 0011: fSAMPLING=fCK_INT, N=8 */
#define ATIM_CCMR_ICF_FDTSd26          (0x4)                              /* 0100: fSAMPLING=fDTS/2, N=6 */
#define ATIM_CCMR_ICF_FDTSd28          (0x5)                              /* 0101: fSAMPLING=fDTS/2, N=8 */
#define ATIM_CCMR_ICF_FDTSd46          (0x6)                              /* 0110: fSAMPLING=fDTS/4, N=6 */
#define ATIM_CCMR_ICF_FDTSd48          (0x7)                              /* 0111: fSAMPLING=fDTS/4, N=8 */
#define ATIM_CCMR_ICF_FDTSd86          (0x8)                              /* 1000: fSAMPLING=fDTS/8, N=6 */
#define ATIM_CCMR_ICF_FDTSd88          (0x9)                              /* 1001: fSAMPLING=fDTS/8, N=8 */
#define ATIM_CCMR_ICF_FDTSd165         (0xa)                              /* 1010: fSAMPLING=fDTS/16, N=5 */
#define ATIM_CCMR_ICF_FDTSd166         (0xb)                              /* 1011: fSAMPLING=fDTS/16, N=6 */
#define ATIM_CCMR_ICF_FDTSd168         (0xc)                              /* 1100: fSAMPLING=fDTS/16, N=8 */
#define ATIM_CCMR_ICF_FDTSd325         (0xd)                              /* 1101: fSAMPLING=fDTS/32, N=5 */
#define ATIM_CCMR_ICF_FDTSd326         (0xe)                              /* 1110: fSAMPLING=fDTS/32, N=6 */
#define ATIM_CCMR_ICF_FDTSd328         (0xf)                              /* 1111: fSAMPLING=fDTS/32, N=8 */

/* ATIM Capture/compare mode register 2 (CCMR2) - Output Compare Mode */

#define ATIM_CCMR2_CC3S_SHIFT          (0)                                /* Bits 0-1: Capture/Compare 3 Selection */
#define ATIM_CCMR2_CC3S_MASK           (0x3 << ATIM_CCMR2_CC3S_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC3FE               (1 << 2)                           /* Bit 2: Output Compare 3 Fast enable */
#define ATIM_CCMR2_OC3PE               (1 << 3)                           /* Bit 3: Output Compare 3 Preload enable */
#define ATIM_CCMR2_OC3M_SHIFT          (4)                                /* Bits 4-6: Output Compare 3 Mode */
#define ATIM_CCMR2_OC3M_MASK           (0x7 << ATIM_CCMR2_OC3M_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC3CE               (1 << 7)                           /* Bit 7: Output Compare 3 Clear Enable */
#define ATIM_CCMR2_CC4S_SHIFT          (8)                                /* Bits 8-9: Capture/Compare 4 Selection */
#define ATIM_CCMR2_CC4S_MASK           (0x3 << ATIM_CCMR2_CC4S_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC4FE               (1 << 10)                          /* Bit 10: Output Compare 4 Fast enable */
#define ATIM_CCMR2_OC4PE               (1 << 11)                          /* Bit 11: Output Compare 4 Preload enable */
#define ATIM_CCMR2_OC4M_SHIFT          (12)                               /* Bits 12-14: Output Compare 4 Mode */
#define ATIM_CCMR2_OC4M_MASK           (0x7 << ATIM_CCMR2_OC4M_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC4CE               (1 << 15)                          /* Bit 15: Output Compare 4 Clear Enable */
#define ATIM_CCMR2_OC3M                (1 << 16)                          /* Bit 16: Output Compare 3 mode - bit 3 */
#define ATIM_CCMR2_OC4M                (1 << 24)                          /* Bit 24: Output Compare 4 mode - bit 3 */

/* ATIM Capture/compare mode register 2 (CCMR2) - Input Capture Mode
 * Bits 0-1 and Bits 8-9 are same as Output Compare Mode
 */

#define ATIM_CCMR2_IC3PSC_SHIFT        (2)                                /* Bits 2-3: Input Capture 3 Prescaler */
#define ATIM_CCMR2_IC3PSC_MASK         (3 << ATIM_CCMR2_IC3PSC_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_IC3F_SHIFT          (4)                                /* Bits 4-7: Input Capture 3 Filter */
#define ATIM_CCMR2_IC3F_MASK           (0x0f << ATIM_CCMR2_IC3F_SHIFT)    /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_IC4PSC_SHIFT        (10)                               /* Bits 10-11: Input Capture 4 Prescaler */
#define ATIM_CCMR2_IC4PSC_MASK         (3 << ATIM_CCMR2_IC4PSC_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_IC4F_SHIFT          (12)                               /* Bits 12-15: Input Capture 4 Filter */
#define ATIM_CCMR2_IC4F_MASK           (0x0f << ATIM_CCMR2_IC4F_SHIFT)    /* (See common (unshifted) bit field definitions above) */

/* ATIM Capture/compare mode register 3 (CCMR3) */

#define ATIM_CCMR3_OC5FE               (1 << 2)                           /* Bit 2: Output Compare 5 Fast enable */
#define ATIM_CCMR3_OC5PE               (1 << 3)                           /* Bit 3: Output Compare 5 Preload enable */
#define ATIM_CCMR3_OC5M_SHIFT          (4)                                /* Bits 4-6: Output Compare 5 Mode */
#define ATIM_CCMR3_OC5M_MASK           (0x7 << ATIM_CCMR3_OC5M_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR3_OC5CE               (1 << 7)                           /* Bit 7: Output Compare 5 Clear Enable */
#define ATIM_CCMR3_OC6FE               (1 << 10)                          /* Bit 10: Output Compare 6 Fast enable */
#define ATIM_CCMR3_OC6PE               (1 << 11)                          /* Bit 11: Output Compare 6 Preload enable */
#define ATIM_CCMR3_OC6M_SHIFT          (12)                               /* Bits 12-14: Output Compare 7 Mode */
#define ATIM_CCMR3_OC6M_MASK           (0x7 << ATIM_CCMR3_OC6M_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR3_OC6CE               (1 << 15)                          /* Bit 15: Output Compare 7 Clear Enable */
#define ATIM_CCMR3_OC5M                (1 << 16)                          /* Bit 16: Output Compare 5 mode - bit 3 */
#define ATIM_CCMR3_OC6M                (1 << 24)                          /* Bit 24: Output Compare 6 mode - bit 3 */

/* ATIM Capture/compare enable register (CCER) */

#define ATIM_CCER_CC1E                 (1 << 0)                           /* Bit 0: Capture/Compare 1 Output Enable */
#define ATIM_CCER_CC1P                 (1 << 1)                           /* Bit 1: Capture/Compare 1 Output Polarity */
#define ATIM_CCER_CC1NE                (1 << 2)                           /* Bit 2: Capture/Compare 1 Complementary Output Enable */
#define ATIM_CCER_CC1NP                (1 << 3)                           /* Bit 3: Capture/Compare 1 Complementary Output Polarity */
#define ATIM_CCER_CC2E                 (1 << 4)                           /* Bit 4: Capture/Compare 2 Output Enable */
#define ATIM_CCER_CC2P                 (1 << 5)                           /* Bit 5: Capture/Compare 2 Output Polarity */
#define ATIM_CCER_CC2NE                (1 << 6)                           /* Bit 6: Capture/Compare 2 Complementary Output Enable */
#define ATIM_CCER_CC2NP                (1 << 7)                           /* Bit 7: Capture/Compare 2 Complementary Output Polarity */
#define ATIM_CCER_CC3E                 (1 << 8)                           /* Bit 8: Capture/Compare 3 Output Enable */
#define ATIM_CCER_CC3P                 (1 << 9)                           /* Bit 9: Capture/Compare 3 Output Polarity */
#define ATIM_CCER_CC3NE                (1 << 10)                          /* Bit 10: Capture/Compare 3 Complementary Output Enable */
#define ATIM_CCER_CC3NP                (1 << 11)                          /* Bit 11: Capture/Compare 3 Complementary Output Polarity */
#define ATIM_CCER_CC4E                 (1 << 12)                          /* Bit 12: Capture/Compare 4 Output Enable */
#define ATIM_CCER_CC4P                 (1 << 13)                          /* Bit 13: Capture/Compare 4 Output Polarity */
#define ATIM_CCER_CC4NP                (1 << 15)                          /* Bit 15: Capture/Compare 4 Complementary Output Polarity */
#define ATIM_CCER_CC5E                 (1 << 16)                          /* Bit 16: Capture/Compare 5 Output Enable */
#define ATIM_CCER_CC5P                 (1 << 17)                          /* Bit 17: Capture/Compare 5 Output Polarity */
#define ATIM_CCER_CC6E                 (1 << 20)                          /* Bit 20: Capture/Compare 6 Output Enable */
#define ATIM_CCER_CC6P                 (1 << 21)                          /* Bit 21: Capture/Compare 6 Output Polarity */
#define ATIM_CCER_CCXBASE(ch)          (ch << 2)                          /* Each channel uses 4-bits */

/* ATIM Counter (CNT) */

#define ATIM_CNT_SHIFT                 (0)                                /* Bits 0-15: Timer counter value */
#define ATIM_CNT_MASK                  (0xffff << ATIM_CNT_SHIFT)
#define ATIM_CCER_UIFCPY               (1 << 31)                          /* Bit 31: UIF copy */

/* ATIM Prescaler (PSC) */

#define ATIM_PSC_SHIFT                 (0)
#define ATIM_PSC_MASK                  (0xffff << ATIM_PSC_SHIFT)         /* Bits 0-15: Prescaler value */

/* ATIM Auto-reload register (ARR) */

#define ATIM_ARR_SHIFT                 (0)
#define ATIM_ARR_MASK                  (0xffffffff << ATIM_ARR_SHIFT)     /* Bits 0-31: Auto reload register */

/* ATIM Repetition counter register (RCR) */

#define ATIM_RCR_SHIFT                 (0)
#define ATIM_RCR_MASK                  (0xffff << ATIM_RCR_SHIFT)         /* Bits 0-15: Repetition counter register */

/* ATIM Capture/compare register 1 (CCR1 - CCR6) */

#define ATIM_CCR_SHIFT                 (0)
#define ATIM_CCR_MASK                  (0xfffff << ATIM_CCR_SHIFT)        /* Bits 0-19: Capture/compare 1 value */
#define ATIM_CCR5_GC5C1                (1 << 29)                          /* CCR5 only, Bit 29: Group Channel 5 and Channel 1 */
#define ATIM_CCR5_GC5C2                (1 << 30)                          /* CCR5 only, Bit 30: Group Channel 5 and Channel 2 */
#define ATIM_CCR5_GC5C3                (1 << 31)                          /* CCR5 only, Bit 31: Group Channel 5 and Channel 3 */

/* ATIM Break and dead-time register (BDTR) */

#define ATIM_BDTR_DTG_SHIFT            (0)                                /* Bits 0-7: Dead-Time Generator set-up */
#define ATIM_BDTR_DTG_MASK             (0xff << ATIM_BDTR_DTG_SHIFT)
#define ATIM_BDTR_LOCK_SHIFT           (8)                                /* Bits 8-9: Lock Configuration */
#define ATIM_BDTR_LOCK_MASK            (3 << ATIM_BDTR_LOCK_SHIFT)
#  define ATIM_BDTR_LOCKOFF            (0 << ATIM_BDTR_LOCK_SHIFT)        /* 00: LOCK OFF - No bit is write protected */
#  define ATIM_BDTR_LOCK1              (1 << ATIM_BDTR_LOCK_SHIFT)        /* 01: LOCK Level 1 protection */
#  define ATIM_BDTR_LOCK2              (2 << ATIM_BDTR_LOCK_SHIFT)        /* 10: LOCK Level 2 protection */
#  define ATIM_BDTR_LOCK3              (3 << ATIM_BDTR_LOCK_SHIFT)        /* 11: LOCK Level 3 protection */
#define ATIM_BDTR_OSSI                 (1 << 10)                          /* Bit 10: Off-State Selection for Idle mode */
#define ATIM_BDTR_OSSR                 (1 << 11)                          /* Bit 11: Off-State Selection for Run mode */
#define ATIM_BDTR_BKE                  (1 << 12)                          /* Bit 12: Break enable */
#define ATIM_BDTR_BKP                  (1 << 13)                          /* Bit 13: Break Polarity */
#define ATIM_BDTR_AOE                  (1 << 14)                          /* Bit 14: Automatic Output enable */
#define ATIM_BDTR_MOE                  (1 << 15)                          /* Bit 15: Main Output enable */

#define ATIM_BDTR_BKF_SHIFT            (16)                               /* Bits 16-19: Break filter */
#define ATIM_BDTR_BKF_MASK             (15 << ATIM_BDTR_BKF_SHIFT)
#  define ATIM_BDTR_BKF_NOFILT         (0 << ATIM_BDTR_BKF_SHIFT)         /* 0000: No filter, BRK acts asynchronously */
#  define ATIM_BDTR_BKF_FCKINT2        (1 << ATIM_BDTR_BKF_SHIFT)         /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define ATIM_BDTR_BKF_FCKINT4        (2 << ATIM_BDTR_BKF_SHIFT)         /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define ATIM_BDTR_BKF_FCKINT8        (3 << ATIM_BDTR_BKF_SHIFT)         /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define ATIM_BDTR_BKF_FDTSd26        (4 << ATIM_BDTR_BKF_SHIFT)         /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define ATIM_BDTR_BKF_FDTSd28        (5 << ATIM_BDTR_BKF_SHIFT)         /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define ATIM_BDTR_BKF_FDTSd36        (6 << ATIM_BDTR_BKF_SHIFT)         /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define ATIM_BDTR_BKF_FDTSd38        (7 << ATIM_BDTR_BKF_SHIFT)         /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define ATIM_BDTR_BKF_FDTSd86        (8 << ATIM_BDTR_BKF_SHIFT)         /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define ATIM_BDTR_BKF_FDTSd88        (9 << ATIM_BDTR_BKF_SHIFT)         /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define ATIM_BDTR_BKF_FDTSd165       (10 << ATIM_BDTR_BKF_SHIFT)        /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define ATIM_BDTR_BKF_FDTSd166       (11 << ATIM_BDTR_BKF_SHIFT)        /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define ATIM_BDTR_BKF_FDTSd168       (12 << ATIM_BDTR_BKF_SHIFT)        /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define ATIM_BDTR_BKF_FDTSd325       (13 << ATIM_BDTR_BKF_SHIFT)        /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define ATIM_BDTR_BKF_FDTSd326       (14 << ATIM_BDTR_BKF_SHIFT)        /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define ATIM_BDTR_BKF_FDTSd328       (15 << ATIM_BDTR_BKF_SHIFT)        /* 1111: fSAMPLING=fDTS/32, N=8 */
#define ATIM_BDTR_BK2F_SHIFT           (20)                               /* Bits 20-23: Break 2 filter */
#define ATIM_BDTR_BK2F_MASK            (15 << ATIM_BDTR_BK2F_SHIFT)
#  define ATIM_BDTR_BK2F_NOFILT        (0 << ATIM_BDTR_BK2F_SHIFT)        /* 0000: No filter, BRK 2 acts asynchronously */
#  define ATIM_BDTR_BK2F_FCKINT2       (1 << ATIM_BDTR_BK2F_SHIFT)        /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define ATIM_BDTR_BK2F_FCKINT4       (2 << ATIM_BDTR_BK2F_SHIFT)        /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define ATIM_BDTR_BK2F_FCKINT8       (3 << ATIM_BDTR_BK2F_SHIFT)        /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define ATIM_BDTR_BK2F_FDTSd26       (4 << ATIM_BDTR_BK2F_SHIFT)        /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define ATIM_BDTR_BK2F_FDTSd28       (5 << ATIM_BDTR_BK2F_SHIFT)        /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define ATIM_BDTR_BK2F_FDTSd36       (6 << ATIM_BDTR_BK2F_SHIFT)        /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define ATIM_BDTR_BK2F_FDTSd38       (7 << ATIM_BDTR_BK2F_SHIFT)        /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define ATIM_BDTR_BK2F_FDTSd86       (8 << ATIM_BDTR_BK2F_SHIFT)        /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define ATIM_BDTR_BK2F_FDTSd88       (9 << ATIM_BDTR_BK2F_SHIFT)        /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define ATIM_BDTR_BK2F_FDTSd165      (10 << ATIM_BDTR_BK2F_SHIFT)       /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define ATIM_BDTR_BK2F_FDTSd166      (11 << ATIM_BDTR_BK2F_SHIFT)       /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define ATIM_BDTR_BK2F_FDTSd168      (12 << ATIM_BDTR_BK2F_SHIFT)       /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define ATIM_BDTR_BK2F_FDTSd325      (13 << ATIM_BDTR_BK2F_SHIFT)       /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define ATIM_BDTR_BK2F_FDTSd326      (14 << ATIM_BDTR_BK2F_SHIFT)       /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define ATIM_BDTR_BK2F_FDTSd328      (15 << ATIM_BDTR_BK2F_SHIFT)       /* 1111: fSAMPLING=fDTS/32, N=8 */
#define ATIM_BDTR_BK2E                 (1 << 24)                          /* Bit 24: Break 2 enable */
#define ATIM_BDTR_BK2P                 (1 << 25)                          /* Bit 25: Break 2 polarity */
#define ATIM_BDTR_BK_DSRM              (1 << 26)                          /* Bit 26: Break disarm */
#define ATIM_BDTR_BK2_DSRM             (1 << 27)                          /* Bit 27: Break 2 disarm */
#define ATIM_BDTR_BK_BID               (1 << 28)                          /* Bit 28: Break bidirectional */
#define ATIM_BDTR_BK2_BID              (1 << 29)                          /* Bit 29: Break 2 bidirectional */

/* ATIM Dead-time register 2 (DTR2) */

#define ATIM_DTR2_DTGF_SHIFT           (0)                                /* Dead time falling edge generator setup */
#define ATIM_DTR2_DTGF_MASK            (0xff << ATIM_DTR2_DTGF_SHIFT)
#define ATIM_DTR2_DTAE                 (1 << 16)                          /* Dead time asymmetric enable */
#define ATIM_DTR2_DTPE                 (1 << 17)                          /* Dead time preload enable */

/* ATIM Encoder control register (ECR) */

#define ATIM_ECR_IE                    (1 << 0)                           /* Index enable (indicates if index resets counter): 0=no, 1=yes */
#define ATIM_ECR_IDIR_SHIFT            (1)                                /* Index direction */
#define ATIM_ECR_IDIR_MASK             (0x3 << ATIM_ECR_IDIR_SHIFT)
#  define ATIM_ECR_IDIR_BOTH           (0x0 << ATIM_ECR_IDIR_SHIFT)       /* Index resets the counter in both directions */
#  define ATIM_ECR_IDIR_UP             (0x1 << ATIM_ECR_IDIR_SHIFT)       /* Index resets the counter when counting up */
#  define ATIM_ECR_IDIR_DOWN           (0x2 << ATIM_ECR_IDIR_SHIFT)       /* Index resets the counter when counting down */
#define ATIM_ECR_FIDX                  (1 << 5)                           /* First index: 0=index always resets counter, 1=only first index resets counter */
#define ATIM_ECR_IPOS_SHIFT            (6)                                /* Index reset position */
#define ATIM_ECR_IPOS_MASK             (0x3 << ATIM_ECR_IPOS_SHIFT)
#  define ATIM_ECR_IPOS_00             (0x0 << ATIM_ECR_IPOS_SHIFT)       /* Index resets position when channels A,B = 0,0 */
#  define ATIM_ECR_IPOS_01             (0x1 << ATIM_ECR_IPOS_SHIFT)       /* Index resets position when channels A,B = 0,1 */
#  define ATIM_ECR_IPOS_10             (0x2 << ATIM_ECR_IPOS_SHIFT)       /* Index resets position when channels A,B = 1,0 */
#  define ATIM_ECR_IPOS_11             (0x3 << ATIM_ECR_IPOS_SHIFT)       /* Index resets position when channels A,B = 1,1 */
#define ATIM_ECR_PW_SHIFT              (16)                               /* Pulse duration in units of t[PWG], see ATIM_ECR_PWPRSC_MASK */
#define ATIM_ECR_PW_MASK               (0xff << ATIM_ECR_PW_SHIFT)
#define ATIM_ECR_PWPRSC_SHIFT          (24)                               /* Pulse width prescaler */
#define ATIM_ECR_PWPRSC_MASK           (0x7 << ATIM_ECR_PWPRSC_SHIFT)     /* t[PWG] = (2^(PWPRSC[2:0])) * t[tim_ker_ck] */

/* ATIM Timer input selection register (TISEL) */

#define ATIM_TISEL_TI1SEL_SHIFT        (0)
#define ATIM_TISEL_TI1SEL_MASK         (0xf << TIM_TISEL_TI1SEL_SHIFT)

#define ATIM_TISEL_TI2SEL_SHIFT        (8)
#define ATIM_TISEL_TI2SEL_MASK         (0xf << TIM_TISEL_TI2SEL_SHIFT)

#define ATIM_TISEL_TI3SEL_SHIFT        (16)
#define ATIM_TISEL_TI3SEL_MASK         (0xf << TIM_TISEL_TI3SEL_SHIFT)

#define ATIM_TISEL_TI4SEL_SHIFT        (24)
#define ATIM_TISEL_TI4SEL_MASK         (0xf << TIM_TISEL_TI4SEL_SHIFT)

/* ATIM Alternate function option register 1 (AF1) */

#define ATIM_AF1_BKINE                 (1 << 0)                           /* BRK BKIN input enable */
#define ATIM_AF1_BKCMP1E               (1 << 1)                           /* BRK COMP1 enable */
#define ATIM_AF1_BKCMP2E               (1 << 2)                           /* BRK COMP2 enable */
#define ATIM_AF1_BKCMP3E               (1 << 3)                           /* BRK COMP3 enable */
#define ATIM_AF1_BKCMP4E               (1 << 4)                           /* BRK COMP4 enable */
#define ATIM_AF1_BKCMP5E               (1 << 5)                           /* BRK COMP5 enable */
#define ATIM_AF1_BKCMP6E               (1 << 6)                           /* BRK COMP6 enable */
#define ATIM_AF1_BKCMP7E               (1 << 7)                           /* BRK COMP7 enable */
#define ATIM_AF1_BKINP                 (1 << 9)                           /* BRK BKIN input polarity */
#define ATIM_AF1_BKCMP1P               (1 << 10)                          /* BRK COMP1 input polarity */
#define ATIM_AF1_BKCMP2P               (1 << 11)                          /* BRK COMP2 input polarity */
#define ATIM_AF1_BKCMP3P               (1 << 12)                          /* BRK COMP3 input polarity */
#define ATIM_AF1_BKCMP4P               (1 << 13)                          /* BRK COMP4 input polarity */
#define ATIM_AF1_ETRSEL_SHIFT          (14)                               /* ETR source selection) */
#define ATIM_AF1_ETRSEL_MASK           (0xf << TIM1_AF1_ETRSEL_Pos)

/* ATIM Alternate function option register 2 (AF2) */

#define ATIM_AF2_BK2INE                (1 << 0)                           /* BRK2 BKIN input enable */
#define ATIM_AF2_BK2CMP1E              (1 << 1)                           /* BRK2 COMP1 enable */
#define ATIM_AF2_BK2CMP2E              (1 << 2)                           /* BRK2 COMP2 enable */
#define ATIM_AF2_BK2CMP3E              (1 << 3)                           /* BRK2 COMP3 enable */
#define ATIM_AF2_BK2CMP4E              (1 << 4)                           /* BRK2 COMP4 enable */
#define ATIM_AF2_BK2CMP5E              (1 << 5)                           /* BRK2 COMP5 enable */
#define ATIM_AF2_BK2CMP6E              (1 << 6)                           /* BRK2 COMP6 enable */
#define ATIM_AF2_BK2CMP7E              (1 << 7)                           /* BRK2 COMP7 enable */
#define ATIM_AF2_BK2INP                (1 << 9)                           /* BRK2 BKIN input polarity */
#define ATIM_AF2_BK2CMP1P              (1 << 10)                          /* BRK2 COMP1 input polarity */
#define ATIM_AF2_BK2CMP2P              (1 << 11)                          /* BRK2 COMP2 input polarity */
#define ATIM_AF2_BK2CMP3P              (1 << 12)                          /* BRK2 COMP3 input polarity */
#define ATIM_AF2_BK2CMP4P              (1 << 13)                          /* BRK2 COMP4 input polarity */
#define ATIM_AF2_OCRSEL_SHIFT          (16)                               /* ocref_clr source selection */
#define ATIM_AF2_OCRSEL_MASK           (0x7 << TIM1_AF2_OCRSEL_Pos)

/* ATIM DMA control register (DCR) */

#define ATIM_DCR_DBA_SHIFT             (0)                                /* Bits 0-4: DMA Base Address */
#define ATIM_DCR_DBA_MASK              (0x1f << ATIM_DCR_DBA_SHIFT)
#define ATIM_DCR_DBL_SHIFT             (8)                                /* Bits 8-12: DMA Burst Length */
#define ATIM_DCR_DBL_MASK              (0x1f << ATIM_DCR_DBL_SHIFT)
#  define ATIM_DCR_DBL(n)              (((n) - 1) << ATIM_DCR_DBL_SHIFT)  /* n transfers, n = 1..18 */

/* Register Bitfield Definitions - GTIM *************************************/

/* GTIM: General Timers
 * 16-/32-bit General Timers with DMA: TIM2, TM3, TIM4, and TIM5
 * 16-bit General Timers with DMA: TIM15, TIM16, and TIM17
 *
 * Caution! TIM2/5, TIM3/4, TIM15, and TIM16/17 are slightly different and
 * have different registers, register sizes, and register bitfields!
 */

/* GTIM Control register 1 (CR1) */

#define GTIM_CR1_CEN                   (1 << 0)                           /* Bit 0: Counter enable */
#define GTIM_CR1_UDIS                  (1 << 1)                           /* Bit 1: Update disable */
#define GTIM_CR1_URS                   (1 << 2)                           /* Bit 2: Update request source */
#define GTIM_CR1_OPM                   (1 << 3)                           /* Bit 3: One pulse mode */
#define GTIM_CR1_DIR                   (1 << 4)                           /* Bit 4: Direction */
#define GTIM_CR1_CMS_SHIFT             (5)                                /* Bits 5-6: Center-aligned mode selection */
#define GTIM_CR1_CMS_MASK              (0x3 << GTIM_CR1_CMS_SHIFT)
#  define GTIM_CR1_EDGE                (0x0 << GTIM_CR1_CMS_SHIFT)        /* 00: Edge-aligned mode */
#  define GTIM_CR1_CENTER1             (0x1 << GTIM_CR1_CMS_SHIFT)        /* 01: Center-aligned mode 1 */
#  define GTIM_CR1_CENTER2             (0x2 << GTIM_CR1_CMS_SHIFT)        /* 10: Center-aligned mode 2 */
#  define GTIM_CR1_CENTER3             (0x3 << GTIM_CR1_CMS_SHIFT)        /* 11: Center-aligned mode 3 */
#define GTIM_CR1_ARPE                  (1 << 7)                           /* Bit 7: Auto-reload preload enable */
#define GTIM_CR1_CKD_SHIFT             (8)                                /* Bits 8-9: Clock division */
#define GTIM_CR1_CKD_MASK              (0x3 << GTIM_CR1_CKD_SHIFT)
#  define GTIM_CR1_TCKINT              (0x0 << GTIM_CR1_CKD_SHIFT)        /* 00: tDTS=1*tCK_INT */
#  define GTIM_CR1_2TCKINT             (0x1 << GTIM_CR1_CKD_SHIFT)        /* 01: tDTS=2*tCK_INT */
#  define GTIM_CR1_4TCKINT             (0x2 << GTIM_CR1_CKD_SHIFT)        /* 10: tDTS=4*tCK_INT */
#define GTIM_CR1_UIFREMAP              (1 << 11)                          /* Bit 11: UIF status bit remapping */
#define GTIM_CR1_DITHEN                (1 << 12)                          /* Bit 12: Dithering enable */

/* GTIM Control register 2 (CR2) */

#define GTIM_CR2_CCDS                  (1 << 3)                           /* Bit 3: Capture/Compare DMA Selection */
#define GTIM_CR2_MMS_SHIFT             (4)                                /* Bits 4-6: Master Mode Selection */
#define GTIM_CR2_MMS_MASK              (0x200007 << GTIM_CR2_MMS_SHIFT)
#  define GTIM_CR2_MMS_RESET           (0x0 << GTIM_CR2_MMS_SHIFT)        /* 0000: Reset - TIMx_EGR UG bit is TRGO */
#  define GTIM_CR2_MMS_ENABLE          (0x1 << GTIM_CR2_MMS_SHIFT)        /* 0001: Enable - CNT_EN is TRGO */
#  define GTIM_CR2_MMS_UPDATE          (0x2 << GTIM_CR2_MMS_SHIFT)        /* 0010: Update event is TRGO */
#  define GTIM_CR2_MMS_COMPP           (0x3 << GTIM_CR2_MMS_SHIFT)        /* 0010: Compare Pulse - CC1IF flag */
#  define GTIM_CR2_MMS_OC1REF          (0x4 << GTIM_CR2_MMS_SHIFT)        /* 0100: Compare OC1REF is TRGO */
#  define GTIM_CR2_MMS_OC2REF          (0x5 << GTIM_CR2_MMS_SHIFT)        /* 0101: Compare OC2REF is TRGO */
#  define GTIM_CR2_MMS_OC3REF          (0x6 << GTIM_CR2_MMS_SHIFT)        /* 0110: Compare OC3REF is TRGO */
#  define GTIM_CR2_MMS_OC4REF          (0x7 << GTIM_CR2_MMS_SHIFT)        /* 0111: Compare OC4REF is TRGO */
#  define GTIM_CR2_MMS_ENCODER         (0x200000 << GTIM_CR2_MMS_SHIFT)   /* 1000: Encoder clock is TRGO */
#define GTIM_CR2_TI1S                  (1 << 7)                           /* Bit 7: TI1 Selection */

/* GTIM Slave mode control register (SMCR) */

#define GTIM_SMCR_SMS_SHIFT            (0)                                /* Bits 0-2: Slave mode selection */
#define GTIM_SMCR_SMS_MASK             (0x10007 << GTIM_SMCR_SMS_SHIFT)
#  define GTIM_SMCR_DISAB              (0x0 << GTIM_SMCR_SMS_SHIFT)       /* 0000: Slave mode disabled */
#  define GTIM_SMCR_ENCMD1             (0x1 << GTIM_SMCR_SMS_SHIFT)       /* 0001: Encoder mode 1 */
#  define GTIM_SMCR_ENCMD2             (0x2 << GTIM_SMCR_SMS_SHIFT)       /* 0010: Encoder mode 2 */
#  define GTIM_SMCR_ENCMD3             (0x3 << GTIM_SMCR_SMS_SHIFT)       /* 0011: Encoder mode 3 */
#  define GTIM_SMCR_RESET              (0x4 << GTIM_SMCR_SMS_SHIFT)       /* 0100: Reset Mode */
#  define GTIM_SMCR_GATED              (0x5 << GTIM_SMCR_SMS_SHIFT)       /* 0101: Gated Mode */
#  define GTIM_SMCR_TRIGGER            (0x6 << GTIM_SMCR_SMS_SHIFT)       /* 0110: Trigger Mode */
#  define GTIM_SMCR_EXTCLK1            (0x7 << GTIM_SMCR_SMS_SHIFT)       /* 0111: External Clock Mode 1 */
#  define GTIM_SMCR_RESET_TRIGGER      (0x10000 << GTIM_SMCR_SMS_SHIFT)   /* 1000: Combined reset + trigger */
#  define GTIM_SMCR_GATED_RESET        (0x10001 << GTIM_SMCR_SMS_SHIFT)   /* 1001: Combined gated + reset */
#  define GTIM_SMCR_ENCMD4             (0x10002 << GTIM_SMCR_SMS_SHIFT)   /* 1010: Encoder mode 4 */
#  define GTIM_SMCR_ENCMD5             (0x10003 << GTIM_SMCR_SMS_SHIFT)   /* 1011: Encoder mode 5 */
#  define GTIM_SMCR_ENCMD6             (0x10004 << GTIM_SMCR_SMS_SHIFT)   /* 1100: Encoder mode 6 */
#  define GTIM_SMCR_ENCMD7             (0x10005 << GTIM_SMCR_SMS_SHIFT)   /* 1101: Encoder mode 7 */
#  define GTIM_SMCR_ENCMD8             (0x10006 << GTIM_SMCR_SMS_SHIFT)   /* 1110: Encoder mode 8 */
#  define GTIM_SMCR_ENCMD9             (0x10007 << GTIM_SMCR_SMS_SHIFT)   /* 1111: Encoder mode 9 */
#define GTIM_SMCR_TS_SHIFT             (4)                                /* Bits 4-6: Trigger selection */
#define GTIM_SMCR_TS_MASK              (0x30007 << GTIM_SMCR_TS_SHIFT)
#  define GTIM_SMCR_ITR0               (0x0 << GTIM_SMCR_TS_SHIFT)        /* 00 000: Internal trigger 0 (ITR0) */
#  define GTIM_SMCR_ITR1               (0x1 << GTIM_SMCR_TS_SHIFT)        /* 00 001: Internal trigger 1 (ITR1) */
#  define GTIM_SMCR_ITR2               (0x2 << GTIM_SMCR_TS_SHIFT)        /* 00 010: Internal trigger 2 (ITR2) */
#  define GTIM_SMCR_ITR3               (0x3 << GTIM_SMCR_TS_SHIFT)        /* 00 011: Internal trigger 3 (ITR3) */
#  define GTIM_SMCR_TI1FED             (0x4 << GTIM_SMCR_TS_SHIFT)        /* 00 100: TI1 Edge Detector (TI1F_ED) */
#  define GTIM_SMCR_TI1FP1             (0x5 << GTIM_SMCR_TS_SHIFT)        /* 00 101: Filtered Timer Input 1 (TI1FP1) */
#  define GTIM_SMCR_TI2FP2             (0x6 << GTIM_SMCR_TS_SHIFT)        /* 00 110: Filtered Timer Input 2 (TI2FP2) */
#  define GTIM_SMCR_ETRF               (0x7 << GTIM_SMCR_TS_SHIFT)        /* 00 111: External Trigger Input (ETRF) */
#  define GTIM_SMCR_ITR4               (0x1000 << GTIM_SMCR_TS_SHIFT)     /* 01 000: Internal Trigger 0 ITR4 */
#  define GTIM_SMCR_ITR5               (0x1001 << GTIM_SMCR_TS_SHIFT)     /* 01 001: Internal Trigger 1 ITR5 */
#  define GTIM_SMCR_ITR6               (0x1002 << GTIM_SMCR_TS_SHIFT)     /* 01 010: Internal Trigger 1 ITR6 */
#  define GTIM_SMCR_ITR7               (0x1003 << GTIM_SMCR_TS_SHIFT)     /* 01 011: Internal Trigger 1 ITR7 */
#  define GTIM_SMCR_ITR8               (0x1004 << GTIM_SMCR_TS_SHIFT)     /* 01 100: Internal Trigger 1 ITR8 */
#  define GTIM_SMCR_ITR9               (0x1005 << GTIM_SMCR_TS_SHIFT)     /* 01 101: Internal Trigger 1 ITR9 */
#  define GTIM_SMCR_ITR10              (0x1006 << GTIM_SMCR_TS_SHIFT)     /* 01 110: Internal Trigger 1 ITR10 */
#define GTIM_SMCR_MSM                  (1 << 7)                           /* Bit 7: Master/slave mode */
#define GTIM_SMCR_ETF_SHIFT            (8)                                /* Bits 8-11: External trigger filter */
#define GTIM_SMCR_ETF_MASK             (0xf << GTIM_SMCR_ETF_SHIFT)
#  define GTIM_SMCR_NOFILT             (0x0 << GTIM_SMCR_ETF_SHIFT)       /* 0000: No filter, sampling is done at fDTS */
#  define GTIM_SMCR_FCKINT2            (0x1 << GTIM_SMCR_ETF_SHIFT)       /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define GTIM_SMCR_FCKINT4            (0x2 << GTIM_SMCR_ETF_SHIFT)       /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define GTIM_SMCR_FCKINT8            (0x3 << GTIM_SMCR_ETF_SHIFT)       /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define GTIM_SMCR_FDTSd26            (0x4 << GTIM_SMCR_ETF_SHIFT)       /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define GTIM_SMCR_FDTSd28            (0x5 << GTIM_SMCR_ETF_SHIFT)       /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define GTIM_SMCR_FDTSd46            (0x6 << GTIM_SMCR_ETF_SHIFT)       /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define GTIM_SMCR_FDTSd48            (0x7 << GTIM_SMCR_ETF_SHIFT)       /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define GTIM_SMCR_FDTSd86            (0x8 << GTIM_SMCR_ETF_SHIFT)       /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define GTIM_SMCR_FDTSd88            (0x9 << GTIM_SMCR_ETF_SHIFT)       /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define GTIM_SMCR_FDTSd165           (0xa << GTIM_SMCR_ETF_SHIFT)       /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define GTIM_SMCR_FDTSd166           (0xb << GTIM_SMCR_ETF_SHIFT)       /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define GTIM_SMCR_FDTSd168           (0xc << GTIM_SMCR_ETF_SHIFT)       /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define GTIM_SMCR_FDTSd325           (0xd << GTIM_SMCR_ETF_SHIFT)       /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define GTIM_SMCR_FDTSd326           (0xe << GTIM_SMCR_ETF_SHIFT)       /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define GTIM_SMCR_FDTSd328           (0xf << GTIM_SMCR_ETF_SHIFT)       /* 1111: fSAMPLING=fDTS/32, N=8 */
#define GTIM_SMCR_ETPS_SHIFT           (12)                               /* Bits 12-13: External trigger prescaler */
#define GTIM_SMCR_ETPS_MASK            (0x3 << GTIM_SMCR_ETPS_SHIFT)
#  define GTIM_SMCR_PSCOFF             (0x0 << GTIM_SMCR_ETPS_SHIFT)      /* 00: Prescaler OFF */
#  define GTIM_SMCR_ETRPd2             (0x1 << GTIM_SMCR_ETPS_SHIFT)      /* 01: ETRP frequency divided by 2 */
#  define GTIM_SMCR_ETRPd4             (0x2 << GTIM_SMCR_ETPS_SHIFT)      /* 10: ETRP frequency divided by 4 */
#  define GTIM_SMCR_ETRPd8             (0x3 << GTIM_SMCR_ETPS_SHIFT)      /* 11: ETRP frequency divided by 8 */
#define GTIM_SMCR_ECE                  (1 << 14)                          /* Bit 14: External clock enable */
#define GTIM_SMCR_ETP                  (1 << 15)                          /* Bit 15: External trigger polarity */
#define GTIM_SMCR_SMS                  (1 << 16)                          /* Bit 16: Slave mode selection - bit 3 */
#define GTIM_SMCR_SMSPE                (1 << 24)                          /* Bit 24: SMS preload enable */
#define GTIM_SMCR_SMSPS                (1 << 25)                          /* Bit 25: SMS preload source */

/* GTIM DMA/Interrupt enable register (DIER) */

#define GTIM_DIER_UIE                  (1 << 0)                           /* Bit 0: Update interrupt enable */
#define GTIM_DIER_CC1IE                (1 << 1)                           /* Bit 1: Capture/Compare 1 interrupt enable */
#define GTIM_DIER_CC2IE                (1 << 2)                           /* Bit 2: Capture/Compare 2 interrupt enable */
#define GTIM_DIER_CC3IE                (1 << 3)                           /* Bit 3: Capture/Compare 3 interrupt enable */
#define GTIM_DIER_CC4IE                (1 << 4)                           /* Bit 4: Capture/Compare 4 interrupt enable */
#define GTIM_DIER_TIE                  (1 << 6)                           /* Bit 6: Trigger interrupt enable */
#define GTIM_DIER_UDE                  (1 << 8)                           /* Bit 8: Update DMA request enable */
#define GTIM_DIER_CC1DE                (1 << 9)                           /* Bit 9: Capture/Compare 1 DMA request enable */
#define GTIM_DIER_CC2DE                (1 << 10)                          /* Bit 10: Capture/Compare 2 DMA request enable */
#define GTIM_DIER_CC3DE                (1 << 11)                          /* Bit 11: Capture/Compare 3 DMA request enable */
#define GTIM_DIER_CC4DE                (1 << 12)                          /* Bit 12: Capture/Compare 4 DMA request enable */
#define GTIM_DIER_COMDE                (1 << 13)                          /* Bit 13: COM DMA request enable */
#define GTIM_DIER_TDE                  (1 << 14)                          /* Bit 14: Trigger DMA request enable */
#define GTIM_DIER_IDXIE                (1 << 20)                          /* Bit 20: Index interrupt enable */
#define GTIM_DIER_DIRIE                (1 << 21)                          /* Bit 21: Direction change interrupt enable */
#define GTIM_DIER_IERRIE               (1 << 22)                          /* Bit 22: Index error interrupt enable */
#define GTIM_DIER_TERRIE               (1 << 23)                          /* Bit 23: Transition error interrupt enable */

/* GTIM Status register (SR) */

#define GTIM_SR_UIF                    (1 << 0)                           /* Bit 0: Update Interrupt Flag */
#define GTIM_SR_CC1IF                  (1 << 1)                           /* Bit 1: Capture/Compare 1 Interrupt Flag */
#define GTIM_SR_CC2IF                  (1 << 2)                           /* Bit 2: Capture/Compare 2 Interrupt Flag */
#define GTIM_SR_CC3IF                  (1 << 3)                           /* Bit 3: Capture/Compare 3 Interrupt Flag */
#define GTIM_SR_CC4IF                  (1 << 4)                           /* Bit 4: Capture/Compare 4 Interrupt Flag */
#define GTIM_SR_TIF                    (1 << 6)                           /* Bit 6: Trigger Interrupt Flag */
#define GTIM_SR_CC1OF                  (1 << 9)                           /* Bit 9: Capture/Compare 1 Overcapture Flag */
#define GTIM_SR_CC2OF                  (1 << 10)                          /* Bit 10: Capture/Compare 2 Overcapture Flag */
#define GTIM_SR_CC3OF                  (1 << 11)                          /* Bit 11: Capture/Compare 3 Overcapture Flag */
#define GTIM_SR_CC4OF                  (1 << 12)                          /* Bit 12: Capture/Compare 4 Overcapture Flag */
#define GTIM_SR_IDXF                   (1 << 20)                          /* Bit 20: Index Interrupt Flag */
#define GTIM_SR_DIRF                   (1 << 21)                          /* Bit 21: Direction Change Interrupt Flag */
#define GTIM_SR_IERRF                  (1 << 22)                          /* Bit 22: Index Error Interrupt Flag */
#define GTIM_SR_TERRF                  (1 << 23)                          /* Bit 23: Transition Error Interrupt Flag */

/* GTIM Event generation register (EGR) */

#define GTIM_EGR_UG                    (1 << 0)                           /* Bit 0: Update Generation */
#define GTIM_EGR_CC1G                  (1 << 1)                           /* Bit 1: Capture/Compare 1 Generation */
#define GTIM_EGR_CC2G                  (1 << 2)                           /* Bit 2: Capture/Compare 2 Generation */
#define GTIM_EGR_CC3G                  (1 << 3)                           /* Bit 3: Capture/Compare 3 Generation */
#define GTIM_EGR_CC4G                  (1 << 4)                           /* Bit 4: Capture/Compare 4 Generation */
#define GTIM_EGR_TG                    (1 << 6)                           /* Bit 6: Trigger Generation */

/* GTIM Capture/compare mode registers (CCMR1) - Output Compare Mode */

#define GTIM_CCMR1_CC1S_SHIFT          (0)                                /* Bits 0-1: Capture/Compare 1 Selection */
#define GTIM_CCMR1_CC1S_MASK           (3 << GTIM_CCMR1_CC1S_SHIFT)       /* (See common (unshifted) bit field definitions below) */
#define GTIM_CCMR1_OC1FE               (1 << 2)                           /* Bit 2: Output Compare 1 Fast enable */
#define GTIM_CCMR1_OC1PE               (1 << 3)                           /* Bit 3: Output Compare 1 Preload enable */
#define GTIM_CCMR1_OC1M_SHIFT          (4)                                /* Bits 4-6: Output Compare 1 Mode */
#define GTIM_CCMR1_OC1M_MASK           (7 << GTIM_CCMR1_OC1M_SHIFT)       /* (See common (unshifted) bit field definitions below) */
#define GTIM_CCMR1_OC1CE               (1 << 7)                           /* Bit 7: Output Compare 1Clear Enable */
#define GTIM_CCMR1_CC2S_SHIFT          (8)                                /* Bits 8-9: Capture/Compare 2 Selection */
#define GTIM_CCMR1_CC2S_MASK           (3 << GTIM_CCMR1_CC2S_SHIFT)       /* (See common (unshifted) bit field definitions below) */
#define GTIM_CCMR1_OC2FE               (1 << 10)                          /* Bit 10: Output Compare 2 Fast enable */
#define GTIM_CCMR1_OC2PE               (1 << 11)                          /* Bit 11: Output Compare 2 Preload enable */
#define GTIM_CCMR1_OC2M_SHIFT          (12)                               /* Bits 12-14: Output Compare 2 Mode */
#define GTIM_CCMR1_OC2M_MASK           (7 << GTIM_CCMR1_OC2M_SHIFT)       /* (See common (unshifted) bit field definitions below) */
#define GTIM_CCMR1_OC2CE               (1 << 15)                          /* Bit 15: Output Compare 2 Clear Enable */
#define GTIM_CCMR1_OC1M                (1 << 16)                          /* Bit 16: Output Compare 1 mode - bit 3 */
#define GTIM_CCMR1_OC2M                (1 << 24)                          /* Bit 24: Output Compare 2 mode - bit 3 */

/* GTIM Common CCMR (unshifted) Capture/Compare Selection
 * bit-field definitions
 */

#define GTIM_CCMR_CCS_CCOUT            (0x0)                              /* 00: CCx channel  output */
#define GTIM_CCMR_CCS_CCIN1            (0x1)                              /* 01: CCx channel input, ICx is TIx */
#define GTIM_CCMR_CCS_CCIN2            (0x2)                              /* 10: CCx channel input, ICx is TIy */
#define GTIM_CCMR_CCS_CCINTRC          (0x3)                              /* 11: CCx channel input, ICx is TRC */

/* GTIM Common CCMR (unshifted) Compare Mode bit field definitions */

#define GTIM_CCMR_MODE_FRZN            (0x0)                              /* 0000: Frozen */
#define GTIM_CCMR_MODE_CHACT           (0x1)                              /* 0001: Channel x active on match */
#define GTIM_CCMR_MODE_CHINACT         (0x2)                              /* 0010: Channel x inactive on match */
#define GTIM_CCMR_MODE_OCREFTOG        (0x3)                              /* 0011: OCxREF toggle GTIM_CNT=GTIM_CCRx */
#define GTIM_CCMR_MODE_OCREFLO         (0x4)                              /* 0100: OCxREF forced low */
#define GTIM_CCMR_MODE_OCREFHI         (0x5)                              /* 0101: OCxREF forced high */
#define GTIM_CCMR_MODE_PWM1            (0x6)                              /* 0110: PWM mode 1 */
#define GTIM_CCMR_MODE_PWM2            (0x7)                              /* 0111: PWM mode 2 */
#define GTIM_CCMR_MODE_OPM1            (0x8)                              /* 1000: Retriggerable OPM mode 1 */
#define GTIM_CCMR_MODE_OPM2            (0x9)                              /* 1001: Retriggerable OPM mode 2 */
#define GTIM_CCMR_MODE_COMBINED1       (0xc)                              /* 1100: Combined PWM mode 1 */
#define GTIM_CCMR_MODE_COMBINED2       (0xd)                              /* 1101: Combined PWM mode 2 */
#define GTIM_CCMR_MODE_ASYMMETRIC1     (0xe)                              /* 1110: Asymmetric PWM mode 1 */
#define GTIM_CCMR_MODE_ASYMMETRIC2     (0xf)                              /* 1111: Asymmetric PWM mode 2 */

/* GTIM Capture/compare mode register 1 (CCMR1) - Input Capture Mode
 * Bits 0-1 and Bits 8-9 are same as Output Compare Mode
 */

#define GTIM_CCMR1_IC1PSC_SHIFT        (2)                                /* Bits 2-3: Input Capture 1 Prescaler */
#define GTIM_CCMR1_IC1PSC_MASK         (0x3 << GTIM_CCMR1_IC1PSC_SHIFT)   /* (See common (unshifted) bit field definitions below) */
#define GTIM_CCMR1_IC1F_SHIFT          (4)                                /* Bits 4-7: Input Capture 1 Filter */
#define GTIM_CCMR1_IC1F_MASK           (0x0f << GTIM_CCMR1_IC1F_SHIFT)    /* (See common (unshifted) bit field definitions below) */
#define GTIM_CCMR1_IC2PSC_SHIFT        (10)                               /* Bits 10-11: Input Capture 2 Prescaler */
#define GTIM_CCMR1_IC2PSC_MASK         (0x3 << GTIM_CCMR1_IC2PSC_SHIFT)   /* (See common (unshifted) bit field definitions below) */
#define GTIM_CCMR1_IC2F_SHIFT          (12)                               /* Bits 12-15: Input Capture 2 Filter */
#define GTIM_CCMR1_IC2F_MASK           (0x0f << GTIM_CCMR1_IC2F_SHIFT)    /* (See common (unshifted) bit field definitions below) */

/* Common CCMR (unshifted) Input Capture Prescaler bit-field definitions */

#define GTIM_CCMR_ICPSC_NOPSC          (0x0)                              /* 00: no prescaler, capture each edge */
#define GTIM_CCMR_ICPSC_EVENTS2        (0x1)                              /* 01: capture once every 2 events */
#define GTIM_CCMR_ICPSC_EVENTS4        (0x2)                              /* 10: capture once every 4 events */
#define GTIM_CCMR_ICPSC_EVENTS8        (0x3)                              /* 11: capture once every 8 events */

/* Common CCMR (unshifted) Input Capture Filter bit-field definitions */

#define GTIM_CCMR_ICF_NOFILT           (0x0)                              /* 0000: No filter, sampling at fDTS */
#define GTIM_CCMR_ICF_FCKINT2          (0x1)                              /* 0001: fSAMPLING=fCK_INT, N=2 */
#define GTIM_CCMR_ICF_FCKINT4          (0x2)                              /* 0010: fSAMPLING=fCK_INT, N=4 */
#define GTIM_CCMR_ICF_FCKINT8          (0x3)                              /* 0011: fSAMPLING=fCK_INT, N=8 */
#define GTIM_CCMR_ICF_FDTSd26          (0x4)                              /* 0100: fSAMPLING=fDTS/2, N=6 */
#define GTIM_CCMR_ICF_FDTSd28          (0x5)                              /* 0101: fSAMPLING=fDTS/2, N=8 */
#define GTIM_CCMR_ICF_FDTSd46          (0x6)                              /* 0110: fSAMPLING=fDTS/4, N=6 */
#define GTIM_CCMR_ICF_FDTSd48          (0x7)                              /* 0111: fSAMPLING=fDTS/4, N=8 */
#define GTIM_CCMR_ICF_FDTSd86          (0x8)                              /* 1000: fSAMPLING=fDTS/8, N=6 */
#define GTIM_CCMR_ICF_FDTSd88          (0x9)                              /* 1001: fSAMPLING=fDTS/8, N=8 */
#define GTIM_CCMR_ICF_FDTSd165         (0xa)                              /* 1010: fSAMPLING=fDTS/16, N=5 */
#define GTIM_CCMR_ICF_FDTSd166         (0xb)                              /* 1011: fSAMPLING=fDTS/16, N=6 */
#define GTIM_CCMR_ICF_FDTSd168         (0xc)                              /* 1100: fSAMPLING=fDTS/16, N=8 */
#define GTIM_CCMR_ICF_FDTSd325         (0xd)                              /* 1101: fSAMPLING=fDTS/32, N=5 */
#define GTIM_CCMR_ICF_FDTSd326         (0xe)                              /* 1110: fSAMPLING=fDTS/32, N=6 */
#define GTIM_CCMR_ICF_FDTSd328         (0xf)                              /* 1111: fSAMPLING=fDTS/32, N=8 */

/* GTIM Capture/compare mode register 2 (CCMR2) - Output Compare Mode */

#define GTIM_CCMR2_CC3S_SHIFT          (0)                                /* Bits 0-1: Capture/Compare 3 Selection */
#define GTIM_CCMR2_CC3S_MASK           (0x3 << GTIM_CCMR2_CC3S_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define GTIM_CCMR2_OC3FE               (1 << 2)                           /* Bit 2: Output Compare 3 Fast enable */
#define GTIM_CCMR2_OC3PE               (1 << 3)                           /* Bit 3: Output Compare 3 Preload enable */
#define GTIM_CCMR2_OC3M_SHIFT          (4)                                /* Bits 4-6: Output Compare 3 Mode */
#define GTIM_CCMR2_OC3M_MASK           (0x7 << GTIM_CCMR2_OC3M_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define GTIM_CCMR2_OC3CE               (1 << 7)                           /* Bit 7: Output Compare 3 Clear Enable */
#define GTIM_CCMR2_CC4S_SHIFT          (8)                                /* Bits 8-9: Capture/Compare 4 Selection */
#define GTIM_CCMR2_CC4S_MASK           (0x3 << GTIM_CCMR2_CC4S_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define GTIM_CCMR2_OC4FE               (1 << 10)                          /* Bit 10: Output Compare 4 Fast enable */
#define GTIM_CCMR2_OC4PE               (1 << 11)                          /* Bit 11: Output Compare 4 Preload enable */
#define GTIM_CCMR2_OC4M_SHIFT          (12)                               /* Bits 12-14: Output Compare 4 Mode */
#define GTIM_CCMR2_OC4M_MASK           (0x7 << GTIM_CCMR2_OC4M_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define GTIM_CCMR2_OC4CE               (1 << 15)                          /* Bit 15: Output Compare 4 Clear Enable */
#define GTIM_CCMR2_OC3M                (1 << 16)                          /* Bit 16: Output Compare 3 mode - bit 3 */
#define GTIM_CCMR2_OC4M                (1 << 24)                          /* Bit 24: Output Compare 4 mode - bit 3 */

/* GTIM Capture/compare mode register 2 (CCMR2) - Input Capture Mode
 * Bits 0-1 and Bits 8-9 are same as Output Compare Mode
 */

#define GTIM_CCMR2_IC3PSC_SHIFT        (2)                                /* Bits 2-3: Input Capture 3 Prescaler */
#define GTIM_CCMR2_IC3PSC_MASK         (3 << GTIM_CCMR2_IC3PSC_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define GTIM_CCMR2_IC3F_SHIFT          (4)                                /* Bits 4-7: Input Capture 3 Filter */
#define GTIM_CCMR2_IC3F_MASK           (0x0f << GTIM_CCMR2_IC3F_SHIFT)    /* (See common (unshifted) bit field definitions above) */
#define GTIM_CCMR2_IC4PSC_SHIFT        (10)                               /* Bits 10-11: Input Capture 4 Prescaler */
#define GTIM_CCMR2_IC4PSC_MASK         (3 << GTIM_CCMR2_IC4PSC_SHIFT)     /* (See common (unshifted) bit field definitions above) */
#define GTIM_CCMR2_IC4F_SHIFT          (12)                               /* Bits 12-15: Input Capture 4 Filter */
#define GTIM_CCMR2_IC4F_MASK           (0x0f << GTIM_CCMR2_IC4F_SHIFT)    /* (See common (unshifted) bit field definitions above) */

/* GTIM Capture/compare enable register (CCER) */

#define GTIM_CCER_CC1E                 (1 << 0)                           /* Bit 0: Capture/Compare 1 Output Enable */
#define GTIM_CCER_CC1P                 (1 << 1)                           /* Bit 1: Capture/Compare 1 Output Polarity */
#define GTIM_CCER_CC1NP                (1 << 3)                           /* Bit 3: Capture/Compare 1 Complementary Output Polarity */
#define GTIM_CCER_CC2E                 (1 << 4)                           /* Bit 4: Capture/Compare 2 Output Enable */
#define GTIM_CCER_CC2P                 (1 << 5)                           /* Bit 5: Capture/Compare 2 Output Polarity */
#define GTIM_CCER_CC2NP                (1 << 7)                           /* Bit 7: Capture/Compare 2 Complementary Output Polarity */
#define GTIM_CCER_CC3E                 (1 << 8)                           /* Bit 8: Capture/Compare 3 Output Enable */
#define GTIM_CCER_CC3P                 (1 << 9)                           /* Bit 9: Capture/Compare 3 Output Polarity */
#define GTIM_CCER_CC3NP                (1 << 11)                          /* Bit 11: Capture/Compare 3 Complementary Output Polarity */
#define GTIM_CCER_CC4E                 (1 << 12)                          /* Bit 12: Capture/Compare 4 Output Enable */
#define GTIM_CCER_CC4P                 (1 << 13)                          /* Bit 13: Capture/Compare 4 Output Polarity */
#define GTIM_CCER_CC4NP                (1 << 15)                          /* Bit 15: Capture/Compare 4 Complementary Output Polarity */
#define GTIM_CCER_CCXBASE(ch)          (ch << 2)                          /* Each channel uses 4-bits */

/* GTIM Counter (CNT) */

#define GTIM_CNT_SHIFT                 (0)                                /* Bits 0-15, 0-30, or 0-31: Timer counter value */
#define GTIM_CNT_MASK16                (0xffff << GTIM_CNT_SHIFT)         /* Bits 0-15 - 16-bit counter value */
#define GTIM_CNT_MASK31                (0x7fffffff << GTIM_CNT_SHIFT)     /* Bits 0-30 - 31-bit counter value, 32-bit counters only */
#define GTIM_CNT_MASK32                (0xffffffff << GTIM_CNT_SHIFT)     /* Bits 0-31 - 32-bit counter value, 32-bit counters only */
#define GTIM_CCER_UIFCPY               (1 << 31)                          /* Bit 31: UIF copy, if not using this bit for counter */

/* GTIM Prescaler (PSC) */

#define GTIM_PSC_SHIFT                 (0)
#define GTIM_PSC_MASK                  (0xffff << GTIM_PSC_SHIFT)         /* Bits 0-15: Prescaler value */

/* GTIM Auto-reload register (ARR) */

#define GTIM_ARR_SHIFT                 (0)
#define GTIM_ARR_MASK_20               (0xfffff << GTIM_ARR_SHIFT)        /* Bits 0-19: Auto reload register */
#define GTIM_ARR_MASK_32               (0xffffffff << GTIM_ARR_SHIFT)     /* Bits 0-31: Auto reload register, 32-bit counters only */

/* GTIM Repetition counter register (16-bit, TIM15 only) */

#define GTIM_RCR_SHIFT                 (0)
#define GTIM_RCR_MASK                  (0xff << GTIM_RCR_SHIFT)

/* GTIM Capture/compare register 1 (CCR1 - CCR4) */

#define GTIM_CCR_SHIFT                 (0)
#define GTIM_CCR_MASK_20               (0xfffff << GTIM_CCR_SHIFT)        /* Bits 0-19: Capture/compare 1 value */
#define GTIM_CCR_MASK_32               (0xffffffff << GTIM_CCR_SHIFT)     /* Bits 0-31: Capture/compare 1 value, 32-bit timers only */

/* GTIM Break and dead-time register (BDTR) - TIM15-TIM17 only */

#define GTIM_BDTR_DTG_SHIFT            (0)                                /* Bits 0-7: Dead-Time Generator set-up */
#define GTIM_BDTR_DTG_MASK             (0xff << GTIM_BDTR_DTG_SHIFT)
#define GTIM_BDTR_LOCK_SHIFT           (8)                                /* Bits 8-9: Lock Configuration */
#define GTIM_BDTR_LOCK_MASK            (3 << GTIM_BDTR_LOCK_SHIFT)
#  define GTIM_BDTR_LOCKOFF            (0 << GTIM_BDTR_LOCK_SHIFT)        /* 00: LOCK OFF - No bit is write protected */
#  define GTIM_BDTR_LOCK1              (1 << GTIM_BDTR_LOCK_SHIFT)        /* 01: LOCK Level 1 protection */
#  define GTIM_BDTR_LOCK2              (2 << GTIM_BDTR_LOCK_SHIFT)        /* 10: LOCK Level 2 protection */
#  define GTIM_BDTR_LOCK3              (3 << GTIM_BDTR_LOCK_SHIFT)        /* 11: LOCK Level 3 protection */
#define GTIM_BDTR_OSSI                 (1 << 10)                          /* Bit 10: Off-State Selection for Idle mode */
#define GTIM_BDTR_OSSR                 (1 << 11)                          /* Bit 11: Off-State Selection for Run mode */
#define GTIM_BDTR_BKE                  (1 << 12)                          /* Bit 12: Break enable */
#define GTIM_BDTR_BKP                  (1 << 13)                          /* Bit 13: Break Polarity */
#define GTIM_BDTR_AOE                  (1 << 14)                          /* Bit 14: Automatic Output enable */
#define GTIM_BDTR_MOE                  (1 << 15)                          /* Bit 15: Main Output enable */

#define GTIM_BDTR_BKF_SHIFT            (16)                               /* Bits 16-19: Break filter */
#define GTIM_BDTR_BKF_MASK             (15 << GTIM_BDTR_BKF_SHIFT)
#  define GTIM_BDTR_BKF_NOFILT         (0 << GTIM_BDTR_BKF_SHIFT)         /* 0000: No filter, BRK acts asynchronously */
#  define GTIM_BDTR_BKF_FCKINT2        (1 << GTIM_BDTR_BKF_SHIFT)         /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define GTIM_BDTR_BKF_FCKINT4        (2 << GTIM_BDTR_BKF_SHIFT)         /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define GTIM_BDTR_BKF_FCKINT8        (3 << GTIM_BDTR_BKF_SHIFT)         /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define GTIM_BDTR_BKF_FDTSd26        (4 << GTIM_BDTR_BKF_SHIFT)         /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define GTIM_BDTR_BKF_FDTSd28        (5 << GTIM_BDTR_BKF_SHIFT)         /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define GTIM_BDTR_BKF_FDTSd36        (6 << GTIM_BDTR_BKF_SHIFT)         /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define GTIM_BDTR_BKF_FDTSd38        (7 << GTIM_BDTR_BKF_SHIFT)         /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define GTIM_BDTR_BKF_FDTSd86        (8 << GTIM_BDTR_BKF_SHIFT)         /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define GTIM_BDTR_BKF_FDTSd88        (9 << GTIM_BDTR_BKF_SHIFT)         /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define GTIM_BDTR_BKF_FDTSd165       (10 << GTIM_BDTR_BKF_SHIFT)        /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define GTIM_BDTR_BKF_FDTSd166       (11 << GTIM_BDTR_BKF_SHIFT)        /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define GTIM_BDTR_BKF_FDTSd168       (12 << GTIM_BDTR_BKF_SHIFT)        /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define GTIM_BDTR_BKF_FDTSd325       (13 << GTIM_BDTR_BKF_SHIFT)        /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define GTIM_BDTR_BKF_FDTSd326       (14 << GTIM_BDTR_BKF_SHIFT)        /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define GTIM_BDTR_BKF_FDTSd328       (15 << GTIM_BDTR_BKF_SHIFT)        /* 1111: fSAMPLING=fDTS/32, N=8 */
#define GTIM_BDTR_BK_DSRM              (1 << 26)                          /* Bit 26: Break disarm */
#define GTIM_BDTR_BK_BID               (1 << 28)                          /* Bit 28: Break bidirectional */

/* GTIM Option register 1 (OR1) - TIM16, TIM17 only */

#define GTIM_OR1_HSE32EN               (1 << 0)                           /* Bit 1: HSE divide by 32 for tim_ti1_in3 */

/* GTIM Dead-time register 2 (DTR2) */

#define GTIM_DTR2_DTGF_SHIFT           (0)                                /* Dead time falling edge generator setup */
#define GTIM_DTR2_DTGF_MASK            (0xff << GTIM_DTR2_DTGF_SHIFT)
#define GTIM_DTR2_DTAE                 (1 << 16)                          /* Dead time asymmetric enable */
#define GTIM_DTR2_DTPE                 (1 << 17)                          /* Dead time preload enable */

/* GTIM Encoder control register (ECR) */

#define GTIM_ECR_IE                    (1 << 0)                           /* Index enable (indicates if index resets counter): 0=no, 1=yes */
#define GTIM_ECR_IDIR_SHIFT            (1)                                /* Index direction */
#define GTIM_ECR_IDIR_MASK             (0x3 << GTIM_ECR_IDIR_SHIFT)
#  define GTIM_ECR_IDIR_BOTH           (0x0 << GTIM_ECR_IDIR_SHIFT)       /* Index resets the counter in both directions */
#  define GTIM_ECR_IDIR_UP             (0x1 << GTIM_ECR_IDIR_SHIFT)       /* Index resets the counter when counting up */
#  define GTIM_ECR_IDIR_DOWN           (0x2 << GTIM_ECR_IDIR_SHIFT)       /* Index resets the counter when counting down */
#define GTIM_ECR_FIDX                  (1 << 5)                           /* First index: 0=index always resets counter, 1=only first index resets counter */
#define GTIM_ECR_IPOS_SHIFT            (6)                                /* Index reset position */
#define GTIM_ECR_IPOS_MASK             (0x3 << GTIM_ECR_IPOS_SHIFT)
#  define GTIM_ECR_IPOS_00             (0x0 << GTIM_ECR_IPOS_SHIFT)       /* Index resets position when channels A,B = 0,0 */
#  define GTIM_ECR_IPOS_01             (0x1 << GTIM_ECR_IPOS_SHIFT)       /* Index resets position when channels A,B = 0,1 */
#  define GTIM_ECR_IPOS_10             (0x2 << GTIM_ECR_IPOS_SHIFT)       /* Index resets position when channels A,B = 1,0 */
#  define GTIM_ECR_IPOS_11             (0x3 << GTIM_ECR_IPOS_SHIFT)       /* Index resets position when channels A,B = 1,1 */
#define GTIM_ECR_PW_SHIFT              (16)                               /* Pulse duration in units of t[PWG], see GTIM_ECR_PWPRSC_MASK */
#define GTIM_ECR_PW_MASK               (0xff << GTIM_ECR_PW_SHIFT)
#define GTIM_ECR_PWPRSC_SHIFT          (24)                               /* Pulse width prescaler */
#define GTIM_ECR_PWPRSC_MASK           (0x7 << GTIM_ECR_PWPRSC_SHIFT)     /* t[PWG] = (2^(PWPRSC[2:0])) * t[tim_ker_ck] */

/* GTIM Timer input selection register (TISEL) */

#define GTIM_TISEL_TI1SEL_SHIFT        (0) /* TIM2-TIM5, TIM15-TIM17 */
#define GTIM_TISEL_TI1SEL_MASK         (0xf << TIM_TISEL_TI1SEL_SHIFT)

#define GTIM_TISEL_TI2SEL_SHIFT        (8) /* TIM2-TIM5, TIM15 */
#define GTIM_TISEL_TI2SEL_MASK         (0xf << TIM_TISEL_TI2SEL_SHIFT)

#define GTIM_TISEL_TI3SEL_SHIFT        (16) /* TIM2-TIM5 */
#define GTIM_TISEL_TI3SEL_MASK         (0xf << TIM_TISEL_TI3SEL_SHIFT)

#define GTIM_TISEL_TI4SEL_SHIFT        (24) /* TIM2-TIM5 */
#define GTIM_TISEL_TI4SEL_MASK         (0xf << TIM_TISEL_TI4SEL_SHIFT)

/* GTIM Alternate function option register 1 (AF1) */

#define GTIM_AF1_BKINE                 (1 << 0)                           /* BRK BKIN input enable TIM15-TIM17 only */
#define GTIM_AF1_BKCMP1E               (1 << 1)                           /* BRK COMP1 enable TIM15-TIM17 only */
#define GTIM_AF1_BKCMP2E               (1 << 2)                           /* BRK COMP2 enable TIM15-TIM17 only */
#define GTIM_AF1_BKCMP3E               (1 << 3)                           /* BRK COMP3 enable TIM15-TIM17 only */
#define GTIM_AF1_BKCMP4E               (1 << 4)                           /* BRK COMP4 enable TIM15-TIM17 only */
#define GTIM_AF1_BKCMP5E               (1 << 5)                           /* BRK COMP5 enable TIM15-TIM17 only */
#define GTIM_AF1_BKCMP6E               (1 << 6)                           /* BRK COMP6 enable TIM15-TIM17 only */
#define GTIM_AF1_BKCMP7E               (1 << 7)                           /* BRK COMP7 enable TIM15-TIM17 only */
#define GTIM_AF1_BKINP                 (1 << 9)                           /* BRK BKIN input polarity TIM15-TIM17 only */
#define GTIM_AF1_BKCMP1P               (1 << 10)                          /* BRK COMP1 input polarity TIM15-TIM17 only */
#define GTIM_AF1_BKCMP2P               (1 << 11)                          /* BRK COMP2 input polarity TIM15-TIM17 only */
#define GTIM_AF1_BKCMP3P               (1 << 12)                          /* BRK COMP3 input polarity TIM15-TIM17 only */
#define GTIM_AF1_BKCMP4P               (1 << 13)                          /* BRK COMP4 input polarity TIM15-TIM17 only */
#define GTIM_AF1_ETRSEL_SHIFT          (14)                               /* ETR source selection) TIM2-TIM5 only */
#define GTIM_AF1_ETRSEL_MASK           (0xf << TIM1_AF1_ETRSEL_Pos)

/* GTIM Alternate function option register 2 (AF2) */

#define GTIM_AF2_OCRSEL_SHIFT          (16)                               /* ocref_clr source selection */
#define GTIM_AF2_OCRSEL_MASK           (0x7 << TIM1_AF2_OCRSEL_Pos)

/* GTIM DMA control register (DCR) */

#define GTIM_DCR_DBA_SHIFT             (0)                                /* Bits 0-4: DMA Base Address */
#define GTIM_DCR_DBA_MASK              (0x1f << GTIM_DCR_DBA_SHIFT)
#define GTIM_DCR_DBL_SHIFT             (8)                                /* Bits 8-12: DMA Burst Length */
#define GTIM_DCR_DBL_MASK              (0x1f << GTIM_DCR_DBL_SHIFT)
#  define GTIM_DCR_DBL(n)              (((n) - 1) << GTIM_DCR_DBL_SHIFT)  /* n transfers, n = 1..18 */

/* Register Bitfield Definitions - BTIM *************************************/

/* BTIM: Basic Timers - TIM6 and TIM7 */

/* BTIM Control register 1 (CR1) */

#define BTIM_CR1_CEN                   (1 << 0)                           /* Bit 0: Counter Enable */
#define BTIM_CR1_UDIS                  (1 << 1)                           /* Bit 1: Update Disable */
#define BTIM_CR1_URS                   (1 << 2)                           /* Bit 2: Update Request Source */
#define BTIM_CR1_OPM                   (1 << 3)                           /* Bit 3: One Pulse Mode */
#define BTIM_CR1_ARPE                  (1 << 7)                           /* Bit 7: Auto-Reload Preload enable */
#define BTIM_CR1_UIFREMAP              (1 << 11)                          /* Bit 11: UIF status bit remapping: 1=copy UIF bit to bit 32 of CNT register */
#define BTIM_CR1_DITHEN                (1 << 12)                          /* Bit 12: Dithering Enable (can change only when CEN bit is 0) */

/* BTIM Control register 2 (CR2) */

#define BTIM_CR2_MMS_SHIFT             (4)                                /* Bits 4-6: Master Mode Selection */
#define BTIM_CR2_MMS_MASK              (0x7 << BTIM_CR2_MMS_SHIFT)
#  define BTIM_CR2_RESET               (0x0 << BTIM_CR2_MMS_SHIFT)        /* 000: Reset */
#  define BTIM_CR2_ENAB                (0x1 << BTIM_CR2_MMS_SHIFT)        /* 001: Enable */
#  define BTIM_CR2_UPDT                (0x2 << BTIM_CR2_MMS_SHIFT)        /* 010: Update */

/* BTIM DMA/Interrupt enable register (DIER) */

#define BTIM_DIER_UIE                  (1 << 0)                           /* Bit 0: Update interrupt enable */
#define BTIM_DIER_UDE                  (1 << 8)                           /* Bit 8: Update DMA request enable */

/* BTIM Status register (SR) */

#define BTIM_SR_UIF                    (1 << 0)                           /* Bit 0: Update Interrupt Flag */

/* BTIM Event generation register (EGR) */

#define BTIM_EGR_UG                    (1 << 0)                           /* Bit 0: Update Generation */

/* BTIM Counter (CNT) */

#define BTIM_CNT_SHIFT                 (0)                                /* Bits 0-15: 16-bit timer counter value */
#define BTIM_CNT_MASK                  (0xffff << BTIM_CNT_SHIFT)
#define BTIM_CCER_UIFCPY               (1 << 31)                          /* Bit 31: UIF copy, if BTIM_CR1_UIFREMAP */

/* BTIM Prescaler (PSC) */

#define BTIM_PSC_SHIFT                 (0)
#define BTIM_PSC_MASK                  (0xffff << BTIM_PSC_SHIFT)         /* Bits 0-15: Prescaler value */

/* BTIM Auto-reload register (ARR) */

#define BTIM_ARR_SHIFT                 (0)                                /* Bits 0-15 or 0-19: Auto reload register */
#define BTIM_ARR_MASK_16               (0xffff << BTIM_ARR_SHIFT)         /* Bits 0-15: Auto reload register */
#define BTIM_ARR_MASK_20               (0xfffff << BTIM_ARR_SHIFT)        /* Bits 0-19: Auto reload register when BTIM_CR1_DITHEN */

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_TIM_V3_H */
