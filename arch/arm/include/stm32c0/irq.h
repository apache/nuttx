/****************************************************************************
 * arch/arm/include/stm32c0/irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32C0_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32C0_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif
#include <arch/stm32c0/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* Common Processor Exceptions (vectors 0-15) */

#define STM32_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define STM32_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define STM32_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
                                     /* Vectors 4-10: Reserved */
#define STM32_IRQ_SVCALL        (11) /* Vector 11: SVC call */
                                     /* Vector 12-13: Reserved */
#define STM32_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define STM32_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define STM32_IRQ_EXTINT        (16) /* Vector number of the first external interrupt */

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* External interrupt vectors */

#define STM32_IRQ_WWDG         (STM32_IRQ_EXTINT + 0)  /* 0: Window Watchdog interrupt */
#define STM32_IRQ_PVM          (STM32_IRQ_EXTINT + 1)  /* 1: VDDIO2 monitor interrupt */
#define STM32_IRQ_RTC          (STM32_IRQ_EXTINT + 2)  /* 2: RTC */
#define STM32_IRQ_FLASH        (STM32_IRQ_EXTINT + 3)  /* 3: Flash */
#define STM32_IRQ_RCC          (STM32_IRQ_EXTINT + 4)  /* 4: RCC */
#define STM32_IRQ_EXTI0_1      (STM32_IRQ_EXTINT + 5)  /* 5: EXTI0_1 */
#define STM32_IRQ_EXTI2_3      (STM32_IRQ_EXTINT + 6)  /* 6: EXTI2_3 */
#define STM32_IRQ_EXTI4_15     (STM32_IRQ_EXTINT + 7)  /* 7: EXTI4_15 */
#define STM32_IRQ_USB          (STM32_IRQ_EXTINT + 8)  /* 8: USB global interrupt */
#define STM32_IRQ_DMA1CH1      (STM32_IRQ_EXTINT + 9)  /* 9: DMA1_CH1 */
#define STM32_IRQ_DMA1CH2      (STM32_IRQ_EXTINT + 10) /* 10: DMA1_CH2 */
#define STM32_IRQ_DMA1CH3      (STM32_IRQ_EXTINT + 10) /* 10: DMA1_CH3 */
#define STM32_IRQ_DMA1CH4      (STM32_IRQ_EXTINT + 11) /* 11: DMA1_CH4 */
#define STM32_IRQ_DMA1CH5      (STM32_IRQ_EXTINT + 11) /* 11: DMA1_CH5 */
#define STM32_IRQ_DMA1CH6      (STM32_IRQ_EXTINT + 11) /* 11: DMA1_CH6 */
#define STM32_IRQ_DMA1CH7      (STM32_IRQ_EXTINT + 11) /* 11: DMA1_CH7 */
#define STM32_IRQ_DMAMUX       (STM32_IRQ_EXTINT + 11) /* 11: DMAMUX */
#define STM32_IRQ_ADC          (STM32_IRQ_EXTINT + 12) /* 12: ADC */
#define STM32_IRQ_TIM1_BRK     (STM32_IRQ_EXTINT + 13) /* 13: TIM1_BRK_UP_TRG_COM */
#define STM32_IRQ_TIM1_CC      (STM32_IRQ_EXTINT + 14) /* 14: TIM1_CC */
#define STM32_IRQ_TIM2         (STM32_IRQ_EXTINT + 15) /* 15: TIM2 */
#define STM32_IRQ_TIM3         (STM32_IRQ_EXTINT + 16) /* 16: TIM3 */
#define STM32_IRQ_TIM6         (STM32_IRQ_EXTINT + 17) /* 17: TIM6 */
#define STM32_IRQ_TIM14        (STM32_IRQ_EXTINT + 19) /* 19: TIM14 */
#define STM32_IRQ_TIM15        (STM32_IRQ_EXTINT + 20) /* 20: TIM15 */
#define STM32_IRQ_TIM16        (STM32_IRQ_EXTINT + 21) /* 21: TIM16 */
#define STM32_IRQ_TIM17        (STM32_IRQ_EXTINT + 22) /* 22: TIM17 */
#define STM32_IRQ_I2C1         (STM32_IRQ_EXTINT + 23) /* 23: I2C1 */
#define STM32_IRQ_I2C2         (STM32_IRQ_EXTINT + 24) /* 24: I2C2 */
#define STM32_IRQ_SPI1         (STM32_IRQ_EXTINT + 25) /* 25: SPI1 */
#define STM32_IRQ_SPI2         (STM32_IRQ_EXTINT + 26) /* 26: SPI2 */
#define STM32_IRQ_USART1       (STM32_IRQ_EXTINT + 27) /* 27: USART1 */
#define STM32_IRQ_USART2       (STM32_IRQ_EXTINT + 28) /* 28: USART2 */
#define STM32_IRQ_USART3       (STM32_IRQ_EXTINT + 29) /* 29: USART3 */
#define STM32_IRQ_USART4       (STM32_IRQ_EXTINT + 29) /* 29: USART4 */
#define STM32_IRQ_FDCAN1_0     (STM32_IRQ_EXTINT + 30) /* 30: FDCAN global interrupt 0 */
#define STM32_IRQ_FDCAN1_1     (STM32_IRQ_EXTINT + 31) /* 31: FDCAN global interrupt 1 */

#define STM32_IRQ_NEXTINTS     (32)

#define NR_IRQS                (STM32_IRQ_EXTINT + STM32_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_STM32C0_IRQ_H */
