/****************************************************************************
 * arch/arm/include/stm32c5/stm32_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32C5_STM32_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32C5_STM32_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to bits
 * in the NVIC.  This does, however, waste several words of memory in the
 * IRQ to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define STM32_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define STM32_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define STM32_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define STM32_IRQ_MEMFAULT       (4) /* Vector  4: Memory management (MPU) */
#define STM32_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define STM32_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define STM32_IRQ_SECUREFAULT    (7) /* Vector  7: Secure fault */
                                     /* Vectors 8-10: Reserved */
#define STM32_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define STM32_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define STM32_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define STM32_IRQ_SYSTICK       (15) /* Vector 15: System tick */

#define STM32_IRQ_FIRST         (16) /* Vector number of the first external interrupt */

/* External interrupts (vectors >= 16) */

#define STM32_IRQ_WWDG          (STM32_IRQ_FIRST + 0)
#define STM32_IRQ_PWR_PVD       (STM32_IRQ_FIRST + 1)
#define STM32_IRQ_RTC           (STM32_IRQ_FIRST + 2)
#define STM32_IRQ_TAMP          (STM32_IRQ_FIRST + 3)
#define STM32_IRQ_RAMCFG        (STM32_IRQ_FIRST + 4)
#define STM32_IRQ_FLASH         (STM32_IRQ_FIRST + 5)
#define STM32_IRQ_RCC           (STM32_IRQ_FIRST + 6)
#define STM32_IRQ_EXTI0         (STM32_IRQ_FIRST + 7)
#define STM32_IRQ_EXTI1         (STM32_IRQ_FIRST + 8)
#define STM32_IRQ_EXTI2         (STM32_IRQ_FIRST + 9)
#define STM32_IRQ_EXTI3         (STM32_IRQ_FIRST + 10)
#define STM32_IRQ_EXTI4         (STM32_IRQ_FIRST + 11)
#define STM32_IRQ_EXTI5         (STM32_IRQ_FIRST + 12)
#define STM32_IRQ_EXTI6         (STM32_IRQ_FIRST + 13)
#define STM32_IRQ_EXTI7         (STM32_IRQ_FIRST + 14)
#define STM32_IRQ_EXTI8         (STM32_IRQ_FIRST + 15)
#define STM32_IRQ_EXTI9         (STM32_IRQ_FIRST + 16)
#define STM32_IRQ_EXTI10        (STM32_IRQ_FIRST + 17)
#define STM32_IRQ_EXTI11        (STM32_IRQ_FIRST + 18)
#define STM32_IRQ_EXTI12        (STM32_IRQ_FIRST + 19)
#define STM32_IRQ_EXTI13        (STM32_IRQ_FIRST + 20)
#define STM32_IRQ_EXTI14        (STM32_IRQ_FIRST + 21)
#define STM32_IRQ_EXTI15        (STM32_IRQ_FIRST + 22)
#define STM32_IRQ_LPDMA1_CH0    (STM32_IRQ_FIRST + 23)
#define STM32_IRQ_LPDMA1_CH1    (STM32_IRQ_FIRST + 24)
#define STM32_IRQ_LPDMA1_CH2    (STM32_IRQ_FIRST + 25)
#define STM32_IRQ_LPDMA1_CH3    (STM32_IRQ_FIRST + 26)
#define STM32_IRQ_LPDMA1_CH4    (STM32_IRQ_FIRST + 27)
#define STM32_IRQ_LPDMA1_CH5    (STM32_IRQ_FIRST + 28)
#define STM32_IRQ_LPDMA1_CH6    (STM32_IRQ_FIRST + 29)
#define STM32_IRQ_LPDMA1_CH7    (STM32_IRQ_FIRST + 30)
#define STM32_IRQ_IWDG          (STM32_IRQ_FIRST + 31)
#define STM32_IRQ_ADC1          (STM32_IRQ_FIRST + 32)
#define STM32_IRQ_ADC2          (STM32_IRQ_FIRST + 33)
#define STM32_IRQ_FDCAN1_IT0    (STM32_IRQ_FIRST + 34)
#define STM32_IRQ_FDCAN1_IT1    (STM32_IRQ_FIRST + 35)
#define STM32_IRQ_TIM1_BRK      (STM32_IRQ_FIRST + 36)
#define STM32_IRQ_TIM1_TERR     (STM32_IRQ_FIRST + 36)
#define STM32_IRQ_TIM1_IERR     (STM32_IRQ_FIRST + 36)
#define STM32_IRQ_TIM1_UP       (STM32_IRQ_FIRST + 37)
#define STM32_IRQ_TIM1_TRG_COM  (STM32_IRQ_FIRST + 38)
#define STM32_IRQ_TIM1_DIR      (STM32_IRQ_FIRST + 38)
#define STM32_IRQ_TIM1_IDX      (STM32_IRQ_FIRST + 38)
#define STM32_IRQ_TIM1_CC       (STM32_IRQ_FIRST + 39)
#define STM32_IRQ_TIM2          (STM32_IRQ_FIRST + 40)
#define STM32_IRQ_TIM5          (STM32_IRQ_FIRST + 41)
#define STM32_IRQ_TIM6          (STM32_IRQ_FIRST + 42)
#define STM32_IRQ_TIM7          (STM32_IRQ_FIRST + 43)
#define STM32_IRQ_I2C1_EV       (STM32_IRQ_FIRST + 44)
#define STM32_IRQ_I2C1_ER       (STM32_IRQ_FIRST + 45)
#define STM32_IRQ_I3C1_EV       (STM32_IRQ_FIRST + 46)
#define STM32_IRQ_I3C1_ER       (STM32_IRQ_FIRST + 47)
#define STM32_IRQ_SPI1          (STM32_IRQ_FIRST + 48)
#define STM32_IRQ_SPI2          (STM32_IRQ_FIRST + 49)
#define STM32_IRQ_SPI3          (STM32_IRQ_FIRST + 50)
#define STM32_IRQ_USART1        (STM32_IRQ_FIRST + 51)
#define STM32_IRQ_USART2        (STM32_IRQ_FIRST + 52)
#define STM32_IRQ_USART3        (STM32_IRQ_FIRST + 53)
#define STM32_IRQ_UART4         (STM32_IRQ_FIRST + 54)
#define STM32_IRQ_UART5         (STM32_IRQ_FIRST + 55)
#define STM32_IRQ_LPUART1       (STM32_IRQ_FIRST + 56)
#define STM32_IRQ_LPTIM1        (STM32_IRQ_FIRST + 57)
#define STM32_IRQ_TIM12         (STM32_IRQ_FIRST + 58)
#define STM32_IRQ_TIM15         (STM32_IRQ_FIRST + 59)
#define STM32_IRQ_TIM16         (STM32_IRQ_FIRST + 60)
#define STM32_IRQ_TIM17         (STM32_IRQ_FIRST + 61)
#define STM32_IRQ_USB_DRD_FS    (STM32_IRQ_FIRST + 62)
#define STM32_IRQ_CRS           (STM32_IRQ_FIRST + 63)
#define STM32_IRQ_RNG           (STM32_IRQ_FIRST + 64)
#define STM32_IRQ_FPU           (STM32_IRQ_FIRST + 65)
#define STM32_IRQ_ICACHE        (STM32_IRQ_FIRST + 66)
#define STM32_IRQ_CORDIC        (STM32_IRQ_FIRST + 67)
#define STM32_IRQ_AES           (STM32_IRQ_FIRST + 68)
#define STM32_IRQ_HASH          (STM32_IRQ_FIRST + 69)
#define STM32_IRQ_I2C2_EV       (STM32_IRQ_FIRST + 70)
#define STM32_IRQ_I2C2_ER       (STM32_IRQ_FIRST + 71)
#define STM32_IRQ_TIM8_BRK      (STM32_IRQ_FIRST + 72)
#define STM32_IRQ_TIM8_TERR     (STM32_IRQ_FIRST + 72)
#define STM32_IRQ_TIM8_IERR     (STM32_IRQ_FIRST + 72)
#define STM32_IRQ_TIM8_UP       (STM32_IRQ_FIRST + 73)
#define STM32_IRQ_TIM8_TRG_COM  (STM32_IRQ_FIRST + 74)
#define STM32_IRQ_TIM8_DIR      (STM32_IRQ_FIRST + 74)
#define STM32_IRQ_TIM8_IDX      (STM32_IRQ_FIRST + 74)
#define STM32_IRQ_TIM8_CC       (STM32_IRQ_FIRST + 75)
#define STM32_IRQ_COMP1         (STM32_IRQ_FIRST + 76)
#define STM32_IRQ_DAC1          (STM32_IRQ_FIRST + 77)
#define STM32_IRQ_LPDMA2_CH0    (STM32_IRQ_FIRST + 78)
#define STM32_IRQ_LPDMA2_CH1    (STM32_IRQ_FIRST + 79)
#define STM32_IRQ_LPDMA2_CH2    (STM32_IRQ_FIRST + 80)
#define STM32_IRQ_LPDMA2_CH3    (STM32_IRQ_FIRST + 81)

#define STM32_IRQ_NEXTINTS      82
#define NR_IRQS                 (STM32_IRQ_FIRST + STM32_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_STM32C5_STM32_IRQ_H */
