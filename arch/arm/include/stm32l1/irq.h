/****************************************************************************
 * arch/arm/include/stm32l1/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32L1_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32L1_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/stm32l1/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in
 * the IRQ to handle mapping tables.
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
#define STM32_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define STM32_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define STM32_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define STM32_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16).
 * These definitions are chip-specific
 */

#define STM32_IRQ_FIRST         (16) /* Vector number of the first external interrupt */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#if defined(CONFIG_STM32L1_LOWDENSITY) || defined(CONFIG_STM32L1_MEDIUMDENSITY)
#  define STM32_IRQ_WWDG        (STM32_IRQ_FIRST + 0)  /* 0:  Window Watchdog interrupt */
#  define STM32_IRQ_PVD         (STM32_IRQ_FIRST + 1)  /* 1:  PVD through EXTI Line detection interrupt */
#  define STM32_IRQ_TAMPER      (STM32_IRQ_FIRST + 2)  /* 2:  Tamper through EXTI line interrupt, or */
#  define STM32_IRQ_TIMESTAMP   (STM32_IRQ_FIRST + 2)  /* 2:  Time stamp through EXTI line interrupt */
#  define STM32_IRQ_RTC_WKUP    (STM32_IRQ_FIRST + 3)  /* 3:  RTC Wakeup through EXTI line interrupt */
#  define STM32_IRQ_FLASH       (STM32_IRQ_FIRST + 4)  /* 4:  Flash global interrupt */
#  define STM32_IRQ_RCC         (STM32_IRQ_FIRST + 5)  /* 5:  RCC global interrupt */
#  define STM32_IRQ_EXTI0       (STM32_IRQ_FIRST + 6)  /* 6:  EXTI Line 0 interrupt */
#  define STM32_IRQ_EXTI1       (STM32_IRQ_FIRST + 7)  /* 7:  EXTI Line 1 interrupt */
#  define STM32_IRQ_EXTI2       (STM32_IRQ_FIRST + 8)  /* 8:  EXTI Line 2 interrupt */
#  define STM32_IRQ_EXTI3       (STM32_IRQ_FIRST + 9)  /* 9:  EXTI Line 3 interrupt */
#  define STM32_IRQ_EXTI4       (STM32_IRQ_FIRST + 10) /* 10: EXTI Line 4 interrupt */
#  define STM32_IRQ_DMA1CH1     (STM32_IRQ_FIRST + 11) /* 11: DMA1 channel 1 global interrupt */
#  define STM32_IRQ_DMA1CH2     (STM32_IRQ_FIRST + 12) /* 12: DMA1 channel 2 global interrupt */
#  define STM32_IRQ_DMA1CH3     (STM32_IRQ_FIRST + 13) /* 13: DMA1 channel 3 global interrupt */
#  define STM32_IRQ_DMA1CH4     (STM32_IRQ_FIRST + 14) /* 14: DMA1 channel 4 global interrupt */
#  define STM32_IRQ_DMA1CH5     (STM32_IRQ_FIRST + 15) /* 15: DMA1 channel 5 global interrupt */
#  define STM32_IRQ_DMA1CH6     (STM32_IRQ_FIRST + 16) /* 16: DMA1 channel 6 global interrupt */
#  define STM32_IRQ_DMA1CH7     (STM32_IRQ_FIRST + 17) /* 17: DMA1 channel 7 global interrupt */
#  define STM32_IRQ_ADC1        (STM32_IRQ_FIRST + 18) /* 18: ADC1 global interrupt */
#  define STM32_IRQ_USBHP       (STM32_IRQ_FIRST + 19) /* 19: USB High Priority interrupts */
#  define STM32_IRQ_USBLP       (STM32_IRQ_FIRST + 20) /* 20: USB Low Priority interrupt */
#  define STM32_IRQ_DAC         (STM32_IRQ_FIRST + 21) /* 21: DAC interrupt */
#  define STM32_IRQ_COMP        (STM32_IRQ_FIRST + 22) /* 22: Comparator wakeup through EXTI interrupt */
#  define STM32_IRQ_EXTI95      (STM32_IRQ_FIRST + 23) /* 23: EXTI Line[9:5] interrupts */
#  define STM32_IRQ_LDC         (STM32_IRQ_FIRST + 24) /* 24: LCD global interrupt */
#  define STM32_IRQ_TIM9        (STM32_IRQ_FIRST + 25) /* 25: TIM9 global interrupt */
#  define STM32_IRQ_TIM10       (STM32_IRQ_FIRST + 26) /* 26: TIM10 global interrupt */
#  define STM32_IRQ_TIM11       (STM32_IRQ_FIRST + 27) /* 27: TIM11 global interrupt */
#  define STM32_IRQ_TIM2        (STM32_IRQ_FIRST + 28) /* 28: TIM2 global interrupt */
#  define STM32_IRQ_TIM3        (STM32_IRQ_FIRST + 29) /* 29: TIM3 global interrupt */
#  define STM32_IRQ_TIM4        (STM32_IRQ_FIRST + 30) /* 30: TIM4 global interrupt */
#  define STM32_IRQ_I2C1EV      (STM32_IRQ_FIRST + 31) /* 31: I2C1 event interrupt */
#  define STM32_IRQ_I2C1ER      (STM32_IRQ_FIRST + 32) /* 32: I2C1 error interrupt */
#  define STM32_IRQ_I2C2EV      (STM32_IRQ_FIRST + 33) /* 33: I2C2 event interrupt */
#  define STM32_IRQ_I2C2ER      (STM32_IRQ_FIRST + 34) /* 34: I2C2 error interrupt */
#  define STM32_IRQ_SPI1        (STM32_IRQ_FIRST + 35) /* 35: SPI1 global interrupt */
#  define STM32_IRQ_SPI2        (STM32_IRQ_FIRST + 36) /* 36: SPI2 global interrupt */
#  define STM32_IRQ_USART1      (STM32_IRQ_FIRST + 37) /* 37: USART1 global interrupt */
#  define STM32_IRQ_USART2      (STM32_IRQ_FIRST + 38) /* 38: USART2 global interrupt */
#  define STM32_IRQ_USART3      (STM32_IRQ_FIRST + 39) /* 39: USART3 global interrupt */
#  define STM32_IRQ_EXTI1510    (STM32_IRQ_FIRST + 40) /* 40: EXTI Line[15:10] interrupts */
#  define STM32_IRQ_RTCALRM     (STM32_IRQ_FIRST + 41) /* 41: RTC alarm through EXTI line interrupt */
#  define STM32_IRQ_USBWKUP     (STM32_IRQ_FIRST + 42) /* 42: USB wakeup from suspend through EXTI line interrupt */
#  define STM32_IRQ_TIM6        (STM32_IRQ_FIRST + 43) /* 43: TIM6 global interrupt */
#  define STM32_IRQ_TIM7        (STM32_IRQ_FIRST + 44) /* 44: TIM7 global interrupt */

#  define STM32_IRQ_NEXTINTS    (45)

#elif defined(CONFIG_STM32_MEDIUMPLUSDENSITY)
#  define STM32_IRQ_WWDG        (STM32_IRQ_FIRST + 0)  /* 0:  Window Watchdog interrupt */
#  define STM32_IRQ_PVD         (STM32_IRQ_FIRST + 1)  /* 1:  PVD through EXTI Line detection interrupt */
#  define STM32_IRQ_TAMPER      (STM32_IRQ_FIRST + 2)  /* 2:  Tamper through EXTI line interrupt, or */
#  define STM32_IRQ_TIMESTAMP   (STM32_IRQ_FIRST + 2)  /* 2:  Time stamp through EXTI line interrupt */
#  define STM32_IRQ_RTC_WKUP    (STM32_IRQ_FIRST + 3)  /* 3:  RTC Wakeup through EXTI line interrupt */
#  define STM32_IRQ_FLASH       (STM32_IRQ_FIRST + 4)  /* 4:  Flash global interrupt */
#  define STM32_IRQ_RCC         (STM32_IRQ_FIRST + 5)  /* 5:  RCC global interrupt */
#  define STM32_IRQ_EXTI0       (STM32_IRQ_FIRST + 6)  /* 6:  EXTI Line 0 interrupt */
#  define STM32_IRQ_EXTI1       (STM32_IRQ_FIRST + 7)  /* 7:  EXTI Line 1 interrupt */
#  define STM32_IRQ_EXTI2       (STM32_IRQ_FIRST + 8)  /* 8:  EXTI Line 2 interrupt */
#  define STM32_IRQ_EXTI3       (STM32_IRQ_FIRST + 9)  /* 9:  EXTI Line 3 interrupt */
#  define STM32_IRQ_EXTI4       (STM32_IRQ_FIRST + 10) /* 10: EXTI Line 4 interrupt */
#  define STM32_IRQ_DMA1CH1     (STM32_IRQ_FIRST + 11) /* 11: DMA1 channel 1 global interrupt */
#  define STM32_IRQ_DMA1CH2     (STM32_IRQ_FIRST + 12) /* 12: DMA1 channel 2 global interrupt */
#  define STM32_IRQ_DMA1CH3     (STM32_IRQ_FIRST + 13) /* 13: DMA1 channel 3 global interrupt */
#  define STM32_IRQ_DMA1CH4     (STM32_IRQ_FIRST + 14) /* 14: DMA1 channel 4 global interrupt */
#  define STM32_IRQ_DMA1CH5     (STM32_IRQ_FIRST + 15) /* 15: DMA1 channel 5 global interrupt */
#  define STM32_IRQ_DMA1CH6     (STM32_IRQ_FIRST + 16) /* 16: DMA1 channel 6 global interrupt */
#  define STM32_IRQ_DMA1CH7     (STM32_IRQ_FIRST + 17) /* 17: DMA1 channel 7 global interrupt */
#  define STM32_IRQ_ADC1        (STM32_IRQ_FIRST + 18) /* 18: ADC1 global interrupt */
#  define STM32_IRQ_USBHP       (STM32_IRQ_FIRST + 19) /* 19: USB High Priority interrupts */
#  define STM32_IRQ_USBLP       (STM32_IRQ_FIRST + 20) /* 20: USB Low Priority interrupt */
#  define STM32_IRQ_DAC         (STM32_IRQ_FIRST + 21) /* 21: DAC interrupt */
#  define STM32_IRQ_COMP        (STM32_IRQ_FIRST + 22) /* 22: Comparator wakeup through EXTI interrupt, or */
#  define STM32_IRQ_CA          (STM32_IRQ_FIRST + 22) /* 22: Channel acquisition interrupt */
#  define STM32_IRQ_EXTI95      (STM32_IRQ_FIRST + 23) /* 23: EXTI Line[9:5] interrupts */
#  define STM32_IRQ_LDC         (STM32_IRQ_FIRST + 24) /* 24: LCD global interrupt */
#  define STM32_IRQ_TIM9        (STM32_IRQ_FIRST + 25) /* 25: TIM9 global interrupt */
#  define STM32_IRQ_TIM10       (STM32_IRQ_FIRST + 26) /* 26: TIM10 global interrupt */
#  define STM32_IRQ_TIM11       (STM32_IRQ_FIRST + 27) /* 27: TIM11 global interrupt */
#  define STM32_IRQ_TIM2        (STM32_IRQ_FIRST + 28) /* 28: TIM2 global interrupt */
#  define STM32_IRQ_TIM3        (STM32_IRQ_FIRST + 29) /* 29: TIM3 global interrupt */
#  define STM32_IRQ_TIM4        (STM32_IRQ_FIRST + 30) /* 30: TIM4 global interrupt */
#  define STM32_IRQ_I2C1EV      (STM32_IRQ_FIRST + 31) /* 31: I2C1 event interrupt */
#  define STM32_IRQ_I2C1ER      (STM32_IRQ_FIRST + 32) /* 32: I2C1 error interrupt */
#  define STM32_IRQ_I2C2EV      (STM32_IRQ_FIRST + 33) /* 33: I2C2 event interrupt */
#  define STM32_IRQ_I2C2ER      (STM32_IRQ_FIRST + 34) /* 34: I2C2 error interrupt */
#  define STM32_IRQ_SPI1        (STM32_IRQ_FIRST + 35) /* 35: SPI1 global interrupt */
#  define STM32_IRQ_SPI2        (STM32_IRQ_FIRST + 36) /* 36: SPI2 global interrupt */
#  define STM32_IRQ_USART1      (STM32_IRQ_FIRST + 37) /* 37: USART1 global interrupt */
#  define STM32_IRQ_USART2      (STM32_IRQ_FIRST + 38) /* 38: USART2 global interrupt */
#  define STM32_IRQ_USART3      (STM32_IRQ_FIRST + 39) /* 39: USART3 global interrupt */
#  define STM32_IRQ_EXTI1510    (STM32_IRQ_FIRST + 40) /* 40: EXTI Line[15:10] interrupts */
#  define STM32_IRQ_RTCALRM     (STM32_IRQ_FIRST + 41) /* 41: RTC alarm through EXTI line interrupt */
#  define STM32_IRQ_USBWKUP     (STM32_IRQ_FIRST + 42) /* 42: USB wakeup from suspend through EXTI line interrupt */
#  define STM32_IRQ_TIM6        (STM32_IRQ_FIRST + 43) /* 43: TIM6 global interrupt */
#  define STM32_IRQ_TIM7        (STM32_IRQ_FIRST + 44) /* 44: TIM7 global interrupt */
#  define STM32_IRQ_TIM5        (STM32_IRQ_FIRST + 45) /* 45: TIM5 global interrupt */
#  define STM32_IRQ_SPI3        (STM32_IRQ_FIRST + 46) /* 46: SPI3 global interrupt */
#  define STM32_IRQ_DMA2CH1     (STM32_IRQ_FIRST + 47) /* 47: DMA2 channel 1 global interrupt */
#  define STM32_IRQ_DMA2CH2     (STM32_IRQ_FIRST + 48) /* 48: DMA2 channel 2 global interrupt */
#  define STM32_IRQ_DMA2CH3     (STM32_IRQ_FIRST + 49) /* 49: DMA2 channel 3 global interrupt */
#  define STM32_IRQ_DMA2CH4     (STM32_IRQ_FIRST + 50) /* 50: DMA2 channel 4 global interrupt */
#  define STM32_IRQ_DMA2CH5     (STM32_IRQ_FIRST + 51) /* 51: DMA2 channel 5 global interrupt */
#  define STM32_IRQ_AES         (STM32_IRQ_FIRST + 52) /* 52: AES global interrupt */
#  define STM32_IRQ_COMPACQ     (STM32_IRQ_FIRST + 53) /* 53: Comparator Channel Acquisition Interrupt */

#  define STM32_IRQ_NEXTINTS    (54)

#elif defined(CONFIG_STM32L1_HIGHDENSITY)
#  define STM32_IRQ_WWDG        (STM32_IRQ_FIRST + 0)  /* 0:  Window Watchdog interrupt */
#  define STM32_IRQ_PVD         (STM32_IRQ_FIRST + 1)  /* 1:  PVD through EXTI Line detection interrupt */
#  define STM32_IRQ_TAMPER      (STM32_IRQ_FIRST + 2)  /* 2:  Tamper through EXTI line interrupt, or */
#  define STM32_IRQ_TIMESTAMP   (STM32_IRQ_FIRST + 2)  /* 2:  Time stamp through EXTI line interrupt */
#  define STM32_IRQ_RTC_WKUP    (STM32_IRQ_FIRST + 3)  /* 3:  RTC Wakeup through EXTI line interrupt */
#  define STM32_IRQ_FLASH       (STM32_IRQ_FIRST + 4)  /* 4:  Flash global interrupt */
#  define STM32_IRQ_RCC         (STM32_IRQ_FIRST + 5)  /* 5:  RCC global interrupt */
#  define STM32_IRQ_EXTI0       (STM32_IRQ_FIRST + 6)  /* 6:  EXTI Line 0 interrupt */
#  define STM32_IRQ_EXTI1       (STM32_IRQ_FIRST + 7)  /* 7:  EXTI Line 1 interrupt */
#  define STM32_IRQ_EXTI2       (STM32_IRQ_FIRST + 8)  /* 8:  EXTI Line 2 interrupt */
#  define STM32_IRQ_EXTI3       (STM32_IRQ_FIRST + 9)  /* 9:  EXTI Line 3 interrupt */
#  define STM32_IRQ_EXTI4       (STM32_IRQ_FIRST + 10) /* 10: EXTI Line 4 interrupt */
#  define STM32_IRQ_DMA1CH1     (STM32_IRQ_FIRST + 11) /* 11: DMA1 channel 1 global interrupt */
#  define STM32_IRQ_DMA1CH2     (STM32_IRQ_FIRST + 12) /* 12: DMA1 channel 2 global interrupt */
#  define STM32_IRQ_DMA1CH3     (STM32_IRQ_FIRST + 13) /* 13: DMA1 channel 3 global interrupt */
#  define STM32_IRQ_DMA1CH4     (STM32_IRQ_FIRST + 14) /* 14: DMA1 channel 4 global interrupt */
#  define STM32_IRQ_DMA1CH5     (STM32_IRQ_FIRST + 15) /* 15: DMA1 channel 5 global interrupt */
#  define STM32_IRQ_DMA1CH6     (STM32_IRQ_FIRST + 16) /* 16: DMA1 channel 6 global interrupt */
#  define STM32_IRQ_DMA1CH7     (STM32_IRQ_FIRST + 17) /* 17: DMA1 channel 7 global interrupt */
#  define STM32_IRQ_ADC1        (STM32_IRQ_FIRST + 18) /* 18: ADC1 global interrupt */
#  define STM32_IRQ_USBHP       (STM32_IRQ_FIRST + 19) /* 19: USB High Priority interrupts */
#  define STM32_IRQ_USBLP       (STM32_IRQ_FIRST + 20) /* 20: USB Low Priority interrupt */
#  define STM32_IRQ_DAC         (STM32_IRQ_FIRST + 21) /* 21: DAC interrupt */
#  define STM32_IRQ_COMP        (STM32_IRQ_FIRST + 22) /* 22: Comparator wakeup through EXTI interrupt, or */
#  define STM32_IRQ_CA          (STM32_IRQ_FIRST + 22) /* 22: Channel acquisition interrupt */
#  define STM32_IRQ_EXTI95      (STM32_IRQ_FIRST + 23) /* 23: EXTI Line[9:5] interrupts */
#  define STM32_IRQ_LDC         (STM32_IRQ_FIRST + 24) /* 24: LCD global interrupt */
#  define STM32_IRQ_TIM9        (STM32_IRQ_FIRST + 25) /* 25: TIM9 global interrupt */
#  define STM32_IRQ_TIM10       (STM32_IRQ_FIRST + 26) /* 26: TIM10 global interrupt */
#  define STM32_IRQ_TIM11       (STM32_IRQ_FIRST + 27) /* 27: TIM11 global interrupt */
#  define STM32_IRQ_TIM2        (STM32_IRQ_FIRST + 28) /* 28: TIM2 global interrupt */
#  define STM32_IRQ_TIM3        (STM32_IRQ_FIRST + 29) /* 29: TIM3 global interrupt */
#  define STM32_IRQ_TIM4        (STM32_IRQ_FIRST + 30) /* 30: TIM4 global interrupt */
#  define STM32_IRQ_I2C1EV      (STM32_IRQ_FIRST + 31) /* 31: I2C1 event interrupt */
#  define STM32_IRQ_I2C1ER      (STM32_IRQ_FIRST + 32) /* 32: I2C1 error interrupt */
#  define STM32_IRQ_I2C2EV      (STM32_IRQ_FIRST + 33) /* 33: I2C2 event interrupt */
#  define STM32_IRQ_I2C2ER      (STM32_IRQ_FIRST + 34) /* 34: I2C2 error interrupt */
#  define STM32_IRQ_SPI1        (STM32_IRQ_FIRST + 35) /* 35: SPI1 global interrupt */
#  define STM32_IRQ_SPI2        (STM32_IRQ_FIRST + 36) /* 36: SPI2 global interrupt */
#  define STM32_IRQ_USART1      (STM32_IRQ_FIRST + 37) /* 37: USART1 global interrupt */
#  define STM32_IRQ_USART2      (STM32_IRQ_FIRST + 38) /* 38: USART2 global interrupt */
#  define STM32_IRQ_USART3      (STM32_IRQ_FIRST + 39) /* 39: USART3 global interrupt */
#  define STM32_IRQ_EXTI1510    (STM32_IRQ_FIRST + 40) /* 40: EXTI Line[15:10] interrupts */
#  define STM32_IRQ_RTCALRM     (STM32_IRQ_FIRST + 41) /* 41: RTC alarm through EXTI line interrupt */
#  define STM32_IRQ_USBWKUP     (STM32_IRQ_FIRST + 42) /* 42: USB wakeup from suspend through EXTI line interrupt */
#  define STM32_IRQ_TIM6        (STM32_IRQ_FIRST + 43) /* 43: TIM6 global interrupt */
#  define STM32_IRQ_TIM7        (STM32_IRQ_FIRST + 44) /* 44: TIM7 global interrupt */
#  define STM32_IRQ_SDIO        (STM32_IRQ_FIRST + 45) /* 45: SDIO Global interrupt */
#  define STM32_IRQ_TIM5        (STM32_IRQ_FIRST + 46) /* 46: TIM5 global interrupt */
#  define STM32_IRQ_SPI3        (STM32_IRQ_FIRST + 47) /* 47: SPI3 global interrupt */
#  define STM32_IRQ_UART4       (STM32_IRQ_FIRST + 48) /* 48: UART4 global interrupt */
#  define STM32_IRQ_UART5       (STM32_IRQ_FIRST + 49) /* 49: UART5 global interrupt */
#  define STM32_IRQ_DMA2CH1     (STM32_IRQ_FIRST + 50) /* 50: DMA2 channel 1 global interrupt */
#  define STM32_IRQ_DMA2CH2     (STM32_IRQ_FIRST + 51) /* 51: DMA2 channel 2 global interrupt */
#  define STM32_IRQ_DMA2CH3     (STM32_IRQ_FIRST + 52) /* 52: DMA2 channel 3 global interrupt */
#  define STM32_IRQ_DMA2CH4     (STM32_IRQ_FIRST + 53) /* 53: DMA2 channel 4 global interrupt */
#  define STM32_IRQ_DMA2CH5     (STM32_IRQ_FIRST + 54) /* 54: DMA2 channel 5 global interrupt */
#  define STM32_IRQ_AES         (STM32_IRQ_FIRST + 55) /* 55: AES global interrupt */
#  define STM32_IRQ_COMPACQ     (STM32_IRQ_FIRST + 56) /* 56: Comparator Channel Acquisition Interrupt */

#  define STM32_IRQ_NEXTINTS    (57)
#else
#  error "Unknown STM32L density"
#endif

#  define NR_IRQS               (STM32_IRQ_FIRST + STM32_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_STM32L1_IRQ_H */
