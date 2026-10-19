/****************************************************************************
 * arch/arm/include/gd32w51x/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_GD32W51X_IRQ_H
#define __ARCH_ARM_INCLUDE_GD32W51X_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds to the vector number and hence
 * maps directly to bits in the NVIC.
 */

/* Processor exceptions (vectors 0-15) */

#define GD32W51X_IRQ_RESERVED       (0)  /* Reset stack pointer value */
                                         /* Vector  1: Reset (not an IRQ) */
#define GD32W51X_IRQ_NMI            (2)  /* Non-maskable interrupt */
#define GD32W51X_IRQ_HARDFAULT      (3)  /* Hard fault */
#define GD32W51X_IRQ_MEMFAULT       (4)  /* Memory management (MPU) */
#define GD32W51X_IRQ_BUSFAULT       (5)  /* Bus fault */
#define GD32W51X_IRQ_USAGEFAULT     (6)  /* Usage fault */
#define GD32W51X_IRQ_SECUREFAULT    (7)  /* Secure fault */
                                         /* Vectors 8-10: Reserved */
#define GD32W51X_IRQ_SVCALL         (11) /* SVC call */
#define GD32W51X_IRQ_DBGMONITOR     (12) /* Debug monitor */
                                         /* Vector 13: Reserved */
#define GD32W51X_IRQ_PENDSV         (14) /* Pendable system service request */
#define GD32W51X_IRQ_SYSTICK        (15) /* System tick */

/* External interrupts (vectors >= 16).  The offsets are the IRQn_Type
 * values of the GD32W51x firmware library; the gaps are reserved vectors.
 * USART1 and USART2 keep the UART1/UART2 names shared with the GD32VW55x
 * drivers.  DCI and HPDF exist only on the QFN56 parts.
 */

#define GD32W51X_IRQ_EXTINT         (16)

#define GD32W51X_IRQ_WWDGT          (GD32W51X_IRQ_EXTINT + 0)
#define GD32W51X_IRQ_LVD            (GD32W51X_IRQ_EXTINT + 1)
#define GD32W51X_IRQ_TAMPER_STAMP   (GD32W51X_IRQ_EXTINT + 2)
#define GD32W51X_IRQ_RTC_WKUP       (GD32W51X_IRQ_EXTINT + 3)
#define GD32W51X_IRQ_FMC            (GD32W51X_IRQ_EXTINT + 4)
#define GD32W51X_IRQ_RCU            (GD32W51X_IRQ_EXTINT + 5)
#define GD32W51X_IRQ_EXTI0          (GD32W51X_IRQ_EXTINT + 6)
#define GD32W51X_IRQ_EXTI1          (GD32W51X_IRQ_EXTINT + 7)
#define GD32W51X_IRQ_EXTI2          (GD32W51X_IRQ_EXTINT + 8)
#define GD32W51X_IRQ_EXTI3          (GD32W51X_IRQ_EXTINT + 9)
#define GD32W51X_IRQ_EXTI4          (GD32W51X_IRQ_EXTINT + 10)
#define GD32W51X_IRQ_DMA_CH0        (GD32W51X_IRQ_EXTINT + 11)
#define GD32W51X_IRQ_DMA_CH1        (GD32W51X_IRQ_EXTINT + 12)
#define GD32W51X_IRQ_DMA_CH2        (GD32W51X_IRQ_EXTINT + 13)
#define GD32W51X_IRQ_DMA_CH3        (GD32W51X_IRQ_EXTINT + 14)
#define GD32W51X_IRQ_DMA_CH4        (GD32W51X_IRQ_EXTINT + 15)
#define GD32W51X_IRQ_DMA_CH5        (GD32W51X_IRQ_EXTINT + 16)
#define GD32W51X_IRQ_DMA_CH6        (GD32W51X_IRQ_EXTINT + 17)
#define GD32W51X_IRQ_DMA_CH7        (GD32W51X_IRQ_EXTINT + 18)
#define GD32W51X_IRQ_ADC            (GD32W51X_IRQ_EXTINT + 19)
#define GD32W51X_IRQ_TAMPER_STAMP_S (GD32W51X_IRQ_EXTINT + 20)
#define GD32W51X_IRQ_RTC_WKUP_S     (GD32W51X_IRQ_EXTINT + 21)
#define GD32W51X_IRQ_RTC_ALARM_S    (GD32W51X_IRQ_EXTINT + 22)
#define GD32W51X_IRQ_EXTI5_9        (GD32W51X_IRQ_EXTINT + 23)
#define GD32W51X_IRQ_TIMER0_BRK     (GD32W51X_IRQ_EXTINT + 24)
#define GD32W51X_IRQ_TIMER0_UP      (GD32W51X_IRQ_EXTINT + 25)
#define GD32W51X_IRQ_TIMER0_CMT     (GD32W51X_IRQ_EXTINT + 26)
#define GD32W51X_IRQ_TIMER0_CC      (GD32W51X_IRQ_EXTINT + 27)
#define GD32W51X_IRQ_TIMER1         (GD32W51X_IRQ_EXTINT + 28)
#define GD32W51X_IRQ_TIMER2         (GD32W51X_IRQ_EXTINT + 29)
#define GD32W51X_IRQ_TIMER3         (GD32W51X_IRQ_EXTINT + 30)
#define GD32W51X_IRQ_I2C0_EV        (GD32W51X_IRQ_EXTINT + 31)
#define GD32W51X_IRQ_I2C0_ER        (GD32W51X_IRQ_EXTINT + 32)
#define GD32W51X_IRQ_I2C1_EV        (GD32W51X_IRQ_EXTINT + 33)
#define GD32W51X_IRQ_I2C1_ER        (GD32W51X_IRQ_EXTINT + 34)
#define GD32W51X_IRQ_SPI            (GD32W51X_IRQ_EXTINT + 35)
#define GD32W51X_IRQ_SPI1           (GD32W51X_IRQ_EXTINT + 36)
#define GD32W51X_IRQ_USART0         (GD32W51X_IRQ_EXTINT + 37)
#define GD32W51X_IRQ_UART1          (GD32W51X_IRQ_EXTINT + 38)
#define GD32W51X_IRQ_UART2          (GD32W51X_IRQ_EXTINT + 39)
#define GD32W51X_IRQ_EXTI10_15      (GD32W51X_IRQ_EXTINT + 40)
#define GD32W51X_IRQ_RTC_ALARM      (GD32W51X_IRQ_EXTINT + 41)
#define GD32W51X_IRQ_VLVDF          (GD32W51X_IRQ_EXTINT + 42)
#define GD32W51X_IRQ_TIMER15        (GD32W51X_IRQ_EXTINT + 44)
#define GD32W51X_IRQ_TIMER16        (GD32W51X_IRQ_EXTINT + 45)
#define GD32W51X_IRQ_SDIO           (GD32W51X_IRQ_EXTINT + 49)
#define GD32W51X_IRQ_TIMER4         (GD32W51X_IRQ_EXTINT + 50)
#define GD32W51X_IRQ_I2C0_WKUP      (GD32W51X_IRQ_EXTINT + 51)
#define GD32W51X_IRQ_USART0_WKUP    (GD32W51X_IRQ_EXTINT + 52)
#define GD32W51X_IRQ_UART2_WKUP     (GD32W51X_IRQ_EXTINT + 53)
#define GD32W51X_IRQ_TIMER5         (GD32W51X_IRQ_EXTINT + 54)
#define GD32W51X_IRQ_DMA1_CH0       (GD32W51X_IRQ_EXTINT + 56)
#define GD32W51X_IRQ_DMA1_CH1       (GD32W51X_IRQ_EXTINT + 57)
#define GD32W51X_IRQ_DMA1_CH2       (GD32W51X_IRQ_EXTINT + 58)
#define GD32W51X_IRQ_DMA1_CH3       (GD32W51X_IRQ_EXTINT + 59)
#define GD32W51X_IRQ_DMA1_CH4       (GD32W51X_IRQ_EXTINT + 60)
#define GD32W51X_IRQ_DMA1_CH5       (GD32W51X_IRQ_EXTINT + 61)
#define GD32W51X_IRQ_DMA1_CH6       (GD32W51X_IRQ_EXTINT + 62)
#define GD32W51X_IRQ_DMA1_CH7       (GD32W51X_IRQ_EXTINT + 63)
#define GD32W51X_IRQ_WIFI_WKUP      (GD32W51X_IRQ_EXTINT + 66)
#define GD32W51X_IRQ_USBFS          (GD32W51X_IRQ_EXTINT + 67)
#define GD32W51X_IRQ_USBFS_WKUP     (GD32W51X_IRQ_EXTINT + 76)
#define GD32W51X_IRQ_DCI            (GD32W51X_IRQ_EXTINT + 78)
#define GD32W51X_IRQ_CAU            (GD32W51X_IRQ_EXTINT + 79)
#define GD32W51X_IRQ_HAU_TRNG       (GD32W51X_IRQ_EXTINT + 80)
#define GD32W51X_IRQ_FPU            (GD32W51X_IRQ_EXTINT + 81)
#define GD32W51X_IRQ_HPDF_INT0      (GD32W51X_IRQ_EXTINT + 89)
#define GD32W51X_IRQ_HPDF_INT1      (GD32W51X_IRQ_EXTINT + 90)
#define GD32W51X_IRQ_WIFI_INT0      (GD32W51X_IRQ_EXTINT + 91)
#define GD32W51X_IRQ_WIFI_INT1      (GD32W51X_IRQ_EXTINT + 92)
#define GD32W51X_IRQ_WIFI_INT2      (GD32W51X_IRQ_EXTINT + 93)
#define GD32W51X_IRQ_EFUSE          (GD32W51X_IRQ_EXTINT + 94)
#define GD32W51X_IRQ_QSPI           (GD32W51X_IRQ_EXTINT + 95)
#define GD32W51X_IRQ_PKCAU          (GD32W51X_IRQ_EXTINT + 96)
#define GD32W51X_IRQ_TSI            (GD32W51X_IRQ_EXTINT + 97)
#define GD32W51X_IRQ_ICACHE         (GD32W51X_IRQ_EXTINT + 98)
#define GD32W51X_IRQ_TZIAC_S        (GD32W51X_IRQ_EXTINT + 99)
#define GD32W51X_IRQ_FMC_S          (GD32W51X_IRQ_EXTINT + 100)
#define GD32W51X_IRQ_QSPI_S         (GD32W51X_IRQ_EXTINT + 101)

#define GD32W51X_IRQ_NEXTINT        102
#define NR_IRQS \
  (GD32W51X_IRQ_EXTINT + GD32W51X_IRQ_NEXTINT)

#endif /* __ARCH_ARM_INCLUDE_GD32W51X_IRQ_H */
