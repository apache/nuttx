/****************************************************************************
 * arch/arm/include/rtl8720f/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_RTL8720F_IRQ_H
#define __ARCH_ARM_INCLUDE_RTL8720F_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Processor Exceptions (vectors 0-15) */

#define RTL8720F_IRQ_RESERVED      (0)  /* Reserved vector (only used with
                                         * CONFIG_DEBUG_FEATURES) */
                                        /* Vector  1: Reset (not handler) */
#define RTL8720F_IRQ_NMI           (2)  /* Vector  2: Non-Maskable Interrupt */
#define RTL8720F_IRQ_HARDFAULT     (3)  /* Vector  3: Hard fault */
#define RTL8720F_IRQ_MEMFAULT      (4)  /* Vector  4: Memory management (MPU) */
#define RTL8720F_IRQ_BUSFAULT      (5)  /* Vector  5: Bus fault */
#define RTL8720F_IRQ_USAGEFAULT    (6)  /* Vector  6: Usage fault */
#define RTL8720F_IRQ_SECUREFAULT   (7)  /* Vector  7: Secure fault */
#define RTL8720F_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define RTL8720F_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
#define RTL8720F_IRQ_PENDSV        (14) /* Vector 14: Pendable system service */
#define RTL8720F_IRQ_SYSTICK       (15) /* Vector 15: System tick */
#define RTL8720F_IRQ_FIRST         (16) /* Vector number of the first external
                                         * interrupt */

/* External interrupts (vectors >= 16).  These map to the RTL8720F km4tz
 * (AP / host) peripheral interrupt vector numbers (APIRQn, 0..57) as defined
 * by the SDK's component/soc/RTL8720F/fwlib/include/ameba_vector_table.h
 * (CONFIG_ARM_CORE_CM4_KM4TZ branch).
 */

#define RTL8720F_IRQ_WIFI_FISR_FESR    (RTL8720F_IRQ_FIRST + 0)
#define RTL8720F_IRQ_WL_PROTOCOL       (RTL8720F_IRQ_FIRST + 1)
#define RTL8720F_IRQ_AP_WAKE           (RTL8720F_IRQ_FIRST + 2)
#define RTL8720F_IRQ_IPC_KM4           (RTL8720F_IRQ_FIRST + 3)  /* IPC_KM4TZ */
#define RTL8720F_IRQ_IWDG              (RTL8720F_IRQ_FIRST + 4)
#define RTL8720F_IRQ_TIMER0            (RTL8720F_IRQ_FIRST + 5)
#define RTL8720F_IRQ_TIMER1            (RTL8720F_IRQ_FIRST + 6)
#define RTL8720F_IRQ_TIMER2            (RTL8720F_IRQ_FIRST + 7)
#define RTL8720F_IRQ_TIMER3            (RTL8720F_IRQ_FIRST + 8)
#define RTL8720F_IRQ_TIMER4            (RTL8720F_IRQ_FIRST + 9)
#define RTL8720F_IRQ_TIMER5            (RTL8720F_IRQ_FIRST + 10)
#define RTL8720F_IRQ_TIMER6            (RTL8720F_IRQ_FIRST + 11)
#define RTL8720F_IRQ_PMC_TIMER0        (RTL8720F_IRQ_FIRST + 12)
#define RTL8720F_IRQ_PMC_TIMER1        (RTL8720F_IRQ_FIRST + 13)
#define RTL8720F_IRQ_UART0             (RTL8720F_IRQ_FIRST + 14)
#define RTL8720F_IRQ_UART1             (RTL8720F_IRQ_FIRST + 15)
#define RTL8720F_IRQ_UART2             (RTL8720F_IRQ_FIRST + 16)
#define RTL8720F_IRQ_UART_LOG          (RTL8720F_IRQ_FIRST + 17)
#define RTL8720F_IRQ_GPIOA             (RTL8720F_IRQ_FIRST + 18)
#define RTL8720F_IRQ_RSVD              (RTL8720F_IRQ_FIRST + 19)
#define RTL8720F_IRQ_I2C0              (RTL8720F_IRQ_FIRST + 20)
#define RTL8720F_IRQ_I2C1              (RTL8720F_IRQ_FIRST + 21)
#define RTL8720F_IRQ_GDMA0_CH0         (RTL8720F_IRQ_FIRST + 22)
#define RTL8720F_IRQ_GDMA0_CH1         (RTL8720F_IRQ_FIRST + 23)
#define RTL8720F_IRQ_GDMA0_CH2         (RTL8720F_IRQ_FIRST + 24)
#define RTL8720F_IRQ_GDMA0_CH3         (RTL8720F_IRQ_FIRST + 25)
#define RTL8720F_IRQ_GDMA0_CH4         (RTL8720F_IRQ_FIRST + 26)
#define RTL8720F_IRQ_GDMA0_CH5         (RTL8720F_IRQ_FIRST + 27)
#define RTL8720F_IRQ_GDMA0_CH6         (RTL8720F_IRQ_FIRST + 28)
#define RTL8720F_IRQ_GDMA0_CH7         (RTL8720F_IRQ_FIRST + 29)
#define RTL8720F_IRQ_SPI0              (RTL8720F_IRQ_FIRST + 30)
#define RTL8720F_IRQ_SPI1              (RTL8720F_IRQ_FIRST + 31)
#define RTL8720F_IRQ_SPORT             (RTL8720F_IRQ_FIRST + 32)
#define RTL8720F_IRQ_RTC               (RTL8720F_IRQ_FIRST + 33)
#define RTL8720F_IRQ_ADC               (RTL8720F_IRQ_FIRST + 34)
#define RTL8720F_IRQ_BOR               (RTL8720F_IRQ_FIRST + 35)
#define RTL8720F_IRQ_PWR_DOWN          (RTL8720F_IRQ_FIRST + 36)
#define RTL8720F_IRQ_PKE               (RTL8720F_IRQ_FIRST + 37)
#define RTL8720F_IRQ_TRNG              (RTL8720F_IRQ_FIRST + 38)
#define RTL8720F_IRQ_AON_TIM           (RTL8720F_IRQ_FIRST + 39)
#define RTL8720F_IRQ_AON_WAKEPIN       (RTL8720F_IRQ_FIRST + 40)
#define RTL8720F_IRQ_SDIO_WIFI         (RTL8720F_IRQ_FIRST + 41)
#define RTL8720F_IRQ_SDIO_BT           (RTL8720F_IRQ_FIRST + 42)
#define RTL8720F_IRQ_RXI300            (RTL8720F_IRQ_FIRST + 43)
#define RTL8720F_IRQ_PSRAMC            (RTL8720F_IRQ_FIRST + 44)
#define RTL8720F_IRQ_SPI_FLASH         (RTL8720F_IRQ_FIRST + 45)
#define RTL8720F_IRQ_RSIP              (RTL8720F_IRQ_FIRST + 46)
#define RTL8720F_IRQ_AES               (RTL8720F_IRQ_FIRST + 47)
#define RTL8720F_IRQ_SHA               (RTL8720F_IRQ_FIRST + 48)
#define RTL8720F_IRQ_AES_S             (RTL8720F_IRQ_FIRST + 49)
#define RTL8720F_IRQ_SHA_S             (RTL8720F_IRQ_FIRST + 50)
#define RTL8720F_IRQ_KM4NS_WDG_RST     (RTL8720F_IRQ_FIRST + 51)
#define RTL8720F_IRQ_KM4TZ_NS_WDG      (RTL8720F_IRQ_FIRST + 52)
#define RTL8720F_IRQ_KM4TZ_S_WDG       (RTL8720F_IRQ_FIRST + 53)
#define RTL8720F_IRQ_OCP               (RTL8720F_IRQ_FIRST + 54)
#define RTL8720F_IRQ_BT_CTRL_HIGH      (RTL8720F_IRQ_FIRST + 55)
#define RTL8720F_IRQ_BT_CTRL_LOW       (RTL8720F_IRQ_FIRST + 56)
#define RTL8720F_IRQ_IR                (RTL8720F_IRQ_FIRST + 57)

#define RTL8720F_IRQ_NEXTINT       (58) /* Number of external interrupts
                                         * (APIRQn PERI_IRQ_MAX) */

#define NR_IRQS                     (RTL8720F_IRQ_FIRST + RTL8720F_IRQ_NEXTINT)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_RTL8720F_IRQ_H */
