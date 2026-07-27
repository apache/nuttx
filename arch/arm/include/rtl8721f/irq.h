/****************************************************************************
 * arch/arm/include/rtl8721f/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_RTL8721F_IRQ_H
#define __ARCH_ARM_INCLUDE_RTL8721F_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Processor Exceptions (vectors 0-15) */

#define RTL8721F_IRQ_RESERVED      (0)  /* Reserved vector (only used with
                                         * CONFIG_DEBUG_FEATURES) */
                                        /* Vector  1: Reset (not handler) */
#define RTL8721F_IRQ_NMI           (2)  /* Vector  2: Non-Maskable Interrupt */
#define RTL8721F_IRQ_HARDFAULT     (3)  /* Vector  3: Hard fault */
#define RTL8721F_IRQ_MEMFAULT      (4)  /* Vector  4: Memory management (MPU) */
#define RTL8721F_IRQ_BUSFAULT      (5)  /* Vector  5: Bus fault */
#define RTL8721F_IRQ_USAGEFAULT    (6)  /* Vector  6: Usage fault */
#define RTL8721F_IRQ_SECUREFAULT   (7)  /* Vector  7: Secure fault */
#define RTL8721F_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define RTL8721F_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
#define RTL8721F_IRQ_PENDSV        (14) /* Vector 14: Pendable system service */
#define RTL8721F_IRQ_SYSTICK       (15) /* Vector 15: System tick */
#define RTL8721F_IRQ_FIRST         (16) /* Vector number of the first external
                                         * interrupt */

/* External interrupts (vectors >= 16).  These map to the amebagreen2 km4tz
 * (AP / host) peripheral interrupt vector numbers (APIRQn, 0..78) as defined
 * by the SDK's
 * component/soc/amebagreen2/fwlib/include/ameba_vector_table.h
 * (CONFIG_ARM_CORE_CM4_KM4TZ branch).  NOTE: this numbering differs from the
 * RTL8721Dx / RTL8720F -- e.g. UART_LOG is 24 (not 17) and IPC is 5 (not 3).
 */

#define RTL8721F_IRQ_WIFI_FISR_FESR    (RTL8721F_IRQ_FIRST + 0)
#define RTL8721F_IRQ_WIFI_FTSR_MBOX    (RTL8721F_IRQ_FIRST + 1)
#define RTL8721F_IRQ_WL_DMA            (RTL8721F_IRQ_FIRST + 2)
#define RTL8721F_IRQ_WL_PROTOCOL       (RTL8721F_IRQ_FIRST + 3)
#define RTL8721F_IRQ_AP_WAKE           (RTL8721F_IRQ_FIRST + 4)
#define RTL8721F_IRQ_IPC_KM4           (RTL8721F_IRQ_FIRST + 5)  /* IPC_CPU0 */
#define RTL8721F_IRQ_IWDG              (RTL8721F_IRQ_FIRST + 6)
#define RTL8721F_IRQ_TIMER0            (RTL8721F_IRQ_FIRST + 7)
#define RTL8721F_IRQ_TIMER1            (RTL8721F_IRQ_FIRST + 8)
#define RTL8721F_IRQ_TIMER2            (RTL8721F_IRQ_FIRST + 9)
#define RTL8721F_IRQ_TIMER3            (RTL8721F_IRQ_FIRST + 10)
#define RTL8721F_IRQ_TIMER4            (RTL8721F_IRQ_FIRST + 11)
#define RTL8721F_IRQ_TIMER5            (RTL8721F_IRQ_FIRST + 12)
#define RTL8721F_IRQ_TIMER6            (RTL8721F_IRQ_FIRST + 13)
#define RTL8721F_IRQ_TIMER7            (RTL8721F_IRQ_FIRST + 14)
#define RTL8721F_IRQ_TIMER8            (RTL8721F_IRQ_FIRST + 15)
#define RTL8721F_IRQ_COEX_MAILBOX      (RTL8721F_IRQ_FIRST + 16)
#define RTL8721F_IRQ_RSVD              (RTL8721F_IRQ_FIRST + 17)
#define RTL8721F_IRQ_PMC_TIMER0        (RTL8721F_IRQ_FIRST + 18)
#define RTL8721F_IRQ_PMC_TIMER1        (RTL8721F_IRQ_FIRST + 19)
#define RTL8721F_IRQ_UART0             (RTL8721F_IRQ_FIRST + 20)
#define RTL8721F_IRQ_UART1             (RTL8721F_IRQ_FIRST + 21)
#define RTL8721F_IRQ_UART2             (RTL8721F_IRQ_FIRST + 22)
#define RTL8721F_IRQ_UART3             (RTL8721F_IRQ_FIRST + 23)
#define RTL8721F_IRQ_UART_LOG          (RTL8721F_IRQ_FIRST + 24)
#define RTL8721F_IRQ_GPIOA             (RTL8721F_IRQ_FIRST + 25)
#define RTL8721F_IRQ_GPIOB             (RTL8721F_IRQ_FIRST + 26)
#define RTL8721F_IRQ_GPIOC             (RTL8721F_IRQ_FIRST + 27)
#define RTL8721F_IRQ_I2C0              (RTL8721F_IRQ_FIRST + 28)
#define RTL8721F_IRQ_I2C1              (RTL8721F_IRQ_FIRST + 29)
#define RTL8721F_IRQ_GDMA0_CH0         (RTL8721F_IRQ_FIRST + 30)
#define RTL8721F_IRQ_GDMA0_CH1         (RTL8721F_IRQ_FIRST + 31)
#define RTL8721F_IRQ_GDMA0_CH2         (RTL8721F_IRQ_FIRST + 32)
#define RTL8721F_IRQ_GDMA0_CH3         (RTL8721F_IRQ_FIRST + 33)
#define RTL8721F_IRQ_GDMA0_CH4         (RTL8721F_IRQ_FIRST + 34)
#define RTL8721F_IRQ_GDMA0_CH5         (RTL8721F_IRQ_FIRST + 35)
#define RTL8721F_IRQ_GDMA0_CH6         (RTL8721F_IRQ_FIRST + 36)
#define RTL8721F_IRQ_GDMA0_CH7         (RTL8721F_IRQ_FIRST + 37)
#define RTL8721F_IRQ_SPI0              (RTL8721F_IRQ_FIRST + 38)
#define RTL8721F_IRQ_SPI1              (RTL8721F_IRQ_FIRST + 39)
#define RTL8721F_IRQ_SPORT0            (RTL8721F_IRQ_FIRST + 40)
#define RTL8721F_IRQ_RTC               (RTL8721F_IRQ_FIRST + 41)
#define RTL8721F_IRQ_ADC               (RTL8721F_IRQ_FIRST + 42)
#define RTL8721F_IRQ_ADC_COMP          (RTL8721F_IRQ_FIRST + 43)
#define RTL8721F_IRQ_CAPTOUCH          (RTL8721F_IRQ_FIRST + 44)
#define RTL8721F_IRQ_THERMAL           (RTL8721F_IRQ_FIRST + 45)
#define RTL8721F_IRQ_BOR               (RTL8721F_IRQ_FIRST + 46)
#define RTL8721F_IRQ_PWR_DOWN          (RTL8721F_IRQ_FIRST + 47)
#define RTL8721F_IRQ_RMII              (RTL8721F_IRQ_FIRST + 48)
#define RTL8721F_IRQ_LCDC              (RTL8721F_IRQ_FIRST + 49)
#define RTL8721F_IRQ_MJPEG             (RTL8721F_IRQ_FIRST + 50)
#define RTL8721F_IRQ_PPE               (RTL8721F_IRQ_FIRST + 51)
#define RTL8721F_IRQ_PKE               (RTL8721F_IRQ_FIRST + 52)
#define RTL8721F_IRQ_TRNG              (RTL8721F_IRQ_FIRST + 53)
#define RTL8721F_IRQ_AON_TIM           (RTL8721F_IRQ_FIRST + 54)
#define RTL8721F_IRQ_AON_WAKEPIN       (RTL8721F_IRQ_FIRST + 55)
#define RTL8721F_IRQ_SDIO_WIFI         (RTL8721F_IRQ_FIRST + 56)
#define RTL8721F_IRQ_SDIO_BT           (RTL8721F_IRQ_FIRST + 57)
#define RTL8721F_IRQ_SDIO_HOST         (RTL8721F_IRQ_FIRST + 58)
#define RTL8721F_IRQ_USB               (RTL8721F_IRQ_FIRST + 59)
#define RTL8721F_IRQ_CAN0              (RTL8721F_IRQ_FIRST + 60)
#define RTL8721F_IRQ_CAN1              (RTL8721F_IRQ_FIRST + 61)
#define RTL8721F_IRQ_IR                (RTL8721F_IRQ_FIRST + 62)
#define RTL8721F_IRQ_RXI300            (RTL8721F_IRQ_FIRST + 63)
#define RTL8721F_IRQ_PSRAMC            (RTL8721F_IRQ_FIRST + 64)
#define RTL8721F_IRQ_SPI_FLASH         (RTL8721F_IRQ_FIRST + 65)
#define RTL8721F_IRQ_RSIP              (RTL8721F_IRQ_FIRST + 66)
#define RTL8721F_IRQ_AES               (RTL8721F_IRQ_FIRST + 67)
#define RTL8721F_IRQ_SHA               (RTL8721F_IRQ_FIRST + 68)
#define RTL8721F_IRQ_CPU0_NS_WDG       (RTL8721F_IRQ_FIRST + 69)
#define RTL8721F_IRQ_CPU0_S_WDG        (RTL8721F_IRQ_FIRST + 70)
#define RTL8721F_IRQ_OCP               (RTL8721F_IRQ_FIRST + 71)
#define RTL8721F_IRQ_SPIC_ECC          (RTL8721F_IRQ_FIRST + 72)
#define RTL8721F_IRQ_UVC_DEC           (RTL8721F_IRQ_FIRST + 73)
#define RTL8721F_IRQ_RTC_DET           (RTL8721F_IRQ_FIRST + 74)
#define RTL8721F_IRQ_BT_MAILBOX        (RTL8721F_IRQ_FIRST + 75)
#define RTL8721F_IRQ_BT_SCB            (RTL8721F_IRQ_FIRST + 76)
#define RTL8721F_IRQ_BT_WAKE_HOST      (RTL8721F_IRQ_FIRST + 77)
#define RTL8721F_IRQ_CPU1_WDG_RST      (RTL8721F_IRQ_FIRST + 78)

#define RTL8721F_IRQ_NEXTINT       (79) /* Number of external interrupts
                                         * (APIRQn count, 0..78) */

#define NR_IRQS                     (RTL8721F_IRQ_FIRST + RTL8721F_IRQ_NEXTINT)

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

#endif /* __ARCH_ARM_INCLUDE_RTL8721F_IRQ_H */
