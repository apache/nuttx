/****************************************************************************
 * arch/arm/include/rtl8730e/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_RTL8730E_IRQ_H
#define __ARCH_ARM_INCLUDE_RTL8730E_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define NR_IRQS                 220  /* Total number of interrupts */

/* GIC SPI interrupts: NuttX IRQ = GIC SPI number + 32.
 * Numbers taken from SDK ameba_vector_table.h (CA32 section).
 */

#define RTL8730E_IRQ_TIMER2     34   /* GIC SPI 2:  TIM2 (LP basic timer) */
#define RTL8730E_IRQ_TIMER3     35   /* GIC SPI 3:  TIM3 (LP basic timer) */
#define RTL8730E_IRQ_RTC        44   /* GIC SPI 12: RTC */
#define RTL8730E_IRQ_ADC        46   /* GIC SPI 14: ADC */
#define RTL8730E_IRQ_IPC_AP     56   /* GIC SPI 24: CA32 AP-IPC (WHC WiFi) */
#define RTL8730E_IRQ_UART0      82   /* GIC SPI 50: UART0 */
#define RTL8730E_IRQ_UART1      83   /* GIC SPI 51: UART1 */
#define RTL8730E_IRQ_UART2      84   /* GIC SPI 52: UART2 */
#define RTL8730E_IRQ_WDG4       94   /* GIC SPI 62: WDG4 (CA32 watchdog) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_RTL8730E_IRQ_H */
