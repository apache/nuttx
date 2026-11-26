/****************************************************************************
 * arch/arm/include/efr32/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_EFR32_IRQ_H
#define __ARCH_ARM_INCLUDE_EFR32_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_EFR32_EFR32MG24)
#  include <arch/efr32/efr32mg24_irq.h>
#else
#  error "Unsupported EFR32 family"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds to the vector number and hence
 * maps directly to bits in the NVIC.
 */

/* Processor exceptions (vectors 0-15) */

#define EFR32_IRQ_RESERVED       (0)  /* Reset stack pointer value */
                                      /* Vector  1: Reset (not an IRQ) */
#define EFR32_IRQ_NMI            (2)  /* Non-maskable interrupt */
#define EFR32_IRQ_HARDFAULT      (3)  /* Hard fault */
#define EFR32_IRQ_MEMFAULT       (4)  /* Memory management (MPU) */
#define EFR32_IRQ_BUSFAULT       (5)  /* Bus fault */
#define EFR32_IRQ_USAGEFAULT     (6)  /* Usage fault */
#define EFR32_IRQ_SECUREFAULT    (7)  /* Secure fault */
                                      /* Vectors 8-10: Reserved */
#define EFR32_IRQ_SVCALL         (11) /* SVC call */
#define EFR32_IRQ_DBGMONITOR     (12) /* Debug monitor */
                                      /* Vector 13: Reserved */
#define EFR32_IRQ_PENDSV         (14) /* Pendable system service request */
#define EFR32_IRQ_SYSTICK        (15) /* System tick */

/* External interrupts (vectors >= 16), numbered from EFR32_IRQ_FIRST in
 * the family header included above.
 */

#define EFR32_IRQ_FIRST          (16)

/* GPIO pin interrupts EXTI0-11, decoded from the GPIO_EVEN and GPIO_ODD
 * interrupts.  Pin n of any port uses EXTIn.
 */

#ifdef CONFIG_EFR32_GPIO_IRQ
#  define EFR32_IRQ_EXTI0        (EFR32_IRQ_FIRST + EFR32_IRQ_NEXTINTS)
#  define EFR32_IRQ_EXTI(n)      (EFR32_IRQ_EXTI0 + (n))
#  define EFR32_IRQ_NEXTI        12
#else
#  define EFR32_IRQ_NEXTI        0
#endif

#define NR_IRQS                  (EFR32_IRQ_FIRST + EFR32_IRQ_NEXTINTS + \
                                  EFR32_IRQ_NEXTI)

#endif /* __ARCH_ARM_INCLUDE_EFR32_IRQ_H */
