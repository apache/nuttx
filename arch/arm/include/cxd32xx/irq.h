/****************************************************************************
 * arch/arm/include/cxd32xx/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD32XX_IRQ_H
#define __ARCH_ARM_INCLUDE_CXD32XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC. This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define CXD32_IRQ_RESERVED      (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                    /* Vector  0: Reset stack pointer value */
                                    /* Vector  1: Reset (not handler as an IRQ) */
#define CXD32_IRQ_NMI           (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define CXD32_IRQ_HARDFAULT     (3) /* Vector  3: Hard fault */
#define CXD32_IRQ_MEMFAULT      (4) /* Vector  4: Memory management (MPU) */
#define CXD32_IRQ_BUSFAULT      (5) /* Vector  5: Bus fault */
#define CXD32_IRQ_USAGEFAULT    (6) /* Vector  6: Usage fault */
#define CXD32_IRQ_SIGNVALUE     (7) /* Vector  7: Sign value */
#define CXD32_IRQ_SVCALL        (11)/* Vector 11: SVC call */
#define CXD32_IRQ_DBGMONITOR    (12)/* Vector 12: Debug Monitor */
                                    /* Vector 13: Reserved */
#define CXD32_IRQ_PENDSV        (14)/* Vector 14: Pendable system service request */
#define CXD32_IRQ_SYSTICK       (15)/* Vector 15: System tick */
#define CXD32_IRQ_EXTINT        (16)/* Vector 16: Vector number of the first external interrupt */

/* Cortex-M4 External interrupts (vectors >= 16) */

#define CXD32_IRQ_XXX           (CXD32_IRQ_EXTINT+0)   /* XXX IRQ number */

/* XXX The details of interrupt number will be described later. */
#define CXD32_IRQ_UART0         (CXD32_IRQ_EXTINT+73)       /* 16+73: UART0 interrupt */

#define CXD32_IRQ_TIM01         (CXD32_IRQ_EXTINT+77)       /* 16+77: TIMER0_1 interrupt */
#define CXD32_IRQ_TIM02         (CXD32_IRQ_EXTINT+78)       /* 16+78: TIMER0_2 interrupt */
#define CXD32_IRQ_TIM11         (CXD32_IRQ_EXTINT+79)       /* 16+79: TIMER1_1 interrupt */
#define CXD32_IRQ_TIM12         (CXD32_IRQ_EXTINT+80)       /* 16+80: TIMER1_2 interrupt */
#define CXD32_IRQ_TIM21         (CXD32_IRQ_EXTINT+81)       /* 16+81: TIMER2_1 interrupt */
#define CXD32_IRQ_TIM22         (CXD32_IRQ_EXTINT+82)       /* 16+82: TIMER2_2 interrupt */
#define CXD32_IRQ_TIM31         (CXD32_IRQ_EXTINT+83)       /* 16+83: TIMER3_1 interrupt */
#define CXD32_IRQ_TIM32         (CXD32_IRQ_EXTINT+84)       /* 16+84: TIMER3_2 interrupt */
#define CXD32_IRQ_TIM41         (CXD32_IRQ_EXTINT+85)       /* 16+85: TIMER4_1 interrupt */
#define CXD32_IRQ_TIM42         (CXD32_IRQ_EXTINT+86)       /* 16+86: TIMER4_2 interrupt */

#define CXD32_IRQ_NEXTINT       (128)
#define CXD32_IRQ_NIRQS         (CXD32_IRQ_EXTINT+CXD32_IRQ_NEXTINT)

/* Total number of IRQ numbers (This will need to be revisited if/when the
 * Cortex-M0 is supported)
 */

#define NR_VECTORS              CXD32_IRQ_NIRQS
#define NR_IRQS                 CXD32_IRQ_NIRQS

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);
#endif

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

#endif /* __ARCH_ARM_INCLUDE_CXD32XX_IRQ_H */
