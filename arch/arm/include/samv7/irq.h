/****************************************************************************
 * arch/arm/include/samv7/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_SAMV7_IRQ_H
#define __ARCH_ARM_INCLUDE_SAMV7_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in
 * the IRQ to handle mapping tables.
 */

/* Common Processor Exceptions (vectors 0-15) */

#define SAM_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                   /* Vector  0: Reset stack pointer value */
                                   /* Vector  1: Reset (not handler as an IRQ) */
#define SAM_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define SAM_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define SAM_IRQ_MEMFAULT       (4) /* Vector  4: Memory management (MPU) */
#define SAM_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define SAM_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
                                   /* Vectors 7-10: Reserved */
#define SAM_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define SAM_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                   /* Vector 13: Reserved */
#define SAM_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define SAM_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* Chip-Specific External interrupts */

#define SAM_IRQ_EXTINT        (16) /* Vector number of the first external interrupt */

#if defined(CONFIG_ARCH_CHIP_SAMV71)
#  include <arch/samv7/samv71_irq.h>
#elif defined(CONFIG_ARCH_CHIP_SAME70)
#  include <arch/samv7/same70_irq.h>
#else
#  error Unrecognized SAMV7 architecture
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_SAMV7_IRQ_H */
