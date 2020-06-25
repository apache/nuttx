/****************************************************************************
 * arch/arm/include/eoss3/irq.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_EOSS3_IRQ_H
#define __ARCH_ARM_INCLUDE_EOSS3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/eoss3/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common Processor Exceptions (vectors 0-15) */

#define EOSS3_IRQ_RESERVED       (0) /* Reserved vector */
                                     /* Vector  0: Reset stack pointer val */
                                     /* Vector  1: Reset (unused) */
#define EOSS3_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt */
#define EOSS3_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define EOSS3_IRQ_MEMFAULT       (4) /* Vector  4: Memory management */
#define EOSS3_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define EOSS3_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
                                     /* Vectors 7-10: Reserved */
#define EOSS3_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define EOSS3_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define EOSS3_IRQ_PENDSV        (14) /* Vector 14: Pendable sys srv req */
#define EOSS3_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* Chip-Specific External interrupts */

#define EOSS3_IRQ_EXTINT        (16) /* Vector num of first ext interrupt */

#define NR_IRQS     EOSS3_IRQ_EXTINT /* Total number of interrupts */

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

#endif /* #define __ARCH_ARM_INCLUDE_EOSS3_IRQ_H */
