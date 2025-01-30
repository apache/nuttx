/****************************************************************************
 * arch/arm/include/mcx-nxxx/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_MCX_NXXX_IRQ_H
#define __ARCH_ARM_INCLUDE_MCX_NXXX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_N236)
#  include <arch/mcx-nxxx/n236_irq.h>
#else
#  error "Unrecognized MCX-NXXx architecture"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words
 * of memory in the IRQ to handle mapping tables.
 */

/* Common Processor Exceptions (vectors 0-15) */

#define NXXX_IRQ_RESERVED       (0) /* Vector  0: Reset stack pointer value */
                                    /* Vector  1: Reset(not handled by IRQ) */
#define NXXX_IRQ_NMI            (2) /* Vector  2: Non-Maskable Int (NMI) */
#define NXXX_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define NXXX_IRQ_MEMFAULT       (4) /* Vector  4: MemManage fault */
#define NXXX_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define NXXX_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define NXXX_IRQ_USAGEFAULT     (6) /* Vector  7: Secure fault */
                                    /* Vectors 8-10: Reserved */
#define NXXX_IRQ_SVCALL        (11) /* Vector 11: Supervisor Call (SVC) */
#define NXXX_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                    /* Vector 13: Reserved */
#define NXXX_IRQ_PENDSV        (14) /* Vector 14: Pendable System Service Request */
#define NXXX_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* Chip-Specific External interrupts */

#define NXXX_IRQ_EXTINT        (16) /* Vector number of the first external interrupt*/

#endif /* __ARCH_ARM_INCLUDE_MCX_NXXX_IRQ_H */
