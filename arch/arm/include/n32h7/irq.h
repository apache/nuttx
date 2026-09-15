/****************************************************************************
 * arch/arm/include/n32h7/irq.h
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

 #ifndef __ARCH_ARM_INCLUDE_N32H7_IRQ_H
 #define __ARCH_ARM_INCLUDE_N32H7_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* N32H7x3xx  Differences between family members:
 *
 *   ----------- ---------------- ----- ----
 *                                       SPI
 *   PART        PACKAGE          GPIOs  I2S
 *   ----------- ---------------- ----- ----
 *   N32H762Ix    LQFP176          138   7/4
 *   ----------- ---------------- ----- ----
 *
 * Parts N32H7xxxI have 2048Kb of FLASH
 *
 * The correct FLASH size will be set CONFIG_N32H7_FLASH_CONFIG_x or
 * overridden with CONFIG_N32H7_FLASH_OVERRIDE_x
 */

#if defined (CONFIG_ARCH_CHIP_N32H762II)
#else
#  error N32 H7 chip not identified
#endif

/* Processor Exceptions (vectors 0-15) */

#define N32_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */

/* Vector  0: Reset stack pointer value */

/* Vector  1: Reset (not handler as an IRQ) */
#define N32_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define N32_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define N32_IRQ_MEMFAULT       (4) /* Vector  4: Memory management (MPU) */
#define N32_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define N32_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */

/* Vectors 7-10: Reserved */
#define N32_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define N32_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */

/* Vector 13: Reserved */
#define N32_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define N32_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16).  These definitions are chip-specific
 */

#define N32_IRQ_FIRST         (16) /* Vector number of the first external interrupt */

#if defined(CONFIG_N32H7_N32H76X)
#  include <arch/n32h7/n32h76x_irq.h>
#else
#  error "Unsupported N32H7 chip"
#endif

#endif /* __ARCH_ARM_INCLUDE_N32H7_IRQ_H */
