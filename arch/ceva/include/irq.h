/****************************************************************************
 * arch/ceva/include/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_CEVA_INCLUDE_IRQ_H
#define __ARCH_CEVA_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/* Include CEVA architecture-specific IRQ definitions (including register
 * save structure and up_irq_save()/up_irq_restore() functions)
 */

#if defined(CONFIG_ARCH_XC5)
#  include <arch/xc5/irq.h>
#elif defined(CONFIG_ARCH_XM6)
#  include <arch/xm6/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_VINT0           (IRQ_VINT_FIRST + 0)
#define IRQ_VINT1           (IRQ_VINT_FIRST + 1)
#define IRQ_VINT2           (IRQ_VINT_FIRST + 2)
#define IRQ_VINT3           (IRQ_VINT_FIRST + 3)
#define IRQ_VINT4           (IRQ_VINT_FIRST + 4)
#define IRQ_VINT5           (IRQ_VINT_FIRST + 5)
#define IRQ_VINT6           (IRQ_VINT_FIRST + 6)
#define IRQ_VINT7           (IRQ_VINT_FIRST + 7)
#define IRQ_VINT8           (IRQ_VINT_FIRST + 8)
#define IRQ_VINT9           (IRQ_VINT_FIRST + 9)
#define IRQ_VINT10          (IRQ_VINT_FIRST + 10)
#define IRQ_VINT11          (IRQ_VINT_FIRST + 11)
#define IRQ_VINT12          (IRQ_VINT_FIRST + 12)
#define IRQ_VINT13          (IRQ_VINT_FIRST + 13)
#define IRQ_VINT14          (IRQ_VINT_FIRST + 14)
#define IRQ_VINT15          (IRQ_VINT_FIRST + 15)
#define IRQ_VINT16          (IRQ_VINT_FIRST + 16)
#define IRQ_VINT17          (IRQ_VINT_FIRST + 17)
#define IRQ_VINT18          (IRQ_VINT_FIRST + 18)
#define IRQ_VINT19          (IRQ_VINT_FIRST + 19)
#define IRQ_VINT20          (IRQ_VINT_FIRST + 20)
#define IRQ_VINT21          (IRQ_VINT_FIRST + 21)
#define IRQ_VINT22          (IRQ_VINT_FIRST + 22)
#define IRQ_VINT23          (IRQ_VINT_FIRST + 23)
#define IRQ_VINT24          (IRQ_VINT_FIRST + 24)
#define IRQ_VINT25          (IRQ_VINT_FIRST + 25)
#define IRQ_VINT26          (IRQ_VINT_FIRST + 26)

#endif /* __ARCH_CEVA_INCLUDE_IRQ_H */
