/****************************************************************************
 * arch/arm/include/ht32f491x3/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_HT32F491X3_IRQ_H
#define __ARCH_ARM_INCLUDE_HT32F491X3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/ht32f491x3/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HT32_IRQ_RESERVED       (0)
#define HT32_IRQ_NMI            (2)
#define HT32_IRQ_HARDFAULT      (3)
#define HT32_IRQ_MEMFAULT       (4)
#define HT32_IRQ_BUSFAULT       (5)
#define HT32_IRQ_USAGEFAULT     (6)
#define HT32_IRQ_SVCALL        (11)
#define HT32_IRQ_DBGMONITOR    (12)
#define HT32_IRQ_PENDSV        (14)
#define HT32_IRQ_SYSTICK       (15)

#define HT32_IRQ_FIRST         (16)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/ht32f491x3/ht32f491x3_irq.h>

#endif /* __ARCH_ARM_INCLUDE_HT32F491X3_IRQ_H */
