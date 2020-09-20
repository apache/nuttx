/****************************************************************************
 * arch/arm/include/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_IRQ_H
#define __ARCH_ARM_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/* Include ARM architecture-specific IRQ definitions (including register
 * save structure and up_irq_save()/up_irq_restore() functions)
 */

#if defined(CONFIG_ARCH_ARMV7A)
#  include <arch/armv7-a/irq.h>
#elif defined(CONFIG_ARCH_ARMV7R)
#  include <arch/armv7-r/irq.h>
#elif defined(CONFIG_ARCH_ARMV7M)
#  include <arch/armv7-m/irq.h>
#elif defined(CONFIG_ARCH_ARMV8M)
#  include <arch/armv8-m/irq.h>
#elif defined(CONFIG_ARCH_ARMV6M)
#  include <arch/armv6-m/irq.h>
#else
#  include <arch/arm/irq.h>
#endif

#endif /* __ARCH_ARM_INCLUDE_IRQ_H */
