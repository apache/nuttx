/****************************************************************************
 * arch/arm/include/rm57/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_RM57_IRQ_H
#define __ARCH_ARM_INCLUDE_RM57_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/rm57/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* The VIM interrupt vector table has one phantom vector plus a set of
 * real interrupt channels, per the classic Hercules VIM layout (see
 * arch/arm/include/tms570/irq.h for the sibling family's equivalent).
 */

#define RM57_VECT_PHANTOM    0  /* The first is the "phantom" interrupt */

/* Default channel assignments are MCU-dependent */

#if defined(CONFIG_ARCH_CHIP_RM57L843)
#  include <arch/rm57/rm57l843_irq.h>
#else
#  error "Unrecognized Hercules RM57 chip"
#endif

/* Total number of IRQ numbers.
 * Includes all channels plus GIO second-level interrupts (if enabled).
 * Excludes the phantom vector.  Zero corresponds to channel 0, vector 1.
 */

#define NR_IRQS  (RM57_IRQ_NCHANNELS + RM57_NGIO_IRQS)

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

#endif /* __ARCH_ARM_INCLUDE_RM57_IRQ_H */
