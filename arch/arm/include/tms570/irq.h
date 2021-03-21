/****************************************************************************
 * arch/arm/include/tms570/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_TMS570_IRQ_H
#define __ARCH_ARM_INCLUDE_TMS570_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tms570/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* The interrupt vector table only has 96 entries, one phantom vector and 95
 * interrupt channels.
 * Channel 95 does not have a dedicated vector and shall not be used.
 */

#define TMS570_VECT_PHANTOM    0  /* The first is the "phantom" interrupt */

/* Default channel assignments are MCU-dependent */

#if defined(CONFIG_ARCH_CHIP_TMS570LS0232PZ)
#  error No IRQ definitions for the TMS570LS0232PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0332PZ)
#  include <arch/tms570/tms570ls04x03x_irq.h>
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0432PZ)
#  include <arch/tms570/tms570ls04x03x_irq.h>
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PZ)
#  error No IRQ definitions for the TMS570LS0714PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PGE)
#  error No IRQ definitions for the TMS570LS0714PGE
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714ZWT)
#  error No IRQ definitions for the TMS570LS0714ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS1227ZWT)
#  error No IRQ definitions for the TMS570LS1227ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
#  include <arch/tms570/tms570ls31xx_irq.h>
#else
#  error "Unrecognized Hercules chip"
#endif

/* Total number of IRQ numbers.
 * Includes all channels plus GIO second-level interrupts (if enabled).
 *Excluds the phantom vector.  Zero corresponds to channel 0, vector 1.
 */

#define NR_IRQS  (TMS570_IRQ_NCHANNELS + TMS570_NGIO_IRQS)

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

#endif /* __ARCH_ARM_INCLUDE_TMS570_IRQ_H */
