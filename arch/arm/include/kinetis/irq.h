/****************************************************************************
 * arch/arm/include/kinetis/irq.h
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

/* This file should never be included directly but, rather, only
 * indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_KINETIS_IRQ_H
#define __ARCH_ARM_INCLUDE_KINETIS_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to bits in
 * the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define KINETIS_IRQ_RESERVED      (0)   /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                        /* Vector  0: Reset stack pointer value */
                                        /* Vector  1: Reset (not handler as an IRQ) */
#define KINETIS_IRQ_NMI           (2)   /* Vector  2: Non-Maskable Interrupt (NMI) */
#define KINETIS_IRQ_HARDFAULT     (3)   /* Vector  3: Hard fault */
#define KINETIS_IRQ_MEMFAULT      (4)   /* Vector  4: Memory management (MPU) */
#define KINETIS_IRQ_BUSFAULT      (5)   /* Vector  5: Bus fault */
#define KINETIS_IRQ_USAGEFAULT    (6)   /* Vector  6: Usage fault */
                                        /* Vectors 7-10: Reserved */
#define KINETIS_IRQ_SVCALL        (11)  /* Vector 11: SVC call */
#define KINETIS_IRQ_DBGMONITOR    (12)  /* Vector 12: Debug Monitor */
                                        /* Vector 13: Reserved */
#define KINETIS_IRQ_PENDSV        (14)  /* Vector 14: Pendable system service request */
#define KINETIS_IRQ_SYSTICK       (15)  /* Vector 15: System tick */

/* External interrupts (vectors >= 16).
 * These definitions are chip-specific
 */

#define KINETIS_IRQ_FIRST        (16) /* Vector number of the first external interrupt */

#if defined(CONFIG_ARCH_FAMILY_K20)
#  include <arch/kinetis/kinetis_k20irq.h>
#elif defined(CONFIG_ARCH_FAMILY_K28)
#  include <arch/kinetis/kinetis_k28irq.h>
#elif defined(CONFIG_ARCH_FAMILY_K40)
#  include <arch/kinetis/kinetis_k40irq.h>
#elif defined(CONFIG_ARCH_FAMILY_K60)
#  include <arch/kinetis/kinetis_k60irq.h>
#elif defined(CONFIG_ARCH_FAMILY_K64)
#  include <arch/kinetis/kinetis_k64irq.h>
#elif defined(CONFIG_ARCH_FAMILY_K66)
#  include <arch/kinetis/kinetis_k66irq.h>
#else
/* The interrupt vectors for other parts are defined in other documents and
 * may or may not be the same as above (the family members are all very
 * similar)  This error just means that you have to look at the document and
 * determine for yourself if the vectors are the same.
 */

#  error "No IRQ numbers for this Kinetis K part"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_IRQ_H */
