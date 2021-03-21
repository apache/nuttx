/****************************************************************************
 * arch/arm/include/tiva/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_TIVA_IRQ_H
#define __ARCH_ARM_INCLUDE_TIVA_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tiva/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Mark GPIO interrupts as disabled for non-existent GPIO ports. */

#if TIVA_NPORTS < 1
#  undef CONFIG_TIVA_GPIOA_IRQS
#endif
#if TIVA_NPORTS < 2
#  undef CONFIG_TIVA_GPIOB_IRQS
#endif
#if TIVA_NPORTS < 3
#  undef CONFIG_TIVA_GPIOC_IRQS
#endif
#if TIVA_NPORTS < 4
#  undef CONFIG_TIVA_GPIOD_IRQS
#endif
#if TIVA_NPORTS < 5
#  undef CONFIG_TIVA_GPIOE_IRQS
#endif
#if TIVA_NPORTS < 6
#  undef CONFIG_TIVA_GPIOF_IRQS
#endif
#if TIVA_NPORTS < 7
#  undef CONFIG_TIVA_GPIOG_IRQS
#endif
#if TIVA_NPORTS < 8
#  undef CONFIG_TIVA_GPIOH_IRQS
#endif
#if TIVA_NPORTS < 9
#  undef CONFIG_TIVA_GPIOJ_IRQS
#endif
#if TIVA_NPORTS < 10
#  undef CONFIG_TIVA_GPIOK_IRQS
#endif
#if TIVA_NPORTS < 11
#  undef CONFIG_TIVA_GPIOL_IRQS
#endif
#if TIVA_NPORTS < 12
#  undef CONFIG_TIVA_GPIOM_IRQS
#endif
#if TIVA_NPORTS < 13
#  undef CONFIG_TIVA_GPION_IRQS
#endif
#if TIVA_NPORTS < 14
#  undef CONFIG_TIVA_GPIOP_IRQS
#endif
#if TIVA_NPORTS < 15
#  undef CONFIG_TIVA_GPIOQ_IRQS
#endif
#if TIVA_NPORTS < 16
#  undef CONFIG_TIVA_GPIOR_IRQS
#endif
#if TIVA_NPORTS < 17
#  undef CONFIG_TIVA_GPIOS_IRQS
#endif
#if TIVA_NPORTS < 18
#  undef CONFIG_TIVA_GPIOT_IRQS
#endif

/* Processor Exceptions (vectors 0-15) */

#define TIVA_IRQ_RESERVED     (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                  /* Vector  0: Reset stack pointer value */
                                  /* Vector  1: Reset (not handler as an IRQ) */
#define TIVA_IRQ_NMI          (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define TIVA_IRQ_HARDFAULT    (3) /* Vector  3: Hard fault */
#define TIVA_IRQ_MEMFAULT     (4) /* Vector  4: Memory management (MPU) */
#define TIVA_IRQ_BUSFAULT     (5) /* Vector  5: Bus fault */
#define TIVA_IRQ_USAGEFAULT   (6) /* Vector  6: Usage fault */
#define TIVA_IRQ_SVCALL      (11) /* Vector 11: SVC call */
#define TIVA_IRQ_DBGMONITOR  (12) /* Vector 12: Debug Monitor */
                                  /* Vector 13: Reserved */
#define TIVA_IRQ_PENDSV      (14) /* Vector 14: Pendable system service request */
#define TIVA_IRQ_SYSTICK     (15) /* Vector 15: System tick */

/* Chip-specific external Interrupts (vectors 16 and above) */

#define TIVA_IRQ_INTERRUPTS (16) /* Vector number of the first external interrupt */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_LM3S)
#  include <arch/tiva/lm3s_irq.h>
#elif defined(CONFIG_ARCH_CHIP_LM4F)
#  include <arch/tiva/lm4f_irq.h>
#elif defined(CONFIG_ARCH_CHIP_TM4C)
#  include <arch/tiva/tm4c_irq.h>
#elif defined(CONFIG_ARCH_CHIP_CC13X0)
#  include <arch/tiva/cc13x0_irq.h>
#elif defined(CONFIG_ARCH_CHIP_CC13X2)
#  include <arch/tiva/cc13x2_cc26x2_irq.h>
#else
#  error "Unsupported Stellaris IRQ file"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_TIVA_IRQ_H */
