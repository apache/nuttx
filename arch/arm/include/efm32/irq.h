/****************************************************************************
 * arch/arm/include/efm32/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_EFM32_IRQ_H
#define __ARCH_ARM_INCLUDE_EFM32_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#include <arch/efm32/chip.h>

#if defined(CONFIG_EFM32_EFM32TG)
#  include <arch/efm32/efm32tg_irq.h>
#elif defined(CONFIG_EFM32_EFM32G)
#  include <arch/efm32/efm32g_irq.h>
#elif defined(CONFIG_EFM32_EFM32GG)
#  include <arch/efm32/efm32gg_irq.h>
#else
#  error "Unsupported EFM32 chip"
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to bits in
 *  the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define EFM32_IRQ_RESERVED     (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                   /* Vector  0: Reset stack pointer value */
                                   /* Vector  1: Reset (not handler as an IRQ) */
#define EFM32_IRQ_NMI          (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define EFM32_IRQ_HARDFAULT    (3) /* Vector  3: Hard fault */
#define EFM32_IRQ_MEMFAULT     (4) /* Vector  4: Memory management (MPU) */
#define EFM32_IRQ_BUSFAULT     (5) /* Vector  5: Bus fault */
#define EFM32_IRQ_USAGEFAULT   (6) /* Vector  6: Usage fault */
#define EFM32_IRQ_SVCALL      (11) /* Vector 11: SVC call */
#define EFM32_IRQ_DBGMONITOR  (12) /* Vector 12: Debug Monitor */
                                   /* Vector 13: Reserved */
#define EFM32_IRQ_PENDSV      (14) /* Vector 14: Pendable system service request */
#define EFM32_IRQ_SYSTICK     (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16).
 * These definitions are chip-specific
 */

#define EFM32_IRQ_INTERRUPTS  (16) /* Vector number of the first external interrupt */

#ifdef CONFIG_EFM32_GPIO_IRQ
/* If GPIO interrupt support is enabled then up to 16 additional GPIO
 * interrupt sources are available.  There are actually only two physical
 * interrupt lines: GPIO_EVEN and GPIO_ODD.  However, from the software point
 * of view, there are 16-additional interrupts generated from a second level
 * of decoding.
 */

#  define EFM32_IRQ_EXTI0   (EFM32_IRQ_NVECTORS + 0)  /* Port[n], pin0 external interrupt */
#  define EFM32_IRQ_EXTI1   (EFM32_IRQ_NVECTORS + 1)  /* Port[n], pin1 external interrupt */
#  define EFM32_IRQ_EXTI2   (EFM32_IRQ_NVECTORS + 2)  /* Port[n], pin2 external interrupt */
#  define EFM32_IRQ_EXTI3   (EFM32_IRQ_NVECTORS + 3)  /* Port[n], pin3 external interrupt */
#  define EFM32_IRQ_EXTI4   (EFM32_IRQ_NVECTORS + 4)  /* Port[n], pin4 external interrupt */
#  define EFM32_IRQ_EXTI5   (EFM32_IRQ_NVECTORS + 5)  /* Port[n], pin5 external interrupt */
#  define EFM32_IRQ_EXTI6   (EFM32_IRQ_NVECTORS + 6)  /* Port[n], pin6 external interrupt */
#  define EFM32_IRQ_EXTI7   (EFM32_IRQ_NVECTORS + 7)  /* Port[n], pin7 external interrupt */
#  define EFM32_IRQ_EXTI8   (EFM32_IRQ_NVECTORS + 8)  /* Port[n], pin8 external interrupt */
#  define EFM32_IRQ_EXTI9   (EFM32_IRQ_NVECTORS + 9)  /* Port[n], pin9 external interrupt */
#  define EFM32_IRQ_EXTI10  (EFM32_IRQ_NVECTORS + 10) /* Port[n], pin10 external interrupt */
#  define EFM32_IRQ_EXTI11  (EFM32_IRQ_NVECTORS + 11) /* Port[n], pin11 external interrupt */
#  define EFM32_IRQ_EXTI12  (EFM32_IRQ_NVECTORS + 12) /* Port[n], pin12 external interrupt */
#  define EFM32_IRQ_EXTI13  (EFM32_IRQ_NVECTORS + 13) /* Port[n], pin13 external interrupt */
#  define EFM32_IRQ_EXTI14  (EFM32_IRQ_NVECTORS + 14) /* Port[n], pin14 external interrupt */
#  define EFM32_IRQ_EXTI15  (EFM32_IRQ_NVECTORS + 15) /* Port[n], pin15 external interrupt */

#  define NR_IRQS           (EFM32_IRQ_NVECTORS + 16) /* Total number of interrupts */
#else
#  define NR_IRQS           EFM32_IRQ_NVECTORS        /* Total number of interrupts */
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

#endif /* __ARCH_ARM_INCLUDE_EFM32_IRQ_H */
