/************************************************************************************
 * arch/arm/include/tiva/irq.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_TIVA_IRQ_H
#define __ARCH_ARM_INCLUDE_TIVA_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/tiva/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#if defined(CONFIG_ARCH_CHIP_LM3S) || defined(CONFIG_ARCH_CHIP_LM4F) || \
    defined(CONFIG_ARCH_CHIP_CC3200)

  /* I don't believe that any of these families support interrupts on port J.  Many
   * do not support interrupts on port H either.
   */

#  undef CONFIG_TIVA_GPIOJ_IRQS

#elif defined(CONFIG_ARCH_CHIP_TM4C)

/* The TM4C123GH6PMI supports ports A-F of which any can support interrupts */

#  if defined(CONFIG_ARCH_CHIP_TM4C123GH6PMI)
#    undef CONFIG_TIVA_GPIOP_IRQS /* P-Q */
#    undef CONFIG_TIVA_GPIOQ_IRQS

/* The TM4C123GH6PGE supports interrupts only on port P */

#  elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PGE)
#    undef CONFIG_TIVA_GPIOA_IRQS /* A-F */
#    undef CONFIG_TIVA_GPIOB_IRQS
#    undef CONFIG_TIVA_GPIOC_IRQS
#    undef CONFIG_TIVA_GPIOD_IRQS
#    undef CONFIG_TIVA_GPIOE_IRQS
#    undef CONFIG_TIVA_GPIOF_IRQS

#    undef CONFIG_TIVA_GPIOQ_IRQS /* Q */

/* The TM4C123GH6ZRB and the TM4C129x support interrupts only on ports P and Q. */

#  else
#    undef CONFIG_TIVA_GPIOA_IRQS /* A-F */
#    undef CONFIG_TIVA_GPIOB_IRQS
#    undef CONFIG_TIVA_GPIOC_IRQS
#    undef CONFIG_TIVA_GPIOD_IRQS
#    undef CONFIG_TIVA_GPIOE_IRQS
#    undef CONFIG_TIVA_GPIOF_IRQS

#  endif

/* No supported architecture supports interrupts on ports G-N or R-T */

#  undef CONFIG_TIVA_GPIOG_IRQS /* G-N */
#  undef CONFIG_TIVA_GPIOH_IRQS
#  undef CONFIG_TIVA_GPIOJ_IRQS
#  undef CONFIG_TIVA_GPIOK_IRQS
#  undef CONFIG_TIVA_GPIOL_IRQS
#  undef CONFIG_TIVA_GPIOM_IRQS
#  undef CONFIG_TIVA_GPION_IRQS

#  undef CONFIG_TIVA_GPIOR_IRQS /* R-T */
#  undef CONFIG_TIVA_GPIOS_IRQS
#  undef CONFIG_TIVA_GPIOT_IRQS

#endif

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
#  undef CONFIG_TIVA_GPIOQ_IRQS
#endif
#if TIVA_NPORTS < 17
#  undef CONFIG_TIVA_GPIOQ_IRQS
#endif
#if TIVA_NPORTS < 18
#  undef CONFIG_TIVA_GPIOQ_IRQS
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

#if defined(CONFIG_ARCH_CHIP_LM3S)
#  include <arch/tiva/lm3s_irq.h>
#elif defined(CONFIG_ARCH_CHIP_LM4F)
#  include <arch/tiva/lm4f_irq.h>
#elif defined(CONFIG_ARCH_CHIP_TM4C)
#  include <arch/tiva/tm4c_irq.h>
#elif defined(CONFIG_ARCH_CHIP_CC3200)
#  include <arch/tiva/cc3200_irq.h>
#else
#  error "Unsupported Stellaris IRQ file"
#endif

#define NR_VECTORS (NR_IRQS - 16)

/* GPIO IRQs -- Note that support for individual GPIO ports can
 * be disabled in order to reduce the size of the implementation.
 */

#if defined(CONFIG_TIVA_GPIOA_IRQS)
#  define TIVA_IRQ_GPIOA_0 (NR_IRQS + 0)
#  define TIVA_IRQ_GPIOA_1 (NR_IRQS + 1)
#  define TIVA_IRQ_GPIOA_2 (NR_IRQS + 2)
#  define TIVA_IRQ_GPIOA_3 (NR_IRQS + 3)
#  define TIVA_IRQ_GPIOA_4 (NR_IRQS + 4)
#  define TIVA_IRQ_GPIOA_5 (NR_IRQS + 5)
#  define TIVA_IRQ_GPIOA_6 (NR_IRQS + 6)
#  define TIVA_IRQ_GPIOA_7 (NR_IRQS + 7)
#  define _NGPIOAIRQS      (NR_IRQS + 8)
#else
#  define _NGPIOAIRQS      NR_IRQS
#endif

#if defined(CONFIG_TIVA_GPIOB_IRQS)
#  define TIVA_IRQ_GPIOB_0 (_NGPIOAIRQS + 0)
#  define TIVA_IRQ_GPIOB_1 (_NGPIOAIRQS + 1)
#  define TIVA_IRQ_GPIOB_2 (_NGPIOAIRQS + 2)
#  define TIVA_IRQ_GPIOB_3 (_NGPIOAIRQS + 3)
#  define TIVA_IRQ_GPIOB_4 (_NGPIOAIRQS + 4)
#  define TIVA_IRQ_GPIOB_5 (_NGPIOAIRQS + 5)
#  define TIVA_IRQ_GPIOB_6 (_NGPIOAIRQS + 6)
#  define TIVA_IRQ_GPIOB_7 (_NGPIOAIRQS + 7)
#  define _NGPIOBIRQS      (_NGPIOAIRQS + 8)
#else
#  define _NGPIOBIRQS      _NGPIOAIRQS
#endif

#if defined(CONFIG_TIVA_GPIOC_IRQS)
#  define TIVA_IRQ_GPIOC_0 (_NGPIOBIRQS + 0)
#  define TIVA_IRQ_GPIOC_1 (_NGPIOBIRQS + 1)
#  define TIVA_IRQ_GPIOC_2 (_NGPIOBIRQS + 2)
#  define TIVA_IRQ_GPIOC_3 (_NGPIOBIRQS + 3)
#  define TIVA_IRQ_GPIOC_4 (_NGPIOBIRQS + 4)
#  define TIVA_IRQ_GPIOC_5 (_NGPIOBIRQS + 5)
#  define TIVA_IRQ_GPIOC_6 (_NGPIOBIRQS + 6)
#  define TIVA_IRQ_GPIOC_7 (_NGPIOBIRQS + 7)
#  define _NGPIOCIRQS      (_NGPIOBIRQS + 8)
#else
#  define _NGPIOCIRQS      _NGPIOBIRQS
#endif

#if defined(CONFIG_TIVA_GPIOD_IRQS)
#  define TIVA_IRQ_GPIOD_0 (_NGPIOCIRQS + 0)
#  define TIVA_IRQ_GPIOD_1 (_NGPIOCIRQS + 1)
#  define TIVA_IRQ_GPIOD_2 (_NGPIOCIRQS + 2)
#  define TIVA_IRQ_GPIOD_3 (_NGPIOCIRQS + 3)
#  define TIVA_IRQ_GPIOD_4 (_NGPIOCIRQS + 4)
#  define TIVA_IRQ_GPIOD_5 (_NGPIOCIRQS + 5)
#  define TIVA_IRQ_GPIOD_6 (_NGPIOCIRQS + 6)
#  define TIVA_IRQ_GPIOD_7 (_NGPIOCIRQS + 7)
#  define _NGPIODIRQS      (_NGPIOCIRQS + 8)
#else
#  define _NGPIODIRQS      _NGPIOCIRQS
#endif

#if defined(CONFIG_TIVA_GPIOE_IRQS)
#  define TIVA_IRQ_GPIOE_0 (_NGPIODIRQS + 0)
#  define TIVA_IRQ_GPIOE_1 (_NGPIODIRQS + 1)
#  define TIVA_IRQ_GPIOE_2 (_NGPIODIRQS + 2)
#  define TIVA_IRQ_GPIOE_3 (_NGPIODIRQS + 3)
#  define TIVA_IRQ_GPIOE_4 (_NGPIODIRQS + 4)
#  define TIVA_IRQ_GPIOE_5 (_NGPIODIRQS + 5)
#  define TIVA_IRQ_GPIOE_6 (_NGPIODIRQS + 6)
#  define TIVA_IRQ_GPIOE_7 (_NGPIODIRQS + 7)
#  define _NGPIOEIRQS      (_NGPIODIRQS + 8)
#else
#  define _NGPIOEIRQS      _NGPIODIRQS
#endif

#if defined(CONFIG_TIVA_GPIOF_IRQS)
#  define TIVA_IRQ_GPIOF_0 (_NGPIOEIRQS + 0)
#  define TIVA_IRQ_GPIOF_1 (_NGPIOEIRQS + 1)
#  define TIVA_IRQ_GPIOF_2 (_NGPIOEIRQS + 2)
#  define TIVA_IRQ_GPIOF_3 (_NGPIOEIRQS + 3)
#  define TIVA_IRQ_GPIOF_4 (_NGPIOEIRQS + 4)
#  define TIVA_IRQ_GPIOF_5 (_NGPIOEIRQS + 5)
#  define TIVA_IRQ_GPIOF_6 (_NGPIOEIRQS + 6)
#  define TIVA_IRQ_GPIOF_7 (_NGPIOEIRQS + 7)
#  define _NGPIOFIRQS      (_NGPIOEIRQS + 8)
#else
#  define _NGPIOFIRQS      _NGPIOEIRQS
#endif

#if defined(CONFIG_TIVA_GPIOG_IRQS)
#  define TIVA_IRQ_GPIOG_0 (_NGPIOFIRQS + 0)
#  define TIVA_IRQ_GPIOG_1 (_NGPIOFIRQS + 1)
#  define TIVA_IRQ_GPIOG_2 (_NGPIOFIRQS + 2)
#  define TIVA_IRQ_GPIOG_3 (_NGPIOFIRQS + 3)
#  define TIVA_IRQ_GPIOG_4 (_NGPIOFIRQS + 4)
#  define TIVA_IRQ_GPIOG_5 (_NGPIOFIRQS + 5)
#  define TIVA_IRQ_GPIOG_6 (_NGPIOFIRQS + 6)
#  define TIVA_IRQ_GPIOG_7 (_NGPIOFIRQS + 7)
#  define _NGPIOGIRQS      (_NGPIOFIRQS + 8)
#else
#  define _NGPIOGIRQS      _NGPIOFIRQS
#endif

#if defined(CONFIG_TIVA_GPIOH_IRQS)
#  define TIVA_IRQ_GPIOH_0 (_NGPIOGIRQS + 0)
#  define TIVA_IRQ_GPIOH_1 (_NGPIOGIRQS + 1)
#  define TIVA_IRQ_GPIOH_2 (_NGPIOGIRQS + 2)
#  define TIVA_IRQ_GPIOH_3 (_NGPIOGIRQS + 3)
#  define TIVA_IRQ_GPIOH_4 (_NGPIOGIRQS + 4)
#  define TIVA_IRQ_GPIOH_5 (_NGPIOGIRQS + 5)
#  define TIVA_IRQ_GPIOH_6 (_NGPIOGIRQS + 6)
#  define TIVA_IRQ_GPIOH_7 (_NGPIOGIRQS + 7)
#  define _NGPIOHIRQS      (_NGPIOGIRQS + 8)
#else
#  define _NGPIOHIRQS      _NGPIOGIRQS
#endif

#if defined(CONFIG_TIVA_GPIOJ_IRQS)
#  define TIVA_IRQ_GPIOJ_0 (_NGPIOHIRQS + 0)
#  define TIVA_IRQ_GPIOJ_1 (_NGPIOHIRQS + 1)
#  define TIVA_IRQ_GPIOJ_2 (_NGPIOHIRQS + 2)
#  define TIVA_IRQ_GPIOJ_3 (_NGPIOHIRQS + 3)
#  define TIVA_IRQ_GPIOJ_4 (_NGPIOHIRQS + 4)
#  define TIVA_IRQ_GPIOJ_5 (_NGPIOHIRQS + 5)
#  define TIVA_IRQ_GPIOJ_6 (_NGPIOHIRQS + 6)
#  define TIVA_IRQ_GPIOJ_7 (_NGPIOHIRQS + 7)
#  define _NGPIOJIRQS      (_NGPIOHIRQS + 8)
#else
#  define _NGPIOJIRQS      _NGPIOHIRQS
#endif

#if defined(CONFIG_TIVA_GPIOK_IRQS)
#  define TIVA_IRQ_GPIOK_0 (_NGPIOJIRQS + 0)
#  define TIVA_IRQ_GPIOK_1 (_NGPIOJIRQS + 1)
#  define TIVA_IRQ_GPIOK_2 (_NGPIOJIRQS + 2)
#  define TIVA_IRQ_GPIOK_3 (_NGPIOJIRQS + 3)
#  define TIVA_IRQ_GPIOK_4 (_NGPIOJIRQS + 4)
#  define TIVA_IRQ_GPIOK_5 (_NGPIOJIRQS + 5)
#  define TIVA_IRQ_GPIOK_6 (_NGPIOJIRQS + 6)
#  define TIVA_IRQ_GPIOK_7 (_NGPIOJIRQS + 7)
#  define _NGPIOKIRQS      (_NGPIOJIRQS + 8)
#else
#  define _NGPIOKIRQS      _NGPIOJIRQS
#endif

#if defined(CONFIG_TIVA_GPIOL_IRQS)
#  define TIVA_IRQ_GPIOL_0 (_NGPIOKIRQS + 0)
#  define TIVA_IRQ_GPIOL_1 (_NGPIOKIRQS + 1)
#  define TIVA_IRQ_GPIOL_2 (_NGPIOKIRQS + 2)
#  define TIVA_IRQ_GPIOL_3 (_NGPIOKIRQS + 3)
#  define TIVA_IRQ_GPIOL_4 (_NGPIOKIRQS + 4)
#  define TIVA_IRQ_GPIOL_5 (_NGPIOKIRQS + 5)
#  define TIVA_IRQ_GPIOL_6 (_NGPIOKIRQS + 6)
#  define TIVA_IRQ_GPIOL_7 (_NGPIOKIRQS + 7)
#  define _NGPIOLIRQS      (_NGPIOKIRQS + 8)
#else
#  define _NGPIOLIRQS      _NGPIOKIRQS
#endif

#if defined(CONFIG_TIVA_GPIOM_IRQS)
#  define TIVA_IRQ_GPIOM_0 (_NGPIOLIRQS + 0)
#  define TIVA_IRQ_GPIOM_1 (_NGPIOLIRQS + 1)
#  define TIVA_IRQ_GPIOM_2 (_NGPIOLIRQS + 2)
#  define TIVA_IRQ_GPIOM_3 (_NGPIOLIRQS + 3)
#  define TIVA_IRQ_GPIOM_4 (_NGPIOLIRQS + 4)
#  define TIVA_IRQ_GPIOM_5 (_NGPIOLIRQS + 5)
#  define TIVA_IRQ_GPIOM_6 (_NGPIOLIRQS + 6)
#  define TIVA_IRQ_GPIOM_7 (_NGPIOLIRQS + 7)
#  define _NGPIOMIRQS      (_NGPIOLIRQS + 8)
#else
#  define _NGPIOMIRQS      _NGPIOLIRQS
#endif

#if defined(CONFIG_TIVA_GPION_IRQS)
#  define TIVA_IRQ_GPION_0 (_NGPIOMIRQS + 0)
#  define TIVA_IRQ_GPION_1 (_NGPIOMIRQS + 1)
#  define TIVA_IRQ_GPION_2 (_NGPIOMIRQS + 2)
#  define TIVA_IRQ_GPION_3 (_NGPIOMIRQS + 3)
#  define TIVA_IRQ_GPION_4 (_NGPIOMIRQS + 4)
#  define TIVA_IRQ_GPION_5 (_NGPIOMIRQS + 5)
#  define TIVA_IRQ_GPION_6 (_NGPIOMIRQS + 6)
#  define TIVA_IRQ_GPION_7 (_NGPIOMIRQS + 7)
#  define _NGPIONIRQS      (_NGPIOMIRQS + 8)
#else
#  define _NGPIONIRQS      _NGPIOMIRQS
#endif

#if defined(CONFIG_TIVA_GPIOP_IRQS)
#  define TIVA_IRQ_GPIOP_0 (_NGPIONIRQS + 0)
#  define TIVA_IRQ_GPIOP_1 (_NGPIONIRQS + 1)
#  define TIVA_IRQ_GPIOP_2 (_NGPIONIRQS + 2)
#  define TIVA_IRQ_GPIOP_3 (_NGPIONIRQS + 3)
#  define TIVA_IRQ_GPIOP_4 (_NGPIONIRQS + 4)
#  define TIVA_IRQ_GPIOP_5 (_NGPIONIRQS + 5)
#  define TIVA_IRQ_GPIOP_6 (_NGPIONIRQS + 6)
#  define TIVA_IRQ_GPIOP_7 (_NGPIONIRQS + 7)
#  define _NGPIOPIRQS      (_NGPIONIRQS + 8)
#else
#  define _NGPIOPIRQS      _NGPIONIRQS
#endif

#if defined(CONFIG_TIVA_GPIOQ_IRQS)
#  define TIVA_IRQ_GPIOQ_0 (_NGPIOPIRQS + 0)
#  define TIVA_IRQ_GPIOQ_1 (_NGPIOPIRQS + 1)
#  define TIVA_IRQ_GPIOQ_2 (_NGPIOPIRQS + 2)
#  define TIVA_IRQ_GPIOQ_3 (_NGPIOPIRQS + 3)
#  define TIVA_IRQ_GPIOQ_4 (_NGPIOPIRQS + 4)
#  define TIVA_IRQ_GPIOQ_5 (_NGPIOPIRQS + 5)
#  define TIVA_IRQ_GPIOQ_6 (_NGPIOPIRQS + 6)
#  define TIVA_IRQ_GPIOQ_7 (_NGPIOPIRQS + 7)
#  define _NGPIOQIRQS      (_NGPIOPIRQS + 8)
#else
#  define _NGPIOQIRQS      _NGPIOPIRQS
#endif

#if defined(CONFIG_TIVA_GPIOR_IRQS)
#  define TIVA_IRQ_GPIOR_0 (_NGPIOQIRQS + 0)
#  define TIVA_IRQ_GPIOR_1 (_NGPIOQIRQS + 1)
#  define TIVA_IRQ_GPIOR_2 (_NGPIOQIRQS + 2)
#  define TIVA_IRQ_GPIOR_3 (_NGPIOQIRQS + 3)
#  define TIVA_IRQ_GPIOR_4 (_NGPIOQIRQS + 4)
#  define TIVA_IRQ_GPIOR_5 (_NGPIOQIRQS + 5)
#  define TIVA_IRQ_GPIOR_6 (_NGPIOQIRQS + 6)
#  define TIVA_IRQ_GPIOR_7 (_NGPIOQIRQS + 7)
#  define _NGPIORIRQS      (_NGPIOQIRQS + 8)
#else
#  define _NGPIORIRQS      _NGPIOQIRQS
#endif

#if defined(CONFIG_TIVA_GPIOS_IRQS)
#  define TIVA_IRQ_GPIOS_0 (_NGPIORIRQS + 0)
#  define TIVA_IRQ_GPIOS_1 (_NGPIORIRQS + 1)
#  define TIVA_IRQ_GPIOS_2 (_NGPIORIRQS + 2)
#  define TIVA_IRQ_GPIOS_3 (_NGPIORIRQS + 3)
#  define TIVA_IRQ_GPIOS_4 (_NGPIORIRQS + 4)
#  define TIVA_IRQ_GPIOS_5 (_NGPIORIRQS + 5)
#  define TIVA_IRQ_GPIOS_6 (_NGPIORIRQS + 6)
#  define TIVA_IRQ_GPIOS_7 (_NGPIORIRQS + 7)
#  define _NGPIOSIRQS      (_NGPIORIRQS + 8)
#else
#  define _NGPIOSIRQS      _NGPIORIRQS
#endif

#if defined(CONFIG_TIVA_GPIOT_IRQS)
#  define TIVA_IRQ_GPIOT_0 (_NGPIOSIRQS + 0)
#  define TIVA_IRQ_GPIOT_1 (_NGPIOSIRQS + 1)
#  define TIVA_IRQ_GPIOT_2 (_NGPIOSIRQS + 2)
#  define TIVA_IRQ_GPIOT_3 (_NGPIOSIRQS + 3)
#  define TIVA_IRQ_GPIOT_4 (_NGPIOSIRQS + 4)
#  define TIVA_IRQ_GPIOT_5 (_NGPIOSIRQS + 5)
#  define TIVA_IRQ_GPIOT_6 (_NGPIOSIRQS + 6)
#  define TIVA_IRQ_GPIOT_7 (_NGPIOSIRQS + 7)
#  define _NGPIOTIRQS      (_NGPIOSIRQS + 8)
#else
#  define _NGPIOTIRQS      _NGPIOSIRQS
#endif

#define NR_GPIO_IRQS       (_NGPIOTIRQS - NR_IRQS)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_TIVA_IRQ_H */
