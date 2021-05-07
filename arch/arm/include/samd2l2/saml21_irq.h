/****************************************************************************
 * arch/arm/include/samd2l2/saml21_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_SAMD2L2_SAML21_IRQ_H
#define __ARCH_ARM_INCLUDE_SAMD2L2_SAML21_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* External interrupts */

#define SAM_IRQ_PM         (SAM_IRQ_INTERRUPT + 0)   /* Power Manager */
#define SAM_IRQ_MCLK       (SAM_IRQ_INTERRUPT + 0)   /* Main Clock */
#define SAM_IRQ_OSCCTRL    (SAM_IRQ_INTERRUPT + 0)   /* Oscillators Controller */
#define SAM_IRQ_OSC32KCTRL (SAM_IRQ_INTERRUPT + 0)   /* 32KHz scillators Controller */
#define SAM_IRQ_SUPC       (SAM_IRQ_INTERRUPT + 0)   /* Supply Controller */
#define SAM_IRQ_PACC       (SAM_IRQ_INTERRUPT + 0)   /* Protection Access Controller */
#define SAM_IRQ_WDT        (SAM_IRQ_INTERRUPT + 1)   /* Watchdog Timer */
#define SAM_IRQ_RTC        (SAM_IRQ_INTERRUPT + 2)   /* Real Time Counter */
#define SAM_IRQ_EIC        (SAM_IRQ_INTERRUPT + 3)   /* External Interrupt Controller */
#define SAM_IRQ_NVMCTRL    (SAM_IRQ_INTERRUPT + 4)   /* Non-Volatile Memory Controller */
#define SAM_IRQ_DMAC       (SAM_IRQ_INTERRUPT + 5)   /* Direct Memory Access Controller */
#define SAM_IRQ_USB        (SAM_IRQ_INTERRUPT + 6)   /* Universal Serial Bus */
#define SAM_IRQ_EVSYS      (SAM_IRQ_INTERRUPT + 7)   /* Event System */
#define SAM_IRQ_SERCOM0    (SAM_IRQ_INTERRUPT + 8)   /* Serial Communication Interface 0 */
#define SAM_IRQ_SERCOM1    (SAM_IRQ_INTERRUPT + 9)   /* Serial Communication Interface 1 */
#define SAM_IRQ_SERCOM2    (SAM_IRQ_INTERRUPT + 10)  /* Serial Communication Interface 2 */
#define SAM_IRQ_SERCOM3    (SAM_IRQ_INTERRUPT + 11)  /* Serial Communication Interface 3 */
#define SAM_IRQ_SERCOM4    (SAM_IRQ_INTERRUPT + 12)  /* Serial Communication Interface 4 */
#define SAM_IRQ_SERCOM5    (SAM_IRQ_INTERRUPT + 13)  /* Serial Communication Interface 5 */
#define SAM_IRQ_TCC0       (SAM_IRQ_INTERRUPT + 14)  /* Timer/Counter for Control 0 */
#define SAM_IRQ_TCC1       (SAM_IRQ_INTERRUPT + 15)  /* Timer/Counter for Control 1 */
#define SAM_IRQ_TCC2       (SAM_IRQ_INTERRUPT + 16)  /* Timer/Counter for Control 2 */
#define SAM_IRQ_TC0        (SAM_IRQ_INTERRUPT + 17)  /* Timer/Counter 0 */
#define SAM_IRQ_TC1        (SAM_IRQ_INTERRUPT + 18)  /* Timer/Counter 1 */
#define SAM_IRQ_TC2        (SAM_IRQ_INTERRUPT + 19)  /* Timer/Counter 2 */
#define SAM_IRQ_TC3        (SAM_IRQ_INTERRUPT + 20)  /* Timer/Counter 3 */
#define SAM_IRQ_TC4        (SAM_IRQ_INTERRUPT + 21)  /* Timer/Counter 4 */
#define SAM_IRQ_ADC        (SAM_IRQ_INTERRUPT + 22)  /* Analog-to-Digital Converter */
#define SAM_IRQ_AC         (SAM_IRQ_INTERRUPT + 23)  /* Analog Comparator */
#define SAM_IRQ_DAC        (SAM_IRQ_INTERRUPT + 24)  /* Digital-to-Analog Converter */
#define SAM_IRQ_PTC        (SAM_IRQ_INTERRUPT + 25)  /* Peripheral Touch Controller */
#define SAM_IRQ_AES        (SAM_IRQ_INTERRUPT + 26)  /* Advanced Encryption Standard Module */
#define SAM_IRQ_TRNG       (SAM_IRQ_INTERRUPT + 27)  /* True Random Number Generator */

#define SAM_IRQ_NINTS      (28)                      /* Total number of interrupts */
#define SAM_IRQ_NIRQS      (SAM_IRQ_INTERRUPT + 28)  /* The number of real interrupts */

/* GPIO interrupts.  Up to 16 pins may be configured to support interrupts */

#ifdef CONFIG_SAMD2L2_GPIOIRQ
#  define SAM_IRQ_EXTINT0  (SAM_IRQ_NIRQS + 0)       /* External interrupt 0 */
#  define SAM_IRQ_EXTINT1  (SAM_IRQ_NIRQS + 1)       /* External interrupt 1 */
#  define SAM_IRQ_EXTINT2  (SAM_IRQ_NIRQS + 2)       /* External interrupt 2 */
#  define SAM_IRQ_EXTINT3  (SAM_IRQ_NIRQS + 3)       /* External interrupt 3 */
#  define SAM_IRQ_EXTINT4  (SAM_IRQ_NIRQS + 4)       /* External interrupt 4 */
#  define SAM_IRQ_EXTINT5  (SAM_IRQ_NIRQS + 5)       /* External interrupt 5 */
#  define SAM_IRQ_EXTINT6  (SAM_IRQ_NIRQS + 6)       /* External interrupt 6 */
#  define SAM_IRQ_EXTINT7  (SAM_IRQ_NIRQS + 7)       /* External interrupt 7 */
#  define SAM_IRQ_EXTINT8  (SAM_IRQ_NIRQS + 8)       /* External interrupt 8 */
#  define SAM_IRQ_EXTINT9  (SAM_IRQ_NIRQS + 9)       /* External interrupt 9 */
#  define SAM_IRQ_EXTINT10 (SAM_IRQ_NIRQS + 10)      /* External interrupt 10 */
#  define SAM_IRQ_EXTINT11 (SAM_IRQ_NIRQS + 11)      /* External interrupt 11 */
#  define SAM_IRQ_EXTINT12 (SAM_IRQ_NIRQS + 12)      /* External interrupt 12 */
#  define SAM_IRQ_EXTINT13 (SAM_IRQ_NIRQS + 13)      /* External interrupt 13 */
#  define SAM_IRQ_EXTINT14 (SAM_IRQ_NIRQS + 14)      /* External interrupt 14 */
#  define SAM_IRQ_EXTINT15 (SAM_IRQ_NIRQS + 15)      /* External interrupt 15 */
#  define SAM_IRQ_NEXTINTS 16
#else
#  define SAM_IRQ_NEXTINTS 0
#endif

/* Total number of IRQ numbers */

#define NR_IRQS            (SAM_IRQ_INTERRUPT + SAM_IRQ_NINTS + SAM_IRQ_NEXTINTS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
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
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_SAMD2L2_SAML21_IRQ_H */
