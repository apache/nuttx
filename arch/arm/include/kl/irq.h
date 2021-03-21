/****************************************************************************
 * arch/arm/include/kl/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_KL_IRQ_H
#define __ARCH_ARM_INCLUDE_KL_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers **************************************************************/

/* The IRQ numbers corresponds directly to vector numbers and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define KL_IRQ_RESERVED      (0)   /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                   /* Vector  0: Reset stack pointer value */
                                   /* Vector  1: Reset (not handler as an IRQ) */
#define KL_IRQ_NMI           (2)   /* Vector  2: Non-Maskable Interrupt (NMI) */
#define KL_IRQ_HARDFAULT     (3)   /* Vector  3: Hard fault */
                                   /* Vectors 4-10: Reserved */
#define KL_IRQ_SVCALL        (11)  /* Vector 11: SVC call */
                                   /* Vector 12-13: Reserved */
#define KL_IRQ_PENDSV        (14)  /* Vector 14: Pendable system service request */
#define KL_IRQ_SYSTICK       (15)  /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define KL_IRQ_EXTINT        (16)

/* K40 Family ***************************************************************
 *
 * The interrupt vectors  for the following parts is defined in Freescale
 * document K40P144M100SF2RM
 */

#if defined(CONFIG_ARCH_CHIP_MKL25Z128) || defined(CONFIG_ARCH_CHIP_MKL25Z64)

#  define KL_IRQ_DMACH0        (16)  /* Vector 16: DMA channel 0 transfer complete */
#  define KL_IRQ_DMACH1        (17)  /* Vector 17: DMA channel 1 transfer complete */
#  define KL_IRQ_DMACH2        (18)  /* Vector 18: DMA channel 2 transfer complete */
#  define KL_IRQ_DMACH3        (19)  /* Vector 19: DMA channel 3 transfer complete */
                                     /* Vector 20: Reserved */
#  define KL_IRQ_FTFA          (21)  /* Vector 21: FTFA */
#  define KL_IRQ_LVDLVW        (22)  /* Vector 22: LVD_LVW */
#  define KL_IRQ_LLW           (23)  /* Vector 23: LLW */
#  define KL_IRQ_I2C0          (24)  /* Vector 24: I2C0 */
#  define KL_IRQ_I2C1          (25)  /* Vector 25: I2C1 */
#  define KL_IRQ_SPI0          (26)  /* Vector 26: SPI0 */
#  define KL_IRQ_SPI1          (27)  /* Vector 27: SPI1 */
#  define KL_IRQ_UART0         (28)  /* Vector 28: UART0 */
#  define KL_IRQ_UART1         (29)  /* Vector 29: UART1 */
#  define KL_IRQ_UART2         (30)  /* Vector 30: UART2 */
#  define KL_IRQ_ADC0          (31)  /* Vector 31: Analog Device Converter 0 */
#  define KL_IRQ_CMP0          (32)  /* Vector 32: Comparator 0 */
#  define KL_IRQ_TPM0          (33)  /* Vector 33: Timer/PWM Module 0 */
#  define KL_IRQ_TPM1          (34)  /* Vector 34: Timer/PWM Module 1 */
#  define KL_IRQ_TPM2          (35)  /* Vector 35: Timer/PWM Module 2 */
#  define KL_IRQ_RTC           (36)  /* Vector 36: Realtime Clock */
#  define KL_IRQ_RTCSEC        (37)  /* Vector 37: Realtime Clock, seconds interrupt */
#  define KL_IRQ_PIT           (38)  /* Vector 38: Programmable Interrupt Timer */
                                     /* Vector 39: Reserved */
#  define KL_IRQ_USB0          (40)  /* Vector 40: USB0 */
#  define KL_IRQ_DAC0          (41)  /* Vector 41: Digital Analog Converter 0 */
#  define KL_IRQ_TSI0          (42)  /* Vector 42: TSI0 */
#  define KL_IRQ_MCG           (43)  /* Vector 43: MCG */
#  define KL_IRQ_LPTIMER       (44)  /* Vector 44: Low Power Timer */
                                     /* Vector 45: Reserved */
#  define KL_IRQ_PORTA         (46)  /* Vector 46: GPIO Port A */
#  define KL_IRQ_PORTD         (47)  /* Vector 47: GPIO Port D */

/* Note that the total number of IRQ numbers supported is equal to the
 * number of valid interrupt vectors.  This is wasteful in that certain
 * tables are sized by this value.  There are only 94 valid interrupts so,
 * potentially the number of IRQs to could be reduced to 94.  However,
 * equating IRQ numbers with vector numbers also simplifies operations on
 * NVIC registers and (at least in my state of mind now) seems to justify
 * the waste.
 */

#  define NR_IRQS              (48) /* 64 interrupts but 48 IRQ numbers */

#elif defined(CONFIG_ARCH_CHIP_MKL26Z128)

#  define KL_IRQ_DMACH0        (16)  /* Vector 16: DMA channel 0 transfer complete */
#  define KL_IRQ_DMACH1        (17)  /* Vector 17: DMA channel 1 transfer complete */
#  define KL_IRQ_DMACH2        (18)  /* Vector 18: DMA channel 2 transfer complete */
#  define KL_IRQ_DMACH3        (19)  /* Vector 19: DMA channel 3 transfer complete */
                                     /* Vector 20: Reserved */
#  define KL_IRQ_FTFA          (21)  /* Vector 21: FTFA */
#  define KL_IRQ_LVDLVW        (22)  /* Vector 22: LVD_LVW */
#  define KL_IRQ_LLW           (23)  /* Vector 23: LLW */
#  define KL_IRQ_I2C0          (24)  /* Vector 24: I2C0 */
#  define KL_IRQ_I2C1          (25)  /* Vector 25: I2C1 */
#  define KL_IRQ_SPI0          (26)  /* Vector 26: SPI0 */
#  define KL_IRQ_SPI1          (27)  /* Vector 27: SPI1 */
#  define KL_IRQ_UART0         (28)  /* Vector 28: UART0 */
#  define KL_IRQ_UART1         (29)  /* Vector 29: UART1 */
#  define KL_IRQ_UART2         (30)  /* Vector 30: UART2 */
#  define KL_IRQ_ADC0          (31)  /* Vector 31: Analog Device Converter 0 */
#  define KL_IRQ_CMP0          (32)  /* Vector 32: Comparator 0 */
#  define KL_IRQ_TPM0          (33)  /* Vector 33: Timer/PWM Module 0 */
#  define KL_IRQ_TPM1          (34)  /* Vector 34: Timer/PWM Module 1 */
#  define KL_IRQ_TPM2          (35)  /* Vector 35: Timer/PWM Module 2 */
#  define KL_IRQ_RTC           (36)  /* Vector 36: Realtime Clock */
#  define KL_IRQ_RTCSEC        (37)  /* Vector 37: Realtime Clock, seconds interrupt */
#  define KL_IRQ_PIT           (38)  /* Vector 38: Programmable Interrupt Timer */
                                     /* Vector 39: Reserved */
#  define KL_IRQ_USB0          (40)  /* Vector 40: USB0 */
#  define KL_IRQ_DAC0          (41)  /* Vector 41: Digital Analog Converter 0 */
#  define KL_IRQ_TSI0          (42)  /* Vector 42: TSI0 */
#  define KL_IRQ_MCG           (43)  /* Vector 43: MCG */
#  define KL_IRQ_LPTIMER       (44)  /* Vector 44: Low Power Timer */
                                     /* Vector 45: Reserved */
#  define KL_IRQ_PORTA         (46)  /* Vector 46: GPIO Port A */
#  define KL_IRQ_PORTD         (47)  /* Vector 47: GPIO Port D */

/* Note that the total number of IRQ numbers supported is equal to the
 * number of valid interrupt vectors.  This is wasteful in that certain
 * tables are sized by this value.  There are only 94 valid interrupts so,
 * potentially the number of IRQs to could be reduced to 94.  However,
 * equating IRQ numbers with vector numbers also simplifies operations on
 * NVIC registers and (at least in my state of mind now) seems to justify
 * the waste.
 */

#  define NR_IRQS              (48) /* 64 interrupts but 48 IRQ numbers */

#else
/* The interrupt vectors for other parts are defined in other documents and
 * may or may not be the same as above (the family members are all very
 * similar)  This error just means that you have to look at the document and
 * determine for yourself if the vectors are the same.
 */

#  error "No IRQ numbers for this Kinetis L part"
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

#endif /* __ARCH_ARM_INCLUDE_KL_IRQ_H */
