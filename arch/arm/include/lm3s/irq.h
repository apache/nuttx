/************************************************************************************
 * arch/arm/include/lm3s/irq.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_LM3S_IRQ_H
#define __ARCH_ARM_INCLUDE_LM3S_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds to the bit number in interrupt registers.
 * This includes all externally generated interrupts (but excludes processor
 * exceptions)
 */

#ifdef CONFIG_CHIP_LM3S6918

                                  /* Vector 0-15: Processor exceptions */
#  define LM3S_IRQ_GPIOA      (0) /* Vector 16: GPIO Port A */
#  define LM3S_IRQ_GPIOB      (1) /* Vector 17: GPIO Port B */
#  define LM3S_IRQ_GPIOC      (2) /* Vector 18: GPIO Port C */
#  define LM3S_IRQ_GPIOD      (3) /* Vector 19: GPIO Port D */
#  define LM3S_IRQ_GPIOE      (4) /* Vector 20: GPIO Port E */
#  define LM3S_IRQ_UART0      (5) /* Vector 21: UART 0 */
#  define LM3S_IRQ_UART1      (6) /* Vector 22: UART 1 */
#  define LM3S_IRQ_SSI0       (7) /* Vector 23: SSI 0 */
#  define LM3S_IRQ_I2C0       (8) /* Vector 24: I2C 0 */
                                  /* Vector 25-29: Reserved */
#  define LM3S_IRQ_ADC0      (14) /* Vector 30: ADC Sequence 0 */
#  define LM3S_IRQ_ADC1      (15) /* Vector 31: ADC Sequence 1 */
#  define LM3S_IRQ_ADC2      (16) /* Vector 32: ADC Sequence 2 */
#  define LM3S_IRQ_ADC3      (17) /* Vector 33: ADC Sequence 3 */
#  define LM3S_IRQ_WDOG      (18) /* Vector 34: Watchdog Timer */
#  define LM3S_IRQ_TIMER0A   (19) /* Vector 35: Timer 0 A */
#  define LM3S_IRQ_TIMER0B   (20) /* Vector 36: Timer 0 B */
#  define LM3S_IRQ_TIMER1A   (21) /* Vector 37: Timer 1 A */
#  define LM3S_IRQ_TIMER1B   (22) /* Vector 38: Timer 1 B */
#  define LM3S_IRQ_TIMER2A   (23) /* Vector 39: Timer 2 A */
#  define LM3S_IRQ_TIMER2B   (24) /* Vector 40: Timer 3 B */
#  define LM3S_IRQ_COMPARE0  (25) /* Vector 41: Analog Comparator 0 */
#  define LM3S_IRQ_COMPARE1  (26) /* Vector 42: Analog Comparator 1 */
                                  /* Vector 43: Reserved */
#  define LM3S_IRQ_SYSCON    (28) /* Vector 44: System Control */
#  define LM3S_IRQ_FLASHCON  (29) /* Vector 45: FLASH Control */
#  define LM3S_IRQ_GPIOF     (30) /* Vector 46: GPIO Port F */
#  define LM3S_IRQ_GPIOG     (31) /* Vector 47: GPIO Port G */
#  define LM3S_IRQ_GPIOH     (32) /* Vector 48: GPIO Port H */
                                  /* Vector 49: Reserved */
#  define LM3S_IRQ_SSI1      (34) /* Vector 50: SSI 1 */
#  define LM3S_IRQ_TIMER3A   (35) /* Vector 51: Timer 3 A */
#  define LM3S_IRQ_TIMER3B   (36) /* Vector 52: Timer 3 B */
#  define LM3S_IRQ_I2C1      (37) /* Vector 53: I2C 1 */
                                  /* Vectors 54-57: Reserved */
#  define LM3S_IRQ_ETHCON    (42) /* Vector 58: Ethernet Controller */
#  define LM3S_IRQ_HIBERNATE (43) /* Vector 59: Hibernation Module */
                                  /* Vectors 60-70: Reserved */
#else
#  error "IRQ Numbers not specified for this LM3S chip"
#endif

#define LM3S_IRQ_SYSTIMER    too-be-determined
#define NR_IRQS              (44)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_LM3S_IRQ_H */

