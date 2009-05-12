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

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define LMSB_IRQ_RESERVED     (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                  /* Vector  0: Reset stack pointer value */
                                  /* Vector  1: Reset (not handler as an IRQ) */
#define LMSB_IRQ_NMI          (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define LMSB_IRQ_HARDFAULT    (3) /* Vector  3: Hard fault */
#define LMSB_IRQ_MPU          (4) /* Vector  4: Memory management (MPU) */
#define LMSB_IRQ_BUSFAULT     (5) /* Vector  5: Bus fault */
#define LMSB_IRQ_USAGEFAULT   (6) /* Vector  6: Usage fault */
#define LMSB_IRQ_SVCALL      (11) /* Vector 11: SVC call */
#define LMSB_IRQ_DBGMONITOR  (12) /* Vector 12: Debug Monitor */
                                  /* Vector 13: Reserved */
#define LMSB_IRQ_PENDSV      (14) /* Vector 14: Pendable system service request */
#define LMSB_IRQ_SYSTICK     (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define LM3S_IRQ_INTERRUPTS  (16) /* Vector number of the first external interrupt */
#ifdef CONFIG_ARCH_CHIP_LM3S6918

#  define LM3S_IRQ_GPIOA     (16) /* Vector 16: GPIO Port A */
#  define LM3S_IRQ_GPIOB     (17) /* Vector 17: GPIO Port B */
#  define LM3S_IRQ_GPIOC     (18) /* Vector 18: GPIO Port C */
#  define LM3S_IRQ_GPIOD     (19) /* Vector 19: GPIO Port D */
#  define LM3S_IRQ_GPIOE     (20) /* Vector 20: GPIO Port E */
#  define LM3S_IRQ_UART0     (21) /* Vector 21: UART 0 */
#  define LM3S_IRQ_UART1     (22) /* Vector 22: UART 1 */
#  define LM3S_IRQ_SSI0      (23) /* Vector 23: SSI 0 */
#  define LM3S_IRQ_I2C0      (24) /* Vector 24: I2C 0 */
                                  /* Vector 25-29: Reserved */
#  define LM3S_IRQ_ADC0      (30) /* Vector 30: ADC Sequence 0 */
#  define LM3S_IRQ_ADC1      (31) /* Vector 31: ADC Sequence 1 */
#  define LM3S_IRQ_ADC2      (32) /* Vector 32: ADC Sequence 2 */
#  define LM3S_IRQ_ADC3      (33) /* Vector 33: ADC Sequence 3 */
#  define LM3S_IRQ_WDOG      (34) /* Vector 34: Watchdog Timer */
#  define LM3S_IRQ_TIMER0A   (35) /* Vector 35: Timer 0 A */
#  define LM3S_IRQ_TIMER0B   (36) /* Vector 36: Timer 0 B */
#  define LM3S_IRQ_TIMER1A   (37) /* Vector 37: Timer 1 A */
#  define LM3S_IRQ_TIMER1B   (38) /* Vector 38: Timer 1 B */
#  define LM3S_IRQ_TIMER2A   (39) /* Vector 39: Timer 2 A */
#  define LM3S_IRQ_TIMER2B   (40) /* Vector 40: Timer 3 B */
#  define LM3S_IRQ_COMPARE0  (41) /* Vector 41: Analog Comparator 0 */
#  define LM3S_IRQ_COMPARE1  (42) /* Vector 42: Analog Comparator 1 */
                                  /* Vector 43: Reserved */
#  define LM3S_IRQ_SYSCON    (44) /* Vector 44: System Control */
#  define LM3S_IRQ_FLASHCON  (45) /* Vector 45: FLASH Control */
#  define LM3S_IRQ_GPIOF     (46) /* Vector 46: GPIO Port F */
#  define LM3S_IRQ_GPIOG     (47) /* Vector 47: GPIO Port G */
#  define LM3S_IRQ_GPIOH     (48) /* Vector 48: GPIO Port H */
                                  /* Vector 49: Reserved */
#  define LM3S_IRQ_SSI1      (50) /* Vector 50: SSI 1 */
#  define LM3S_IRQ_TIMER3A   (51) /* Vector 51: Timer 3 A */
#  define LM3S_IRQ_TIMER3B   (52) /* Vector 52: Timer 3 B */
#  define LM3S_IRQ_I2C1      (53) /* Vector 53: I2C 1 */
                                  /* Vectors 54-57: Reserved */
#  define LM3S_IRQ_ETHCON    (58) /* Vector 58: Ethernet Controller */
#  define LM3S_IRQ_HIBERNATE (59) /* Vector 59: Hibernation Module */
                                  /* Vectors 60-70: Reserved */
#else
#  error "IRQ Numbers not specified for this LM3S chip"
#endif

#define NR_IRQS              (60) /* Really only 43 */

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

