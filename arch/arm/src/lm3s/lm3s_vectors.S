/************************************************************************************
 * arch/arm/src/lm3s/lm3s_vectors.S
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

#ifndef CONFIG_DEBUG
#  define lm3s_uncaught 0
#endif

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.globl		_vectors
	.globl		lm3s_pendsvhandler
	.globl		lm3s_systickhandler
	.globl		lm3s_faulthandler
	.globl		lm3s_portsvchandler

/************************************************************************************
 * Macros
 ************************************************************************************/

	.macro	HANDLER, label, irqno
\label:
#warning "Missing logic"
	.endm

/************************************************************************************
 * Vectors
 ************************************************************************************/

	.section	.vectors, "ax"
	.code		16
	.align		0

_vectors:
/* Processor Exceptions */
	.word	__stack_end__		/* Vector 0: Reset stack pointer */
	.word	_start			/* Vector 1: Reset vector */
	.word	lm3s_nmihandler		/* Vector 2: Non-Maskable Interrupt (NMI) */
	.word	lm3s_faulthandler	/* Vector 3: Hard fault */
	.word	lm3s_uncaught		/* Vector 4: Memory management (MPU) */
	.word	lm3s_uncaught		/* Vector 5: Bus fault */
	.word	lm3s_uncaught		/* Vector 6: Usage fault */
	.word	lm3s_uncaught		/* Vector 7: Reserved */
	.word	lm3s_uncaught		/* Vector 8: Reserved */
	.word	lm3s_uncaught		/* Vector 9: Reserved */
	.word	lm3s_uncaught		/* Vector 10: Reserved */
	.word	lm3s_svchandler		/* Vector 11: SVC call */
	.word	lm3s_uncaught		/* Vector 12: Debug monitor */
	.word	lm3s_uncaught		/* Vector 13: Reserved */
	.word	lm3s_pendsvhandler	/* Vector 14: Pendable system service request */
	.word	lm3s_systickhandler	/* Vector 15: System tick */

/* External Interrupts */

#ifdef CONFIG_ARCH_CHIP_LM3S6918
	.word	lm3s_gpioahandler	/* Vector 16: GPIO Port A */
	.word	lm3s_gpiobhandler	/* Vector 17: GPIO Port B */
	.word	lm3s_gpiodhandler	/* Vector 18: GPIO Port C */
	.word	lm3s_gpioehandler	/* Vector 19: GPIO Port D */
	.word	lm3s_gpioehandler	/* Vector 20: GPIO Port E */
	.word	lm3s_uart0handler	/* Vector 21: UART 0 */
	.word	lm3s_uart1handler	/* Vector 22: UART 1 */
	.word	lm3s_ssi0handler	/* Vector 23: SSI 0 */
	.word	lm3s_i2c0handler	/* Vector 24: I2C 0 */
	.word	lm3s_uncaught		/* Vector 25: Reserved */
	.word	lm3s_uncaught		/* Vector 26: Reserved */
	.word	lm3s_uncaught		/* Vector 27: Reserved */
	.word	lm3s_uncaught		/* Vector 28: Reserved */
	.word	lm3s_uncaught		/* Vector 29: Reserved */
	.word	lm3s_adc0handler	/* Vector 30: ADC Sequence 0 */
	.word	lm3s_adc1handler	/* Vector 31: ADC Sequence 1 */
	.word	lm3s_adc2handler	/* Vector 32: ADC Sequence 2 */
	.word	lm3s_adc3handler	/* Vector 33: ADC Sequence 3 */
	.word	lm3s_wdoghandler	/* Vector 34: Watchdog Timer */
	.word	lm3s_tmr0ahandler	/* Vector 35: Timer 0 A */
	.word	lm3s_tmr0bhandler	/* Vector 36: Timer 0 B */
	.word	lm3s_tmr1ahandler	/* Vector 37: Timer 1 A */
	.word	lm3s_tmr1bhandler	/* Vector 38: Timer 1 B */
	.word	lm3s_tmr2ahandler	/* Vector 39: Timer 2 A */
	.word	lm3s_tmr2bhandler	/* Vector 40: Timer 3 B */
	.word	lm3s_cmp0handler	/* Vector 41: Analog Comparator 0 */
	.word	lm3s_dmp1handler	/* Vector 42: Analog Comparator 1 */
	.word	lm3s_uncaught		/* Vector 43: Reserved */
	.word	lm3s_sysconhandler	/* Vector 44: System Control */
	.word	lm3s_flashconhandler	/* Vector 45: FLASH Control */
	.word	lm3s_gpiofhandler	/* Vector 46: GPIO Port F */
	.word	lm3s_gpioghandler	/* Vector 47: GPIO Port G */
	.word	lm3s_gpiohhandler	/* Vector 48: GPIO Port H */
	.word	lm3s_uncaught		/* Vector 49: Reserved */
	.word	lm3s_ssi1handler	/* Vector 50: SSI 1 */
	.word	lm3s_tmr3ahandler	/* Vector 51: Timer 3 A */
	.word	lm3s_tmr3bhandler	/* Vector 52: Timer 3 B */
	.word	lm3s_i2c1handler	/* Vector 53: I2C 1 */
	.word	lm3s_uncaught		/* Vector 54: Reserved */
	.word	lm3s_uncaught		/* Vector 55: Reserved */
	.word	lm3s_uncaught		/* Vector 56: Reserved */
	.word	lm3s_uncaught		/* Vector 57: Reserved */
 	.word	lm3s_ethhandler		/* Vector 58: Ethernet Controller */
	.word	lm3s_hibhandler		/* Vector 59: Hibernation Module */
	.word	lm3s_uncaught		/* Vector 60: Reserved */
	.word	lm3s_uncaught		/* Vector 61: Reserved */
	.word	lm3s_uncaught		/* Vector 62: Reserved */
	.word	lm3s_uncaught		/* Vector 63: Reserved */
	.word	lm3s_uncaught		/* Vector 64: Reserved */
	.word	lm3s_uncaught		/* Vector 65: Reserved */
	.word	lm3s_uncaught		/* Vector 66: Reserved */
	.word	lm3s_uncaught		/* Vector 67: Reserved */
	.word	lm3s_uncaught		/* Vector 68: Reserved */
	.word	lm3s_uncaught		/* Vector 69: Reserved */
	.word	lm3s_uncaught		/* Vector 70: Reserved */
#else
#  error "Vectors not specified for this LM3S chip"
#endif
	.text
	.thumb_func
#ifdef CONFIG_ARCH_CHIP_LM3S6918
	HANDLER	lm3s_gpioahandler, LM3S_IRQ_GPIOA	/* Vector 16: GPIO Port A */
	HANDLER	lm3s_gpiobhandler, LM3S_IRQ_GPIOB	/* Vector 17: GPIO Port B */
	HANDLER	lm3s_gpiodhandler, LM3S_IRQ_GPIOC	/* Vector 18: GPIO Port C */
	HANDLER	lm3s_gpioehandler, LM3S_IRQ_GPIOD	/* Vector 19: GPIO Port D */
	HANDLER	lm3s_gpioehandler, LM3S_IRQ_GPIOE	/* Vector 20: GPIO Port E */
	HANDLER	lm3s_uart0handler, LM3S_IRQ_UART0	/* Vector 21: UART 0 */
	HANDLER	lm3s_uart1handler, LM3S_IRQ_UART1	/* Vector 22: UART 1 */
	HANDLER	lm3s_ssi0handler, LM3S_IRQ_SSI0		/* Vector 23: SSI 0 */
	HANDLER	lm3s_i2c0handler, LM3S_IRQ_I2C0		/* Vector 24: I2C 0 */
	HANDLER	lm3s_adc0handler, LM3S_IRQ_ADC0		/* Vector 30: ADC Sequence 0 */
	HANDLER	lm3s_adc1handler, LM3S_IRQ_ADC1		/* Vector 31: ADC Sequence 1 */
	HANDLER	lm3s_adc2handler, LM3S_IRQ_ADC2		/* Vector 32: ADC Sequence 2 */
	HANDLER	lm3s_adc3handler, LM3S_IRQ_ADC3		/* Vector 33: ADC Sequence 3 */
	HANDLER	lm3s_wdoghandler, LM3S_IRQ_WDOG		/* Vector 34: Watchdog Timer */
	HANDLER	lm3s_tmr0ahandler, LM3S_IRQ_TIMER0A	/* Vector 35: Timer 0 A */
	HANDLER	lm3s_tmr0bhandler, LM3S_IRQ_TIMER0B	/* Vector 36: Timer 0 B */
	HANDLER	lm3s_tmr1ahandler, LM3S_IRQ_TIMER1A	/* Vector 37: Timer 1 A */
	HANDLER	lm3s_tmr1bhandler, LM3S_IRQ_TIMER1B	/* Vector 38: Timer 1 B */
	HANDLER	lm3s_tmr2ahandler, LM3S_IRQ_TIMER2A	/* Vector 39: Timer 2 A */
	HANDLER	lm3s_tmr2bhandler, LM3S_IRQ_TIMER2B	/* Vector 40: Timer 3 B */
	HANDLER	lm3s_cmp0handler, LM3S_IRQ_COMPARE0	/* Vector 41: Analog Comparator 0 */
	HANDLER	lm3s_dmp1handler, LM3S_IRQ_COMPARE1	/* Vector 42: Analog Comparator 1 */
	HANDLER	lm3s_sysconhandler, LM3S_IRQ_SYSCON	/* Vector 44: System Control */
	HANDLER	lm3s_flashconhandler, LM3S_IRQ_FLASHCON	/* Vector 45: FLASH Control */
	HANDLER	lm3s_gpiofhandler, LM3S_IRQ_GPIOF	/* Vector 46: GPIO Port F */
	HANDLER	lm3s_gpioghandler, LM3S_IRQ_GPIOG	/* Vector 47: GPIO Port G */
	HANDLER	lm3s_gpiohhandler, LM3S_IRQ_GPIOH	/* Vector 48: GPIO Port H */
	HANDLER	lm3s_ssi1handler, LM3S_IRQ_SSI1		/* Vector 50: SSI 1 */
	HANDLER	lm3s_tmr3ahandler, LM3S_IRQ_TIMER3A	/* Vector 51: Timer 3 A */
	HANDLER	lm3s_tmr3bhandler, LM3S_IRQ_TIMER3B	/* Vector 52: Timer 3 B */
	HANDLER	lm3s_i2c1handler, LM3S_IRQ_I2C1		/* Vector 53: I2C 1 */
 	HANDLER	lm3s_ethhandler	, LM3S_IRQ_ETHCON	/* Vector 58: Ethernet Controller */
	HANDLER	lm3s_hibhandler	, LM3S_IRQ_HIBERNATE	/* Vector 59: Hibernation Module */
#else
#  error "Vectors not specified for this LM3S chip"
#endif

#ifdef CONFIG_DEBUG
	HANDLER	lm3s_uncaught, NR_IRQS			/* Unexpected vector (default is to reset) */
#endif
	.end
