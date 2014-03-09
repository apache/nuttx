/************************************************************************************
 * arch/arm/src/tiva/chip/lm3s_vectors.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

/************************************************************************************
 * Vectors
 ************************************************************************************/

/* This file is included by tiva_vectors.S.  It provides the macro VECTOR that
 * supplies ach Stellaris vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/tiva/lm3s_irq.h.
 * tiva_vectors.S will define the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

#if defined(CONFIG_ARCH_CHIP_LM3S6918)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 71 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 71

#else

VECTOR(tiva_gpioa, TIVA_IRQ_GPIOA)       /* Vector 16: GPIO Port A */
VECTOR(tiva_gpiob, TIVA_IRQ_GPIOB)       /* Vector 17: GPIO Port B */
VECTOR(tiva_gpioc, TIVA_IRQ_GPIOC)       /* Vector 18: GPIO Port C */
VECTOR(tiva_gpiod, TIVA_IRQ_GPIOD)       /* Vector 19: GPIO Port D */

VECTOR(tiva_gpioe, TIVA_IRQ_GPIOE)       /* Vector 20: GPIO Port E */
VECTOR(tiva_uart0, TIVA_IRQ_UART0)       /* Vector 21: UART 0 */
VECTOR(tiva_uart1, TIVA_IRQ_UART1)       /* Vector 22: UART 1 */
VECTOR(tiva_ssi0, TIVA_IRQ_SSI0)         /* Vector 23: SSI 0 */
VECTOR(tiva_i2c0, TIVA_IRQ_I2C0)         /* Vector 24: I2C 0 */
UNUSED(TIVA_RESERVED_25)                 /* Vector 25: Reserved */
UNUSED(TIVA_RESERVED_26)                 /* Vector 26: Reserved */
UNUSED(TIVA_RESERVED_27)                 /* Vector 27: Reserved */
UNUSED(TIVA_RESERVED_28)                 /* Vector 28: Reserved */
UNUSED(TIVA_RESERVED_29)                 /* Vector 29: Reserved */

VECTOR(tiva_adc0, TIVA_IRQ_ADC0)         /* Vector 30: ADC Sequence 0 */
VECTOR(tiva_adc1, TIVA_IRQ_ADC1)         /* Vector 31: ADC Sequence 1 */
VECTOR(tiva_adc2, TIVA_IRQ_ADC2)         /* Vector 32: ADC Sequence 2 */
VECTOR(tiva_adc3, TIVA_IRQ_ADC3)         /* Vector 33: ADC Sequence 3 */
VECTOR(tiva_wdog, TIVA_IRQ_WDOG)         /* Vector 34: Watchdog Timer */
VECTOR(tiva_tmr0a, TIVA_IRQ_TIMER0A)     /* Vector 35: Timer 0 A */
VECTOR(tiva_tmr0b, TIVA_IRQ_TIMER0B)     /* Vector 36: Timer 0 B */
VECTOR(tiva_tmr1a, TIVA_IRQ_TIMER1A)     /* Vector 37: Timer 1 A */
VECTOR(tiva_tmr1b, TIVA_IRQ_TIMER1B)     /* Vector 38: Timer 1 B */
VECTOR(tiva_tmr2a, TIVA_IRQ_TIMER2A)     /* Vector 39: Timer 2 A */

VECTOR(tiva_tmr2b, TIVA_IRQ_TIMER2B)     /* Vector 40: Timer 3 B */
VECTOR(tiva_cmp0, TIVA_IRQ_COMPARE0)     /* Vector 41: Analog Comparator 0 */
VECTOR(tiva_cmp1, TIVA_IRQ_COMPARE1)     /* Vector 42: Analog Comparator 1 */
UNUSED(TIVA_RESERVED_43)                 /* Vector 43: Reserved */
VECTOR(tiva_syscon, TIVA_IRQ_SYSCON)     /* Vector 44: System Control */
VECTOR(tiva_flashcon, TIVA_IRQ_FLASHCON) /* Vector 45: FLASH Control */
VECTOR(tiva_gpiof, TIVA_IRQ_GPIOF)       /* Vector 46: GPIO Port F */
VECTOR(tiva_gpiog, TIVA_IRQ_GPIOG)       /* Vector 47: GPIO Port G */
VECTOR(tiva_gpioh, TIVA_IRQ_GPIOH)       /* Vector 48: GPIO Port H */
UNUSED(TIVA_RESERVED_49)                 /* Vector 49: Reserved */

VECTOR(tiva_ssi1, TIVA_IRQ_SSI1)         /* Vector 50: SSI 1 */
VECTOR(tiva_tmr3a, TIVA_IRQ_TIMER3A)     /* Vector 51: Timer 3 A */
VECTOR(tiva_tmr3b, TIVA_IRQ_TIMER3B)     /* Vector 52: Timer 3 B */
VECTOR(tiva_i2c1, TIVA_IRQ_I2C1)         /* Vector 53: I2C 1 */
UNUSED(TIVA_RESERVED_54)                 /* Vector 54: Reserved */
UNUSED(TIVA_RESERVED_55)                 /* Vector 55: Reserved */
UNUSED(TIVA_RESERVED_56)                 /* Vector 56: Reserved */
UNUSED(TIVA_RESERVED_57)                 /* Vector 57: Reserved */
VECTOR(tiva_eth, TIVA_IRQ_ETHCON)        /* Vector 58: Ethernet Controller */
VECTOR(tiva_hib, TIVA_IRQ_HIBERNATE)     /* Vector 59: Hibernation Module */

UNUSED(TIVA_RESERVED_60)                 /* Vector 60: Reserved */
UNUSED(TIVA_RESERVED_61)                 /* Vector 61: Reserved */
UNUSED(TIVA_RESERVED_62)                 /* Vector 62: Reserved */
UNUSED(TIVA_RESERVED_63)                 /* Vector 63: Reserved */
UNUSED(TIVA_RESERVED_64)                 /* Vector 64: Reserved */
UNUSED(TIVA_RESERVED_65)                 /* Vector 65: Reserved */
UNUSED(TIVA_RESERVED_66)                 /* Vector 66: Reserved */
UNUSED(TIVA_RESERVED_67)                 /* Vector 67: Reserved */
UNUSED(TIVA_RESERVED_68)                 /* Vector 68: Reserved */
UNUSED(TIVA_RESERVED_69)                 /* Vector 69: Reserved */

UNUSED(TIVA_RESERVED_70)                 /* Vector 70: Reserved */
#endif

#elif defined(CONFIG_ARCH_CHIP_LM3S6432)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 71 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 71

#else

VECTOR(tiva_gpioa, TIVA_IRQ_GPIOA)       /* Vector 16: GPIO Port A */
VECTOR(tiva_gpiob, TIVA_IRQ_GPIOB)       /* Vector 17: GPIO Port B */
VECTOR(tiva_gpioc, TIVA_IRQ_GPIOC)       /* Vector 18: GPIO Port C */
VECTOR(tiva_gpiod, TIVA_IRQ_GPIOD)       /* Vector 19: GPIO Port D */

VECTOR(tiva_gpioe, TIVA_IRQ_GPIOE)       /* Vector 20: GPIO Port E */
VECTOR(tiva_uart0, TIVA_IRQ_UART0)       /* Vector 21: UART 0 */
VECTOR(tiva_uart1, TIVA_IRQ_UART1)       /* Vector 22: UART 1 */
VECTOR(tiva_ssi0, TIVA_IRQ_SSI0)         /* Vector 23: SSI 0 */
VECTOR(tiva_i2c0, TIVA_IRQ_I2C0)         /* Vector 24: I2C 0 */
UNUSED(TIVA_RESERVED_25)                 /* Vector 25: Reserved */
VECTOR(tiva_pwm0, TIVA_IRQ_PWM0)         /* Vector 26: PWM Generator 0 */
UNUSED(TIVA_RESERVED_27)                 /* Vector 27: Reserved */
UNUSED(TIVA_RESERVED_28)                 /* Vector 28: Reserved */
UNUSED(TIVA_RESERVED_29)                 /* Vector 29: Reserved */

VECTOR(tiva_adc0, TIVA_IRQ_ADC0)         /* Vector 30: ADC Sequence 0 */
VECTOR(tiva_adc1, TIVA_IRQ_ADC1)         /* Vector 31: ADC Sequence 1 */
VECTOR(tiva_adc2, TIVA_IRQ_ADC2)         /* Vector 32: ADC Sequence 2 */
VECTOR(tiva_adc3, TIVA_IRQ_ADC3)         /* Vector 33: ADC Sequence 3 */
VECTOR(tiva_wdog, TIVA_IRQ_WDOG)         /* Vector 34: Watchdog Timer */
VECTOR(tiva_tmr0a, TIVA_IRQ_TIMER0A)     /* Vector 35: Timer 0 A */
VECTOR(tiva_tmr0b, TIVA_IRQ_TIMER0B)     /* Vector 36: Timer 0 B */
VECTOR(tiva_tmr1a, TIVA_IRQ_TIMER1A)     /* Vector 37: Timer 1 A */
VECTOR(tiva_tmr1b, TIVA_IRQ_TIMER1B)     /* Vector 38: Timer 1 B */
VECTOR(tiva_tmr2a, TIVA_IRQ_TIMER2A)     /* Vector 39: Timer 2 A */

VECTOR(tiva_tmr2b, TIVA_IRQ_TIMER2B)     /* Vector 40: Timer 3 B */
VECTOR(tiva_cmp0, TIVA_IRQ_COMPARE0)     /* Vector 41: Analog Comparator 0 */
VECTOR(tiva_cmp1, TIVA_IRQ_COMPARE1)     /* Vector 42: Analog Comparator 1 */
UNUSED(TIVA_RESERVED_43)                 /* Vector 43: Reserved */
VECTOR(tiva_syscon, TIVA_IRQ_SYSCON)     /* Vector 44: System Control */
VECTOR(tiva_flashcon, TIVA_IRQ_FLASHCON) /* Vector 45: FLASH Control */
VECTOR(tiva_gpiof, TIVA_IRQ_GPIOF)       /* Vector 46: GPIO Port F */
VECTOR(tiva_gpiog, TIVA_IRQ_GPIOG)       /* Vector 47: GPIO Port G */
UNUSED(TIVA_RESERVED_48)                 /* Vector 48: Reserved */
UNUSED(TIVA_RESERVED_49)                 /* Vector 49: Reserved */

UNUSED(TIVA_RESERVED_50)                 /* Vector 50: Reserved */
UNUSED(TIVA_RESERVED_51)                 /* Vector 51: Reserved */
UNUSED(TIVA_RESERVED_52)                 /* Vector 52: Reserved */
UNUSED(TIVA_RESERVED_53)                 /* Vector 53: Reserved */
UNUSED(TIVA_RESERVED_54)                 /* Vector 54: Reserved */
UNUSED(TIVA_RESERVED_55)                 /* Vector 55: Reserved */
UNUSED(TIVA_RESERVED_56)                 /* Vector 56: Reserved */
UNUSED(TIVA_RESERVED_57)                 /* Vector 57: Reserved */
VECTOR(tiva_eth, TIVA_IRQ_ETHCON)        /* Vector 58: Ethernet Controller */
UNUSED(TIVA_RESERVED_59)                 /* Vector 59: Reserved */

UNUSED(TIVA_RESERVED_60)                 /* Vector 60: Reserved */
UNUSED(TIVA_RESERVED_61)                 /* Vector 61: Reserved */
UNUSED(TIVA_RESERVED_62)                 /* Vector 62: Reserved */
UNUSED(TIVA_RESERVED_63)                 /* Vector 63: Reserved */
UNUSED(TIVA_RESERVED_64)                 /* Vector 64: Reserved */
UNUSED(TIVA_RESERVED_65)                 /* Vector 65: Reserved */
UNUSED(TIVA_RESERVED_66)                 /* Vector 66: Reserved */
UNUSED(TIVA_RESERVED_67)                 /* Vector 67: Reserved */
UNUSED(TIVA_RESERVED_68)                 /* Vector 68: Reserved */
UNUSED(TIVA_RESERVED_69)                 /* Vector 69: Reserved */

UNUSED(TIVA_RESERVED_70)                 /* Vector 70: Reserved */
#endif

#elif defined(CONFIG_ARCH_CHIP_LM3S6965)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 71 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 71

#else

VECTOR(tiva_gpioa, TIVA_IRQ_GPIOA)       /* Vector 16: GPIO Port A */
VECTOR(tiva_gpiob, TIVA_IRQ_GPIOB)       /* Vector 17: GPIO Port B */
VECTOR(tiva_gpioc, TIVA_IRQ_GPIOC)       /* Vector 18: GPIO Port C */
VECTOR(tiva_gpiod, TIVA_IRQ_GPIOD)       /* Vector 19: GPIO Port D */

VECTOR(tiva_gpioe, TIVA_IRQ_GPIOE)       /* Vector 20: GPIO Port E */
VECTOR(tiva_uart0, TIVA_IRQ_UART0)       /* Vector 21: UART 0 */
VECTOR(tiva_uart1, TIVA_IRQ_UART1)       /* Vector 22: UART 1 */
VECTOR(tiva_ssi0, TIVA_IRQ_SSI0)         /* Vector 23: SSI 0 */
VECTOR(tiva_i2c0, TIVA_IRQ_I2C0)         /* Vector 24: I2C 0 */
VECTOR(tiva_pwmfault, TIVA_IRQ_PWMFAULT) /* Vector 25: PWM Fault */
VECTOR(tiva_pwm0, TIVA_IRQ_PWM0)         /* Vector 26: PWM Generator 0 */
VECTOR(tiva_pwm1, TIVA_IRQ_PWM1)         /* Vector 27: PWM Generator 1 */
VECTOR(tiva_pwm2, TIVA_IRQ_PWM2)         /* Vector 28: PWM Generator 2 */
VECTOR(tiva_qei0, TIVA_IRQ_QEI0)         /* Vector 29: QEI 0 */

VECTOR(tiva_adc0, TIVA_IRQ_ADC0)         /* Vector 30: ADC Sequence 0 */
VECTOR(tiva_adc1, TIVA_IRQ_ADC1)         /* Vector 31: ADC Sequence 1 */
VECTOR(tiva_adc2, TIVA_IRQ_ADC2)         /* Vector 32: ADC Sequence 2 */
VECTOR(tiva_adc3, TIVA_IRQ_ADC3)         /* Vector 33: ADC Sequence 3 */
VECTOR(tiva_wdog, TIVA_IRQ_WDOG)         /* Vector 34: Watchdog Timer */
VECTOR(tiva_tmr0a, TIVA_IRQ_TIMER0A)     /* Vector 35: Timer 0 A */
VECTOR(tiva_tmr0b, TIVA_IRQ_TIMER0B)     /* Vector 36: Timer 0 B */
VECTOR(tiva_tmr1a, TIVA_IRQ_TIMER1A)     /* Vector 37: Timer 1 A */
VECTOR(tiva_tmr1b, TIVA_IRQ_TIMER1B)     /* Vector 38: Timer 1 B */
VECTOR(tiva_tmr2a, TIVA_IRQ_TIMER2A)     /* Vector 39: Timer 2 A */

VECTOR(tiva_tmr2b, TIVA_IRQ_TIMER2B)     /* Vector 40: Timer 3 B */
VECTOR(tiva_cmp0, TIVA_IRQ_COMPARE0)     /* Vector 41: Analog Comparator 0 */
VECTOR(tiva_cmp1, TIVA_IRQ_COMPARE1)     /* Vector 42: Analog Comparator 1 */
UNUSED(TIVA_RESERVED_43)                 /* Vector 43: Reserved */
VECTOR(tiva_syscon, TIVA_IRQ_SYSCON)     /* Vector 44: System Control */
VECTOR(tiva_flashcon, TIVA_IRQ_FLASHCON) /* Vector 45: FLASH Control */
VECTOR(tiva_gpiof, TIVA_IRQ_GPIOF)       /* Vector 46: GPIO Port F */
VECTOR(tiva_gpiog, TIVA_IRQ_GPIOG)       /* Vector 47: GPIO Port G */
UNUSED(TIVA_RESERVED_48)                 /* Vector 48: Reserved */
VECTOR(tiva_uart2, TIVA_IRQ_UART1)       /* Vector 49: UART 1 */

UNUSED(TIVA_RESERVED_50)                 /* Vector 50: Reserved */
VECTOR(tiva_tmr3a, TIVA_IRQ_TIMER3A)     /* Vector 51: Timer 3 A */
VECTOR(tiva_tmr3b, TIVA_IRQ_TIMER3B)     /* Vector 52: Timer 3 B */
VECTOR(tiva_i2c1, TIVA_IRQ_I2C1)         /* Vector 53: I2C 1 */
VECTOR(tiva_qei1, TIVA_IRQ_QEI1)         /* Vector 54: QEI 1 */
UNUSED(TIVA_RESERVED_55)                 /* Vector 55: Reserved */
UNUSED(TIVA_RESERVED_56)                 /* Vector 56: Reserved */
UNUSED(TIVA_RESERVED_57)                 /* Vector 57: Reserved */
VECTOR(tiva_eth, TIVA_IRQ_ETHCON)        /* Vector 58: Ethernet Controller */
VECTOR(tiva_hib, TIVA_IRQ_HIBERNATE)     /* Vector 59: Hibernation Module */

UNUSED(TIVA_RESERVED_60)                 /* Vector 60: Reserved */
UNUSED(TIVA_RESERVED_61)                 /* Vector 61: Reserved */
UNUSED(TIVA_RESERVED_62)                 /* Vector 62: Reserved */
UNUSED(TIVA_RESERVED_63)                 /* Vector 63: Reserved */
UNUSED(TIVA_RESERVED_64)                 /* Vector 64: Reserved */
UNUSED(TIVA_RESERVED_65)                 /* Vector 65: Reserved */
UNUSED(TIVA_RESERVED_66)                 /* Vector 66: Reserved */
UNUSED(TIVA_RESERVED_67)                 /* Vector 67: Reserved */
UNUSED(TIVA_RESERVED_68)                 /* Vector 68: Reserved */
UNUSED(TIVA_RESERVED_69)                 /* Vector 69: Reserved */

UNUSED(TIVA_RESERVED_70)                 /* Vector 70: Reserved */
#endif

#elif defined(CONFIG_ARCH_CHIP_LM3S8962)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 71 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 71

#else

VECTOR(tiva_gpioa, TIVA_IRQ_GPIOA)       /* Vector 16: GPIO Port A */
VECTOR(tiva_gpiob, TIVA_IRQ_GPIOB)       /* Vector 17: GPIO Port B */
VECTOR(tiva_gpioc, TIVA_IRQ_GPIOC)       /* Vector 18: GPIO Port C */
VECTOR(tiva_gpiod, TIVA_IRQ_GPIOD)       /* Vector 19: GPIO Port D */

VECTOR(tiva_gpioe, TIVA_IRQ_GPIOE)       /* Vector 20: GPIO Port E */
VECTOR(tiva_uart0, TIVA_IRQ_UART0)       /* Vector 21: UART 0 */
VECTOR(tiva_uart1, TIVA_IRQ_UART1)       /* Vector 22: UART 1 */
VECTOR(tiva_ssi0, TIVA_IRQ_SSI0)         /* Vector 23: SSI 0 */
VECTOR(tiva_i2c0, TIVA_IRQ_I2C0)         /* Vector 24: I2C 0 */
VECTOR(tiva_pwmfault, TIVA_IRQ_PWMFAULT) /* Vector 25: PWM Fault */
VECTOR(tiva_pwm0, TIVA_IRQ_PWM0)         /* Vector 26: PWM Generator 0 */
VECTOR(tiva_pwm1, TIVA_IRQ_PWM1)         /* Vector 27: PWM Generator 1 */
VECTOR(tiva_pwm2, TIVA_IRQ_PWM2)         /* Vector 28: PWM Generator 2 */
VECTOR(tiva_qei0, TIVA_IRQ_QEI0)         /* Vector 29: QEI 0 */

VECTOR(tiva_adc0, TIVA_IRQ_ADC0)         /* Vector 30: ADC Sequence 0 */
VECTOR(tiva_adc1, TIVA_IRQ_ADC1)         /* Vector 31: ADC Sequence 1 */
VECTOR(tiva_adc2, TIVA_IRQ_ADC2)         /* Vector 32: ADC Sequence 2 */
VECTOR(tiva_adc3, TIVA_IRQ_ADC3)         /* Vector 33: ADC Sequence 3 */
VECTOR(tiva_wdog, TIVA_IRQ_WDOG)         /* Vector 34: Watchdog Timer */
VECTOR(tiva_tmr0a, TIVA_IRQ_TIMER0A)     /* Vector 35: Timer 0 A */
VECTOR(tiva_tmr0b, TIVA_IRQ_TIMER0B)     /* Vector 36: Timer 0 B */
VECTOR(tiva_tmr1a, TIVA_IRQ_TIMER1A)     /* Vector 37: Timer 1 A */
VECTOR(tiva_tmr1b, TIVA_IRQ_TIMER1B)     /* Vector 38: Timer 1 B */
VECTOR(tiva_tmr2a, TIVA_IRQ_TIMER2A)     /* Vector 39: Timer 2 A */

VECTOR(tiva_tmr2b, TIVA_IRQ_TIMER2B)     /* Vector 40: Timer 3 B */
VECTOR(tiva_cmp0, TIVA_IRQ_COMPARE0)     /* Vector 41: Analog Comparator 0 */
UNUSED(TIVA_RESERVED_42)                 /* Vector 42: Reserved */
UNUSED(TIVA_RESERVED_43)                 /* Vector 43: Reserved */
VECTOR(tiva_syscon, TIVA_IRQ_SYSCON)     /* Vector 44: System Control */
VECTOR(tiva_flashcon, TIVA_IRQ_FLASHCON) /* Vector 45: FLASH Control */
VECTOR(tiva_gpiof, TIVA_IRQ_GPIOF)       /* Vector 46: GPIO Port F */
VECTOR(tiva_gpiog, TIVA_IRQ_GPIOG)       /* Vector 47: GPIO Port G */
UNUSED(TIVA_RESERVED_48)                 /* Vector 48: Reserved */
UNUSED(TIVA_RESERVED_49)                 /* Vector 49: Reserved */

UNUSED(TIVA_RESERVED_50)                 /* Vector 50: Reserved */
VECTOR(tiva_tmr3a, TIVA_IRQ_TIMER3A)     /* Vector 51: Timer 3 A */
VECTOR(tiva_tmr3b, TIVA_IRQ_TIMER3B)     /* Vector 52: Timer 3 B */
VECTOR(tiva_i2c1, TIVA_IRQ_I2C1)         /* Vector 53: I2C 1 */
VECTOR(tiva_qei1, TIVA_IRQ_QEI1)         /* Vector 54: QEI 1 */
VECTOR(tiva_can0, TIVA_IRQ_CAN0)         /* Vector 55: CAN 0 */
UNUSED(TIVA_RESERVED_56)                 /* Vector 56: Reserved */
UNUSED(TIVA_RESERVED_57)                 /* Vector 57: Reserved */
VECTOR(tiva_eth, TIVA_IRQ_ETHCON)        /* Vector 58: Ethernet Controller */
VECTOR(tiva_hib, TIVA_IRQ_HIBERNATE)     /* Vector 59: Hibernation Module */

UNUSED(TIVA_RESERVED_60)               /* Vector 60: Reserved */
UNUSED(TIVA_RESERVED_61)               /* Vector 61: Reserved */
UNUSED(TIVA_RESERVED_62)               /* Vector 62: Reserved */
UNUSED(TIVA_RESERVED_63)               /* Vector 63: Reserved */
UNUSED(TIVA_RESERVED_64)               /* Vector 64: Reserved */
UNUSED(TIVA_RESERVED_65)               /* Vector 65: Reserved */
UNUSED(TIVA_RESERVED_66)               /* Vector 66: Reserved */
UNUSED(TIVA_RESERVED_67)               /* Vector 67: Reserved */
UNUSED(TIVA_RESERVED_68)               /* Vector 68: Reserved */
UNUSED(TIVA_RESERVED_69)               /* Vector 69: Reserved */

UNUSED(TIVA_RESERVED_70)               /* Vector 70: Reserved */
#endif

#elif defined(CONFIG_ARCH_CHIP_LM3S9B96)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 72 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 72

#else

VECTOR(tiva_gpioa, TIVA_IRQ_GPIOA)       /* Vector 16: GPIO Port A */
VECTOR(tiva_gpiob, TIVA_IRQ_GPIOB)       /* Vector 17: GPIO Port B */
VECTOR(tiva_gpioc, TIVA_IRQ_GPIOC)       /* Vector 18: GPIO Port C */
VECTOR(tiva_gpiod, TIVA_IRQ_GPIOD)       /* Vector 19: GPIO Port D */
VECTOR(tiva_gpioe, TIVA_IRQ_GPIOE)       /* Vector 20: GPIO Port E */

VECTOR(tiva_uart0, TIVA_IRQ_UART0)       /* Vector 21: UART 0 */
VECTOR(tiva_uart1, TIVA_IRQ_UART1)       /* Vector 22: UART 1 */
VECTOR(tiva_ssi0, TIVA_IRQ_SSI0)         /* Vector 23: SSI 0 */
VECTOR(tiva_i2c0, TIVA_IRQ_I2C0)         /* Vector 24: I2C 0 */
VECTOR(tiva_pwmfault, TIVA_IRQ_PWMFAULT) /* Vector 25: PWM Fault */
VECTOR(tiva_pwm0, TIVA_IRQ_PWM0)         /* Vector 26: PWM Generator 0 */
VECTOR(tiva_pwm1, TIVA_IRQ_PWM1)         /* Vector 27: PWM Generator 1 */
VECTOR(tiva_pwm2, TIVA_IRQ_PWM2)         /* Vector 28: PWM Generator 2 */
VECTOR(tiva_qei0, TIVA_IRQ_QEI0)         /* Vector 29: QEI 0 */

VECTOR(tiva_adc0, TIVA_IRQ_ADC0)         /* Vector 30: ADC Sequence 0 */
VECTOR(tiva_adc1, TIVA_IRQ_ADC1)         /* Vector 31: ADC Sequence 1 */
VECTOR(tiva_adc2, TIVA_IRQ_ADC2)         /* Vector 32: ADC Sequence 2 */
VECTOR(tiva_adc3, TIVA_IRQ_ADC3)         /* Vector 33: ADC Sequence 3 */
VECTOR(tiva_wdog, TIVA_IRQ_WDOG)         /* Vector 34: Watchdog Timer */
VECTOR(tiva_tmr0a, TIVA_IRQ_TIMER0A)     /* Vector 35: Timer 0 A */
VECTOR(tiva_tmr0b, TIVA_IRQ_TIMER0B)     /* Vector 36: Timer 0 B */
VECTOR(tiva_tmr1a, TIVA_IRQ_TIMER1A)     /* Vector 37: Timer 1 A */
VECTOR(tiva_tmr1b, TIVA_IRQ_TIMER1B)     /* Vector 38: Timer 1 B */
VECTOR(tiva_tmr2a, TIVA_IRQ_TIMER2A)     /* Vector 39: Timer 2 A */

VECTOR(tiva_tmr2b, TIVA_IRQ_TIMER2B)     /* Vector 40: Timer 3 B */
VECTOR(tiva_cmp0, TIVA_IRQ_COMPARE0)     /* Vector 41: Analog Comparator 0 */
VECTOR(tiva_cmp1, TIVA_IRQ_COMPARE1)     /* Vector 42: Analog Comparator 1 */
VECTOR(tiva_cmp2, TIVA_IRQ_COMPARE2)     /* Vector 43: Analog Comparator 2 */
VECTOR(tiva_syscon, TIVA_IRQ_SYSCON)     /* Vector 44: System Control */
VECTOR(tiva_flashcon, TIVA_IRQ_FLASHCON) /* Vector 45: FLASH Control */
VECTOR(tiva_gpiof, TIVA_IRQ_GPIOF)       /* Vector 46: GPIO Port F */
VECTOR(tiva_gpiog, TIVA_IRQ_GPIOG)       /* Vector 47: GPIO Port G */
VECTOR(tiva_gpioh, TIVA_IRQ_GPIOH)       /* Vector 48: GPIO Port H */
VECTOR(tiva_uart2, TIVA_IRQ_UART2)       /* Vector 49: UART 2 */

VECTOR(tiva_ssi1, TIVA_IRQ_SSI1)         /* Vector 50: GPIO Port H */
VECTOR(tiva_tmr3a, TIVA_IRQ_TIMER3A)     /* Vector 51: Timer 3 A */
VECTOR(tiva_tmr3b, TIVA_IRQ_TIMER3B)     /* Vector 52: Timer 3 B */
VECTOR(tiva_i2c1, TIVA_IRQ_I2C1)         /* Vector 53: I2C 1 */
VECTOR(tiva_qei1, TIVA_IRQ_QEI1)         /* Vector 54: QEI 1 */
VECTOR(tiva_can0, TIVA_IRQ_CAN0)         /* Vector 55: CAN 0 */
VECTOR(tiva_can1, TIVA_IRQ_CAN1)         /* Vector 56: CAN 1 */
UNUSED(TIVA_RESERVED_57)                 /* Vector 57: Reserved */
VECTOR(tiva_eth, TIVA_IRQ_ETHCON)        /* Vector 58: Ethernet Controller */
UNUSED(TIVA_RESERVED_59)                 /* Vector 59: Reserved */

VECTOR(tiva_usb, TIVA_IRQ_USB)           /* Vector 60: USB */
VECTOR(tiva_pwm3, TIVA_IRQ_PWM3)         /* Vector 61: PWM 3  */
VECTOR(tiva_udmasoft, TIVA_IRQ_UDMASOFT) /* Vector 62: uDMA Software  */
VECTOR(tiva_udmaerror, TIVA_IRQ_UDMAERROR) /* Vector 63: uDMA Error */
VECTOR(tiva_adc1_0, TIVA_IRQ_ADC1_0)     /* Vector 64: ADC1 Sequence 0 */
VECTOR(tiva_adc1_1, TIVA_IRQ_ADC1_1)     /* Vector 65: ADC1 Sequence 1 */
VECTOR(tiva_adc1_2, TIVA_IRQ_ADC1_2)     /* Vector 66: ADC1 Sequence 2 */
VECTOR(tiva_adc1_3, TIVA_IRQ_ADC1_3)     /* Vector 67: ADC1 Sequence 3 */
VECTOR(tiva_i2s0, TIVA_IRQ_I2S0)         /* Vector 68: I2S 0 */
VECTOR(tiva_epi, TIVA_IRQ_EPI)           /* Vector 69: EPI  */

VECTOR(tiva_gpioj, TIVA_IRQ_GPIOJ)       /* Vector 70: GPIO Port J */
UNUSED(TIVA_RESERVED_71)                 /* Vector 71: Reserved */
#endif

#else
#  error "Vectors not specified for this Stellaris chip"
#endif
