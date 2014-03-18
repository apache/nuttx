/************************************************************************************
 * arch/arm/src/tiva/chip/tm4c_vectors.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 * supplies each Tiva vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/tiva/tm4c_irq.h.
 * tiva_vectors.S will define the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#  ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 155 interrupt table entries for I/O interrupts. */

ARMV7M_PERIPHERAL_INTERRUPTS 155

#  else

VECTOR(tiva_gpioa, TIVA_IRQ_GPIOA)           /* Vector 16: GPIO Port A */
VECTOR(tiva_gpiob, TIVA_IRQ_GPIOB)           /* Vector 17: GPIO Port B */
VECTOR(tiva_gpioc, TIVA_IRQ_GPIOC)           /* Vector 18: GPIO Port C */
VECTOR(tiva_gpiod, TIVA_IRQ_GPIOD)           /* Vector 19: GPIO Port D */

VECTOR(tiva_gpioe, TIVA_IRQ_GPIOE)           /* Vector 20: GPIO Port E */
VECTOR(tiva_uart0, TIVA_IRQ_UART0)           /* Vector 21: UART 0 */
VECTOR(tiva_uart1, TIVA_IRQ_UART1)           /* Vector 22: UART 1 */
VECTOR(tiva_ssi0, TIVA_IRQ_SSI0)             /* Vector 23: SSI 0 */
VECTOR(tiva_i2c0, TIVA_IRQ_I2C0)             /* Vector 24: I2C 0 */
VECTOR(tiva_pwm0_fault, TIVA_IRQ_PWM0_FAULT) /* Vector 25: PWM0 Fault */
VECTOR(tiva_pwm0_gen0, TIVA_IRQ_PWM0_GEN0)   /* Vector 26: PWM0 Generator 0 */
VECTOR(tiva_pwm0_gen1, TIVA_IRQ_PWM0_GEN1)   /* Vector 27: PWM0 Generator 1 */
VECTOR(tiva_pwm0_gen2, TIVA_IRQ_PWM0_GEN2)   /* Vector 28: PWM0 Generator 2 */
VECTOR(tiva_qei0, TIVA_IRQ_QEI0)             /* Vector 29: QEI0 */

VECTOR(tiva_adc0, TIVA_IRQ_ADC0)             /* Vector 30: ADC Sequence 0 */
VECTOR(tiva_adc1, TIVA_IRQ_ADC1)             /* Vector 31: ADC Sequence 1 */
VECTOR(tiva_adc2, TIVA_IRQ_ADC2)             /* Vector 32: ADC Sequence 2 */
VECTOR(tiva_adc3, TIVA_IRQ_ADC3)             /* Vector 33: ADC Sequence 3 */
VECTOR(tiva_wdog, TIVA_IRQ_WDOG)             /* Vector 34: Watchdog Timers 0 and 1 */
VECTOR(tiva_timer0a, TIVA_IRQ_TIMER0A)       /* Vector 35: 16/32-Bit Timer 0 A */
VECTOR(tiva_timer0b, TIVA_IRQ_TIMER0B)       /* Vector 36: 16/32-Bit Timer 0 B */
VECTOR(tiva_timer1a, TIVA_IRQ_TIMER1A)       /* Vector 37: 16/32-Bit Timer 1 A */
VECTOR(tiva_timer1b, TIVA_IRQ_TIMER1B)       /* Vector 38: 16/32-Bit Timer 1 B */
VECTOR(tiva_timer2a, TIVA_IRQ_TIMER2A)       /* Vector 39: 16/32-Bit Timer 2 A */

VECTOR(tiva_timer2b, TIVA_IRQ_TIMER2B)       /* Vector 40: 16/32-Bit Timer 2 B */
VECTOR(tiva_compare0, TIVA_IRQ_COMPARE0)     /* Vector 41: Analog Comparator 0 */
VECTOR(tiva_compare1, TIVA_IRQ_COMPARE1)     /* Vector 42: Analog Comparator 1 */
VECTOR(tiva_compare2, TIVA_IRQ_COMPARE2)     /* Vector 43: Analog Comparator 2 */
VECTOR(tiva_syscon, TIVA_IRQ_SYSCON)         /* Vector 44: System Control */
VECTOR(tiva_flashcon, TIVA_IRQ_FLASHCON)     /* Vector 45: FLASH and EEPROM Control */
VECTOR(tiva_gpiof, TIVA_IRQ_GPIOF)           /* Vector 46: GPIO Port F */
VECTOR(tiva_gpiog, TIVA_IRQ_GPIOG)           /* Vector 47: GPIO Port G */
VECTOR(tiva_gpioh, TIVA_IRQ_GPIOH)           /* Vector 48: GPIO Port H */
VECTOR(tiva_uart2, TIVA_IRQ_UART2)           /* Vector 49: UART 2 */

VECTOR(tiva_ssi1, TIVA_IRQ_SSI1)             /* Vector 50: SSI 1 */
VECTOR(tiva_timer3a, TIVA_IRQ_TIMER3A)       /* Vector 51: 16/32-Bit Timer 3 A */
VECTOR(tiva_timer3b, TIVA_IRQ_TIMER3B)       /* Vector 52: 16/32-Bit Timer 3 B */
VECTOR(tiva_i2c1, TIVA_IRQ_I2C1)             /* Vector 53: I2C 1 */
VECTOR(tiva_qei1, TIVA_IRQ_QEI1)             /* Vector 54: QEI1 */
VECTOR(tiva_can0, TIVA_IRQ_CAN0)             /* Vector 55: CAN 0 */
VECTOR(tiva_can1, TIVA_IRQ_CAN1)             /* Vector 56: CAN 1 */
UNUSED(TIVA_RESERVED_57)                     /* Vector 57: Reserved */
UNUSED(TIVA_RESERVED_58)                     /* Vector 58: Reserved */
VECTOR(tiva_hibernate, TIVA_IRQ_HIBERNATE)   /* Vector 59: Hibernation Module */

VECTOR(tiva_usb, TIVA_IRQ_USB)               /* Vector 60: USB */
VECTOR(tiva_pwm0_gen3, TIVA_IRQ_PWM0_GEN3)   /* Vector 61: PWM0 Generator 3 */
VECTOR(tiva_udmasoft, TIVA_IRQ_UDMASOFT)     /* Vector 62: uDMA Software */
VECTOR(tiva_udmaerro, TIVA_IRQ_UDMAERROR)    /* Vector 63: uDMA Error */
VECTOR(tiva_adc1_0, TIVA_IRQ_ADC1_0)         /* Vector 64: ADC1 Sequence 0 */
VECTOR(tiva_adc1_1, TIVA_IRQ_ADC1_1)         /* Vector 65: ADC1 Sequence 1 */
VECTOR(tiva_adc1_2, TIVA_IRQ_ADC1_2)         /* Vector 66: ADC1 Sequence 2 */
VECTOR(tiva_adc1_3, TIVA_IRQ_ADC1_3)         /* Vector 67: ADC1 Sequence 3 */
UNUSED(TIVA_RESERVED_68)                     /* Vector 68: Reserved */
UNUSED(TIVA_RESERVED_69)                     /* Vector 69: Reserved */

VECTOR(tiva_gpioj, TIVA_IRQ_GPIOJ)           /* Vector 70: GPIO Port J */
VECTOR(tiva_gpiok, TIVA_IRQ_GPIOK)           /* Vector 71: GPIO Port K */
VECTOR(tiva_gpiol, TIVA_IRQ_GPIOL)           /* Vector 72: GPIO Port L */
VECTOR(tiva_ssi2, TIVA_IRQ_SSI2)             /* Vector 73: SSI 2 */
VECTOR(tiva_ssi3, TIVA_IRQ_SSI3)             /* Vector 74: SSI 3 */
VECTOR(tiva_uart3, TIVA_IRQ_UART3)           /* Vector 75: UART 3 */
VECTOR(tiva_uart4, TIVA_IRQ_UART4)           /* Vector 76: UART 4 */
VECTOR(tiva_uart5, TIVA_IRQ_UART5)           /* Vector 77: UART 5 */
VECTOR(tiva_uart6, TIVA_IRQ_UART6)           /* Vector 78: UART 6 */
VECTOR(tiva_uart7, TIVA_IRQ_UART7)           /* Vector 79: UART 7 */

UNUSED(TIVA_RESERVED_80)                     /* Vector 80: Reserved */
UNUSED(TIVA_RESERVED_81)                     /* Vector 81: Reserved */
UNUSED(TIVA_RESERVED_82)                     /* Vector 82: Reserved */
UNUSED(TIVA_RESERVED_83)                     /* Vector 83: Reserved */
VECTOR(tiva_i2c2, TIVA_IRQ_I2C2)             /* Vector 84: I2C 2 */
VECTOR(tiva_i2c3, TIVA_IRQ_I2C3)             /* Vector 85: I2C 3 */
VECTOR(tiva_timer4a, TIVA_IRQ_TIMER4A)       /* Vector 86: 16/32-Bit Timer 4 A */
VECTOR(tiva_timer4b, TIVA_IRQ_TIMER4B)       /* Vector 87: 16/32-Bit Timer 4 B */
UNUSED(TIVA_RESERVED_88)                     /* Vector 88: Reserved */
UNUSED(TIVA_RESERVED_89)                     /* Vector 89: Reserved */

UNUSED(TIVA_RESERVED_90)                     /* Vector 90: Reserved */
UNUSED(TIVA_RESERVED_91)                     /* Vector 91: Reserved */
UNUSED(TIVA_RESERVED_92)                     /* Vector 92: Reserved */
UNUSED(TIVA_RESERVED_93)                     /* Vector 93: Reserved */
UNUSED(TIVA_RESERVED_94)                     /* Vector 94: Reserved */
UNUSED(TIVA_RESERVED_95)                     /* Vector 95: Reserved */
UNUSED(TIVA_RESERVED_96)                     /* Vector 96: Reserved */
UNUSED(TIVA_RESERVED_97)                     /* Vector 97: Reserved */
UNUSED(TIVA_RESERVED_98)                     /* Vector 98: Reserved */
UNUSED(TIVA_RESERVED_99)                     /* Vector 99: Reserved */

UNUSED(TIVA_RESERVED_100)                    /* Vector 100: Reserved */
UNUSED(TIVA_RESERVED_101)                    /* Vector 101: Reserved */
UNUSED(TIVA_RESERVED_102)                    /* Vector 102: Reserved */
UNUSED(TIVA_RESERVED_103)                    /* Vector 103: Reserved */
UNUSED(TIVA_RESERVED_104)                    /* Vector 104: Reserved */
UNUSED(TIVA_RESERVED_105)                    /* Vector 105: Reserved */
UNUSED(TIVA_RESERVED_106)                    /* Vector 106: Reserved */
UNUSED(TIVA_RESERVED_107)                    /* Vector 107: Reserved */
VECTOR(tiva_timer5a, TIVA_IRQ_TIMER5A)       /* Vector 108: 16/32-Bit Timer 5 A */
VECTOR(tiva_timer5b, TIVA_IRQ_TIMER5B)       /* Vector 109: 16/32-Bit Timer 5 B */

VECTOR(tiva_wtimer0a, TIVA_IRQ_WTIMER0A)     /* Vector 110: 32/64-Bit Timer 0 A */
VECTOR(tiva_wtimer0b, TIVA_IRQ_WTIMER0B)     /* Vector 111: 32/64-Bit Timer 0 B */
VECTOR(tiva_wtimer1a, TIVA_IRQ_WTIMER1A)     /* Vector 112: 32/64-Bit Timer 1 A */
VECTOR(tiva_wtimer1b, TIVA_IRQ_WTIMER1B)     /* Vector 113: 32/64-Bit Timer 1 B */
VECTOR(tiva_wtimer2a, TIVA_IRQ_WTIMER2A)     /* Vector 114: 32/64-Bit Timer 2 A */
VECTOR(tiva_wtimer2b, TIVA_IRQ_WTIMER2B)     /* Vector 115: 32/64-Bit Timer 2 B */
VECTOR(tiva_wtimer3a, TIVA_IRQ_WTIMER3A)     /* Vector 116: 32/64-Bit Timer 3 A */
VECTOR(tiva_wtimer3b, TIVA_IRQ_WTIMER3B)     /* Vector 117: 32/64-Bit Timer 3 B */
VECTOR(tiva_wtimer4a, TIVA_IRQ_WTIMER4A)     /* Vector 118: 32/64-Bit Timer 4 A */
VECTOR(tiva_WTIMER4B, TIVA_IRQ_WTIMER4B)     /* Vector 119: 32/64-Bit Timer 4 B */

VECTOR(tiva_wtimer5a, TIVA_IRQ_WTIMER5A)     /* Vector 120: 32/64-Bit Timer 5 A */
VECTOR(tiva_wtimer5b, TIVA_IRQ_WTIMER5B)     /* Vector 121: 32/64-Bit Timer 5 B */
VECTOR(tiva_system, TIVA_IRQ_SYSTEM)         /* Vector 122: System Exception (imprecise) */
UNUSED(TIVA_RESERVED_123)                    /* Vector 123: Reserved */
UNUSED(TIVA_RESERVED_124)                    /* Vector 124: Reserved */
VECTOR(tiva_i2c4, TIVA_IRQ_I2C4)             /* Vector 125: I2C4 */
VECTOR(tiva_i2c5, TIVA_IRQ_I2C5)             /* Vector 126: I2C5 */
VECTOR(tiva_gpiom, TIVA_IRQ_GPIOM)           /* Vector 127: GPIO Port M */
VECTOR(tiva_gpion, TIVA_IRQ_GPION)           /* Vector 128: GPIO Port N */
UNUSED(TIVA_RESERVED_129)                    /* Vector 129: Reserved */

UNUSED(TIVA_RESERVED_130)                    /* Vector 130: Reserved */
UNUSED(TIVA_RESERVED_131)                    /* Vector 131: Reserved */
VECTOR(tiva_gpiop, TIVA_IRQ_GPIOP)           /* Vector 132: GPIO Port P (Summary or P0) */
VECTOR(tiva_gpiop1, TIVA_IRQ_GPIOP1)         /* Vector 133: GPIO Port P1 */
VECTOR(tiva_gpiop2, TIVA_IRQ_GPIOP2)         /* Vector 134: GPIO Port P2 */
VECTOR(tiva_gpiop3, TIVA_IRQ_GPIOP3)         /* Vector 135: GPIO Port P3 */
VECTOR(tiva_gpiop4, TIVA_IRQ_GPIOP4)         /* Vector 136: GPIO Port P4 */
VECTOR(tiva_gpiop5, TIVA_IRQ_GPIOP5)         /* Vector 137: GPIO Port P5 */
VECTOR(tiva_gpiop6, TIVA_IRQ_GPIOP6)         /* Vector 138: GPIO Port P6 */
VECTOR(tiva_gpiop7, TIVA_IRQ_GPIOP7)         /* Vector 139: GPIO Port P7 */

VECTOR(tiva_gpioq, TIVA_IRQ_GPIOQ)           /* Vector 140: GPIO Port Q (Summary or Q0) */
VECTOR(tiva_gpioq1, TIVA_IRQ_GPIOQ1)         /* Vector 141: GPIO Port Q1 */
VECTOR(tiva_gpioq2, TIVA_IRQ_GPIOQ2)         /* Vector 142: GPIO Port Q2 */
VECTOR(tiva_gpioq3, TIVA_IRQ_GPIOQ3)         /* Vector 143: GPIO Port Q3 */
VECTOR(tiva_gpioq4, TIVA_IRQ_GPIOQ4)         /* Vector 144: GPIO Port Q4 */
VECTOR(tiva_gpioq5, TIVA_IRQ_GPIOQ5)         /* Vector 145: GPIO Port Q5 */
VECTOR(tiva_gpioq6, TIVA_IRQ_GPIOQ6)         /* Vector 146: GPIO Port Q6 */
VECTOR(tiva_gpioq7, TIVA_IRQ_GPIOQ7)         /* Vector 147: GPIO Port Q7 */
UNUSED(TIVA_RESERVED_148)                    /* Vector 148: Reserved */
UNUSED(TIVA_RESERVED_149)                    /* Vector 149: Reserved */

VECTOR(tiva_pwm1_gen0, TIVA_IRQ_PWM1_GEN0)   /* Vector 150: PWM1 Generator 0 */
VECTOR(tiva_pwm1_gen1, TIVA_IRQ_PWM1_GEN1)   /* Vector 151: PWM1 Generator 1 */
VECTOR(tiva_pwm1_gen2, TIVA_IRQ_PWM1_GEN2)   /* Vector 152: PWM1 Generator 2 */
VECTOR(tiva_pwm1_gen3, TIVA_IRQ_PWM1_GEN3)   /* Vector 153: PWM1 Generator 3 */
VECTOR(tiva_pwm1_fault, TIVA_IRQ_PWM1_FAULT) /* Vector 154: PWM1 Fault */

# endif /* CONFIG_ARMV7M_CMNVECTOR */

#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PMI)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#  ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 155 interrupt table entries for I/O interrupts. */

ARMV7M_PERIPHERAL_INTERRUPTS 155

#  else

VECTOR(tiva_gpioa, TIVA_IRQ_GPIOA)           /* Vector 16: GPIO Port A */
VECTOR(tiva_gpiob, TIVA_IRQ_GPIOB)           /* Vector 17: GPIO Port B */
VECTOR(tiva_gpioc, TIVA_IRQ_GPIOC)           /* Vector 18: GPIO Port C */
VECTOR(tiva_gpiod, TIVA_IRQ_GPIOD)           /* Vector 19: GPIO Port D */

VECTOR(tiva_gpioe, TIVA_IRQ_GPIOE)           /* Vector 20: GPIO Port E */
VECTOR(tiva_uart0, TIVA_IRQ_UART0)           /* Vector 21: UART 0 */
VECTOR(tiva_uart1, TIVA_IRQ_UART1)           /* Vector 22: UART 1 */
VECTOR(tiva_ssi0, TIVA_IRQ_SSI0)             /* Vector 23: SSI 0 */
VECTOR(tiva_i2c0, TIVA_IRQ_I2C0)             /* Vector 24: I2C 0 */
VECTOR(tiva_pwm0_fault, TIVA_IRQ_PWM0_FAULT) /* Vector 25: PWM0 Fault */
VECTOR(tiva_pwm0_gen0, TIVA_IRQ_PWM0_GEN0)   /* Vector 26: PWM0 Generator 0 */
VECTOR(tiva_pwm0_gen1, TIVA_IRQ_PWM0_GEN1)   /* Vector 27: PWM0 Generator 1 */
VECTOR(tiva_pwm0_gen2, TIVA_IRQ_PWM0_GEN2)   /* Vector 28: PWM0 Generator 2 */
VECTOR(tiva_qei0, TIVA_IRQ_QEI0)             /* Vector 29: QEI0 */

VECTOR(tiva_adc0, TIVA_IRQ_ADC0)             /* Vector 30: ADC Sequence 0 */
VECTOR(tiva_adc1, TIVA_IRQ_ADC1)             /* Vector 31: ADC Sequence 1 */
VECTOR(tiva_adc2, TIVA_IRQ_ADC2)             /* Vector 32: ADC Sequence 2 */
VECTOR(tiva_adc3, TIVA_IRQ_ADC3)             /* Vector 33: ADC Sequence 3 */
VECTOR(tiva_wdog, TIVA_IRQ_WDOG)             /* Vector 34: Watchdog Timers 0 and 1 */
VECTOR(tiva_timer0a, TIVA_IRQ_TIMER0A)       /* Vector 35: 16/32-Bit Timer 0 A */
VECTOR(tiva_timer0b, TIVA_IRQ_TIMER0B)       /* Vector 36: 16/32-Bit Timer 0 B */
VECTOR(tiva_timer1a, TIVA_IRQ_TIMER1A)       /* Vector 37: 16/32-Bit Timer 1 A */
VECTOR(tiva_timer1b, TIVA_IRQ_TIMER1B)       /* Vector 38: 16/32-Bit Timer 1 B */
VECTOR(tiva_timer2a, TIVA_IRQ_TIMER2A)       /* Vector 39: 16/32-Bit Timer 2 A */

VECTOR(tiva_timer2b, TIVA_IRQ_TIMER2B)       /* Vector 40: 16/32-Bit Timer 2 B */
VECTOR(tiva_compare0, TIVA_IRQ_COMPARE0)     /* Vector 41: Analog Comparator 0 */
VECTOR(tiva_compare1, TIVA_IRQ_COMPARE1)     /* Vector 42: Analog Comparator 1 */
VECTOR(tiva_compare2, TIVA_IRQ_COMPARE2)     /* Vector 43: Analog Comparator 2 */
VECTOR(tiva_syscon, TIVA_IRQ_SYSCON)         /* Vector 44: System Control */
VECTOR(tiva_flashcon, TIVA_IRQ_FLASHCON)     /* Vector 45: FLASH and EEPROM Control */
VECTOR(tiva_gpiof, TIVA_IRQ_GPIOF)           /* Vector 46: GPIO Port F */
UNUSED(TIVA_RESERVED_47)                     /* Vector 47: GPIO Port G */
UNUSED(TIVA_RESERVED_48)                     /* Vector 48: GPIO Port H */
VECTOR(tiva_uart2, TIVA_IRQ_UART2)           /* Vector 49: UART 2 */

VECTOR(tiva_ssi1, TIVA_IRQ_SSI1)             /* Vector 50: SSI 1 */
VECTOR(tiva_timer3a, TIVA_IRQ_TIMER3A)       /* Vector 51: 16/32-Bit Timer 3 A */
VECTOR(tiva_timer3b, TIVA_IRQ_TIMER3B)       /* Vector 52: 16/32-Bit Timer 3 B */
VECTOR(tiva_i2c1, TIVA_IRQ_I2C1)             /* Vector 53: I2C 1 */
VECTOR(tiva_qei1, TIVA_IRQ_QEI1)             /* Vector 54: QEI1 */
VECTOR(tiva_can0, TIVA_IRQ_CAN0)             /* Vector 55: CAN 0 */
VECTOR(tiva_can1, TIVA_IRQ_CAN1)             /* Vector 56: CAN 1 */
UNUSED(TIVA_RESERVED_57)                     /* Vector 57: Reserved */
UNUSED(TIVA_RESERVED_58)                     /* Vector 58: Reserved */
VECTOR(tiva_hibernate, TIVA_IRQ_HIBERNATE)   /* Vector 59: Hibernation Module */

VECTOR(tiva_usb, TIVA_IRQ_USB)               /* Vector 60: USB */
VECTOR(tiva_pwm0_gen3, TIVA_IRQ_PWM0_GEN3)   /* Vector 61: PWM0 Generator 3 */
VECTOR(tiva_udmasoft, TIVA_IRQ_UDMASOFT)     /* Vector 62: uDMA Software */
VECTOR(tiva_udmaerro, TIVA_IRQ_UDMAERROR)    /* Vector 63: uDMA Error */
VECTOR(tiva_adc1_0, TIVA_IRQ_ADC1_0)         /* Vector 64: ADC1 Sequence 0 */
VECTOR(tiva_adc1_1, TIVA_IRQ_ADC1_1)         /* Vector 65: ADC1 Sequence 1 */
VECTOR(tiva_adc1_2, TIVA_IRQ_ADC1_2)         /* Vector 66: ADC1 Sequence 2 */
VECTOR(tiva_adc1_3, TIVA_IRQ_ADC1_3)         /* Vector 67: ADC1 Sequence 3 */
UNUSED(TIVA_RESERVED_68)                     /* Vector 68: Reserved */
UNUSED(TIVA_RESERVED_69)                     /* Vector 69: Reserved */

UNUSED(TIVA_RESERVED_70)                     /* Vector 70: GPIO Port J */
UNUSED(TIVA_RESERVED_71)                     /* Vector 71: GPIO Port K */
UNUSED(TIVA_RESERVED_72)                     /* Vector 72: GPIO Port L */
VECTOR(tiva_ssi2, TIVA_IRQ_SSI2)             /* Vector 73: SSI 2 */
VECTOR(tiva_ssi3, TIVA_IRQ_SSI3)             /* Vector 74: SSI 3 */
VECTOR(tiva_uart3, TIVA_IRQ_UART3)           /* Vector 75: UART 3 */
VECTOR(tiva_uart4, TIVA_IRQ_UART4)           /* Vector 76: UART 4 */
VECTOR(tiva_uart5, TIVA_IRQ_UART5)           /* Vector 77: UART 5 */
VECTOR(tiva_uart6, TIVA_IRQ_UART6)           /* Vector 78: UART 6 */
VECTOR(tiva_uart7, TIVA_IRQ_UART7)           /* Vector 79: UART 7 */

UNUSED(TIVA_RESERVED_80)                     /* Vector 80: Reserved */
UNUSED(TIVA_RESERVED_81)                     /* Vector 81: Reserved */
UNUSED(TIVA_RESERVED_82)                     /* Vector 82: Reserved */
UNUSED(TIVA_RESERVED_83)                     /* Vector 83: Reserved */
VECTOR(tiva_i2c2, TIVA_IRQ_I2C2)             /* Vector 84: I2C 2 */
VECTOR(tiva_i2c3, TIVA_IRQ_I2C3)             /* Vector 85: I2C 3 */
VECTOR(tiva_timer4a, TIVA_IRQ_TIMER4A)       /* Vector 86: 16/32-Bit Timer 4 A */
VECTOR(tiva_timer4b, TIVA_IRQ_TIMER4B)       /* Vector 87: 16/32-Bit Timer 4 B */
UNUSED(TIVA_RESERVED_88)                     /* Vector 88: Reserved */
UNUSED(TIVA_RESERVED_89)                     /* Vector 89: Reserved */

UNUSED(TIVA_RESERVED_90)                     /* Vector 90: Reserved */
UNUSED(TIVA_RESERVED_91)                     /* Vector 91: Reserved */
UNUSED(TIVA_RESERVED_92)                     /* Vector 92: Reserved */
UNUSED(TIVA_RESERVED_93)                     /* Vector 93: Reserved */
UNUSED(TIVA_RESERVED_94)                     /* Vector 94: Reserved */
UNUSED(TIVA_RESERVED_95)                     /* Vector 95: Reserved */
UNUSED(TIVA_RESERVED_96)                     /* Vector 96: Reserved */
UNUSED(TIVA_RESERVED_97)                     /* Vector 97: Reserved */
UNUSED(TIVA_RESERVED_98)                     /* Vector 98: Reserved */
UNUSED(TIVA_RESERVED_99)                     /* Vector 99: Reserved */

UNUSED(TIVA_RESERVED_100)                    /* Vector 100: Reserved */
UNUSED(TIVA_RESERVED_101)                    /* Vector 101: Reserved */
UNUSED(TIVA_RESERVED_102)                    /* Vector 102: Reserved */
UNUSED(TIVA_RESERVED_103)                    /* Vector 103: Reserved */
UNUSED(TIVA_RESERVED_104)                    /* Vector 104: Reserved */
UNUSED(TIVA_RESERVED_105)                    /* Vector 105: Reserved */
UNUSED(TIVA_RESERVED_106)                    /* Vector 106: Reserved */
UNUSED(TIVA_RESERVED_107)                    /* Vector 107: Reserved */
VECTOR(tiva_timer5a, TIVA_IRQ_TIMER5A)       /* Vector 108: 16/32-Bit Timer 5 A */
VECTOR(tiva_timer5b, TIVA_IRQ_TIMER5B)       /* Vector 109: 16/32-Bit Timer 5 B */

VECTOR(tiva_wtimer0a, TIVA_IRQ_WTIMER0A)     /* Vector 110: 32/64-Bit Timer 0 A */
VECTOR(tiva_wtimer0b, TIVA_IRQ_WTIMER0B)     /* Vector 111: 32/64-Bit Timer 0 B */
VECTOR(tiva_wtimer1a, TIVA_IRQ_WTIMER1A)     /* Vector 112: 32/64-Bit Timer 1 A */
VECTOR(tiva_wtimer1b, TIVA_IRQ_WTIMER1B)     /* Vector 113: 32/64-Bit Timer 1 B */
VECTOR(tiva_wtimer2a, TIVA_IRQ_WTIMER2A)     /* Vector 114: 32/64-Bit Timer 2 A */
VECTOR(tiva_wtimer2b, TIVA_IRQ_WTIMER2B)     /* Vector 115: 32/64-Bit Timer 2 B */
VECTOR(tiva_wtimer3a, TIVA_IRQ_WTIMER3A)     /* Vector 116: 32/64-Bit Timer 3 A */
VECTOR(tiva_wtimer3b, TIVA_IRQ_WTIMER3B)     /* Vector 117: 32/64-Bit Timer 3 B */
VECTOR(tiva_wtimer4a, TIVA_IRQ_WTIMER4A)     /* Vector 118: 32/64-Bit Timer 4 A */
VECTOR(tiva_WTIMER4B, TIVA_IRQ_WTIMER4B)     /* Vector 119: 32/64-Bit Timer 4 B */

VECTOR(tiva_wtimer5a, TIVA_IRQ_WTIMER5A)     /* Vector 120: 32/64-Bit Timer 5 A */
VECTOR(tiva_wtimer5b, TIVA_IRQ_WTIMER5B)     /* Vector 121: 32/64-Bit Timer 5 B */
VECTOR(tiva_system, TIVA_IRQ_SYSTEM)         /* Vector 122: System Exception (imprecise) */
UNUSED(TIVA_RESERVED_123)                    /* Vector 123: Reserved */
UNUSED(TIVA_RESERVED_124)                    /* Vector 124: Reserved */
UNUSED(TIVA_RESERVED_125)                    /* Vector 125: Reserved */
UNUSED(TIVA_RESERVED_126)                    /* Vector 126: Reserved */
UNUSED(TIVA_RESERVED_127)                    /* Vector 127: Reserved */
UNUSED(TIVA_RESERVED_128)                    /* Vector 128: Reserved */
UNUSED(TIVA_RESERVED_129)                    /* Vector 129: Reserved */

UNUSED(TIVA_RESERVED_130)                    /* Vector 130: Reserved */
UNUSED(TIVA_RESERVED_131)                    /* Vector 131: Reserved */
UNUSED(TIVA_RESERVED_132)                    /* Vector 132: Reserved */
UNUSED(TIVA_RESERVED_133)                    /* Vector 133: Reserved */
UNUSED(TIVA_RESERVED_134)                    /* Vector 134: Reserved */
UNUSED(TIVA_RESERVED_135)                    /* Vector 135: Reserved */
UNUSED(TIVA_RESERVED_136)                    /* Vector 136: Reserved */
UNUSED(TIVA_RESERVED_137)                    /* Vector 137: Reserved */
UNUSED(TIVA_RESERVED_138)                    /* Vector 138: Reserved */
UNUSED(TIVA_RESERVED_139)                    /* Vector 139: Reserved */

UNUSED(TIVA_RESERVED_140)                    /* Vector 140: Reserved */
UNUSED(TIVA_RESERVED_141)                    /* Vector 141: Reserved */
UNUSED(TIVA_RESERVED_142)                    /* Vector 142: Reserved */
UNUSED(TIVA_RESERVED_143)                    /* Vector 143: Reserved */
UNUSED(TIVA_RESERVED_144)                    /* Vector 144: Reserved */
UNUSED(TIVA_RESERVED_145)                    /* Vector 145: Reserved */
UNUSED(TIVA_RESERVED_146)                    /* Vector 146: Reserved */
UNUSED(TIVA_RESERVED_147)                    /* Vector 147: Reserved */
UNUSED(TIVA_RESERVED_148)                    /* Vector 148: Reserved */
UNUSED(TIVA_RESERVED_149)                    /* Vector 149: Reserved */

VECTOR(tiva_pwm1_gen0, TIVA_IRQ_PWM1_GEN0)   /* Vector 150: PWM1 Generator 0 */
VECTOR(tiva_pwm1_gen1, TIVA_IRQ_PWM1_GEN1)   /* Vector 151: PWM1 Generator 1 */
VECTOR(tiva_pwm1_gen2, TIVA_IRQ_PWM1_GEN2)   /* Vector 152: PWM1 Generator 2 */
VECTOR(tiva_pwm1_gen3, TIVA_IRQ_PWM1_GEN3)   /* Vector 153: PWM1 Generator 3 */
VECTOR(tiva_pwm1_fault, TIVA_IRQ_PWM1_FAULT) /* Vector 154: PWM1 Fault */

# endif /* CONFIG_ARMV7M_CMNVECTOR */

#else
#  error "Vectors not known for this Tiva chip"
#endif
