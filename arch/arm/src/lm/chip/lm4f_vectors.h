/************************************************************************************
 * arch/arm/src/lm/chip/lm4f_vectors.S
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

/* This file is included by lm_vectors.S.  It provides the macro VECTOR that
 * supplies ach Stellaris vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/lm/lm4f_irq.h.
 * lm_vectors.S will define the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

#if defined(CONFIG_ARCH_CHIP_LM4F120)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#  ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 155 interrupt table entries for I/O interrupts. */

ARMV7M_PERIPHERAL_INTERRUPTS 155

#  else

VECTOR(lm_gpioa, LM_IRQ_GPIOA)         /* Vector 16: GPIO Port A */
VECTOR(lm_gpiob, LM_IRQ_GPIOB)         /* Vector 17: GPIO Port B */
VECTOR(lm_gpioc, LM_IRQ_GPIOC)         /* Vector 18: GPIO Port C */
VECTOR(lm_gpiod, LM_IRQ_GPIOD)         /* Vector 19: GPIO Port D */

VECTOR(lm_gpioe, LM_IRQ_GPIOE)         /* Vector 20: GPIO Port E */
VECTOR(lm_uart0, LM_IRQ_UART0)         /* Vector 21: UART 0 */
VECTOR(lm_uart1, LM_IRQ_UART1)         /* Vector 22: UART 1 */
VECTOR(lm_ssi0, LM_IRQ_SSI0)           /* Vector 23: SSI 0 */
VECTOR(lm_i2c0, LM_IRQ_I2C0)           /* Vector 24: I2C 0 */
UNUSED(LM_RESERVED_25)                 /* Vector 25: Reserved */
UNUSED(LM_RESERVED_26)                 /* Vector 26: Reserved */
UNUSED(LM_RESERVED_27)                 /* Vector 27: Reserved */
UNUSED(LM_RESERVED_28)                 /* Vector 28: Reserved */
UNUSED(LM_RESERVED_29)                 /* Vector 29: Reserved */

VECTOR(lm_adc0, LM_IRQ_ADC0)           /* Vector 30: ADC Sequence 0 */
VECTOR(lm_adc1, LM_IRQ_ADC1)           /* Vector 31: ADC Sequence 1 */
VECTOR(lm_adc2, LM_IRQ_ADC2)           /* Vector 32: ADC Sequence 2 */
VECTOR(lm_adc3, LM_IRQ_ADC3)           /* Vector 33: ADC Sequence 3 */
VECTOR(lm_wdog, LM_IRQ_WDOG)           /* Vector 34: Watchdog Timers 0 and 1 */
VECTOR(lm_timer0a, LM_IRQ_TIMER0A)     /* Vector 35: 16/32-Bit Timer 0 A */
VECTOR(lm_timer0b, LM_IRQ_TIMER0B)     /* Vector 36: 16/32-Bit Timer 0 B */
VECTOR(lm_timer1a, LM_IRQ_TIMER1A)     /* Vector 37: 16/32-Bit Timer 1 A */
VECTOR(lm_timer1b, LM_IRQ_TIMER1B)     /* Vector 38: 16/32-Bit Timer 1 B */
VECTOR(lm_timer2a, LM_IRQ_TIMER2A)     /* Vector 39: 16/32-Bit Timer 2 A */

VECTOR(lm_timer2b, LM_IRQ_TIMER2B)     /* Vector 40: 16/32-Bit Timer 2 B */
VECTOR(lm_compare0, LM_IRQ_COMPARE0)   /* Vector 41: Analog Comparator 0 */
VECTOR(lm_compare1, LM_IRQ_COMPARE1)   /* Vector 42: Analog Comparator 1 */
UNUSED(LM_RESERVED_43)                 /* Vector 43: Reserved */
VECTOR(lm_syscon, LM_IRQ_SYSCON)       /* Vector 44: System Control */
VECTOR(lm_flashcon, LM_IRQ_FLASHCON)   /* Vector 45: FLASH and EEPROM Control */
VECTOR(lm_gpiof, LM_IRQ_GPIOF)         /* Vector 46: GPIO Port F */
UNUSED(LM_RESERVED_47)                 /* Vector 47: Reserved */
UNUSED(LM_RESERVED_48)                 /* Vector 48: Reserved */
VECTOR(lm_uart2, LM_IRQ_UART2)         /* Vector 22: UART 2 */

VECTOR(lm_ssi1, LM_IRQ_SSI1)           /* Vector 50: SSI 1 */
VECTOR(lm_timer3a, LM_IRQ_TIMER3A)     /* Vector 51: 16/32-Bit Timer 3 A */
VECTOR(lm_timer3b, LM_IRQ_TIMER3B)     /* Vector 52: 16/32-Bit Timer 3 B */
VECTOR(lm_i2c1, LM_IRQ_I2C1)           /* Vector 53: I2C 1 */
UNUSED(LM_RESERVED_54)                 /* Vector 54: Reserved */
VECTOR(lm_can0, LM_IRQ_CAN0)           /* Vector 55: CAN 0 */
UNUSED(LM_RESERVED_56)                 /* Vector 56: Reserved */
UNUSED(LM_RESERVED_57)                 /* Vector 57: Reserved */
UNUSED(LM_RESERVED_58)                 /* Vector 58: Reserved */
VECTOR(lm_hibernate, LM_IRQ_HIBERNATE) /* Vector 59: Hibernation Module */

VECTOR(lm_usb, LM_IRQ_USB)             /* Vector 60: USB */
UNUSED(LM_RESERVED_61)                 /* Vector 61: Reserved */
VECTOR(lm_udmasoft, LM_IRQ_UDMASOFT)   /* Vector 62: uDMA Software */
VECTOR(lm_udmaerro, LM_IRQ_UDMAERROR)  /* Vector 63: uDMA Error */
VECTOR(lm_adc1_0, LM_IRQ_ADC1_0)       /* Vector 64: ADC1 Sequence 0 */
VECTOR(lm_adc1_1, LM_IRQ_ADC1_1)       /* Vector 65: ADC1 Sequence 1 */
VECTOR(lm_adc1_2, LM_IRQ_ADC1_2)       /* Vector 66: ADC1 Sequence 2 */
VECTOR(lm_adc1_3, LM_IRQ_ADC1_3)       /* Vector 67: ADC1 Sequence 3 */
UNUSED(LM_RESERVED_68)                 /* Vector 68: Reserved */
UNUSED(LM_RESERVED_69)                 /* Vector 69: Reserved */

UNUSED(LM_RESERVED_70)                 /* Vector 70: Reserved */
UNUSED(LM_RESERVED_71)                 /* Vector 71: Reserved */
UNUSED(LM_RESERVED_72)                 /* Vector 72: Reserved */
VECTOR(lm_ssi2, LM_IRQ_SSI2)           /* Vector 73: SSI 2 */
VECTOR(lm_ssi3, LM_IRQ_SSI3)           /* Vector 74: SSI 3 */
VECTOR(lm_uart3, LM_IRQ_UART3)         /* Vector 75: UART 3 */
VECTOR(lm_uart4, LM_IRQ_UART4)         /* Vector 76: UART 4 */
VECTOR(lm_uart5, LM_IRQ_UART5)         /* Vector 77: UART 5 */
VECTOR(lm_uart6, LM_IRQ_UART6)         /* Vector 78: UART 6 */
VECTOR(lm_uart7, LM_IRQ_UART7)         /* Vector 79: UART 7 */

UNUSED(LM_RESERVED_80)                 /* Vector 80: Reserved */
UNUSED(LM_RESERVED_81)                 /* Vector 81: Reserved */
UNUSED(LM_RESERVED_82)                 /* Vector 82: Reserved */
UNUSED(LM_RESERVED_83)                 /* Vector 83: Reserved */
VECTOR(lm_i2c2, LM_IRQ_I2C2)           /* Vector 84: I2C 2 */
VECTOR(lm_i2c3, LM_IRQ_I2C3)           /* Vector 85: I2C 3 */
VECTOR(lm_timer4a, LM_IRQ_TIMER4A)     /* Vector 86: 16/32-Bit Timer 4 A */
VECTOR(lm_timer4b, LM_IRQ_TIMER4B)     /* Vector 87: 16/32-Bit Timer 4 B */
UNUSED(LM_RESERVED_88)                 /* Vector 88: Reserved */
UNUSED(LM_RESERVED_89)                 /* Vector 89: Reserved */

UNUSED(LM_RESERVED_90)                 /* Vector 90: Reserved */
UNUSED(LM_RESERVED_91)                 /* Vector 91: Reserved */
UNUSED(LM_RESERVED_92)                 /* Vector 92: Reserved */
UNUSED(LM_RESERVED_93)                 /* Vector 93: Reserved */
UNUSED(LM_RESERVED_94)                 /* Vector 94: Reserved */
UNUSED(LM_RESERVED_95)                 /* Vector 95: Reserved */
UNUSED(LM_RESERVED_96)                 /* Vector 96: Reserved */
UNUSED(LM_RESERVED_97)                 /* Vector 97: Reserved */
UNUSED(LM_RESERVED_98)                 /* Vector 98: Reserved */
UNUSED(LM_RESERVED_99)                 /* Vector 99: Reserved */

UNUSED(LM_RESERVED_100)                /* Vector 100: Reserved */
UNUSED(LM_RESERVED_101)                /* Vector 101: Reserved */
UNUSED(LM_RESERVED_102)                /* Vector 102: Reserved */
UNUSED(LM_RESERVED_103)                /* Vector 103: Reserved */
UNUSED(LM_RESERVED_104)                /* Vector 104: Reserved */
UNUSED(LM_RESERVED_105)                /* Vector 105: Reserved */
UNUSED(LM_RESERVED_106)                /* Vector 106: Reserved */
UNUSED(LM_RESERVED_107)                /* Vector 107: Reserved */
VECTOR(lm_timer5a, LM_IRQ_TIMER5A)     /* Vector 108: 16/32-Bit Timer 5 A */
VECTOR(lm_timer5b, LM_IRQ_TIMER5B)     /* Vector 109: 16/32-Bit Timer 5 B */

VECTOR(lm_wtimer0a, LM_IRQ_WTIMER0A)   /* Vector 110: 32/64-Bit Timer 0 A */
VECTOR(lm_wtimer0b, LM_IRQ_WTIMER0B)   /* Vector 111: 32/64-Bit Timer 0 B */
VECTOR(lm_wtimer1a, LM_IRQ_WTIMER1A)   /* Vector 112: 32/64-Bit Timer 1 A */
VECTOR(lm_wtimer1b, LM_IRQ_WTIMER1B)   /* Vector 113: 32/64-Bit Timer 1 B */
VECTOR(lm_wtimer2a, LM_IRQ_WTIMER2A)   /* Vector 114: 32/64-Bit Timer 2 A */
VECTOR(lm_wtimer2b, LM_IRQ_WTIMER2B)   /* Vector 115: 32/64-Bit Timer 2 B */
VECTOR(lm_wtimer3a, LM_IRQ_WTIMER3A)   /* Vector 116: 32/64-Bit Timer 3 A */
VECTOR(lm_wtimer3b, LM_IRQ_WTIMER3B)   /* Vector 117: 32/64-Bit Timer 3 B */
VECTOR(lm_wtimer4a, LM_IRQ_WTIMER4A)   /* Vector 118: 32/64-Bit Timer 4 A */
VECTOR(lm_WTIMER4B, LM_IRQ_WTIMER4B)   /* Vector 119: 32/64-Bit Timer 4 B */

VECTOR(lm_wtimer5a, LM_IRQ_WTIMER5A)   /* Vector 120: 32/64-Bit Timer 5 A */
VECTOR(lm_wtimer5b, LM_IRQ_WTIMER5B)   /* Vector 121: 32/64-Bit Timer 5 B */
VECTOR(lm_system, LM_IRQ_SYSTEM)       /* Vector 122: System Exception (imprecise) */
UNUSED(LM_RESERVED_123)                /* Vector 123: Reserved */
UNUSED(LM_RESERVED_124)                /* Vector 124: Reserved */
UNUSED(LM_RESERVED_125)                /* Vector 125: Reserved */
UNUSED(LM_RESERVED_126)                /* Vector 126: Reserved */
UNUSED(LM_RESERVED_127)                /* Vector 127: Reserved */
UNUSED(LM_RESERVED_128)                /* Vector 128: Reserved */
UNUSED(LM_RESERVED_129)                /* Vector 129: Reserved */

UNUSED(LM_RESERVED_130)                /* Vector 130: Reserved */
UNUSED(LM_RESERVED_131)                /* Vector 131: Reserved */
UNUSED(LM_RESERVED_132)                /* Vector 132: Reserved */
UNUSED(LM_RESERVED_133)                /* Vector 133: Reserved */
UNUSED(LM_RESERVED_134)                /* Vector 134: Reserved */
UNUSED(LM_RESERVED_135)                /* Vector 135: Reserved */
UNUSED(LM_RESERVED_136)                /* Vector 136: Reserved */
UNUSED(LM_RESERVED_137)                /* Vector 137: Reserved */
UNUSED(LM_RESERVED_138)                /* Vector 138: Reserved */
UNUSED(LM_RESERVED_139)                /* Vector 139: Reserved */

UNUSED(LM_RESERVED_140)                /* Vector 140: Reserved */
UNUSED(LM_RESERVED_141)                /* Vector 141: Reserved */
UNUSED(LM_RESERVED_142)                /* Vector 142: Reserved */
UNUSED(LM_RESERVED_143)                /* Vector 143: Reserved */
UNUSED(LM_RESERVED_144)                /* Vector 144: Reserved */
UNUSED(LM_RESERVED_145)                /* Vector 145: Reserved */
UNUSED(LM_RESERVED_146)                /* Vector 146: Reserved */
UNUSED(LM_RESERVED_147)                /* Vector 147: Reserved */
UNUSED(LM_RESERVED_148)                /* Vector 148: Reserved */
UNUSED(LM_RESERVED_149)                /* Vector 149: Reserved */

UNUSED(LM_RESERVED_150)                /* Vector 150: Reserved */
UNUSED(LM_RESERVED_151)                /* Vector 151: Reserved */
UNUSED(LM_RESERVED_152)                /* Vector 152: Reserved */
UNUSED(LM_RESERVED_153)                /* Vector 153: Reserved */
UNUSED(LM_RESERVED_154)                /* Vector 154: Reserved */

# endif /* CONFIG_ARMV7M_CMNVECTOR */

#else
#  error "Vectors not known for this Stellaris chip"
#endif
