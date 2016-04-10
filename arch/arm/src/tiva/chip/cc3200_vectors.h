/************************************************************************************
 * arch/arm/src/tiva/chip/cc3200_vectors.h
 *
 *   Copyright (C) 2014 Droidifi LLC. All rights reserved.
 *   Author: Jim Ewing <jim@droidifi.com>
 *
 *   Adapted for the cc3200 from code:
 *
 *   Copyright (C) Gregory Nutt.
 *   Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name Droidifi nor the names of its contributors may be
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
 * Pre-processor Definitions
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

#if defined(CONFIG_ARCH_CHIP_CC3200)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#  ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 155 interrupt table entries for I/O interrupts. */

ARMV7M_PERIPHERAL_INTERRUPTS 155

#  else
VECTOR(tiva_gpioa, TIVA_IRQ_GPIOA)       /* Vector 16: GPIO Port A */
VECTOR(tiva_gpiob, TIVA_IRQ_GPIOB)       /* Vector 17: GPIO Port B */
VECTOR(tiva_gpioc, TIVA_IRQ_GPIOC)       /* Vector 18: GPIO Port C */
VECTOR(tiva_gpiod, TIVA_IRQ_GPIOD)       /* Vector 19: GPIO Port D */
UNUSED(TIVA_RESERVED_20)                 /* Vector 20: Reserved */
VECTOR(tiva_uart0, TIVA_IRQ_UART0)       /* Vector 21: UART 0 */
VECTOR(tiva_uart1, TIVA_IRQ_UART1)       /* Vector 22: UART 1 */
UNUSED(TIVA_RESERVED_23)                 /* Vector 23: Reserved */
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
VECTOR(tiva_wdog, TIVA_IRQ_WDOG)         /* Vector 34: Watchdog Timers 0 and 1 */
VECTOR(tiva_timer0a, TIVA_IRQ_TIMER0A)   /* Vector 35: 16/32-Bit Timer 0 A */
VECTOR(tiva_timer0b, TIVA_IRQ_TIMER0B)   /* Vector 36: 16/32-Bit Timer 0 B */
VECTOR(tiva_timer1a, TIVA_IRQ_TIMER1A)   /* Vector 37: 16/32-Bit Timer 1 A */
VECTOR(tiva_timer1b, TIVA_IRQ_TIMER1B)   /* Vector 38: 16/32-Bit Timer 1 B */
VECTOR(tiva_timer2a, TIVA_IRQ_TIMER2A)   /* Vector 39: 16/32-Bit Timer 2 A */

VECTOR(tiva_timer2b, TIVA_IRQ_TIMER2B)   /* Vector 40: 16/32-Bit Timer 2 B */
UNUSED(TIVA_RESERVED_41)                 /* Vector 41: Reserved */
UNUSED(TIVA_RESERVED_42)                 /* Vector 42: Reserved */
UNUSED(TIVA_RESERVED_43)                 /* Vector 43: Reserved */
VECTOR(tiva_syscon, TIVA_IRQ_SYSCON)     /* Vector 44: System Control */
VECTOR(tiva_flashcon, TIVA_IRQ_FLASHCON) /* Vector 45: FLASH and EEPROM Control */
UNUSED(TIVA_RESERVED_46)                 /* Vector 46: Reserved */
UNUSED(TIVA_RESERVED_47)                 /* Vector 47: Reserved */
UNUSED(TIVA_RESERVED_48)                 /* Vector 48: Reserved */
UNUSED(TIVA_RESERVED_49)                 /* Vector 49: Reserved */

UNUSED(TIVA_RESERVED_50)                 /* Vector 50: Reserved */
VECTOR(tiva_timer3a, TIVA_IRQ_TIMER3A)   /* Vector 51: 16/32-Bit Timer 3 A */
VECTOR(tiva_timer3b, TIVA_IRQ_TIMER3B)   /* Vector 52: 16/32-Bit Timer 3 B */
UNUSED(TIVA_RESERVED_53)                 /* Vector 53: Reserved */
UNUSED(TIVA_RESERVED_54)                 /* Vector 54: Reserved */
UNUSED(TIVA_RESERVED_55)                 /* Vector 55: Reserved */
UNUSED(TIVA_RESERVED_56)                 /* Vector 56: Reserved */
UNUSED(TIVA_RESERVED_57)                 /* Vector 57: Reserved */
UNUSED(TIVA_RESERVED_58)                 /* Vector 58: Reserved */
VECTOR(tiva_hibernate, TIVA_IRQ_HIBERNATE) /* Vector 59: Hibernation Module */

UNUSED(TIVA_RESERVED_60)                 /* Vector 60: Reserved */
UNUSED(TIVA_RESERVED_61)                 /* Vector 61: Reserved */
VECTOR(tiva_udmasoft, TIVA_IRQ_UDMASOFT) /* Vector 62: uDMA Software */
VECTOR(tiva_udmaerro, TIVA_IRQ_UDMAERROR)/* Vector 63: uDMA Error */
UNUSED(TIVA_RESERVED_64)                 /* Vector 64: Reserved */
UNUSED(TIVA_RESERVED_65)                 /* Vector 65: Reserved */
UNUSED(TIVA_RESERVED_66)                 /* Vector 66: Reserved */
UNUSED(TIVA_RESERVED_67)                 /* Vector 67: Reserved */
VECTOR(tiva_i2s0, TIVA_IRQ_I2S0)         /* Vector 68: I2S 0 */
VECTOR(tiva_epi, TIVA_IRQ_EPI)           /* Vector 69: EPI  */

UNUSED(TIVA_RESERVED_70)                 /* Vector 70: Reserved */
UNUSED(TIVA_RESERVED_71)                 /* Vector 71: Reserved */
UNUSED(TIVA_RESERVED_72)                 /* Vector 72: Reserved */
UNUSED(TIVA_RESERVED_73)                 /* Vector 73: Reserved */
UNUSED(TIVA_RESERVED_74)                 /* Vector 74: Reserved */
UNUSED(TIVA_RESERVED_75)                 /* Vector 75: Reserved */
UNUSED(TIVA_RESERVED_76)                 /* Vector 76: Reserved */
UNUSED(TIVA_RESERVED_77)                 /* Vector 77: Reserved */
UNUSED(TIVA_RESERVED_78)                 /* Vector 78: Reserved */
UNUSED(TIVA_RESERVED_79)                 /* Vector 79: Reserved */

UNUSED(TIVA_RESERVED_80)                 /* Vector 80: Reserved */
UNUSED(TIVA_RESERVED_81)                 /* Vector 81: Reserved */
UNUSED(TIVA_RESERVED_82)                 /* Vector 82: Reserved */
UNUSED(TIVA_RESERVED_83)                 /* Vector 83: Reserved */
UNUSED(TIVA_RESERVED_84)                 /* Vector 84: Reserved */
UNUSED(TIVA_RESERVED_85)                 /* Vector 85: Reserved */
UNUSED(TIVA_RESERVED_86)                 /* Vector 86: Reserved */
UNUSED(TIVA_RESERVED_87)                 /* Vector 87: Reserved */
UNUSED(TIVA_RESERVED_88)                 /* Vector 88: Reserved */
UNUSED(TIVA_RESERVED_89)                 /* Vector 89: Reserved */

UNUSED(TIVA_RESERVED_90)                 /* Vector 90: Reserved */
UNUSED(TIVA_RESERVED_91)                 /* Vector 91: Reserved */
UNUSED(TIVA_RESERVED_92)                 /* Vector 92: Reserved */
UNUSED(TIVA_RESERVED_93)                 /* Vector 93: Reserved */
UNUSED(TIVA_RESERVED_94)                 /* Vector 94: Reserved */
UNUSED(TIVA_RESERVED_95)                 /* Vector 95: Reserved */
UNUSED(TIVA_RESERVED_96)                 /* Vector 96: Reserved */
UNUSED(TIVA_RESERVED_97)                 /* Vector 97: Reserved */
UNUSED(TIVA_RESERVED_98)                 /* Vector 98: Reserved */
UNUSED(TIVA_RESERVED_99)                 /* Vector 99: Reserved */
UNUSED(TIVA_RESERVED_100)                /* Vector 100: Reserved */
UNUSED(TIVA_RESERVED_101)                /* Vector 101: Reserved */
UNUSED(TIVA_RESERVED_102)                /* Vector 102: Reserved */
UNUSED(TIVA_RESERVED_103)                /* Vector 103: Reserved */
UNUSED(TIVA_RESERVED_104)                /* Vector 104: Reserved */
UNUSED(TIVA_RESERVED_105)                /* Vector 105: Reserved */
UNUSED(TIVA_RESERVED_106)                /* Vector 106: Reserved */
UNUSED(TIVA_RESERVED_107)                /* Vector 107: Reserved */
UNUSED(TIVA_RESERVED_108)                /* Vector 108: Reserved */
UNUSED(TIVA_RESERVED_109)                /* Vector 109: Reserved */

UNUSED(TIVA_RESERVED_110)                /* Vector 110: Reserved */
UNUSED(TIVA_RESERVED_111)                /* Vector 111: Reserved */
UNUSED(TIVA_RESERVED_112)                /* Vector 112: Reserved */
UNUSED(TIVA_RESERVED_113)                /* Vector 113: Reserved */
UNUSED(TIVA_RESERVED_114)                /* Vector 114: Reserved */
UNUSED(TIVA_RESERVED_115)                /* Vector 115: Reserved */
UNUSED(TIVA_RESERVED_116)                /* Vector 116: Reserved */
UNUSED(TIVA_RESERVED_117)                /* Vector 117: Reserved */
UNUSED(TIVA_RESERVED_118)                /* Vector 118: Reserved */
UNUSED(TIVA_RESERVED_119)                /* Vector 119: Reserved */

UNUSED(TIVA_RESERVED_120)                /* Vector 120: Reserved */
UNUSED(TIVA_RESERVED_121)                /* Vector 121: Reserved */
VECTOR(tiva_system, TIVA_IRQ_SYSTEM)     /* Vector 122: System Exception (imprecise) */
UNUSED(TIVA_RESERVED_123)                /* Vector 123: Reserved */
UNUSED(TIVA_RESERVED_124)                /* Vector 124: Reserved */
UNUSED(TIVA_RESERVED_125)                /* Vector 125: Reserved */
UNUSED(TIVA_RESERVED_126)                /* Vector 126: Reserved */
UNUSED(TIVA_RESERVED_127)                /* Vector 127: Reserved */
UNUSED(TIVA_RESERVED_128)                /* Vector 128: Reserved */
UNUSED(TIVA_RESERVED_129)                /* Vector 129: Reserved */

UNUSED(TIVA_RESERVED_130)                /* Vector 130: Reserved */
UNUSED(TIVA_RESERVED_131)                /* Vector 131: Reserved */
UNUSED(TIVA_RESERVED_132)                /* Vector 132: Reserved */
UNUSED(TIVA_RESERVED_133)                /* Vector 133: Reserved */
UNUSED(TIVA_RESERVED_134)                /* Vector 134: Reserved */
UNUSED(TIVA_RESERVED_135)                /* Vector 135: Reserved */
UNUSED(TIVA_RESERVED_136)                /* Vector 136: Reserved */
UNUSED(TIVA_RESERVED_137)                /* Vector 137: Reserved */
UNUSED(TIVA_RESERVED_138)                /* Vector 138: Reserved */
UNUSED(TIVA_RESERVED_139)                /* Vector 139: Reserved */

UNUSED(TIVA_RESERVED_140)                /* Vector 140: Reserved */
UNUSED(TIVA_RESERVED_141)                /* Vector 141: Reserved */
UNUSED(TIVA_RESERVED_142)                /* Vector 142: Reserved */
UNUSED(TIVA_RESERVED_143)                /* Vector 143: Reserved */
UNUSED(TIVA_RESERVED_144)                /* Vector 144: Reserved */
UNUSED(TIVA_RESERVED_145)                /* Vector 145: Reserved */
UNUSED(TIVA_RESERVED_146)                /* Vector 146: Reserved */
UNUSED(TIVA_RESERVED_147)                /* Vector 147: Reserved */
UNUSED(TIVA_RESERVED_148)                /* Vector 148: Reserved */
UNUSED(TIVA_RESERVED_149)                /* Vector 149: Reserved */

UNUSED(TIVA_RESERVED_150)                /* Vector 150: Reserved */
UNUSED(TIVA_RESERVED_151)                /* Vector 151: Reserved */
UNUSED(TIVA_RESERVED_152)                /* Vector 152: Reserved */
UNUSED(TIVA_RESERVED_153)                /* Vector 153: Reserved */
UNUSED(TIVA_RESERVED_154)                /* Vector 154: Reserved */
UNUSED(TIVA_RESERVED_155)                /* Vector 155: Reserved */
UNUSED(TIVA_RESERVED_156)                /* Vector 156: Reserved */
UNUSED(TIVA_RESERVED_157)                /* Vector 157: Reserved */
UNUSED(TIVA_RESERVED_158)                /* Vector 158: Reserved */
UNUSED(TIVA_RESERVED_159)                /* Vector 159: Reserved */

UNUSED(TIVA_RESERVED_160)                /* Vector 160: Reserved */
UNUSED(TIVA_RESERVED_161)                /* Vector 161: Reserved */
UNUSED(TIVA_RESERVED_162)                /* Vector 162: Reserved */
UNUSED(TIVA_RESERVED_163)                /* Vector 163: Reserved */
UNUSED(TIVA_RESERVED_164)                /* Vector 164: SHA HW */
UNUSED(TIVA_RESERVED_165)                /* Vector 165: Reserved */
UNUSED(TIVA_RESERVED_166)                /* Vector 166: Reserved */
UNUSED(TIVA_RESERVED_167)                /* Vector 167: AES HW */
UNUSED(TIVA_RESERVED_168)                /* Vector 168: Reserved */
UNUSED(TIVA_RESERVED_169)                /* Vector 169: DES HW */
UNUSED(TIVA_RESERVED_170)                /* Vector 170: Reserved */
UNUSED(TIVA_RESERVED_171)                /* Vector 171: Reserved */
UNUSED(TIVA_RESERVED_172)                /* Vector 172: Reserved */
UNUSED(TIVA_RESERVED_173)                /* Vector 173: Reserved */
UNUSED(TIVA_RESERVED_174)                /* Vector 174: Reserved */
UNUSED(TIVA_RESERVED_175)                /* Vector 175: SDIO */
UNUSED(TIVA_RESERVED_176)                /* Vector 176: Reserved */
UNUSED(TIVA_RESERVED_177)                /* Vector 177: McASP 0 */
UNUSED(TIVA_RESERVED_178)                /* Vector 178: Reserved */
UNUSED(TIVA_RESERVED_179)                /* Vector 179: Camera A0 */

UNUSED(TIVA_RESERVED_180)                /* Vector 180: Reserved */
UNUSED(TIVA_RESERVED_181)                /* Vector 181: Reserved */
UNUSED(TIVA_RESERVED_182)                /* Vector 182: Reserved */
UNUSED(TIVA_RESERVED_183)                /* Vector 183: Reserved */
UNUSED(TIVA_RESERVED_184)                /* Vector 184: RAM Err */
UNUSED(TIVA_RESERVED_185)                /* Vector 185: Reserved */
UNUSED(TIVA_RESERVED_186)                /* Vector 186: Reserved */
UNUSED(TIVA_RESERVED_187)                /* Vector 187: NWP IC interocessor comm */
UNUSED(TIVA_RESERVED_188)                /* Vector 188: Pwr, Rst, Clk */
UNUSED(TIVA_RESERVED_189)                /* Vector 189: From Top Die */

UNUSED(TIVA_RESERVED_190)                /* Vector 190: Reserved */
UNUSED(TIVA_RESERVED_191)                /* Vector 191: SPI S0 */
UNUSED(TIVA_RESERVED_192)                /* Vector 192: SPI A0 */
UNUSED(TIVA_RESERVED_193)                /* Vector 193: SPI A1 */
UNUSED(TIVA_RESERVED_194)                /* Vector 194: Reserved */

# endif /* CONFIG_ARMV7M_CMNVECTOR */

#else
#  error "Vectors not known for this Tiva chip"
#endif /* defined(CONFIG_ARCH_CHIP_CC3200) */
