/************************************************************************************
 * arch/arm/include/tiva/cc3200_irq.h
 *
 *   Copyright (C) 2014 Droidifi LLC.
 *   Author: Jim Ewing <jim@droidifi.com>
 *
 *   Adapted for the cc3200 from code:
 *
 *   Copyright (C) 2011-2012 Gregory Nutt.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_TIVA_CC3200_IRQ_H
#define __ARCH_ARM_INCLUDE_TIVA_CC3200_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* External interrupts (vectors >= 16) */

#define TIVA_IRQ_INTERRUPTS (16) /* Vector number of the first external interrupt */

#if defined(CONFIG_ARCH_CHIP_CC3200)
#  define TIVA_IRQ_GPIOA     (16)  /* Vector 16: GPIO Port A */
#  define TIVA_IRQ_GPIOB     (17)  /* Vector 17: GPIO Port B */
#  define TIVA_IRQ_GPIOC     (18)  /* Vector 18: GPIO Port C */
#  define TIVA_IRQ_GPIOD     (19)  /* Vector 19: GPIO Port D */

#  define TIVA_RESERVED_20   (20)  /* Vector 20: Reserved */
#  define TIVA_IRQ_UART0     (21)  /* Vector 21: UART 0 */
#  define TIVA_IRQ_UART1     (22)  /* Vector 22: UART 1 */
#  define TIVA_RESERVED_23   (23)  /* Vector 23: Reserved */
#  define TIVA_IRQ_I2C0      (24)  /* Vector 24: I2C 0 */

#  define TIVA_RESERVED_25   (25)  /* Vector 25: Reserved */
#  define TIVA_RESERVED_26   (26)  /* Vector 26: Reserved */
#  define TIVA_RESERVED_27   (27)  /* Vector 27: Reserved */
#  define TIVA_RESERVED_28   (28)  /* Vector 28: Reserved */
#  define TIVA_RESERVED_29   (29)  /* Vector 29: Reserved */

#  define TIVA_IRQ_ADC0      (30)  /* Vector 30: ADC Sequence 0 */
#  define TIVA_IRQ_ADC1      (31)  /* Vector 31: ADC Sequence 1 */
#  define TIVA_IRQ_ADC2      (32)  /* Vector 32: ADC Sequence 2 */
#  define TIVA_IRQ_ADC3      (33)  /* Vector 33: ADC Sequence 3 */
#  define TIVA_IRQ_WDOG      (34)  /* Vector 34: Watchdog Timers 0 and 1 */
#  define TIVA_IRQ_TIMER0A   (35)  /* Vector 35: 16/32-Bit Timer 0 A */
#  define TIVA_IRQ_TIMER0B   (36)  /* Vector 36: 16/32-Bit Timer 0 B */
#  define TIVA_IRQ_TIMER1A   (37)  /* Vector 37: 16/32-Bit Timer 1 A */
#  define TIVA_IRQ_TIMER1B   (38)  /* Vector 38: 16/32-Bit Timer 1 B */
#  define TIVA_IRQ_TIMER2A   (39)  /* Vector 39: 16/32-Bit Timer 2 A */
#  define TIVA_IRQ_TIMER2B   (40)  /* Vector 40: 16/32-Bit Timer 2 B */

#  define TIVA_RESERVED_41   (41)  /* Vector 41: Reserved */
#  define TIVA_RESERVED_42   (42)  /* Vector 42: Reserved */
#  define TIVA_RESERVED_43   (43)  /* Vector 43: Reserved */

#  define TIVA_IRQ_SYSCON    (44)  /* Vector 44: System Control */
#  define TIVA_IRQ_FLASHCON  (45)  /* Vector 45: FLASH and EEPROM Control */

#  define TIVA_RESERVED_46   (46)  /* Vector 46: Reserved */
#  define TIVA_RESERVED_47   (47)  /* Vector 47: Reserved */
#  define TIVA_RESERVED_48   (48)  /* Vector 48: Reserved */
#  define TIVA_RESERVED_49   (49)  /* Vector 49: Reserved */
#  define TIVA_RESERVED_50   (50)  /* Vector 50: Reserved */

#  define TIVA_IRQ_TIMER3A   (51)  /* Vector 51: 16/32-Bit Timer 3 A */
#  define TIVA_IRQ_TIMER3B   (52)  /* Vector 52: 16/32-Bit Timer 3 B */

#  define TIVA_RESERVED_53   (53)  /* Vector 53: Reserved */
#  define TIVA_RESERVED_54   (54)  /* Vector 54: Reserved */
#  define TIVA_RESERVED_55   (55)  /* Vector 55: Reserved */
#  define TIVA_RESERVED_56   (56)  /* Vector 56: Reserved */
#  define TIVA_RESERVED_57   (57)  /* Vector 57: Reserved */
#  define TIVA_RESERVED_58   (58)  /* Vector 58: Reserved */

#  define TIVA_IRQ_HIBERNATE (59)  /* Vector 59: Hibernation Module */

#  define TIVA_RESERVED_60   (60)  /* Vector 60: Reserved */
#  define TIVA_RESERVED_61   (61)  /* Vector 61: Reserved */

#  define TIVA_IRQ_UDMASOFT  (62)  /* Vector 62: uDMA Software */
#  define TIVA_IRQ_UDMAERROR (63)  /* Vector 63: uDMA Error */

#  define TIVA_IRQ_ADC1_0    (64)  /* Vector 64: ADC1 Sequence 0 */
#  define TIVA_IRQ_ADC1_1    (65)  /* Vector 65: ADC1 Sequence 1 */
#  define TIVA_IRQ_ADC1_2    (66)  /* Vector 66: ADC1 Sequence 2 */
#  define TIVA_IRQ_ADC1_3    (67)  /* Vector 67: ADC1 Sequence 3 */
#  define TIVA_IRQ_I2S0      (68)  /* Vector 68: I2S0 */
#  define TIVA_IRQ_EPI       (69)  /* Vector 69: EPI */

#  define TIVA_RESERVED_70   (70)  /* Vector 70: Reserved */
#  define TIVA_RESERVED_71   (71)  /* Vector 71: Reserved */
#  define TIVA_RESERVED_72   (72)  /* Vector 72: Reserved */
#  define TIVA_RESERVED_73   (73)  /* Vector 73: Reserved */
#  define TIVA_RESERVED_74   (74)  /* Vector 74: Reserved */
#  define TIVA_RESERVED_75   (75)  /* Vector 75: Reserved */
#  define TIVA_RESERVED_76   (76)  /* Vector 76: Reserved */
#  define TIVA_RESERVED_77   (77)  /* Vector 77: Reserved */
#  define TIVA_RESERVED_78   (78)  /* Vector 78: Reserved */
#  define TIVA_RESERVED_79   (79)  /* Vector 79: Reserved */

#  define TIVA_RESERVED_80   (80)  /* Vector 80: Reserved */
#  define TIVA_RESERVED_81   (81)  /* Vector 81: Reserved */
#  define TIVA_RESERVED_82   (82)  /* Vector 82: Reserved */
#  define TIVA_RESERVED_83   (83)  /* Vector 83: Reserved */
#  define TIVA_RESERVED_84   (84)  /* Vector 84: Reserved */
#  define TIVA_RESERVED_85   (85)  /* Vector 85: Reserved */
#  define TIVA_RESERVED_86   (86)  /* Vector 86: Reserved */
#  define TIVA_RESERVED_87   (87)  /* Vector 87: Reserved */
#  define TIVA_RESERVED_88   (88)  /* Vector 88: Reserved */
#  define TIVA_RESERVED_89   (89)  /* Vector 89: Reserved */

#  define TIVA_RESERVED_90   (90)  /* Vector 90: Reserved */
#  define TIVA_RESERVED_91   (91)  /* Vector 91: Reserved */
#  define TIVA_RESERVED_92   (92)  /* Vector 92: Reserved */
#  define TIVA_RESERVED_93   (93)  /* Vector 93: Reserved */
#  define TIVA_RESERVED_94   (94)  /* Vector 94: Reserved */
#  define TIVA_RESERVED_95   (95)  /* Vector 95: Reserved */
#  define TIVA_RESERVED_96   (96)  /* Vector 96: Reserved */
#  define TIVA_RESERVED_97   (97)  /* Vector 97: Reserved */
#  define TIVA_RESERVED_98   (98)  /* Vector 98: Reserved */
#  define TIVA_RESERVED_99   (99)  /* Vector 99: Reserved */

#  define TIVA_RESERVED_100  (100) /* Vector 100: Reserved */
#  define TIVA_RESERVED_101  (101) /* Vector 101: Reserved */
#  define TIVA_RESERVED_102  (102) /* Vector 102: Reserved */
#  define TIVA_RESERVED_103  (103) /* Vector 103: Reserved */
#  define TIVA_RESERVED_104  (104) /* Vector 104: Reserved */
#  define TIVA_RESERVED_105  (105) /* Vector 105: Reserved */
#  define TIVA_RESERVED_106  (106) /* Vector 106: Reserved */
#  define TIVA_RESERVED_107  (107) /* Vector 107: Reserved */
#  define TIVA_RESERVED_108  (108) /* Vector 108: Reserved  */
#  define TIVA_RESERVED_109  (109) /* Vector 109: Reserved  */
#  define TIVA_RESERVED_110  (110) /* Vector 110: Reserved  */
#  define TIVA_RESERVED_111  (111) /* Vector 111: Reserved  */
#  define TIVA_RESERVED_112  (112) /* Vector 112: Reserved  */
#  define TIVA_RESERVED_113  (113) /* Vector 113: Reserved  */
#  define TIVA_RESERVED_114  (114) /* Vector 114: Reserved  */
#  define TIVA_RESERVED_115  (115) /* Vector 115: Reserved  */
#  define TIVA_RESERVED_116  (116) /* Vector 116: Reserved  */
#  define TIVA_RESERVED_117  (117) /* Vector 117: Reserved  */
#  define TIVA_RESERVED_118  (118) /* Vector 118: Reserved  */
#  define TIVA_RESERVED_119  (119) /* Vector 119: Reserved  */

#  define TIVA_RESERVED_120  (120) /* Vector 120: Reserved  */
#  define TIVA_RESERVED_121  (121) /* Vector 121: Reserved  */
#  define TIVA_IRQ_SYSTEM    (122) /* Vector 122: System Exception (imprecise) */
#  define TIVA_RESERVED_123  (123) /* Vector 123: Reserved */
#  define TIVA_RESERVED_124  (124) /* Vector 124: Reserved */
#  define TIVA_RESERVED_125  (125) /* Vector 125: Reserved */
#  define TIVA_RESERVED_126  (126) /* Vector 126: Reserved */
#  define TIVA_RESERVED_127  (127) /* Vector 127: Reserved */
#  define TIVA_RESERVED_128  (128) /* Vector 128: Reserved */
#  define TIVA_RESERVED_129  (129) /* Vector 129: Reserved */

#  define TIVA_RESERVED_130  (130) /* Vector 130: Reserved */
#  define TIVA_RESERVED_131  (131) /* Vector 131: Reserved */
#  define TIVA_RESERVED_132  (132) /* Vector 132: Reserved */
#  define TIVA_RESERVED_133  (133) /* Vector 133: Reserved */
#  define TIVA_RESERVED_134  (134) /* Vector 134: Reserved */
#  define TIVA_RESERVED_135  (135) /* Vector 135: Reserved */
#  define TIVA_RESERVED_136  (136) /* Vector 136: Reserved */
#  define TIVA_RESERVED_137  (137) /* Vector 137: Reserved */
#  define TIVA_RESERVED_138  (138) /* Vector 138: Reserved */
#  define TIVA_RESERVED_139  (139) /* Vector 139: Reserved */

#  define TIVA_RESERVED_140  (140) /* Vector 140: Reserved */
#  define TIVA_RESERVED_141  (141) /* Vector 141: Reserved */
#  define TIVA_RESERVED_142  (142) /* Vector 142: Reserved */
#  define TIVA_RESERVED_143  (143) /* Vector 143: Reserved */
#  define TIVA_RESERVED_144  (144) /* Vector 144: Reserved */
#  define TIVA_RESERVED_145  (145) /* Vector 145: Reserved */
#  define TIVA_RESERVED_146  (146) /* Vector 146: Reserved */
#  define TIVA_RESERVED_147  (147) /* Vector 147: Reserved */
#  define TIVA_RESERVED_148  (148) /* Vector 148: Reserved */
#  define TIVA_RESERVED_149  (149) /* Vector 149: Reserved */

#  define TIVA_RESERVED_150  (150) /* Vector 150: Reserved */
#  define TIVA_RESERVED_151  (151) /* Vector 151: Reserved */
#  define TIVA_RESERVED_152  (152) /* Vector 152: Reserved */
#  define TIVA_RESERVED_153  (153) /* Vector 153: Reserved */
#  define TIVA_RESERVED_154  (154) /* Vector 154: Reserved */
#  define TIVA_RESERVED_155  (155) /* Vector 155: Reserved */
#  define TIVA_RESERVED_156  (156) /* Vector 156: Reserved */
#  define TIVA_RESERVED_157  (157) /* Vector 157: Reserved */
#  define TIVA_RESERVED_158  (158) /* Vector 158: Reserved */
#  define TIVA_RESERVED_159  (159) /* Vector 159: Reserved */

#  define TIVA_RESERVED_160  (160) /* Vector 160: Reserved */
#  define TIVA_RESERVED_161  (161) /* Vector 161: Reserved */
#  define TIVA_RESERVED_162  (162) /* Vector 162: Reserved */
#  define TIVA_RESERVED_163  (163) /* Vector 162: Reserved */
#  define TIVA_IRQ_SHA       (164) /* Vector 162: SHA HW */
#  define TIVA_RESERVED_165  (165) /* Vector 165: Reserved */
#  define TIVA_RESERVED_166  (166) /* Vector 166: Reserved */
#  define TIVA_IRQ_AES       (167) /* Vector 167: AES HW */
#  define TIVA_RESERVED_168  (168) /* Vector 168: Reserved */
#  define TIVA_IRQ_DES       (169) /* Vector 169: DES HW */

#  define TIVA_RESERVED_170  (170) /* Vector 170: Reserved */
#  define TIVA_RESERVED_171  (171) /* Vector 171: Reserved */
#  define TIVA_RESERVED_172  (172) /* Vector 172: Reserved */
#  define TIVA_RESERVED_173  (173) /* Vector 173: Reserved */
#  define TIVA_RESERVED_174  (174) /* Vector 174: Reserved */
#  define TIVA_RESERVED_175  (175) /* Vector 175: Reserved */
#  define TIVA_RESERVED_176  (176) /* Vector 176: Reserved */
#  define TIVA_IRQ_MC_ASP_0  (177) /* Vector 177: McASP 0 */
#  define TIVA_RESERVED_178  (178) /* Vector 178: Reserved */
#  define TIVA_IRQ_CAM_0     (179) /* Vector 179: Camera A0 */

#  define TIVA_RESERVED_180  (180) /* Vector 180: Reserved */
#  define TIVA_RESERVED_181  (181) /* Vector 181: Reserved */
#  define TIVA_RESERVED_182  (182) /* Vector 182: Reserved */
#  define TIVA_RESERVED_183  (183) /* Vector 183: Reserved */
#  define TIVA_IRQ_RAM_ERR   (184) /* Vector 184: RAM Err */
#  define TIVA_RESERVED_185  (185) /* Vector 185: Reserved */
#  define TIVA_RESERVED_186  (186) /* Vector 186: Reserved */
#  define TIVA_IRQ_NWPIC     (187) /* Vector 187: NWP IC interocessor comm */
#  define TIVA_IRQ_PRCM      (188) /* Vector 188: Pwr, Rst, Clk */
#  define TIVA_IRQ_TOPDIE    (189) /* Vector 189: From Top Die */

#  define TIVA_RESERVED_190  (190) /* Vector 190: Reserved */
#  define TIVA_IRQ_MCSPI_S0  (191) /* Vector 191: SPI S0 */
#  define TIVA_IRQ_MCSPI_A1  (192) /* Vector 191: SPI A0 */
#  define TIVA_IRQ_MCSPI_A2  (193) /* Vector 191: SPI A1 */
#  define TIVA_RESERVED_194  (194) /* Vector 194: Reserved */

#  define NR_IRQS            (195) /* (Really fewer because of reserved vectors) */

#else
#  error "IRQ Numbers not known for this Tiva chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_TIVA_CC3200_IRQ_H */
