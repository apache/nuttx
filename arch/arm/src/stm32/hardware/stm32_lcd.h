/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_lcd.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_LCD_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/* These definitions are valid only if the MCU supports a segment LCD */

#if STM32_NLCD > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_LCD_CR_OFFSET      0x0000 /* LCD control register */
#define STM32_LCD_FCR_OFFSET     0x0004 /* LCD frame control register */
#define STM32_LCD_SR_OFFSET      0x0008 /* LCD status register */
#define STM32_LCD_CLR_OFFSET     0x000c /* LCD clear register */

#define STM32_LCD_RAM_OFFSET(n)  (0x0014 + ((n) << 3)) /* LCD display memory, COMn */
#define STM32_LCD_RAML_OFFSET(n) (0x0014 + ((n) << 3)) /* LCD display memory, COMn, S00-S31 */
#define STM32_LCD_RAMH_OFFSET(n) (0x0018 + ((n) << 3)) /* LCD display memory, COMn, S32-S39 */

#define STM32_LCD_RAM0L_OFFSET    0x0014 /* LCD display memory, COM0, S00-S31 */
#define STM32_LCD_RAM0H_OFFSET    0x0018 /* LCD display memory, COM0, S32-S39 */
#define STM32_LCD_RAM1L_OFFSET    0x001c /* LCD display memory, COM1, S00-S31 */
#define STM32_LCD_RAM1H_OFFSET    0x0020 /* LCD display memory, COM1, S32-S39 */
#define STM32_LCD_RAM2L_OFFSET    0x0024 /* LCD display memory, COM2, S00-S31 */
#define STM32_LCD_RAM2H_OFFSET    0x0028 /* LCD display memory, COM2, S32-S39 */
#define STM32_LCD_RAM3L_OFFSET    0x002c /* LCD display memory, COM3, S00-S31 */
#define STM32_LCD_RAM3H_OFFSET    0x0020 /* LCD display memory, COM3, S32-S39 */
#define STM32_LCD_RAM4L_OFFSET    0x0034 /* LCD display memory, COM4, S00-S31 */
#define STM32_LCD_RAM4H_OFFSET    0x0038 /* LCD display memory, COM4, S32-S39 */
#define STM32_LCD_RAM5L_OFFSET    0x003c /* LCD display memory, COM5, S00-S31 */
#define STM32_LCD_RAM5H_OFFSET    0x0040 /* LCD display memory, COM5, S32-S39 */
#define STM32_LCD_RAM6L_OFFSET    0x0044 /* LCD display memory, COM6, S00-S31 */
#define STM32_LCD_RAM6H_OFFSET    0x0048 /* LCD display memory, COM6, S32-S39 */
#define STM32_LCD_RAM7L_OFFSET    0x004c /* LCD display memory, COM7, S00-S31 */
#define STM32_LCD_RAM7H_OFFSET    0x0050 /* LCD display memory, COM7, S32-S39 */

/* Register Addresses *******************************************************/

#define STM32_LCD_CR              (STM32_LCD_BASE+STM32_LCD_CR_OFFSET)
#define STM32_LCD_FCR             (STM32_LCD_BASE+STM32_LCD_FCR_OFFSET)
#define STM32_LCD_SR              (STM32_LCD_BASE+STM32_LCD_SR_OFFSET)
#define STM32_LCD_CLR             (STM32_LCD_BASE+STM32_LCD_CLR_OFFSET)

#define STM32_LCD_RAM(n)          (STM32_LCD_BASE+STM32_LCD_RAM_OFFSET(n))
#define STM32_LCD_RAML(n)         (STM32_LCD_BASE+STM32_LCD_RAML_OFFSET(n))
#define STM32_LCD_RAMH(n)         (STM32_LCD_BASE+STM32_LCD_RAMH_OFFSET(n))

#define STM32_LCD_RAM0L           (STM32_LCD_BASE+STM32_LCD_RAM0L_OFFSET)
#define STM32_LCD_RAM0H           (STM32_LCD_BASE+STM32_LCD_RAM0H_OFFSET)
#define STM32_LCD_RAM1L           (STM32_LCD_BASE+STM32_LCD_RAM1L_OFFSET)
#define STM32_LCD_RAM1H           (STM32_LCD_BASE+STM32_LCD_RAM1H_OFFSET)
#define STM32_LCD_RAM2L           (STM32_LCD_BASE+STM32_LCD_RAM2L_OFFSET)
#define STM32_LCD_RAM2H           (STM32_LCD_BASE+STM32_LCD_RAM2H_OFFSET)
#define STM32_LCD_RAM3L           (STM32_LCD_BASE+STM32_LCD_RAM3L_OFFSET)
#define STM32_LCD_RAM3H           (STM32_LCD_BASE+STM32_LCD_RAM3H_OFFSET)
#define STM32_LCD_RAM4L           (STM32_LCD_BASE+STM32_LCD_RAM4L_OFFSET)
#define STM32_LCD_RAM4H           (STM32_LCD_BASE+STM32_LCD_RAM4H_OFFSET)
#define STM32_LCD_RAM5L           (STM32_LCD_BASE+STM32_LCD_RAM5L_OFFSET)
#define STM32_LCD_RAM5H           (STM32_LCD_BASE+STM32_LCD_RAM5H_OFFSET)
#define STM32_LCD_RAM6L           (STM32_LCD_BASE+STM32_LCD_RAM6L_OFFSET)
#define STM32_LCD_RAM6H           (STM32_LCD_BASE+STM32_LCD_RAM6H_OFFSET)
#define STM32_LCD_RAM7L           (STM32_LCD_BASE+STM32_LCD_RAM7L_OFFSET)
#define STM32_LCD_RAM7H           (STM32_LCD_BASE+STM32_LCD_RAM7H_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* LCD control register */

#define LCD_CR_LCDEN              (1 << 0)  /* Bit 0:  LCD controller enable */
#define LCD_CR_VSEL               (1 << 1)  /* Bit 1:  Voltage source selection */
#define LCD_CR_DUTY_SHIFT         (2)       /* Bits 2-4: Duty selection */
#define LCD_CR_DUTY_MASK          (7 << LCD_CR_DUTY_SHIFT)
#  define LCD_CR_DUTY_STATIC      (0 << LCD_CR_DUTY_SHIFT) /* 000: Static duty */
#  define LCD_CR_DUTY_1TO2        (1 << LCD_CR_DUTY_SHIFT) /* 001: 1/2 duty */
#  define LCD_CR_DUTY_1TO3        (2 << LCD_CR_DUTY_SHIFT) /* 010: 1/3 duty */
#  define LCD_CR_DUTY_1TO4        (3 << LCD_CR_DUTY_SHIFT) /* 011: 1/4 duty */
#  define LCD_CR_DUTY_1TO8        (4 << LCD_CR_DUTY_SHIFT) /* 100: 1/8 duty */

#define LCD_CR_BIAS_SHIFT         (5)       /* Bits 5-6: Bias selector */
#define LCD_CR_BIAS_MASK          (3 << LCD_CR_BIAS_SHIFT)
#  define LCD_CR_BIAS_1TO4        (0 << LCD_CR_BIAS_SHIFT) /* 00: Bias 1/4 */
#  define LCD_CR_BIAS_1TO2        (1 << LCD_CR_BIAS_SHIFT) /* 01: Bias 1/2 */
#  define LCD_CR_BIAS_1TO3        (2 << LCD_CR_BIAS_SHIFT) /* 10: Bias 1/3 */

#define LCD_CR_MUX_SEG            (1 << 7)  /* Bit 7:  Mux segment enable */
                                            /* Bits 8-31 Reserved */

/* LCD frame control register */

#define LCD_FCR_HD                (1 << 0)  /* Bit 0: High drive enable */
#define LCD_FCR_SOFIE             (1 << 1)  /* Bit 1: Start of frame interrupt enable */
                                            /* Bit 2 Reserved */
#define LCD_FCR_UDDIE             (1 << 3)  /* Bit 3: Update display done interrupt enable */
#define LCD_FCR_PON_SHIFT         (4)       /* Bits 4-6: Pulse ON duration */
#define LCD_FCR_PON_MASK          (7 << LCD_FCR_PON_SHIFT)
#  define LCD_FCR_PON(n)          ((n) << LCD_FCR_PON_SHIFT) /* n=0-7 */

#define LCD_FCR_DEAD_SHIFT        (7)       /* Bits 7-9: Dead time duration */
#define LCD_FCR_DEAD_MASK         (7 << LCD_FCR_DEAD_SHIFT)
#  define LCD_FCR_DEAD_NONE       (0 << LCD_FCR_DEAD_SHIFT)
#  define LCD_FCR_DEAD(n)         ((n) << LCD_FCR_DEAD_SHIFT) /* n=1..7 */

#define LCD_FCR_CC_SHIFT          (10)      /* Bits 10-12: Contrast control */
#define LCD_FCR_CC_MASK           (7 << LCD_FCR_CC_SHIFT)
#  define LCD_FCR_CC_VLCD(n)      ((n) << LCD_FCR_CC_SHIFT) /* VLCDn, n=0..7 */

#define LCD_FCR_BLINKF_SHIFT      (13)      /* Bits 13-15: Blink frequency selection */
#define LCD_FCR_BLINKF_MASK       (7 << LCD_FCR_BLINKF_SHIFT)
#  define LCD_FCR_BLINKF_DIV8     (0 << LCD_FCR_BLINKF_SHIFT) /* 000: fLCD/8 */
#  define LCD_FCR_BLINKF_DIV16    (1 << LCD_FCR_BLINKF_SHIFT) /* 001: fLCD/16 */
#  define LCD_FCR_BLINKF_DIV32    (2 << LCD_FCR_BLINKF_SHIFT) /* 010: fLCD/32 */
#  define LCD_FCR_BLINKF_DIV64    (3 << LCD_FCR_BLINKF_SHIFT) /* 011: fLCD/64 */
#  define LCD_FCR_BLINKF_DIV128   (4 << LCD_FCR_BLINKF_SHIFT) /* 100: fLCD/128 */
#  define LCD_FCR_BLINKF_DIV256   (5 << LCD_FCR_BLINKF_SHIFT) /* 101: fLCD/256 */
#  define LCD_FCR_BLINKF_DIV512   (6 << LCD_FCR_BLINKF_SHIFT) /* 110: fLCD/512 */
#  define LCD_FCR_BLINKF_DIV1024  (7 << LCD_FCR_BLINKF_SHIFT) /* 111: fLCD/1024 */

#define LCD_FCR_BLINK_SHIFT       (16)      /* Bits 16-17: Blink mode selection */
#define LCD_FCR_BLINK_MASK        (3 << LCD_FCR_BLINK_SHIFT)
#  define LCD_FCR_BLINK_DISABLE   (0 << LCD_FCR_BLINK_SHIFT) /* 00: Blink disabled */
#  define LCD_FCR_BLINK_S0C0      (1 << LCD_FCR_BLINK_SHIFT) /* 01: Blink enabled on SEG[0], COM[0] (1 pixel) */
#  define LCD_FCR_BLINK_S0CALL    (2 << LCD_FCR_BLINK_SHIFT) /* 10: Blink enabled on SEG[0], all COMs */
#  define LCD_FCR_BLINK_SALLCALL  (3 << LCD_FCR_BLINK_SHIFT) /* 11: Blink enabled on all SEGs and all COMs */

#define LCD_FCR_DIV_SHIFT         (18)      /* Bits 18-21: DIV clock divider */
#define LCD_FCR_DIV_MASK          (15 << LCD_FCR_DIV_SHIFT)
#  define LCD_FCR_DIV(n)          (((n)-16) << LCD_FCR_DIV_SHIFT) /* n=16-31 */

#define LCD_FCR_PS_SHIFT          (22)      /* Bits 22-25: PS 16-bit prescaler */
#define LCD_FCR_PS_MASK           (15 << LCD_FCR_PS_SHIFT)
#  define LCD_FCR_PS_DIV1         (0 << LCD_FCR_PS_SHIFT)  /* 0000: ck_ps = LCDCLK */
#  define LCD_FCR_PS_DIV2         (1 << LCD_FCR_PS_SHIFT)  /* 0001: ck_ps = LCDCLK/2 */
#  define LCD_FCR_PS_DIV4         (2 << LCD_FCR_PS_SHIFT)  /* 0011: ck_ps = LCDCLK/4 */
#  define LCD_FCR_PS_DIV8         (3 << LCD_FCR_PS_SHIFT)  /* 0011: ck_ps = LCDCLK/8 */
#  define LCD_FCR_PS_DIV16        (4 << LCD_FCR_PS_SHIFT)  /* 0011: ck_ps = LCDCLK/16 */
#  define LCD_FCR_PS_DIV32        (5 << LCD_FCR_PS_SHIFT)  /* 0011: ck_ps = LCDCLK/32 */
#  define LCD_FCR_PS_DIV64        (6 << LCD_FCR_PS_SHIFT)  /* 0011: ck_ps = LCDCLK/64 */
#  define LCD_FCR_PS_DIV128       (7 << LCD_FCR_PS_SHIFT)  /* 0011: ck_ps = LCDCLK/128 */
#  define LCD_FCR_PS_DIV256       (8 << LCD_FCR_PS_SHIFT)  /* 0011: ck_ps = LCDCLK/256 */
#  define LCD_FCR_PS_DIV512       (9 << LCD_FCR_PS_SHIFT)  /* 0011: ck_ps = LCDCLK/512 */
#  define LCD_FCR_PS_DIV1024      (10 << LCD_FCR_PS_SHIFT) /* 0011: ck_ps = LCDCLK/1024 */
#  define LCD_FCR_PS_DIV2048      (11 << LCD_FCR_PS_SHIFT) /* 0011: ck_ps = LCDCLK/2048 */
#  define LCD_FCR_PS_DIV4096      (12 << LCD_FCR_PS_SHIFT) /* 0011: ck_ps = LCDCLK/4096 */
#  define LCD_FCR_PS_DIV8192      (13 << LCD_FCR_PS_SHIFT) /* 0011: ck_ps = LCDCLK/8192 */
#  define LCD_FCR_PS_DIV16384     (14 << LCD_FCR_PS_SHIFT) /* 0011: ck_ps = LCDCLK/16384 */
#  define LCD_FCR_PS_DIV32768     (15 << LCD_FCR_PS_SHIFT) /* 0011: ck_ps = LCDCLK/32768 */

                                            /* Bits 26-31 Reserved */

/* LCD status register */

#define LCD_SR_ENS                (1 << 0)  /* Bit 0: LCD enabled status */
#define LCD_SR_SOF                (1 << 1)  /* Bit 1: Start of frame flag */
#define LCD_SR_UDR                (1 << 2)  /* Bit 2: Update display request */
#define LCD_SR_UDD                (1 << 3)  /* Bit 3: Update Display Done */
#define LCD_SR_RDY                (1 << 4)  /* Bit 4: Ready flag */
#define LCD_SR_FCRSF              (1 << 5)  /* Bit 5: LCD Frame Control Register Synchronization flag */
                                            /* Bits 6-31 Reserved */

/* LCD clear register */

                                            /* Bit 0 Reserved */
#define LCD_CLR_SOFC              (1 << 1)  /* Bit 1: Start of frame flag clear */
                                            /* Bit 2 Reserved */
#define LCD_CLR_UDDC              (1 << 2)  /* Bit 3: Update display done clear */
                                            /* Bits 31:2-31 Reserved */

/* LCD display memory, COMn, S00-S31 */

#define LCD_RAML_S(n)             (1 << (n))

/* LCD display memory, COMn, S32-S39 */

#define LCD_RAMH_S(n)             (1 << ((n)-32))

#endif /* STM32_NLCD */
#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_LCD_H */
