/************************************************************************************
 * arch/arm/src/imx6/imx_gpt.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual," Document Number
 *   IMX6DQRM, Rev. 3, 07/2015, FreeScale.
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

#ifndef __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPT_H
#define __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <chip/imx_memorymap.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPT Register Offsets ************************************************************/

#define IMX_GPT_CR_OFFSET          0x0000 /* GPT Control Register */
#define IMX_GPT_PR_OFFSET          0x0004 /* GPT Prescaler Register */
#define IMX_GPT_SR_OFFSET          0x0008 /* GPT Status Register */
#define IMX_GPT_IR_OFFSET          0x000c /* GPT Interrupt Register */
#define IMX_GPT_OCR1_OFFSET        0x0010 /* GPT Output Compare Register 1 */
#define IMX_GPT_OCR2_OFFSET        0x0014 /* GPT Output Compare Register 2 */
#define IMX_GPT_OCR3_OFFSET        0x0018 /* GPT Output Compare Register 3 */
#define IMX_GPT_ICR1_OFFSET        0x001c /* GPT Input Capture Register 1 */
#define IMX_GPT_ICR2_OFFSET        0x0020 /* GPT Input Capture Register 2 */
#define IMX_GPT_CNT_OFFSET         0x0024 /* GPT Counter Register */

/* GPT Register Addresses **********************************************************/

#define IMX_GPT_CR                 (IMX_GPT_VBASE+IMX_GPT_CR_OFFSET)
#define IMX_GPT_PR                 (IMX_GPT_VBASE+IMX_GPT_PR_OFFSET)
#define IMX_GPT_SR                 (IMX_GPT_VBASE+IMX_GPT_SR_OFFSET)
#define IMX_GPT_IR                 (IMX_GPT_VBASE+IMX_GPT_IR_OFFSET)
#define IMX_GPT_OCR1               (IMX_GPT_VBASE+IMX_GPT_OCR1_OFFSET)
#define IMX_GPT_OCR2               (IMX_GPT_VBASE+IMX_GPT_OCR2_OFFSET)
#define IMX_GPT_OCR3               (IMX_GPT_VBASE+IMX_GPT_OCR3_OFFSET)
#define IMX_GPT_ICR1               (IMX_GPT_VBASE+IMX_GPT_ICR1_OFFSET)
#define IMX_GPT_ICR2               (IMX_GPT_VBASE+IMX_GPT_ICR2_OFFSET)
#define IMX_GPT_CNT                (IMX_GPT_VBASE+IMX_GPT_CNT_OFFSET)

/* GPT Register Bit Definitions ****************************************************/

/* GPT Control Register */

#define GPT_CR_EN                  (1 << 0)  /* Bit 0: GPT Enable */
#define GPT_CR_ENMOD               (1 << 1)  /* Bit 1: GPT Enable mode */
#define GPT_CR_DBGEN               (1 << 2)  /* Bit 2: GPT debug mode enable */
#define GPT_CR_WAITEN              (1 << 3)  /* Bit 3: GPT Wait Mode enable */
#define GPT_CR_DOZEEN              (1 << 4)  /* Bit 4: GPT Doze Mode Enable */
#define GPT_CR_STOPEN              (1 << 5)  /* Bit 5:  GPT Stop Mode enable */
#define GPT_CR_CLKSRC_SHIFT        (6)       /* Bits 6-8: Clock source select */
#define GPT_CR_CLKSRC_MASK         (7 << GPT_CR_CLKSRC_SHIFT)
#  define GPT_CR_CLKSRC_NONE       (0 << GPT_CR_CLKSRC_SHIFT) /* No clock */
#  define GPT_CR_CLKSRC_PERIPHCLK  (1 << GPT_CR_CLKSRC_SHIFT) /* Peripheral Clock */
#  define GPT_CR_CLKSRC_HFREF      (2 << GPT_CR_CLKSRC_SHIFT) /* High Frequency Reference Clock */
#  define GPT_CR_CLKSRC_CLKIN      (3 << GPT_CR_CLKSRC_SHIFT) /* External Clock (CLKIN) */
#  define GPT_CR_CLKSRC_LFREF      (4 << GPT_CR_CLKSRC_SHIFT) /* Low Frequency Reference Clock */
#  define GPT_CR_CLKSRC_OSCDIV8    (5 << GPT_CR_CLKSRC_SHIFT) /* Crystal oscillator divided by 8 as Reference Clock */
#  define GPT_CR_CLKSRC_OSC        (7 << GPT_CR_CLKSRC_SHIFT) /* Crystal oscillator as Reference Clock */
#define GPT_CR_FFR                 (1 << 9)  /* Bit 9:  Free-Run or Restart mode */
#define GPT_CR_SWR                 (1 << 15) /* Bit 15: Software reset */
#define GPT_CR_IM1_SHIFT           (16)      /* Bits 16-17: Input Capture Channel 1 operating mode */
#define GPT_CR_IM1_MASK            (3 << GPT_CR_IM1_SHIFT)
#  define GPT_CR_IM1_DISABLED      (0 << GPT_CR_IM1_SHIFT) /* Capture disabled */
#  define GPT_CR_IM1_RISING        (1 << GPT_CR_IM1_SHIFT) /* Capture on rising edge only */
#  define GPT_CR_IM1_FALLING       (2 << GPT_CR_IM1_SHIFT) /* Capture on falling edge only */
#  define GPT_CR_IM1_BOTH          (3 << GPT_CR_IM1_SHIFT) /* Capture on both edges */
#define GPT_CR_IM2_SHIFT           (18)      /* Bits 18-19: Input Capture Channel 2 operating mode */
#define GPT_CR_IM2_MASK            (3 << GPT_CR_IM2_SHIFT)
#  define GPT_CR_IM2_DISABLED      (0 << GPT_CR_IM2_SHIFT) /* Capture disabled */
#  define GPT_CR_IM2_RISING        (1 << GPT_CR_IM2_SHIFT) /* Capture on rising edge only */
#  define GPT_CR_IM2_FALLING       (2 << GPT_CR_IM2_SHIFT) /* Capture on falling edge only */
#  define GPT_CR_IM2_BOTH          (3 << GPT_CR_IM2_SHIFT) /* Capture on both edges */
#define GPT_CR_OM1_SHIFT           (22)      /* Bits 20-22: Output Compare Channel 1 operating mode */
#define GPT_CR_OM1_MASK            (7 << GPT_CR_OM1_SHIFT)
#  define GPT_CR_OM1_DISCON        (0 << GPT_CR_OM1_SHIFT) /* Output disconnected */
#  define GPT_CR_OM1_TOGGLE        (1 << GPT_CR_OM1_SHIFT) /* Toggle output pin */
#  define GPT_CR_OM1_CLEAR         (2 << GPT_CR_OM1_SHIFT) /* Clear output pin */
#  define GPT_CR_OM1_SET           (3 << GPT_CR_OM1_SHIFT) /* Set output pin */
#  define GPT_CR_OM1_PULSE         (4 << GPT_CR_OM1_SHIFT) /* Generate an active low pulse */
#define GPT_CR_OM2_SHIFT           (23)      /* Bits 23-25: Output Compare Channel 2 operating mode */
#define GPT_CR_OM2_MASK            (7 << GPT_CR_OM2_SHIFT)
#  define GPT_CR_OM2_DISCON        (0 << GPT_CR_OM2_SHIFT) /* Output disconnected */
#  define GPT_CR_OM2_TOGGLE        (1 << GPT_CR_OM2_SHIFT) /* Toggle output pin */
#  define GPT_CR_OM2_CLEAR         (2 << GPT_CR_OM2_SHIFT) /* Clear output pin */
#  define GPT_CR_OM2_SET           (3 << GPT_CR_OM2_SHIFT) /* Set output pin */
#  define GPT_CR_OM2_PULSE         (4 << GPT_CR_OM2_SHIFT) /* Generate an active low pulse */
#define GPT_CR_OM3_SHIFT           (26)      /* Bits 26-28: Output Compare Channel 3 operating mode */
#define GPT_CR_OM3_MASK            (7 << GPT_CR_OM3_SHIFT)
#  define GPT_CR_OM3_DISCON        (0 << GPT_CR_OM3_SHIFT) /* Output disconnected */
#  define GPT_CR_OM3_TOGGLE        (1 << GPT_CR_OM3_SHIFT) /* Toggle output pin */
#  define GPT_CR_OM3_CLEAR         (2 << GPT_CR_OM3_SHIFT) /* Clear output pin */
#  define GPT_CR_OM3_SET           (3 << GPT_CR_OM3_SHIFT) /* Set output pin */
#  define GPT_CR_OM3_PULSE         (4 << GPT_CR_OM3_SHIFT) /* Generate an active low pulse */
#define GPT_CR_FO1                 (1 << 29) /* FO1 Force Output Compare Channel 1 */
#define GPT_CR_FO2                 (1 << 30) /* FO2 Force Output Compare Channel 2 */
#define GPT_CR_FO3                 (1 << 31) /* Force Output Compare Channel 3 */

/* GPT Prescaler Register */

#define GPT_PR_MASK                0xfff     /* Bits 0-11: Prescaler value - 1 */

/* GPT Status Register and GPT Interrupt Register */

#define GPT_INT_OF1                (1 << 0)  /* Bit 0: OF3 Output Compare 1 Flag */
#define GPT_INT_OF2                (1 << 1)  /* Bit 1: OF3 Output Compare 2 Flag */
#define GPT_INT_OF3                (1 << 2)  /* Bit 2: OF3 Output Compare 3 Flag */
#define GPT_INT_IF1                (1 << 3)  /* Bit 3: IF2 Input capture 1 Flag */
#define GPT_INT_IF2                (1 << 4)  /* Bit 4: IF2 Input capture 2 Flag */
#define GPT_INT_ROV                (1 << 5)  /* Bit 5: Rollover flag */

#define GPT_INT_ALL                0x0000003f

/* GPT Output Compare Register 1,2,3 -- 32-bit compare registers */
/* GPT Input Capture Register 1,2 -- 32-bit capture registers */
/* GPT Counter Register -- 32-bit counter */

#endif /* __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPT_H */
