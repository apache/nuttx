/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_osc.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OSC_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MX_OSCCON_OFFSET    0x0000 /* Oscillator control register offset */
#define PIC32MX_OSCTUN_OFFSET    0x0010 /* FRC tuning register offset */

/* Register Addresses *******************************************************/

#define PIC32MX_OSCCON           (PIC32MX_OSC_K1BASE+PIC32MX_OSCCON_OFFSET)
#define PIC32MX_OSCTUN           (PIC32MX_OSC_K1BASE+PIC32MX_OSCTUN_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Oscillator control register offset */

#define OSCCON_OSWEN            (1 << 0)  /* Bit 0: Oscillator switch enable */
#define OSCCON_SOSCEN           (1 << 1)  /* Bit 1: 32.768kHz secondary oscillator enable */
#define OSCCON_UFRCEN           (1 << 2)  /* Bit 2: USB FRC clock enable */
#define OSCCON_CF               (1 << 3)  /* Bit 3: Clock fail detect */
#define OSCCON_SLPEN            (1 << 4)  /* Bit 4: Sleep mode enable */
#define OSCCON_SLOCK            (1 << 5)  /* Bit 5: PLL lock status */
#define OSCCON_ULOCK            (1 << 6)  /* Bit 6: USB PLL lock status */
#define OSCCON_CLKLOCK          (1 << 7)  /* Bit 7: Clock selection lock enable */
#define OSCCON_NOSC_SHIFT       (8)       /* Bits 8-10: New oscillator selection */
#define OSCCON_NOSC_MASK        (7 << OSCCON_NOSC_SHIFT)
#  define OSCCON_NOSC_FRC       (0 << OSCCON_NOSC_SHIFT) /* FRC oscillator */
#  define OSCCON_NOSC_FRCPLL    (1 << OSCCON_NOSC_SHIFT) /* FRC w/PLL postscaler */
#  define OSCCON_NOSC_POSC      (2 << OSCCON_NOSC_SHIFT) /* Primary oscillator */
#  define OSCCON_NOSC_POSCPLL   (3 << OSCCON_NOSC_SHIFT) /* Primary oscillator with PLL */
#  define OSCCON_NOSC_SOSC      (4 << OSCCON_NOSC_SHIFT) /* Secondary oscillator */
#  define OSCCON_NOSC_LPRC      (5 << OSCCON_NOSC_SHIFT) /* Low power RC oscillator */
#  define OSCCON_NOSC_FRCDIV16  (6 << OSCCON_NOSC_SHIFT) /* FRC divided by 16 */
#  define OSCCON_NOSC_FRCDIV    (7 << OSCCON_NOSC_SHIFT) /* FRC dived by FRCDIV */

#define OSCCON_COSC_SHIFT       (12)      /* Bits 12-14: Current oscillator selection */
#define OSCCON_COSC_MASK        (7 << OSCCON_COSC_SHIFT)
#  define OSCCON_COSC_FRC       (0 << OSCCON_COSC_SHIFT) /* FRC oscillator */
#  define OSCCON_COSC_FRCPLL    (1 << OSCCON_COSC_SHIFT) /* FRC w/PLL postscaler */
#  define OSCCON_COSC_POSC      (2 << OSCCON_COSC_SHIFT) /* Primary oscillator */
#  define OSCCON_COSC_POSCPLL   (3 << OSCCON_COSC_SHIFT) /* Primary oscillator with PLL */
#  define OSCCON_COSC_SOSC      (4 << OSCCON_COSC_SHIFT) /* Secondary oscillator */
#  define OSCCON_COSC_LPRC      (5 << OSCCON_COSC_SHIFT) /* Low power RC oscillator */
#  define OSCCON_COSC_FRCDIV16  (6 << OSCCON_COSC_SHIFT) /* FRC divided by 16 */
#  define OSCCON_COSC_FRCDIV    (7 << OSCCON_COSC_SHIFT) /* FRC dived by FRCDIV */

#define OSCCON_PLLMULT_SHIFT    (16)      /* Bits 16-18: PLL multiplier */
#define OSCCON_PLLMULT_MASK     (7 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL15  (0 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL16  (1 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL17  (2 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL18  (3 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL19  (4 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL20  (5 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL21  (6 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL24  (7 << OSCCON_PLLMULT_SHIFT)
#define OSCCON_PBDIV_SHIFT      (19)      /* Bits 19-20: PBVLK divisor */
#define OSCCON_PBDIV_SMASK      (3 << OSCCON_PBDIV_SHIFT)
#  define OSCCON_PBDIV_DIV1     (0 << OSCCON_PBDIV_SHIFT)
#  define OSCCON_PBDIV_DIV2     (1 << OSCCON_PBDIV_SHIFT)
#  define OSCCON_PBDIV_DIV4     (2 << OSCCON_PBDIV_SHIFT)
#  define OSCCON_PBDIV_DIV8     (3 << OSCCON_PBDIV_SHIFT)
#define OSCCON_SOSCRDY          (1 << 22) /* Bit 22: Secondary oscillator ready */
#define OSCCON_FRCDIV_SHIFT     (24)      /* Bits 24-26: FRC oscillator divider */
#define OSCCON_FRCDIV_MASK      (7 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV1    (0 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV2    (1 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV4    (2 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV8    (3 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV16   (4 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV32   (5 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV64   (6 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV256  (7 << OSCCON_FRCDIV_SHIFT)
#define OSCCON_PLL0DIV_SHIFT    (27)      /* Bits 27-29: Output divider for PLL */
#define OSCCON_PLL0DIV_MASK     (7 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV1   (0 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV2   (1 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV4   (2 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV8   (3 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV16  (4 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV32  (5 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV64  (6 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV256 (7 << OSCCON_PLL0DIV_SHIFT)

/* FRC tuning register offset (6-bit, signed twos complement) */

#define OSCTUN_SHIFT         (0)       /* Bits 0-5: FRC tuning bits */
#define OSCTUN_MASK          (0x3f << OSCTUN_SHIFT)
#  define OSCTUN_MIN         (0x20 << OSCTUN_SHIFT)
#  define OSCTUN_CENTER      (0x00 << OSCTUN_SHIFT)
#  define OSCTUN_MAX         (0x1f << OSCTUN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OSC_H */
