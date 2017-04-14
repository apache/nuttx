/************************************************************************************
 * arch/arm/src/stm32f0/chip/stm32f0_comp.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_STM32F0_CHIP_STM32F0_COMP_H
#define __ARCH_ARM_SRC_STM32F0_CHIP_STM32F0_COMP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32F0_COMP_CSR_OFFSET       0x001c      /* COMP1/COMP2 Control register */

/* Register Addresses ***************************************************************/

#define STM32F0_COMP_CSR              (STM32F0_COMP_BASE+STM32F0_COMP_CSR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* COMP control and status register */

#define COMP_CSR_COMP1EN              (1 << 0)    /* Bit 0: Comparator 1 enable */
#define COMP_CSR_COMP1SW1             (1 << 1)    /* Bit 1: Comparator 1 non inverting input DAC switch */
#define COMP_CSR_COMP1MODE_SHIFT      (2)         /* Bits 2-3: Compator 1 mode */
#define COMP_CSR_COMP1MODE_MASK       (3 << COMP_CSR_COMP1MODE_SHIFT)
#  define COMP_CSR_COMP1MODE_HIGH     (0 << COMP_CSR_COMP1MODE_SHIFT) /* 00: High Speed / full power */
#  define COMP_CSR_COMP1MODE_MEDIUM   (1 << COMP_CSR_COMP1MODE_SHIFT) /* 01: Medium Speed / medium power */
#  define COMP_CSR_COMP1MODE_LOW      (2 << COMP_CSR_COMP1MODE_SHIFT) /* 10: Low Speed / low-power */
#  define COMP_CSR_COMP1MODE_VLOW     (3 << COMP_CSR_COMP1MODE_SHIFT) /* 11: Very-low Speed / ultra-low power */
#define COMP_CSR_COMP1INSEL_SHIFT     (4)         /* Bits 4-6: Comparator 1 inverting input selection */
#define COMP_CSR_COMP1INSEL_MASK      (7 << COMP_CSR_COMP1INSEL_SHIFT)
#  define COMP_CSR_COMP1INSEL_1P4VREF (0 << COMP_CSR_COMP1INSEL_SHIFT)  /* 000: 1/4 of Vrefint */
#  define COMP_CSR_COMP1INSEL_1P2VREF (1 << COMP_CSR_COMP1INSEL_SHIFT)  /* 001: 1/2 of Vrefint */
#  define COMP_CSR_COMP1INSEL_3P4VREF (2 << COMP_CSR_COMP1INSEL_SHIFT)  /* 010: 3/4 of Vrefint */
#  define COMP_CSR_COMP1INSEL_VREF    (3 << COMP_CSR_COMP1INSEL_SHIFT)  /* 011: Vrefint */
#  define COMP_CSR_COMP1INSEL_INM4    (4 << COMP_CSR_COMP1INSEL_SHIFT)  /* 100: COMP1_INM4 (PA4 DAC_OUT1 if enabled) */
#  define COMP_CSR_COMP1INSEL_INM5    (5 << COMP_CSR_COMP1INSEL_SHIFT)  /* 101: COMP1_INM5 (PA5 DAC_OUT2 if present and enabled) */
#  define COMP_CSR_COMP1INSEL_INM6    (6 << COMP_CSR_COMP1INSEL_SHIFT)  /* 110: COMP1_INM6 (PA0) */
#define COMP_CSR_COMP1OUTSEL_SHIFT    (8)         /* Bits 8-10: Comparator 1 output selection*/
#define COMP_CSR_COMP1OUTSEL_MASK     (7 << COMP_CSR_COMP1OUTSEL_MASK)
#  define COMP_CSR_COMP1OUTSEL_NOSEL  (0 << COMP_CSR_COMP1OUTSEL_MASK)  /* 000: no selection */
#  define COMP_CSR_COMP1OUTSEL_T1BRK  (1 << COMP_CSR_COMP1OUTSEL_MASK)  /* 001: Timer 1 break input */
#  define COMP_CSR_COMP1OUTSEL_T1ICAP (2 << COMP_CSR_COMP1OUTSEL_MASK)  /* 010: Timer 1 Input capture 1 */
#  define COMP_CSR_COMP1OUTSEL_T1OCRC (3 << COMP_CSR_COMP1OUTSEL_MASK)  /* 011: Timer 1 OCrefclear input */
#  define COMP_CSR_COMP1OUTSEL_T2ICAP (4 << COMP_CSR_COMP1OUTSEL_MASK)  /* 100: Timer 2 input capture 4 */
#  define COMP_CSR_COMP1OUTSEL_T2OCRC (5 << COMP_CSR_COMP1OUTSEL_MASK)  /* 101: Timer 2 OCrefclear input */
#  define COMP_CSR_COMP1OUTSEL_T3ICAP (6 << COMP_CSR_COMP1OUTSEL_MASK)  /* 110: Timer 3 input capture 1 */
#  define COMP_CSR_COMP1OUTSEL_T3OCRC (7 << COMP_CSR_COMP1OUTSEL_MASK)  /* 111: Timer 3 OCrefclear input */
#define COMP_CSR_COMP1POL             (1 << 11)   /* Bit 11: Comparator 1 output polarity */
#define COMP_CSR_COMP1HYST_SHIFT      (12)        /* Bits 12-13: Comparator 1 hysteresis */
#define COMP_CSR_COMP1HYST_MASK       (3 << COMP_CSR_COMP1HYST_SHIFT)
#  define COMP_CSR_COMP1HYST_NOHYST   (0 << COMP_CSR_COMP1HYST_MASK)    /* 00: No hysteresis */
#  define COMP_CSR_COMP1HYST_LOWHYST  (1 << COMP_CSR_COMP1HYST_MASK)    /* 01: Low hysteresis */
#  define COMP_CSR_COMP1HYST_MDHYST   (2 << COMP_CSR_COMP1HYST_MASK)    /* 10: Medium hysteresis */
#  define COMP_CSR_COMP1HYST_HIHYST   (3 << COMP_CSR_COMP1HYST_MASK)    /* 11: Low hysteresis */
#define COMP_CSR_COMP1OUT             (1 << 14)   /* Bit 14: Comparator 1 output */
#define COMP_CSR_COMP1LOCK            (1 << 15)   /* Bit 15: Comparator 1 lock */

#define COMP_CSR_COMP2EN              (1 << 16)   /* Bit 16: Comparator 2 enable */
#define COMP_CSR_COMP2MODE_SHIFT      (18)        /* Bits 18-19: Compator 2 mode */
#define COMP_CSR_COMP2MODE_MASK       (3 << COMP_CSR_COMP2MODE_SHIFT)
#  define COMP_CSR_COMP2MODE_HIGH     (0 << COMP_CSR_COMP2MODE_SHIFT) /* 00: High Speed / full power */
#  define COMP_CSR_COMP2MODE_MEDIUM   (1 << COMP_CSR_COMP2MODE_SHIFT) /* 01: Medium Speed / medium power */
#  define COMP_CSR_COMP2MODE_LOW      (2 << COMP_CSR_COMP2MODE_SHIFT) /* 10: Low Speed / low-power */
#  define COMP_CSR_COMP2MODE_VLOW     (3 << COMP_CSR_COMP2MODE_SHIFT) /* 11: Very-low Speed / ultra-low power */
#define COMP_CSR_COMP2INSEL_SHIFT     (20)         /* Bits 20-22: Comparator 2 inverting input selection */
#define COMP_CSR_COMP2INSEL_MASK      (7 << COMP_CSR_COMP2INSEL_SHIFT)
#  define COMP_CSR_COMP2INSEL_1P4VREF (0 << COMP_CSR_COMP2INSEL_SHIFT)  /* 000: 1/4 of Vrefint */
#  define COMP_CSR_COMP2INSEL_1P2VREF (1 << COMP_CSR_COMP2INSEL_SHIFT)  /* 001: 1/2 of Vrefint */
#  define COMP_CSR_COMP2INSEL_3P4VREF (2 << COMP_CSR_COMP2INSEL_SHIFT)  /* 010: 3/4 of Vrefint */
#  define COMP_CSR_COMP2INSEL_VREF    (3 << COMP_CSR_COMP2INSEL_SHIFT)  /* 011: Vrefint */
#  define COMP_CSR_COMP2INSEL_INM4    (4 << COMP_CSR_COMP2INSEL_SHIFT)  /* 100: COMP1_INM4 (PA4 DAC_OUT1 if enabled) */
#  define COMP_CSR_COMP2INSEL_INM5    (5 << COMP_CSR_COMP2INSEL_SHIFT)  /* 101: COMP1_INM5 (PA5 DAC_OUT2 if present and enabled) */
#  define COMP_CSR_COMP2INSEL_INM6    (6 << COMP_CSR_COMP2INSEL_SHIFT)  /* 110: COMP1_INM6 (PA2) */
#define COMP_CSR_WNDWEN               (1 << 23)   /* Bit 23: Window mode enable */
#define COMP_CSR_COMP2OUTSEL_SHIFT    (24)         /* Bits 24-26: Comparator 1 output selection*/
#define COMP_CSR_COMP2OUTSEL_MASK     (7 << COMP_CSR_COMP2OUTSEL_MASK)
#  define COMP_CSR_COMP2OUTSEL_NOSEL  (0 << COMP_CSR_COMP2OUTSEL_MASK)  /* 000: no selection */
#  define COMP_CSR_COMP2OUTSEL_T1BRK  (1 << COMP_CSR_COMP2OUTSEL_MASK)  /* 001: Timer 1 break input */
#  define COMP_CSR_COMP2OUTSEL_T1ICAP (2 << COMP_CSR_COMP2OUTSEL_MASK)  /* 010: Timer 1 Input capture 1 */
#  define COMP_CSR_COMP2OUTSEL_T1OCRC (3 << COMP_CSR_COMP2OUTSEL_MASK)  /* 011: Timer 1 OCrefclear input */
#  define COMP_CSR_COMP2OUTSEL_T2ICAP (4 << COMP_CSR_COMP2OUTSEL_MASK)  /* 100: Timer 2 input capture 4 */
#  define COMP_CSR_COMP2OUTSEL_T2OCRC (5 << COMP_CSR_COMP2OUTSEL_MASK)  /* 101: Timer 2 OCrefclear input */
#  define COMP_CSR_COMP2OUTSEL_T3ICAP (6 << COMP_CSR_COMP2OUTSEL_MASK)  /* 110: Timer 3 input capture 1 */
#  define COMP_CSR_COMP2OUTSEL_T3OCRC (7 << COMP_CSR_COMP2OUTSEL_MASK)  /* 111: Timer 3 OCrefclear input */
#define COMP_CSR_COMP2POL             (1 << 27)   /* Bit 27: Comparator 2 output polarity */
#define COMP_CSR_COMP2HYST_SHIFT      (12)        /* Bits 12-13: Comparator 1 hysteresis */
#define COMP_CSR_COMP2HYST_MASK       (3 << COMP_CSR_COMP2HYST_SHIFT)
#  define COMP_CSR_COMP2HYST_NOHYST   (0 << COMP_CSR_COMP2HYST_MASK)    /* 00: No hysteresis */
#  define COMP_CSR_COMP2HYST_LOWHYST  (1 << COMP_CSR_COMP2HYST_MASK)    /* 01: Low hysteresis */
#  define COMP_CSR_COMP2HYST_MDHYST   (2 << COMP_CSR_COMP2HYST_MASK)    /* 10: Medium hysteresis */
#  define COMP_CSR_COMP2HYST_HIHYST   (3 << COMP_CSR_COMP2HYST_MASK)    /* 11: Low hysteresis */
#define COMP_CSR_COMP2OUT             (1 << 14)   /* Bit 14: Comparator 1 output */
#define COMP_CSR_COMP2LOCK            (1 << 15)   /* Bit 15: Comparator 1 lock */

#endif /* __ARCH_ARM_SRC_STM32F0_CHIP_STM32F0_COMP_H */
