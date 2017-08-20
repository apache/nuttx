/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32f33xxx_comp.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_COMP_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_COMP_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define STM32_COMP2_CSR_OFFSET      0x0020      /* COMP2 Control register */
#define STM32_COMP4_CSR_OFFSET      0x0028      /* COMP4 Control register */
#define STM32_COMP6_CSR_OFFSET      0x0030      /* COMP6 Control register */

/* Register Addresses *******************************************************************************/

#define STM32_COMP2_CSR             (STM32_COMP_BASE+STM32_COMP2_CSR_OFFSET)
#define STM32_COMP4_CSR             (STM32_COMP_BASE+STM32_COMP4_CSR_OFFSET)
#define STM32_COMP6_CSR             (STM32_COMP_BASE+STM32_COMP6_CSR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* COMP control and status register */

#define COMP_CSR_COMPEN             (1 << 0)    /* Bit 0: Comparator enable */
                                                /* Bits 1-3: Reserved */
#define COMP_CSR_INMSEL_SHIFT       (4)         /* Bits 4-6: Comparator inverting input selection */
#define COMP_CSR_INMSEL_MASK        (15 << COMP_CSR_INMSEL_SHIFT)
#  define COMP_CSR_INMSEL_1P4VREF   (0 << COMP_CSR_INMSEL_SHIFT)  /* 0000: 1/4 of Vrefint */
#  define COMP_CSR_INMSEL_1P2VREF   (1 << COMP_CSR_INMSEL_SHIFT)  /* 0001: 1/2 of Vrefint */
#  define COMP_CSR_INMSEL_3P4VREF   (2 << COMP_CSR_INMSEL_SHIFT)  /* 0010: 3/4 of Vrefint */
#  define COMP_CSR_INMSEL_VREF      (3 << COMP_CSR_INMSEL_SHIFT)  /* 0011: Vrefint */
#  define COMP_CSR_INMSEL_PA4       (4 << COMP_CSR_INMSEL_SHIFT)  /* 0100: PA4 or */
#  define COMP_CSR_INMSEL_DAC1CH1   (4 << COMP_CSR_INMSEL_SHIFT)  /* 0100: DAC1_CH1 output if enabled */
#  define COMP_CSR_INMSEL_DAC1CH2   (5 << COMP_CSR_INMSEL_SHIFT)  /* 0101: DAC1_CH2 output */
#  define COMP_CSR_INMSEL_PA2       (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: PA2 (COMP2 only) */
#  define COMP_CSR_INMSEL_PB2       (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PB2 (COMP4 only) */
#  define COMP_CSR_INMSEL_PB15      (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PB15 (COMP6 only) */
                                                                  /* 1000: DAC2_CH1 output, look at bit 22 */
                                                /* Bits 7-9: Reserved */
#define COMP_CSR_OUTSEL_SHIFT       (4)         /* Bits 10-13: Comparator output selection */
#define COMP_CSR_OUTSEL_MASK        (15 << COMP_CSR_INMSEL_SHIFT)
#  define COMP_CSR_OUTSEL_NOSEL     (0 << COMP_CSR_INMSEL_SHIFT)  /* 0000: No selection */
#  define COMP_CSR_OUTSEL_BRKACTH   (1 << COMP_CSR_INMSEL_SHIFT)  /* 0001: Timer 1 break input */
#  define COMP_CSR_OUTSEL_BRK2      (2 << COMP_CSR_INMSEL_SHIFT)  /* 0010: Timer 1 break input 2 */
                                                                  /* 0011: Reserved */
                                                                  /* 0100: Reserved */
#  define COMP_CSR_OUTSEL_BRK2_     (5 << COMP_CSR_INMSEL_SHIFT)  /* 0101: Timer 1 break input2 */
#  define COMP_CSR_OUTSEL_T1OCC     (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: Timer 1 OCREF_CLR input (COMP2 only) */
#  define COMP_CSR_OUTSEL_T3CAP3    (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: Timer 3 input apture 3 (COMP4 only) */
#  define COMP_CSR_OUTSEL_T2CAP2    (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: Timer 2 input apture 2 (COMP6 only) */
#  define COMP_CSR_OUTSEL_T1CAP1    (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: Timer 1 input capture 1 (COMP2 only) */
#  define COMP_CSR_OUTSEL_T2CAP4    (8 << COMP_CSR_INMSEL_SHIFT)  /* 1000: Timer 2 input capture 4 (COMP2 only) */
#  define COMP_CSR_OUTSEL_T15CAP2   (8 << COMP_CSR_INMSEL_SHIFT)  /* 1000: Timer 15 input capture 2 (COMP4 only) */
#  define COMP6_CSR_OUTSEL_T2OCC    (8 << COMP_CSR_INMSEL_SHIFT)  /* 1000: Timer 2 OCREF CLR input (COMP6 only) */
#  define COMP2_CSR_OUTSEL_T2OCC    (9 << COMP_CSR_INMSEL_SHIFT)  /* 1001: Timer 2 OCREF_CLR input (COMP2 only) */
#  define COMP_CSR_OUTSEL_T16OCC    (9 << COMP_CSR_INMSEL_SHIFT)  /* 1001: Timer 16 OCREF_CLR input (COMP6 only) */
#  define COMP_CSR_OUTSEL_T3CAP1    (10 << COMP_CSR_INMSEL_SHIFT) /* 1010: Timer 3 input capture 1 (COMP2 only) */
#  define COMP_CSR_OUTSEL_T15OCC    (10 << COMP_CSR_INMSEL_SHIFT) /* 1010: Timer 15 OCREF_CLR input (COMP4 only) */
#  define COMP_CSR_OUTSEL_T16CAP1   (10 << COMP_CSR_INMSEL_SHIFT) /* 1010: Timer 16 input capture 1 (COMP6 only) */
#  define COMP_CSR_OUTSEL_T3OCC     (11 << COMP_CSR_INMSEL_SHIFT) /* 1011: Timer 3 OCREF_CLR input (COMP2,COMP4 only) */
                                                /* Bit 14: Reserved */
#define COMP_CSR_POL                (1 << 15)   /* Bit 15: comparator output polarity */
                                                /* Bits 16-17: Reserved */
#define COMP_CSR_BLANKING_SHIFT    (18)        /* Bit 18-20: comparator output blanking source */
#define COMP_CSR_BLANKING_MASK     (7 << COMP_CSR_BLANKING_SHIFT)
#  define COMP_CSR_BLANKING_DIS    (0 << COMP_CSR_BLANKING_SHIFT)  /* 000: No blanking */
#  define COMP_CSR_BLANKING_T1OC5  (1 << COMP_CSR_BLANKING_SHIFT)  /* 001: TIM1 OC5 as blanking source (COMP2 only) */
#  define COMP_CSR_BLANKING_T3OC4  (1 << COMP_CSR_BLANKING_SHIFT)  /* 001: TIM3 OC4 as blanking source (COMP4 only) */
#  define COMP_CSR_BLANKING_T2OC3  (2 << COMP_CSR_BLANKING_SHIFT)  /* 010: TIM2 OC3 as blanking source (COMP2 only) */
#  define COMP_CSR_BLANKING_T3OC3  (3 << COMP_CSR_BLANKING_SHIFT)  /* 011: TIM3 OC3 as blanking source (COMP2 only) */
#  define COMP_CSR_BLANKING_T15OC1 (3 << COMP_CSR_BLANKING_SHIFT)  /* 011: TIM15 OC1 as blanking source (COMP4 only) */
#  define COMP_CSR_BLANKING_T2OC4  (3 << COMP_CSR_BLANKING_SHIFT)  /* 011: TIM2 OC4 as blanking source (COMP6 only) */
#  define COMP_CSR_BLANKING_T15OC2 (4 << COMP_CSR_BLANKING_SHIFT)  /* 011: TIM15 OC2 as blanking source (COMP6 only) */
                                                /* Bit 21: Reserved  */
#define COMP_CSR_INMSEL_DAC2CH1     (1 << 22)   /* Bit 22: used with bits 4-6, DAC2_CH1 output */
                                                /* Bits 23-29: Reserved */
#define COMP_CSR_OUT                (1 << 30)   /* Bit 30: comparator output */
#define COMP_CSR_LOCK               (1 << 31)   /* Bit 31: comparator lock */



#endif                          /* __ARCH_ARM_SRC_STM32_CHIP_STM32_COMP_H */
