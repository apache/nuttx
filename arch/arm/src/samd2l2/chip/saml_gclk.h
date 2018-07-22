/********************************************************************************************
 * arch/arm/src/samd2l2/chip/saml_gclk.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_GCLK_H
#define __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_GCLK_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* GCLK register offsets ********************************************************************/

#define SAM_GCLK_CTRLA_OFFSET        0x0000 /* Control register */
#define SAM_GCLK_SYNCHBUSY_OFFSET    0x0004 /* Status register */

#define SAM_GCLK_GENCTRL_OFFSET(n)   (0x0020 + ((n) << 2)) /* General clock generator n */
#define SAM_GCLK_PCHCTRL_OFFSET(m)   (0x0080 + ((m) << 2)) /* Peripheral channel control m */

/* GCLK register addresses ******************************************************************/

#define SAM_GCLK_CTRLA               (SAM_GCLK_BASE+SAM_GCLK_CTRLA_OFFSET)
#define SAM_GCLK_SYNCHBUSY           (SAM_GCLK_BASE+SAM_GCLK_SYNCHBUSY_OFFSET)

#define SAM_GCLK_GENCTRL(n)          (SAM_GCLK_BASE+SAM_GCLK_GENCTRL_OFFSET(n))
#define SAM_GCLK_PCHCTRL(m)          (SAM_GCLK_BASE+SAM_GCLK_PCHCTRL_OFFSET(m))

/* GCLK register bit definitions ************************************************************/

/* Control register */

#define GCLK_CTRLA_SWRST             (1 << 0)  /* Bit 0:  Software Reset */

/* Status register */

#define GCLK_SYNCHBUSY_SWRST         (1 << 0)  /* Bit 0:  SWRST synchronization busy */
#define GCLK_SYNCHBUSY_GENCTRL(n)    (1 << ((n) + 2))  /* Bit n+2: Generator control n busy */
#  define GCLK_SYNCHBUSY_GENCTRL0    (1 << 2)  /* Bit 2:  Generator control 0 busy */
#  define GCLK_SYNCHBUSY_GENCTRL1    (1 << 3)  /* Bit 3:  Generator control 1 busy */
#  define GCLK_SYNCHBUSY_GENCTRL2    (1 << 4)  /* Bit 4:  Generator control 2 busy */
#  define GCLK_SYNCHBUSY_GENCTRL3    (1 << 5)  /* Bit 5:  Generator control 3 busy */
#  define GCLK_SYNCHBUSY_GENCTRL4    (1 << 6)  /* Bit 6:  Generator control 4 busy */
#  define GCLK_SYNCHBUSY_GENCTRL5    (1 << 7)  /* Bit 7:  Generator control 5 busy */
#  define GCLK_SYNCHBUSY_GENCTRL6    (1 << 8)  /* Bit 8:  Generator control 6 busy */
#  define GCLK_SYNCHBUSY_GENCTRL7    (1 << 9)  /* Bit 9:  Generator control 7 busy */
#  define GCLK_SYNCHBUSY_GENCTRL8    (1 << 10) /* Bit 10: Generator control 8 busy */

/* General clock generator n */

#define GCLK_GENCTRL_SRC_SHIFT       (0)       /* Bits 0-4: Generator source selection */
#define GCLK_GENCTRL_SRC_MASK        (31 << GCLK_GENCTRL_SRC_SHIFT)
#  define GCLK_GENCTRL_SRC_XOSC      (0 << GCLK_GENCTRL_SRC_SHIFT) /* XOSC oscillator inpupt */
#  define GCLK_GENCTRL_SRC_GCLK_IN   (1 << GCLK_GENCTRL_SRC_SHIFT) /* Generator input pad */
#  define GCLK_GENCTRL_SRC_GLCK_GEN1 (2 << GCLK_GENCTRL_SRC_SHIFT) /* Generic clock generater 1 output */
#  define GCLK_GENCTRL_SRC_OSCULP32K (3 << GCLK_GENCTRL_SRC_SHIFT) /* OSCULP32K oscillator output */
#  define GCLK_GENCTRL_SRC_OSC32K    (4 << GCLK_GENCTRL_SRC_SHIFT) /* OSC32K osccillator output */
#  define GCLK_GENCTRL_SRC_XOSC32K   (5 << GCLK_GENCTRL_SRC_SHIFT) /* XOSC32K oscillator output */
#  define GCLK_GENCTRL_SRC_OSC16M    (6 << GCLK_GENCTRL_SRC_SHIFT) /* OSC16M oscillator output */
#  define GCLK_GENCTRL_SRC_DFLL48M   (7 << GCLK_GENCTRL_SRC_SHIFT) /* DFLL48M output */
#  define GCLK_GENCTRL_SRC_DPLL96M   (8 << GCLK_GENCTRL_SRC_SHIFT) /* DPLL96M output */
#define GCLK_GENCTRL_GENEN           (1 << 8)  /* Bit 8:  Generator enable */
#define GCLK_GENCTRL_IDC             (1 << 9)  /* Bit 9:  Improve duty cycle */
#define GCLK_GENCTRL_OOV             (1 << 10) /* Bit 10: Clock output selection */
#define GCLK_GENCTRL_OE              (1 << 11) /* Bit 11: Clock output enable */
#define GCLK_GENCTRL_DIVSEL          (1 << 12) /* Bit 12: Clock source divider */
#define GCLK_GENCTRL_RUNSTDBY        (1 << 13) /* Bit 13: Run in standby */
#define GCLK_GENCTRL_DIV_SHIFT       (16)      /* Bits 16-31: Generator 0,2-8 Division factor */
#define GCLK_GENCTRL_DIV_MASK        (0xff << GCLK_GENCTRL_DIV_SHIFT)
#  define GCLK_GENCTRL_DIV(n)        ((uint32_t)(n) << GCLK_GENCTRL_DIV_SHIFT)
#define GCLK_GENCTRL1_DIV_SHIFT      (16)      /* Bits 16-23: Generator 1 Division factor **/
#define GCLK_GENCTRL1_DIV_MASK       (0xffff << GCLK_GENCTRL1_DIV_SHIFT)
#  define GCLK_GENCTRL1_DIV(n)       ((uint32_t)(n) << GCLK_GENCTRL1_DIV_SHIFT)

/* Peripheral channel control m */

#define GCLK_PCHCTRL_GEN_SHIFT       (0)       /* Bits 0-3: Generator selection */
#define GCLK_PCHCTRL_GEN_MASK        (15 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN(n)        ((uint32_t)(n) << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN0          (0 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN1          (1 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN2          (2 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN3          (3 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN4          (4 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN5          (5 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN6          (6 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN7          (7 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN8          (8 << GCLK_PCHCTRL_GEN_SHIFT)
#define GCLK_PCHCTRL_CHEN            (1 << 6)  /* Bit 6:  Channel enable */
#define GCLK_PCHCTRL_WRTLOCK         (1 << 7)  /* Bit 7:  Write lock */

/* PCHCTRL channel mapping ******************************************************************/

#define GCLK_CHAN_DFLL48M_REF        0         /* DFLL48M Reference */
#define GCLK_CHAN_DPLL               1         /* FDPLL96M input clock source for reference */
#define GCLK_CHAN_DPLL_32K           2         /* FDPLL96M 32kHz clock for FDPLL96M internal lock timer */
#define GCLK_CHAN_EIC                3         /* EIC */
#define GCLK_CHAN_USB                4         /* USB */
#define GCLK_CHAN_EVSYS_CH0          5         /* EVSYS_CHANNEL_0 */
#define GCLK_CHAN_EVSYS_CH1          6         /* EVSYS_CHANNEL_1 */
#define GCLK_CHAN_EVSYS_CH2          7         /* EVSYS_CHANNEL_2 */
#define GCLK_CHAN_EVSYS_CH3          8         /* EVSYS_CHANNEL_3 */
#define GCLK_CHAN_EVSYS_CH4          9         /* EVSYS_CHANNEL_4 */
#define GCLK_CHAN_EVSYS_CH5          10        /* EVSYS_CHANNEL_5 */
#define GCLK_CHAN_EVSYS_CH6          11        /* EVSYS_CHANNEL_6 */
#define GCLK_CHAN_EVSYS_CH7          12        /* EVSYS_CHANNEL_7 */
#define GCLK_CHAN_EVSYS_CH8          13        /* EVSYS_CHANNEL_8 */
#define GCLK_CHAN_EVSYS_CH9          14        /* EVSYS_CHANNEL_9 */
#define GCLK_CHAN_EVSYS_CH10         15        /* EVSYS_CHANNEL_10 */
#define GCLK_CHAN_EVSYS_CH11         16        /* EVSYS_CHANNEL_11 */
#define GCLK_CHAN_SERCOM0_SLOW       17        /* SERCOM0_SLOW */
#define GCLK_CHAN_SERCOM1_SLOW       17        /* SERCOM1_SLOW */
#define GCLK_CHAN_SERCOM2_SLOW       17        /* SERCOM2_SLOW */
#define GCLK_CHAN_SERCOM3_SLOW       17        /* SERCOM3_SLOW */
#define GCLK_CHAN_SERCOM4_SLOW       17        /* SERCOM4_SLOW */
#define GCLK_CHAN_SERCOM0_CORE       18        /* SERCOM0_CORE */
#define GCLK_CHAN_SERCOM1_CORE       19        /* SERCOM1_CORE */
#define GCLK_CHAN_SERCOM2_CORE       20        /* SERCOM2_CORE */
#define GCLK_CHAN_SERCOM3_CORE       21        /* SERCOM3_CORE */
#define GCLK_CHAN_SERCOM4_CORE       22        /* SERCOM4_CORE */
#define GCLK_CHAN_SERCOM5_SLOW       23        /* SERCOM5_SLOW */
#define GCLK_CHAN_SERCOM5_CORE       24        /* SERCOM5_CORE */
#define GCLK_CHAN_TCC0               25        /* TCC0 */
#define GCLK_CHAN_TCC1               25        /* TCC1 */
#define GCLK_CHAN_TCC2               26        /* TCC2 */
#define GCLK_CHAN_TC3_1              26        /* TC3 */
#define GCLK_CHAN_TC0                27        /* TC0 */
#define GCLK_CHAN_TC1                27        /* TC1 */
#define GCLK_CHAN_TC2                28        /* TC2 */
#define GCLK_CHAN_TC3_2              28        /* TC3 */
#define GCLK_CHAN_TC4                29        /* TC4 */
#define GCLK_CHAN_ADC                30        /* ADC */
#define GCLK_CHAN_AC                 31        /* AC */
#define GCLK_CHAN_DAC                32        /* DAC */
#define GCLK_CHAN_PTC                33        /* PTC */
#define GCLK_CHAN_CCL                34        /* CCL */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_GCLK_H */
