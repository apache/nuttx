/********************************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_gclk.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_GCLK_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_GCLK_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define SAM_NGCLK                    12     /* 12 Clock generators, 0-11 */
#define SAM_NCHANNELS                48     /* 48 Clock generators, 0..47 */

/* GCLK register offsets ********************************************************************/

#define SAM_GCLK_CTRLA_OFFSET        0x0000 /* Control register */
#define SAM_GCLK_SYNCHBUSY_OFFSET    0x0004 /* Status register */

#define SAM_GCLK_GENCTRL_OFFSET(n)   (0x0020 + ((n) << 2)) /* General clock generator n */
#define SAM_GCLK_PCHCTRL_OFFSET(m)   (0x0080 + ((m) << 2)) /* Peripheral channel control m */

/* GCLK register addresses ******************************************************************/

#define SAM_GCLK_CTRLA               (SAM_GCLK_BASE + SAM_GCLK_CTRLA_OFFSET)
#define SAM_GCLK_SYNCHBUSY           (SAM_GCLK_BASE + SAM_GCLK_SYNCHBUSY_OFFSET)

#define SAM_GCLK_GENCTRL(n)          (SAM_GCLK_BASE + SAM_GCLK_GENCTRL_OFFSET(n))
#define SAM_GCLK_PCHCTRL(n)          (SAM_GCLK_BASE + SAM_GCLK_PCHCTRL_OFFSET(n))

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
#  define GCLK_SYNCHBUSY_GENCTRL9    (1 << 11) /* Bit 11: Generator control 9 busy */
#  define GCLK_SYNCHBUSY_GENCTRL10   (1 << 12) /* Bit 12: Generator control 10 busy */
#  define GCLK_SYNCHBUSY_GENCTRL11   (1 << 13) /* Bit 13: Generator control 11 busy */

/* General clock generator n */

#define GCLK_GENCTRL_SRC_SHIFT       (0)       /* Bits 0-4: Generator source selection */
#define GCLK_GENCTRL_SRC_MASK        (31 << GCLK_GENCTRL_SRC_SHIFT)
#  define GCLK_GENCTRL_SRC(n)        ((uint32_t)(n) << GCLK_GENCTRL_SRC_SHIFT)
#  define GCLK_GENCTRL_SRC_XOSC0     (0 << GCLK_GENCTRL_SRC_SHIFT) /* XOSC 0 oscillator input */
#  define GCLK_GENCTRL_SRC_XOSC1     (1 << GCLK_GENCTRL_SRC_SHIFT) /* XOSC 1 oscillator input */
#  define GCLK_GENCTRL_SRC_GCLK_IN   (2 << GCLK_GENCTRL_SRC_SHIFT) /* Generator input pad */
#  define GCLK_GENCTRL_SRC_GCLK_GEN1 (3 << GCLK_GENCTRL_SRC_SHIFT) /* Generic clock generator 1 output */
#  define GCLK_GENCTRL_SRC_OSCULP32K (4 << GCLK_GENCTRL_SRC_SHIFT) /* OSCULP32K oscillator output */
#  define GCLK_GENCTRL_SRC_XOSC32K   (5 << GCLK_GENCTRL_SRC_SHIFT) /* XOSC32K oscillator output */
#  define GCLK_GENCTRL_SRC_DFLL      (6 << GCLK_GENCTRL_SRC_SHIFT) /* DFLL oscillator output */
#  define GCLK_GENCTRL_SRC_DPLL0     (7 << GCLK_GENCTRL_SRC_SHIFT) /* DPLL0 output */
#  define GCLK_GENCTRL_SRC_DPLL1     (8 << GCLK_GENCTRL_SRC_SHIFT) /* DPLL1 output */
#define GCLK_GENCTRL_GENEN           (1 << 8)  /* Bit 8:  Generator enable */
#define GCLK_GENCTRL_IDC             (1 << 9)  /* Bit 9:  Improve duty cycle */
#define GCLK_GENCTRL_OOV             (1 << 10) /* Bit 10: Clock output selection */
#define GCLK_GENCTRL_OE              (1 << 11) /* Bit 11: Clock output enable */
#define GCLK_GENCTRL_DIVSEL          (1 << 12) /* Bit 12: Clock source divider */
#define GCLK_GENCTRL_RUNSTDBY        (1 << 13) /* Bit 13: Run in standby */
#define GCLK_GENCTRL_DIV_SHIFT       (16)      /* Bits 16-23: Generator 0,2-11 Division factor */
#define GCLK_GENCTRL_DIV_MASK        (0xff << GCLK_GENCTRL_DIV_SHIFT)
#  define GCLK_GENCTRL_DIV(n)        ((uint32_t)(n) << GCLK_GENCTRL_DIV_SHIFT)
#define GCLK_GENCTRL1_DIV_SHIFT      (16)      /* Bits 16-31: Generator 1 Division factor **/
#define GCLK_GENCTRL1_DIV_MASK       (0xffff << GCLK_GENCTRL1_DIV_SHIFT)
#  define GCLK_GENCTRL1_DIV(n)       ((uint32_t)(n) << GCLK_GENCTRL1_DIV_SHIFT)

/* Peripheral channel control m */

#define GCLK_PCHCTRL_GEN_SHIFT       (0)       /* Bits 0-3: Generator selection */
#define GCLK_PCHCTRL_GEN_MASK        (15 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN(n)        ((uint32_t)(n) << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN0          (0  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN1          (1  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN2          (2  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN3          (3  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN4          (4  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN5          (5  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN6          (6  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN7          (7  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN8          (8  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN9          (9  << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN10         (10 << GCLK_PCHCTRL_GEN_SHIFT)
#  define GCLK_PCHCTRL_GEN11         (11 << GCLK_PCHCTRL_GEN_SHIFT)
#define GCLK_PCHCTRL_CHEN            (1 << 6)  /* Bit 6:  Channel enable */
#define GCLK_PCHCTRL_WRTLOCK         (1 << 7)  /* Bit 7:  Write lock */

/* PCHCTRL channel mapping ******************************************************************/

#define GCLK_CHAN_OSCCTRL_DFLL       0         /* DFLL input clock source */
#define GCLK_CHAN_OSCCTRL_DPLL0      1         /* Reference clock for DPLL0 */
#define GCLK_CHAN_OSCCTRL_DPLL1      2         /* Reference clock for DPLL1 */
#define GCLK_CHAN_OSCCTRL_DPLL0_32K  3         /* DPLL0 32KHz clock for internal lock timer */
#define GCLK_CHAN_OSCCTRL_DPLL1_32K  3         /* DPLL1 32KHz clock for internal lock timer */
#define GCLK_CHAN_SDHCn_SLOW         3         /* SDHC0-1 Slow */
#define GCLK_CHAN_SDHC0_SLOW         3         /* SDHC0 Slow */
#define GCLK_CHAN_SDHC1_SLOW         3         /* SDHC1 Slow */
#define GCLK_CHAN_SERCOMn_SLOW       3         /* SERCOM Slow (common) */
#define GCLK_CHAN_SERCOM0_SLOW       3         /* SERCOM0 Slow */
#define GCLK_CHAN_SERCOM1_SLOW       3         /* SERCOM1 Slow */
#define GCLK_CHAN_SERCOM2_SLOW       3         /* SERCOM2 Slow */
#define GCLK_CHAN_SERCOM3_SLOW       3         /* SERCOM3 Slow */
#define GCLK_CHAN_SERCOM4_SLOW       3         /* SERCOM4 Slow */
#define GCLK_CHAN_SERCOM5_SLOW       3         /* SERCOM5 Slow */
#define GCLK_CHAN_SERCOM6_SLOW       3         /* SERCOM6 Slow */
#define GCLK_CHAN_SERCOM7_SLOW       3         /* SERCOM7 Slow */
#define GCLK_CHAN_EIC                4         /* EIC */
#define GCLK_CHAN_FREQM_MSR          5         /* FREQM Measure */
#define GCLK_CHAN_FREQM_REF          6         /* FREQM Reference */
#define GCLK_CHAN_SERCOM0_CORE       7         /* SERCOM0 Core */
#define GCLK_CHAN_SERCOM1_CORE       8         /* SERCOM1 Core */
#define GCLK_CHAN_TCn                9         /* TC0-1 */
#define GCLK_CHAN_TC0                9         /* TC0 */
#define GCLK_CHAN_TC1                9         /* TC1 */
#define GCLK_CHAN_USB                10        /* USB */
#define GCLK_CHAN_EVSYS0             11        /* EVSYS0 */
#define GCLK_CHAN_EVSYS1             12        /* EVSYS1 */
#define GCLK_CHAN_EVSYS2             13        /* EVSYS2 */
#define GCLK_CHAN_EVSYS3             14        /* EVSYS3 */
#define GCLK_CHAN_EVSYS4             15        /* EVSYS4 */
#define GCLK_CHAN_EVSYS5             16        /* EVSYS5 */
#define GCLK_CHAN_EVSYS6             17        /* EVSYS6 */
#define GCLK_CHAN_EVSYS7             18        /* EVSYS7 */
#define GCLK_CHAN_EVSYS8             19        /* EVSYS8 */
#define GCLK_CHAN_EVSYS9             20        /* EVSYS9 */
#define GCLK_CHAN_EVSYS10            21        /* EVSYS10 */
#define GCLK_CHAN_EVSYS11            22        /* EVSYS11 */
#define GCLK_CHAN_SERCOM2_CORE       23        /* SERCOM2 Core */
#define GCLK_CHAN_SERCOM3_CORE       24        /* SERCOM3 Core */
#define GCLK_CHAN_TCC0               25        /* TCC0 */
#define GCLK_CHAN_TCC1               25        /* TCC1 */
#define GCLK_CHAN_TC2                26        /* TC2 */
#define GCLK_CHAN_TC3                26        /* TC3 */
#define GCLK_CHAN_CAN0               27        /* CAN0 */
#define GCLK_CHAN_CAN1               28        /* CAN1 */
#define GCLK_CHAN_TCC2               29        /* TCC2 */
#define GCLK_CHAN_TCC3               29        /* TCC3 */
#define GCLK_CHAN_TC4                30        /* TC4 */
#define GCLK_CHAN_TC5                30        /* TC5 */
#define GCLK_CHAN_PDEC               31        /* PDEC */
#define GCLK_CHAN_AC                 32        /* AC */
#define GCLK_CHAN_CCL                33        /* CCL */
#define GCLK_CHAN_SERCOM4_CORE       34        /* SERCOM4 Core */
#define GCLK_CHAN_SERCOM5_CORE       35        /* SERCOM5 Core */
#define GCLK_CHAN_SERCOM6_CORE       36        /* SERCOM6 Core */
#define GCLK_CHAN_SERCOM7_CORE       37        /* SERCOM7 Core */
#define GCLK_CHAN_TCC4               38        /* TCC4 */
#define GCLK_CHAN_TC6                39        /* TC6 */
#define GCLK_CHAN_TC7                39        /* TC7 */
#define GCLK_CHAN_ADC0               40        /* ADC0 */
#define GCLK_CHAN_ADC1               41        /* ADC1 */
#define GCLK_CHAN_DAC                42        /* DAC */
#define GCLK_CHAN_I2S_1              43        /* I2S */
#define GCLK_CHAN_I2S_2              44        /* I2S */
#define GCLK_CHAN_SDHC0              45        /* SDHC0 */
#define GCLK_CHAN_SDHC1              46        /* SDHC1 */
#define GCLK_CHAN_CM4_TRACE          47        /* CM4 Trace */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_GCLK_H */
