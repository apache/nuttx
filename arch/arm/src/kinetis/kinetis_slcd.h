/************************************************************************************
 * arch/arm/src/kinetis/kinetis_slcd.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_SLCD_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_SLCD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_LCD_GCR_OFFSET       0x0000 /* LCD general control register */
#define KINETIS_LCD_AR_OFFSET        0x0004 /* LCD auxiliary register */
#define KINETIS_LCD_FDCR_OFFSET      0x0008 /* LCD fault detect control register */
#define KINETIS_LCD_FDSR_OFFSET      0x000c /* LCD fault detect status register */
#define KINETIS_LCD_PENL_OFFSET      0x0010 /* LCD pin enable register */
#define KINETIS_LCD_PENH_OFFSET      0x0014 /* LCD pin enable register */
#define KINETIS_LCD_BPENL_OFFSET     0x0018 /* LCD backplane enable register */
#define KINETIS_LCD_BPENH_OFFSET     0x001c /* LCD backplane enable register */
#define KINETIS_LCD_WF3TO0_OFFSET    0x0020 /* LCD waveform register */
#define KINETIS_LCD_WF7TO4_OFFSET    0x0024 /* LCD waveform register */
#define KINETIS_LCD_WF11TO8_OFFSET   0x0028 /* LCD waveform register */
#define KINETIS_LCD_WF15TO12_OFFSET  0x002c /* LCD waveform register */
#define KINETIS_LCD_WF19TO16_OFFSET  0x0030 /* LCD waveform register */
#define KINETIS_LCD_WF23TO20_OFFSET  0x0034 /* LCD waveform register */
#define KINETIS_LCD_WF27TO24_OFFSET  0x0038 /* LCD waveform register */
#define KINETIS_LCD_WF31TO28_OFFSET  0x003c /* LCD waveform register */
#define KINETIS_LCD_WF35TO32_OFFSET  0x0040 /* LCD waveform register */
#define KINETIS_LCD_WF39TO36_OFFSET  0x0044 /* LCD waveform register */
#define KINETIS_LCD_WF43TO40_OFFSET  0x0048 /* LCD waveform register */
#define KINETIS_LCD_WF47TO44_OFFSET  0x004c /* LCD waveform register */
#define KINETIS_LCD_WF51TO48_OFFSET  0x0050 /* LCD waveform register */
#define KINETIS_LCD_WF55TO52_OFFSET  0x0054 /* LCD waveform register */
#define KINETIS_LCD_WF59TO56_OFFSET  0x0058 /* LCD waveform register */
#define KINETIS_LCD_WF63TO60_OFFSET  0x005C /* LCD waveform register */

/* Register Addresses ***************************************************************/

#define KINETIS_LCD_GCR              (KINETIS_SLCD_BASE+KINETIS_LCD_GCR_OFFSET)
#define KINETIS_LCD_AR               (KINETIS_SLCD_BASE+KINETIS_LCD_AR_OFFSET)
#define KINETIS_LCD_FDCR             (KINETIS_SLCD_BASE+KINETIS_LCD_FDCR_OFFSET)
#define KINETIS_LCD_FDSR             (KINETIS_SLCD_BASE+KINETIS_LCD_FDSR_OFFSET)
#define KINETIS_LCD_PENL             (KINETIS_SLCD_BASE+KINETIS_LCD_PENL_OFFSET)
#define KINETIS_LCD_PENH             (KINETIS_SLCD_BASE+KINETIS_LCD_PENH_OFFSET)
#define KINETIS_LCD_BPENL            (KINETIS_SLCD_BASE+KINETIS_LCD_BPENL_OFFSET)
#define KINETIS_LCD_BPENH            (KINETIS_SLCD_BASE+KINETIS_LCD_BPENH_OFFSET)
#define KINETIS_LCD_WF3TO0           (KINETIS_SLCD_BASE+KINETIS_LCD_WF3TO0_OFFSET)
#define KINETIS_LCD_WF7TO4           (KINETIS_SLCD_BASE+KINETIS_LCD_WF7TO4_OFFSET)
#define KINETIS_LCD_WF11TO8          (KINETIS_SLCD_BASE+KINETIS_LCD_WF11TO8_OFFSET)
#define KINETIS_LCD_WF15TO12         (KINETIS_SLCD_BASE+KINETIS_LCD_WF15TO12_OFFSET)
#define KINETIS_LCD_WF19TO16         (KINETIS_SLCD_BASE+KINETIS_LCD_WF19TO16_OFFSET)
#define KINETIS_LCD_WF23TO20         (KINETIS_SLCD_BASE+KINETIS_LCD_WF23TO20_OFFSET)
#define KINETIS_LCD_WF27TO24         (KINETIS_SLCD_BASE+KINETIS_LCD_WF27TO24_OFFSET)
#define KINETIS_LCD_WF31TO28         (KINETIS_SLCD_BASE+KINETIS_LCD_WF31TO28_OFFSET)
#define KINETIS_LCD_WF35TO32         (KINETIS_SLCD_BASE+KINETIS_LCD_WF35TO32_OFFSET)
#define KINETIS_LCD_WF39TO36         (KINETIS_SLCD_BASE+KINETIS_LCD_WF39TO36_OFFSET)
#define KINETIS_LCD_WF43TO40         (KINETIS_SLCD_BASE+KINETIS_LCD_WF43TO40_OFFSET)
#define KINETIS_LCD_WF47TO44         (KINETIS_SLCD_BASE+KINETIS_LCD_WF47TO44_OFFSET)
#define KINETIS_LCD_WF51TO48         (KINETIS_SLCD_BASE+KINETIS_LCD_WF51TO48_OFFSET)
#define KINETIS_LCD_WF55TO52         (KINETIS_SLCD_BASE+KINETIS_LCD_WF55TO52_OFFSET)
#define KINETIS_LCD_WF59TO56         (KINETIS_SLCD_BASE+KINETIS_LCD_WF59TO56_OFFSET)
#define KINETIS_LCD_WF63TO60         (KINETIS_SLCD_BASE+KINETIS_LCD_WF63TO60_OFFSET)

/* Register Bit Definitions *********************************************************/

/* LCD general control register */
#define LCD_GCR_
/* LCD auxiliary register */
#define LCD_AR_
/* LCD fault detect control register */
#define LCD_FDCR_
/* LCD fault detect status register */
#define LCD_FDSR_
/* LCD pin enable register */
#define LCD_PENL_
/* LCD pin enable register */
#define LCD_PENH_
/* LCD backplane enable register */
#define LCD_BPENL_
/* LCD backplane enable register */
#define LCD_BPENH_
/* LCD waveform register */
#define LCD_WF3TO0_
/* LCD waveform register */
#define LCD_WF7TO4_
/* LCD waveform register */
#define LCD_WF11TO8_
/* LCD waveform register */
#define LCD_WF15TO12_
/* LCD waveform register */
#define LCD_WF19TO16_
/* LCD waveform register */
#define LCD_WF23TO20_
/* LCD waveform register */
#define LCD_WF27TO24_
/* LCD waveform register */
#define LCD_WF31TO28_
/* LCD waveform register */
#define LCD_WF35TO32_
/* LCD waveform register */
#define LCD_WF39TO36_
/* LCD waveform register */
#define LCD_WF43TO40_
/* LCD waveform register */
#define LCD_WF47TO44_
/* LCD waveform register */
#define LCD_WF51TO48_
/* LCD waveform register */
#define LCD_WF55TO52_
/* LCD waveform register */
#define LCD_WF59TO56_
/* LCD waveform register */
#define LCD_WF63TO60_

                (1 << nn)  /* Bit nn:  
_SHIFT          (nn)       /* Bits nn-nn: 
_MASK           (nn << nn)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_SLCD_H */
