/************************************************************************************
 * arch/arm/src/kinetis/kinetis_tsi.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_TSI_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_TSI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_TSI_GENCS_OFFSET       0x0000 /* General Control and Status Register */
#define KINETIS_TSI_SCANC_OFFSET       0x0004 /* SCAN control register */
#define KINETIS_TSI_PEN_OFFSET         0x0008 /* Pin enable register */
#define KINETIS_TSI_STATUS_OFFSET      0x000c /* Status Register */

#define KINETIS_TSI_CNTR_OFFSET(n)     (0x0100+(((n)-1)<<1) /* Counter Register n */
#define KINETIS_TSI_CNTR1_OFFSET       0x0100 /* Counter Register 1 */
#define KINETIS_TSI_CNTR3_OFFSET       0x0104 /* Counter Register 3 */
#define KINETIS_TSI_CNTR5_OFFSET       0x0108 /* Counter Register 5 */
#define KINETIS_TSI_CNTR7_OFFSET       0x010c /* Counter Register 7 */
#define KINETIS_TSI_CNTR9_OFFSET       0x0110 /* Counter Register 9 */
#define KINETIS_TSI_CNTR11_OFFSET      0x0114 /* Counter Register 11 */
#define KINETIS_TSI_CNTR13_OFFSET      0x0118 /* Counter Register 13 */
#define KINETIS_TSI_CNTR15_OFFSET      0x011c /* Counter Register 15 */

#define KINETIS_TSI_THRESHLD_OFFSET(n) (0x0120+((n)<<2)) /* Channel n threshold register */
#define KINETIS_TSI_THRESHLD0_OFFSET   0x0120 /* Channel 0 threshold register */
#define KINETIS_TSI_THRESHLD1_OFFSET   0x0124 /* Channel 1 threshold register */
#define KINETIS_TSI_THRESHLD2_OFFSET   0x0128 /* Channel 2 threshold register */
#define KINETIS_TSI_THRESHLD3_OFFSET   0x012c /* Channel 3 threshold register */
#define KINETIS_TSI_THRESHLD4_OFFSET   0x0130 /* Channel 4 threshold register */
#define KINETIS_TSI_THRESHLD5_OFFSET   0x0134 /* Channel 5 threshold register */
#define KINETIS_TSI_THRESHLD6_OFFSET   0x0138 /* Channel 6 threshold register */
#define KINETIS_TSI_THRESHLD7_OFFSET   0x013c /* Channel 7 threshold register */
#define KINETIS_TSI_THRESHLD8_OFFSET   0x0140 /* Channel 8 threshold register */
#define KINETIS_TSI_THRESHLD9_OFFSET   0x0144 /* Channel 9 threshold register */
#define KINETIS_TSI_THRESHLD10_OFFSET  0x0148 /* Channel 10 threshold register */
#define KINETIS_TSI_THRESHLD11_OFFSET  0x014c /* Channel 11 threshold register */
#define KINETIS_TSI_THRESHLD12_OFFSET  0x0150 /* Channel 12 threshold register */
#define KINETIS_TSI_THRESHLD13_OFFSET  0x0154 /* Channel 13 threshold register */
#define KINETIS_TSI_THRESHLD14_OFFSET  0x0158 /* Channel 14 threshold register */
#define KINETIS_TSI_THRESHLD15_OFFSET  0x015c /* Channel 15 threshold register */

/* Register Addresses ***************************************************************/

#define KINETIS_TSI0_GENCS             (KINETIS_TSI0_BASE+KINETIS_TSI_GENCS_OFFSET)
#define KINETIS_TSI0_SCANC             (KINETIS_TSI0_BASE+KINETIS_TSI_SCANC_OFFSET)
#define KINETIS_TSI0_PEN               (KINETIS_TSI0_BASE+KINETIS_TSI_PEN_OFFSET)
#define KINETIS_TSI0_STATUS            (KINETIS_TSI0_BASE+KINETIS_TSI_STATUS_OFFSET)

#define KINETIS_TSI0_CNTR              (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR_OFFSET(n))
#define KINETIS_TSI0_CNTR1             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR1_OFFSET)
#define KINETIS_TSI0_CNTR3             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR3_OFFSET)
#define KINETIS_TSI0_CNTR5             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR5_OFFSET)
#define KINETIS_TSI0_CNTR7             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR7_OFFSET)
#define KINETIS_TSI0_CNTR9             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR9_OFFSET)
#define KINETIS_TSI0_CNTR11            (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR11_OFFSET)
#define KINETIS_TSI0_CNTR13            (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR13_OFFSET)
#define KINETIS_TSI0_CNTR15            (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR15_OFFSET)

#define KINETIS_TSI0_THRESHLD(n)       (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD_OFFSET(n))
#define KINETIS_TSI0_THRESHLD0         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD0_OFFSET)
#define KINETIS_TSI0_THRESHLD1         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD1_OFFSET)
#define KINETIS_TSI0_THRESHLD2         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD2_OFFSET)
#define KINETIS_TSI0_THRESHLD3         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD3_OFFSET)
#define KINETIS_TSI0_THRESHLD4         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD4_OFFSET)
#define KINETIS_TSI0_THRESHLD5         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD5_OFFSET)
#define KINETIS_TSI0_THRESHLD6         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD6_OFFSET)
#define KINETIS_TSI0_THRESHLD7         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD7_OFFSET)
#define KINETIS_TSI0_THRESHLD8         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD8_OFFSET)
#define KINETIS_TSI0_THRESHLD9         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD9_OFFSET)
#define KINETIS_TSI0_THRESHLD10        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD10_OFFSET)
#define KINETIS_TSI0_THRESHLD11        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD11_OFFSET)
#define KINETIS_TSI0_THRESHLD12        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD12_OFFSET)
#define KINETIS_TSI0_THRESHLD13        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD13_OFFSET)
#define KINETIS_TSI0_THRESHLD14        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD14_OFFSET)
#define KINETIS_TSI0_THRESHLD15        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD15_OFFSET)

/* Register Bit Definitions *********************************************************/

/* General Control and Status Register */
#define TSI_GENCS_
/* SCAN control register */
#define TSI_SCANC_
/* Pin enable register */
#define TSI_PEN_
/* Status Register */
#define TSI_STATUS_
/* Counter Register n */
#define TSI_CNTR_
/* Channel n threshold register */
#define TSI_THRESHLD_

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

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_TSI_H */
