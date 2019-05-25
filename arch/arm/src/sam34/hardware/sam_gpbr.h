/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam_gpbr.h
 *
 *   Copyright (C) 2009, 2013-2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_GPBR_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_GPBR_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* GPBR register offsets ****************************************************************/

#define SAM_GPBR_OFFSET(n)   ((n)<<2) /* General purpose back-up registers */
#define SAM_GPBR0_OFFSET     0x00
#define SAM_GPBR1_OFFSET     0x04
#define SAM_GPBR2_OFFSET     0x08
#define SAM_GPBR3_OFFSET     0x0c
#define SAM_GPBR4_OFFSET     0x10
#define SAM_GPBR5_OFFSET     0x14
#define SAM_GPBR6_OFFSET     0x18
#define SAM_GPBR7_OFFSET     0x1c

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_GPBR8_OFFSET   0x20
#  define SAM_GPBR9_OFFSET   0x24
#  define SAM_GPBR10_OFFSET  0x28
#  define SAM_GPBR11_OFFSET  0x2c
#  define SAM_GPBR12_OFFSET  0x30
#  define SAM_GPBR13_OFFSET  0x34
#  define SAM_GPBR14_OFFSET  0x38
#  define SAM_GPBR15_OFFSET  0x3c
#  define SAM_GPBR16_OFFSET  0x40
#  define SAM_GPBR17_OFFSET  0x44
#  define SAM_GPBR18_OFFSET  0x48
#  define SAM_GPBR19_OFFSET  0x4c
#endif

/* GPBR register addresses **************************************************************/

#define SAM_GPBR(n))        (SAM_GPBR_BASE+SAM_GPBR_OFFSET(n))
#define SAM_GPBR0           (SAM_GPBR_BASE+SAM_GPBR0_OFFSET)
#define SAM_GPBR1           (SAM_GPBR_BASE+SAM_GPBR1_OFFSET)
#define SAM_GPBR2           (SAM_GPBR_BASE+SAM_GPBR2_OFFSET)
#define SAM_GPBR3           (SAM_GPBR_BASE+SAM_GPBR3_OFFSET)
#define SAM_GPBR4           (SAM_GPBR_BASE+SAM_GPBR4_OFFSET)
#define SAM_GPBR5           (SAM_GPBR_BASE+SAM_GPBR5_OFFSET)
#define SAM_GPBR6           (SAM_GPBR_BASE+SAM_GPBR6_OFFSET)
#define SAM_GPBR7           (SAM_GPBR_BASE+SAM_GPBR7_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_GPBR8         (SAM_GPBR_BASE+SAM_GPBR8_OFFSET)
#  define SAM_GPBR9         (SAM_GPBR_BASE+SAM_GPBR9_OFFSET)
#  define SAM_GPBR10        (SAM_GPBR_BASE+SAM_GPBR10_OFFSET)
#  define SAM_GPBR11        (SAM_GPBR_BASE+SAM_GPBR11_OFFSET)
#  define SAM_GPBR12        (SAM_GPBR_BASE+SAM_GPBR12_OFFSET)
#  define SAM_GPBR13        (SAM_GPBR_BASE+SAM_GPBR13_OFFSET)
#  define SAM_GPBR14        (SAM_GPBR_BASE+SAM_GPBR14_OFFSET)
#  define SAM_GPBR15        (SAM_GPBR_BASE+SAM_GPBR15_OFFSET)
#  define SAM_GPBR16        (SAM_GPBR_BASE+SAM_GPBR16_OFFSET)
#  define SAM_GPBR17        (SAM_GPBR_BASE+SAM_GPBR17_OFFSET)
#  define SAM_GPBR18        (SAM_GPBR_BASE+SAM_GPBR18_OFFSET)
#  define SAM_GPBR19        (SAM_GPBR_BASE+SAM_GPBR19_OFFSET)
#endif

/* GPBR register bit definitions ********************************************************/

/* All 32-bit values */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_GPBR_H */
