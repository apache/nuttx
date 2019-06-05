/****************************************************************************************
 * arch/arm/src/samv7/hardware/sam_sysc.h
 * Supply Controller (SYSC) definitions for the SAMV71
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SYSC_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SYSC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SYSC register offsets ****************************************************************/

#define SAM_SYSC_WPMR_OFFSET   0x0004 /* Write Protection Mode Register */

/* SYSC register addresses **************************************************************/

#define SAM_SYSC_WPMR          (SAM_SYSC_BASE+SAM_SYSC_WPMR_OFFSET)

/* SYSC register bit definitions ********************************************************/
/* System Controller Write Protect Mode Register */

#define SYSC_WPMR_WPEN         (1 << 0)  /* Bit 0:  Write Protect Enable */
#define SYSC_WPMR_WPKEY_SHIFT  (8)       /* Bits 8-31: Write Protect KEY */
#define SYSC_WPMR_WPKEY_MASK   (0x00ffffff << SYSC_WPMR_WPKEY_SHIFT)
#  define SYSC_WPMR_WPKEY      (0x00525443 << SYSC_WPMR_WPKEY_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SYSC_H */
