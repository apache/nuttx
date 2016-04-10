/************************************************************************************
 * arch/arm/src/sama5/chip/sam_bsc.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_BSC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_BSC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* BSC Register Offsets *************************************************************/

#define SAM_BSC_CR_OFFSET    0x0000 /* Boot Sequence Configuration Register */

/* BSC Register Addresses ***********************************************************/

#define SAM_BSC_CR           (SAM_BSC_VBASE+SAM_BSC_CR_OFFSET)

/* BSC Register Bit Definitions *****************************************************/

/* Boot Sequence Configuration Register */

#define BSC_CR_BOOT_SHIFT    (0)    /* Bits 0-7: Boot Media Sequence */
#define BSC_CR_BOOT_MASK     (0xff << BSC_CR_BOOT_SHIFT)
#define BSC_CR_BOOTKEY_SHIFT (16)   /* Bits 16-31: Book key */
#define BSC_CR_BOOTKEY_MASK  (0xffff << BSC_CR_BOOTKEY_SHIFT)
#  define BSC_CR_BOOTKEY     (0x6683 << BSC_CR_BOOTKEY_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_BSC_H */
