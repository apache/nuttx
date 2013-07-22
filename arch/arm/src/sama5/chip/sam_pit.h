/************************************************************************************
 * arch/arm/src/sama5/chip/sam_pit.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PIT_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PIT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* PIT Register Offsets *************************************************************/

#define SAM_PIT_MR_OFFSET    0x0000 /* Mode Register */
#define SAM_PIT_SR_OFFSET    0x0004 /* Status Register */
#define SAM_PIT_PIVR_OFFSET  0x0008 /* Periodic Interval Value Register */
#define SAM_PIT_PIIR_OFFSET  0x000c /* Periodic Interval Image Register */

/* PIT Register Addresses ***********************************************************/

#define SAM_PIT_MR          (SAM_PITC_VBASE+SAM_PIT_MR_OFFSET)
#define SAM_PIT_SR          (SAM_PITC_VBASE+SAM_PIT_SR_OFFSET)
#define SAM_PIT_PIVR        (SAM_PITC_VBASE+SAM_PIT_PIVR_OFFSET)
#define SAM_PIT_PIIR        (SAM_PITC_VBASE+SAM_PIT_PIIR_OFFSET)

/* PIT Register Bit Definitions *****************************************************/

/* Mode Register */

#define PIT_MR_PIV_SHIFT    (0) /* Bits 0-19: Periodic Interval Value */
#define PIT_MR_PIV_MASK     (0x000fffff)
#  define PIT_MR_PIV(n)     (n)
#define PIT_MR_PITEN        (1 << 24) /* Bit 24: Period Interval Timer Enable */
#define PIT_MR_PITIEN       (1 << 25) /* Bit 25: Periodic Interval Timer Interrupt Enable */

/* Status Register */

#define PIT_SR_S            (1 << 0)  /* Bit 0: Periodic Interval Timer Status */

/* Periodic Interval Value Register */
/* Periodic Interval Image Register */

#define PIT_CPIV_SHIFT      (0) /* Bits 0-19: Current Periodic Interval Value */
#define PIT_CPIV_MASK       (0x000fffff)
#define PIT_PICNT_SHIFT     (20) /* Bits 20-31: Periodic Interval Counter */
#define PIT_PICNT_MASK      (0xfff << PIT_PIVR_PICNT_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PIT_H */
