/****************************************************************************************
 * arch/arm/src/samv71/chip/sam_rstc.h
 * Reset Controller (RSTC) definitions for the SAMV71
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Michael Spahlinger <michael.spahlinger@avat.de>
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

#ifndef __ARCH_ARM_SRC_SAMV7_CHIP_SAM_RSTC_H
#define __ARCH_ARM_SRC_SAMV7_CHIP_SAM_RSTC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* RSTC register offsets ****************************************************************/

#define SAM_RSTC_CR_OFFSET      0x00 /* Control Register */
#define SAM_RSTC_SR_OFFSET      0x04 /* Status Register */
#define SAM_RSTC_MR_OFFSET      0x08 /* Mode Register  */

/* RSTC register addresses **************************************************************/

#define SAM_RSTC_CR             (SAM_RSTC_BASE+SAM_RSTC_CR_OFFSET)
#define SAM_RSTC_SR             (SAM_RSTC_BASE+SAM_RSTC_SR_OFFSET)
#define SAM_RSTC_MR             (SAM_RSTC_BASE+SAM_RSTC_MR_OFFSET)

/* RSTC register bit definitions ********************************************************/

/* Reset Controller Control Register */

#define RSTC_CR_PROCRST         (1 << 0)  /* Bit 0:  Processor Reset */
#define RSTC_CR_EXTRST          (1 << 3)  /* Bit 3:  External Reset */
#define RSTC_CR_KEY_SHIFT       (24)      /* Bits 24-31:  Password */
#define RSTC_CR_KEY_MASK        (0xff << RSTC_CR_KEY_SHIFT)
#  define RSTC_CR_KEY           (0xa5 << RSTC_CR_KEY_SHIFT)

/* Reset Controller Status Register */

#define RSTC_SR_URSTS           (1 << 0)  /* Bit 0:  User Reset Status */
#define RSTC_SR_RSTTYP_SHIFT    (8)       /* Bits 8-10:  Reset Type */
#define RSTC_SR_RSTTYP_MASK     (7 << RSTC_SR_RSTTYP_SHIFT)
#  define RSTC_SR_RSTTYP_PWRUP  (0 << RSTC_SR_RSTTYP_SHIFT) /* General Reset */
#  define RSTC_SR_RSTTYP_BACKUP (1 << RSTC_SR_RSTTYP_SHIFT) /* Backup Reset */
#  define RSTC_SR_RSTTYP_WDOG   (2 << RSTC_SR_RSTTYP_SHIFT) /* Watchdog Reset */
#  define RSTC_SR_RSTTYP_SWRST  (3 << RSTC_SR_RSTTYP_SHIFT) /* Software Reset */
#  define RSTC_SR_RSTTYP_NRST   (4 << RSTC_SR_RSTTYP_SHIFT) /* User Reset NRST pin */
#define RSTC_SR_NRSTL           (1 << 16) /* Bit 16:  NRST Pin Level */
#define RSTC_SR_SRCMP           (1 << 17) /* Bit 17:  Software Reset Command in Progress */

/* Reset Controller Mode Register */

#define RSTC_MR_URSTEN          (1 << 0)  /* Bit 0:  User Reset Enable */
#define RSTC_MR_URSTIEN         (1 << 4)  /* Bit 4:  User Reset Interrupt Enable */
#define RSTC_MR_ERSTL_SHIFT     (8)       /* Bits 8-11:  External Reset Length */
#define RSTC_MR_ERSTL_MASK      (15 << RSTC_MR_ERSTL_SHIFT)
#  define RSTC_MR_ERSTL(n)      ((uint32_t)(n) << RSTC_MR_ERSTL_SHIFT)
#define RSTC_MR_KEY_SHIFT       (24)      /* Bits 24-31:  Password */
#define RSTC_MR_KEY_MASK        (0xff << RSTC_CR_KEY_SHIFT)
#  define RSTC_MR_KEY           (0xa5 << RSTC_CR_KEY_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_CHIP_SAM_RSTC_H */
