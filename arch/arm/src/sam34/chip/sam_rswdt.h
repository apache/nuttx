/****************************************************************************************
 * arch/arm/src/sam34/chip/sam_rswdt.h
 * Reinforced Safety Watchdog Timer (RSWDT) for the SAM4E
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM_RSWDT_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM_RSWDT_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* RSWDT register offsets ***************************************************************/

#define SAM_RSWDT_CR_OFFSET         0x0000 /* Control Register */
#define SAM_RSWDT_MR_OFFSET         0x0004 /* Mode Register */
#define SAM_RSWDT_SR_OFFSET         0x0008 /* Status Register */

/* RSWDT register addresses *************************************************************/

#define SAM_RSWDT_CR                (SAM_RSWDT_BASE+SAM_RSWDT_CR_OFFSET)
#define SAM_RSWDT_MR                (SAM_RSWDT_BASE+SAM_RSWDT_MR_OFFSET)
#define SAM_RSWDT_SR                (SAM_RSWDT_BASE+SAM_RSWDT_SR_OFFSET)

/* RSWDT register bit definitions *******************************************************/
/* Watchdog Timer Control Register */

#define RSWDT_CR_WDRSTT             (1 << 0)   /* Bit 0:  Watchdog Rest */
#define RSWDT_CR_KEY_SHIFT          (24)       /* Bits 24-31:  Password */
#define RSWDT_CR_KEY_MASK           (0xff << RSWDT_CR_KEY_SHIFT)
#  define RSWDT_CR_KEY              (0xc4 << RSWDT_CR_KEY_SHIFT)

/* Watchdog Timer Mode Register */

#define RSWDT_MR_WDV_SHIFT          (0)       /* Bits 0-11:  Watchdog Counter Value */
#define RSWDT_MR_WDV_MASK           (0xfff << RSWDT_MR_WDV_SHIFT)
#  define RSWDT_MR_WDV(n)           ((uint32_t)(n) << RSWDT_MR_WDV_SHIFT)
#define RSWDT_MR_WDFIEN             (1 << 12) /* Bit 12: Watchdog Fault Interrupt Enable */
#define RSWDT_MR_WDRSTEN            (1 << 13) /* Bit 13: Watchdog Reset Enable */
#define RSWDT_MR_WDRPROC            (1 << 14) /* Bit 14: Watchdog Reset Processor */
#define RSWDT_MR_WDDIS              (1 << 15) /* Bit 15: Watchdog Disable */
#define RSWDT_MR_WDD_SHIFT          (16)      /* Bits 16-27:  Watchdog Delta Value */
#define RSWDT_MR_WDD_MASK           (0xfff << RSWDT_MR_WDD_SHIFT)
#  define RSWDT_MR_WDD(n)           ((uint32_t)(n) << RSWDT_MR_WDD_SHIFT)
#define RSWDT_MR_WDDBGHLT           (1 << 28) /* Bit 28: Watchdog Debug Halt */
#define RSWDT_MR_WDIDLEHLT          (1 << 29) /* Bit 29: Watchdog Idle Halt */

/* Watchdog Timer Status Register */

#define RSWDT_SR_WDUNF              (1 << 0)  /* Bit 0:  Watchdog Underflow */
#define RSWDT_SR_WDERR              (1 << 1)  /* Bit 1:  Watchdog Error */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM_RSWDT_H */
