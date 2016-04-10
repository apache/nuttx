/********************************************************************************************
 * arch/mips/src/pic32mz/chip/pic32mz-prefetch.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_CHIP_PIC32MZ_PREFETCH_H
#define __ARCH_MIPS_SRC_PIC32MZ_CHIP_PIC32MZ_PREFETCH_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "pic32mz-memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Prefetch register offsets ****************************************************************/

#define PIC32MZ_PRECON_OFFSET   0x0000 /* Prefetch module control register */
#define PIC32MZ_PRESTAT_OFFSET  0x0000 /* Prefetch module status register */

/* Prefetch register addresses **************************************************************/

#define PIC32MZ_PRECON          (PIC32MZ_PREFETCH_K1BASE+PIC32MZ_PRECON_OFFSET)
#define PIC32MZ_PRESTAT         (PIC32MZ_PREFETCH_K1BASE+PIC32MZ_PRESTAT_OFFSET)

/* Prefetch register bit field definitions **************************************************/

/* Prefetch module control register */

#define PRECON_PFMWS_SHIFT      (0)       /* Bits 0-2: PFM Access Time */
#define PRECON_PFMWS_MASK       (7 << PRECON_PFMWS_SHIFT)
#  define PRECON_PFMWS(n)       ((uint32_t)(n) << PRECON_PFMWS_SHIFT) /* n wait states, n=0..7 */
#define PRECON_PREFEN_SHIFT     (4)       /* Bit 4-5: Predictive Prefetch Enable */
#define PRECON_PREFEN_MASK      (3 << PRECON_PREFEN_SHIFT)
#  define PRECON_PREFEN_DISABLE (0 << PRECON_PREFEN_SHIFT) /* Disable predictive prefetch */
#  define PRECON_PREFEN_CPUI    (1 << PRECON_PREFEN_SHIFT) /* Predictive prefetch CPU instructions */
#  define PRECON_PREFEN_CPUID   (2 << PRECON_PREFEN_SHIFT) /* Predictive prefetch CPU instructions and data */
#  define PRECON_PREFEN_ANY     (3 << PRECON_PREFEN_SHIFT) /* Predictive prefetch any address */
#define PRECON_PFMSECEN         (1 << 26) /* Bit 26: Flash SEC Interrupt Enable */

/* Prefetch module status register */

#define PRESTAT_PFMSECCNT_SHIFT (0)       /* Bits 0-7: Flash SEC Count bits */
#define PRESTAT_PFMSECCNT_MASK  (0xff << PRESTAT_PFMSECCNT_SHIFT)
#define PRESTAT_PFMSEC          (1 << 26) /* Bit 26: Flash Single-bit Error Corrected Status */
#define PRESTAT_PFMDED          (1 << 27) /* Bit 27: Flash Double-bit Error Detected Status */

#endif /* __ARCH_MIPS_SRC_PIC32MZ_CHIP_PIC32MZ_PREFETCH_H */
