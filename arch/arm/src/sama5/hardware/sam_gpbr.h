/************************************************************************************
 * arch/arm/src/sama5/hardware/sam_gpbr.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_GPBR_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_GPBR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* GPBR Register Offsets ************************************************************/

#define SAM_SYS_GPBR_OFFSET(n) ((n) << 2) /* General Purpose Backup Register n, 1=0..3 */
#define SAM_SYS_GPBR0_OFFSET   0x0000 /* General Purpose Backup Register 0 */
#define SAM_SYS_GPBR1_OFFSET   0x0004 /* General Purpose Backup Register 0 */
#define SAM_SYS_GPBR2_OFFSET   0x0008 /* General Purpose Backup Register 0 */
#define SAM_SYS_GPBR3_OFFSET   0x000c /* General Purpose Backup Register 0 */

/* GPBR Register Addresses **********************************************************/

#define SAM_SYS_GPBR(n)        (SAM_GPBR_VBASE+SAM_SYS_GPBR_OFFSET(n))
#define SAM_SYS_GPBR0          (SAM_GPBR_VBASE+SAM_SYS_GPBR0_OFFSET)
#define SAM_SYS_GPBR1          (SAM_GPBR_VBASE+SAM_SYS_GPBR1_OFFSET)
#define SAM_SYS_GPBR2          (SAM_GPBR_VBASE+SAM_SYS_GPBR2_OFFSET)
#define SAM_SYS_GPBR3          (SAM_GPBR_VBASE+SAM_SYS_GPBR3_OFFSET)

/* GPBR Register Bit Definitions ****************************************************/
/* All GPBR registers hold user-defined, 32-bit values */

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_GPBR_H */
