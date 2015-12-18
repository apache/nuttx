/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_sys.h
 * Peripheral Control Register Definitions
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller, Technical Reference Manual, Texas
 *   Instruments, Literature Number: SPNU517A, September 2013
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PCR_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PCR_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define TMS570_PCR_PMPROTSET0_OFFSET    0x0000  /* Peripheral Memory Protection Set Register 0 */
#define TMS570_PCR_PMPROTSET1_OFFSET    0x0004  /* Peripheral Memory Protection Set Register 1 */
#define TMS570_PCR_PMPROTCLR0_OFFSET    0x0010  /* Peripheral Memory Protection Clear Register 0 */
#define TMS570_PCR_PMPROTCLR1_OFFSET    0x0014  /* Peripheral Memory Protection Clear Register 1 */
#define TMS570_PCR_PPROTSET0_OFFSET     0x0020  /* Peripheral Protection Set Register 0 */
#define TMS570_PCR_PPROTSET1_OFFSET     0x0024  /* Peripheral Protection Set Register 1 */
#define TMS570_PCR_PPROTSET2_OFFSET     0x0028  /* Peripheral Protection Set Register 2 */
#define TMS570_PCR_PPROTSET3_OFFSET     0x002c  /* Peripheral Protection Set Register 3 */
#define TMS570_PCR_PPROTCLR0_OFFSET     0x0040  /* Peripheral Protection Clear Register 0 */
#define TMS570_PCR_PPROTCLR1_OFFSET     0x0044  /* Peripheral Protection Clear Register 1 */
#define TMS570_PCR_PPROTCLR2_OFFSET     0x0048  /* Peripheral Protection Clear Register 2 */
#define TMS570_PCR_PPROTCLR3_OFFSET     0x004c  /* Peripheral Protection Clear Register 3 */
#define TMS570_PCR_PCSPWRDWNSET0_OFFSET 0x0060  /* Peripheral Memory Power-Down Set Register 0 */
#define TMS570_PCR_PCSPWRDWNSET1_OFFSET 0x0064  /* Peripheral Memory Power-Down Set Register 1 */
#define TMS570_PCR_PCSPWRDWNCLR0_OFFSET 0x0070  /* Peripheral Memory Power-Down Clear Register 0 */
#define TMS570_PCR_PCSPWRDWNCLR1_OFFSET 0x0074  /* Peripheral Memory Power-Down Clear Register 1 */
#define TMS570_PCR_PSPWRDWNSET0_OFFSET  0x0080  /* Peripheral Power-Down Set Register 0 */
#define TMS570_PCR_PSPWRDWNSET1_OFFSET  0x0084  /* Peripheral Power-Down Set Register 1 */
#define TMS570_PCR_PSPWRDWNSET2_OFFSET  0x0088  /* Peripheral Power-Down Set Register 2 */
#define TMS570_PCR_PSPWRDWNSET3_OFFSET  0x008c  /* Peripheral Power-Down Set Register 3 */
#define TMS570_PCR_PSPWRDWNCLR0_OFFSET  0x00a0  /* Peripheral Power-Down Clear Register 0 */
#define TMS570_PCR_PSPWRDWNCLR1_OFFSET  0x00a4  /* Peripheral Power-Down Clear Register 1 */
#define TMS570_PCR_PSPWRDWNCLR2_OFFSET  0x00a8  /* Peripheral Power-Down Clear Register 2 */
#define TMS570_PCR_PSPWRDWNCLR3_OFFSET  0x00ac  /* Peripheral Power-Down Clear Register 3 */

/* Register Addresses *******************************************************************************/

#define TMS570_PCR_PMPROTSET0           (TMS570_PCR_BASE+TMS570_PCR_PMPROTSET0_OFFSET)
#define TMS570_PCR_PMPROTSET1           (TMS570_PCR_BASE+TMS570_PCR_PMPROTSET1_OFFSET)
#define TMS570_PCR_PMPROTCLR0           (TMS570_PCR_BASE+TMS570_PCR_PMPROTCLR0_OFFSET)
#define TMS570_PCR_PMPROTCLR1           (TMS570_PCR_BASE+TMS570_PCR_PMPROTCLR1_OFFSET)
#define TMS570_PCR_PPROTSET0            (TMS570_PCR_BASE+TMS570_PCR_PPROTSET0_OFFSET)
#define TMS570_PCR_PPROTSET1            (TMS570_PCR_BASE+TMS570_PCR_PPROTSET1_OFFSET)
#define TMS570_PCR_PPROTSET2            (TMS570_PCR_BASE+TMS570_PCR_PPROTSET2_OFFSET)
#define TMS570_PCR_PPROTSET3            (TMS570_PCR_BASE+TMS570_PCR_PPROTSET3_OFFSET)
#define TMS570_PCR_PPROTCLR0            (TMS570_PCR_BASE+TMS570_PCR_PPROTCLR0_OFFSET)
#define TMS570_PCR_PPROTCLR1            (TMS570_PCR_BASE+TMS570_PCR_PPROTCLR1_OFFSET)
#define TMS570_PCR_PPROTCLR2            (TMS570_PCR_BASE+TMS570_PCR_PPROTCLR2_OFFSET)
#define TMS570_PCR_PPROTCLR3            (TMS570_PCR_BASE+TMS570_PCR_PPROTCLR3_OFFSET)
#define TMS570_PCR_PCSPWRDWNSET0        (TMS570_PCR_BASE+TMS570_PCR_PCSPWRDWNSET0_OFFSET)
#define TMS570_PCR_PCSPWRDWNSET1        (TMS570_PCR_BASE+TMS570_PCR_PCSPWRDWNSET1_OFFSET)
#define TMS570_PCR_PCSPWRDWNCLR0        (TMS570_PCR_BASE+TMS570_PCR_PCSPWRDWNCLR0_OFFSET)
#define TMS570_PCR_PCSPWRDWNCLR1        (TMS570_PCR_BASE+TMS570_PCR_PCSPWRDWNCLR1_OFFSET)
#define TMS570_PCR_PSPWRDWNSET0         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNSET0_OFFSET)
#define TMS570_PCR_PSPWRDWNSET1         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNSET1_OFFSET)
#define TMS570_PCR_PSPWRDWNSET2         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNSET2_OFFSET)
#define TMS570_PCR_PSPWRDWNSET3         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNSET3_OFFSET)
#define TMS570_PCR_PSPWRDWNCLR0         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNCLR0_OFFSET)
#define TMS570_PCR_PSPWRDWNCLR1         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNCLR1_OFFSET)
#define TMS570_PCR_PSPWRDWNCLR2         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNCLR2_OFFSET)
#define TMS570_PCR_PSPWRDWNCLR3         (TMS570_PCR_BASE+TMS570_PCR_PSPWRDWNCLR3_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/

/* Peripheral Memory Protection Set Register 0 */
#define PCR_PMPROTSET0_
/* Peripheral Memory Protection Set Register 1 */
#define PCR_PMPROTSET1_
/* Peripheral Memory Protection Clear Register 0 */
#define PCR_PMPROTCLR0_
/* Peripheral Memory Protection Clear Register 1 */
#define PCR_PMPROTCLR1_
/* Peripheral Protection Set Register 0 */
#define PCR_PPROTSET0_
/* Peripheral Protection Set Register 1 */
#define PCR_PPROTSET1_
/* Peripheral Protection Set Register 2 */
#define PCR_PPROTSET2_
/* Peripheral Protection Set Register 3 */
#define PCR_PPROTSET3_
/* Peripheral Protection Clear Register 0 */
#define PCR_PPROTCLR0_
/* Peripheral Protection Clear Register 1 */
#define PCR_PPROTCLR1_
/* Peripheral Protection Clear Register 2 */
#define PCR_PPROTCLR2_
/* Peripheral Protection Clear Register 3 */
#define PCR_PPROTCLR3_
/* Peripheral Memory Power-Down Set Register 0 */
#define PCR_PCSPWRDWNSET0_
/* Peripheral Memory Power-Down Set Register 1 */
#define PCR_PCSPWRDWNSET1_
/* Peripheral Memory Power-Down Clear Register 0 */
#define PCR_PCSPWRDWNCLR0_
/* Peripheral Memory Power-Down Clear Register 1 */
#define PCR_PCSPWRDWNCLR1_
/* Peripheral Power-Down Set Register 0 */
#define PCR_PSPWRDWNSET0_
/* Peripheral Power-Down Set Register 1 */
#define PCR_PSPWRDWNSET1_
/* Peripheral Power-Down Set Register 2 */
#define PCR_PSPWRDWNSET2_
/* Peripheral Power-Down Set Register 3 */
#define PCR_PSPWRDWNSET3_
/* Peripheral Power-Down Clear Register 0 */
#define PCR_PSPWRDWNCLR0_
/* Peripheral Power-Down Clear Register 1 */
#define PCR_PSPWRDWNCLR1_
/* Peripheral Power-Down Clear Register 2 */
#define PCR_PSPWRDWNCLR2_
/* Peripheral Power-Down Clear Register 3 */
#define PCR_PSPWRDWNCLR3_

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PCR_H */
