/************************************************************************************
 * arch/arm/src/sama5/chip/sam_sckc.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_SCKC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_SCKC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* SCKC Register Offsets ************************************************************/

#define SAM_SCKC_CR_OFFSET    0x0000 /* Slow Clock Controller Configuration Register  */

/* SCKC Register Addresses **********************************************************/

#define SAM_SCKC_CR           (SAM_SCKCR_VBASE+SAM_SCKC_CR_OFFSET)

/* SCKC Register Bit Definitions ****************************************************/

/* Slow Clock Controller Configuration Register */

#ifdef ATSAMA5D3
#  define SCKC_CR_RCEN        (1 << 0)  /* Bit 0: Internal 32 kHz RC Oscillator */
#  define SCKC_CR_OSC32EN     (1 << 1)  /* Bit 1: 32768 Hz Oscillator */
#  define SCKC_CR_OSC32BYP    (1 << 2)  /* Bit 2: 2768Hz Oscillator Bypass */
#endif

#define SCKC_CR_OSCSEL        (1 << 3)  /* Bit 3:  Slow Clock Selector */

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_SCKC_H */
