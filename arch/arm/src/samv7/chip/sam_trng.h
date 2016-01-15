/************************************************************************************
 * arch/arm/src/samv7/chip/sam_trng.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_CHIP_SAM_TRNG_H
#define __ARCH_ARM_SRC_SAMV7_CHIP_SAM_TRNG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* TRNG Register Offsets ************************************************************/

#define SAM_TRNG_CR_OFFSET        0x0000 /* Control Register */
#define SAM_TRNG_IER_OFFSET       0x0010 /* Interrupt Enable Register */
#define SAM_TRNG_IDR_OFFSET       0x0014 /* Interrupt Disable Register */
#define SAM_TRNG_IMR_OFFSET       0x0018 /* Interrupt Mask Register */
#define SAM_TRNG_ISR_OFFSET       0x001c /* Interrupt Status Register */
#define SAM_TRNG_ODATA_OFFSET     0x0050 /* Output Data Register */

/* TRNG Register Addresses **********************************************************/

#define SAM_TRNG_CR               (SAM_TRNG_BASE+SAM_TRNG_CR_OFFSET)
#define SAM_TRNG_IER              (SAM_TRNG_BASE+SAM_TRNG_IER_OFFSET)
#define SAM_TRNG_IDR              (SAM_TRNG_BASE+SAM_TRNG_IDR_OFFSET)
#define SAM_TRNG_IMR              (SAM_TRNG_BASE+SAM_TRNG_IMR_OFFSET)
#define SAM_TRNG_ISR              (SAM_TRNG_BASE+SAM_TRNG_ISR_OFFSET)
#define SAM_TRNG_ODATA            (SAM_TRNG_BASE+SAM_TRNG_ODATA_OFFSET)

/* TRNG Register Bit Definitions ****************************************************/

/* Control Register */

#define TRNG_CR_ENABLE            (1 << 0)  /* Bit 0:  1=Enables the TRNG */
#  define TRNG_CR_DISABLE         (0)       /* Bit 0:  0=Disables the TRNG */
#define TRNG_CR_KEY_SHIFT         (8)       /* Bits 8-31: Security key */
#define TRNG_CR_KEY_MASK          (0xffffff << TRNG_CR_KEY_SHIFT)
# define TRNG_CR_KEY              (0x524e47 << TRNG_CR_KEY_SHIFT) /* RNG in ASCII */

/* Interrupt Enable Register, Interrupt Disable Register, Interrupt Mask Register,
 * and Interrupt Status Register
 */

#define TRNG_INT_DATRDY           (1 << 0)  /* Bit 0:  Data ready */

/* Output Data Register (32-bit output data) */

#endif /* __ARCH_ARM_SRC_SAMV7_CHIP_SAM_TRNG_H */
