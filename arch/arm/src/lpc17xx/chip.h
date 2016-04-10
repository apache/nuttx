/************************************************************************************
 * arch/arm/src/lpc17xx/chip.h
 *
 *   Copyright (C) 2010-2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "nvic.h"

/* Include the chip capabilities file */

#include <arch/lpc17xx/chip.h>

/* If the common ARMv7-M vector handling logic is used, then include the
 * required vector definitions as well.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR
#  if defined(LPC176x)
#    include "chip/lpc176x_vectors.h"
#  elif defined(LPC178x)
#    include "chip/lpc178x_vectors.h"
#  else
#    error "No vector file for this LPC17xx family"
#  endif
#endif

/* Vector Table Offset Register (VECTAB).  Redefine the mask defined in
 * arch/arm/src/armv7-m/nvic.h; The LPC178x/7x User manual definitions
 * do not match the ARMv7M field definitions.  Any bits set above bit
 * 29 would be an error and apparently the register wants 8- not 6-bit
 * alignment.
 */

#undef  NVIC_VECTAB_TBLOFF_MASK
#define NVIC_VECTAB_TBLOFF_MASK         (0x3fffff00)

/* Include the memory map file.  Other chip hardware files should then include
 * this file for the proper setup.
 */

#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_H */
