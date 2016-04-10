/************************************************************************************
 * arch/arm/src/sama5/chip/sam_aximx.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_AXIMX_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_AXIMX_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/sama5/chip.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* AXIMX Register Offsets ***********************************************************/

#define SAM_AXIMX_REMAP_OFFSET 0x0000 /* Remap Register */

/* AXIMX Register Addresses *********************************************************/

#define SAM_AXIMX_REMAP        (SAM_AXIMX_VSECTION+SAM_AXIMX_REMAP_OFFSET)

/* AXIMX Register Bit Definitions ***************************************************/

/* Remap Register
 *
 * Boot state:    ROM is seen at address 0x00000000
 * Remap State 0: SRAM is seen at address 0x00000000 (through AHB slave interface)
 *                instead of ROM.
 * Remap State 1: HEBI is seen at address 0x00000000 (through AHB slave interface)
 *                instead of ROM for external boot.
 */

#define AXIMX_REMAP_REMAP0     (1 << 0) /* Remap State 0 */

#ifdef ATSAMA5D3
#  define AXIMX_REMAP_REMAP1   (1 << 1) /* Remap State 1 */
#endif

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_AXIMX_H */
