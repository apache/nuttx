/********************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_dmamux.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 *******************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_DMAMUX_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_DMAMUX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Number of DMA channels */

#if defined(CONFIG_ARCH_CHIP_S32K11X)
#  define S32K1XX_NDMACH  4
#elif defined(CONFIG_ARCH_CHIP_S32K14X
#  define S32K1XX_NDMACH  16
#endif

/* DMAMUX Register Offsets ******************************************************************/

#define S32K1XX_DMAMUX_CHCFG_OFFSET(n)  (n)  /* Channel Configuration register, n=1..15 */

/* DMAMUX Register Addresses ****************************************************************/

#define S32K1XX_DMAMUX_CHCFG(n)         (S32K1XX_DMAMUX_BASE + S32K1XX_DMAMUX_CHCFG_OFFSET(n)) n=1..15 */

/* DMAMUX Register Bitfield Definitions *****************************************************/

/* Channel Configuration register, n=1..15 */

#define DMAMUX_CHCFG_SOURCE_SHIFT       (0)       /* Bits 0-5:  DMA Channel Source (Slot) */
#define DMAMUX_CHCFG_SOURCE_MASK        (0x3f << DMAMUX_CHCFG_SOURCE_SHIFT)
#  define DMAMUX_CHCFG_SOURCE(s)        ((uint32_t)(s) << DMAMUX_CHCFG_SOURCE_SHIFT) /* chip-specific */
#define DMAMUX_CHCFG_TRIG               (1 << 6)  /* Bit 6:  DMA Channel Trigger Enable */
#define DMAMUX_CHCFG_ENBL               (1 << 7)  /* Bit 7:  DMA Channel Enable */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_DMAMUX_H */
