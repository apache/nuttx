/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_dmamux.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_DMAMUX_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "hardware/rt102x/imxrt102x_dmamux.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "hardware/rt105x/imxrt105x_dmamux.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "hardware/rt106x/imxrt106x_dmamux.h"
#else
#  error Unrecognized i.MX RT architecture
#endif

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define IMXRT_DMAMUX_NCHAN             32

/* DMAMUX Register Offsets **************************************************/

#define IMXRT_DMAMUX_CHCFG_OFFSET(n)   ((uintptr_t)(n) << 2)
#  define IMXRT_DMAMUX_CHCFG0_OFFSET   0x0000  /* Channel configuration register 0 */
#  define IMXRT_DMAMUX_CHCFG1_OFFSET   0x0004  /* Channel configuration register 1 */
#  define IMXRT_DMAMUX_CHCFG2_OFFSET   0x0008  /* Channel configuration register 2 */
#  define IMXRT_DMAMUX_CHCFG3_OFFSET   0x000c  /* Channel configuration register 3 */
#  define IMXRT_DMAMUX_CHCFG4_OFFSET   0x0010  /* Channel configuration register 4 */
#  define IMXRT_DMAMUX_CHCFG5_OFFSET   0x0014  /* Channel configuration register 5 */
#  define IMXRT_DMAMUX_CHCFG6_OFFSET   0x0018  /* Channel configuration register 6 */
#  define IMXRT_DMAMUX_CHCFG7_OFFSET   0x001c  /* Channel configuration register 7 */
#  define IMXRT_DMAMUX_CHCFG8_OFFSET   0x0020  /* Channel configuration register 8 */
#  define IMXRT_DMAMUX_CHCFG9_OFFSET   0x0024  /* Channel configuration register 9 */
#  define IMXRT_DMAMUX_CHCFG10_OFFSET  0x0028  /* Channel configuration register 10 */
#  define IMXRT_DMAMUX_CHCFG11_OFFSET  0x002c  /* Channel configuration register 11 */
#  define IMXRT_DMAMUX_CHCFG12_OFFSET  0x0030  /* Channel configuration register 12 */
#  define IMXRT_DMAMUX_CHCFG13_OFFSET  0x0034  /* Channel configuration register 13 */
#  define IMXRT_DMAMUX_CHCFG14_OFFSET  0x0038  /* Channel configuration register 14 */
#  define IMXRT_DMAMUX_CHCFG15_OFFSET  0x003c  /* Channel configuration register 15 */
#  define IMXRT_DMAMUX_CHCFG16_OFFSET  0x0040  /* Channel configuration register 16 */
#  define IMXRT_DMAMUX_CHCFG17_OFFSET  0x0044  /* Channel configuration register 17 */
#  define IMXRT_DMAMUX_CHCFG18_OFFSET  0x0048  /* Channel configuration register 18 */
#  define IMXRT_DMAMUX_CHCFG19_OFFSET  0x004c  /* Channel configuration register 19 */
#  define IMXRT_DMAMUX_CHCFG20_OFFSET  0x0050  /* Channel configuration register 20 */
#  define IMXRT_DMAMUX_CHCFG21_OFFSET  0x0054  /* Channel configuration register 21 */
#  define IMXRT_DMAMUX_CHCFG22_OFFSET  0x0058  /* Channel configuration register 22 */
#  define IMXRT_DMAMUX_CHCFG23_OFFSET  0x005c  /* Channel configuration register 23 */
#  define IMXRT_DMAMUX_CHCFG24_OFFSET  0x0060  /* Channel configuration register 24 */
#  define IMXRT_DMAMUX_CHCFG25_OFFSET  0x0064  /* Channel configuration register 25 */
#  define IMXRT_DMAMUX_CHCFG26_OFFSET  0x0068  /* Channel configuration register 26 */
#  define IMXRT_DMAMUX_CHCFG27_OFFSET  0x006c  /* Channel configuration register 27 */
#  define IMXRT_DMAMUX_CHCFG28_OFFSET  0x0070  /* Channel configuration register 28 */
#  define IMXRT_DMAMUX_CHCFG29_OFFSET  0x0074  /* Channel configuration register 29 */
#  define IMXRT_DMAMUX_CHCFG30_OFFSET  0x0078  /* Channel configuration register 30 */
#  define IMXRT_DMAMUX_CHCFG31_OFFSET  0x007c  /* Channel configuration register 31 */

/* DMAMUX Register Addresses ************************************************/

#define IMXRT_DMAMUX_CHCFG(n)          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG_OFFSET(n))
#  define IMXRT_DMAMUX_CHCFG0          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG0_OFFSET)
#  define IMXRT_DMAMUX_CHCFG1          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG1_OFFSET)
#  define IMXRT_DMAMUX_CHCFG2          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG2_OFFSET)
#  define IMXRT_DMAMUX_CHCFG3          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG3_OFFSET)
#  define IMXRT_DMAMUX_CHCFG4          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG4_OFFSET)
#  define IMXRT_DMAMUX_CHCFG5          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG5_OFFSET)
#  define IMXRT_DMAMUX_CHCFG6          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG6_OFFSET)
#  define IMXRT_DMAMUX_CHCFG7          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG7_OFFSET)
#  define IMXRT_DMAMUX_CHCFG8          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG8_OFFSET)
#  define IMXRT_DMAMUX_CHCFG9          (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG9_OFFSET)
#  define IMXRT_DMAMUX_CHCFG10         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG10_OFFSET)
#  define IMXRT_DMAMUX_CHCFG11         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG11_OFFSET)
#  define IMXRT_DMAMUX_CHCFG12         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG12_OFFSET)
#  define IMXRT_DMAMUX_CHCFG13         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG13_OFFSET)
#  define IMXRT_DMAMUX_CHCFG14         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG14_OFFSET)
#  define IMXRT_DMAMUX_CHCFG15         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG15_OFFSET)
#  define IMXRT_DMAMUX_CHCFG16         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG16_OFFSET)
#  define IMXRT_DMAMUX_CHCFG17         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG17_OFFSET)
#  define IMXRT_DMAMUX_CHCFG18         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG18_OFFSET)
#  define IMXRT_DMAMUX_CHCFG19         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG19_OFFSET)
#  define IMXRT_DMAMUX_CHCFG20         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG20_OFFSET)
#  define IMXRT_DMAMUX_CHCFG21         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG21_OFFSET)
#  define IMXRT_DMAMUX_CHCFG22         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG22_OFFSET)
#  define IMXRT_DMAMUX_CHCFG23         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG23_OFFSET)
#  define IMXRT_DMAMUX_CHCFG24         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG24_OFFSET)
#  define IMXRT_DMAMUX_CHCFG25         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG25_OFFSET)
#  define IMXRT_DMAMUX_CHCFG26         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG26_OFFSET)
#  define IMXRT_DMAMUX_CHCFG27         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG27_OFFSET)
#  define IMXRT_DMAMUX_CHCFG28         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG28_OFFSET)
#  define IMXRT_DMAMUX_CHCFG29         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG29_OFFSET)
#  define IMXRT_DMAMUX_CHCFG30         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG30_OFFSET)
#  define IMXRT_DMAMUX_CHCFG31         (IMXRT_DMAMUX_BASE + IMXRT_DMAMUX_CHCFG31_OFFSET)

/* DMAMUX Bit-Field Definitions *********************************************/

/* Channel configuration registers 0-31 */

#define DMAMUX_CHCFG_SOURCE_SHIFT      (0)       /* Bits 0-6: Chip-specific DMA source */
#define DMAMUX_CHCFG_SOURCE_MASK       (0x7e << DMAMUX_CHCFG_SOURCE_SHIFT)
#  define DMAMUX_CHCFG_SOURCE(n)       ((uint32_t)(n) << DMAMUX_CHCFG_SOURCE_SHIFT)
                                                 /* Bits 7-28: Reserved */
#define DMAMUX_CHCFG_AON               (1 << 29) /* Bit 29: DMA Channel Always Enable */
#define DMAMUX_CHCFG_TRIG              (1 << 30) /* Bit 30: DMA Channel Trigger Enable */
#define DMAMUX_CHCFG_ENBL              (1 << 31) /* Bit 31: DMA Mux Channel Enable */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_DMAMUX_H */
