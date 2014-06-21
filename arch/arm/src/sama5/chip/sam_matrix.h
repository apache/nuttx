/****************************************************************************************
 * arch/arm/src/sama5/chip/sam_matrix.h
 * Bux matrix definitions for the SAMA5
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_MATRIX_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_MATRIX_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

#ifdef ATSAMA5D4
#  define H64MX_DDR_SLAVE_PORT0          3

/* These are bits maps of PIDs in the H64MX SPSELR registers.  These are used by
 * application code to quickly determine if a given PID is served by H32MX or H64MX
 * which, in turn, is needed to know if the peripheral secured in SPSELR).
 * Reference: "In Matrix" column of "Table 9-1. Peripheral identifiers."
 *
 * NOTE that these hard-code bit values must match the PID assignments in
 * arch/arm/include/sama5/sama5*_irq.h.
 */

/* ARM=2, XDMAC0=8, CPKCC=10, AESB=13, MPDDRC=16, MATRIX0=18, VDEC=19 */

#  define H64MX_SPSELR0_PIDS             0x000d2504

/* XDMAC1=50, LCDC=51, ISI=52 */

#  define H64MX_SPSELR1_PIDS             0x001c0000

/* L2CC=67 */

#  define H64MX_SPSELR2_PIDS             0x00000008

#endif

/* MATRIX register offsets **************************************************************/

#define SAM_MATRIX_MCFG_OFFSET(n)        ((n)<<2)
#define SAM_MATRIX_MCFG0_OFFSET          0x0000 /* Master Configuration Register 0 */
#define SAM_MATRIX_MCFG1_OFFSET          0x0004 /* Master Configuration Register 1 */
#define SAM_MATRIX_MCFG2_OFFSET          0x0008 /* Master Configuration Register 2 */
#define SAM_MATRIX_MCFG3_OFFSET          0x000c /* Master Configuration Register 3 */
#define SAM_MATRIX_MCFG4_OFFSET          0x0010 /* Master Configuration Register 4 */
#define SAM_MATRIX_MCFG5_OFFSET          0x0014 /* Master Configuration Register 5 */
#define SAM_MATRIX_MCFG6_OFFSET          0x0018 /* Master Configuration Register 6 */
#define SAM_MATRIX_MCFG7_OFFSET          0x001c /* Master Configuration Register 7 */
#define SAM_MATRIX_MCFG8_OFFSET          0x0020 /* Master Configuration Register 8 */
#define SAM_MATRIX_MCFG9_OFFSET          0x0024 /* Master Configuration Register 9 */

#ifdef ATSAMA5D3
#  define SAM_MATRIX_MCFG10_OFFSET       0x0028 /* Master Configuration Register 10 */
#  define SAM_MATRIX_MCFG11_OFFSET       0x002c /* Master Configuration Register 11 */
#  define SAM_MATRIX_MCFG12_OFFSET       0x0030 /* Master Configuration Register 12 */
#  define SAM_MATRIX_MCFG13_OFFSET       0x0034 /* Master Configuration Register 13 */
#  define SAM_MATRIX_MCFG14_OFFSET       0x0038 /* Master Configuration Register 14 */
#  define SAM_MATRIX_MCFG15_OFFSET       0x003c /* Master Configuration Register 15 */
#endif

#define SAM_MATRIX_SCFG_OFFSET(n)        (0x0040+((n)<<2))
#define SAM_MATRIX_SCFG0_OFFSET          0x0040 /* Slave Configuration Register 0 */
#define SAM_MATRIX_SCFG1_OFFSET          0x0044 /* Slave Configuration Register 1 */
#define SAM_MATRIX_SCFG2_OFFSET          0x0048 /* Slave Configuration Register 2 */
#define SAM_MATRIX_SCFG3_OFFSET          0x004c /* Slave Configuration Register 3 */
#define SAM_MATRIX_SCFG4_OFFSET          0x0050 /* Slave Configuration Register 4 */
#define SAM_MATRIX_SCFG5_OFFSET          0x0054 /* Slave Configuration Register 5 */
#define SAM_MATRIX_SCFG6_OFFSET          0x0058 /* Slave Configuration Register 6 */
#define SAM_MATRIX_SCFG7_OFFSET          0x005c /* Slave Configuration Register 7 */
#define SAM_MATRIX_SCFG8_OFFSET          0x0060 /* Slave Configuration Register 8 */
#define SAM_MATRIX_SCFG9_OFFSET          0x0064 /* Slave Configuration Register 9 */
#define SAM_MATRIX_SCFG10_OFFSET         0x0068 /* Slave Configuration Register 10 */
#define SAM_MATRIX_SCFG11_OFFSET         0x006c /* Slave Configuration Register 11 */
#define SAM_MATRIX_SCFG12_OFFSET         0x0070 /* Slave Configuration Register 12 */

#ifdef ATSAMA5D3
#  define SAM_MATRIX_SCFG13_OFFSET       0x0074 /* Slave Configuration Register 13 */
#  define SAM_MATRIX_SCFG14_OFFSET       0x0078 /* Slave Configuration Register 14 */
#  define SAM_MATRIX_SCFG15_OFFSET       0x007c /* Slave Configuration Register 15 */
#endif

#define SAM_MATRIX_PRAS_OFFSET(n)        (0x0080+((n)<<3))
#define SAM_MATRIX_PRBS_OFFSET(n)        (0x0084+((n)<<3))
#define SAM_MATRIX_PRAS0_OFFSET          0x0080 /* Priority Register A for Slave 0 */
#define SAM_MATRIX_PRBS0_OFFSET          0x0084 /* Priority Register B for Slave 0 */
#define SAM_MATRIX_PRAS1_OFFSET          0x0088 /* Priority Register A for Slave 1 */
#define SAM_MATRIX_PRBS1_OFFSET          0x008c /* Priority Register B for Slave 1 */
#define SAM_MATRIX_PRAS2_OFFSET          0x0090 /* Priority Register A for Slave 2 */
#define SAM_MATRIX_PRBS2_OFFSET          0x0094 /* Priority Register B for Slave 2 */
#define SAM_MATRIX_PRAS3_OFFSET          0x0098 /* Priority Register A for Slave 3 */
#define SAM_MATRIX_PRBS3_OFFSET          0x009c /* Priority Register B for Slave 3 */
#define SAM_MATRIX_PRAS4_OFFSET          0x00a0 /* Priority Register A for Slave 4 */
#define SAM_MATRIX_PRBS4_OFFSET          0x00a4 /* Priority Register B for Slave 4 */
#define SAM_MATRIX_PRAS5_OFFSET          0x00a8 /* Priority Register A for Slave 5 */
#define SAM_MATRIX_PRBS5_OFFSET          0x00ac /* Priority Register B for Slave 5 */
#define SAM_MATRIX_PRAS6_OFFSET          0x00b0 /* Priority Register A for Slave 6 */
#define SAM_MATRIX_PRBS6_OFFSET          0x00b4 /* Priority Register B for Slave 6 */
#define SAM_MATRIX_PRAS7_OFFSET          0x00b8 /* Priority Register A for Slave 7 */
#define SAM_MATRIX_PRBS7_OFFSET          0x00bc /* Priority Register B for Slave 7 */
#define SAM_MATRIX_PRAS8_OFFSET          0x00c0 /* Priority Register A for Slave 8 */
#define SAM_MATRIX_PRBS8_OFFSET          0x00c4 /* Priority Register B for Slave 8 */
#define SAM_MATRIX_PRAS9_OFFSET          0x00c8 /* Priority Register A for Slave 9 */
#define SAM_MATRIX_PRBS9_OFFSET          0x00cc /* Priority Register B for Slave 9 */
#define SAM_MATRIX_PRAS10_OFFSET         0x00d0 /* Priority Register A for Slave 10 */
#define SAM_MATRIX_PRBS10_OFFSET         0x00d4 /* Priority Register B for Slave 10 */
#define SAM_MATRIX_PRAS11_OFFSET         0x00d8 /* Priority Register A for Slave 11 */
#define SAM_MATRIX_PRBS11_OFFSET         0x00dc /* Priority Register B for Slave 11 */
#define SAM_MATRIX_PRAS12_OFFSET         0x00e0 /* Priority Register A for Slave 12 */
#define SAM_MATRIX_PRBS12_OFFSET         0x00e4 /* Priority Register B for Slave 12 */

#ifdef ATSAMA5D3
#  define SAM_MATRIX_PRAS13_OFFSET       0x00e8 /* Priority Register A for Slave 13 */
#  define SAM_MATRIX_PRBS13_OFFSET       0x00ec /* Priority Register B for Slave 13 */
#  define SAM_MATRIX_PRAS14_OFFSET       0x00f0 /* Priority Register A for Slave 14 */
#  define SAM_MATRIX_PRBS14_OFFSET       0x00f4 /* Priority Register B for Slave 14 */
#  define SAM_MATRIX_PRAS15_OFFSET       0x00f8 /* Priority Register A for Slave 15 */
#  define SAM_MATRIX_PRBS15_OFFSET       0x00fc /* Priority Register B for Slave 15 */
#  define SAM_MATRIX_MRCR_OFFSET         0x0100 /* Master Remap Control Register */
#endif

#ifdef ATSAMA5D4
#  define SAM_MATRIX_MEIER_OFFSET        0x0150 /* Master Error Interrupt Enable Register */
#  define SAM_MATRIX_MEIDR_OFFSET        0x0154 /* Master Error Interrupt Disable Register */
#  define SAM_MATRIX_MEIMR_OFFSET        0x0158 /* Master Error Interrupt Mask Register */
#  define SAM_MATRIX_MESR_OFFSET         0x015c /* Master Error Status Register */

#  define SAM_MATRIX_MEAR0_OFFSET        0x0160 /* Master 0 Error Address Register */
#  define SAM_MATRIX_MEAR1_OFFSET        0x0164 /* Master 1 Error Address Register */
#  define SAM_MATRIX_MEAR2_OFFSET        0x0168 /* Master 2 Error Address Register */
#  define SAM_MATRIX_MEAR3_OFFSET        0x016c /* Master 3 Error Address Register */
#  define SAM_MATRIX_MEAR4_OFFSET        0x0170 /* Master 4 Error Address Register */
#  define SAM_MATRIX_MEAR5_OFFSET        0x0174 /* Master 5 Error Address Register */
#  define SAM_MATRIX_MEAR6_OFFSET        0x0178 /* Master 6 Error Address Register */
#  define SAM_MATRIX_MEAR7_OFFSET        0x017c /* Master 7 Error Address Register */
#  define SAM_MATRIX_MEAR8_OFFSET        0x0180 /* Master 8 Error Address Register */
#  define SAM_MATRIX_MEAR9_OFFSET        0x0184 /* Master 9 Error Address Register */
#endif

#define SAM_MATRIX_WPMR_OFFSET           0x01e4 /* Write Protect Mode Register */
#define SAM_MATRIX_WPSR_OFFSET           0x01e8 /* Write Protect Status Register */

#ifdef ATSAMA5D4
#  define SAM_MATRIX_SSR_OFFSET(n)       (0x0200+((n)<<2))
#  define SAM_MATRIX_SSR0_OFFSET         0x0200 /* Security Slave 0 Register */
#  define SAM_MATRIX_SSR1_OFFSET         0x0204 /* Security Slave 1 Register */
#  define SAM_MATRIX_SSR2_OFFSET         0x0208 /* Security Slave 2 Register */
#  define SAM_MATRIX_SSR3_OFFSET         0x020c /* Security Slave 3 Register */
#  define SAM_MATRIX_SSR4_OFFSET         0x0210 /* Security Slave 4 Register */
#  define SAM_MATRIX_SSR5_OFFSET         0x0214 /* Security Slave 5 Register */
#  define SAM_MATRIX_SSR6_OFFSET         0x0218 /* Security Slave 6 Register */
#  define SAM_MATRIX_SSR7_OFFSET         0x021c /* Security Slave 7 Register */
#  define SAM_MATRIX_SSR8_OFFSET         0x0220 /* Security Slave 8 Register */
#  define SAM_MATRIX_SSR9_OFFSET         0x0224 /* Security Slave 9 Register */
#  define SAM_MATRIX_SSR10_OFFSET        0x0228 /* Security Slave 10 Register */
#  define SAM_MATRIX_SSR11_OFFSET        0x022c /* Security Slave 11 Register */
#  define SAM_MATRIX_SSR12_OFFSET        0x0230 /* Security Slave 12 Register */

#  define SAM_MATRIX_SASSR_OFFSET(n)     (0x0240+((n)<<2))
#  define SAM_MATRIX_SASSR0_OFFSET       0x0240 /* Security Areas Split Slave 0 Register */
#  define SAM_MATRIX_SASSR1_OFFSET       0x0244 /* Security Areas Split Slave 1 Register */
#  define SAM_MATRIX_SASSR2_OFFSET       0x0248 /* Security Areas Split Slave 2 Register */
#  define SAM_MATRIX_SASSR3_OFFSET       0x024c /* Security Areas Split Slave 3 Register */
#  define SAM_MATRIX_SASSR4_OFFSET       0x0250 /* Security Areas Split Slave 4 Register */
#  define SAM_MATRIX_SASSR5_OFFSET       0x0254 /* Security Areas Split Slave 5 Register */
#  define SAM_MATRIX_SASSR6_OFFSET       0x0258 /* Security Areas Split Slave 6 Register */
#  define SAM_MATRIX_SASSR7_OFFSET       0x025c /* Security Areas Split Slave 7 Register */
#  define SAM_MATRIX_SASSR8_OFFSET       0x0260 /* Security Areas Split Slave 8 Register */
#  define SAM_MATRIX_SASSR9_OFFSET       0x0264 /* Security Areas Split Slave 9 Register */
#  define SAM_MATRIX_SASSR10_OFFSET      0x0268 /* Security Areas Split Slave 10 Register */
#  define SAM_MATRIX_SASSR11_OFFSET      0x026c /* Security Areas Split Slave 11 Register */
#  define SAM_MATRIX_SASSR12_OFFSET      0x0270 /* Security Areas Split Slave 12 Register */

#  define SAM_MATRIX_SRTSR_OFFSET(n)     (0x0280+((n)<<2))
#  define SAM_MATRIX_SRTSR1_OFFSET       0x0284 /* Security Region Top Slave 1 Register */
#  define SAM_MATRIX_SRTSR2_OFFSET       0x0288 /* Security Region Top Slave 2 Register */
#  define SAM_MATRIX_SRTSR3_OFFSET       0x028c /* Security Region Top Slave 3 Register */
#  define SAM_MATRIX_SRTSR4_OFFSET       0x0290 /* Security Region Top Slave 4 Register */
#  define SAM_MATRIX_SRTSR5_OFFSET       0x0294 /* Security Region Top Slave 5 Register */
#  define SAM_MATRIX_SRTSR6_OFFSET       0x0298 /* Security Region Top Slave 6 Register */
#  define SAM_MATRIX_SRTSR7_OFFSET       0x029c /* Security Region Top Slave 7 Register */
#  define SAM_MATRIX_SRTSR8_OFFSET       0x02a0 /* Security Region Top Slave 8 Register */
#  define SAM_MATRIX_SRTSR9_OFFSET       0x02a4 /* Security Region Top Slave 9 Register */
#  define SAM_MATRIX_SRTSR10_OFFSET      0x02a8 /* Security Region Top Slave 10 Register */
#  define SAM_MATRIX_SRTSR11_OFFSET      0x02ac /* Security Region Top Slave 11 Register */
#  define SAM_MATRIX_SRTSR12_OFFSET      0x02b0 /* Security Region Top Slave 12 Register */

#  define SAM_MATRIX_SPSELR_OFFSET(n)    (0x02c0 + ((n) << 2))
#  define SAM_MATRIX_SPSELR1_OFFSET      0x02c0 /* Security Peripheral Select 1 Register */
#  define SAM_MATRIX_SPSELR2_OFFSET      0x02c4 /* Security Peripheral Select 2 Register */
#  define SAM_MATRIX_SPSELR3_OFFSET      0x02c8 /* Security Peripheral Select 3 Register */
#endif

/* MATRIX register addresses ************************************************************/

#ifdef ATSAMA5D3
#  define SAM_MATRIX_MCFG(n))            (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG_OFFSET(n))
#  define SAM_MATRIX_MCFG0               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG0_OFFSET)
#  define SAM_MATRIX_MCFG1               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG1_OFFSET)
#  define SAM_MATRIX_MCFG2               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG2_OFFSET)
#  define SAM_MATRIX_MCFG3               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG3_OFFSET)
#  define SAM_MATRIX_MCFG4               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG4_OFFSET)
#  define SAM_MATRIX_MCFG5               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG5_OFFSET)
#  define SAM_MATRIX_MCFG6               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG6_OFFSET)
#  define SAM_MATRIX_MCFG7               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG7_OFFSET)
#  define SAM_MATRIX_MCFG8               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG8_OFFSET)
#  define SAM_MATRIX_MCFG9               (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG9_OFFSET)
#  define SAM_MATRIX_MCFG10              (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG10_OFFSET)
#  define SAM_MATRIX_MCFG11              (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG11_OFFSET)
#  define SAM_MATRIX_MCFG12              (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG12_OFFSET)
#  define SAM_MATRIX_MCFG13              (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG13_OFFSET)
#  define SAM_MATRIX_MCFG14              (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG14_OFFSET)
#  define SAM_MATRIX_MCFG15              (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG15_OFFSET)

#  define SAM_MATRIX_SCFG(n)             (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG_OFFSET(n))
#  define SAM_MATRIX_SCFG0               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG0_OFFSET)
#  define SAM_MATRIX_SCFG1               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG1_OFFSET)
#  define SAM_MATRIX_SCFG2               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG2_OFFSET)
#  define SAM_MATRIX_SCFG3               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG3_OFFSET)
#  define SAM_MATRIX_SCFG4               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG4_OFFSET)
#  define SAM_MATRIX_SCFG5               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG5_OFFSET)
#  define SAM_MATRIX_SCFG6               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG6_OFFSET)
#  define SAM_MATRIX_SCFG7               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG7_OFFSET)
#  define SAM_MATRIX_SCFG8               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG8_OFFSET)
#  define SAM_MATRIX_SCFG9               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG9_OFFSET)
#  define SAM_MATRIX_SCFG10              (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG10_OFFSET)
#  define SAM_MATRIX_SCFG11              (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG11_OFFSET)
#  define SAM_MATRIX_SCFG12              (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG12_OFFSET)
#  define SAM_MATRIX_SCFG13              (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG13_OFFSET)
#  define SAM_MATRIX_SCFG14              (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG14_OFFSET)
#  define SAM_MATRIX_SCFG15              (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG15_OFFSET)

#  define SAM_MATRIX_PRAS(n)             (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS_OFFSET(n))
#  define SAM_MATRIX_PRBS(n)             (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS_OFFSET(n))
#  define SAM_MATRIX_PRAS0               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS0_OFFSET)
#  define SAM_MATRIX_PRBS0               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS0_OFFSET)
#  define SAM_MATRIX_PRAS1               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS1_OFFSET)
#  define SAM_MATRIX_PRBS1               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS1_OFFSET)
#  define SAM_MATRIX_PRAS2               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS2_OFFSET)
#  define SAM_MATRIX_PRBS2               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS2_OFFSET)
#  define SAM_MATRIX_PRAS3               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS3_OFFSET)
#  define SAM_MATRIX_PRBS3               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS3_OFFSET)
#  define SAM_MATRIX_PRAS4               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS4_OFFSET)
#  define SAM_MATRIX_PRBS4               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS4_OFFSET)
#  define SAM_MATRIX_PRAS5               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS5_OFFSET)
#  define SAM_MATRIX_PRBS5               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS5_OFFSET)
#  define SAM_MATRIX_PRAS6               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS6_OFFSET)
#  define SAM_MATRIX_PRBS6               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS6_OFFSET)
#  define SAM_MATRIX_PRAS7               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS7_OFFSET)
#  define SAM_MATRIX_PRBS7               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS7_OFFSET)
#  define SAM_MATRIX_PRAS8               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS8_OFFSET)
#  define SAM_MATRIX_PRBS8               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS8_OFFSET)
#  define SAM_MATRIX_PRAS9               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS9_OFFSET)
#  define SAM_MATRIX_PRBS9               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS9_OFFSET)
#  define SAM_MATRIX_PRAS10              (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS10_OFFSET)
#  define SAM_MATRIX_PRBS10              (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS10_OFFSET)
#  define SAM_MATRIX_PRAS11              (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS11_OFFSET)
#  define SAM_MATRIX_PRBS11              (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS11_OFFSET)
#  define SAM_MATRIX_PRAS12              (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS12_OFFSET)
#  define SAM_MATRIX_PRBS12              (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS12_OFFSET)

#  define SAM_MATRIX_PRAS13              (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS13_OFFSET)
#  define SAM_MATRIX_PRBS13              (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS13_OFFSET)
#  define SAM_MATRIX_PRAS14              (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS14_OFFSET)
#  define SAM_MATRIX_PRBS14              (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS14_OFFSET)
#  define SAM_MATRIX_PRAS15              (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS15_OFFSET)
#  define SAM_MATRIX_PRBS15              (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS15_OFFSET)
#  define SAM_MATRIX_MRCR                (SAM_MATRIX_VBASE+SAM_MATRIX_MRCR_OFFSET)

#  define SAM_MATRIX_WPMR                (SAM_MATRIX_VBASE+SAM_MATRIX_WPMR_OFFSET)
#  define SAM_MATRIX_WPSR                (SAM_MATRIX_VBASE+SAM_MATRIX_WPSR_OFFSET)

#endif /* ATSAMA5D3 */

#ifdef ATSAMA5D4
/* HMATRIX0 (H64MX) */

#  define SAM_MATRIX0_MCFG(n))           (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG_OFFSET(n))
#  define SAM_MATRIX0_MCFG0              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG0_OFFSET)
#  define SAM_MATRIX0_MCFG1              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG1_OFFSET)
#  define SAM_MATRIX0_MCFG2              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG2_OFFSET)
#  define SAM_MATRIX0_MCFG3              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG3_OFFSET)
#  define SAM_MATRIX0_MCFG4              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG4_OFFSET)
#  define SAM_MATRIX0_MCFG5              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG5_OFFSET)
#  define SAM_MATRIX0_MCFG6              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG6_OFFSET)
#  define SAM_MATRIX0_MCFG7              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG7_OFFSET)
#  define SAM_MATRIX0_MCFG8              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG8_OFFSET)
#  define SAM_MATRIX0_MCFG9              (SAM_MATRIX64_VBASE+SAM_MATRIX_MCFG9_OFFSET)

#  define SAM_MATRIX0_SCFG(n)            (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG_OFFSET(n))
#  define SAM_MATRIX0_SCFG0              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG0_OFFSET)
#  define SAM_MATRIX0_SCFG1              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG1_OFFSET)
#  define SAM_MATRIX0_SCFG2              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG2_OFFSET)
#  define SAM_MATRIX0_SCFG3              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG3_OFFSET)
#  define SAM_MATRIX0_SCFG4              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG4_OFFSET)
#  define SAM_MATRIX0_SCFG5              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG5_OFFSET)
#  define SAM_MATRIX0_SCFG6              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG6_OFFSET)
#  define SAM_MATRIX0_SCFG7              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG7_OFFSET)
#  define SAM_MATRIX0_SCFG8              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG8_OFFSET)
#  define SAM_MATRIX0_SCFG9              (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG9_OFFSET)
#  define SAM_MATRIX0_SCFG10             (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG10_OFFSET)
#  define SAM_MATRIX0_SCFG11             (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG11_OFFSET)
#  define SAM_MATRIX0_SCFG12             (SAM_MATRIX64_VBASE+SAM_MATRIX_SCFG12_OFFSET)

#  define SAM_MATRIX0_PRAS(n)            (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS_OFFSET(n))
#  define SAM_MATRIX0_PRBS(n)            (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS_OFFSET(n))
#  define SAM_MATRIX0_PRAS0              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS0_OFFSET)
#  define SAM_MATRIX0_PRBS0              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS0_OFFSET)
#  define SAM_MATRIX0_PRAS1              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS1_OFFSET)
#  define SAM_MATRIX0_PRBS1              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS1_OFFSET)
#  define SAM_MATRIX0_PRAS2              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS2_OFFSET)
#  define SAM_MATRIX0_PRBS2              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS2_OFFSET)
#  define SAM_MATRIX0_PRAS3              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS3_OFFSET)
#  define SAM_MATRIX0_PRBS3              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS3_OFFSET)
#  define SAM_MATRIX0_PRAS4              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS4_OFFSET)
#  define SAM_MATRIX0_PRBS4              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS4_OFFSET)
#  define SAM_MATRIX0_PRAS5              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS5_OFFSET)
#  define SAM_MATRIX0_PRBS5              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS5_OFFSET)
#  define SAM_MATRIX0_PRAS6              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS6_OFFSET)
#  define SAM_MATRIX0_PRBS6              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS6_OFFSET)
#  define SAM_MATRIX0_PRAS7              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS7_OFFSET)
#  define SAM_MATRIX0_PRBS7              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS7_OFFSET)
#  define SAM_MATRIX0_PRAS8              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS8_OFFSET)
#  define SAM_MATRIX0_PRBS8              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS8_OFFSET)
#  define SAM_MATRIX0_PRAS9              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS9_OFFSET)
#  define SAM_MATRIX0_PRBS9              (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS9_OFFSET)
#  define SAM_MATRIX0_PRAS10             (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS10_OFFSET)
#  define SAM_MATRIX0_PRBS10             (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS10_OFFSET)
#  define SAM_MATRIX0_PRAS11             (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS11_OFFSET)
#  define SAM_MATRIX0_PRBS11             (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS11_OFFSET)
#  define SAM_MATRIX0_PRAS12             (SAM_MATRIX64_VBASE+SAM_MATRIX_PRAS12_OFFSET)
#  define SAM_MATRIX0_PRBS12             (SAM_MATRIX64_VBASE+SAM_MATRIX_PRBS12_OFFSET)

#  define SAM_MATRIX0_MEIER              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEIER_OFFSET)
#  define SAM_MATRIX0_MEIDR              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEIDR_OFFSET
#  define SAM_MATRIX0_MEIMR              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEIMR_OFFSET)
#  define SAM_MATRIX0_MESR               (SAM_MATRIX64_VBASE+SAM_MATRIX_MESR_OFFSET)

#  define SAM_MATRIX0_MEAR0              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR0_OFFSET)
#  define SAM_MATRIX0_MEAR1              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR1_OFFSET)
#  define SAM_MATRIX0_MEAR2              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR2_OFFSET)
#  define SAM_MATRIX0_MEAR3              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR3_OFFSET)
#  define SAM_MATRIX0_MEAR4              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR4_OFFSET)
#  define SAM_MATRIX0_MEAR5              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR5_OFFSET)
#  define SAM_MATRIX0_MEAR6              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR6_OFFSET)
#  define SAM_MATRIX0_MEAR7              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR7_OFFSET)
#  define SAM_MATRIX0_MEAR8              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR8_OFFSET)
#  define SAM_MATRIX0_MEAR9              (SAM_MATRIX64_VBASE+SAM_MATRIX_MEAR9_OFFSET)

#  define SAM_MATRIX0_WPMR               (SAM_MATRIX64_VBASE+SAM_MATRIX_WPMR_OFFSET)
#  define SAM_MATRIX0_WPSR               (SAM_MATRIX64_VBASE+SAM_MATRIX_WPSR_OFFSET)

#  define SAM_MATRIX0_SSR(n)             (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR_OFFSET(n))
#  define SAM_MATRIX0_SSR0               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR0_OFFSET)
#  define SAM_MATRIX0_SSR1               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR1_OFFSET)
#  define SAM_MATRIX0_SSR2               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR2_OFFSET)
#  define SAM_MATRIX0_SSR3               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR3_OFFSET)
#  define SAM_MATRIX0_SSR4               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR4_OFFSET)
#  define SAM_MATRIX0_SSR5               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR5_OFFSET)
#  define SAM_MATRIX0_SSR6               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR6_OFFSET)
#  define SAM_MATRIX0_SSR7               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR7_OFFSET)
#  define SAM_MATRIX0_SSR8               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR8_OFFSET)
#  define SAM_MATRIX0_SSR9               (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR9_OFFSET)
#  define SAM_MATRIX0_SSR10              (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR10_OFFSET)
#  define SAM_MATRIX0_SSR11              (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR11_OFFSET)
#  define SAM_MATRIX0_SSR12              (SAM_MATRIX64_VBASE+SAM_MATRIX_SSR12_OFFSET)

#  define SAM_MATRIX0_SASSR(n)           (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR_OFFSET(n))
#  define SAM_MATRIX0_SASSR0             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR0_OFFSET)
#  define SAM_MATRIX0_SASSR1             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR1_OFFSET)
#  define SAM_MATRIX0_SASSR2             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR2_OFFSET)
#  define SAM_MATRIX0_SASSR3             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR3_OFFSET)
#  define SAM_MATRIX0_SASSR4             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR4_OFFSET)
#  define SAM_MATRIX0_SASSR5             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR5_OFFSET)
#  define SAM_MATRIX0_SASSR6             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR6_OFFSET)
#  define SAM_MATRIX0_SASSR7             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR7_OFFSET)
#  define SAM_MATRIX0_SASSR8             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR8_OFFSET)
#  define SAM_MATRIX0_SASSR9             (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR9_OFFSET)
#  define SAM_MATRIX0_SASSR10            (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR10_OFFSET)
#  define SAM_MATRIX0_SASSR11            (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR11_OFFSET)
#  define SAM_MATRIX0_SASSR12            (SAM_MATRIX64_VBASE+SAM_MATRIX_SASSR12_OFFSET)

#  define SAM_MATRIX0_SRTSR(n)           (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR_OFFSET(n))
#  define SAM_MATRIX0_SRTSR1             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR1_OFFSET)
#  define SAM_MATRIX0_SRTSR2             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR2_OFFSET)
#  define SAM_MATRIX0_SRTSR3             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR3_OFFSET)
#  define SAM_MATRIX0_SRTSR4             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR4_OFFSET)
#  define SAM_MATRIX0_SRTSR5             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR5_OFFSET)
#  define SAM_MATRIX0_SRTSR6             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR6_OFFSET)
#  define SAM_MATRIX0_SRTSR7             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR7_OFFSET)
#  define SAM_MATRIX0_SRTSR8             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR8_OFFSET)
#  define SAM_MATRIX0_SRTSR9             (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR9_OFFSET)
#  define SAM_MATRIX0_SRTSR10            (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR10_OFFSET)
#  define SAM_MATRIX0_SRTSR11            (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR11_OFFSET)
#  define SAM_MATRIX0_SRTSR12            (SAM_MATRIX64_VBASE+SAM_MATRIX_SRTSR12_OFFSET)

#  define SAM_MATRIX0_SPSELR(n)          (SAM_MATRIX64_VBASE+SAM_MATRIX_SPSELR_OFFSET(n))
#  define SAM_MATRIX0_SPSELR1            (SAM_MATRIX64_VBASE+SAM_MATRIX_SPSELR1_OFFSET)
#  define SAM_MATRIX0_SPSELR2            (SAM_MATRIX64_VBASE+SAM_MATRIX_SPSELR2_OFFSET)
#  define SAM_MATRIX0_SPSELR3            (SAM_MATRIX64_VBASE+SAM_MATRIX_SPSELR3_OFFSET)

/* HMATRIX 1 (H32MX) */

#  define SAM_MATRIX1_MCFG(n))           (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG_OFFSET(n))
#  define SAM_MATRIX1_MCFG0              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG0_OFFSET)
#  define SAM_MATRIX1_MCFG1              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG1_OFFSET)
#  define SAM_MATRIX1_MCFG2              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG2_OFFSET)
#  define SAM_MATRIX1_MCFG3              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG3_OFFSET)
#  define SAM_MATRIX1_MCFG4              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG4_OFFSET)
#  define SAM_MATRIX1_MCFG5              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG5_OFFSET)
#  define SAM_MATRIX1_MCFG6              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG6_OFFSET)
#  define SAM_MATRIX1_MCFG7              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG7_OFFSET)
#  define SAM_MATRIX1_MCFG8              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG8_OFFSET)
#  define SAM_MATRIX1_MCFG9              (SAM_MATRIX32_VBASE+SAM_MATRIX_MCFG9_OFFSET)

#  define SAM_MATRIX1_SCFG(n)            (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG_OFFSET(n))
#  define SAM_MATRIX1_SCFG0              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG0_OFFSET)
#  define SAM_MATRIX1_SCFG1              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG1_OFFSET)
#  define SAM_MATRIX1_SCFG2              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG2_OFFSET)
#  define SAM_MATRIX1_SCFG3              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG3_OFFSET)
#  define SAM_MATRIX1_SCFG4              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG4_OFFSET)
#  define SAM_MATRIX1_SCFG5              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG5_OFFSET)
#  define SAM_MATRIX1_SCFG6              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG6_OFFSET)
#  define SAM_MATRIX1_SCFG7              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG7_OFFSET)
#  define SAM_MATRIX1_SCFG8              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG8_OFFSET)
#  define SAM_MATRIX1_SCFG9              (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG9_OFFSET)
#  define SAM_MATRIX1_SCFG10             (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG10_OFFSET)
#  define SAM_MATRIX1_SCFG11             (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG11_OFFSET)
#  define SAM_MATRIX1_SCFG12             (SAM_MATRIX32_VBASE+SAM_MATRIX_SCFG12_OFFSET)

#  define SAM_MATRIX1_PRAS(n)            (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS_OFFSET(n))
#  define SAM_MATRIX1_PRBS(n)            (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS_OFFSET(n))
#  define SAM_MATRIX1_PRAS0              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS0_OFFSET)
#  define SAM_MATRIX1_PRBS0              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS0_OFFSET)
#  define SAM_MATRIX1_PRAS1              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS1_OFFSET)
#  define SAM_MATRIX1_PRBS1              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS1_OFFSET)
#  define SAM_MATRIX1_PRAS2              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS2_OFFSET)
#  define SAM_MATRIX1_PRBS2              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS2_OFFSET)
#  define SAM_MATRIX1_PRAS3              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS3_OFFSET)
#  define SAM_MATRIX1_PRBS3              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS3_OFFSET)
#  define SAM_MATRIX1_PRAS4              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS4_OFFSET)
#  define SAM_MATRIX1_PRBS4              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS4_OFFSET)
#  define SAM_MATRIX1_PRAS5              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS5_OFFSET)
#  define SAM_MATRIX1_PRBS5              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS5_OFFSET)
#  define SAM_MATRIX1_PRAS6              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS6_OFFSET)
#  define SAM_MATRIX1_PRBS6              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS6_OFFSET)
#  define SAM_MATRIX1_PRAS7              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS7_OFFSET)
#  define SAM_MATRIX1_PRBS7              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS7_OFFSET)
#  define SAM_MATRIX1_PRAS8              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS8_OFFSET)
#  define SAM_MATRIX1_PRBS8              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS8_OFFSET)
#  define SAM_MATRIX1_PRAS9              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS9_OFFSET)
#  define SAM_MATRIX1_PRBS9              (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS9_OFFSET)
#  define SAM_MATRIX1_PRAS10             (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS10_OFFSET)
#  define SAM_MATRIX1_PRBS10             (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS10_OFFSET)
#  define SAM_MATRIX1_PRAS11             (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS11_OFFSET)
#  define SAM_MATRIX1_PRBS11             (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS11_OFFSET)
#  define SAM_MATRIX1_PRAS12             (SAM_MATRIX32_VBASE+SAM_MATRIX_PRAS12_OFFSET)
#  define SAM_MATRIX1_PRBS12             (SAM_MATRIX32_VBASE+SAM_MATRIX_PRBS12_OFFSET)

#  define SAM_MATRIX1_MEIER              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEIER_OFFSET)
#  define SAM_MATRIX1_MEIDR              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEIDR_OFFSET
#  define SAM_MATRIX1_MEIMR              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEIMR_OFFSET)
#  define SAM_MATRIX1_MESR               (SAM_MATRIX32_VBASE+SAM_MATRIX_MESR_OFFSET)

#  define SAM_MATRIX1_MEAR0              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR0_OFFSET)
#  define SAM_MATRIX1_MEAR1              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR1_OFFSET)
#  define SAM_MATRIX1_MEAR2              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR2_OFFSET)
#  define SAM_MATRIX1_MEAR3              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR3_OFFSET)
#  define SAM_MATRIX1_MEAR4              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR4_OFFSET)
#  define SAM_MATRIX1_MEAR5              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR5_OFFSET)
#  define SAM_MATRIX1_MEAR6              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR6_OFFSET)
#  define SAM_MATRIX1_MEAR7              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR7_OFFSET)
#  define SAM_MATRIX1_MEAR8              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR8_OFFSET)
#  define SAM_MATRIX1_MEAR9              (SAM_MATRIX32_VBASE+SAM_MATRIX_MEAR9_OFFSET)

#  define SAM_MATRIX1_WPMR               (SAM_MATRIX32_VBASE+SAM_MATRIX_WPMR_OFFSET)
#  define SAM_MATRIX1_WPSR               (SAM_MATRIX32_VBASE+SAM_MATRIX_WPSR_OFFSET)

#  define SAM_MATRIX1_SSR(n)             (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR_OFFSET(n))
#  define SAM_MATRIX1_SSR0               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR0_OFFSET)
#  define SAM_MATRIX1_SSR1               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR1_OFFSET)
#  define SAM_MATRIX1_SSR2               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR2_OFFSET)
#  define SAM_MATRIX1_SSR3               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR3_OFFSET)
#  define SAM_MATRIX1_SSR4               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR4_OFFSET)
#  define SAM_MATRIX1_SSR5               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR5_OFFSET)
#  define SAM_MATRIX1_SSR6               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR6_OFFSET)
#  define SAM_MATRIX1_SSR7               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR7_OFFSET)
#  define SAM_MATRIX1_SSR8               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR8_OFFSET)
#  define SAM_MATRIX1_SSR9               (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR9_OFFSET)
#  define SAM_MATRIX1_SSR10              (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR10_OFFSET)
#  define SAM_MATRIX1_SSR11              (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR11_OFFSET)
#  define SAM_MATRIX1_SSR12              (SAM_MATRIX32_VBASE+SAM_MATRIX_SSR12_OFFSET)

#  define SAM_MATRIX1_SASSR(n)           (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR_OFFSET(n))
#  define SAM_MATRIX1_SASSR0             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR0_OFFSET)
#  define SAM_MATRIX1_SASSR1             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR1_OFFSET)
#  define SAM_MATRIX1_SASSR2             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR2_OFFSET)
#  define SAM_MATRIX1_SASSR3             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR3_OFFSET)
#  define SAM_MATRIX1_SASSR4             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR4_OFFSET)
#  define SAM_MATRIX1_SASSR5             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR5_OFFSET)
#  define SAM_MATRIX1_SASSR6             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR6_OFFSET)
#  define SAM_MATRIX1_SASSR7             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR7_OFFSET)
#  define SAM_MATRIX1_SASSR8             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR8_OFFSET)
#  define SAM_MATRIX1_SASSR9             (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR9_OFFSET)
#  define SAM_MATRIX1_SASSR10            (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR10_OFFSET)
#  define SAM_MATRIX1_SASSR11            (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR11_OFFSET)
#  define SAM_MATRIX1_SASSR12            (SAM_MATRIX32_VBASE+SAM_MATRIX_SASSR12_OFFSET)

#  define SAM_MATRIX1_SRTSR(n)           (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR_OFFSET(n))
#  define SAM_MATRIX1_SRTSR1             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR1_OFFSET)
#  define SAM_MATRIX1_SRTSR2             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR2_OFFSET)
#  define SAM_MATRIX1_SRTSR3             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR3_OFFSET)
#  define SAM_MATRIX1_SRTSR4             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR4_OFFSET)
#  define SAM_MATRIX1_SRTSR5             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR5_OFFSET)
#  define SAM_MATRIX1_SRTSR6             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR6_OFFSET)
#  define SAM_MATRIX1_SRTSR7             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR7_OFFSET)
#  define SAM_MATRIX1_SRTSR8             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR8_OFFSET)
#  define SAM_MATRIX1_SRTSR9             (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR9_OFFSET)
#  define SAM_MATRIX1_SRTSR10            (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR10_OFFSET)
#  define SAM_MATRIX1_SRTSR11            (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR11_OFFSET)
#  define SAM_MATRIX1_SRTSR12            (SAM_MATRIX32_VBASE+SAM_MATRIX_SRTSR12_OFFSET)

#  define SAM_MATRIX1_SPSELR(n)          (SAM_MATRIX32_VBASE+SAM_MATRIX_SPSELR_OFFSET(n))
#  define SAM_MATRIX1_SPSELR1            (SAM_MATRIX32_VBASE+SAM_MATRIX_SPSELR1_OFFSET)
#  define SAM_MATRIX1_SPSELR2            (SAM_MATRIX32_VBASE+SAM_MATRIX_SPSELR2_OFFSET)
#  define SAM_MATRIX1_SPSELR3            (SAM_MATRIX32_VBASE+SAM_MATRIX_SPSELR3_OFFSET)

#endif /* ATSAMA5D4 */

/* MATRIX register bit definitions ******************************************************/
/* Master Configuration Registers */

#define MATRIX_MCFG_ULBT_SHIFT           (0)       /* Bits 0-2:  Undefined Length Burst Type */
#define MATRIX_MCFG_ULBT_MASK            (7 << MATRIX_MCFG_ULBT_SHIFT)
#  define MATRIX_MCFG_ULBT_INF           (0 << MATRIX_MCFG_ULBT_SHIFT) /* Infinite Length Burst */
#  define MATRIX_MCFG_ULBT_SINGLE        (1 << MATRIX_MCFG_ULBT_SHIFT) /* Single Access */
#  define MATRIX_MCFG_ULBT_4BEAT         (2 << MATRIX_MCFG_ULBT_SHIFT) /* 4-Beat Burst */
#  define MATRIX_MCFG_ULBT_8BEAT         (3 << MATRIX_MCFG_ULBT_SHIFT) /* 8-Beat Burst */
#  define MATRIX_MCFG_ULBT_16BEAT        (4 << MATRIX_MCFG_ULBT_SHIFT) /* 16-Beat Burst */
#  define MATRIX_MCFG_ULBT_32BEAT        (5 << MATRIX_MCFG_ULBT_SHIFT) /* 32-Beat Burst */
#  define MATRIX_MCFG_ULBT_64BEAT        (6 << MATRIX_MCFG_ULBT_SHIFT) /* 64-Beat Burst */
#  define MATRIX_MCFG_ULBT_128BEAT       (7 << MATRIX_MCFG_ULBT_SHIFT) /* 128-Beat Burst */

/* Bus Matrix Slave Configuration Registers */

#define MATRIX_SCFG_SLOTCYCLE_SHIFT      (0)       /* Bits 0-8:  Maximum Number of Allowed Cycles for a Burst */
#define MATRIX_SCFG_SLOTCYCLE_MASK       (0x1ff << MATRIX_SCFG_SLOTCYCLE_SHIFT)
#  define MATRIX_SCFG_SLOTCYCLE(n)       ((uint32_t)(n) << MATRIX_SCFG_SLOTCYCLE_SHIFT)
#define MATRIX_SCFG_DEFMSTRTYPE_SHIFT    (16)      /* Bits 16-17:  Default Master Type */
#define MATRIX_SCFG_DEFMSTRTYPE_MASK     (3 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_NONE   (0 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_LAST   (1 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_FIXED  (2 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#define MATRIX_SCFG_FIXEDDEFMSTR_SHIFT   (18)      /* Bits 18-21:   Fixed Default Master */
#define MATRIX_SCFG_FIXEDDEFMSTR_MASK    (15 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG_FIXEDDEFMSTR(n)    ((uint32_t)(n) << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)

/* Bus Matrix Priority Registers A For Slaves */

#define MATRIX_PRAS_MPR_SHIFT(n)         ((n)<<2)
#define MATRIX_PRAS_MPR_MASK(n)          (3 << MATRIX_PRAS_MPR_SHIFT(n))
#  define MATRIX_PRAS_MPR(n,v)           ((uint32_t)(v) << MATRIX_PRAS_MPR_SHIFT(n))
#  define MATRIX_PRAS_M0PR_SHIFT         (0)       /* Bits 0-1:  Master 0 Priority */
#  define MATRIX_PRAS_M0PR_MASK          (3 << MATRIX_PRAS_M0PR_SHIFT)
#    define MATRIX_PRAS_M0PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M0PR_SHIFT)
#  define MATRIX_PRAS_M1PR_SHIFT         (4)       /* Bits 4-5:  Master 1 Priority */
#  define MATRIX_PRAS_M1PR_MASK          (3 << MATRIX_PRAS_M1PR_SHIFT)
#    define MATRIX_PRAS_M1PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M1PR_SHIFT)
#  define MATRIX_PRAS_M2PR_SHIFT         (8)       /* Bits 8-9:  Master 2 Priority */
#  define MATRIX_PRAS_M2PR_MASK          (3 << MATRIX_PRAS_M2PR_SHIFT)
#    define MATRIX_PRAS_M2PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M2PR_SHIFT)
#  define MATRIX_PRAS_M3PR_SHIFT         (12)      /* Bits 12-13: Master 3 Priority */
#  define MATRIX_PRAS_M3PR_MASK          (3 << MATRIX_PRAS_M3PR_SHIFT)
#    define MATRIX_PRAS_M3PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M3PR_SHIFT)
#  define MATRIX_PRAS_M4PR_SHIFT         (16)      /* Bits 16-17: Master 4 Priority */
#  define MATRIX_PRAS_M4PR_MASK          (3 << MATRIX_PRAS_M4PR_SHIFT)
#    define MATRIX_PRAS_M4PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M4PR_SHIFT)
#  define MATRIX_PRAS_M5PR_SHIFT         (20)      /* Bits 20-21: Master 5 Priority */
#  define MATRIX_PRAS_M5PR_MASK          (3 << MATRIX_PRAS_M5PR_SHIFT)
#    define MATRIX_PRAS_M5PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M5PR_SHIFT)
#  define MATRIX_PRAS_M6PR_SHIFT         (24)      /* Bits 24-25: Master 6 Priority */
#  define MATRIX_PRAS_M6PR_MASK          (3 << MATRIX_PRAS_M6PR_SHIFT)
#    define MATRIX_PRAS_M6PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M6PR_SHIFT)
#  define MATRIX_PRAS_M7PR_SHIFT         (28)      /* Bits 28-29: Master 7 Priority */
#  define MATRIX_PRAS_M7PR_MASK          (3 << MATRIX_PRAS_M7PR_SHIFT)
#    define MATRIX_PRAS_M7PR(n)          (((uint32_t)(v) << MATRIX_PRAS_M7PR_SHIFT)

/* Bus Matrix Priority Registers B For Slaves */

#define MATRIX_PRBS_MPR_SHIFT(n)         ((n)<<2)
#define MATRIX_PRBS_MPR_MASK(n)          (3 << MATRIX_PRBS_MPR_SHIFT(n))
#  define MATRIX_PRBS_MPR(n,v)           ((uint32_t)(v) << MATRIX_PRBS_MPR_SHIFT(n))
#  define MATRIX_PRBS_M8PR_SHIFT         (0)       /* Bits 0-1:  Master 8 Priority */
#  define MATRIX_PRBS_M8PR_MASK          (3 << MATRIX_PRBS_M8PR_SHIFT)
#    define MATRIX_PRBS_M8PR(n)          ((uint32_t)(n) << MATRIX_PRBS_M8PR_SHIFT)
#  define MATRIX_PRBS_M9PR_SHIFT         (4)       /* Bits 4-5:  Master 9 Priority */
#  define MATRIX_PRBS_M9PR_MASK          (3 << MATRIX_PRBS_M9PR_SHIFT)
#    define MATRIX_PRBS_M9PR(n)          ((uint32_t)(n) << MATRIX_PRBS_M9PR_SHIFT)

#ifdef ATSAMA5D3
#  define MATRIX_PRBS_M10PR_SHIFT        (8)       /* Bits 8-9:  Master 10 Priority */
#  define MATRIX_PRBS_M10PR_MASK         (3 << MATRIX_PRBS_M10PR_SHIFT)
#    define MATRIX_PRBS_M10PR(n)         ((uint32_t)(n) << MATRIX_PRBS_M10PR_SHIFT)
#  define MATRIX_PRBS_M11PR_SHIFT        (12)      /* Bits 12-13: Master 11 Priority */
#  define MATRIX_PRBS_M11PR_MASK         (3 << MATRIX_PRBS_M11PR_SHIFT)
#    define MATRIX_PRBS_M11PR(n)         ((uint32_t)(n) << MATRIX_PRBS_M11PR_SHIFT)
#  define MATRIX_PRBS_M12PR_SHIFT        (16)      /* Bits 16-17: Master 12 Priority */
#  define MATRIX_PRBS_M12PR_MASK         (3 << MATRIX_PRBS_M12PR_SHIFT)
#    define MATRIX_PRBS_M12PR(n)         ((uint32_t)(n) << MATRIX_PRBS_M12PR_SHIFT)
#  define MATRIX_PRBS_M13PR_SHIFT        (20)      /* Bits 20-21: Master 13 Priority */
#  define MATRIX_PRBS_M13PR_MASK         (3 << MATRIX_PRBS_M13PR_SHIFT)
#    define MATRIX_PRBS_M13PR(n)         ((uint32_t)(n) << MATRIX_PRBS_M13PR_SHIFT)
#  define MATRIX_PRBS_M14PR_SHIFT        (24)      /* Bits 24-25: Master 14 Priority */
#  define MATRIX_PRBS_M14PR_MASK         (3 << MATRIX_PRBS_M14PR_SHIFT)
#    define MATRIX_PRBS_M14PR(n)         ((uint32_t)(n) << MATRIX_PRBS_M14PR_SHIFT)
#  define MATRIX_PRBS_M15PR_SHIFT        (28)      /* Bits 28-29: Master 15 Priority */
#  define MATRIX_PRBS_M15PR_MASK         (3 << MATRIX_PRBS_M15PR_SHIFT)
#    define MATRIX_PRBS_M15PR(n)         ((uint32_t)(n) << MATRIX_PRBS_M15PR_SHIFT)
#endif

#ifdef ATSAMA5D3
/* Master Remap Control Register */

#  define MATRIX_MRCR_RCB(n)             (1 << (n))
#    define MATRIX_MRCR_RCB0             (1 << 0)  /* Bit 0:  Remap Command Bit for Master 0 */
#    define MATRIX_MRCR_RCB1             (1 << 1)  /* Bit 1:  Remap Command Bit for Master 1 */
#    define MATRIX_MRCR_RCB2             (1 << 2)  /* Bit 2:  Remap Command Bit for Master 2 */
#    define MATRIX_MRCR_RCB3             (1 << 3)  /* Bit 3:  Remap Command Bit for Master 3 */
#    define MATRIX_MRCR_RCB4             (1 << 4)  /* Bit 4:  Remap Command Bit for Master 4 */
#    define MATRIX_MRCR_RCB5             (1 << 5)  /* Bit 5:  Remap Command Bit for Master 5 */
#    define MATRIX_MRCR_RCB6             (1 << 6)  /* Bit 6:  Remap Command Bit for Master 6 */
#    define MATRIX_MRCR_RCB7             (1 << 7)  /* Bit 7:  Remap Command Bit for Master 7 */
#    define MATRIX_MRCR_RCB8             (1 << 8)  /* Bit 8:  Remap Command Bit for Master 8 */
#    define MATRIX_MRCR_RCB9             (1 << 9)  /* Bit 9:  Remap Command Bit for Master 9 */
#    define MATRIX_MRCR_RCB10            (1 << 10) /* Bit 10: Remap Command Bit for Master 10 */
#    define MATRIX_MRCR_RCB11            (1 << 11) /* Bit 11: Remap Command Bit for Master 11 */
#    define MATRIX_MRCR_RCB12            (1 << 12) /* Bit 12: Remap Command Bit for Master 12 */
#    define MATRIX_MRCR_RCB13            (1 << 13) /* Bit 13: Remap Command Bit for Master 13 */
#    define MATRIX_MRCR_RCB14            (1 << 14) /* Bit 14: Remap Command Bit for Master 14 */
#    define MATRIX_MRCR_RCB15            (1 << 15) /* Bit 15: Remap Command Bit for Master 15 */
#endif

#ifdef ATSAMA5D4
/* Master Error Interrupt Enable Register, Master Error Interrupt Disable Register,
 * Master Error Interrupt Mask Register, and Master Error Status Register 
 */

#  define MATRIX_MEINT_MERR(n)           (1 << (n)) /* Master x Access Error, n=0..9 */
#    define MATRIX_MEINT_MERR0           (1 << 0)   /* Master 0 Access Error */
#    define MATRIX_MEINT_MERR1           (1 << 1)   /* Master 1 Access Error */
#    define MATRIX_MEINT_MERR2           (1 << 2)   /* Master 2 Access Error */
#    define MATRIX_MEINT_MERR3           (1 << 3)   /* Master 3 Access Error */
#    define MATRIX_MEINT_MERR4           (1 << 4)   /* Master 4 Access Error */
#    define MATRIX_MEINT_MERR5           (1 << 5)   /* Master 5 Access Error */
#    define MATRIX_MEINT_MERR6           (1 << 6)   /* Master 6 Access Error */
#    define MATRIX_MEINT_MERR7           (1 << 7)   /* Master 7 Access Error */
#    define MATRIX_MEINT_MERR8           (1 << 8)   /* Master 8 Access Error */
#    define MATRIX_MEINT_MERR9           (1 << 9)   /* Master 9 Access Error */

/* Master 0-9 Error Address Register (32-bit addresses) */
#endif

/* Write Protect Mode Register */

#define MATRIX_WPMR_WPEN                 (1 << 0)  /* Bit 0:  Write Protect Enable */
#define MATRIX_WPMR_WPKEY_SHIFT          (8)       /* Bits 8-31:   Write Protect KEY (Write-only) */
#define MATRIX_WPMR_WPKEY_MASK           (0x00ffffff << MATRIX_WPMR_WPKEY_SHIFT)
#  define MATRIX_WPMR_WPKEY              (0x004d4154 << MATRIX_WPMR_WPKEY_SHIFT)

/* Write Protect Status Register */

#define MATRIX_WPSR_WPVS                 (1 << 0)  /* Bit 0:  Enable Write Protect */
#define MATRIX_WPSR_WPVSRC_SHIFT         (8)       /* Bits 8-23:  Write Protect Violation Source */
#define MATRIX_WPSR_WPVSRC_MASK          (0xffff << MATRIX_WPSR_WPVSRC_SHIFT)

#ifdef ATSAMA5D4
/* Security Slave 0-12 Registers */

#  define MATRIX_SSR_LANSECH_SHIFT(n)   (0)       /* Bits 0-7: Low Area Not Secured in HSELn Security Region, n=0..7 */
#  define MATRIX_SSR_LANSECH_MASK(n)    (0xff << MATRIX_SSR_LANSECH_SHIFT(n))
#    define MATRIX_SSR_LANSECH(n)       (1 << (n))
#    define MATRIX_SSR_LANSECH0         (1 << 0)  /* Bit 0:  Low Area Not Secured in HSEL0 Security Region */
#    define MATRIX_SSR_LANSECH1         (1 << 1)  /* Bit 1:  Low Area Not Secured in HSEL1 Security Region */
#    define MATRIX_SSR_LANSECH2         (1 << 2)  /* Bit 2:  Low Area Not Secured in HSEL2 Security Region */
#    define MATRIX_SSR_LANSECH3         (1 << 3)  /* Bit 3:  Low Area Not Secured in HSEL3 Security Region */
#    define MATRIX_SSR_LANSECH4         (1 << 4)  /* Bit 4:  Low Area Not Secured in HSEL4 Security Region */
#    define MATRIX_SSR_LANSECH5         (1 << 5)  /* Bit 5:  Low Area Not Secured in HSEL5 Security Region */
#    define MATRIX_SSR_LANSECH6         (1 << 6)  /* Bit 6:  Low Area Not Secured in HSEL6 Security Region */
#    define MATRIX_SSR_LANSECH7         (1 << 7)  /* Bit 7:  Low Area Not Secured in HSEL7 Security Region */
#  define MATRIX_SSR_RDNSECH_SHIFT(n)   (8)       /* Bits 8-15: Read Not Secured for HSELn Security Region, n=0..7 */
#  define MATRIX_SSR_RDNSECH_MASK(n)    (0xff << MATRIX_SSR_RDNSECH_SHIFT(n))
#    define MATRIX_SSR_RDNSECH(n)       (1 << ((n)+ 8))
#    define MATRIX_SSR_RDNSECH0         (1 << 8)  /* Bit 8:  Read Not Secured for HSEL0 Security Region */
#    define MATRIX_SSR_RDNSECH1         (1 << 9)  /* Bit 9:  Read Not Secured for HSEL1 Security Region */
#    define MATRIX_SSR_RDNSECH2         (1 << 10) /* Bit 10: Read Not Secured for HSEL2 Security Region */
#    define MATRIX_SSR_RDNSECH3         (1 << 11) /* Bit 11: Read Not Secured for HSEL3 Security Region */
#    define MATRIX_SSR_RDNSECH4         (1 << 12) /* Bit 12: Read Not Secured for HSEL4 Security Region */
#    define MATRIX_SSR_RDNSECH5         (1 << 13) /* Bit 13: Read Not Secured for HSEL5 Security Region */
#    define MATRIX_SSR_RDNSECH6         (1 << 14) /* Bit 14: Read Not Secured for HSEL6 Security Region */
#    define MATRIX_SSR_RDNSECH7         (1 << 15) /* Bit 15: Read Not Secured for HSEL7 Security Region */
#  define MATRIX_SSR_WRNSECH_SHIFT(n)   (16)      /* Bit 16-23: Write Not Secured for HSELn Security Region, n=0..7 */
#  define MATRIX_SSR_WRNSECH_MASK(n)    (0xff << MATRIX_SSR_WRNSECH_SHIFT(n))
#    define MATRIX_SSR_WRNSECH(n)       (1 << ((n)+ 16))
#    define MATRIX_SSR_WRNSECH0         (1 << 16) /* Bit 16: Write Not Secured for HSEL0 Security Region */
#    define MATRIX_SSR_WRNSECH1         (1 << 17) /* Bit 17: Write Not Secured for HSEL1 Security Region */
#    define MATRIX_SSR_WRNSECH2         (1 << 18) /* Bit 18: Write Not Secured for HSEL2 Security Region */
#    define MATRIX_SSR_WRNSECH3         (1 << 19) /* Bit 19: Write Not Secured for HSEL3 Security Region */
#    define MATRIX_SSR_WRNSECH4         (1 << 20) /* Bit 20: Write Not Secured for HSEL4 Security Region */
#    define MATRIX_SSR_WRNSECH5         (1 << 21) /* Bit 21: Write Not Secured for HSEL5 Security Region */
#    define MATRIX_SSR_WRNSECH6         (1 << 22) /* Bit 22: Write Not Secured for HSEL6 Security Region */
#    define MATRIX_SSR_WRNSECH7         (1 << 23) /* Bit 23: Write Not Secured for HSEL7 Security Region */

/* Security Areas Split Slave 1-12 Registers */

#  define SASSR_SASPLIT_4KB              0  /* 0x00001000 4 Kbytes */
#  define SASSR_SASPLIT_8KB              1  /* 0x00002000 8 Kbytes */
#  define SASSR_SASPLIT_16KB             2  /* 0x00004000 16 Kbytes */
#  define SASSR_SASPLIT_32KB             3  /* 0x00008000 32 Kbytes */
#  define SASSR_SASPLIT_64KB             4  /* 0x00010000 64 Kbytes */
#  define SASSR_SASPLIT_128KB            5  /* 0x00020000 128 Kbytes */
#  define SASSR_SASPLIT_256KB            6  /* 0x00040000 256 Kbytes */
#  define SASSR_SASPLIT_512KB            7  /* 0x00080000 512 Kbytes */
#  define SASSR_SASPLIT_1MB              8  /* 0x00100000 1 Mbyte */
#  define SASSR_SASPLIT_2MB              9  /* 0x00200000 2 Mbytes */
#  define SASSR_SASPLIT_4MB              10 /* 0x00400000 4 Mbytes */
#  define SASSR_SASPLIT_8MB              11 /* 0x00800000 8 Mbytes */
#  define SASSR_SASPLIT_16MB             12 /* 0x01000000 16 Mbytes */
#  define SASSR_SASPLIT_32MB             13 /* 0x02000000 32 Mbytes */
#  define SASSR_SASPLIT_64MB             14 /* 0x04000000 64 Mbytes */
#  define SASSR_SASPLIT_128MB            15 /* 0x08000000 128 Mbytes */

#  define MATRIX_SASSR_SASPLIT_SHIFT(n) ((n) << 4) /* Security Areas Split for HSELn Security Region, n=0..7 */
#  define MATRIX_SASSR_SASPLIT_MASK(n)  (15 << MATRIX_SASSR_SASPLIT_SHIFT(n))
#    define MATRIX_SASSR_SASPLIT(n,v)   ((uint32_t)(v) << MATRIX_SASSR_SASPLIT_SHIFT(n)) /* See definitions above */
#  define MATRIX_SASSR_SASPLIT0_SHIFT   (0)       /* Bits 0-3: Security Areas Split for HSEL0 Security Region */
#  define MATRIX_SASSR_SASPLIT0_MASK    (15 << MATRIX_SASSR_SASPLIT0_SHIFT)
#    define MATRIX_SASSR_SASPLIT0(n)    ((uint32_t)(n) << MATRIX_SASSR_SASPLIT0_SHIFT) /* See definitions above */
#  define MATRIX_SASSR_SASPLIT1_SHIFT   (0)       /* Bits 0-3: Security Areas Split for HSEL0 Security Region */
#  define MATRIX_SASSR_SASPLIT1_MASK    (15 << MATRIX_SASSR_SASPLIT1_SHIFT)
#    define MATRIX_SASSR_SASPLIT1(n)    ((uint32_t)(n) << MATRIX_SASSR_SASPLIT1_SHIFT) /* See definitions above */
#  define MATRIX_SASSR_SASPLIT2_SHIFT   (0)       /* Bits 0-3: Security Areas Split for HSEL0 Security Region */
#  define MATRIX_SASSR_SASPLIT2_MASK    (15 << MATRIX_SASSR_SASPLIT2_SHIFT)
#    define MATRIX_SASSR_SASPLIT2(n)    ((uint32_t)(n) << MATRIX_SASSR_SASPLIT2_SHIFT) /* See definitions above */
#  define MATRIX_SASSR_SASPLIT3_SHIFT   (0)       /* Bits 0-3: Security Areas Split for HSEL0 Security Region */
#  define MATRIX_SASSR_SASPLIT3_MASK    (15 << MATRIX_SASSR_SASPLIT3_SHIFT)
#    define MATRIX_SASSR_SASPLIT3(n)    ((uint32_t)(n) << MATRIX_SASSR_SASPLIT3_SHIFT) /* See definitions above */
#  define MATRIX_SASSR_SASPLIT4_SHIFT   (0)       /* Bits 0-3: Security Areas Split for HSEL0 Security Region */
#  define MATRIX_SASSR_SASPLIT4_MASK    (15 << MATRIX_SASSR_SASPLIT4_SHIFT)
#    define MATRIX_SASSR_SASPLIT4(n)    ((uint32_t)(n) << MATRIX_SASSR_SASPLIT4_SHIFT) /* See definitions above */
#  define MATRIX_SASSR_SASPLIT5_SHIFT   (0)       /* Bits 0-3: Security Areas Split for HSEL0 Security Region */
#  define MATRIX_SASSR_SASPLIT5_MASK    (15 << MATRIX_SASSR_SASPLIT5_SHIFT)
#    define MATRIX_SASSR_SASPLIT5(n)    ((uint32_t)(n) << MATRIX_SASSR_SASPLIT5_SHIFT) /* See definitions above */
#  define MATRIX_SASSR_SASPLIT6_SHIFT   (0)       /* Bits 0-3: Security Areas Split for HSEL0 Security Region */
#  define MATRIX_SASSR_SASPLIT6_MASK    (15 << MATRIX_SASSR_SASPLIT6_SHIFT)
#    define MATRIX_SASSR_SASPLIT6(n)    ((uint32_t)(n) << MATRIX_SASSR_SASPLIT6_SHIFT) /* See definitions above */
#  define MATRIX_SASSR_SASPLIT7_SHIFT   (0)       /* Bits 0-3: Security Areas Split for HSEL0 Security Region */
#  define MATRIX_SASSR_SASPLIT7_MASK    (15 << MATRIX_SASSR_SASPLIT7_SHIFT)
#    define MATRIX_SASSR_SASPLIT7(n)    ((uint32_t)(n) << MATRIX_SASSR_SASPLIT7_SHIFT) /* See definitions above */

/* Security Region Top Slave 1-12 Register */

#  define SRTSR_SRTOP_4KB               0  /* 0x00001000 4 Kbytes */
#  define SRTSR_SRTOP_8KB               1  /* 0x00002000 8 Kbytes */
#  define SRTSR_SRTOP_16KB              2  /* 0x00004000 16 Kbytes */
#  define SRTSR_SRTOP_32KB              3  /* 0x00008000 32 Kbytes */
#  define SRTSR_SRTOP_64KB              4  /* 0x00010000 64 Kbytes */
#  define SRTSR_SRTOP_128KB             5  /* 0x00020000 128 Kbytes */
#  define SRTSR_SRTOP_256KB             6  /* 0x00040000 256 Kbytes */
#  define SRTSR_SRTOP_512KB             7  /* 0x00080000 512 Kbytes */
#  define SRTSR_SRTOP_1MB               8  /* 0x00100000 1 Mbyte */
#  define SRTSR_SRTOP_2MB               9  /* 0x00200000 2 Mbytes */
#  define SRTSR_SRTOP_4MB               10 /* 0x00400000 4 Mbytes */
#  define SRTSR_SRTOP_8MB               11 /* 0x00800000 8 Mbytes */
#  define SRTSR_SRTOP_16MB              12 /* 0x01000000 16 Mbytes */
#  define SRTSR_SRTOP_32MB              13 /* 0x02000000 32 Mbytes */
#  define SRTSR_SRTOP_64MB              14 /* 0x04000000 64 Mbytes */
#  define SRTSR_SRTOP_128MB             15 /* 0x08000000 128 Mbytes */

#  define MATRIX_SRTSR_SRTOP_SHIFT(n)   ((n) << 4) /* HSELn Security Region Top, n=0..7 */
#  define MATRIX_SRTSR_SRTOP_MASK(n)    (15 << MATRIX_SRTSR_SRTOP_SHIFT(n))
#    define MATRIX_SRTSR_SRTOP(n,v)     ((uint32_t)(v) << MATRIX_SRTSR_SRTOP_SHIFT(n)) /* See definitions above */
#  define MATRIX_SRTSR_SRTOP0_SHIFT     (0)       /* Bits 0-3: HSEL0 Security Region Top */
#  define MATRIX_SRTSR_SRTOP0_MASK      (15 << MATRIX_SRTSR_SRTOP0_SHIFT)
#    define MATRIX_SRTSR_SRTOP0(n)      ((uint32_t)(n) << MATRIX_SRTSR_SRTOP0_SHIFT) /* See definitions above */
#  define MATRIX_SRTSR_SRTOP1_SHIFT     (0)       /* Bits 0-3: HSEL0 Security Region Top */
#  define MATRIX_SRTSR_SRTOP1_MASK      (15 << MATRIX_SRTSR_SRTOP1_SHIFT)
#    define MATRIX_SRTSR_SRTOP1(n)      ((uint32_t)(n) << MATRIX_SRTSR_SRTOP1_SHIFT) /* See definitions above */
#  define MATRIX_SRTSR_SRTOP2_SHIFT     (0)       /* Bits 0-3: HSEL0 Security Region Top */
#  define MATRIX_SRTSR_SRTOP2_MASK      (15 << MATRIX_SRTSR_SRTOP2_SHIFT)
#    define MATRIX_SRTSR_SRTOP2(n)      ((uint32_t)(n) << MATRIX_SRTSR_SRTOP2_SHIFT) /* See definitions above */
#  define MATRIX_SRTSR_SRTOP3_SHIFT     (0)       /* Bits 0-3: HSEL0 Security Region Top */
#  define MATRIX_SRTSR_SRTOP3_MASK      (15 << MATRIX_SRTSR_SRTOP3_SHIFT)
#    define MATRIX_SRTSR_SRTOP3(n)      ((uint32_t)(n) << MATRIX_SRTSR_SRTOP3_SHIFT) /* See definitions above */
#  define MATRIX_SRTSR_SRTOP4_SHIFT     (0)       /* Bits 0-3: HSEL0 Security Region Top */
#  define MATRIX_SRTSR_SRTOP4_MASK      (15 << MATRIX_SRTSR_SRTOP4_SHIFT)
#    define MATRIX_SRTSR_SRTOP4(n)      ((uint32_t)(n) << MATRIX_SRTSR_SRTOP4_SHIFT) /* See definitions above */
#  define MATRIX_SRTSR_SRTOP5_SHIFT     (0)       /* Bits 0-3: HSEL0 Security Region Top */
#  define MATRIX_SRTSR_SRTOP5_MASK      (15 << MATRIX_SRTSR_SRTOP5_SHIFT)
#    define MATRIX_SRTSR_SRTOP5(n)      ((uint32_t)(n) << MATRIX_SRTSR_SRTOP5_SHIFT) /* See definitions above */
#  define MATRIX_SRTSR_SRTOP6_SHIFT     (0)       /* Bits 0-3: HSEL0 Security Region Top */
#  define MATRIX_SRTSR_SRTOP6_MASK      (15 << MATRIX_SRTSR_SRTOP6_SHIFT)
#    define MATRIX_SRTSR_SRTOP6(n)      ((uint32_t)(n) << MATRIX_SRTSR_SRTOP6_SHIFT) /* See definitions above */
#  define MATRIX_SRTSR_SRTOP7_SHIFT     (0)       /* Bits 0-3: HSEL0 Security Region Top */
#  define MATRIX_SRTSR_SRTOP7_MASK      (15 << MATRIX_SRTSR_SRTOP7_SHIFT)
#    define MATRIX_SRTSR_SRTOP7(n)      ((uint32_t)(n) << MATRIX_SRTSR_SRTOP7_SHIFT) /* See definitions above */

/* Security Peripheral Select 1 Register */

#  define MATRIX_SPSELR1_NSECP(n)       (1 << (n))      /* PID n Not Secured Peripheral, n=0-31 */

/* Security Peripheral Select 2 Register */

#  define MATRIX_SPSELR2_NSECP(n)       (1 << ((n)-32)) /* PID n Not Secured Peripheral, n=32-63 */

/* Security Peripheral Select 3 Register */

#  define MATRIX_SPSELR3_NSECP(n)       (1 << ((n)-64)) /* PID n Not Secured Peripheral, n=64-96 */
#endif

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_MATRIX_H */
