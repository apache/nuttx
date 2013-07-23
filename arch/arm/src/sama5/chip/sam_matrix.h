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
#define SAM_MATRIX_MCFG10_OFFSET         0x0028 /* Master Configuration Register 10 */
#define SAM_MATRIX_MCFG11_OFFSET         0x002c /* Master Configuration Register 11 */
#define SAM_MATRIX_MCFG12_OFFSET         0x0030 /* Master Configuration Register 12 */
#define SAM_MATRIX_MCFG13_OFFSET         0x0034 /* Master Configuration Register 13 */
#define SAM_MATRIX_MCFG14_OFFSET         0x0038 /* Master Configuration Register 14 */
#define SAM_MATRIX_MCFG15_OFFSET         0x003c /* Master Configuration Register 15 */
                                                /* 0x0018-0x003c: Reserved */
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
#define SAM_MATRIX_SCFG13_OFFSET         0x0074 /* Slave Configuration Register 13 */
#define SAM_MATRIX_SCFG14_OFFSET         0x0078 /* Slave Configuration Register 14 */
#define SAM_MATRIX_SCFG15_OFFSET         0x007c /* Slave Configuration Register 15 */

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
#define SAM_MATRIX_PRAS13_OFFSET         0x00e8 /* Priority Register A for Slave 13 */
#define SAM_MATRIX_PRBS13_OFFSET         0x00ec /* Priority Register B for Slave 13 */
#define SAM_MATRIX_PRAS14_OFFSET         0x00f0 /* Priority Register A for Slave 14 */
#define SAM_MATRIX_PRBS14_OFFSET         0x00f4 /* Priority Register B for Slave 14 */
#define SAM_MATRIX_PRAS15_OFFSET         0x00f8 /* Priority Register A for Slave 15 */
#define SAM_MATRIX_PRBS15_OFFSET         0x00fc /* Priority Register B for Slave 15 */
#define SAM_MATRIX_MRCR_OFFSET           0x0100 /* Master Remap Control Register */
                                                /* 0X104-0X1e0: Reserved */
#define SAM_MATRIX_WPMR_OFFSET           0x01e4 /* Write Protect Mode Register */
#define SAM_MATRIX_WPSR_OFFSET           0x01e8 /* Write Protect Status Register */
                                                /* 0x0110 - 0x01fc: Reserved */

/* MATRIX register adresses *************************************************************/

#define SAM_MATRIX_MCFG(n))              (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG_OFFSET(n))
#define SAM_MATRIX_MCFG0                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG0_OFFSET)
#define SAM_MATRIX_MCFG1                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG1_OFFSET)
#define SAM_MATRIX_MCFG2                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG2_OFFSET)
#define SAM_MATRIX_MCFG3                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG3_OFFSET)
#define SAM_MATRIX_MCFG4                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG4_OFFSET)
#define SAM_MATRIX_MCFG5                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG5_OFFSET)
#define SAM_MATRIX_MCFG6                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG6_OFFSET)
#define SAM_MATRIX_MCFG7                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG7_OFFSET)
#define SAM_MATRIX_MCFG8                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG8_OFFSET)
#define SAM_MATRIX_MCFG9                 (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG9_OFFSET)
#define SAM_MATRIX_MCFG10                (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG10_OFFSET)
#define SAM_MATRIX_MCFG11                (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG11_OFFSET)
#define SAM_MATRIX_MCFG12                (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG12_OFFSET)
#define SAM_MATRIX_MCFG13                (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG13_OFFSET)
#define SAM_MATRIX_MCFG14                (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG14_OFFSET)
#define SAM_MATRIX_MCFG15                (SAM_MATRIX_VBASE+SAM_MATRIX_MCFG15_OFFSET)

#define SAM_MATRIX_SCFG(n)               (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG_OFFSET(n))
#define SAM_MATRIX_SCFG0                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG0_OFFSET)
#define SAM_MATRIX_SCFG1                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG1_OFFSET)
#define SAM_MATRIX_SCFG2                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG2_OFFSET)
#define SAM_MATRIX_SCFG3                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG3_OFFSET)
#define SAM_MATRIX_SCFG4                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG4_OFFSET)
#define SAM_MATRIX_SCFG5                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG5_OFFSET)
#define SAM_MATRIX_SCFG6                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG6_OFFSET)
#define SAM_MATRIX_SCFG7                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG7_OFFSET)
#define SAM_MATRIX_SCFG8                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG8_OFFSET)
#define SAM_MATRIX_SCFG9                 (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG9_OFFSET)
#define SAM_MATRIX_SCFG10                (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG10_OFFSET)
#define SAM_MATRIX_SCFG11                (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG11_OFFSET)
#define SAM_MATRIX_SCFG12                (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG12_OFFSET)
#define SAM_MATRIX_SCFG13                (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG13_OFFSET)
#define SAM_MATRIX_SCFG14                (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG14_OFFSET)
#define SAM_MATRIX_SCFG15                (SAM_MATRIX_VBASE+SAM_MATRIX_SCFG15_OFFSET)

#define SAM_MATRIX_PRAS(n)               (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS_OFFSET(n))
#define SAM_MATRIX_PRBS(n)               (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS_OFFSET(n))
#define SAM_MATRIX_PRAS0                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS0_OFFSET)
#define SAM_MATRIX_PRBS0                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS0_OFFSET)
#define SAM_MATRIX_PRAS1                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS1_OFFSET)
#define SAM_MATRIX_PRBS1                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS1_OFFSET)
#define SAM_MATRIX_PRAS2                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS2_OFFSET)
#define SAM_MATRIX_PRBS2                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS2_OFFSET)
#define SAM_MATRIX_PRAS3                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS3_OFFSET)
#define SAM_MATRIX_PRBS3                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS3_OFFSET)
#define SAM_MATRIX_PRAS4                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS4_OFFSET)
#define SAM_MATRIX_PRBS4                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS4_OFFSET)
#define SAM_MATRIX_PRAS5                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS5_OFFSET)
#define SAM_MATRIX_PRBS5                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS5_OFFSET)
#define SAM_MATRIX_PRAS6                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS6_OFFSET)
#define SAM_MATRIX_PRBS6                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS6_OFFSET)
#define SAM_MATRIX_PRAS7                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS7_OFFSET)
#define SAM_MATRIX_PRBS7                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS7_OFFSET)
#define SAM_MATRIX_PRAS8                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS8_OFFSET)
#define SAM_MATRIX_PRBS8                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS8_OFFSET)
#define SAM_MATRIX_PRAS9                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS9_OFFSET)
#define SAM_MATRIX_PRBS9                 (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS9_OFFSET)
#define SAM_MATRIX_PRAS10                (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS10_OFFSET)
#define SAM_MATRIX_PRBS10                (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS10_OFFSET)
#define SAM_MATRIX_PRAS11                (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS11_OFFSET)
#define SAM_MATRIX_PRBS11                (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS11_OFFSET)
#define SAM_MATRIX_PRAS12                (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS12_OFFSET)
#define SAM_MATRIX_PRBS12                (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS12_OFFSET)
#define SAM_MATRIX_PRAS13                (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS13_OFFSET)
#define SAM_MATRIX_PRBS13                (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS13_OFFSET)
#define SAM_MATRIX_PRAS14                (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS14_OFFSET)
#define SAM_MATRIX_PRBS14                (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS14_OFFSET)
#define SAM_MATRIX_PRAS15                (SAM_MATRIX_VBASE+SAM_MATRIX_PRAS15_OFFSET)
#define SAM_MATRIX_PRBS15                (SAM_MATRIX_VBASE+SAM_MATRIX_PRBS15_OFFSET)

#define SAM_MATRIX_MRCR                  (SAM_MATRIX_VBASE+SAM_MATRIX_MRCR_OFFSET)
#define SAM_MATRIX_WPMR                  (SAM_MATRIX_VBASE+SAM_MATRIX_WPMR_OFFSET)
#define SAM_MATRIX_WPSR                  (SAM_MATRIX_VBASE+SAM_MATRIX_WPSR_OFFSET)

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
#define MATRIX_SCFG_DEFMSTRTYPE_SHIFT    (16)      /* Bits 16-17:  Default Master Type */
#define MATRIX_SCFG_DEFMSTRTYPE_MASK     (3 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_NONE   (0 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_LAST   (1 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_FIXED  (2 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#define MATRIX_SCFG_FIXEDDEFMSTR_SHIFT   (18)      /* Bits 18-21:   Fixed Default Master */
#define MATRIX_SCFG_FIXEDDEFMSTR_MASK    (15 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)

/* Bus Matrix Priority Registers A For Slaves */

#define MATRIX_PRAS_MPR_SHIFT(x)         ((n)<<2)
#define MATRIX_PRAS_MPR_MASK(x)          (3 << MATRIX_PRAS_MPR_SHIFT(x))
#  define MATRIX_PRAS_M0PR_SHIFT         (0)       /* Bits 0-1:  Master 0 Priority */
#  define MATRIX_PRAS_M0PR_MASK          (3 << MATRIX_PRAS_M0PR_SHIFT)
#  define MATRIX_PRAS_M1PR_SHIFT         (4)       /* Bits 4-5:  Master 1 Priority */
#  define MATRIX_PRAS_M1PR_MASK          (3 << MATRIX_PRAS_M1PR_SHIFT)
#  define MATRIX_PRAS_M2PR_SHIFT         (8)       /* Bits 8-9:  Master 2 Priority */
#  define MATRIX_PRAS_M2PR_MASK          (3 << MATRIX_PRAS_M2PR_SHIFT)
#  define MATRIX_PRAS_M3PR_SHIFT         (12)      /* Bits 12-13: Master 3 Priority */
#  define MATRIX_PRAS_M3PR_MASK          (3 << MATRIX_PRAS_M3PR_SHIFT)
#  define MATRIX_PRAS_M4PR_SHIFT         (16)      /* Bits 16-17: Master 4 Priority */
#  define MATRIX_PRAS_M4PR_MASK          (3 << MATRIX_PRAS_M4PR_SHIFT)
#  define MATRIX_PRAS_M5PR_SHIFT         (20)      /* Bits 20-21: Master 5 Priority */
#  define MATRIX_PRAS_M5PR_MASK          (3 << MATRIX_PRAS_M5PR_SHIFT)
#  define MATRIX_PRAS_M6PR_SHIFT         (24)      /* Bits 24-25: Master 6 Priority */
#  define MATRIX_PRAS_M6PR_MASK          (3 << MATRIX_PRAS_M6PR_SHIFT)
#  define MATRIX_PRAS_M7PR_SHIFT         (28)      /* Bits 28-29: Master 7 Priority */
#  define MATRIX_PRAS_M7PR_MASK          (3 << MATRIX_PRAS_M7PR_SHIFT)

/* Bus Matrix Priority Registers B For Slaves */

#define MATRIX_PRBS_MPR_SHIFT(x)         ((n)<<2)
#define MATRIX_PRBS_MPR_MASK(x)          (3 << MATRIX_PRBS_MPR_SHIFT(x))
#  define MATRIX_PRBS_M8PR_SHIFT         (0)       /* Bits 0-1:  Master 8 Priority */
#  define MATRIX_PRBS_M8PR_MASK          (3 << MATRIX_PRBS_M8PR_SHIFT)
#  define MATRIX_PRBS_M9PR_SHIFT         (4)       /* Bits 4-5:  Master 9 Priority */
#  define MATRIX_PRBS_M9PR_MASK          (3 << MATRIX_PRBS_M9PR_SHIFT)
#  define MATRIX_PRBS_M10PR_SHIFT        (8)       /* Bits 8-9:  Master 10 Priority */
#  define MATRIX_PRBS_M10PR_MASK         (3 << MATRIX_PRBS_M10PR_SHIFT)
#  define MATRIX_PRBS_M11PR_SHIFT        (12)      /* Bits 12-13: Master 11 Priority */
#  define MATRIX_PRBS_M11PR_MASK         (3 << MATRIX_PRBS_M11PR_SHIFT)
#  define MATRIX_PRBS_M12PR_SHIFT        (16)      /* Bits 16-17: Master 12 Priority */
#  define MATRIX_PRBS_M12PR_MASK         (3 << MATRIX_PRBS_M12PR_SHIFT)
#  define MATRIX_PRBS_M13PR_SHIFT        (20)      /* Bits 20-21: Master 13 Priority */
#  define MATRIX_PRBS_M13PR_MASK         (3 << MATRIX_PRBS_M13PR_SHIFT)
#  define MATRIX_PRBS_M14PR_SHIFT        (24)      /* Bits 24-25: Master 14 Priority */
#  define MATRIX_PRBS_M14PR_MASK         (3 << MATRIX_PRBS_M14PR_SHIFT)
#  define MATRIX_PRBS_M15PR_SHIFT        (28)      /* Bits 28-29: Master 15 Priority */
#  define MATRIX_PRBS_M15PR_MASK         (3 << MATRIX_PRBS_M15PR_SHIFT)

/* Master Remap Control Register */

#define MATRIX_MRCR_RCB(x)               (1 << (x))
#  define MATRIX_MRCR_RCB0               (1 << 0)  /* Bit 0:  Remap Command Bit for Master 0 */
#  define MATRIX_MRCR_RCB1               (1 << 1)  /* Bit 1:  Remap Command Bit for Master 1 */
#  define MATRIX_MRCR_RCB2               (1 << 2)  /* Bit 2:  Remap Command Bit for Master 2 */
#  define MATRIX_MRCR_RCB3               (1 << 3)  /* Bit 3:  Remap Command Bit for Master 3 */
#  define MATRIX_MRCR_RCB4               (1 << 4)  /* Bit 4:  Remap Command Bit for Master 4 */
#  define MATRIX_MRCR_RCB5               (1 << 5)  /* Bit 5:  Remap Command Bit for Master 5 */
#  define MATRIX_MRCR_RCB6               (1 << 6)  /* Bit 6:  Remap Command Bit for Master 6 */
#  define MATRIX_MRCR_RCB7               (1 << 7)  /* Bit 7:  Remap Command Bit for Master 7 */
#  define MATRIX_MRCR_RCB8               (1 << 8)  /* Bit 8:  Remap Command Bit for Master 8 */
#  define MATRIX_MRCR_RCB9               (1 << 9)  /* Bit 9:  Remap Command Bit for Master 9 */
#  define MATRIX_MRCR_RCB10              (1 << 10) /* Bit 10: Remap Command Bit for Master 10 */
#  define MATRIX_MRCR_RCB11              (1 << 11) /* Bit 11: Remap Command Bit for Master 11 */
#  define MATRIX_MRCR_RCB12              (1 << 12) /* Bit 12: Remap Command Bit for Master 12 */
#  define MATRIX_MRCR_RCB13              (1 << 13) /* Bit 13: Remap Command Bit for Master 13 */
#  define MATRIX_MRCR_RCB14              (1 << 14) /* Bit 14: Remap Command Bit for Master 14 */
#  define MATRIX_MRCR_RCB15              (1 << 15) /* Bit 15: Remap Command Bit for Master 15 */

/* Write Protect Mode Register */

#define MATRIX_WPMR_WPEN                 (1 << 0)  /* Bit 0:  Write Protect Enable */
#define MATRIX_WPMR_WPKEY_SHIFT          (8)       /* Bits 8-31:   Write Protect KEY (Write-only) */
#define MATRIX_WPMR_WPKEY_MASK           (0x00ffffff << MATRIX_WPMR_WPKEY_SHIFT)
#  define MATRIX_WPMR_WPKEY              (0x004d4154 << MATRIX_WPMR_WPKEY_SHIFT)

/* Write Protect Status Register */

#define MATRIX_WPSR_WPVS                 (1 << 0)  /* Bit 0:  Enable Write Protect */
#define MATRIX_WPSR_WPVSRC_SHIFT         (8)       /* Bits 8-23:  Write Protect Violation Source */
#define MATRIX_WPSR_WPVSRC_MASK          (0xffff << MATRIX_WPSR_WPVSRC_SHIFT)

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
