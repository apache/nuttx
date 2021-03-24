/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_matrix.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_MATRIX_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_MATRIX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MATRIX register offsets **************************************************/

#define SAM_MATRIX_MCFG_OFFSET(n)        ((n)<<2)
#define SAM_MATRIX_MCFG0_OFFSET          0x0000 /* Master Configuration Register 0 */
#define SAM_MATRIX_MCFG1_OFFSET          0x0004 /* Master Configuration Register 1 */
#define SAM_MATRIX_MCFG2_OFFSET          0x0008 /* Master Configuration Register 2 */
#define SAM_MATRIX_MCFG3_OFFSET          0x000c /* Master Configuration Register 3 */
#define SAM_MATRIX_MCFG4_OFFSET          0x0010 /* Master Configuration Register 4 */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_MCFG5_OFFSET        0x0014 /* Master Configuration Register 5 */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_MCFG6_OFFSET        0x0018 /* Master Configuration Register 6 */
#endif
                                                /* 0x0018-0x003c: Reserved */
#define SAM_MATRIX_SCFG_OFFSET(n)        (0x0040+((n)<<2))
#define SAM_MATRIX_SCFG0_OFFSET          0x0040 /* Slave Configuration Register 0 */
#define SAM_MATRIX_SCFG1_OFFSET          0x0044 /* Slave Configuration Register 1 */
#define SAM_MATRIX_SCFG2_OFFSET          0x0048 /* Slave Configuration Register 2 */
#define SAM_MATRIX_SCFG3_OFFSET          0x004c /* Slave Configuration Register 3 */
#define SAM_MATRIX_SCFG4_OFFSET          0x0050 /* Slave Configuration Register 4 */
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_SCFG5_OFFSET        0x0054 /* Slave Configuration Register 5 */
#endif
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_MATRIX_SCFG6_OFFSET        0x0058 /* Slave Configuration Register 6 */
#  define SAM_MATRIX_SCFG7_OFFSET        0x005c /* Slave Configuration Register 7 */
#  define SAM_MATRIX_SCFG8_OFFSET        0x0060 /* Slave Configuration Register 8 */
#endif
#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  define SAM_MATRIX_SCFG9_OFFSET        0x0064 /* Slave Configuration Register 9 */
#endif

#define SAM_MATRIX_PRAS_OFFSET(n)        (0x0080+((n)<<3))
#define SAM_MATRIX_PRAS0_OFFSET          0x0080 /* Priority Register A for Slave 0 */
#define SAM_MATRIX_PRAS1_OFFSET          0x0088 /* Priority Register A for Slave 1 */
#define SAM_MATRIX_PRAS2_OFFSET          0x0090 /* Priority Register A for Slave 2 */
#define SAM_MATRIX_PRAS3_OFFSET          0x0098 /* Priority Register A for Slave 3 */
#define SAM_MATRIX_PRAS4_OFFSET          0x00a0 /* Priority Register A for Slave 4 */
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_PRAS5_OFFSET        0x00a8 /* Priority Register A for Slave 5 */
#endif
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_MATRIX_PRAS6_OFFSET        0x00b0 /* Priority Register A for Slave 6 */
#  define SAM_MATRIX_PRAS7_OFFSET        0x00b8 /* Priority Register A for Slave 7 */
#  define SAM_MATRIX_PRAS8_OFFSET        0x00c0 /* Priority Register A for Slave 8 */
#endif
#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  define SAM_MATRIX_PRAS9_OFFSET        0x00c8 /* Priority Register A for Slave 9 */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_MRCR_OFFSET         0x0100 /* Master Remap Control Register */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_CCFG_SYSIO_OFFSET   0x0114 /* System I/O Configuration Register */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_CCFG_SMCNFCS_OFFSET 0x011c /* SMC Chip Select NAND Flash Assignment Register */
#endif

#define SAM_MATRIX_WPMR_OFFSET           0x01e4 /* Write Protect Mode Register */
#define SAM_MATRIX_WPSR_OFFSET           0x01e8 /* Write Protect Status Register */
                                                /* 0x0110 - 0x01fc: Reserved */

/* MATRIX register addresses ************************************************/

#define SAM_MATRIX_MCFG(n)               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG_OFFSET(n))
#define SAM_MATRIX_MCFG0                 (SAM_MATRIX_BASE+SAM_MATRIX_MCFG0_OFFSET)
#define SAM_MATRIX_MCFG1                 (SAM_MATRIX_BASE+SAM_MATRIX_MCFG1_OFFSET)
#define SAM_MATRIX_MCFG2                 (SAM_MATRIX_BASE+SAM_MATRIX_MCFG2_OFFSET)
#define SAM_MATRIX_MCFG3                 (SAM_MATRIX_BASE+SAM_MATRIX_MCFG3_OFFSET)
#define SAM_MATRIX_MCFG4                 (SAM_MATRIX_BASE+SAM_MATRIX_MCFG4_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_MCFG5               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG5_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_MCFG6               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG6_OFFSET)
#endif

#define SAM_MATRIX_SCFG(n)               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG_OFFSET(n))
#define SAM_MATRIX_SCFG0                 (SAM_MATRIX_BASE+SAM_MATRIX_SCFG0_OFFSET)
#define SAM_MATRIX_SCFG1                 (SAM_MATRIX_BASE+SAM_MATRIX_SCFG1_OFFSET)
#define SAM_MATRIX_SCFG2                 (SAM_MATRIX_BASE+SAM_MATRIX_SCFG2_OFFSET)
#define SAM_MATRIX_SCFG3                 (SAM_MATRIX_BASE+SAM_MATRIX_SCFG3_OFFSET)
#define SAM_MATRIX_SCFG4                 (SAM_MATRIX_BASE+SAM_MATRIX_SCFG4_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_SCFG5               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG5_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_MATRIX_SCFG6               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG6_OFFSET)
#  define SAM_MATRIX_SCFG7               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG7_OFFSET)
#  define SAM_MATRIX_SCFG8               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG8_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  define SAM_MATRIX_SCFG9               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG9_OFFSET)
#endif

#define SAM_MATRIX_PRAS(n)               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS_OFFSET(n))
#define SAM_MATRIX_PRAS0                 (SAM_MATRIX_BASE+SAM_MATRIX_PRAS0_OFFSET)
#define SAM_MATRIX_PRAS1                 (SAM_MATRIX_BASE+SAM_MATRIX_PRAS1_OFFSET)
#define SAM_MATRIX_PRAS2                 (SAM_MATRIX_BASE+SAM_MATRIX_PRAS2_OFFSET)
#define SAM_MATRIX_PRAS3                 (SAM_MATRIX_BASE+SAM_MATRIX_PRAS3_OFFSET)
#define SAM_MATRIX_PRAS4                 (SAM_MATRIX_BASE+SAM_MATRIX_PRAS4_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_PRAS5               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS5_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_MATRIX_PRAS6               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS6_OFFSET)
#  define SAM_MATRIX_PRAS7               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS7_OFFSET)
#  define SAM_MATRIX_PRAS8               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS8_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  define SAM_MATRIX_PRAS9               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS9_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_MRCR                (SAM_MATRIX_BASE+SAM_MATRIX_MRCR_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_CCFG_SYSIO          (SAM_MATRIX_BASE+SAM_MATRIX_CCFG_SYSIO_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MATRIX_CCFG_SMCNFCS        (SAM_MATRIX_BASE+SAM_MATRIX_CCFG_SMCNFCS_OFFSET)
#endif

#define SAM_MATRIX_WPMR                  (SAM_MATRIX_BASE+SAM_MATRIX_WPMR_OFFSET)
#define SAM_MATRIX_WPSR                  (SAM_MATRIX_BASE+SAM_MATRIX_WPSR_OFFSET)

/* MATRIX register bit definitions ******************************************/

/* Master Configuration Registers */

#define MATRIX_MCFG_ULBT_SHIFT           (0)       /* Bits 0-2:  Undefined Length Burst Type */
#define MATRIX_MCFG_ULBT_MASK            (7 << MATRIX_MCFG_ULBT_SHIFT)
#  define MATRIX_MCFG_ULBT_INF           (0 << MATRIX_MCFG_ULBT_SHIFT) /* Infinite Length Burst */
#  define MATRIX_MCFG_ULBT_SINGLE        (1 << MATRIX_MCFG_ULBT_SHIFT) /* Single Access */
#  define MATRIX_MCFG_ULBT_4BEAT         (2 << MATRIX_MCFG_ULBT_SHIFT) /* 4-beat Burst */
#  define MATRIX_MCFG_ULBT_8BEAT         (3 << MATRIX_MCFG_ULBT_SHIFT) /* 8-beat Burst */
#  define MATRIX_MCFG_ULBT_16BEAT        (4 << MATRIX_MCFG_ULBT_SHIFT) /* 16-beat Burst */
#  if defined(CONFIG_ARCH_CHIP_SAM4E)
#    define MATRIX_MCFG_ULBT_32BEAT      (5 << MATRIX_MCFG_ULBT_SHIFT) /* 32-beat Burst */
#    define MATRIX_MCFG_ULBT_64BEAT      (6 << MATRIX_MCFG_ULBT_SHIFT) /* 64-beat Burst */
#    define MATRIX_MCFG_ULBT_128BEAT     (7 << MATRIX_MCFG_ULBT_SHIFT) /* 128-beat Burst */
#  endif

/* Bus Matrix Slave Configuration Registers */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define MATRIX_SCFG_SLOTCYCLE_SHIFT    (0)       /* Bits 0-8:  Maximum Number of Allowed Cycles for a Burst */
#  define MATRIX_SCFG_SLOTCYCLE_MASK     (0x1ff << MATRIX_SCFG_SLOTCYCLE_SHIFT)
#    define MATRIX_SCFG_SLOTCYCLE(n)     ((uint32_t)(n) << MATRIX_SCFG_SLOTCYCLE_SHIFT)
#else
#  define MATRIX_SCFG_SLOTCYCLE_SHIFT    (0)       /* Bits 0-7:  Maximum Number of Allowed Cycles for a Burst */
#  define MATRIX_SCFG_SLOTCYCLE_MASK     (0xff << MATRIX_SCFG_SLOTCYCLE_SHIFT)
#    define MATRIX_SCFG_SLOTCYCLE(n)     ((uint32_t)(n) << MATRIX_SCFG_SLOTCYCLE_SHIFT)
#endif

#define MATRIX_SCFG_DEFMSTRTYPE_SHIFT    (16)      /* Bits 16-17:  Default Master Type */
#define MATRIX_SCFG_DEFMSTRTYPE_MASK     (3 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_NONE   (0 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_LAST   (1 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_FIXED  (2 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)

#define MATRIX_SCFG_FIXEDDEFMSTR_SHIFT   (18)      /* Bits 18-20:   Fixed Default Master */
#define MATRIX_SCFG_FIXEDDEFMSTR_MASK    (7 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG0_FIXEDDEFMSTR(n)   ((uint32_t)(n) << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG0_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG1_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG2_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG3_FIXEDDEFMSTR_ARMC (0 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG4_FIXEDDEFMSTR_ARMC (0 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG5_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG6_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG7_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG8_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG8_FIXEDDEFMSTR_HDMA (4 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG9_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG9_FIXEDDEFMSTR_HDMA (4 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)

#if !defined(CONFIG_ARCH_CHIP_SAM4E)
#  define MATRIX_SCFG_ARBT_SHIFT         (24)      /* Bits 24-25:   Arbitration Type */
#  define MATRIX_SCFG_ARBT_MASK          (3 << MATRIX_SCFG_ARBT_SHIFT)
#    define MATRIX_SCFG_ARBT_RR          (0 << MATRIX_SCFG_ARBT_SHIFT) /* Round-Robin Arbitration */
#    define MATRIX_SCFG_ARBT_FIXED       (1 << MATRIX_SCFG_ARBT_SHIFT) /* Fixed Priority Arbitration */
#endif

/* Bus Matrix Priority Registers For Slaves */

#define MATRIX_PRAS_MPR_SHIFT(x)         ((n)<<2)
#define MATRIX_PRAS_MPR_MASK(x)          (3 << MATRIX_PRAS_MPR_SHIFT(x))
#  define MATRIX_PRAS_M0PR_SHIFT         (0)       /* Bits 0-1:  Master 0 Priority */
#  define MATRIX_PRAS_M0PR_MASK          (3 << MATRIX_PRAS_M0PR_SHIFT)
#    define MATRIX_PRAS_M0PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M0PR_SHIFT)
#  define MATRIX_PRAS_M1PR_SHIFT         (4)       /* Bits 4-5:   Master 1 Priority */
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
#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define MATRIX_PRAS_M5PR_SHIFT         (20)      /* Bits 20-21: Master 5 Priority */
#  define MATRIX_PRAS_M5PR_MASK          (3 << MATRIX_PRAS_M5PR_SHIFT)
#    define MATRIX_PRAS_M5PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M5PR_SHIFT)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define MATRIX_PRAS_M6PR_SHIFT         (24)      /* Bits 24-25: Master 6 Priority */
#  define MATRIX_PRAS_M6PR_MASK          (3 << MATRIX_PRAS_M6PR_SHIFT)
#    define MATRIX_PRAS_M6PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M6PR_SHIFT)
#  define MATRIX_PRAS_M7PR_SHIFT         (28)      /* Bits 28-29: Master 7 Priority */
#  define MATRIX_PRAS_M7PR_MASK          (3 << MATRIX_PRAS_M7PR_SHIFT)
#    define MATRIX_PRAS_M7PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M7PR_SHIFT)
#endif

/* System I/O Configuration Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define MATRIX_CCFG_SYSIO_SYSIO4       (1 << 4)  /* Bit 4:  PB4 or TDI Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO5       (1 << 5)  /* Bit 5:  PB5 or TDO/TRACESWO Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO6       (1 << 6)  /* Bit 6:  PB6 or TMS/SWDIO Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO7       (1 << 7)  /* Bit 7:  PB7 or TCK/SWCLK Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO10      (1 << 10) /* Bit 10: PB10 or DDM Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO11      (1 << 11) /* Bit 11: PB11 or DDP Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO12      (1 << 12) /* Bit 12: PB12 or ERASE Assignment */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define MATRIX_CCFG_SYSIO_SYSIO12      (1 << 12) /* Bit 12: PC0 or ERASE Assignment */
#endif

/* SMC Chip Select NAND Flash Assignment Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#define MATRIX_CCFG_SMCNFCS_SMC_NFCS(n)  (1<<(n))  /* Bit n:  SMC NAND Flash Chip Select n Assignment */
#  define MATRIX_CCFG_SMCNFCS_SMC_NFCS0  (1 << 0)  /* Bit 0:  SMC NAND Flash Chip Select 0 Assignment */
#  define MATRIX_CCFG_SMCNFCS_SMC_NFCS1  (1 << 1)  /* Bit 1:  SMC NAND Flash Chip Select 2 Assignment */
#  define MATRIX_CCFG_SMCNFCS_SMC_NFCS2  (1 << 2)  /* Bit 2:  SMC NAND Flash Chip Select 2 Assignment */
#  define MATRIX_CCFG_SMCNFCS_SMC_NFCS3  (1 << 3)  /* Bit 3:  SMC NAND Flash Chip Select 3 Assignment */
#endif

/* Master Remap Control Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define MATRIX_MRCR_RCB(x)             (1 << (x))
#  define MATRIX_MRCR_RCB0               (1 << 0)  /* Bit 0:  Remap Command Bit for AHB Master 0 */
#  define MATRIX_MRCR_RCB1               (1 << 1)  /* Bit 1:  Remap Command Bit for AHB Master 1 */
#  define MATRIX_MRCR_RCB2               (1 << 2)  /* Bit 2:  Remap Command Bit for AHB Master 2 */
#  define MATRIX_MRCR_RCB3               (1 << 3)  /* Bit 3:  Remap Command Bit for AHB Master 3 */
#  define MATRIX_MRCR_RCB4               (1 << 4)  /* Bit 4:  Remap Command Bit for AHB Master 4 */
#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define MATRIX_MRCR_RCB5               (1 << 5)  /* Bit 5:  Remap Command Bit for AHB Master 5 */
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define MATRIX_MRCR_RCB6               (1 << 6)  /* Bit 6:  Remap Command Bit for AHB Master 6 */
#  define MATRIX_MRCR_RCB7               (1 << 7)  /* Bit 7:  Remap Command Bit for AHB Master 7 */
#  define MATRIX_MRCR_RCB8               (1 << 8)  /* Bit 8:  Remap Command Bit for AHB Master 8 */
#  define MATRIX_MRCR_RCB9               (1 << 9)  /* Bit 9:  Remap Command Bit for AHB Master 9 */
#  define MATRIX_MRCR_RCB10              (1 << 10) /* Bit 10: Remap Command Bit for AHB Master 10 */
#  define MATRIX_MRCR_RCB11              (1 << 11) /* Bit 11: Remap Command Bit for AHB Master 11 */
#  define MATRIX_MRCR_RCB12              (1 << 12) /* Bit 12: Remap Command Bit for AHB Master 12 */
#  define MATRIX_MRCR_RCB13              (1 << 13) /* Bit 13: Remap Command Bit for AHB Master 13 */
#  define MATRIX_MRCR_RCB14              (1 << 14) /* Bit 14: Remap Command Bit for AHB Master 14 */
#  define MATRIX_MRCR_RCB15              (1 << 15) /* Bit 15: Remap Command Bit for AHB Master 15 */
#endif
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_MATRIX_H */
