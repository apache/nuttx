/****************************************************************************
 * arch/arm/src/samv7/hardware/sam_matrix.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_MATRIX_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_MATRIX_H

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
#  define SAM_MATRIX_MCFG0_OFFSET        0x0000 /* Master Configuration Register 0 */
#  define SAM_MATRIX_MCFG1_OFFSET        0x0004 /* Master Configuration Register 1 */
#  define SAM_MATRIX_MCFG2_OFFSET        0x0008 /* Master Configuration Register 2 */
#  define SAM_MATRIX_MCFG3_OFFSET        0x000c /* Master Configuration Register 3 */
#  define SAM_MATRIX_MCFG4_OFFSET        0x0010 /* Master Configuration Register 4 */
#  define SAM_MATRIX_MCFG5_OFFSET        0x0014 /* Master Configuration Register 5 */
#  define SAM_MATRIX_MCFG6_OFFSET        0x0018 /* Master Configuration Register 6 */
#  define SAM_MATRIX_MCFG7_OFFSET        0x001c /* Master Configuration Register 7 */
#  define SAM_MATRIX_MCFG8_OFFSET        0x0020 /* Master Configuration Register 8 */
#  define SAM_MATRIX_MCFG9_OFFSET        0x0024 /* Master Configuration Register 9 */
#  define SAM_MATRIX_MCFG10_OFFSET       0x0028 /* Master Configuration Register 10 */
#  define SAM_MATRIX_MCFG11_OFFSET       0x002c /* Master Configuration Register 11 */
                                                /* 0x0030-0x003c: Reserved */
#define SAM_MATRIX_SCFG_OFFSET(n)        (0x0040+((n)<<2))
#  define SAM_MATRIX_SCFG0_OFFSET        0x0040 /* Slave Configuration Register 0 */
#  define SAM_MATRIX_SCFG1_OFFSET        0x0044 /* Slave Configuration Register 1 */
#  define SAM_MATRIX_SCFG2_OFFSET        0x0048 /* Slave Configuration Register 2 */
#  define SAM_MATRIX_SCFG3_OFFSET        0x004c /* Slave Configuration Register 3 */
#  define SAM_MATRIX_SCFG4_OFFSET        0x0050 /* Slave Configuration Register 4 */
#  define SAM_MATRIX_SCFG5_OFFSET        0x0054 /* Slave Configuration Register 5 */
#  define SAM_MATRIX_SCFG6_OFFSET        0x0058 /* Slave Configuration Register 6 */
#  define SAM_MATRIX_SCFG7_OFFSET        0x005c /* Slave Configuration Register 7 */
#  define SAM_MATRIX_SCFG8_OFFSET        0x0060 /* Slave Configuration Register 8 */
                                                /* 0x0064-0x007c: Reserved */
#define SAM_MATRIX_PRAS_OFFSET(n)        (0x0080+((n)<<3))
#  define SAM_MATRIX_PRAS0_OFFSET        0x0080 /* Priority Register A for Slave 0 */
#  define SAM_MATRIX_PRAS1_OFFSET        0x0088 /* Priority Register A for Slave 1 */
#  define SAM_MATRIX_PRAS2_OFFSET        0x0090 /* Priority Register A for Slave 2 */
#  define SAM_MATRIX_PRAS3_OFFSET        0x0098 /* Priority Register A for Slave 3 */
#  define SAM_MATRIX_PRAS4_OFFSET        0x00a0 /* Priority Register A for Slave 4 */
#  define SAM_MATRIX_PRAS5_OFFSET        0x00a8 /* Priority Register A for Slave 5 */
#  define SAM_MATRIX_PRAS6_OFFSET        0x00b0 /* Priority Register A for Slave 6 */
#  define SAM_MATRIX_PRAS7_OFFSET        0x00b8 /* Priority Register A for Slave 7 */
#  define SAM_MATRIX_PRAS8_OFFSET        0x00c0 /* Priority Register A for Slave 8 */

#define SAM_MATRIX_PRBS_OFFSET(n)        (0x0084+((n)<<3))
#  define SAM_MATRIX_PRBS0_OFFSET        0x0084 /* Priority Register B for Slave 0 */
#  define SAM_MATRIX_PRBS1_OFFSET        0x008c /* Priority Register B for Slave 1 */
#  define SAM_MATRIX_PRBS2_OFFSET        0x0094 /* Priority Register B for Slave 2 */
#  define SAM_MATRIX_PRBS3_OFFSET        0x009c /* Priority Register B for Slave 3 */
#  define SAM_MATRIX_PRBS4_OFFSET        0x00a4 /* Priority Register B for Slave 4 */
#  define SAM_MATRIX_PRBS5_OFFSET        0x00ac /* Priority Register B for Slave 5 */
#  define SAM_MATRIX_PRBS6_OFFSET        0x00b4 /* Priority Register B for Slave 6 */
#  define SAM_MATRIX_PRBS7_OFFSET        0x00bc /* Priority Register B for Slave 7 */
#  define SAM_MATRIX_PRBS8_OFFSET        0x00c4 /* Priority Register B for Slave 8 */
                                                /* 0x006c8-0x00fc: Reserved */
#define SAM_MATRIX_MRCR_OFFSET           0x0100 /* Master Remap Control Register */
                                                /* 0x0104-0x010c: Reserved */
#define SAM_MATRIX_CAN0_OFFSET           0x0110 /* CAN0 Configuration Register */
#define SAM_MATRIX_CCFG_SYSIO_OFFSET     0x0114 /* System I/O Configuration Register */
                                                /* 0x0118-0x0120: Reserved */
#define SAM_MATRIX_CCFG_SMCNFCS_OFFSET   0x0124 /* SMC Chip Select NAND Flash Assignment Register */
                                                /* 0x0128-0x01e0: Reserved */
#define SAM_MATRIX_WPMR_OFFSET           0x01e4 /* Write Protect Mode Register */
#define SAM_MATRIX_WPSR_OFFSET           0x01e8 /* Write Protect Status Register */
                                                /* 0x0110-0x01fc: Reserved */

/* MATRIX register addresses ************************************************/

#define SAM_MATRIX_MCFG(n))              (SAM_MATRIX_BASE+SAM_MATRIX_MCFG_OFFSET(n))
#  define SAM_MATRIX_MCFG0               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG0_OFFSET)
#  define SAM_MATRIX_MCFG1               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG1_OFFSET)
#  define SAM_MATRIX_MCFG2               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG2_OFFSET)
#  define SAM_MATRIX_MCFG3               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG3_OFFSET)
#  define SAM_MATRIX_MCFG4               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG4_OFFSET)
#  define SAM_MATRIX_MCFG5               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG5_OFFSET)
#  define SAM_MATRIX_MCFG6               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG6_OFFSET)
#  define SAM_MATRIX_MCFG7               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG7_OFFSET)
#  define SAM_MATRIX_MCFG8               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG8_OFFSET)
#  define SAM_MATRIX_MCFG9               (SAM_MATRIX_BASE+SAM_MATRIX_MCFG9_OFFSET)
#  define SAM_MATRIX_MCFG10              (SAM_MATRIX_BASE+SAM_MATRIX_MCFG10_OFFSET)
#  define SAM_MATRIX_MCFG11              (SAM_MATRIX_BASE+SAM_MATRIX_MCFG11_OFFSET)

#define SAM_MATRIX_SCFG(n)               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG_OFFSET(n))
#  define SAM_MATRIX_SCFG0               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG0_OFFSET)
#  define SAM_MATRIX_SCFG1               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG1_OFFSET)
#  define SAM_MATRIX_SCFG2               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG2_OFFSET)
#  define SAM_MATRIX_SCFG3               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG3_OFFSET)
#  define SAM_MATRIX_SCFG4               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG4_OFFSET)
#  define SAM_MATRIX_SCFG5               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG5_OFFSET)
#  define SAM_MATRIX_SCFG6               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG6_OFFSET)
#  define SAM_MATRIX_SCFG7               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG7_OFFSET)
#  define SAM_MATRIX_SCFG8               (SAM_MATRIX_BASE+SAM_MATRIX_SCFG8_OFFSET)

#define SAM_MATRIX_PRAS(n)               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS_OFFSET(n))
#  define SAM_MATRIX_PRAS0               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS0_OFFSET)
#  define SAM_MATRIX_PRAS1               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS1_OFFSET)
#  define SAM_MATRIX_PRAS2               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS2_OFFSET)
#  define SAM_MATRIX_PRAS3               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS3_OFFSET)
#  define SAM_MATRIX_PRAS4               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS4_OFFSET)
#  define SAM_MATRIX_PRAS5               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS5_OFFSET)
#  define SAM_MATRIX_PRAS6               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS6_OFFSET)
#  define SAM_MATRIX_PRAS7               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS7_OFFSET)
#  define SAM_MATRIX_PRAS8               (SAM_MATRIX_BASE+SAM_MATRIX_PRAS8_OFFSET)

#define SAM_MATRIX_PRBS(n)               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS_OFFSET(n))
#  define SAM_MATRIX_PRBS0               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS0_OFFSET)
#  define SAM_MATRIX_PRBS1               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS1_OFFSET)
#  define SAM_MATRIX_PRBS2               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS2_OFFSET)
#  define SAM_MATRIX_PRBS3               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS3_OFFSET)
#  define SAM_MATRIX_PRBS4               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS4_OFFSET)
#  define SAM_MATRIX_PRBS5               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS5_OFFSET)
#  define SAM_MATRIX_PRBS6               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS6_OFFSET)
#  define SAM_MATRIX_PRBS7               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS7_OFFSET)
#  define SAM_MATRIX_PRBS8               (SAM_MATRIX_BASE+SAM_MATRIX_PRBS8_OFFSET)

#define SAM_MATRIX_MRCR                  (SAM_MATRIX_BASE+SAM_MATRIX_MRCR_OFFSET)
#define SAM_MATRIX_CAN0                  (SAM_MATRIX_BASE+SAM_MATRIX_CAN0_OFFSET)
#define SAM_MATRIX_CCFG_SYSIO            (SAM_MATRIX_BASE+SAM_MATRIX_CCFG_SYSIO_OFFSET)
#define SAM_MATRIX_CCFG_SMCNFCS         (SAM_MATRIX_BASE+SAM_MATRIX_CCFG_SMCNFCS_OFFSET)
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
#  define MATRIX_MCFG_ULBT_32BEAT        (5 << MATRIX_MCFG_ULBT_SHIFT) /* 32-beat Burst */
#  define MATRIX_MCFG_ULBT_64BEAT        (6 << MATRIX_MCFG_ULBT_SHIFT) /* 64-beat Burst */
#  define MATRIX_MCFG_ULBT_128BEAT       (7 << MATRIX_MCFG_ULBT_SHIFT) /* 128-beat Burst */

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
#  define MATRIX_SCFG0_FIXEDDEFMSTR(n)   ((uint32_t)(n) << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)

/* Bus Matrix Priority Registers A For Slaves */

#define MATRIX_PRAS_MPR_SHIFT(x)         ((n)<<2)  /* n=0-7 */
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
#  define MATRIX_PRAS_M5PR_SHIFT         (20)      /* Bits 20-21: Master 5 Priority */
#  define MATRIX_PRAS_M5PR_MASK          (3 << MATRIX_PRAS_M5PR_SHIFT)
#    define MATRIX_PRAS_M5PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M5PR_SHIFT)
#  define MATRIX_PRAS_M6PR_SHIFT         (24)      /* Bits 24-25: Master 6 Priority */
#  define MATRIX_PRAS_M6PR_MASK          (3 << MATRIX_PRAS_M6PR_SHIFT)
#    define MATRIX_PRAS_M6PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M6PR_SHIFT)
#  define MATRIX_PRAS_M7PR_SHIFT         (28)      /* Bits 28-29: Master 7 Priority */
#  define MATRIX_PRAS_M7PR_MASK          (3 << MATRIX_PRAS_M7PR_SHIFT)
#    define MATRIX_PRAS_M7PR(n)          ((uint32_t)(n) << MATRIX_PRAS_M7PR_SHIFT)

/* Bus Matrix Priority Registers B For Slaves */

#define MATRIX_PRBS_MPR_SHIFT(x)         (((n)-8) << 2) /* n = 8-11 */
#define MATRIX_PRBS_MPR_MASK(x)          (3 << MATRIX_PRBS_MPR_SHIFT(x))

#  define MATRIX_PRBS_M8PR_SHIFT         (0)       /* Bits 0-1:  Master 8 Priority */
#  define MATRIX_PRBS_M8PR_MASK          (3 << MATRIX_PRBS_M8PR_SHIFT)
#    define MATRIX_PRBS_M8PR(n)          ((uint32_t)(n) << MATRIX_PRBS_M8PR_SHIFT)
#  define MATRIX_PRBS_M9PR_SHIFT         (4)       /* Bits 4-5:   Master 9 Priority */
#  define MATRIX_PRBS_M9PR_MASK          (3 << MATRIX_PRBS_M9PR_SHIFT)
#    define MATRIX_PRBS_M9PR(n)          ((uint32_t)(n) << MATRIX_PRBS_M9PR_SHIFT)
#  define MATRIX_PRBS_M10PR_SHIFT         (8)       /* Bits 8-9:  Master 10 Priority */
#  define MATRIX_PRBS_M10PR_MASK          (3 << MATRIX_PRBS_M10PR_SHIFT)
#    define MATRIX_PRBS_M10PR(n)          ((uint32_t)(n) << MATRIX_PRBS_M10PR_SHIFT)
#  define MATRIX_PRBS_M11PR_SHIFT         (12)      /* Bits 12-13: Master 11 Priority */
#  define MATRIX_PRBS_M11PR_MASK          (3 << MATRIX_PRBS_M11PR_SHIFT)
#    define MATRIX_PRBS_M11PR(n)          ((uint32_t)(n) << MATRIX_PRBS_M11PR_SHIFT)

/* Master Remap Control Register */

#define MATRIX_MRCR_RCB(n)               (1 << (n)) /* n=0-11 */

#  define MATRIX_MRCR_RCB0               (1 << 0)  /* Bit 0:  Remap Command Bit for AHB Master 0 */
#  define MATRIX_MRCR_RCB1               (1 << 1)  /* Bit 1:  Remap Command Bit for AHB Master 1 */
#  define MATRIX_MRCR_RCB2               (1 << 2)  /* Bit 2:  Remap Command Bit for AHB Master 2 */
#  define MATRIX_MRCR_RCB3               (1 << 3)  /* Bit 3:  Remap Command Bit for AHB Master 3 */
#  define MATRIX_MRCR_RCB4               (1 << 4)  /* Bit 4:  Remap Command Bit for AHB Master 4 */
#  define MATRIX_MRCR_RCB5               (1 << 5)  /* Bit 5:  Remap Command Bit for AHB Master 5 */
#  define MATRIX_MRCR_RCB6               (1 << 6)  /* Bit 6:  Remap Command Bit for AHB Master 6 */
#  define MATRIX_MRCR_RCB7               (1 << 7)  /* Bit 7:  Remap Command Bit for AHB Master 7 */
#  define MATRIX_MRCR_RCB8               (1 << 8)  /* Bit 8:  Remap Command Bit for AHB Master 8 */
#  define MATRIX_MRCR_RCB9               (1 << 9)  /* Bit 9:  Remap Command Bit for AHB Master 9 */
#  define MATRIX_MRCR_RCB10              (1 << 10) /* Bit 10: Remap Command Bit for AHB Master 10 */
#  define MATRIX_MRCR_RCB11              (1 << 11) /* Bit 11: Remap Command Bit for AHB Master 11 */

/* CAN0 Configuration Register */

#define MATRIX_CAN0_RESERVED             0x000001ff /* Bits 0-9: Reserved */
#define MATRIX_CAN0_CAN0DMABA_MASK       0xffff0000 /* Bits 16-31: CAN0 DMA Base Address */

/* System I/O and CAN1 Configuration Register */

#define MATRIX_CCFG_SYSIO_SYSIO(n)       (1<<(n))  /* n=4-7, 12 */
#  define MATRIX_CCFG_SYSIO_SYSIO4       (1 << 4)  /* Bit 4:  PB4 or TDI Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO5       (1 << 5)  /* Bit 5:  PB5 or TDO/TRACESWO Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO6       (1 << 6)  /* Bit 6:  PB6 or TMS/SWDIO Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO7       (1 << 7)  /* Bit 7:  PB7 or TCK/SWCLK Assignment */
#  define MATRIX_CCFG_SYSIO_SYSIO12      (1 << 12) /* Bit 12: PB12 or ERASE Assignment */

#define MATRIX_CCFG_CAN1DMABA_MASK       0xffff0000 /* Bits 16-31: CAN1 DMA Base Address */

/* SMC Chip Select NAND Flash Assignment Register */

#define MATRIX_CCFG_SMCNFCS_SMC_NFCS(n)  (1<<(n))  /* Bit n:  SMC NAND Flash Chip Select n Assignment */
#  define MATRIX_CCFG_SMCNFCS_SMC_NFCS0  (1 << 0)  /* Bit 0:  SMC NAND Flash Chip Select 0 Assignment */
#  define MATRIX_CCFG_SMCNFCS_SMC_NFCS1  (1 << 1)  /* Bit 1:  SMC NAND Flash Chip Select 2 Assignment */
#  define MATRIX_CCFG_SMCNFCS_SMC_NFCS2  (1 << 2)  /* Bit 2:  SMC NAND Flash Chip Select 2 Assignment */
#  define MATRIX_CCFG_SMCNFCS_SMC_NFCS3  (1 << 3)  /* Bit 3:  SMC NAND Flash Chip Select 3 Assignment */
#define MATRIX_CCFG_SMCNFCS_SDRAMEN      (1 << 4)  /* Bit 4:  SDRAM Enable */

/* Write Protect Mode Register */

#define MATRIX_WPMR_WPEN                 (1 << 0)  /* Bit 0:  Write Protect Enable */
#define MATRIX_WPMR_WPKEY_SHIFT          (8)       /* Bits 8-31:   Write Protect KEY (Write-only) */
#define MATRIX_WPMR_WPKEY_MASK           (0x00ffffff << MATRIX_WPMR_WPKEY_SHIFT)
#  define MATRIX_WPMR_WPKEY              (0x004d4154 << MATRIX_WPMR_WPKEY_SHIFT)

/* Write Protect Status Register */

#define MATRIX_WPSR_WPVS                 (1 << 0)  /* Bit 0:  Enable Write Protect */
#define MATRIX_WPSR_WPVSRC_SHIFT         (8)       /* Bits 8-23:  Write Protect Violation Source */
#define MATRIX_WPSR_WPVSRC_MASK          (0xffff << MATRIX_WPSR_WPVSRC_SHIFT)

/* Masters ******************************************************************/

#define MATRIX_MSTR_CORTEXM7_1           0         /* Cortex-M7 */
#define MATRIX_MSTR_CORTEXM7_2           1         /* Cortex-M7 */
#define MATRIX_MSTR_CORTEXM7_P           2         /* Cortex-M7 Peripheral Port */
#define MATRIX_MSTR_ICM                  3         /* Integrated Check Monitor */
#define MATRIX_MSTR_XDMAC_1              4         /* XDMAC */
#define MATRIX_MSTR_XDMAC_2              5         /* XDMAC */
#define MATRIX_MSTR_ISI                  6         /* ISI DMA */
#define MATRIX_MSTR_MLB                  7         /* Media LB */
#define MATRIX_MSTR_USB                  8         /* USB DMA */
#define MATRIX_MSTR_EMAC                 9         /* Ethernet MAC DMA */
#define MATRIX_MSTR_CAN0                 10        /* CAN0 DMA */
#define MATRIX_MSTR_CAN1                 11        /* CAN1 DMA */

/* Slaves *******************************************************************/

#define MATRIX_SLAVE_ISRAM_1             0         /* Internal SRAM */
#define MATRIX_SLAVE_ISRAM_2             1         /* Internal SRAM */
#define MATRIX_SLAVE_IROM                2         /* Internal ROM */
#define MATRIX_SLAVE_IFLASH              3         /* Internal Flash */
#define MATRIX_SLAVE_USBRAM              4         /* USB High Speed Dual Port RAM (DPR) */
#define MATRIX_SLAVE_EBI                 5         /* External Bus Interface */
#define MATRIX_SLAVE_QSPI                6         /* QSPI */
#define MATRIX_SLAVE_PB                  7         /* Peripheral Bridge */
#define MATRIX_SLAVE_AHB                 8         /* AHB Slave */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_MATRIX_H */
