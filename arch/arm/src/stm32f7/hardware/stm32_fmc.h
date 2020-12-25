/****************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32_fmc.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_FMC_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_FMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_FMC_BCR_OFFSET(n)         (8*((n)-1))
#define STM32_FMC_BCR1_OFFSET           0x0000                          /* SRAM/NOR-Flash chip-select control registers 1 */
#define STM32_FMC_BCR2_OFFSET           0x0008                          /* SRAM/NOR-Flash chip-select control registers 2 */
#define STM32_FMC_BCR3_OFFSET           0x0010                          /* SRAM/NOR-Flash chip-select control registers 3 */
#define STM32_FMC_BCR4_OFFSET           0x0018                          /* SRAM/NOR-Flash chip-select control registers 4 */

#define STM32_FMC_BTR_OFFSET(n)         (8*((n)-1)+0x0004)
#define STM32_FMC_BTR1_OFFSET           0x0004                          /* SRAM/NOR-Flash chip-select timing registers 1 */
#define STM32_FMC_BTR2_OFFSET           0x000c                          /* SRAM/NOR-Flash chip-select timing registers 2 */
#define STM32_FMC_BTR3_OFFSET           0x0014                          /* SRAM/NOR-Flash chip-select timing registers 3 */
#define STM32_FMC_BTR4_OFFSET           0x001c                          /* SRAM/NOR-Flash chip-select timing registers 4 */

#define STM32_FMC_BWTR_OFFSET(n)        (8*((n)-1)+0x0104)
#define STM32_FMC_BWTR1_OFFSET          0x0104                          /* SRAM/NOR-Flash write timing registers 1 */
#define STM32_FMC_BWTR2_OFFSET          0x010c                          /* SRAM/NOR-Flash write timing registers 2 */
#define STM32_FMC_BWTR3_OFFSET          0x0114                          /* SRAM/NOR-Flash write timing registers 3 */
#define STM32_FMC_BWTR4_OFFSET          0x011c                          /* SRAM/NOR-Flash write timing registers 4 */

#define STM32_FMC_PCR_OFFSET            0x0080                          /* NAND Flash control register */

#define STM32_FMC_SR_OFFSET             0x0084                          /* FIFO status and interrupt register */

#define STM32_FMC_PMEM_OFFSET           0x0088                          /* Common memory space timing register */

#define STM32_FMC_PATT_OFFSET           0x008c                          /* Attribute memory space timing register */

#define STM32_FMC_ECCR_OFFSET           0x0094                          /* ECC result register */

#define STM32_FMC_SDCR_OFFSET(n)        (4*((n)-1)+0x0140)
#define STM32_FMC_SDCR1_OFFSET          0x0140                          /* SDRAM Control Register, Bank 1 */
#define STM32_FMC_SDCR2_OFFSET          0x0144                          /* SDRAM Control Register, Bank 1 */

#define STM32_FMC_SDTR_OFFSET(n)        (4*((n)-1)+0x0148)
#define STM32_FMC_SDTR1_OFFSET          0x0148                          /* SDRAM Timing Register, Bank 0 */
#define STM32_FMC_SDTR2_OFFSET          0x014c                          /* SDRAM Timing Register, Bank 1 */

#define STM32_FMC_SDCMR_OFFSET          0x0150                          /* SDRAM Command mode register */
#define STM32_FMC_SDRTR_OFFSET          0x0154                          /* SDRAM Refresh Timing Register maybe */
#define STM32_FMC_SDSR_OFFSET           0x0158                          /* SDRAM Status Register */

/* Register Addresses *******************************************************/

#define STM32_FMC_BCR(n)                (STM32_FMC_BASE+STM32_FMC_BCR_OFFSET(n))
#define STM32_FMC_BCR1                  (STM32_FMC_BASE+STM32_FMC_BCR1_OFFSET )
#define STM32_FMC_BCR2                  (STM32_FMC_BASE+STM32_FMC_BCR2_OFFSET )
#define STM32_FMC_BCR3                  (STM32_FMC_BASE+STM32_FMC_BCR3_OFFSET )
#define STM32_FMC_BCR4                  (STM32_FMC_BASE+STM32_FMC_BCR4_OFFSET )

#define STM32_FMC_BTR(n)                (STM32_FMC_BASE+STM32_FMC_BTR_OFFSET(n))
#define STM32_FMC_BTR1                  (STM32_FMC_BASE+STM32_FMC_BTR1_OFFSET )
#define STM32_FMC_BTR2                  (STM32_FMC_BASE+STM32_FMC_BTR2_OFFSET )
#define STM32_FMC_BTR3                  (STM32_FMC_BASE+STM32_FMC_BTR3_OFFSET )
#define STM32_FMC_BTR4                  (STM32_FMC_BASE+STM32_FMC_BTR4_OFFSET )

#define STM32_FMC_BWTR(n)               (STM32_FMC_BASE+STM32_FMC_BWTR_OFFSET(n))
#define STM32_FMC_BWTR1                 (STM32_FMC_BASE+STM32_FMC_BWTR1_OFFSET )
#define STM32_FMC_BWTR2                 (STM32_FMC_BASE+STM32_FMC_BWTR2_OFFSET )
#define STM32_FMC_BWTR3                 (STM32_FMC_BASE+STM32_FMC_BWTR3_OFFSET )
#define STM32_FMC_BWTR4                 (STM32_FMC_BASE+STM32_FMC_BWTR4_OFFSET )

#define STM32_FMC_PCR                   (STM32_FMC_BASE+STM32_FMC_PCR_OFFSET)

#define STM32_FMC_SR                    (STM32_FMC_BASE+STM32_FMC_SR_OFFSET)

#define STM32_FMC_PMEM                  (STM32_FMC_BASE+STM32_FMC_PMEM_OFFSET)

#define STM32_FMC_PATT                  (STM32_FMC_BASE+STM32_FMC_PATT_OFFSET)

#define STM32_FMC_ECCR                  (STM32_FMC_BASE+STM32_FMC_ECCR_OFFSET)

#define STM32_FMC_SDCR1                 (STM32_FMC_BASE+STM32_FMC_SDCR1_OFFSET)
#define STM32_FMC_SDCR2                 (STM32_FMC_BASE+STM32_FMC_SDCR2_OFFSET)

#define STM32_FMC_SDTR1                 (STM32_FMC_BASE+STM32_FMC_SDTR1_OFFSET)
#define STM32_FMC_SDTR2                 (STM32_FMC_BASE+STM32_FMC_SDTR2_OFFSET)

#define STM32_FMC_SDCMR                 (STM32_FMC_BASE+STM32_FMC_SDCMR_OFFSET)
#define STM32_FMC_SDRTR                 (STM32_FMC_BASE+STM32_FMC_SDRTR_OFFSET)
#define STM32_FMC_SDSR                  (STM32_FMC_BASE+STM32_FMC_SDSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define FMC_BCR_MBKEN                   (1 << 0)                        /* Memory bank enable bit */
#define FMC_BCR_MUXEN                   (1 << 1)                        /* Address/data multiplexing enable bit */
#define FMC_BCR_MTYP_SHIFT              (2)                             /* Memory type */
#define FMC_BCR_MTYP_MASK               (3 << FMC_BCR_MTYP_SHIFT)
#  define FMC_BCR_SRAM                  (0 << FMC_BCR_MTYP_SHIFT)
#  define FMC_BCR_ROM                   (0 << FMC_BCR_MTYP_SHIFT)
#  define FMC_BCR_PSRAM                 (1 << FMC_BCR_MTYP_SHIFT)
#  define FMC_BCR_CRAM                  (1 << FMC_BCR_MTYP_SHIFT)
#  define FMC_BCR_NOR                   (2 << FMC_BCR_MTYP_SHIFT)
#define FMC_BCR_MWID_SHIFT              (4)                             /* Memory data bus width */
#define FMC_BCR_MWID_MASK               (3 <<  FMC_BCR_MWID_SHIFT)
#  define FMC_BCR_MWID8                 (0 << FMC_BCR_MWID_SHIFT)
#  define FMC_BCR_MWID16                (1 << FMC_BCR_MWID_SHIFT)
#  define FMC_BCR_MWID32                (2 << FMC_BCR_MWID_SHIFT)
#define FMC_BCR_FACCEN                  (1 << 6)                        /* Flash access enable */
#define FMC_BCR_BURSTEN                 (1 << 8)                        /* Burst enable bit */
#define FMC_BCR_WAITPOL                 (1 << 9)                        /* Wait signal polarity bit */
#define FMC_BCR_WAITCFG                 (1 << 11)                       /* Wait timing configuration */
#define FMC_BCR_WREN                    (1 << 12)                       /* Write enable bit */
#define FMC_BCR_WAITEN                  (1 << 13)                       /* Wait enable bit */
#define FMC_BCR_EXTMOD                  (1 << 14)                       /* Extended mode enable */
#define FMC_BCR_ASYNCWAIT               (1 << 15)                       /* Wait signal during asynchronous transfers */
#define FMC_BCR_CPSIZE_SHIFT            (16)
#define FMC_BCR_CPSIZE_MASK             (7 << FMC_BCR_CPSIZE_SHIFT)
#  define FMC_BCR_CPSIZE0               (0 << FMC_BCR_CPSIZE_SHIFT)
#  define FMC_BCR_CPSIZE128             (1 << FMC_BCR_CPSIZE_SHIFT)
#  define FMC_BCR_CPSIZE256             (2 << FMC_BCR_CPSIZE_SHIFT)
#  define FMC_BCR_CPSIZE1024            (3 << FMC_BCR_CPSIZE_SHIFT)
#define FMC_BCR_CBURSTRW                (1 << 19)                       /* Write burst enable */
#define FMC_BCR_CCLKEN                  (1 << 20)                       /* Continuous Clock Enable */
#define FMC_BCR_WFDIS                   (1 << 21)                       /* Write FIFO Disable */
#define FMC_BCR_BMAP_SHIFT              (24)
#define FMC_BCR_BMAP_MASK               (3 << FMC_BCR_BMAP_SHIFT)
#  define FMC_BCR_BMAP0                 (0 << FMC_BCR_BMAP_SHIFT)       /* Default mapping */
#  define FMC_BCR_BMAP1                 (1 << FMC_BCR_BMAP_SHIFT)       /* NOR/PSRAM bank and SDRAM bank 1/bank2 are swapped */
#  define FMC_BCR_BMAP2                 (2 << FMC_BCR_BMAP_SHIFT)       /* SDRAM Bank2 remapped on FMC bank2 and still accessible at default mapping */
#define FMC_BCR_FMCEN                   (1 << 31)                       /* FMC controller Enable */

#define FMC_BCR_RSTVALUE(n)             ((n == 0) ? 0x000030db : 0x000030d2) /* Reset value for BCR */

#define FMC_BTR_ADDSET_SHIFT            (0)                             /* Address setup phase duration */
#define FMC_BTR_ADDSET_MASK             (15 << FMC_BTR_ADDSET_SHIFT)
#  define FMC_BTR_ADDSET(n)             ((n-1) << FMC_BTR_ADDSET_SHIFT) /* (n)xHCLK n=1..16 */
#define FMC_BTR_ADDHLD_SHIFT            (4)                             /* Address-hold phase duration */
#define FMC_BTR_ADDHLD_MASK             (15 << FMC_BTR_ADDHLD_SHIFT)
#  define FMC_BTR_ADDHLD(n)             ((n-1) << FMC_BTR_ADDHLD_SHIFT) /* (n)xHCLK n=2..16*/
#define FMC_BTR_DATAST_SHIFT            (8)                             /* Data-phase duration */
#define FMC_BTR_DATAST_MASK             (255 << FMC_BTR_DATAST_SHIFT)
#  define FMC_BTR_DATAST(n)             ((n-1) << FMC_BTR_DATAST_SHIFT) /* (n)xHCLK n=2..256 */
#define FMC_BTR_BUSTURN_SHIFT           (16)                            /* Bus turnaround phase duration */
#define FMC_BTR_BUSTURN_MASK            (15 << FMC_BTR1_BUSTURN_SHIFT)
#  define FMC_BTR_BUSTURN(n)            ((n-1) << FMC_BTR_BUSTURN_SHIFT)/* (n)xHCLK n=1..16 */
#define FMC_BTR_CLKDIV_SHIFT            (20)                            /* Clock divide ratio */
#define FMC_BTR_CLKDIV_MASK             (15 << FMC_BTR_CLKDIV_SHIFT)
#  define FMC_BTR_CLKDIV(n)             ((n-1) << FMC_BTR_CLKDIV_SHIFT) /* (n)xHCLK n=2..16 */
#define FMC_BTR_DATLAT_SHIFT            (24)                            /* Data latency */
#define FMC_BTR_DATLAT_MASK             (15 << FMC_BTR_DATLAT_SHIFT)
#  define FMC_BTR_DATLAT(n)             ((n-2) << FMC_BTR_DATLAT_SHIFT)
                                                                        /* (n)xHCLK n=2..17 */
#define FMC_BTR_ACCMOD_SHIFT            (28)                            /* Access mode */
#define FMC_BTR_ACCMOD_MASK             (3 << FMC_BTR_ACCMOD_SHIFT)
#  define FMC_BTR_ACCMODA               (0 << FMC_BTR_ACCMOD_SHIFT)
#  define FMC_BTR_ACCMODB               (1 << FMC_BTR_ACCMOD_SHIFT)
#  define FMC_BTR_ACCMODC               (2 << FMC_BTR_ACCMOD_SHIFT)
#  define FMC_BTR_ACCMODD               (3 << FMC_BTR_ACCMOD_SHIFT)

#define FMC_BTR_RSTVALUE                0xffffffff

#define FMC_BWTR_ADDSET_SHIFT           (0)                             /* Address setup phase duration */
#define FMC_BWTR_ADDSET_MASK            (15 << FMC_BWTR_ADDSET_SHIFT)
#  define FMC_BWTR_ADDSET(n)            ((n-1) << FMC_BWTR_ADDSET_SHIFT)/* (n)xHCLK n=1..16 */
#define FMC_BWTR_ADDHLD_SHIFT           (4)                             /* Address-hold phase duration */
#define FMC_BWTR_ADDHLD_MASK            (15 << FMC_BWTR_ADDHLD_SHIFT)
#  define FMC_BWTR_ADDHLD(n)            ((n-1) << FMC_BWTR_ADDHLD_SHIFT)/* (n)xHCLK n=2..16 */
#define FMC_BWTR_DATAST_SHIFT           (8)                             /* Data-phase duration */
#define FMC_BWTR_DATAST_MASK            (255 << FMC_BWTR_DATAST_SHIFT)
#  define FMC_BWTR_DATAST(n)            ((n-1) << FMC_BWTR_DATAST_SHIFT)/* (n)xHCLK n=2..256 */
#define FMC_BWTR_CLKDIV_SHIFT           (20)                            /* Clock divide ratio */
#define FMC_BWTR_CLKDIV_MASK            (15 << FMC_BWTR_CLKDIV_SHIFT)
#  define FMC_BWTR_CLKDIV(n)            ((n-1) << FMC_BWTR_CLKDIV_SHIFT)/* (n)xHCLK n=2..16 */
#define FMC_BWTR_DATLAT_SHIFT           (24)                            /* Data latency */
#define FMC_BWTR_DATLAT_MASK            (15 << FMC_BWTR_DATLAT_SHIFT)
#  define FMC_BWTR_DATLAT(n)            ((n-2) << FMC_BWTR_DATLAT_SHIFT)/* (n)xHCLK n=2..17 */
#define FMC_BWTR_ACCMOD_SHIFT           (28)                            /* Access mode */
#define FMC_BWTR_ACCMOD_MASK            (3 << FMC_BWTR_ACCMOD_SHIFT)
#  define FMC_BWTR_ACCMODA              (0 << FMC_BWTR_ACCMOD_SHIFT)
#  define FMC_BWTR_ACCMODB              (1 << FMC_BWTR_ACCMOD_SHIFT)
#  define FMC_BWTR_ACCMODC              (2 << FMC_BWTR_ACCMOD_SHIFT)
#  define FMC_BWTR_ACCMODD              (3 << FMC_BWTR_ACCMOD_SHIFT)

#define FMC_PCR_RSTVALUE                (0x00000018)

#define FMC_PCR_PWAITEN                 (1 << 1)                        /* Wait feature enable bit */
#define FMC_PCR_PBKEN                   (1 << 2)                        /* PC Card/NAND Flash memory bank enable bit */
#define FMC_PCR_PWID_SHIFT              (4)                             /* NAND Flash databus width */
#define FMC_PCR_PWID_MASK               (3 <<  FMC_PCR_PWID_SHIFT)
#  define FMC_PCR_PWID8                 (0 <<  FMC_PCR_PWID_SHIFT)
#  define FMC_PCR_PWID16                (1 <<  FMC_PCR_PWID_SHIFT)
#define FMC_PCR_ECCEN                   (1 << 6)                        /* ECC computation logic enable bit */
#define FMC_PCR_TCLR_SHIFT              (9)                             /* CLE to RE delay */
#define FMC_PCR_TCLR_MASK               (15 << FMC_PCR_TCLR_SHIFT)
#  define FMC_PCR_TCLR(n)               ((n-1) << FMC_PCR_TCLR_SHIFT)   /* (n)xHCLK n=1..16 */
#define FMC_PCR_TAR_SHIFT               (13)                            /* ALE to RE delay */
#define FMC_PCR_TAR_MASK                (15 <<  FMC_PCR_TAR_MASK)
#  define FMC_PCR_TAR(n)                ((n-1) << FMC_PCR_TAR_SHIFT)    /* (n)xHCLK n=1..16 */
#define FMC_PCR_ECCPS_SHIFT             (17)                            /* ECC page size */
#define FMC_PCR_ECCPS_MASK              (7 << FMC_PCR_ECCPS_SHIFT)
#  define FMC_PCR_ECCPS256              (0 << FMC_PCR_ECCPS_SHIFT)      /* 256 bytes */
#  define FMC_PCR_ECCPS512              (1 << FMC_PCR_ECCPS_SHIFT)      /* 512 bytes */
#  define FMC_PCR_ECCPS1024             (2 << FMC_PCR_ECCPS_SHIFT)      /* 1024 bytes */
#  define FMC_PCR_ECCPS2048             (3 << FMC_PCR_ECCPS_SHIFT)      /* 2048 bytes */
#  define FMC_PCR_ECCPS4096             (4 << FMC_PCR_ECCPS_SHIFT)      /* 8192 bytes */
#  define FMC_PCR_ECCPS8192             (5 << FMC_PCR_ECCPS_SHIFT)      /* 1024 bytes */

#define FMC_SR_IRS                      (1 << 0)                        /* Interrupt Rising Edge status */
#define FMC_SR_ILS                      (1 << 1)                        /* Interrupt Level status */
#define FMC_SR_IFS                      (1 << 2)                        /* Interrupt Falling Edge status */
#define FMC_SR_IREN                     (1 << 3)                        /* Interrupt Rising Edge detection Enable bit */
#define FMC_SR_ILEN                     (1 << 4)                        /* Interrupt Level detection Enable bit */
#define FMC_SR_IFEN                     (1 << 5)                        /* Interrupt Falling Edge detection Enable bit */
#define FMC_SR_FEMPT                    (1 << 6)                        /* FIFO empty */

#define FMC_PMEM_RSTVALUE               (0xfcfcfcfc)

#define FMC_PMEM_MEMSET_SHIFT           (0)                             /* Common memory setup time */
#define FMC_PMEM_MEMSET_MASK            (255 << FMC_PMEM_MEMSET_SHIFT)
#  define FMC_PMEM_MEMSET(n)            ((n-1) << FMC_PMEM_MEMSET_SHIFT)/* (n)xHCLK n=1..256 */
#define FMC_PMEM_MEMWAIT_SHIFT          (8)                             /* Common memory wait time */
#define FMC_PMEM_MEMWAIT_MASK           (255 << FMC_PMEM_MEMWAIT_SHIFT)
#  define FMC_PMEM_MEMWAIT(n)           ((n-1)<<FMC_PMEM_MEMWAIT_SHIFT) /* (n)xHCLK n=2..256 */
#define FMC_PMEM_MEMHOLD_SHIFT          (16)                            /* Common memoryhold time */
#define FMC_PMEM_MEMHOLD_MASK           (255 << FMC_PMEM_MEMHOLD_SHIFT)
#  define FMC_PMEM_MEMHOLD(n)           ((n) <<  FMC_PMEM_MEMHOLD_SHIFT)/* (n)xHCLK n=1..255 */
#define FMC_PMEM_MEMHIZ_SHIFT           (24)                            /* Common memory databus HiZ time */
#define FMC_PMEM_MEMHIZ_MASK            (255 << FMC_PMEM_MEMHIZ_SHIFT)
#  define FMC_PMEM_MEMHIZ(n)            ((n) << FMC_PMEM_MEMHIZ_SHIFT)  /* (n)xHCLK n=0..255 */

#define FMC_PATT_ATTSET_SHIFT           (0)                             /* Attribute memory setup time */
#define FMC_PATT_ATTSET_MASK            (255 << FMC_PATT_ATTSET_SHIFT)
#  define FMC_PATT_ATTSET(n)            ((n-1) << FMC_PATT_ATTSET_SHIFT)/* (n)xHCLK n=1..256 */
#define FMC_PATT_ATTWAIT_SHIFT          (8)                             /* Attribute memory wait time */
#define FMC_PATT_ATTWAIT_MASK           (255 << FMC_PATT_ATTWAIT_SHIFT)
#  define FMC_PATT_ATTWAIT(n)           ((n-1)<<FMC_PATT_ATTWAIT_SHIFT) /* (n)xHCLK n=2..256 */
#define FMC_PATT_ATTHOLD_SHIFT          (16)                            /* Attribute memory hold time */
#define FMC_PATT_ATTHOLD_MASK           (255 << FMC_PATT_ATTHOLD_SHIFT)
#  define FMC_PATT_ATTHOLD(n)           ((n) <<  FMC_PATT_ATTHOLD_SHIFT)/* (n)xHCLK n=1..255 */
#define FMC_PATT_ATTHIZ_SHIFT           (24)                            /* Attribute memory databus HiZ time */
#define FMC_PATT_ATTHIZ_MASK            (255 << FMC_PATT_ATTHIZ_SHIFT)
#  define FMC_PATT_ATTHIZ(n)            ((n) << FMC_PATT_ATTHIZ_SHIFT)  /* (n)xHCLK n=0..255 */

#  define FMC_SDCR_COLBITS_SHIFT        (0)
#  define FMC_SDCR_COLBITS_MASK         (3 << FMC_SDCR_COLBITS_SHIFT)
#  define FMC_SDCR_COLBITS_8            0x00000000
#  define FMC_SDCR_COLBITS_9            0x00000001
#  define FMC_SDCR_COLBITS_10           0x00000002
#  define FMC_SDCR_COLBITS_11           0x00000003

#  define FMC_SDCR_ROWBITS_SHIFT        (2)
#  define FMC_SDCR_ROWBITS_MASK         (3 << FMC_SDCR_ROWBITS_SHIFT)
#  define FMC_SDCR_ROWBITS_11           0x00000000
#  define FMC_SDCR_ROWBITS_12           0x00000004
#  define FMC_SDCR_ROWBITS_13           0x00000008

#  define FMC_SDCR_WIDTH_SHIFT          (4)
#  define FMC_SDCR_WIDTH_MASK           (3 << FMC_SDCR_WIDTH_SHIFT)
#  define FMC_SDCR_WIDTH_8              0x00000000
#  define FMC_SDCR_WIDTH_16             0x00000010
#  define FMC_SDCR_WIDTH_32             0x00000020

#  define FMC_SDCR_BANKS_SHIFT          (6)
#  define FMC_SDCR_BANKS_MASK           (1 << FMC_SDCR_BANKS_SHIFT)
#  define FMC_SDCR_BANKS_2              0x00000000
#  define FMC_SDCR_BANKS_4              0x00000040

#  define FMC_SDCR_CASLAT_SHIFT         (7)
#  define FMC_SDCR_CASLAT_MASK          (3 << FMC_SDCR_CASLAT_SHIFT)
#  define FMC_SDCR_CASLAT_1             0x00000080
#  define FMC_SDCR_CASLAT_2             0x00000100
#  define FMC_SDCR_CASLAT_3             0x00000180

#  define FMC_SDCR_WP                   0x00000200

#  define FMC_SDCR_SDCLK_SHIFT          (10)
#  define FMC_SDCR_SDCLK_MASK           (3 << FMC_SDCR_SDCLK_SHIFT)
#  define FMC_SDCR_SDCLK_DISABLE        0x00000000
#  define FMC_SDCR_SDCLK_2X             0x00000800
#  define FMC_SDCR_SDCLK_3X             0x00000C00

#  define FMC_SDCR_BURST_READ           0x00001000

#  define FMC_SDCR_RPIPE_SHIFT          (13)
#  define FMC_SDCR_RPIPE_MASK           (3 << FMC_SDCR_RPIPE_SHIFT)
#  define FMC_SDCR_RPIPE_0              0x00000000
#  define FMC_SDCR_RPIPE_1              0x00002000
#  define FMC_SDCR_RPIPE_2              0x00004000

#define FMC_SDCR_RESERVED               (0x1FFFF << 15)                 /* reserved bits */

#define FMC_SDTR_TMRD_SHIFT             (0)
#define FMC_SDTR_TMRD_MASK              FMC_SDTR_TMRD(16)
#define FMC_SDTR_TMRD(n)                (((n)-1)<<FMC_SDTR_TMRD_SHIFT)  /* 1 to 16 cycles */
#define FMC_SDTR_TXSR_SHIFT             (4)
#define FMC_SDTR_TXSR_MASK              FMC_SDTR_TXSR(16)
#define FMC_SDTR_TXSR(n)                (((n)-1) << FMC_SDTR_TXSR_SHIFT)/* 1 to 16 cycles */
#define FMC_SDTR_TRAS_SHIFT             (8)
#define FMC_SDTR_TRAS_MASK              FMC_SDTR_TRAS(16)
#define FMC_SDTR_TRAS(n)                (((n)-1) << FMC_SDTR_TRAS_SHIFT)/* 1 to 16 cycles */
#define FMC_SDTR_TRC_SHIFT              (12)
#define FMC_SDTR_TRC_MASK               FMC_SDTR_TRC(16)
#define FMC_SDTR_TRC(n)                 (((n)-1) << FMC_SDTR_TRC_SHIFT) /* 1 to 16 cycles */
#define FMC_SDTR_TWR_SHIFT              (16)
#define FMC_SDTR_TWR_MASK               FMC_SDTR_TWR(16)
#define FMC_SDTR_TWR(n)                 (((n)-1) << FMC_SDTR_TWR_SHIFT) /* 1 to 16 cycles */
#define FMC_SDTR_TRP_SHIFT              (20)
#define FMC_SDTR_TRP_MASK               FMC_SDTR_TRP(16)
#define FMC_SDTR_TRP(n)                 (((n)-1) << FMC_SDTR_TRP_SHIFT) /* 1 to 16 cycles */
#define FMC_SDTR_TRCD_SHIFT             (24)
#define FMC_SDTR_TRCD_MASK              FMC_SDTR_TRCD(16)
#define FMC_SDTR_TRCD(n)                (((n)-1) << FMC_SDTR_TRCD_SHIFT)/* 1 to 16 cycles */

#define FMC_SDTR_RESERVED               (0xF << 28)                     /* reserved bits */

#define FMC_SDCMR_MODE_SHIFT            (0)
#define FMC_SDCMR_MODE_MASK             (7 << FMC_SDCMR_MODE_SHIFT)
#  define FMC_SDCMR_MODE_NORMAL         (0 << FMC_SDCMR_MODE_SHIFT)
#  define FMC_SDCMR_MODE_CLK_ENABLE     (1 << FMC_SDCMR_MODE_SHIFT)
#  define FMC_SDCMR_MODE_PALL           (2 << FMC_SDCMR_MODE_SHIFT)
#  define FMC_SDCMR_MODE_AUTO_REFRESH   (3 << FMC_SDCMR_MODE_SHIFT)
#  define FMC_SDCMR_MODE_LOAD_MODE      (4 << FMC_SDCMR_MODE_SHIFT)
#  define FMC_SDCMR_MODE_SELF_REFRESH   (5 << FMC_SDCMR_MODE_SHIFT)
#  define FMC_SDCMR_MODE_POWER_DOWN     (6 << FMC_SDCMR_MODE_SHIFT)
#define FMC_SDCMR_CTB2                  (1 << 3)
#define FMC_SDCMR_CTB1                  (1 << 4)
#define FMC_SDCMR_NRFS_SHIFT            (5)
#define FMC_SDCMR_NRFS(n)               (((n)-1)<<FMC_SDCMR_NRFS_SHIFT) /* 1 to 16 cycles */
#define FMC_SDCMR_MRD_SHIFT             (9)
#define FMC_SDCMR_MRD_MASK              FMC_SDCMR_MRD(16383)
#define FMC_SDCMR_MRD(n)                ((n) << FMC_SDCMR_MRD_SHIFT)    /* SDRAM Mode Register */
#  define FMC_SDCMR_MRD_BURST_LENGTH_1             FMC_SDCMR_MRD(0x0000)
#  define FMC_SDCMR_MRD_BURST_LENGTH_2             FMC_SDCMR_MRD(0x0001)
#  define FMC_SDCMR_MRD_BURST_LENGTH_4             FMC_SDCMR_MRD(0x0002)
#  define FMC_SDCMR_MRD_BURST_LENGTH_8             FMC_SDCMR_MRD(0x0004)
#  define FMC_SDCMR_MRD_BURST_TYPE_SEQUENTIAL      FMC_SDCMR_MRD(0x0000)
#  define FMC_SDCMR_MRD_BURST_TYPE_INTERLEAVED     FMC_SDCMR_MRD(0x0008)
#  define FMC_SDCMR_MRD_CAS_LATENCY_2              FMC_SDCMR_MRD(0x0020)
#  define FMC_SDCMR_MRD_CAS_LATENCY_3              FMC_SDCMR_MRD(0x0030)
#  define FMC_SDCMR_MRD_OPERATING_MODE_STANDARD    FMC_SDCMR_MRD(0x0000)
#  define FMC_SDCMR_MRD_WRITEBURST_MODE_PROGRAMMED FMC_SDCMR_MRD(0x0000)
#  define FMC_SDCMR_MRD_WRITEBURST_MODE_SINGLE     FMC_SDCMR_MRD(0x0200)

#define FMC_SDCMR_RESERVED              (0x1ff << 23)                   /* reserved bits */

#define FMC_SDRTR_CRE                   (1 << 0)
#define FMC_SDRTR_COUNT_SHIFT           (1)
#define FMC_SDRTR_COUNT_MASK            FMC_SDRTR_COUNT(0x1fff)
#define FMC_SDRTR_COUNT(val)            ((val) << FMC_SDRTR_COUNT_SHIFT)
#define FMC_SDRTR_REIE                  (14 << 0)

#define FMC_SDSR_RE                     (1 << 0)
#define FMC_SDSR_MODES1_SHIFT           (1)
#define FMC_SDSR_MODES1_MASK            FMC_SDSR_MODES1(3)
#define FMC_SDSR_MODES1(n)              ((n) << FMC_SDSR_MODES1_SHIFT)
#  define FMC_SDSR_MODES1_NORMAL        FMC_SDSR_MODES1(FMC_SDSR_MODES_NORMAL)
#  define FMC_SDSR_MODES1_SELF_REFRESH  FMC_SDSR_MODES1(FMC_SDSR_MODES_SELF_REFRESH)
#  define FMC_SDSR_MODES1_POWER_DOWN    FMC_SDSR_MODES1(FMC_SDSR_MODES_POWER_DOWN)
#define FMC_SDSR_MODES2_SHIFT           (3)
#define FMC_SDSR_MODES2_MASK            FMC_SDSR_MODES2(3)
#define FMC_SDSR_MODES2(n)              ((n) << FMC_SDSR_MODES2_SHIFT)
#  define FMC_SDSR_MODES2_NORMAL        FMC_SDSR_MODES2(FMC_SDSR_MODES_NORMAL)
#  define FMC_SDSR_MODES2_SELF_REFRESH  FMC_SDSR_MODES2(FMC_SDSR_MODES_SELF_REFRESH)
#  define FMC_SDSR_MODES2_POWER_DOWN    FMC_SDSR_MODES2(FMC_SDSR_MODES_POWER_DOWN)
#define FMC_SDSR_MODES_NORMAL           (0)
#define FMC_SDSR_MODES_SELF_REFRESH     (1)
#define FMC_SDSR_MODES_POWER_DOWN       (2)
#define FMC_SDSR_BUSY                   (1 << 5)

#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_FMC_H */
