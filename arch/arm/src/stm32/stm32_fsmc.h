/************************************************************************************
 * arch/arm/src/stm32/stm32_fsmc.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_FSMC_H
#define __ARCH_ARM_SRC_STM32_STM32_FSMC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_FSMC_BCR_OFFSET(n)  (8*((n)-1))
#define STM32_FSMC_BCR1_OFFSET    0x0000 /* SRAM/NOR-Flash chip-select control registers 1 */
#define STM32_FSMC_BCR2_OFFSET    0x0008 /* SRAM/NOR-Flash chip-select control registers 2 */
#define STM32_FSMC_BCR3_OFFSET    0x0010 /* SRAM/NOR-Flash chip-select control registers 3 */
#define STM32_FSMC_BCR4_OFFSET    0x0018 /* SRAM/NOR-Flash chip-select control registers 4 */

#define STM32_FSMC_BTR_OFFSET(n)  (8*((n)-1)+0x0004)
#define STM32_FSMC_BTR1_OFFSET    0x0004 /* SRAM/NOR-Flash chip-select timing registers 1 */
#define STM32_FSMC_BTR2_OFFSET    0x000c /* SRAM/NOR-Flash chip-select timing registers 2 */
#define STM32_FSMC_BTR3_OFFSET    0x0014 /* SRAM/NOR-Flash chip-select timing registers 3 */
#define STM32_FSMC_BTR4_OFFSET    0x001c /* SRAM/NOR-Flash chip-select timing registers 4 */

#define STM32_FSMC_BWTR_OFFSET(n) (8*((n)-1)+0x0104)
#define STM32_FSMC_BWTR1_OFFSET   0x0104 /* SRAM/NOR-Flash write timing registers 1 */
#define STM32_FSMC_BWTR2_OFFSET   0x010c /* SRAM/NOR-Flash write timing registers 2 */
#define STM32_FSMC_BWTR3_OFFSET   0x0114 /* SRAM/NOR-Flash write timing registers 3 */
#define STM32_FSMC_BWTR4_OFFSET   0x011c /* SRAM/NOR-Flash write timing registers 4 */

#define STM32_FSMC_PCR_OFFSET(n)  (0x0020*((n)-1)+0x0040)
#define STM32_FSMC_PCR2_OFFSET    0x0060 /* NAND Flash/PC Card controller register 2 */
#define STM32_FSMC_PCR3_OFFSET    0x0080 /* NAND Flash/PC Card controller register 3 */
#define STM32_FSMC_PCR4_OFFSET    0x00a0 /* NAND Flash/PC Card controller register 4 */

#define STM32_FSMC_SR_OFFSET(n)   (0x0020*((n)-1)+0x0044)
#define STM32_FSMC_SR2_OFFSET     0x0064 /* NAND Flash/PC Card controller register 2 */
#define STM32_FSMC_SR3_OFFSET     0x0084 /* NAND Flash/PC Card controller register 3 */
#define STM32_FSMC_SR4_OFFSET     0x00a4 /* NAND Flash/PC Card controller register 4 */

#define STM32_FSMC_PMEM_OFFSET(n) (0x0020*((n)-1)+0x0048)
#define STM32_FSMC_PMEM2_OFFSET   0x0068 /* Common memory space timing register 2 */
#define STM32_FSMC_PMEM3_OFFSET   0x0088 /* Common memory space timing register 3 */
#define STM32_FSMC_PMEM4_OFFSET   0x00a8 /* Common memory space timing register 4 */

#define STM32_FSMC_PATT_OFFSET(n) (0x0020*((n)-1)+0x004c)
#define STM32_FSMC_PATT2_OFFSET   0x006c /* Attribute memory space timing register 2 */
#define STM32_FSMC_PATT3_OFFSET   0x008c /* Attribute memory space timing register 3 */
#define STM32_FSMC_PATT4_OFFSET   0x00ac /* Attribute memory space timing register 4 */

#define STM32_PIO4_OFFSET         0x00b0  /* I/O space timing register 4 */

#define STM32_FSMC_ECCR_OFFSET(n) (0x0020*((n)-1)+0x003c)
#define STM32_FSMC_ECCR2_OFFSET   0x0054 /* ECC result register 2 */
#define STM32_FSMC_ECCR3_OFFSET   0x0074 /* ECC result register 3 */

#if defined(CONFIG_STM32_STM32F429)
#  define STM32_FSMC_SDCR1_OFFSET 0x0140 /* SDRAM Control Register, Bank 0 */
#  define STM32_FSMC_SDCR2_OFFSET 0x0144 /* SDRAM Control Register, Bank 1 */

#  define STM32_FSMC_SDTR1_OFFSET 0x0148 /* SDRAM Timing Register?, Bank 0 */
#  define STM32_FSMC_SDTR2_OFFSET 0x014c /* SDRAM Timing Register?, Bank 1 */

#  define STM32_FSMC_SDCMR_OFFSET 0x0150 /* SDRAM Config register? */
#  define STM32_FSMC_SDRTR_OFFSET 0x0154 /* SDRAM Refresh Timing Register maybe? */
#  define STM32_FSMC_SDSR_OFFSET  0x0158 /* SDRAM Status Register */
#endif

/* Register Addresses ***************************************************************/

#define STM32_FSMC_BCR(n)         (STM32_FSMC_BASE+STM32_FSMC_BCR_OFFSET(n))
#define STM32_FSMC_BCR1           (STM32_FSMC_BASE+STM32_FSMC_BCR1_OFFSET )
#define STM32_FSMC_BCR2           (STM32_FSMC_BASE+STM32_FSMC_BCR2_OFFSET )
#define STM32_FSMC_BCR3           (STM32_FSMC_BASE+STM32_FSMC_BCR3_OFFSET )
#define STM32_FSMC_BCR4           (STM32_FSMC_BASE+STM32_FSMC_BCR4_OFFSET )

#define STM32_FSMC_BTR(n)         (STM32_FSMC_BASE+STM32_FSMC_BTR_OFFSET(n))
#define STM32_FSMC_BTR1           (STM32_FSMC_BASE+STM32_FSMC_BTR1_OFFSET )
#define STM32_FSMC_BTR2           (STM32_FSMC_BASE+STM32_FSMC_BTR2_OFFSET )
#define STM32_FSMC_BTR3           (STM32_FSMC_BASE+STM32_FSMC_BTR3_OFFSET )
#define STM32_FSMC_BTR4           (STM32_FSMC_BASE+STM32_FSMC_BTR4_OFFSET )

#define STM32_FSMC_BWTR(n)        (STM32_FSMC_BASE+STM32_FSMC_BWTR_OFFSET(n))
#define STM32_FSMC_BWTR1          (STM32_FSMC_BASE+STM32_FSMC_BWTR1_OFFSET )
#define STM32_FSMC_BWTR2          (STM32_FSMC_BASE+STM32_FSMC_BWTR2_OFFSET )
#define STM32_FSMC_BWTR3          (STM32_FSMC_BASE+STM32_FSMC_BWTR3_OFFSET )
#define STM32_FSMC_BWTR4          (STM32_FSMC_BASE+STM32_FSMC_BWTR4_OFFSET )

#define STM32_FSMC_PCR(n)         (STM32_FSMC_BASE+STM32_FSMC_PCR_OFFSET(n))
#define STM32_FSMC_PCR2           (STM32_FSMC_BASE+STM32_FSMC_PCR2_OFFSET )
#define STM32_FSMC_PCR3           (STM32_FSMC_BASE+STM32_FSMC_PCR3_OFFSET )
#define STM32_FSMC_PCR4           (STM32_FSMC_BASE+STM32_FSMC_PCR4_OFFSET )

#define STM32_FSMC_SR(n)          (STM32_FSMC_BASE+STM32_FSMC_SR_OFFSET(n))
#define STM32_FSMC_SR2            (STM32_FSMC_BASE+STM32_FSMC_SR2_OFFSET )
#define STM32_FSMC_SR3            (STM32_FSMC_BASE+STM32_FSMC_SR3_OFFSET )
#define STM32_FSMC_SR4            (STM32_FSMC_BASE+STM32_FSMC_SR4_OFFSET )

#define STM32_FSMC_PMEM(n)        (STM32_FSMC_BASE+STM32_FSMC_PMEM_OFFSET(n))
#define STM32_FSMC_PMEM2          (STM32_FSMC_BASE+STM32_FSMC_PMEM2_OFFSET )
#define STM32_FSMC_PMEM3          (STM32_FSMC_BASE+STM32_FSMC_PMEM3_OFFSET )
#define STM32_FSMC_PMEM4          (STM32_FSMC_BASE+STM32_FSMC_PMEM4_OFFSET )

#define STM32_FSMC_PATT(n)        (STM32_FSMC_BASE+STM32_FSMC_PATT_OFFSET(n))
#define STM32_FSMC_PATT2          (STM32_FSMC_BASE+STM32_FSMC_PATT2_OFFSET )
#define STM32_FSMC_PATT3          (STM32_FSMC_BASE+STM32_FSMC_PATT3_OFFSET )
#define STM32_FSMC_PATT4          (STM32_FSMC_BASE+STM32_FSMC_PATT4_OFFSET )

#define STM32_PIO4                (STM32_FSMC_BASE+STM32_FSMC_PIO4_OFFSET )

#define STM32_FSMC_ECCR(n)        (STM32_FSMC_BASE+STM32_FSMC_ECCR_OFFSET(n))
#define STM32_FSMC_ECCR2          (STM32_FSMC_BASE+STM32_FSMC_ECCR2_OFFSET )
#define STM32_FSMC_ECCR3          (STM32_FSMC_BASE+STM32_FSMC_ECCR3_OFFSET )

#if defined(CONFIG_STM32_STM32F429)
#  define STM32_FSMC_SDCR1        (STM32_FSMC_BASE+STM32_FSMC_SDCR1_OFFSET)
#  define STM32_FSMC_SDCR2        (STM32_FSMC_BASE+STM32_FSMC_SDCR2_OFFSET)

#  define STM32_FSMC_SDTR1        (STM32_FSMC_BASE+STM32_FSMC_SDTR1_OFFSET)
#  define STM32_FSMC_SDTR2        (STM32_FSMC_BASE+STM32_FSMC_SDTR2_OFFSET)

#  define STM32_FSMC_SDCMR        (STM32_FSMC_BASE+STM32_FSMC_SDCMR_OFFSET)
#  define STM32_FSMC_SDRTR        (STM32_FSMC_BASE+STM32_FSMC_SDRTR_OFFSET)
#  define STM32_FSMC_SDSR         (STM32_FSMC_BASE+STM32_FSMC_SDSR_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

#define FSMC_BCR_MBKEN            (1 << 0)   /* Memory bank enable bit */
#define FSMC_BCR_MUXEN            (1 << 1)   /* Address/data multiplexing enable bit */
#define FSMC_BCR_MTYP_SHIFT       (2)        /* Memory type */
#define FSMC_BCR_MTYP_MASK        (3 << FSMC_BCR_MTYP_SHIFT)
#  define FSMC_BCR_SRAM           (0 << FSMC_BCR_MTYP_SHIFT)
#  define FSMC_BCR_ROM            (0 << FSMC_BCR_MTYP_SHIFT)
#  define FSMC_BCR_PSRAM          (1 << FSMC_BCR_MTYP_SHIFT)
#  define FSMC_BCR_CRAM           (1 << FSMC_BCR_MTYP_SHIFT)
#  define FSMC_BCR_NOR            (2 << FSMC_BCR_MTYP_SHIFT)
#define FSMC_BCR_MWID_SHIFT       (4)        /* Memory data bus width */
#define FSMC_BCR_MWID_MASK        (3 <<  FSMC_BCR_MWID_SHIFT)
#  define FSMC_BCR_MWID8          (0 << FSMC_BCR_MWID_SHIFT)
#  define FSMC_BCR_MWID16         (1 << FSMC_BCR_MWID_SHIFT)
#define FSMC_BCR_FACCEN           (1 << 6)   /* Flash access enable */
#define FSMC_BCR_BURSTEN          (1 << 8)   /* Burst enable bit */
#define FSMC_BCR_WAITPOL          (1 << 9)   /* Wait signal polarity bit */
#define FSMC_BCR_WRAPMOD          (1 << 10)  /* Wrapped burst mode support */
#define FSMC_BCR_WAITCFG          (1 << 11)  /* Wait timing configuration */
#define FSMC_BCR_WREN             (1 << 12)  /* Write enable bit */
#define FSMC_BCR_WAITEN           (1 << 13)  /* Wait enable bit */
#define FSMC_BCR_EXTMOD           (1 << 14)  /* Extended mode enable */
#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define FSMC_BCR_ASYNCWAIT      (1 << 15)  /* Wait signal during asynchronous transfers */
#endif
#define FSMC_BCR_CBURSTRW         (1 << 19)  /* Write burst enable */

#define FSMC_BCR_RSTVALUE         0x000003d2

#define FSMC_BTR_ADDSET_SHIFT     (0)        /* Address setup phase duration */
#define FSMC_BTR_ADDSET_MASK      (15 << FSMC_BTR_ADDSET_SHIFT)
#  define FSMC_BTR_ADDSET(n)      ((n-1) << FSMC_BTR_ADDSET_SHIFT)  /* (n)xHCLK n=1..16 */
#define FSMC_BTR_ADDHLD_SHIFT     (4)        /* Address-hold phase duration */
#define FSMC_BTR_ADDHLD_MASK      (15 << FSMC_BTR_ADDHLD_SHIFT)
#  define FSMC_BTR_ADDHLD(n)      ((n-1) << FSMC_BTR_ADDHLD_SHIFT)  /* (n)xHCLK n=2..16*/
#define FSMC_BTR_DATAST_SHIFT     (8)        /* Data-phase duration */
#define FSMC_BTR_DATAST_MASK      (255 << FSMC_BTR_DATAST_SHIFT)
#  define FSMC_BTR_DATAST(n)      ((n-1) << FSMC_BTR_DATAST_SHIFT)  /* (n)xHCLK n=2..256 */
#define FSMC_BTR_BUSTURN_SHIFT    (16)       /* Bus turnaround phase duration */
#define FSMC_BTR_BUSTURN_MASK     (15 << FSMC_BTR1_BUSTURN_SHIFT)
#  define FSMC_BTR_BUSTURN(n)     ((n-1) << FSMC_BTR_BUSTURN_SHIFT) /* (n)xHCLK n=1..16 */
#define FSMC_BTR_CLKDIV_SHIFT     (20)       /* Clock divide ratio */
#define FSMC_BTR_CLKDIV_MASK      (15 << FSMC_BTR_CLKDIV_SHIFT)
#  define FSMC_BTR_CLKDIV(n)      ((n-1) << FSMC_BTR_CLKDIV_SHIFT)  /* (n)xHCLK n=2..16 */
#define FSMC_BTR_DATLAT_SHIFT     (24)      /* Data latency */
#define FSMC_BTR_DATLAT_MASK      (15 << FSMC_BTR_DATLAT_SHIFT)
#  define FSMC_BTR_DATLAT(n)      ((n-2) << FSMC_BTR_DATLAT_SHIFT)  /* (n)xHCLK n=2..17 */
#define FSMC_BTR_ACCMOD_SHIFT     (28)      /* Access mode */
#define FSMC_BTR_ACCMOD_MASK      (3 << FSMC_BTR_ACCMOD_SHIFT)
#  define FSMC_BTR_ACCMODA        (0 << FSMC_BTR_ACCMOD_SHIFT)
#  define FSMC_BTR_ACCMODB        (1 << FSMC_BTR_ACCMOD_SHIFT)
#  define FSMC_BTR_ACCMODC        (2 << FSMC_BTR_ACCMOD_SHIFT)
#  define FSMC_BTR_ACCMODD        (3 << FSMC_BTR_ACCMOD_SHIFT)

#define FSMC_BTR_RSTVALUE         0xffffffff

#define FSMC_BWTR_ADDSET_SHIFT    (0)        /* Address setup phase duration */
#define FSMC_BWTR_ADDSET_MASK     (15 << FSMC_BWTR_ADDSET_SHIFT)
#  define FSMC_BWTR_ADDSET(n)     ((n-1) << FSMC_BWTR_ADDSET_SHIFT)  /* (n)xHCLK n=1..16 */
#define FSMC_BWTR_ADDHLD_SHIFT    (4)        /* Address-hold phase duration */
#define FSMC_BWTR_ADDHLD_MASK     (15 << FSMC_BWTR_ADDHLD_SHIFT)
#  define FSMC_BWTR_ADDHLD(n)     ((n-1) << FSMC_BWTR_ADDHLD_SHIFT)  /* (n)xHCLK n=2..16*/
#define FSMC_BWTR_DATAST_SHIFT    (8)        /* Data-phase duration */
#define FSMC_BWTR_DATAST_MASK     (255 << FSMC_BWTR_DATAST_SHIFT)
#  define FSMC_BWTR_DATAST(n)     ((n-1) << FSMC_BWTR_DATAST_SHIFT)  /* (n)xHCLK n=2..256 */
#define FSMC_BWTR_CLKDIV_SHIFT    (20)       /* Clock divide ratio */
#define FSMC_BWTR_CLKDIV_MASK     (15 << FSMC_BWTR_CLKDIV_SHIFT)
#  define FSMC_BWTR_CLKDIV(n)     ((n-1) << FSMC_BWTR_CLKDIV_SHIFT)  /* (n)xHCLK n=2..16 */
#define FSMC_BWTR_DATLAT_SHIFT    (24)      /* Data latency */
#define FSMC_BWTR_DATLAT_MASK     (15 << FSMC_BWTR_DATLAT_SHIFT)
#  define FSMC_BWTR_DATLAT(n)     ((n-2) << FSMC_BWTR_DATLAT_SHIFT)  /* (n)xHCLK n=2..17 */
#define FSMC_BWTR_ACCMOD_SHIFT    (28)      /* Access mode */
#define FSMC_BWTR_ACCMOD_MASK     (3 << FSMC_BWTR_ACCMOD_SHIFT)
#  define FSMC_BWTR_ACCMODA       (0 << FSMC_BWTR_ACCMOD_SHIFT)
#  define FSMC_BWTR_ACCMODB       (1 << FSMC_BWTR_ACCMOD_SHIFT)
#  define FSMC_BWTR_ACCMODC       (2 << FSMC_BWTR_ACCMOD_SHIFT)
#  define FSMC_BWTR_ACCMODD       (3 << FSMC_BTR_ACCMOD_SHIFT)

#define FSMC_PCR_PWAITEN          (1 << 1)  /* Wait feature enable bit */
#define FSMC_PCR_PBKEN            (1 << 2)  /* PC Card/NAND Flash memory bank enable bit */
#define FSMC_PCR_PTYP             (1 << 3)  /* Memory type */
#define FSMC_PCR_PWID_SHIFT       (4)       /* NAND Flash databus width */
#define FSMC_PCR_PWID_MASK        (3 <<  FSMC_PCR_PWID_SHIFT)
#  define FSMC_PCR_PWID8          (0 <<  FSMC_PCR_PWID_SHIFT)
#  define FSMC_PCR_PWID16         (1 <<  FSMC_PCR_PWID_SHIFT)
#define FSMC_PCR_ECCEN            (1 << 6)  /* ECC computation logic enable bit */
#define FSMC_PCR_TCLR_SHIFT       (9)       /* CLE to RE delay */
#define FSMC_PCR_TCLR_MASK        (15 << FSMC_PCR_TCLR_SHIFT)
#  define FSMC_PCR_TCLR(n)        ((n-1) << FSMC_PCR_TCLR_SHIFT) /* (n)xHCLK n=1..16 */
#define FSMC_PCR_TAR_SHIFT        (13)      /* ALE to RE delay */
#define FSMC_PCR_TAR_MASK         (15 <<  FSMC_PCR_TAR_MASK)
#  define FSMC_PCR_TAR(n)         ((n-1) << FSMC_PCR_TAR_SHIFT)  /* (n)xHCLK n=1..16 */
#define FSMC_PCR_ECCPS_SHIFT      (17)      /* ECC page size */
#define FSMC_PCR_ECCPS_MASK       (7 << FSMC_PCR_ECCPS_SHIFT)
#  define FSMC_PCR_ECCPS256       (0 << FSMC_PCR_ECCPS_SHIFT) /* 256 bytes */
#  define FSMC_PCR_ECCPS512       (1 << FSMC_PCR_ECCPS_SHIFT) /* 512 bytes */
#  define FSMC_PCR_ECCPS1024      (2 << FSMC_PCR_ECCPS_SHIFT) /* 1024 bytes */
#  define FSMC_PCR_ECCPS2048      (3 << FSMC_PCR_ECCPS_SHIFT) /* 2048 bytes */
#  define FSMC_PCR_ECCPS4096      (4 << FSMC_PCR_ECCPS_SHIFT) /* 8192 bytes */
#  define FSMC_PCR_ECCPS8192      (5 << FSMC_PCR_ECCPS_SHIFT) /* 1024 bytes */

#define FSMC_SR_IRS               (1 << 0)  /* Interrupt Rising Edge status */
#define FSMC_SR_ILS               (1 << 1)  /* Interrupt Level status */
#define FSMC_SR_IFS               (1 << 2)  /* Interrupt Falling Edge status */
#define FSMC_SR_IREN              (1 << 3)  /* Interrupt Rising Edge detection Enable bit */
#define FSMC_SR_ILEN              (1 << 4)  /* Interrupt Level detection Enable bit */
#define FSMC_SR_IFEN              (1 << 5)  /* Interrupt Falling Edge detection Enable bit */
#define FSMC_SR_FEMPT             (1 << 6)  /* FIFO empty */

#define FSMC_PMEM_MEMSET_SHIFT    (0)       /* Common memory setup time */
#define FSMC_PMEM_MEMSET_MASK     (255 << FSMC_PMEM_MEMSET_SHIFT)
#  define FSMC_PMEM_MEMSET(n)     ((n-1) << FSMC_PMEM_MEMSET_SHIFT) /* (n)xHCLK n=1..256 */
#define FSMC_PMEM_MEMWAIT_SHIFT   (8)       /* Common memory wait time */
#define FSMC_PMEM_MEMWAIT_MASK    (255 << FSMC_PMEM_MEMWAIT_SHIFT)
#  define FSMC_PMEM_MEMWAIT(n)    ((n-1) << FSMC_PMEM_MEMWAIT_SHIFT) /* (n)xHCLK n=2..256 */
#define FSMC_PMEM_MEMHOLD_SHIFT   (16)      /* Common memoryhold time */
#define FSMC_PMEM_MEMHOLD_MASK    (255 << FSMC_PMEM_MEMHOLD_SHIFT)
#  define FSMC_PMEM_MEMHOLD(n)    ((n) <<  FSMC_PMEM_MEMHOLD_SHIFT) /* (n)xHCLK n=1..255 */
#define FSMC_PMEM_MEMHIZ_SHIFT    (24)       /* Common memory databus HiZ time */
#define FSMC_PMEM_MEMHIZ_MASK     (255 << FSMC_PMEM_MEMHIZ_SHIFT)
#  define FSMC_PMEM_MEMHIZ(n)     ((n) << FSMC_PMEM_MEMHIZ_SHIFT) /* (n)xHCLK n=0..255 */

#define FSMC_PATT_ATTSET_SHIFT    (0)       /* Attribute memory setup time */
#define FSMC_PATT_ATTSET_MASK     (255 << FSMC_PATT_ATTSET_SHIFT)
#  define FSMC_PATT_ATTSET(n)     ((n-1) << FSMC_PATT_ATTSET_SHIFT) /* (n)xHCLK n=1..256 */
#define FSMC_PATT_ATTWAIT_SHIFT   (8)       /* Attribute memory wait time */
#define FSMC_PATT_ATTWAIT_MASK    (255 << FSMC_PATT_ATTWAIT_SHIFT)
#  define FSMC_PATT_ATTWAIT(n)    ((n-1) << FSMC_PATT_ATTWAIT_SHIFT) /* (n)xHCLK n=2..256 */
#define FSMC_PATT_ATTHOLD_SHIFT   (16)      /* Attribute memory hold time */
#define FSMC_PATT_ATTHOLD_MASK    (255 << FSMC_PATT_ATTHOLD_SHIFT)
#  define FSMC_PATT_ATTHOLD(n)    ((n) <<  FSMC_PATT_ATTHOLD_SHIFT) /* (n)xHCLK n=1..255 */
#define FSMC_PATT_ATTHIZ_SHIFT    (24)       /* Attribute memory databus HiZ time */
#define FSMC_PATT_ATTHIZ_MASK     (255 << FSMC_PATT_ATTHIZ_SHIFT)
#  define FSMC_PATT_ATTHIZ(n)     ((n) << FSMC_PATT_ATTHIZ_SHIFT) /* (n)xHCLK n=0..255 */

#define FSMC_PIO4_IOSET_SHIFT     (0)       /* IOribute memory setup time */
#define FSMC_PIO4_IOSET_MASK      (255 << FSMC_PIO4_IOSET_SHIFT)
#  define FSMC_PIO4_IOSET(n)      ((n-1) << FSMC_PIO4_IOSET_SHIFT) /* (n)xHCLK n=1..256 */
#define FSMC_PIO4_IOWAIT_SHIFT    (8)       /* IOribute memory wait time */
#define FSMC_PIO4_IOWAIT_MASK     (255 << FSMC_PIO4_IOWAIT_SHIFT)
#  define FSMC_PIO4_IOWAIT(n)     ((n-1) << FSMC_PIO4_IOWAIT_SHIFT) /* (n)xHCLK n=2..256 */
#define FSMC_PIO4_IOHOLD_SHIFT    (16)      /* IOribute memory hold time */
#define FSMC_PIO4_IOHOLD_MASK     (255 << FSMC_PIO4_IOHOLD_SHIFT)
#  define FSMC_PIO4_IOHOLD(n)     ((n) <<  FSMC_PIO4_IOHOLD_SHIFT) /* (n)xHCLK n=1..255 */
#define FSMC_PIO4_IOHIZ_SHIFT     (24)       /* IOribute memory databus HiZ time */
#define FSMC_PIO4_IOHIZ_MASK      (255 << FSMC_PIO4_IOHIZ_SHIFT)
#  define FSMC_PIO4_IOHIZ(n)      ((n) << FSMC_PIO4_IOHIZ_SHIFT) /* (n)xHCLK n=0..255 */

#if defined(CONFIG_STM32_STM32F429)

#  define FSMC_SDRAM_CR_COLBITS_8                       0x00000000
#  define FSMC_SDRAM_CR_COLBITS_9                       0x00000001
#  define FSMC_SDRAM_CR_COLBITS_10                      0x00000002
#  define FSMC_SDRAM_CR_COLBITS_11                      0x00000003

#  define FSMC_SDRAM_CR_ROWBITS_11                      0x00000000
#  define FSMC_SDRAM_CR_ROWBITS_12                      0x00000004
#  define FSMC_SDRAM_CR_ROWBITS_13                      0x00000008

#  define FSMC_SDRAM_CR_WIDTH_8                         0x00000000
#  define FSMC_SDRAM_CR_WIDTH_16                        0x00000010
#  define FSMC_SDRAM_CR_WIDTH_32                        0x00000020

#  define FSMC_SDRAM_CR_BANKS_2                         0x00000000
#  define FSMC_SDRAM_CR_BANKS_4                         0x00000040

#  define FSMC_SDRAM_CR_CASLAT_1                        0x00000080
#  define FSMC_SDRAM_CR_CASLAT_2                        0x00000100
#  define FSMC_SDRAM_CR_CASLAT_3                        0x00000180

#  define FSMC_SDRAM_CR_WRITE_PROT                      0x00000200

#  define FSMC_SDRAM_CR_SDCLK_DISABLE                   0x00000000
#  define FSMC_SDRAM_CR_SDCLK_2X                        0x00000800
#  define FSMC_SDRAM_CR_SDCLK_3X                        0x00000C00

#  define FSMC_SDRAM_CR_BURST_READ                      0x00001000

#  define FSMC_SDRAM_CR_RPIPE_0                         0x00000000
#  define FSMC_SDRAM_CR_RPIPE_1                         0x00002000
#  define FSMC_SDRAM_CR_RPIPE_2                         0x00004000

#  define FSMC_SDRAM_TR_TMRD_SHIFT                      0
#  define FSMC_SDRAM_TR_TXSR_SHIFT                      4
#  define FSMC_SDRAM_TR_TRAS_SHIFT                      8
#  define FSMC_SDRAM_TR_TRC_SHIFT                       12
#  define FSMC_SDRAM_TR_TWR_SHIFT                       16
#  define FSMC_SDRAM_TR_TRP_SHIFT                       20
#  define FSMC_SDRAM_TR_TRCD_SHIFT                      24

#  define FSMC_SDRAM_MODE_CMD_NORMAL                    0
#  define FSMC_SDRAM_MODE_CMD_CLK_ENABLE                1
#  define FSMC_SDRAM_MODE_CMD_PALL                      2
#  define FSMC_SDRAM_MODE_CMD_AUTO_REFRESH              3
#  define FSMC_SDRAM_MODE_CMD_LOAD_MODE                 4
#  define FSMC_SDRAM_MODE_CMD_SELF_REFRESH              5
#  define FSMC_SDRAM_MODE_CMD_POWER_DOWN                6

#  define FSMC_SDRAM_CMD_BANK_1                         0x00000010
#  define FSMC_SDRAM_CMD_BANK_2                         0x00000008

#  define FSMC_SDRAM_AUTO_REFRESH_SHIFT                 5
#  define FSMC_SDRAM_MODEREG_SHIFT                      9

#  define FSMC_SDRAM_MODEREG_BURST_LENGTH_1             (0x0000 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_BURST_LENGTH_2             (0x0001 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_BURST_LENGTH_4             (0x0002 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_BURST_LENGTH_8             (0x0004 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      (0x0000 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     (0x0008 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_CAS_LATENCY_2              (0x0020 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_CAS_LATENCY_3              (0x0030 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_OPERATING_MODE_STANDARD    (0x0000 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED (0x0000 << FSMC_SDRAM_MODEREG_SHIFT)
#  define FSMC_SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     (0x0200 << FSMC_SDRAM_MODEREG_SHIFT)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_STM32_FSMC_H */
