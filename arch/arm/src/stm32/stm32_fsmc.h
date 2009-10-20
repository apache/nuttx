/************************************************************************************
 * arch/arm/src/stm32/stm32_fsmc.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>
#include "chip.h"

/************************************************************************************
 * Definitions
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

/* Register Addresses ***************************************************************/

#define STM32_FSMC_BCR(n)         (STM32_FSMC_BASE+STM32_FSMC_BCR_OFFSET (n))
#define STM32_FSMC_BCR1           (STM32_FSMC_BASE+STM32_FSMC_BCR1_OFFSET )
#define STM32_FSMC_BCR2           (STM32_FSMC_BASE+STM32_FSMC_BCR2_OFFSET )
#define STM32_FSMC_BCR3           (STM32_FSMC_BASE+STM32_FSMC_BCR3_OFFSET )
#define STM32_FSMC_BCR4           (STM32_FSMC_BASE+STM32_FSMC_BCR4_OFFSET )

#define STM32_FSMC_BTR(n)         (STM32_FSMC_BASE+STM32_FSMC_BTR_OFFSET (n))
#define STM32_FSMC_BTR1           (STM32_FSMC_BASE+STM32_FSMC_BTR1_OFFSET )
#define STM32_FSMC_BTR2           (STM32_FSMC_BASE+STM32_FSMC_BTR2_OFFSET )
#define STM32_FSMC_BTR3           (STM32_FSMC_BASE+STM32_FSMC_BTR3_OFFSET )
#define STM32_FSMC_BTR4           (STM32_FSMC_BASE+STM32_FSMC_BTR4_OFFSET )

#define STM32_FSMC_BWTR(n)        (STM32_FSMC_BASE+STM32_FSMC_BWTR_OFFSET (n))
#define STM32_FSMC_BWTR1          (STM32_FSMC_BASE+STM32_FSMC_BWTR1_OFFSET )
#define STM32_FSMC_BWTR2          (STM32_FSMC_BASE+STM32_FSMC_BWTR2_OFFSET )
#define STM32_FSMC_BWTR3          (STM32_FSMC_BASE+STM32_FSMC_BWTR3_OFFSET )
#define STM32_FSMC_BWTR4          (STM32_FSMC_BASE+STM32_FSMC_BWTR4_OFFSET )

#define STM32_FSMC_PCR(n)         (STM32_FSMC_BASE+STM32_FSMC_PCR_OFFSET (n))
#define STM32_FSMC_PCR2           (STM32_FSMC_BASE+STM32_FSMC_PCR2_OFFSET )
#define STM32_FSMC_PCR3           (STM32_FSMC_BASE+STM32_FSMC_PCR3_OFFSET )
#define STM32_FSMC_PCR4           (STM32_FSMC_BASE+STM32_FSMC_PCR4_OFFSET )

#define STM32_FSMC_SR(n)          (STM32_FSMC_BASE+STM32_FSMC_SR_OFFSET (n))
#define STM32_FSMC_SR2            (STM32_FSMC_BASE+STM32_FSMC_SR2_OFFSET )
#define STM32_FSMC_SR3            (STM32_FSMC_BASE+STM32_FSMC_SR3_OFFSET )
#define STM32_FSMC_SR4            (STM32_FSMC_BASE+STM32_FSMC_SR4_OFFSET )

#define STM32_FSMC_PMEM(n)        (STM32_FSMC_BASE+STM32_FSMC_PMEM_OFFSET (n))
#define STM32_FSMC_PMEM2          (STM32_FSMC_BASE+STM32_FSMC_PMEM2_OFFSET )
#define STM32_FSMC_PMEM3          (STM32_FSMC_BASE+STM32_FSMC_PMEM3_OFFSET )
#define STM32_FSMC_PMEM4          (STM32_FSMC_BASE+STM32_FSMC_PMEM4_OFFSET )

#define STM32_FSMC_PATT(n)        (STM32_FSMC_BASE+STM32_FSMC_PATT_OFFSET (n))
#define STM32_FSMC_PATT2          (STM32_FSMC_BASE+STM32_FSMC_PATT2_OFFSET )
#define STM32_FSMC_PATT3          (STM32_FSMC_BASE+STM32_FSMC_PATT3_OFFSET )
#define STM32_FSMC_PATT4          (STM32_FSMC_BASE+STM32_FSMC_PATT4_OFFSET )

#define STM32_PIO4                (STM32_FSMC_BASE+STM32_FSMC_PIO4_OFFSET )

#define STM32_FSMC_ECCR(n)        (STM32_FSMC_BASE+STM32_FSMC_ECCR_OFFSET (n))
#define STM32_FSMC_ECCR2          (STM32_FSMC_BASE+STM32_FSMC_ECCR2_OFFSET )
#define STM32_FSMC_ECCR3          (STM32_FSMC_BASE+STM32_FSMC_ECCR3_OFFSET )

/* Register Bitfield Definitions ****************************************************/

#define FSMC_BCR1_MBKEN          (1 << 0)   /* Memory bank enable bit */
#define FSMC_BCR1_MUXEN          (1 << 1)   /* Address/data multiplexing enable bit */
#define FSMC_BCR1_MTYP_MASK      0x0000000c /* Memory type */
#  define FSMC_BCR1_MTYP0        (1 << 2)
#  define FSMC_BCR1_MTYP1        (1 << 3)
#define FSMC_BCR1_MWID_MASK      0x00000030 /* Memory data bus width */
#  define FSMC_BCR1_MWID0        (1 << 4)
#  define FSMC_BCR1_MWID1        (1 << 5)
#define FSMC_BCR1_FACCEN         (1 << 6)   /* Flash access enable */
#define FSMC_BCR1_BURSTEN        (1 << 8)   /* Burst enable bit */
#define FSMC_BCR1_WAITPOL        (1 << 9)   /* Wait signal polarity bit */
#define FSMC_BCR1_WRAPMOD        (1 << 10)  /* Wrapped burst mode support */
#define FSMC_BCR1_WAITCFG        (1 << 11)  /* Wait timing configuration */
#define FSMC_BCR1_WREN           (1 << 12)  /* Write enable bit */
#define FSMC_BCR1_WAITEN         (1 << 13)  /* Wait enable bit */
#define FSMC_BCR1_EXTMOD         (1 << 14)  /* Extended mode enable */
#define FSMC_BCR1_CBURSTRW       (1 << 19)  /* Write burst enable */

#define FSMC_BCR2_MBKEN          (1 << 0)   /* Memory bank enable bit */
#define FSMC_BCR2_MUXEN          (1 << 1)   /* Address/data multiplexing enable bit */
#define FSMC_BCR2_MTYP_MASK      0x0000000c /* Memory type */
#  define FSMC_BCR2_MTYP0        (1 << 2)
#  define FSMC_BCR2_MTYP1        (1 << 3)
#define FSMC_BCR2_MWID_MASK      0x00000030 /* Memory data bus width */
#  define FSMC_BCR2_MWID0        (1 << 4)
#  define FSMC_BCR2_MWID1        (1 << 5)
#define FSMC_BCR2_FACCEN         (1 << 6)   /* Flash access enable */
#define FSMC_BCR2_BURSTEN        (1 << 8)   /* Burst enable bit */
#define FSMC_BCR2_WAITPOL        (1 << 9)   /* Wait signal polarity bit */
#define FSMC_BCR2_WRAPMOD        (1 << 10)  /* Wrapped burst mode support */
#define FSMC_BCR2_WAITCFG        (1 << 11)  /* Wait timing configuration */
#define FSMC_BCR2_WREN           (1 << 12)  /* Write enable bit */
#define FSMC_BCR2_WAITEN         (1 << 13)  /* Wait enable bit */
#define FSMC_BCR2_EXTMOD         (1 << 14)  /* Extended mode enable */
#define FSMC_BCR2_CBURSTRW       (1 << 19)  /* Write burst enable */

#define FSMC_BCR3_MBKEN          (1 << 0)   /* Memory bank enable bit */
#define FSMC_BCR3_MUXEN          (1 << 1)   /* Address/data multiplexing enable bit */
#define FSMC_BCR3_MTYP_MASK      0x0000000c /* Memory type */
#  define FSMC_BCR3_MTYP0        (1 << 2)
#  define FSMC_BCR3_MTYP1        (1 << 3)
#define FSMC_BCR3_MWID_MASK      0x00000030 /* Memory data bus width */
#  define FSMC_BCR3_MWID0        (1 << 4)
#  define FSMC_BCR3_MWID1        (1 << 5)
#define FSMC_BCR3_FACCEN         (1 << 6)   /* Flash access enable */
#define FSMC_BCR3_BURSTEN        (1 << 8)   /* Burst enable bit */
#define FSMC_BCR3_WAITPOL        (1 << 9)   /* Wait signal polarity bit. */
#define FSMC_BCR3_WRAPMOD        (1 << 10)  /* Wrapped burst mode support */
#define FSMC_BCR3_WAITCFG        (1 << 11)  /* Wait timing configuration */
#define FSMC_BCR3_WREN           (1 << 12)  /* Write enable bit */
#define FSMC_BCR3_WAITEN         (1 << 13)  /* Wait enable bit */
#define FSMC_BCR3_EXTMOD         (1 << 14)  /* Extended mode enable */
#define FSMC_BCR3_CBURSTRW       (1 << 19)  /* Write burst enable */

#define FSMC_BCR4_MBKEN          (1 << 0)   /* Memory bank enable bit */
#define FSMC_BCR4_MUXEN          (1 << 1)   /* Address/data multiplexing enable bit */
#define FSMC_BCR4_MTYP_MASK      0x0000000c /* Memory type */
#  define FSMC_BCR4_MTYP0        (1 << 2)
#  define FSMC_BCR4_MTYP1        (1 << 3)
#define FSMC_BCR4_MWID_MASK      0x00000030 /* Memory data bus width */
#  define FSMC_BCR4_MWID0        (1 << 4)
#  define FSMC_BCR4_MWID1        (1 << 5)
#define FSMC_BCR4_FACCEN         (1 << 6)   /* Flash access enable */
#define FSMC_BCR4_BURSTEN        (1 << 8)   /* Burst enable bit */
#define FSMC_BCR4_WAITPOL        (1 << 9)   /* Wait signal polarity bit */
#define FSMC_BCR4_WRAPMOD        (1 << 10)  /* Wrapped burst mode support */
#define FSMC_BCR4_WAITCFG        (1 << 11)  /* Wait timing configuration */
#define FSMC_BCR4_WREN           (1 << 12)  /* Write enable bit */
#define FSMC_BCR4_WAITEN         (1 << 13)  /* Wait enable bit */
#define FSMC_BCR4_EXTMOD         (1 << 14)  /* Extended mode enable */
#define FSMC_BCR4_CBURSTRW       (1 << 19)  /* Write burst enable */

#define FSMC_BTR1_ADDSET_MASK    0x0000000f /* Address setup phase duration */
#  define FSMC_BTR1_ADDSET0      (1 << 0)
#  define FSMC_BTR1_ADDSET1      (1 << 1)
#  define FSMC_BTR1_ADDSET2      (1 << 2)
#  define FSMC_BTR1_ADDSET3      (1 << 3)
#define FSMC_BTR1_ADDHLD_MASK    0x000000f0 /* Address-hold phase duration */
#  define FSMC_BTR1_ADDHLD0      (1 << 4)
#  define FSMC_BTR1_ADDHLD1      (1 << 5)
#  define FSMC_BTR1_ADDHLD2      (1 << 6)
#  define FSMC_BTR1_ADDHLD3      (1 << 7)
#define FSMC_BTR1_DATAST_MASK    0x0000ff00 /* Data-phase duration */
#  define FSMC_BTR1_DATAST0      (1 << 8)
#  define FSMC_BTR1_DATAST1      (1 << 9)
#  define FSMC_BTR1_DATAST2      (1 << 10)
#  define FSMC_BTR1_DATAST3      (1 << 11)
#define FSMC_BTR1_BUSTURN_MASK   0x000f0000 /* Bus turnaround phase duration */
#  define FSMC_BTR1_BUSTURN0     (1 << 16)
#  define FSMC_BTR1_BUSTURN1     (1 << 17)
#  define FSMC_BTR1_BUSTURN2     (1 << 18)
#  define FSMC_BTR1_BUSTURN3     (1 << 19)
#define FSMC_BTR1_CLKDIV_MASK    0x00f00000 /* Clock divide ratio */
#  define FSMC_BTR1_CLKDIV0      (1 << 20)
#  define FSMC_BTR1_CLKDIV1      (1 << 21)
#  define FSMC_BTR1_CLKDIV2      (1 << 22)
#  define FSMC_BTR1_CLKDIV3      (1 << 23)
#define FSMC_BTR1_DATLAT_MASK    0x0f000000 /* Data latency */
#  define FSMC_BTR1_DATLAT0      (1 << 24)
#  define FSMC_BTR1_DATLAT1      (1 << 25)
#  define FSMC_BTR1_DATLAT2      (1 << 26)
#  define FSMC_BTR1_DATLAT3      (1 << 27)
#define FSMC_BTR1_ACCMOD_MASK    0x30000000 /* Access mode */
#  define FSMC_BTR1_ACCMOD0      (1 << 28)
#  define FSMC_BTR1_ACCMOD1      (1 << 29)

#define FSMC_BTR2_ADDSET_MASK    0x0000000f /* Address setup phase duration */
#  define FSMC_BTR2_ADDSET0      (1 << 0)
#  define FSMC_BTR2_ADDSET1      (1 << 1)
#  define FSMC_BTR2_ADDSET2      (1 << 2)
#  define FSMC_BTR2_ADDSET3      (1 << 3)
#define FSMC_BTR2_ADDHLD_MASK    0x000000f0 /* Address-hold phase duration */
#  define FSMC_BTR2_ADDHLD0      (1 << 4)
#  define FSMC_BTR2_ADDHLD1      (1 << 5)
#  define FSMC_BTR2_ADDHLD2      (1 << 6)
#  define FSMC_BTR2_ADDHLD3      (1 << 7)
#define FSMC_BTR2_DATAST_MASK    0x0000ff00 /* Data-phase duration */
#  define FSMC_BTR2_DATAST0      (1 << 8)
#  define FSMC_BTR2_DATAST1      (1 << 9)
#  define FSMC_BTR2_DATAST2      (1 << 10)
#  define FSMC_BTR2_DATAST3      (1 << 11)
#define FSMC_BTR2_BUSTURN_MASK   0x000f0000 /* Bus turnaround phase duration */
#  define FSMC_BTR2_BUSTURN0     (1 << 16)
#  define FSMC_BTR2_BUSTURN1     (1 << 17)
#  define FSMC_BTR2_BUSTURN2     (1 << 18)
#  define FSMC_BTR2_BUSTURN3     (1 << 19)
#  define FSMC_BTR2_CLKDIV_MASK  0x00f00000 /* Clock divide ratio */
#  define FSMC_BTR2_CLKDIV0      (1 << 20)
#  define FSMC_BTR2_CLKDIV1      (1 << 21)
#  define FSMC_BTR2_CLKDIV2      (1 << 22)
#  define FSMC_BTR2_CLKDIV3      (1 << 23)
#define FSMC_BTR2_DATLAT_MASK    0x0f000000 /* Data latency */
#  define FSMC_BTR2_DATLAT0      (1 << 24)
#  define FSMC_BTR2_DATLAT1      (1 << 25)
#  define FSMC_BTR2_DATLAT2      (1 << 26)
#  define FSMC_BTR2_DATLAT3      (1 << 27)
#define FSMC_BTR2_ACCMOD_MASK    0x30000000 /* Access mode */
#  define FSMC_BTR2_ACCMOD0      (1 << 28)
#  define FSMC_BTR2_ACCMOD1      (1 << 29)

#define FSMC_BTR3_ADDSET_MASK    0x0000000f /* Address setup phase duration */
#  define FSMC_BTR3_ADDSET0      (1 << 0)
#  define FSMC_BTR3_ADDSET1      (1 << 1)
#  define FSMC_BTR3_ADDSET2      (1 << 2)
#  define FSMC_BTR3_ADDSET3      (1 << 3)
#define FSMC_BTR3_ADDHLD_MASK    0x000000f0 /* Address-hold phase duration */
#  define FSMC_BTR3_ADDHLD0      (1 << 4)
#  define FSMC_BTR3_ADDHLD1      (1 << 5)
#  define FSMC_BTR3_ADDHLD2      (1 << 6)
#  define FSMC_BTR3_ADDHLD3      (1 << 7)
#define FSMC_BTR3_DATAST_MASK    0x0000ff00 /* Data-phase duration */
#  define FSMC_BTR3_DATAST0      (1 << 8)
#  define FSMC_BTR3_DATAST1      (1 << 9)
#  define FSMC_BTR3_DATAST2      (1 << 10)
#  define FSMC_BTR3_DATAST3      (1 << 11)
#define FSMC_BTR3_BUSTURN_MASK   0x000f0000 /* Bus turnaround phase duration */
#  define FSMC_BTR3_BUSTURN0     (1 << 16)
#  define FSMC_BTR3_BUSTURN1     (1 << 17)
#  define FSMC_BTR3_BUSTURN2     (1 << 18)
#  define FSMC_BTR3_BUSTURN3     (1 << 19)
#define FSMC_BTR3_CLKDIV_MASK    0x00f00000 /* Clock divide ratio */
#  define FSMC_BTR3_CLKDIV0      (1 << 20)
#  define FSMC_BTR3_CLKDIV1      (1 << 21)
#  define FSMC_BTR3_CLKDIV2      (1 << 22)
#  define FSMC_BTR3_CLKDIV3      (1 << 23)
#define FSMC_BTR3_DATLAT_MASK    0x0f000000 /* Data latency */
#  define FSMC_BTR3_DATLAT0      (1 << 24)
#  define FSMC_BTR3_DATLAT1      (1 << 25)
#  define FSMC_BTR3_DATLAT2      (1 << 26)
#  define FSMC_BTR3_DATLAT3      (1 << 27)
#define FSMC_BTR3_ACCMOD_MASK    0x30000000 /* Access mode */
#  define FSMC_BTR3_ACCMOD0      (1 << 28)
#  define FSMC_BTR3_ACCMOD1      (1 << 29)

#define FSMC_BTR4_ADDSET_MASK    0x0000000f /* Address setup phase duration */
#  define FSMC_BTR4_ADDSET0     (1 << 0)
#  define FSMC_BTR4_ADDSET1     (1 << 1)
#  define FSMC_BTR4_ADDSET2     (1 << 2)
#  define FSMC_BTR4_ADDSET3     (1 << 3)
#define FSMC_BTR4_ADDHLD_MASK    0x000000f0 /* Address-hold phase duration */
#  define FSMC_BTR4_ADDHLD0     (1 << 4)
#  define FSMC_BTR4_ADDHLD1     (1 << 5)
#  define FSMC_BTR4_ADDHLD2     (1 << 6)
#  define FSMC_BTR4_ADDHLD3     (1 << 7)
#define FSMC_BTR4_DATAST_MASK   0x0000ff00 /* Data-phase duration */
#  define FSMC_BTR4_DATAST0     (1 << 8)
#  define FSMC_BTR4_DATAST1     (1 << 9)
#  define FSMC_BTR4_DATAST2     (1 << 10)
#  define FSMC_BTR4_DATAST3     (1 << 11)
#define FSMC_BTR4_BUSTURN_MASK  0x000f0000 /* Bus turnaround phase duration */
#  define FSMC_BTR4_BUSTURN0    (1 << 16)
#  define FSMC_BTR4_BUSTURN1    (1 << 17)
#  define FSMC_BTR4_BUSTURN2    (1 << 18)
#  define FSMC_BTR4_BUSTURN3    (1 << 19)
#define FSMC_BTR4_CLKDIV_MASK   0x00f00000 /* Clock divide ratio */
#  define FSMC_BTR4_CLKDIV0     (1 << 20)
#  define FSMC_BTR4_CLKDIV1     (1 << 21)
#  define FSMC_BTR4_CLKDIV2     (1 << 22)
#  define FSMC_BTR4_CLKDIV3     (1 << 23)
#define FSMC_BTR4_DATLAT_MASK   0x0f000000 /* Data latency */
#  define FSMC_BTR4_DATLAT0     (1 << 24)
#  define FSMC_BTR4_DATLAT1     (1 << 25)
#  define FSMC_BTR4_DATLAT2     (1 << 26)
#  define FSMC_BTR4_DATLAT3     (1 << 27)
#define FSMC_BTR4_ACCMOD_MASK   0x30000000 /* Access mode */
#  define FSMC_BTR4_ACCMOD0     (1 << 28)
#  define FSMC_BTR4_ACCMOD1     (1 << 29)

#define FSMC_BWTR1_ADDSET_MASK  0x0000000f /* Address setup phase duration */
#  define FSMC_BWTR1_ADDSET0    (1 << 0)
#  define FSMC_BWTR1_ADDSET1    (1 << 1)
#  define FSMC_BWTR1_ADDSET2    (1 << 2)
#  define FSMC_BWTR1_ADDSET3    (1 << 3)
#define FSMC_BWTR1_ADDHLD_MASK  0x000000f0 /* Address-hold phase duration */
#  define FSMC_BWTR1_ADDHLD0    (1 << 4)
#  define FSMC_BWTR1_ADDHLD1    (1 << 5)
#  define FSMC_BWTR1_ADDHLD2    (1 << 6)
#  define FSMC_BWTR1_ADDHL3     (1 << 7)
#define FSMC_BWTR1_DATAST_MASK  0x0000ff00 /* Data-phase duration */
#  define FSMC_BWTR1_DATAST0    (1 << 8)
#  define FSMC_BWTR1_DATAST1    (1 << 9)
#  define FSMC_BWTR1_DATAST2    (1 << 10)
#  define FSMC_BWTR1_DATAST3    (1 << 11)
#define FSMC_BWTR1_CLKDIV_MASK  0x00f00000 /* Clock divide ratio */
#  define FSMC_BWTR1_CLKDIV0    (1 << 20)
#  define FSMC_BWTR1_CLKDIV1    (1 << 21)
#  define FSMC_BWTR1_CLKDIV2    (1 << 22)
#  define FSMC_BWTR1_CLKDIV3    (1 << 23)
#define FSMC_BWTR1_DATLAT_MASK  0x0f000000 /* Data latency */
#  define FSMC_BWTR1_DATLAT0    (1 << 24)
#  define FSMC_BWTR1_DATLAT1    (1 << 25)
#  define FSMC_BWTR1_DATLAT2    (1 << 26)
#  define FSMC_BWTR1_DATLAT3    (1 << 27)
#define FSMC_BWTR1_ACCMOD_MASK  0x30000000 /* Access mode */
#  define FSMC_BWTR1_ACCMOD0    (1 << 28)
#  define FSMC_BWTR1_ACCMOD1    (1 << 29)

#define FSMC_BWTR2_ADDSET_MASK  0x0000000f /* Address setup phase duration */
#  define FSMC_BWTR2_ADDSET0    (1 << 0)
#  define FSMC_BWTR2_ADDSET1    (1 << 1)
#  define FSMC_BWTR2_ADDSET2    (1 << 2)
#  define FSMC_BWTR2_ADDSET3    (1 << 3)
#define FSMC_BWTR2_ADDHLD_MASK  0x000000f0 /* Address-hold phase duration */
#  define FSMC_BWTR2_ADDHLD0    (1 << 4)
#  define FSMC_BWTR2_ADDHLD1    (1 << 5)
#  define FSMC_BWTR2_ADDHLD2    (1 << 6)
#  define FSMC_BWTR2_ADDHLD3    (1 << 7)
#define FSMC_BWTR2_DATAST_MASK  0x0000ff00 /* Data-phase duration */
#  define FSMC_BWTR2_DATAST0    (1 << 8)
#  define FSMC_BWTR2_DATAST1    (1 << 9)
#  define FSMC_BWTR2_DATAST2    (1 << 10)
#  define FSMC_BWTR2_DATAST3    (1 << 11)
#define FSMC_BWTR2_CLKDIV_MASK  0x00f00000 /* Clock divide ratio */
#  define FSMC_BWTR2_CLKDIV0    (1 << 20)
#  define FSMC_BWTR2_CLKDIV1    (1 << 21)
#  define FSMC_BWTR2_CLKDIV2    (1 << 22)
#  define FSMC_BWTR2_CLKDIV3    (1 << 23)
#define FSMC_BWTR2_DATLAT_MASK  0x0f000000 /* Data latency */
#  define FSMC_BWTR2_DATLAT0    (1 << 24)
#  define FSMC_BWTR2_DATLAT1    (1 << 25)
#  define FSMC_BWTR2_DATLAT2    (1 << 26)
#  define FSMC_BWTR2_DATLAT3    (1 << 27)
#define FSMC_BWTR2_ACCMOD_MASK  0x30000000 /* Access mode */
#  define FSMC_BWTR2_ACCMOD0    (1 << 28)
#  define FSMC_BWTR2_ACCMOD1    (1 << 29)

#define FSMC_BWTR3_ADDSET_MASK  0x0000000f /* Address setup phase duration */
#  define FSMC_BWTR3_ADDSET0    (1 << 0)
#  define FSMC_BWTR3_ADDSET1    (1 << 1)
#  define FSMC_BWTR3_ADDSET2    (1 << 2)
#  define FSMC_BWTR3_ADDSET3    (1 << 3)
#define FSMC_BWTR3_ADDHLD_MASK  0x000000f0 /* Address-hold phase duration */
#  define FSMC_BWTR3_ADDHLD0    (1 << 4)
#  define FSMC_BWTR3_ADDHLD1    (1 << 5)
#  define FSMC_BWTR3_ADDHLD2    (1 << 6)
#  define FSMC_BWTR3_ADDHLD3    (1 << 7)
#define FSMC_BWTR3_DATAST_MASK  0x0000ff00 /* Data-phase duration */
#  define FSMC_BWTR3_DATAST0    (1 << 8)
#  define FSMC_BWTR3_DATAST1    (1 << 9)
#  define FSMC_BWTR3_DATAST2    (1 << 10)
#  define FSMC_BWTR3_DATAST3    (1 << 11)
#define FSMC_BWTR3_CLKDIV_MASK  0x00f00000 /* Clock divide ratio */
#  define FSMC_BWTR3_CLKDIV0    (1 << 20)
#  define FSMC_BWTR3_CLKDIV1    (1 << 21)
#  define FSMC_BWTR3_CLKDIV2    (1 << 22)
#  define FSMC_BWTR3_CLKDIV3    (1 << 23)
#define FSMC_BWTR3_DATLAT_MASK  0x0f000000 /* Data latency */
#  define FSMC_BWTR3_DATLAT0    (1 << 24)
#  define FSMC_BWTR3_DATLAT1    (1 << 25)
#  define FSMC_BWTR3_DATLAT2    (1 << 26)
#  define FSMC_BWTR3_DATLAT3    (1 << 27)
#define FSMC_BWTR3_ACCMOD_MASK  0x30000000 /* Access mode */
#  define FSMC_BWTR3_ACCMOD0    (1 << 28)
#  define FSMC_BWTR3_ACCMOD1    (1 << 29)

#define FSMC_BWTR4_ADDSET_MASK  0x0000000f /* Address setup phase duration */
#  define FSMC_BWTR4_ADDSET0    (1 << 0)
#  define FSMC_BWTR4_ADDSET1    (1 << 1)
#  define FSMC_BWTR4_ADDSET2    (1 << 2)
#  define FSMC_BWTR4_ADDSET3    (1 << 3)
#define FSMC_BWTR4_ADDHLD_MASK  0x000000f0 /* Address-hold phase duration */
#  define FSMC_BWTR4_ADDHLD0    (1 << 4)
#  define FSMC_BWTR4_ADDHLD1    (1 << 5)
#  define FSMC_BWTR4_ADDHLD2    (1 << 6)
#  define FSMC_BWTR4_ADDHLD3    (1 << 7)
#define FSMC_BWTR4_DATAST_MASK  0x0000ff00 /* Data-phase duration */
#  define FSMC_BWTR4_DATAST0    (1 << 8)
#  define FSMC_BWTR4_DATAST1    (1 << 9)
#  define FSMC_BWTR4_DATAST2    (1 << 10)
#  define FSMC_BWTR4_DATAST3    (1 << 11)
#define FSMC_BWTR4_CLKDIV_MASK  0x00f00000 /* Clock divide ratio */
#  define FSMC_BWTR4_CLKDIV0    (1 << 20)
#  define FSMC_BWTR4_CLKDIV1    (1 << 21)
#  define FSMC_BWTR4_CLKDIV2    (1 << 22)
#  define FSMC_BWTR4_CLKDIV3    (1 << 23)
#define FSMC_BWTR4_DATLAT_MASK  0x0f000000 /* Data latency */
#  define FSMC_BWTR4_DATLAT0    (1 << 24)
#  define FSMC_BWTR4_DATLAT1    (1 << 25)
#  define FSMC_BWTR4_DATLAT2    (1 << 26)
#  define FSMC_BWTR4_DATLAT3    (1 << 27)
#define FSMC_BWTR4_ACCMOD_MASK  0x30000000 /* Access mode */
#  define FSMC_BWTR4_ACCMOD0    (1 << 28)
#  define FSMC_BWTR4_ACCMOD1    (1 << 29)

#define FSMC_PCR2_PWAITEN       (1 << 1)   /* Wait feature enable bit */
#  define FSMC_PCR2_PBKEN       (1 << 2)   /* PC Card/NAND Flash memory bank enable bit */
#  define FSMC_PCR2_PTYP        (1 << 3)   /* Memory type */
#define FSMC_PCR2_PWID_MASK     0x00000030 /* NAND Flash databus width */
#  define FSMC_PCR2_PWID0       (1 << 4)
#  define FSMC_PCR2_PWID1       (1 << 5)
#define FSMC_PCR2_ECCEN         (1 << 6)   /* ECC computation logic enable bit */
#define FSMC_PCR2_TCLR_MASK     0x00001e00 /* CLE to RE delay */
#  define FSMC_PCR2_TCLR0       (1 << 9)
#  define FSMC_PCR2_TCLR1       (1 << 10)
#  define FSMC_PCR2_TCLR2       (1 << 11)
#  define FSMC_PCR2_TCLR3       (1 << 12)
#define FSMC_PCR2_TAR_MASK      0x0001e000 /* ALE to RE delay */
#  define FSMC_PCR2_TAR0        (1 << 13)
#  define FSMC_PCR2_TAR1        (1 << 14)
#  define FSMC_PCR2_TAR2        (1 << 15)
#  define FSMC_PCR2_TAR3        (1 << 16)
#define FSMC_PCR2_ECCPS_MASK    0x000e0000 /* ECC page size */
#  define FSMC_PCR2_ECCPS0      (1 << 17)
#  define FSMC_PCR2_ECCPS1      (1 << 18)
#  define FSMC_PCR2_ECCPS2      (1 << 19)

#define FSMC_PCR3_PWAITEN       (1 << 1)   /* Wait feature enable bit */
#  define FSMC_PCR3_PBKEN       (1 << 2)   /* PC Card/NAND Flash memory bank enable bit */
#  define FSMC_PCR3_PTYP        (1 << 3)   /* Memory type */
#define FSMC_PCR3_PWID_MASK     0x00000030 /* NAND Flash databus width */
#  define FSMC_PCR3_PWID0       (1 << 4)
#  define FSMC_PCR3_PWID1       (1 << 5)
#define FSMC_PCR3_ECCEN         (1 << 6)   /* ECC computation logic enable bit */
#define FSMC_PCR3_TCLR_MASK     0x00001e00 /* CLE to RE delay */
#  define FSMC_PCR3_TCLR0       (1 << 9)
#  define FSMC_PCR3_TCLR1       (1 << 10)
#  define FSMC_PCR3_TCLR2       (1 << 11)
#  define FSMC_PCR3_TCLR3       (1 << 12)
#define FSMC_PCR3_TAR_MASK      0x0001e000 /* ALE to RE delay */
#  define FSMC_PCR3_TAR0        (1 << 13)
#  define FSMC_PCR3_TAR1        (1 << 14)
#  define FSMC_PCR3_TAR2        (1 << 15)
#  define FSMC_PCR3_TAR3        (1 << 16)
#define FSMC_PCR3_ECCPS_MASK    0x000e0000 /* ECC page size */
#  define FSMC_PCR3_ECCPS0      (1 << 17)
#  define FSMC_PCR3_ECCPS1      (1 << 18)
#  define FSMC_PCR3_ECCPS2      (1 << 19)

#define FSMC_PCR4_PWAITEN       (1 << 1)   /* Wait feature enable bit */
#  define FSMC_PCR4_PBKEN       (1 << 2)   /* PC Card/NAND Flash memory bank enable bit */
#  define FSMC_PCR4_PTYP        (1 << 3)   /* Memory type */
#define FSMC_PCR4_PWID_MASK     0x00000030 /* NAND Flash databus width */
#  define FSMC_PCR4_PWID0       (1 << 4)
#  define FSMC_PCR4_PWID1       (1 << 5)
#define FSMC_PCR4_ECCEN         (1 << 6)   /* ECC computation logic enable bit */
#define FSMC_PCR4_TCLR_MASK     0x00001e00 /* CLE to RE delay */
#  define FSMC_PCR4_TCLR0       (1 << 9)
#  define FSMC_PCR4_TCLR1       (1 << 10)
#  define FSMC_PCR4_TCLR2       (1 << 11)
#  define FSMC_PCR4_TCLR3       (1 << 12)
#define FSMC_PCR4_TAR_MASK      0x0001e000 /* ALE to RE delay */
#  define FSMC_PCR4_TAR0        (1 << 13)
#  define FSMC_PCR4_TAR1        (1 << 14)
#  define FSMC_PCR4_TAR2        (1 << 15)
#  define FSMC_PCR4_TAR3        (1 << 16)
#define FSMC_PCR4_ECCPS_MASK    0x000e0000 /* ECC page size */
#  define FSMC_PCR4_ECCPS0      (1 << 17)
#  define FSMC_PCR4_ECCPS1      (1 << 18)
#  define FSMC_PCR4_ECCPS2      (1 << 19)

#define FSMC_SR2_IRS            (1 << 0)  /* Interrupt Rising Edge status */
#define FSMC_SR2_ILS            (1 << 1)  /* Interrupt Level status */
#define FSMC_SR2_IFS            (1 << 2)  /* Interrupt Falling Edge status */
#define FSMC_SR2_IREN           (1 << 3)  /* Interrupt Rising Edge detection Enable bit */
#define FSMC_SR2_ILEN           (1 << 4)  /* Interrupt Level detection Enable bit */
#define FSMC_SR2_IFEN           (1 << 5)  /* Interrupt Falling Edge detection Enable bit */
#define FSMC_SR2_FEMPT          (1 << 6)  /* FIFO empty */

#define FSMC_SR3_IRS            (1 << 0)  /* Interrupt Rising Edge status */
#define FSMC_SR3_ILS            (1 << 1)  /* Interrupt Level status */
#define FSMC_SR3_IFS            (1 << 2)  /* Interrupt Falling Edge status */
#define FSMC_SR3_IREN           (1 << 3)  /* Interrupt Rising Edge detection Enable bit */
#define FSMC_SR3_ILEN           (1 << 4)  /* Interrupt Level detection Enable bit */
#define FSMC_SR3_IFEN           (1 << 5)  /* Interrupt Falling Edge detection Enable bit */
#define FSMC_SR3_FEMPT          (1 << 6)  /* FIFO empty */

#define FSMC_SR4_IRS            (1 << 0)  /* Interrupt Rising Edge status */
#define FSMC_SR4_ILS            (1 << 1)  /* Interrupt Level status */
#define FSMC_SR4_IFS            (1 << 2)  /* Interrupt Falling Edge status */
#define FSMC_SR4_IREN           (1 << 3)  /* Interrupt Rising Edge detection Enable bit */
#define FSMC_SR4_ILEN           (1 << 4)  /* Interrupt Level detection Enable bit */
#define FSMC_SR4_IFEN           (1 << 5)  /* Interrupt Falling Edge detection Enable bit */
#define FSMC_SR4_FEMPT          (1 << 6)  /* FIFO empty */

#define FSMC_PMEM2_MEMSET2_MASK 0x000000ff /* Common memory 2 setup time */
#  define FSMC_PMEM2_MEMSET20   (1 << 0)
#  define FSMC_PMEM2_MEMSET21   (1 << 1)
#  define FSMC_PMEM2_MEMSET22   (1 << 2)
#  define FSMC_PMEM2_MEMSET23   (1 << 3)
#  define FSMC_PMEM2_MEMSET24   (1 << 4)
#  define FSMC_PMEM2_MEMSET25   (1 << 5)
#  define FSMC_PMEM2_MEMSET26   (1 << 6)
#  define FSMC_PMEM2_MEMSET27   (1 << 7)
#define FSMC_PMEM2_MEMWAIT2     0x0000ff00 /* Common memory 2 wait time */
#  define FSMC_PMEM2_MEMWAIT20  (1 << 8)
#  define FSMC_PMEM2_MEMWAIT21  (1 << 9)
#  define FSMC_PMEM2_MEMWAIT22  (1 << 10)
#  define FSMC_PMEM2_MEMWAIT23  (1 << 11)
#  define FSMC_PMEM2_MEMWAIT24  (1 << 12)
#  define FSMC_PMEM2_MEMWAIT25  (1 << 13)
#  define FSMC_PMEM2_MEMWAIT26  (1 << 14)
#  define FSMC_PMEM2_MEMWAIT27  (1 << 15)
#define FSMC_PMEM2_MEMHOLD2     0x00ff0000 /* Common memory 2 hold time */
#  define FSMC_PMEM2_MEMHOLD20  (1 << 16)
#  define FSMC_PMEM2_MEMHOLD21  (1 << 17)
#  define FSMC_PMEM2_MEMHOLD22  (1 << 18)
#  define FSMC_PMEM2_MEMHOLD23  (1 << 19)
#  define FSMC_PMEM2_MEMHOLD24  (1 << 20)
#  define FSMC_PMEM2_MEMHOLD25  (1 << 21)
#  define FSMC_PMEM2_MEMHOLD26  (1 << 22)
#  define FSMC_PMEM2_MEMHOLD27  (1 << 23)
#define FSMC_PMEM2_MEMHIZ2_MASK  0xff000000 /* Common memory 2 databus HiZ time */
#  define FSMC_PMEM2_MEMHIZ20   (1 << 24)
#  define FSMC_PMEM2_MEMHIZ21   (1 << 25)
#  define FSMC_PMEM2_MEMHIZ22   (1 << 26)
#  define FSMC_PMEM2_MEMHIZ23   (1 << 27)
#  define FSMC_PMEM2_MEMHIZ24   (1 << 28)
#  define FSMC_PMEM2_MEMHIZ25   (1 << 29)
#  define FSMC_PMEM2_MEMHIZ26   (1 << 30)
#  define FSMC_PMEM2_MEMHIZ27   (1 << 31)

#define FSMC_PMEM3_MEMSET3_MASK 0x000000ff /* Common memory 3 setup time */
#  define FSMC_PMEM3_MEMSET30   (1 << 0)
#  define FSMC_PMEM3_MEMSET31   (1 << 1)
#  define FSMC_PMEM3_MEMSET32   (1 << 2)
#  define FSMC_PMEM3_MEMSET33   (1 << 3)
#  define FSMC_PMEM3_MEMSET34   (1 << 4)
#  define FSMC_PMEM3_MEMSET35   (1 << 5)
#  define FSMC_PMEM3_MEMSET36   (1 << 6)
#  define FSMC_PMEM3_MEMSET37   (1 << 7)
#define FSMC_PMEM3_MEMWAIT3     0x0000ff00 /* Common memory 3 wait time */
#  define FSMC_PMEM3_MEMWAIT30  (1 << 8)
#  define FSMC_PMEM3_MEMWAIT31  (1 << 9)
#  define FSMC_PMEM3_MEMWAIT32  (1 << 10)
#  define FSMC_PMEM3_MEMWAIT33  (1 << 11)
#  define FSMC_PMEM3_MEMWAIT34  (1 << 12)
#  define FSMC_PMEM3_MEMWAIT35  (1 << 13)
#  define FSMC_PMEM3_MEMWAIT36  (1 << 14)
#  define FSMC_PMEM3_MEMWAIT37  (1 << 15)
#define FSMC_PMEM3_MEMHOLD3     0x00ff0000 /* Common memory 3 hold time */
#  define FSMC_PMEM3_MEMHOLD30  (1 << 16)
#  define FSMC_PMEM3_MEMHOLD31  (1 << 17)
#  define FSMC_PMEM3_MEMHOLD32  (1 << 18)
#  define FSMC_PMEM3_MEMHOLD33  (1 << 19)
#  define FSMC_PMEM3_MEMHOLD34  (1 << 20)
#  define FSMC_PMEM3_MEMHOLD35  (1 << 21)
#  define FSMC_PMEM3_MEMHOLD36  (1 << 22)
#  define FSMC_PMEM3_MEMHOLD37  (1 << 23)
#define FSMC_PMEM3_MEMHIZ3_MASK 0xff000000 /* Common memory 3 databus HiZ time */
#  define FSMC_PMEM3_MEMHIZ30   (1 << 24)
#  define FSMC_PMEM3_MEMHIZ31   (1 << 25)
#  define FSMC_PMEM3_MEMHIZ32   (1 << 26)
#  define FSMC_PMEM3_MEMHIZ33   (1 << 27)
#  define FSMC_PMEM3_MEMHIZ34   (1 << 28)
#  define FSMC_PMEM3_MEMHIZ35   (1 << 29)
#  define FSMC_PMEM3_MEMHIZ36   (1 << 30)
#  define FSMC_PMEM3_MEMHIZ37   (1 << 31)

#define FSMC_PMEM4_MEMSET4_MASK 0x000000ff /* Common memory 4 setup time */
#  define FSMC_PMEM4_MEMSET40   (1 << 0)
#  define FSMC_PMEM4_MEMSET41   (1 << 1)
#  define FSMC_PMEM4_MEMSET42   (1 << 2)
#  define FSMC_PMEM4_MEMSET43   (1 << 3)
#  define FSMC_PMEM4_MEMSET44   (1 << 4)
#  define FSMC_PMEM4_MEMSET45   (1 << 5)
#  define FSMC_PMEM4_MEMSET46   (1 << 6)
#  define FSMC_PMEM4_MEMSET47   (1 << 7)
#define FSMC_PMEM4_MEMWAIT4     0x0000ff00 /* Common memory 4 wait time */
#  define FSMC_PMEM4_MEMWAIT40  (1 << 8)
#  define FSMC_PMEM4_MEMWAIT41  (1 << 9)
#  define FSMC_PMEM4_MEMWAIT42  (1 << 10)
#  define FSMC_PMEM4_MEMWAIT43  (1 << 11)
#  define FSMC_PMEM4_MEMWAIT44  (1 << 12)
#  define FSMC_PMEM4_MEMWAIT45  (1 << 13)
#  define FSMC_PMEM4_MEMWAIT46  (1 << 14)
#  define FSMC_PMEM4_MEMWAIT47  (1 << 15)
#define FSMC_PMEM4_MEMHOLD4     0x00ff0000 /* Common memory 4 hold time */
#  define FSMC_PMEM4_MEMHOLD40  (1 << 16)
#  define FSMC_PMEM4_MEMHOLD41  (1 << 17)
#  define FSMC_PMEM4_MEMHOLD42  (1 << 18)
#  define FSMC_PMEM4_MEMHOLD43  (1 << 19)
#  define FSMC_PMEM4_MEMHOLD44  (1 << 20)
#  define FSMC_PMEM4_MEMHOLD45  (1 << 21)
#  define FSMC_PMEM4_MEMHOLD46  (1 << 22)
#  define FSMC_PMEM4_MEMHOLD47  (1 << 23)
#define FSMC_PMEM4_MEMHIZ4_MASK 0xff000000 /* Common memory 4 databus HiZ time */
#  define FSMC_PMEM4_MEMHIZ40   (1 << 24)
#  define FSMC_PMEM4_MEMHIZ41   (1 << 25)
#  define FSMC_PMEM4_MEMHIZ42   (1 << 26)
#  define FSMC_PMEM4_MEMHIZ43   (1 << 27)
#  define FSMC_PMEM4_MEMHIZ44   (1 << 28)
#  define FSMC_PMEM4_MEMHIZ45   (1 << 29)
#  define FSMC_PMEM4_MEMHIZ46   (1 << 30)
#  define FSMC_PMEM4_MEMHIZ47   (1 << 31)

#define FSMC_PATT2_ATTSET2_MASK 0x000000ff /* Attribute memory 2 setup time */
#  define FSMC_PATT2_ATTSET20   (1 << 0)
#  define FSMC_PATT2_ATTSET21   (1 << 1)
#  define FSMC_PATT2_ATTSET22   (1 << 2)
#  define FSMC_PATT2_ATTSET23   (1 << 3)
#  define FSMC_PATT2_ATTSET24   (1 << 4)
#  define FSMC_PATT2_ATTSET25   (1 << 5)
#  define FSMC_PATT2_ATTSET26   (1 << 6)
#  define FSMC_PATT2_ATTSET27   (1 << 7)
#define FSMC_PATT2_ATTWAIT2     0x0000ff00 /* Attribute memory 2 wait time */
#  define FSMC_PATT2_ATTWAIT20  (1 << 8)
#  define FSMC_PATT2_ATTWAIT21  (1 << 9)
#  define FSMC_PATT2_ATTWAIT22  (1 << 10)
#  define FSMC_PATT2_ATTWAIT23  (1 << 11)
#  define FSMC_PATT2_ATTWAIT24  (1 << 12)
#  define FSMC_PATT2_ATTWAIT25  (1 << 13)
#  define FSMC_PATT2_ATTWAIT26  (1 << 14)
#  define FSMC_PATT2_ATTWAIT27  (1 << 15)
#define FSMC_PATT2_ATTHOLD2     0x00ff0000 /* Attribute memory 2 hold time */
#  define FSMC_PATT2_ATTHOLD20  (1 << 16)
#  define FSMC_PATT2_ATTHOLD21  (1 << 17)
#  define FSMC_PATT2_ATTHOLD22  (1 << 18)
#  define FSMC_PATT2_ATTHOLD23  (1 << 19)
#  define FSMC_PATT2_ATTHOLD24  (1 << 20)
#  define FSMC_PATT2_ATTHOLD25  (1 << 21)
#  define FSMC_PATT2_ATTHOLD26  (1 << 22)
#  define FSMC_PATT2_ATTHOLD27  (1 << 23)
#define FSMC_PATT2_ATTHIZ2_MASK 0xff000000 /* Attribute memory 2 databus HiZ time */
#  define FSMC_PATT2_ATTHIZ20   (1 << 24)
#  define FSMC_PATT2_ATTHIZ21   (1 << 25)
#  define FSMC_PATT2_ATTHIZ22   (1 << 26)
#  define FSMC_PATT2_ATTHIZ23   (1 << 27)
#  define FSMC_PATT2_ATTHIZ24   (1 << 28)
#  define FSMC_PATT2_ATTHIZ25   (1 << 29)
#  define FSMC_PATT2_ATTHIZ26   (1 << 30)
#  define FSMC_PATT2_ATTHIZ27   (1 << 31)

#define FSMC_PATT3_ATTSET3_MASK 0x000000ff /* Attribute memory 3 setup time */
#  define FSMC_PATT3_ATTSET30   (1 << 0)
#  define FSMC_PATT3_ATTSET31   (1 << 1)
#  define FSMC_PATT3_ATTSET32   (1 << 2)
#  define FSMC_PATT3_ATTSET33   (1 << 3)
#  define FSMC_PATT3_ATTSET34   (1 << 4)
#  define FSMC_PATT3_ATTSET35   (1 << 5)
#  define FSMC_PATT3_ATTSET36   (1 << 6)
#  define FSMC_PATT3_ATTSET37   (1 << 7)
#define FSMC_PATT3_ATTWAIT3     0x0000ff00 /* Attribute memory 3 wait time */
#  define FSMC_PATT3_ATTWAIT30  (1 << 8)
#  define FSMC_PATT3_ATTWAIT31  (1 << 9)
#  define FSMC_PATT3_ATTWAIT32  (1 << 10)
#  define FSMC_PATT3_ATTWAIT33  (1 << 11)
#  define FSMC_PATT3_ATTWAIT34  (1 << 12)
#  define FSMC_PATT3_ATTWAIT35  (1 << 13)
#  define FSMC_PATT3_ATTWAIT36  (1 << 14)
#  define FSMC_PATT3_ATTWAIT37  (1 << 15)
#define FSMC_PATT3_ATTHOLD3     0x00ff0000 /* Attribute memory 3 hold time */
#  define FSMC_PATT3_ATTHOLD30  (1 << 16)
#  define FSMC_PATT3_ATTHOLD31  (1 << 17)
#  define FSMC_PATT3_ATTHOLD32  (1 << 18)
#  define FSMC_PATT3_ATTHOLD33  (1 << 19)
#  define FSMC_PATT3_ATTHOLD34  (1 << 20)
#  define FSMC_PATT3_ATTHOLD35  (1 << 21)
#  define FSMC_PATT3_ATTHOLD36  (1 << 22)
#  define FSMC_PATT3_ATTHOLD37  (1 << 23)
#define FSMC_PATT3_ATTHIZ3_MASK 0xff000000 /* Attribute memory 3 databus HiZ time */
#  define FSMC_PATT3_ATTHIZ30   (1 << 24)
#  define FSMC_PATT3_ATTHIZ31   (1 << 25)
#  define FSMC_PATT3_ATTHIZ32   (1 << 26)
#  define FSMC_PATT3_ATTHIZ33   (1 << 27)
#  define FSMC_PATT3_ATTHIZ34   (1 << 28)
#  define FSMC_PATT3_ATTHIZ35   (1 << 29)
#  define FSMC_PATT3_ATTHIZ36   (1 << 30)
#  define FSMC_PATT3_ATTHIZ37   (1 << 31)

#define FSMC_PATT4_ATTSET4_MASK 0x000000ff /* Attribute memory 4 setup time */
#  define FSMC_PATT4_ATTSET40   (1 << 0)
#  define FSMC_PATT4_ATTSET41   (1 << 1)
#  define FSMC_PATT4_ATTSET42   (1 << 2)
#  define FSMC_PATT4_ATTSET43   (1 << 3)
#  define FSMC_PATT4_ATTSET44   (1 << 4)
#  define FSMC_PATT4_ATTSET45   (1 << 5)
#  define FSMC_PATT4_ATTSET46   (1 << 6)
#  define FSMC_PATT4_ATTSET47   (1 << 7)
#define FSMC_PATT4_ATTWAIT4     0x0000ff00 /* Attribute memory 4 wait time */
#  define FSMC_PATT4_ATTWAIT40  (1 << 8)
#  define FSMC_PATT4_ATTWAIT41  (1 << 9)
#  define FSMC_PATT4_ATTWAIT42  (1 << 10)
#  define FSMC_PATT4_ATTWAIT43  (1 << 11)
#  define FSMC_PATT4_ATTWAIT44  (1 << 12)
#  define FSMC_PATT4_ATTWAIT45  (1 << 13)
#  define FSMC_PATT4_ATTWAIT46  (1 << 14)
#  define FSMC_PATT4_ATTWAIT47  (1 << 15)
#define FSMC_PATT4_ATTHOLD4     0x00ff0000 /* Attribute memory 4 hold time */
#  define FSMC_PATT4_ATTHOLD40  (1 << 16)
#  define FSMC_PATT4_ATTHOLD41  (1 << 17)
#  define FSMC_PATT4_ATTHOLD42  (1 << 18)
#  define FSMC_PATT4_ATTHOLD43  (1 << 19)
#  define FSMC_PATT4_ATTHOLD44  (1 << 20)
#  define FSMC_PATT4_ATTHOLD45  (1 << 21)
#  define FSMC_PATT4_ATTHOLD46  (1 << 22)
#  define FSMC_PATT4_ATTHOLD47  (1 << 23)
#define FSMC_PATT4_ATTHIZ4_MASK 0xff000000 /* Attribute memory 4 databus HiZ time */
#  define FSMC_PATT4_ATTHIZ40   (1 << 24)
#  define FSMC_PATT4_ATTHIZ41   (1 << 25)
#  define FSMC_PATT4_ATTHIZ42   (1 << 26)
#  define FSMC_PATT4_ATTHIZ43   (1 << 27)
#  define FSMC_PATT4_ATTHIZ44   (1 << 28)
#  define FSMC_PATT4_ATTHIZ45   (1 << 29)
#  define FSMC_PATT4_ATTHIZ46   (1 << 30)
#  define FSMC_PATT4_ATTHIZ47   (1 << 31)

#define FSMC_PIO4_IOSET4_MASK   0x000000ff /* I/O 4 setup time */
#  define FSMC_PIO4_IOSET40     (1 << 0)
#  define FSMC_PIO4_IOSET41     (1 << 1)
#  define FSMC_PIO4_IOSET42     (1 << 2)
#  define FSMC_PIO4_IOSET43     (1 << 3)
#  define FSMC_PIO4_IOSET44     (1 << 4)
#  define FSMC_PIO4_IOSET45     (1 << 5)
#  define FSMC_PIO4_IOSET46     (1 << 6)
#  define FSMC_PIO4_IOSET47     (1 << 7)
#define FSMC_PIO4_IOWAIT4_MASK  0x0000ff00 /* I/O 4 wait time */
#  define FSMC_PIO4_IOWAIT40    (1 << 8)
#  define FSMC_PIO4_IOWAIT41    (1 << 9)
#  define FSMC_PIO4_IOWAIT42    (1 << 10)
#  define FSMC_PIO4_IOWAIT43    (1 << 11)
#  define FSMC_PIO4_IOWAIT44    (1 << 12)
#  define FSMC_PIO4_IOWAIT45    (1 << 13)
#  define FSMC_PIO4_IOWAIT46    (1 << 14)
#  define FSMC_PIO4_IOWAIT47    (1 << 15)
#define FSMC_PIO4_IOHOLD4_MASK  0x00ff0000 /* I/O 4 hold time */
#  define FSMC_PIO4_IOHOLD40    (1 << 16)
#  define FSMC_PIO4_IOHOLD41    (1 << 17)
#  define FSMC_PIO4_IOHOLD42    (1 << 18)
#  define FSMC_PIO4_IOHOLD43    (1 << 19)
#  define FSMC_PIO4_IOHOLD44    (1 << 20)
#  define FSMC_PIO4_IOHOLD45    (1 << 21)
#  define FSMC_PIO4_IOHOLD46    (1 << 22)
#  define FSMC_PIO4_IOHOLD47    (1 << 23)
#define FSMC_PIO4_IOHIZ4_MASK   0xff000000 /* I/O 4 databus HiZ time */
#  define FSMC_PIO4_IOHIZ40     (1 << 24)
#  define FSMC_PIO4_IOHIZ41     (1 << 25)
#  define FSMC_PIO4_IOHIZ42     (1 << 26)
#  define FSMC_PIO4_IOHIZ43     (1 << 27)
#  define FSMC_PIO4_IOHIZ44     (1 << 28)
#  define FSMC_PIO4_IOHIZ45     (1 << 29)
#  define FSMC_PIO4_IOHIZ46     (1 << 30)
#  define FSMC_PIO4_IOHIZ47     (1 << 31)

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
