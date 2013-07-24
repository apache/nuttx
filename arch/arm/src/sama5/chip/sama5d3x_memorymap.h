/************************************************************************************
 * arch/arm/src/sama5/sama5d3x_memorymap.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAMA5D3X_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAMA5_SAMA5D3X_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/sama5/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Overview:
 *
 *  SAMA5 Physical (unmapped) Memory Map
 *  - SAMA5 Internal Memories
 *  - SAMA5 Internal Peripheral Offsets
 *  - System Controller Peripheral Offsets
 *  Sizes of memory regions in bytes
 *  Sizes of memory regions in sections
 *  Section MMU Flags
 *  SAMA5 Virtual (mapped) Memory Map
 *  - Peripheral virtual base addresses
 *  - NuttX vitual base address
 *  MMU Page Table Location
 *  Page table start addresses
 *  Base address of the interrupt vector table
 */

/* SAMA5 Physical (unmapped) Memory Map */

#define SAM_INTMEM_PSECTION      0x00000000 /* 0x00000000-0x0fffffff: Internal Memories */
#define SAM_EBICS0_PSECTION      0x10000000 /* 0x10000000-0x1fffffff: EBI Chip select 0 */
#define SAM_DDRCS_PSECTION       0x20000000 /* 0x20000000-0x3fffffff: EBI DDRCS */
#define SAM_EBICS1_PSECTION      0x40000000 /* 0x40000000-0x4fffffff: EBI Chip select 1 */
#define SAM_EBICS2_PSECTION      0x50000000 /* 0x50000000-0x5fffffff: EBI Chip select 2 */
#define SAM_EBICS3_PSECTION      0x60000000 /* 0x60000000-0x6fffffff: EBI Chip select 2 */
#define SAM_NFCCR_PSECTION       0x70000000 /* 0x70000000-0x7fffffff: NFC Command Registers */
                                            /* 0x80000000-0xefffffff: Undefined */
#define SAM_PERIPH_PSECTION      0xf0000000 /* 0xf0000000-0xffffffff: Internal Peripherals */

/* SAMA5 Internal Memories */

#define SAM_BOOTMEM_PSECTION     0x00000000 /* 0x00000000-0x000fffff: Boot memory */
#define SAM_ROM_PSECTION         0x00100000 /* 0x00100000-0x001fffff: ROM */
#define SAM_NFCSRAM_PSECTION     0x00200000 /* 0x00200000-0x002fffff: NFC SRAM */
#define SAM_ISRAM_PSECTION       0x00300000 /* 0x00300000-0x0030ffff: SRAM */
#  define SAM_ISRAM0_PADDR       0x00300000 /* 0x00300000-0x0030ffff: SRAM0 */
#  define SAM_ISRAM1_PADDR       0x00310000 /* 0x00310000-0x003fffff: SRAM1 */
#define SAM_SMD_PSECTION         0x00400000 /* 0x00400000-0x004fffff: SMD */
#define SAM_UDPHSRAM_PSECTION    0x00500000 /* 0x00500000-0x005fffff: UDPH SRAM */
#define SAM_UHPOHCI_PSECTION     0x00600000 /* 0x00600000-0x006fffff: UHP OHCI */
#define SAM_UHPEHCI_PSECTION     0x00700000 /* 0x00700000-0x007fffff: UHP EHCI */
#define SAM_AXIMX_PSECTION       0x00800000 /* 0x00800000-0x008fffff: AXI Matr */
#define SAM_DAP_PSECTION         0x00900000 /* 0x00900000-0x009fffff: DAP */
                                            /* 0x000a0000-0x0fffffff: Undefined */
/* SAMA5 Internal Peripheral Offsets */

#define SAM_PERIPHA_PSECTION     0xf0000000 /* 0xf0000000-0xffffffff: Internal Peripherals */
#  define SAM_HSMCI0_OFFSET      0x00000000 /* 0x00000000-0x00003fff: HSMCI0 */
#  define SAM_SPI0_OFFSET        0x00004000 /* 0x00004000-0x00007fff: SPI0 */
#  define SAM_SSC0_OFFSET        0x00008000 /* 0x00008000-0x0000bfff: SSC0 */
#  define SAM_CAN0_OFFSET        0x0000c000 /* 0x0000c000-0x0000ffff: CAN0 */
#  define SAM_TC012_OFFSET       0x00010000 /* 0x00010000-0x00013fff: TC0, TC1, TC2 */
#  define SAM_TWI0_OFFSET        0x00014000 /* 0x00014000-0x00017fff: TWI0 */
#  define SAM_TWI1_OFFSET        0x00018000 /* 0x00018000-0x0001bfff: TWI1 */
#  define SAM_USART0_OFFSET      0x0001c000 /* 0x0001c000-0x0001ffff: USART0 */
#  define SAM_USART1_OFFSET      0x00020000 /* 0x00020000-0x00023fff: USART1 */
#  define SAM_UART0_OFFSET       0x00024000 /* 0x00024000-0x00027fff: UART0 */
#  define SAM_GMAC_OFFSET        0x00028000 /* 0x00028000-0x0002bfff: GMAC */
#  define SAM_PWMC_OFFSET        0x0002c000 /* 0x0002c000-0x0002ffff: PWMC */
#  define SAM_LCDC_OFFSET        0x00030000 /* 0x00030000-0x00033fff: LCDC */
#  define SAM_ISI_OFFSET         0x00034000 /* 0x00034000-0x00037fff: ISI */
#  define SAM_SFR_OFFSET         0x00038000 /* 0x00038000-0x0003bfff: SFR */
                                            /* 0x0003c000-0x07ffffff: Reserved */

#define SAM_PERIPHB_PSECTION     0xf8000000 /* 0xf8000000-0xffffbfff: Internal Peripherals B */
#  define SAM_HSMCI1_OFFSET      0x00000000 /* 0x00000000-0x00000fff: HSMCI1 */
#  define SAM_HSMCI2_OFFSET      0x00004000 /* 0x00004000-0x00007fff: HSMCI2 */
#  define SAM_SPI1_OFFSET        0x00008000 /* 0x00008000-0x0000bfff: SPI1 */
#  define SAM_SSC1_OFFSET        0x0000c000 /* 0x0000c000-0x0000ffff: SSC1 */
#  define SAM_CAN1_OFFSET        0x00010000 /* 0x00010000-0x00013fff: CAN1 */
#  define SAM_TC345_OFFSET       0x00014000 /* 0x00014000-0x00017fff: TC3, TC4, TC5 */
#  define SAM_TSADC_OFFSET       0x00018000 /* 0x00018000-0x0001bfff: TSADC */
#  define SAM_TWI2_OFFSET        0x0001c000 /* 0x0001c000-0x0001ffff: TWI2 */
#  define SAM_USART2_OFFSET      0x00020000 /* 0x00020000-0x00023fff: USART2 */
#  define SAM_USART3_OFFSET      0x00024000 /* 0x00024000-0x00027fff: USART3 */
#  define SAM_UART1_OFFSET       0x00028000 /* 0x00028000-0x0002bfff: UART1 */
#  define SAM_EMAC_OFFSET        0x0002c000 /* 0x0002c000-0x0002ffff: EMAC */
#  define SAM_UDPHS_OFFSET       0x00030000 /* 0x00030000-0x00033fff: UDPHS */
#  define SAM_SHA_OFFSET         0x00034000 /* 0x00034000-0x00037fff: SHA */
#  define SAM_AES_OFFSET         0x00038000 /* 0x00038000-0x0003bfff: AES */
#  define SAM_TDES_OFFSET        0x0003c000 /* 0x0003c000-0x0003ffff: TDES */
#  define SAM_TRNG_OFFSET        0x00040000 /* 0x00040000-0x00043fff: TRNG */
                                            /* 0x00044000-0x00ffbfff: Reserved */
#define SAM_SYSC_PSECTION        0xff000000 /* 0xff000000-0xffffffff: System Controller */
#define SAM_SYSC_PADDR           0xffffc000 /* 0xffffc000-0xffffffff: System Controller */
#  define SAM_SYSC_OFFSET        0x00000000 /* 0x0fffc000-0x0fffffff: System Controller */

/* System Controller Peripheral Offsets */

#define SAM_HSMC_OFFSET          0x00ffc000 /* 0x0fffc000-0x0fffcfff: HSMC */
                                            /* 0x0fffd000-0x0fffe3ff: Reserved */
#define SAM_FUSE_OFFSET          0x00ffe400 /* 0x0fffe400-0x0fffe5ff: FUSE */
#define SAM_DMAC0_OFFSET         0x00ffe600 /* 0x0fffe600-0x0fffe7ff: DMAC0 */
#define SAM_DMAC1_OFFSET         0x00ffe800 /* 0x0fffe800-0x0fffe9ff: DMAC1 */
#define SAM_MPDDRC_OFFSET        0x00ffea00 /* 0x0fffea00-0x0fffebff: MPDDRC */
#define SAM_MATRIX_OFFSET        0x00ffec00 /* 0x0fffec00-0x0fffedff: MATRIX */
#define SAM_DBGU_OFFSET          0x00ffee00 /* 0x0fffee00-0x0fffefff: DBGU */
#define SAM_AIC_OFFSET           0x00fff000 /* 0x0ffff000-0x0ffff1ff: AIC */
#define SAM_PION_OFFSET(n)       (0x00fff200+((n) << 9))
#define SAM_PIOA_OFFSET          0x00fff200 /* 0x0ffff200-0x0ffff3ff: PIOA */
#define SAM_PIOB_OFFSET          0x00fff400 /* 0x0ffff400-0x0ffff5ff: PIOB */
#define SAM_PIOC_OFFSET          0x00fff600 /* 0x0ffff600-0x0ffff7ff: PIOC */
#define SAM_PIOD_OFFSET          0x00fff800 /* 0x0ffff800-0x0ffff9ff: PIOD */
#define SAM_PIOE_OFFSET          0x00fffa00 /* 0x0ffffa00-0x0ffffbff: PIOE */
#define SAM_PMC_OFFSET           0x00fffc00 /* 0x0ffffc00-0x0ffffdff: PMC */
#define SAM_RSTC_OFFSET          0x00fffe00 /* 0x0ffffe00-0x0ffffe0f: RSTC */
#define SAM_SHDC_OFFSET          0x00fffe10 /* 0x0ffffe10-0x0ffffe1f: SHDC */
                                            /* 0x0ffffe20-0x0ffffe2f: Reserved */
#define SAM_PITC_OFFSET          0x00fffe30 /* 0x0ffffe30-0x0ffffe3f: PITC */
#define SAM_WDT_OFFSET           0x00fffe40 /* 0x0ffffe40-0x0ffffe4f: WDT */
#define SAM_SCKCR_OFFSET         0x00fffe50 /* 0x0ffffe50-0x0ffffe53: SCKCR */
#define SAM_BSC_OFFSET           0x00fffe54 /* 0x0ffffe54-0x0ffffe5f: BSC */
#define SAM_GPBR_OFFSET          0x00fffe60 /* 0x0ffffe60-0x0ffffe6f: GPBR */
                                            /* 0x0ffffe70-0x0ffffeaf: Reserved */
#define SAM_RTCC_OFFSET          0x00fffeb0 /* 0x0ffffeb0-0x0ffffedf: RTCC */
                                            /* 0x0ffffee0-0x0fffffff: Reserved */

/* Sizes of memory regions in bytes.
 *
 * These sizes exclude the undefined addresses at the end of the memory
 * region.  The implemented sizes of the EBI CS0-3 and DDRCS regions
 * are not known apriori and must be specified with configuration settings.
 */
                                                 /* 0x00000000-0x0fffffff: Internal Memories */
#define SAM_BOOTMEM_SIZE         (1*1024*1024)   /* 0x00000000-0x000fffff: Boot memory */
#define SAM_ROM_SIZE             (1*1024*1024)   /* 0x00100000-0x001fffff: ROM */
#define SAM_NFCSRAM_SIZE         (1*1024*1024)   /* 0x00200000-0x002fffff: NFC SRAM */
                                                 /* 0x00300000-0x003fffff: SRAM0 and SRAM1 */
#define SAM_ISRAM_SIZE           (64*1024 + SAM_ISRAM1_SIZE)
#define SAM_SMD_SIZE             (1*1024*1024)   /* 0x00400000-0x004fffff: SMD */
#define SAM_UDPHSRAM_SIZE        (1*1024*1024)   /* 0x00500000-0x005fffff: UDPH SRAM */
#define SAM_UHPOHCI_SIZE         (1*1024*1024)   /* 0x00600000-0x006fffff: UHP OHCI */
#define SAM_UHPEHCI_SIZE         (1*1024*1024)   /* 0x00700000-0x007fffff: UHP EHCI */
#define SAM_AXIMX_SIZE           (4)             /* 0x00800000-0x008fffff: AXI Matr */
#define SAM_DAP_SIZE             (1*1024*1024)   /* 0x00900000-0x009fffff: DAP */
#define SAM_NFCCR_SIZE           (256*1024*1024) /* 0x70000000-0x7fffffff: NFC Command Registers */
                                                 /* 0xf0000000-0xffffffff: Internal Peripherals */
#define SAM_PERIPHA_SIZE         (15*1024)       /* 0xf0000000-0xf003bfff: Internal Peripherals */
#define SAM_PERIPHB_SIZE         (272*1024)      /* 0xf8000000-0xf8043fff: Internal Peripherals */
#define SAM_SYSC_SIZE            (1*1024*1024)   /* 0xff000000-0x0ffffedf: Internal Peripherals */

/* Convert size in bytes to number of sections (in Mb). */

#define _NSECTIONS(b)            (((b)+0x000fffff) >> 20)

/* Sizes of memory regions in sections.
 *
 * The boot logic in sam_boot.c, will select 1Mb level 1 MMU mappings to
 * span the entire physical address space.  The definitions below specify
 * the number of 1Mb entries that are required to span a particular address
 * region.
 */

#define SAM_BOOTMEM_NSECTIONS    _NSECTIONS(SAM_BOOTMEM_SIZE)
#define SAM_ROM_NSECTIONS        _NSECTIONS(SAM_ROM_SIZE)
#define SAM_NFCSRAM_NSECTIONS    _NSECTIONS(SAM_NFCSRAM_SIZE)
#define SAM_ISRAM_NSECTIONS      _NSECTIONS(SAM_ISRAM_SIZE)
#define SAM_SMD_NSECTIONS        _NSECTIONS(SAM_SMD_SIZE)
#define SAM_UDPHSRAM_NSECTIONS   _NSECTIONS(SAM_UDPHSRAM_SIZE)
#define SAM_UHPOHCI_NSECTIONS    _NSECTIONS(SAM_UHPOHCI_SIZE)
#define SAM_UHPEHCI_NSECTIONS    _NSECTIONS(SAM_UHPEHCI_SIZE)
#define SAM_AXIMX_NSECTIONS      _NSECTIONS(SAM_AXIMX_SIZE)
#define SAM_DAP_NSECTIONS        _NSECTIONS(SAM_DAP_SIZE)

#define SAM_EBICS0_NSECTIONS     _NSECTIONS(CONFIG_SAMA5_EBICS0_SIZE)
#define SAM_DDRCS_NSECTIONS      _NSECTIONS(CONFIG_SAMA5_DDRCS_SIZE)
#define SAM_EBICS1_NSECTIONS     _NSECTIONS(CONFIG_SAMA5_EBICS1_SIZE)
#define SAM_EBICS2_NSECTIONS     _NSECTIONS(CONFIG_SAMA5_EBICS2_SIZE)
#define SAM_EBICS3_NSECTIONS     _NSECTIONS(CONFIG_SAMA5_EBICS3_SIZE)
#define SAM_NFCCR_NSECTIONS      _NSECTIONS(SAM_NFCCR_SIZE)

#define SAM_PERIPHA_NSECTIONS    _NSECTIONS(SAM_PERIPHA_SIZE)
#define SAM_PERIPHB_NSECTIONS    _NSECTIONS(SAM_PERIPHB_SIZE)
#define SAM_SYSC_NSECTIONS       _NSECTIONS(SAM_SYSC_SIZE)

/* Section MMU Flags */

#define SAM_BOOTMEM_MMUFLAGS     MMU_ROMFLAGS
#define SAM_ROM_MMUFLAGS         MMU_ROMFLAGS
#define SAM_NFCSRAM_MMUFLAGS     MMU_IOFLAGS
#define SAM_ISRAM_MMUFLAGS       MMU_MEMFLAGS
#define SAM_SMD_MMUFLAGS         MMU_MEMFLAGS
#define SAM_UDPHSRAM_MMUFLAGS    MMU_IOFLAGS
#define SAM_UHPOHCI_MMUFLAGS     MMU_IOFLAGS
#define SAM_UHPEHCI_MMUFLAGS     MMU_IOFLAGS
#define SAM_AXIMX_MMUFLAGS       MMU_IOFLAGS
#define SAM_DAP_MMUFLAGS         MMU_IOFLAGS

#define SAM_EBICS0_MMUFLAGS      MMU_MEMFLAGS
#define SAM_DDRCS_MMUFLAGS       MMU_MEMFLAGS
#define SAM_EBICS1_MMUFLAGS      MMU_MEMFLAGS
#define SAM_EBICS2_MMUFLAGS      MMU_MEMFLAGS
#define SAM_EBICS3_MMUFLAGS      MMU_MEMFLAGS
#define SAM_NFCCR_MMUFLAGS       MMU_IOFLAGS

#define SAM_PERIPHA_MMUFLAGS     MMU_IOFLAGS
#define SAM_PERIPHB_MMUFLAGS     MMU_IOFLAGS
#define SAM_SYSC_MMUFLAGS        MMU_IOFLAGS

/* SAMA5 Virtual (mapped) Memory Map
 *
 * board_memorymap.h contains special mappings that are needed when a ROM
 * memory map is used.  It is included in this odd location becaue it depends
 * on some the virtual address definitions provided above.
 */

#include <arch/board/board_memorymap.h>

/* SAMA5 Virtual (mapped) Memory Map.  These are the mappings that will
 * be created if the page table lies in RAM.  If the platform has another,
 * read-only, pre-initialized page table (perhaps in ROM), then the board.h
 * file must provide these definitions.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE

#define SAM_INTMEM_VSECTION      0x00000000 /* 0x00000000-0x0fffffff: Internal Memories */
#  define SAM_BOOTMEM_VSECTION   0x00000000 /* 0x00000000-0x000fffff: Boot memory */
#  define SAM_ROM_VSECTION       0x00100000 /* 0x00100000-0x001fffff: ROM */
#  define SAM_NFCSRAM_VSECTION   0x00200000 /* 0x00200000-0x002fffff: NFC SRAM */
#  define SAM_ISRAM_VSECTION     0x00300000 /* 0x00300000-0x0030ffff: SRAM */
#    define SAM_ISRAM0_VADDR     0x00300000 /* 0x00300000-0x0030ffff: SRAM0 */
#    define SAM_ISRAM1_VADDR     0x00310000 /* 0x00310000-0x003fffff: SRAM1 */
#  define SAM_SMD_VSECTION       0x00400000 /* 0x00400000-0x004fffff: SMD */
#  define SAM_UDPHSRAM_VSECTION  0x00500000 /* 0x00500000-0x005fffff: UDPH SRAM */
#  define SAM_UHPOHCI_VSECTION   0x00600000 /* 0x00600000-0x006fffff: UHP OHCI */
#  define SAM_UHPEHCI_VSECTION   0x00700000 /* 0x00700000-0x007fffff: UHP EHCI */
#  define SAM_AXIMX_VSECTION     0x00800000 /* 0x00800000-0x008fffff: AXI Matrix */
#  define SAM_DAP_VSECTION       0x00900000 /* 0x00900000-0x009fffff: DAP */
#define SAM_EBICS0_VSECTION      0x10000000 /* 0x10000000-0x1fffffff: EBI Chip select 0 */
#define SAM_DDRCS_VSECTION       0x20000000 /* 0x20000000-0x3fffffff: EBI DDRCS */
#define SAM_EBICS1_VSECTION      0x40000000 /* 0x40000000-0x4fffffff: EBI Chip select 1 */
#define SAM_EBICS2_VSECTION      0x50000000 /* 0x50000000-0x5fffffff: EBI Chip select 2 */
#define SAM_EBICS3_VSECTION      0x60000000 /* 0x60000000-0x6fffffff: EBI Chip select 2 */
#define SAM_NFCCR_VSECTION       0x70000000 /* 0x70000000-0x7fffffff: NFC Command Registers */
                                            /* 0x80000000-0xefffffff: Undefined */
#define SAM_PERIPH_VSECTION      0xf0000000 /* 0xf0000000-0xffffffff: Internal Peripherals */
#  define SAM_PERIPHA_VSECTION   0xf0000000 /* 0xf0000000-0xffffffff: Internal Peripherals */
#  define SAM_PERIPHB_VSECTION   0xf8000000 /* 0xf8000000-0xffffbfff: Internal Peripherals B */
#  define SAM_SYSC_VSECTION      0xff000000 /* 0xff000000-0xffffffff: System Controller */
#  define SAM_SYSC_VADDR         0xffffc000 /* 0xffffc000-0xffffffff: System Controller */

#endif

/* Peripheral virtual base addresses */

#define SAM_HSMCI0_VBASE         (SAM_PERIPHA_VSECTION+SAM_HSMCI0_OFFSET)
#define SAM_SPI0_VBASE           (SAM_PERIPHA_VSECTION+SAM_SPI0_OFFSET)
#define SAM_SSC0_VBASE           (SAM_PERIPHA_VSECTION+SAM_SSC0_OFFSET)
#define SAM_CAN0_VBASE           (SAM_PERIPHA_VSECTION+SAM_CAN0_OFFSET)
#define SAM_TC012_VBASE          (SAM_PERIPHA_VSECTION+SAM_TC012_OFFSET)
#define SAM_TWI0_VBASE           (SAM_PERIPHA_VSECTION+SAM_TWI0_OFFSET)
#define SAM_TWI1_VBASE           (SAM_PERIPHA_VSECTION+SAM_TWI1_OFFSET)
#define SAM_USART0_VBASE         (SAM_PERIPHA_VSECTION+SAM_USART0_OFFSET)
#define SAM_USART1_VBASE         (SAM_PERIPHA_VSECTION+SAM_USART1_OFFSET)
#define SAM_UART0_VBASE          (SAM_PERIPHA_VSECTION+SAM_UART0_OFFSET)
#define SAM_GMAC_VBASE           (SAM_PERIPHA_VSECTION+SAM_GMAC_OFFSET)
#define SAM_PWMC_VBASE           (SAM_PERIPHA_VSECTION+SAM_PWMC_OFFSET)
#define SAM_LCDC_VBASE           (SAM_PERIPHA_VSECTION+SAM_LCDC_OFFSET)
#define SAM_ISI_VBASE            (SAM_PERIPHA_VSECTION+SAM_ISI_OFFSET)
#define SAM_SFR_VBASE            (SAM_PERIPHA_VSECTION+SAM_SFR_OFFSET)

#define SAM_HSMCI1_VBASE         (SAM_PERIPHB_VSECTION+SAM_HSMCI1_OFFSET)
#define SAM_HSMCI2_VBASE         (SAM_PERIPHB_VSECTION+SAM_HSMCI2_OFFSET)
#define SAM_SPI1_VBASE           (SAM_PERIPHB_VSECTION+SAM_SPI1_OFFSET)
#define SAM_SSC1_VBASE           (SAM_PERIPHB_VSECTION+SAM_SSC1_OFFSET)
#define SAM_CAN1_VBASE           (SAM_PERIPHB_VSECTION+SAM_CAN1_OFFSET)
#define SAM_TC345_VBASE          (SAM_PERIPHB_VSECTION+SAM_TC345_OFFSET)
#define SAM_TSADC_VBASE          (SAM_PERIPHB_VSECTION+SAM_TSADC_OFFSET)
#define SAM_TWI2_VBASE           (SAM_PERIPHB_VSECTION+SAM_TWI2_OFFSET)
#define SAM_USART2_VBASE         (SAM_PERIPHB_VSECTION+SAM_USART2_OFFSET)
#define SAM_USART3_VBASE         (SAM_PERIPHB_VSECTION+SAM_USART3_OFFSET)
#define SAM_UART1_VBASE          (SAM_PERIPHB_VSECTION+SAM_UART1_OFFSET)
#define SAM_EMAC_VBASE           (SAM_PERIPHB_VSECTION+SAM_EMAC_OFFSET)
#define SAM_UDPHS_VBASE          (SAM_PERIPHB_VSECTION+SAM_UDPHS_OFFSET)
#define SAM_SHA_VBASE            (SAM_PERIPHB_VSECTION+SAM_SHA_OFFSET)
#define SAM_AES_VBASE            (SAM_PERIPHB_VSECTION+SAM_AES_OFFSET)
#define SAM_TDES_VBASE           (SAM_PERIPHB_VSECTION+SAM_TDES_OFFSET)
#define SAM_TRNG_VBASE           (SAM_PERIPHB_VSECTION+SAM_TRNG_OFFSET)

#define SAM_HSMC_VBASE           (SAM_SYSC_VSECTION+SAM_HSMC_OFFSET)
#define SAM_FUSE_VBASE           (SAM_SYSC_VSECTION+SAM_FUSE_OFFSET)
#define SAM_DMAC0_VBASE          (SAM_SYSC_VSECTION+SAM_DMAC0_OFFSET)
#define SAM_DMAC1_VBASE          (SAM_SYSC_VSECTION+SAM_DMAC1_OFFSET)
#define SAM_MPDDRC_VBASE         (SAM_SYSC_VSECTION+SAM_MPDDRC_OFFSET)
#define SAM_MATRIX_VBASE         (SAM_SYSC_VSECTION+SAM_MATRIX_OFFSET)
#define SAM_DBGU_VBASE           (SAM_SYSC_VSECTION+SAM_DBGU_OFFSET)
#define SAM_AIC_VBASE            (SAM_SYSC_VSECTION+SAM_AIC_OFFSET)
#define SAM_PION_VBASE(n)        (SAM_SYSC_VSECTION+SAM_PION_OFFSET(n))
#define SAM_PIOA_VBASE           (SAM_SYSC_VSECTION+SAM_PIOA_OFFSET)
#define SAM_PIOB_VBASE           (SAM_SYSC_VSECTION+SAM_PIOB_OFFSET)
#define SAM_PIOC_VBASE           (SAM_SYSC_VSECTION+SAM_PIOC_OFFSET)
#define SAM_PIOD_VBASE           (SAM_SYSC_VSECTION+SAM_PIOD_OFFSET)
#define SAM_PIOE_VBASE           (SAM_SYSC_VSECTION+SAM_PIOE_OFFSET)
#define SAM_PMC_VBASE            (SAM_SYSC_VSECTION+SAM_PMC_OFFSET)
#define SAM_RSTC_VBASE           (SAM_SYSC_VSECTION+SAM_RSTC_OFFSET)
#define SAM_SHDC_VBASE           (SAM_SYSC_VSECTION+SAM_SHDC_OFFSET)
#define SAM_PITC_VBASE           (SAM_SYSC_VSECTION+SAM_PITC_OFFSET)
#define SAM_WDT_VBASE            (SAM_SYSC_VSECTION+SAM_WDT_OFFSET)
#define SAM_SCKCR_VBASE          (SAM_SYSC_VSECTION+SAM_SCKCR_OFFSET)
#define SAM_BSC_VBASE            (SAM_SYSC_VSECTION+SAM_BSC_OFFSET)
#define SAM_GPBR_VBASE           (SAM_SYSC_VSECTION+SAM_GPBR_OFFSET)
#define SAM_RTCC_VBASE           (SAM_SYSC_VSECTION+SAM_RTCC_OFFSET)

/* NuttX vitual base address
 *
 * The boot logic will create a temporarily mapping based on where NuttX is
 * executing in memory.  In this case, NuttX could be running from NOR FLASH,
 * SDRAM, external SRAM, or internal SRAM.
 */

#if defined(CONFIG_BOOT_RUNFROMFLASH)
#  define NUTTX_START_VADDR       CONFIG_SAMA5_NORFLASH_VBASE
#elif defined(CONFIG_BOOT_RUNFROMSDRAM)
#  define NUTTX_START_VADDR       SAM_DDRCS_VSECTION
#elif defined(CONFIG_BOOT_RUNFROMEXTSRAM)
#  define NUTTX_START_VADDR       CONFIG_SAMA5_SRAM_VBASE
#else /* CONFIG_BOOT_RUNFROMISRAM, CONFIG_PAGING */
#  define NUTTX_START_VADDR       SAM_ISRAM_VSECTION
#endif

/* MMU Page Table Location
 *
 * Determine the address of the MMU page table.  We will try to place that page
 * table at the beginng of ISRAM0 if the vectors are at the high address, 0xffff:0000
 * or at the end of ISRAM1 (or ISRAM0 if ISRAM1 is not available in this architecture)
 * if the vectors are at 0x0000:0000
 *
 * Or... the user may specify the address of the page table explicitly be defining
 * CONFIG_PGTABLE_VADDR and CONFIG_PGTABLE_PADDR in the configuration or board.h file.
 */

#undef PGTABLE_IN_HIGHSRAM
#undef PGTABLE_IN_LOWSRAM

#if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)

  /* Sanity check.. if one is undefined, both should be undefined */

#  if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif

  /* A sanity check, if the configuration says that the page table is read-only
   * and pre-initialized (maybe ROM), then it should have also defined both of
   * the page table base addresses.
   */

#  ifdef CONFIG_ARCH_ROMPGTABLE
#    error "CONFIG_ARCH_ROMPGTABLE defined; PGTABLE_BASE_P/VADDR not defined"
#  endif

  /* If CONFIG_PAGING is selected, then parts of the 1-to-1 virtual memory
   * map probably do not apply because paging logic will probably partition
   * the SRAM section differently.  In particular, if the page table is located
   * at the end of SRAM, then the virtual page table address defined below
   * will probably be in error.  In that case PGTABLE_BASE_VADDR is defined
   * in the file mmu.h
   *
   * We must declare the page table in ISRAM0 or 1.  We decide depending upon
   * where the vector table was place.
   */

#  ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */

  /* In this case, table must lie at the top 16Kb of ISRAM1 (or ISRAM0 if ISRAM1
   * is not available in this architecture)
   *
   * If CONFIG_PAGING is defined, then mmu.h assign the virtual address
   * of the page table.
   */

#    if SAM_ISRAM1_SIZE > 0
#        define PGTABLE_BASE_PADDR (SAM_ISRAM1_PADDR+SAM_ISRAM1_SIZE-PGTABLE_SIZE)
#        ifndef CONFIG_PAGING
#          define PGTABLE_BASE_VADDR (SAM_ISRAM1_VADDR+SAM_ISRAM1_SIZE-PGTABLE_SIZE)
#        endif
#    else
#        define PGTABLE_BASE_PADDR (SAM_ISRAM0_PADDR+SAM_ISRAM0_SIZE-PGTABLE_SIZE)
#        ifndef CONFIG_PAGING
#          define PGTABLE_BASE_VADDR (SAM_ISRAM0_VADDR+SAM_ISRAM0_SIZE-PGTABLE_SIZE)
#        endif
#    endif
#    define PGTABLE_IN_HIGHSRAM   1
#  else

  /* Otherwise, ISRAM1 (or ISRAM0 if ISRAM1 is not available in this
   * architecture) will be mapped so that the end of the SRAM region will
   * provide memory for the vectors.  The page table will then be places at
   * the first 16Kb of ISRAM0.
   */

#    define PGTABLE_BASE_PADDR    SAM_ISRAM0_PADDR
#    ifndef CONFIG_PAGING
#      define PGTABLE_BASE_VADDR  SAM_ISRAM0_VADDR
#    endif
#    define PGTABLE_IN_LOWSRAM    1
#  endif
#endif

/* Page table start addresses.
 *
 * 16Kb of memory is reserved hold the page table for the virtual mappings.  A
 * portion of this table is not accessible in the virtual address space (for
 * normal operation). We will reuse this memory for coarse page tables as follows:
 *
 * NOTE: If CONFIG_PAGING is defined, mmu.h will re-assign the virtual address
 * of the page table.
 */

#define PGTABLE_L2_PBASE          (PGTABLE_BASE_PADDR+0x00000800)
#define PGTABLE_L2_VBASE          (PGTABLE_BASE_VADDR+0x00000800)

/* Page table end addresses: */

#define PGTABLE_L2_END_PADDR      (PGTABLE_BASE_PADDR+PGTABLE_SIZE)
#define PGTABLE_L2_END_VADDR      (PGTABLE_BASE_VADDR+PGTABLE_SIZE)

/* Page table sizes */

#define PGTABLE_L2_ALLOC          (PGTABLE_L2_END_VADDR-PGTABLE_L2_VBASE)
#define PGTABLE_L2_SIZE           (4*256)
#define PGTABLE_L2_NENTRIES       (PGTABLE_L2_ALLOC / PGTABLE_L2_SIZE)

/* Base address of the interrupt vector table.
 *
 *   SAM_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
 *   SAM_VECTOR_VSRAM - Virtual address of vector table in SRAM
 *   SAM_VECTOR_VADDR - Virtual address of vector table (0x00000000 or 0xffff0000)
 */

#define VECTOR_TABLE_SIZE         0x00010000
#ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */
#  define SAM_VECTOR_PADDR        SAM_ISRAM0_PADDR
#  define SAM_VECTOR_VSRAM        SAM_ISRAM0_VADDR
#  define SAM_VECTOR_VADDR        0x00000000
#  define SAM_VECTOR_VCOARSE      0x00000000
#else  /* Vectors located at 0xffff:0000 -- this probably does not work */
#  ifdef HAVE_ISRAM1
#    define SAM_VECTOR_PADDR      (SAM_ISRAM1_PADDR+SAM_ISRAM1_SIZE-VECTOR_TABLE_SIZE)
#    define SAM_VECTOR_VSRAM      (SAM_ISRAM1_VADDR+SAM_ISRAM1_SIZE-VECTOR_TABLE_SIZE)
#  else
#    define SAM_VECTOR_PADDR      (SAM_ISRAM0_PADDR+SAM_ISRAM0_SIZE-VECTOR_TABLE_SIZE)
#    define SAM_VECTOR_VSRAM      (SAM_ISRAM0_VADDR+SAM_ISRAM0_SIZE-VECTOR_TABLE_SIZE)
#  endif
#  define SAM_VECTOR_VADDR        0xffff0000
#  define SAM_VECTOR_VCOARSE      0xfff00000
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

#endif /* __ARCH_ARM_SRC_SAMA5_SAMA5D3X_MEMORYMAP_H */
