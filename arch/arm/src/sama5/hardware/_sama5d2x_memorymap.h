/****************************************************************************
 * arch/arm/src/sama5/hardware/_sama5d2x_memorymap.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D2X_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D2X_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/sama5/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Decimal configuration values may exceed 2Gb and, hence, overflow to
 * negative values unless we force them to unsigned long:
 */

#define __CONCAT(a,b) a ## b
#define MKULONG(a) __CONCAT(a,ul)

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
 *  - NuttX virtual base address
 *  MMU Page Table Location
 *  Page table start addresses
 *  Base address of the interrupt vector table
 */

/* SAMA5D2 Physical (unmapped) Memory Map */

#define SAM_INTMEM_PSECTION      0x00000000 /* 0x00000000-0x0fffffff: Internal Memories */
#define SAM_EBICS0_PSECTION      0x10000000 /* 0x10000000-0x1fffffff: EBI Chip Select 0 */
#define SAM_DDRCS_PSECTION       0x20000000 /* 0x20000000-0x3fffffff: EBI DDR Chip Select */
#define SAM_DDRAESCS_PSECTION    0x40000000 /* 0x40000000-0x5fffffff: EBI DDR AES Chip Select */
#define SAM_EBICS1_PSECTION      0x60000000 /* 0x60000000-0x6fffffff: EBI Chip select 1 */
#define SAM_EBICS2_PSECTION      0x70000000 /* 0x70000000-0x7fffffff: EBI Chip select 2 */
#define SAM_EBICS3_PSECTION      0x80000000 /* 0x80000000-0x8fffffff: EBI Chip select 3 */
#define SAM_QSPI0AES_PSECTION    0x90000000 /* 0x90000000-0x97ffffff: QSPI0 AES Memory */
#define SAM_QSPI1AES_PSECTION    0x98000000 /* 0x98000000-0x9fffffff: QSPI1 AES Memory */
#define SAM_SDMMC0_PSECTION      0xa0000000 /* 0xa0000000-0xafffffff: SDMMC0 */
#define SAM_SDMMC1_PSECTION      0xb0000000 /* 0xb0000000-0xbfffffff: SDMMC1 */
#define SAM_NFCCR_PSECTION       0xc0000000 /* 0xc0000000-0xcfffffff: NFC Command Registers */
#define SAM_QSPI0_PSECTION       0xd0000000 /* 0xd0000000-0xd7ffffff: QSPI0 Memory */
#define SAM_QSPI1_PSECTION       0xd8000000 /* 0xd8000000-0xdfffffff: QSPI1 Memory */
                                            /* 0xe0000000-0xefffffff: Undefined */
#define SAM_PERIPH_PSECTION      0xf0000000 /* 0xf0000000-0xffffffff: Internal Peripherals */

/* SAMA5D2 Internal Memories */

#define SAM_ROM_PSECTION         0x00000000 /* 0x00000000-0x0003ffff: ROM */
#define SAM_SRAMREMAP_PSECTION   0x00000000 /* 0x00000000-0x0003ffff: Remapped ISRAM */
#define SAM_ECCROM_PSECTION      0x00040000 /* 0x00040000-0x000fffff: EEC ROM */
#define SAM_NFCSRAM_PSECTION     0x00100000 /* 0x00100000-0x001fffff: NFC SRAM */
#define SAM_ISRAM_PSECTION       0x00200000 /* 0x00200000-0x002fffff: SRAM */
#  define SAM_ISRAM0_PADDR       0x00200000 /* 0x00200000-0x0021ffff: SRAM0 */
#  define SAM_ISRAM1_PADDR       0x00220000 /* 0x00220000-0x002fffff: SRAM1 */
#define SAM_UDPHSRAM_PSECTION    0x00300000 /* 0x00300000-0x004fffff: UDPH SRAM */
#define SAM_UHPOHCI_PSECTION     0x00400000 /* 0x00400000-0x005fffff: UHP OHCI */
#define SAM_UHPEHCI_PSECTION     0x00500000 /* 0x00500000-0x005fffff: UHP EHCI */
#define SAM_AXIMX_PSECTION       0x00600000 /* 0x00600000-0x006fffff: AXI Matrix */
#define SAM_DAP_PSECTION         0x00700000 /* 0x00700000-0x007fffff: DAP */
                                            /* 0x00800000-0x009fffff: Reserved */
#define SAM_L2CC_PSECTION        0x00a00000 /* 0x00a00000-0x00bfffff: L2CC */
                                            /* 0x00c00000-0x00ffffff: Undefined (Abort) */

/* SAMA5D2 Internal Peripheral Offsets */

#define SAM_PERIPHA_PSECTION     0xf0000000 /* 0xf0000000-0xf7ffffff: Internal Peripherals A */
#  define SAM_LCDC_OFFSET        0x00000000 /* 0x00000000-0x00003fff: LCDC */
#  define SAM_XDMAC1_OFFSET      0x00004000 /* 0x00004000-0x00007fff: XDMAC1 */
#  define SAM_ISC_OFFSET         0x00008000 /* 0x00008000-0x0000bfff: ISC */
#  define SAM_MPDDRC_OFFSET      0x0000c000 /* 0x0000c000-0x0000ffff: MPDDRC */
#  define SAM_XDMAC0_OFFSET      0x00010000 /* 0x00010000-0x00013fff: XDMAC0 */
#  define SAM_PMC_OFFSET         0x00014000 /* 0x00014000-0x00017fff: PMC */
#  define SAM_MATRIX0_OFFSET     0x00018000 /* 0x00018000-0x0001bfff: MATRIX0 */
#  define SAM_AESB_OFFSET        0x0001c000 /* 0x0001c000-0x0001ffff: AESB */
#  define SAM_QSPI0_OFFSET       0x00020000 /* 0x00020000-0x00023fff: QSPI0 */
#  define SAM_QSPI1_OFFSET       0x00024000 /* 0x00024000-0x00027fff: QSPI1 */
#  define SAM_SHA_OFFSET         0x00028000 /* 0x00028000-0x0002bfff: SHA */
#  define SAM_AES_OFFSET         0x0002c000 /* 0x0002c000-0x0002ffff: AES */
                                            /* 0x00030000-0xf7ffffff: Reserved */
#define SAM_PERIPHB_PSECTION     0xf8000000 /* 0xf8000000-0xfbffffff: Internal Peripherals B */
#  define SAM_SPI0_OFFSET        0x00000000 /* 0x00000000-0x00003fff: SPI0 */
#  define SAM_SSC0_OFFSET        0x00004000 /* 0x00004000-0x00007fff: SSC0 */
#  define SAM_EMAC0_OFFSET       0x00008000 /* 0x00008000-0x0000bfff: GMAC 0 */
#  define SAM_TC012_OFFSET       0x0000c000 /* 0x0000c000-0x0000ffff: TC channels 0, 1, and 2 */
#  define SAM_TC345_OFFSET       0x00010000 /* 0x00010000-0x00013fff: TC channels 3, 4, and 5 */
#  define SAM_HSMC_OFFSET        0x00014000 /* 0x00014000-0x00017fff: HSMC */
#  define SAM_PDMIC_OFFSET       0x00018000 /* 0x00018000-0x0001bfff: HSMC */
#  define SAM_UART0_OFFSET       0x0001c000 /* 0x0001c000-0x0001ffff: UART0 */
#  define SAM_UART1_OFFSET       0x00020000 /* 0x00020000-0x00023fff: UART1 */
#  define SAM_UART2_OFFSET       0x00024000 /* 0x00024000-0x00027fff: UART2 */
#  define SAM_TWI0_OFFSET        0x00028000 /* 0x00028000-0x0002bfff: TWIHS0 */
#  define SAM_PWMC_OFFSET        0x0002c000 /* 0x0002c000-0x0002ffff: PWMC */
#  define SAM_SFR_OFFSET         0x00030000 /* 0x00030000-0x00033fff: SFR */
#  define SAM_FLEXCOM0_OFFSET    0x00034000 /* 0x00034000-0x00037fff: FLEXCOM0 */
#  define SAM_FLEXCOM1_OFFSET    0x00038000 /* 0x00038000-0x0003bfff: FLEXCOM1 */
#  define SAM_SAIC_OFFSET        0x0003C000 /* 0x0003C000-0x0003ffff: SAIC */
#  define SAM_ICM_OFFSET         0x00040000 /* 0x00040000-0x00043fff: ICM */
#  define SAM_SECURAM_OFFSET     0x00044000 /* 0x00044000-0x00044fff: SECURAM */
#  define SAM_SYSC_OFFSET        0x00048000 /* 0x00048000-0x00048fff: System Controller peripherals */
#  define SAM_RSTC_OFFSET        0x00048000 /* 0x00048000-0x0004800f: RSTC */
#  define SAM_SHDWC_OFFSET       0x00048010 /* 0x00048010-0x0004802f: SHDWC */
#  define SAM_PITC_OFFSET        0x00048030 /* 0x00048030-0x0004803f: PITC */
#  define SAM_WDT_OFFSET         0x00048040 /* 0x00048040-0x0004804f: WDT */
#  define SAM_SCKCR_OFFSET       0x00048050 /* 0x00048050-0x000480af: SCKCR */
#  define SAM_RTCC_OFFSET        0x000480b0 /* 0x000480b0-0x00048fff: RTCC */
#  define SAM_RXLP_OFFSET        0x00049000 /* 0x00049000-0x00049fff: RXLP */
#  define SAM_ACC_OFFSET         0x0004a000 /* 0x0004a000-0x0004afff: ACC */
                                            /* 0x0004b000-0x0004bfff: Reserved */
#  define SAM_SFC_OFFSET         0x0004c000 /* 0x0004c000-0x0004ffff: SFC */
#  define SAM_I2SC0_OFFSET       0x00050000 /* 0x00050000-0x00053fff: I2SC0 */
#  define SAM_CAN0_OFFSET        0x00054000 /* 0x00054000-0x00057fff: CAN0 */
#  define SAM_SYSC_PSECTION      0xf8048000 /* 0xf8048000-0xf8048fff: System Controller */
#  define SAM_SYSC_PADDR         0xf8048000 /* 0xf8048000-0xf8048fff: System Controller */

#define SAM_PERIPHC_PSECTION     0xfc000000 /* 0xfc000000-0xffffffff: Internal Peripherals C */
#  define SAM_SPI1_OFFSET        0x00000000 /* 0x00000000-0x00003fff: SPI1 */
#  define SAM_SSC1_OFFSET        0x00004000 /* 0x00004000-0x00007fff: SSC1 */
#  define SAM_UART3_OFFSET       0x00008000 /* 0x00008000-0x0000bfff: UART3 */
#  define SAM_UART4_OFFSET       0x0000c000 /* 0x0000c000-0x0000ffff: UART4 */
#  define SAM_FLEXCOM2_OFFSET    0x00010000 /* 0x00010000-0x00013fff: FLEXCOM2 */
#  define SAM_FLEXCOM3_OFFSET    0x00014000 /* 0x00014000-0x00017fff: FLEXCOM3 */
#  define SAM_FLEXCOM4_OFFSET    0x00018000 /* 0x00018000-0x0001bfff: FLEXCOM4 */
#  define SAM_TRNG_OFFSET        0x0001c000 /* 0x0001c000-0x0001ffff: TRNG */
#  define SAM_AIC_OFFSET         0x00020000 /* 0x00020000-0x00023fff: AIC */
                                            /* 0x00024000-0x00027fff: Reserved */
#  define SAM_TWI1_OFFSET        0x00028000 /* 0x00028000-0x0002bfff: TWIHS1 */
#  define SAM_UDPHS_OFFSET       0x0002c000 /* 0x0002c000-0x0002ffff: UDPHS */
#  define SAM_ADC_OFFSET         0x00030000 /* 0x00030000-0x00033fff: ADC */
                                            /* 0x00034000-0x00037fff: Reserved */
#  define SAM_PIO_OFFSET         0x00038000 /* 0x00038000-0x0003bfff: PIOA-D */
#  define SAM_MATRIX1_OFFSET     0x0003c000 /* 0x0003c000-0x0003ffff: MATRIX1 */
#  define SAM_SECUMOD_OFFSET     0x00040000 /* 0x00040000-0x00043fff: SECUMOD */
#  define SAM_TDES_OFFSET        0x00044000 /* 0x00044000-0x00047fff: TDES */
#  define SAM_CLASSD_OFFSET      0x00048000 /* 0x00048000-0x0004bfff: Class D */
#  define SAM_I2SC1_OFFSET       0x0004c000 /* 0x0004c000-0x0004ffff: I2SC1 */
#  define SAM_CAN1_OFFSET        0x00050000 /* 0x00050000-0x00053fff: CAN1 */
#  define SAM_UTMI_OFFSET        0x00054000 /* 0x00054000-0x00057fff: UTMI */
                                            /* 0x00058000-0x0005bfff: Reserved */
#  define SAM_SFRBU_OFFSET       0x0005c000 /* 0x0005c000-0x0005ffff: SFRBU */
                                            /* 0x00060000-0x00068fff: Reserved */
#  define SAM_CHIPID_OFFSET      0x00069000 /* 0x00069000-0x0006bfff: SFRBU */
                                            /* 0x0006a000-0xffffffff: Reserved */

/* Sizes of memory regions in bytes.
 *
 * These sizes exclude the undefined addresses at the end of the memory
 * region.  The implemented sizes of the EBI CS0-3 and DDRCS regions
 * are not known apriori and must be specified with configuration settings.
 */

                             /* 0x00000000-0x0fffffff: Internal Memories */
#define SAM_ROM_SIZE             (256*1024)      /* 0x00000000-0x0003ffff: ROM */
#ifdef CONFIG_ARMV7A_L2CC_PL310
#  define SAM_SRAMREMAP_SIZE     (128*1024)      /* 0x00000000-0x0001ffff: Remapped ISRAM0 */
#else
#  define SAM_SRAMREMAP_SIZE     (256*1024)      /* 0x00000000-0x0003ffff: Remapped ISRAM0+1 */
#endif
#define SAM_ECCROM_SIZE          (768*1024)      /* 0x00040000-0x000fffff: EEC ROM */
#define SAM_NFCSRAM_SIZE         (1*1024*1024)   /* 0x00100000-0x001fffff: NFC SRAM */
#ifdef CONFIG_ARMV7A_L2CC_PL310
#  define SAM_ISRAM_SIZE         (128*1024)      /* 0x00200000-0x0001ffff: SRAM0 */
#else
#  define SAM_ISRAM_SIZE         (256*1024)      /* 0x00200000-0x0003ffff: SRAM0+1 */
#endif
#define SAM_UDPHSRAM_SIZE        (1*1024*1024)   /* 0x00300000-0x004fffff: UDPH SRAM */
#define SAM_UHPOHCI_SIZE         (1*1024*1024)   /* 0x00400000-0x005fffff: UHP OHCI */
#define SAM_UHPEHCI_SIZE         (1*1024*1024)   /* 0x00500000-0x005fffff: UHP EHCI */
#define SAM_AXIMX_SIZE           (4)             /* 0x00600000-0x006fffff: AXI Matrix */
#define SAM_DAP_SIZE             (1*1024*1024)   /* 0x00700000-0x007fffff: DAP */
#define SAM_L2CC_SIZE            (1*1024*1024)   /* 0x00a00000-0x00bfffff: L2CC */
#define SAM_NFCCR_SIZE           (256*1024*1024) /* 0xc0000000-0xcfffffff: NFC Command Registers */
                                                 /* 0xf0000000-0xffffffff: Internal Peripherals */
#define SAM_PERIPHA_SIZE         (192*1024)      /* 0xf0000000-0xf002ffff: Internal Peripherals A */
#define SAM_PERIPHB_SIZE         (352*1024)      /* 0xf8000000-0xf8057fff: Internal Peripherals B */
#define SAM_PERIPHC_SIZE         (431*1024)      /* 0xfc000000-0xfc06bfff: Internal Peripherals C */
#define SAM_SYSC_SIZE            (1*1024*1024)   /* 0xf8048000-0xf8048fff: Internal Peripherals */

/* Force configured sizes that might exceed 2GB to be unsigned long */

#define SAMA5_EBICS0_SIZE        MKULONG(CONFIG_SAMA5_EBICS0_SIZE)
#define SAMA5_DDRCS_SIZE         MKULONG(CONFIG_SAMA5_DDRCS_SIZE)
#define SAMA5_DDRAESCS_SIZE      MKULONG(CONFIG_SAMA5_DDRAESCS_SIZE)
#define SAMA5_EBICS1_SIZE        MKULONG(CONFIG_SAMA5_EBICS1_SIZE)
#define SAMA5_EBICS2_SIZE        MKULONG(CONFIG_SAMA5_EBICS2_SIZE)
#define SAMA5_EBICS3_SIZE        MKULONG(CONFIG_SAMA5_EBICS3_SIZE)
#define SAMA5_QSPI0AES_SIZE      MKULONG(CONFIG_SAMA5_QSPI0AES_SIZE)
#define SAMA5_QSPI1AES_SIZE      MKULONG(CONFIG_SAMA5_QSPI1AES_SIZE)
#define SAMA5_SDMMC0_SIZE        MKULONG(CONFIG_SAMA5_SDMMC0_SIZE)
#define SAMA5_SDMMC1_SIZE        MKULONG(CONFIG_SAMA5_SDMMC1_SIZE)
#define SAMA5_QSPI0_SIZE         MKULONG(CONFIG_SAMA5_QSPI0_SIZE)
#define SAMA5_QSPI1_SIZE         MKULONG(CONFIG_SAMA5_QSPI1_SIZE)

#define SAMA5_EBICS0_HEAP_OFFSET   MKULONG(CONFIG_SAMA5_EBICS0_HEAP_OFFSET)
#define SAMA5_DDRCS_HEAP_OFFSET    MKULONG(CONFIG_SAMA5_DDRCS_HEAP_OFFSET)
#define SAMA5_DDRAES_HEAP_OFFSET   MKULONG(CONFIG_SAMA5_DDRAES_HEAP_OFFSET)
#define SAMA5_EBICS1_HEAP_OFFSET   MKULONG(CONFIG_SAMA5_EBICS1_HEAP_OFFSET)
#define SAMA5_EBICS2_HEAP_OFFSET   MKULONG(CONFIG_SAMA5_EBICS2_HEAP_OFFSET)
#define SAMA5_EBICS3_HEAP_OFFSET   MKULONG(CONFIG_SAMA5_EBICS3_HEAP_OFFSET)
#define SAMA5_QSPI0AES_HEAP_OFFSET MKULONG(CONFIG_SAMA5_QSPI0AES_HEAP_OFFSET)
#define SAMA5_QSPI1AES_HEAP_OFFSET MKULONG(CONFIG_SAMA5_QSPI1AES_HEAP_OFFSET)
#define SAMA5_SDMMC0_HEAP_OFFSET   MKULONG(CONFIG_SAMA5_SDMMC0_HEAP_OFFSET)
#define SAMA5_SDMMC1_HEAP_OFFSET   MKULONG(CONFIG_SAMA5_SDMMC1_HEAP_OFFSET)
#define SAMA5_QSPI0_HEAP_OFFSET    MKULONG(CONFIG_SAMA5_QSPI0_HEAP_OFFSET)
#define SAMA5_QSPI1_HEAP_OFFSET    MKULONG(CONFIG_SAMA5_QSPI1_HEAP_OFFSET)

#define SAMA5_EBICS0_HEAP_SIZE   MKULONG(CONFIG_SAMA5_EBICS0_HEAP_SIZE)
#define SAMA5_DDRCS_HEAP_SIZE    MKULONG(CONFIG_SAMA5_DDRCS_HEAP_SIZE)
#define SAMA5_DDRAESCS_HEAP_SIZE MKULONG(CONFIG_SAMA5_DDRAESCS_HEAP_SIZE)
#define SAMA5_EBICS1_HEAP_SIZE   MKULONG(CONFIG_SAMA5_EBICS1_HEAP_SIZE)
#define SAMA5_EBICS2_HEAP_SIZE   MKULONG(CONFIG_SAMA5_EBICS2_HEAP_SIZE)
#define SAMA5_EBICS3_HEAP_SIZE   MKULONG(CONFIG_SAMA5_EBICS3_HEAP_SIZE)
#define SAMA5_QSPI0AES_HEAP_SIZE MKULONG(CONFIG_SAMA5_QSPI0AES_HEAP_SIZE)
#define SAMA5_QSPI1AES_HEAP_SIZE MKULONG(CONFIG_SAMA5_QSPI1AES_HEAP_SIZE)
#define SAMA5_SDMMC0_HEAP_SIZE   MKULONG(CONFIG_SAMA5_SDMMC0_HEAP_SIZE)
#define SAMA5_SDMMC1_HEAP_SIZE   MKULONG(CONFIG_SAMA5_SDMMC1_HEAP_SIZE)
#define SAMA5_QSPI0_HEAP_SIZE    MKULONG(CONFIG_SAMA5_QSPI0_HEAP_SIZE)
#define SAMA5_QSPI1_HEAP_SIZE    MKULONG(CONFIG_SAMA5_QSPI1_HEAP_SIZE)

/* Convert size in bytes to number of sections (in Mb). */

#define _NSECTIONS(b)            (((b)+0x000fffff) >> 20)

/* Sizes of memory regions in sections.
 *
 * The boot logic in sam_boot.c, will select 1Mb level 1 MMU mappings to
 * span the entire physical address space.  The definitions below specify
 * the number of 1Mb entries that are required to span a particular address
 * region.
 */

#define SAM_ROM_NSECTIONS        _NSECTIONS(SAM_ROM_SIZE)
#define SAM_SRAMREMAP_NSECTIONS  _NSECTIONS(SAM_SRAMREMAP_SIZE)
#define SAM_ECCROM_NSECTIONS     _NSECTIONS(SAM_ECCROM_SIZE)
#define SAM_NFCSRAM_NSECTIONS    _NSECTIONS(SAM_NFCSRAM_SIZE)
#define SAM_ISRAM_NSECTIONS      _NSECTIONS(SAM_ISRAM_SIZE)
#define SAM_UDPHSRAM_NSECTIONS   _NSECTIONS(SAM_UDPHSRAM_SIZE)
#define SAM_UHPOHCI_NSECTIONS    _NSECTIONS(SAM_UHPOHCI_SIZE)
#define SAM_UHPEHCI_NSECTIONS    _NSECTIONS(SAM_UHPEHCI_SIZE)
#define SAM_AXIMX_NSECTIONS      _NSECTIONS(SAM_AXIMX_SIZE)
#define SAM_DAP_NSECTIONS        _NSECTIONS(SAM_DAP_SIZE)
#define SAM_L2CC_NSECTIONS       _NSECTIONS(SAM_L2CC_SIZE)

#define SAM_EBICS0_NSECTIONS     _NSECTIONS(SAMA5_EBICS0_SIZE)
#define SAM_DDRCS_NSECTIONS      _NSECTIONS(SAMA5_DDRCS_SIZE)
#define SAM_DDRAESCS_NSECTIONS   _NSECTIONS(SAMA5_DDRAESCS_SIZE)
#define SAM_EBICS1_NSECTIONS     _NSECTIONS(SAMA5_EBICS1_SIZE)
#define SAM_EBICS2_NSECTIONS     _NSECTIONS(SAMA5_EBICS2_SIZE)
#define SAM_EBICS3_NSECTIONS     _NSECTIONS(SAMA5_EBICS3_SIZE)
#define SAM_QSPI0AES_NSECTIONS   _NSECTIONS(SAMA5_QSPI0AES_SIZE)
#define SAM_QSPI1AES_NSECTIONS   _NSECTIONS(SAMA5_QSPI1AES_SIZE)
#define SAM_SDMMC0_NSECTIONS     _NSECTIONS(SAMA5_SDMMC0_SIZE)
#define SAM_SDMMC1_NSECTIONS     _NSECTIONS(SAMA5_SDMMC1_SIZE)
#define SAM_QSPI0_NSECTIONS      _NSECTIONS(SAMA5_QSPI0_SIZE)
#define SAM_QSPI1_NSECTIONS      _NSECTIONS(SAMA5_QSPI1_SIZE)
#define SAM_NFCCR_NSECTIONS      _NSECTIONS(SAM_NFCCR_SIZE)

#define SAM_PERIPHA_NSECTIONS    _NSECTIONS(SAM_PERIPHA_SIZE)
#define SAM_PERIPHB_NSECTIONS    _NSECTIONS(SAM_PERIPHB_SIZE)
#define SAM_PERIPHC_NSECTIONS    _NSECTIONS(SAM_PERIPHC_SIZE)

/* Section MMU Flags */

#define SAM_ROM_MMUFLAGS         MMU_ROMFLAGS
#define SAM_SRAMREMAP_MMUFLAGS   MMU_MEMFLAGS
#define SAM_ECCROM_MMUFLAGS      MMU_ROMFLAGS
#define SAM_ISRAM_MMUFLAGS       MMU_MEMFLAGS
#define SAM_UDPHSRAM_MMUFLAGS    MMU_IOFLAGS
#define SAM_UHPOHCI_MMUFLAGS     MMU_IOFLAGS
#define SAM_UHPEHCI_MMUFLAGS     MMU_IOFLAGS
#define SAM_AXIMX_MMUFLAGS       MMU_IOFLAGS
#define SAM_DAP_MMUFLAGS         MMU_IOFLAGS
#define SAM_L2CC_MMUFLAGS        MMU_IOFLAGS
#define SAM_SDMMC0_MMUFLAGS      MMU_IOFLAGS
#define SAM_SDMMC1_MMUFLAGS      MMU_IOFLAGS

/* If the NFC is not being used, the NFC SRAM can be used as general purpose
 * SRAM (cached).  If the NFC is used, then the NFC SRAM should be treated
 * as an I/O devices (uncached).
 */

#ifdef CONFIG_SAMA5_HAVE_NAND
#  define SAM_NFCSRAM_MMUFLAGS   MMU_IOFLAGS
#else
#  define SAM_NFCSRAM_MMUFLAGS   MMU_MEMFLAGS
#endif

/* SDRAM is a special case because it requires non-cached access of its
 * initial configuration, then cached access thereafter.
 */

#define SAM_DDRCS_MMUFLAGS       MMU_MEMFLAGS
#define SAM_DDRAESCS_MMUFLAGS    MMU_MEMFLAGS

/* The external memory regions may support all access if they host SRAM,
 * PSRAM, or SDRAM.  NAND memory requires write access for NAND control and
 * so should be uncached.
 */

#if defined(CONFIG_SAMA5_EBICS0_SRAM) || defined(CONFIG_SAMA5_EBICS0_PSRAM) || \
    defined(CONFIG_SAMA5_EBICS0_NAND)
#  define SAM_EBICS0_MMUFLAGS    MMU_MEMFLAGS
#elif defined(CONFIG_SAMA5_EBICS0_NAND)
#  define SAM_EBICS0_MMUFLAGS    MMU_IOFLAGS
#else
#  define SAM_EBICS0_MMUFLAGS    MMU_ROMFLAGS
#endif

#if defined(CONFIG_SAMA5_EBICS1_SRAM) || defined(CONFIG_SAMA5_EBICS1_PSRAM)
#  define SAM_EBICS1_MMUFLAGS    MMU_MEMFLAGS
#elif defined(CONFIG_SAMA5_EBICS1_NAND)
#  define SAM_EBICS2_MMUFLAGS    MMU_IOFLAGS
#else
#  define SAM_EBICS1_MMUFLAGS    MMU_ROMFLAGS
#endif

#if defined(CONFIG_SAMA5_EBICS2_SRAM) || defined(CONFIG_SAMA5_EBICS2_PSRAM)
#  define SAM_EBICS2_MMUFLAGS    MMU_MEMFLAGS
#elif defined(CONFIG_SAMA5_EBICS2_NAND)
#  define SAM_EBICS2_MMUFLAGS    MMU_IOFLAGS
#else
#  define SAM_EBICS2_MMUFLAGS    MMU_ROMFLAGS
#endif

#if defined(CONFIG_SAMA5_EBICS3_SRAM) || defined(CONFIG_SAMA5_EBICS3_PSRAM)
#  define SAM_EBICS3_MMUFLAGS    MMU_MEMFLAGS
#elif defined(CONFIG_SAMA5_EBICS3_NAND)
#  define SAM_EBICS3_MMUFLAGS    MMU_IOFLAGS
#else
#  define SAM_EBICS3_MMUFLAGS    MMU_ROMFLAGS
#endif

#define SAM_QSPI0AES_MMUFLAGS    MMU_IOFLAGS
#define SAM_QSPI1AES_MMUFLAGS    MMU_IOFLAGS
#define SAM_SDMMC0_MMUFLAGS      MMU_IOFLAGS
#define SAM_SDMMC1_MMUFLAGS      MMU_IOFLAGS
#define SAM_QSPI0_MMUFLAGS       MMU_IOFLAGS
#define SAM_QSPI1_MMUFLAGS       MMU_IOFLAGS
#define SAM_NFCCR_MMUFLAGS       MMU_IOFLAGS

#define SAM_PERIPHA_MMUFLAGS     MMU_IOFLAGS
#define SAM_PERIPHB_MMUFLAGS     MMU_IOFLAGS
#define SAM_PERIPHC_MMUFLAGS     MMU_IOFLAGS

/* SAMA5 Virtual (mapped) Memory Map
 *
 * board_memorymap.h contains special mappings that are needed when a ROM
 * memory map is used.  It is included in this odd location because it
 * depends on some the virtual address definitions provided above.
 */

#include <arch/board/board_memorymap.h>

/* SAMA5 Virtual (mapped) Memory Map.  These are the mappings that will
 * be created if the page table lies in RAM.  If the platform has another,
 * read-only, pre-initialized page table (perhaps in ROM), then the
 * board.h file must provide these definitions.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE

/* Notice that these mappings are a simple 1-to-1 mapping *unless*
 * CONFIG_ARCH_LOWVECTORS is not defined.  In the high vector case, the
 * register system controls register area is moved out 0f 0xffff:000 where
 * the high vectors must reside.
 */

#define SAM_INTMEM_VSECTION      0x00000000 /* 0x00000000-0x0fffffff: Internal Memories */
#  define SAM_ROM_VSECTION       0x00000000 /* 0x00000000-0x0003ffff: ROM */
#  define SAM_SRAMREMAP_VSECTION 0x00000000 /* 0x00000000-0x0003ffff: Remapped ISRAM */
#  define SAM_ECCROM_VSECTION    0x00040000 /* 0x00040000-0x000fffff: EEC ROM */
#  define SAM_NFCSRAM_VSECTION   0x00100000 /* 0x00100000-0x001fffff: NFC SRAM */
#  define SAM_ISRAM_VSECTION     0x00200000 /* 0x00200000-0x002fffff: SRAM */
#    define SAM_ISRAM0_VADDR     0x00200000 /* 0x00200000-0x0021ffff: SRAM0 */
#    define SAM_ISRAM1_VADDR     0x00220000 /* 0x00220000-0x002fffff: SRAM1 */
#  define SAM_UDPHSRAM_VSECTION  0x00300000 /* 0x00300000-0x004fffff: UDPH SRAM */
#  define SAM_UHPOHCI_VSECTION   0x00400000 /* 0x00400000-0x005fffff: UHP OHCI */
#  define SAM_UHPEHCI_VSECTION   0x00500000 /* 0x00500000-0x005fffff: UHP EHCI */
#  define SAM_AXIMX_VSECTION     0x00600000 /* 0x00600000-0x006fffff: AXI Matrix */
#  define SAM_DAP_VSECTION       0x00700000 /* 0x00700000-0x007fffff: DAP */
#  define SAM_L2CC_VSECTION      0x00a00000 /* 0x00a00000-0x00bfffff: L2CC */
#define SAM_EBICS0_VSECTION      0x10000000 /* 0x10000000-0x1fffffff: EBI Chip select 0 */
#define SAM_DDRCS_VSECTION       0x20000000 /* 0x20000000-0x3fffffff: EBI DDR Chip Select */
#define SAM_DDRAESCS_VSECTION    0x40000000 /* 0x40000000-0x5fffffff: EBI DDR AES Chip Select */
#define SAM_EBICS1_VSECTION      0x60000000 /* 0x60000000-0x6fffffff: EBI Chip Select 1 */
#define SAM_EBICS2_VSECTION      0x70000000 /* 0x70000000-0x7fffffff: EBI Chip Select 2 */
#define SAM_EBICS3_VSECTION      0x80000000 /* 0x80000000-0x8fffffff: EBI Chip Select 3 */
#define SAM_QSPI0AES_VSECTION    0x90000000 /* 0x90000000-0x97ffffff: QSPI0 AES Memory */
#define SAM_QSPI1AES_VSECTION    0x98000000 /* 0x98000000-0x9fffffff: QSPI1 AES Memory */
#define SAM_SDMMC0_VSECTION      0xa0000000 /* 0xa0000000-0xafffffff: SDMMC0 */
#define SAM_SDMMC1_VSECTION      0xb0000000 /* 0xb0000000-0xbfffffff: SDMMC1 */
#define SAM_NFCCR_VSECTION       0xc0000000 /* 0xc0000000-0xcfffffff: NFC Command Registers */
#define SAM_QSPI0_VSECTION       0xd0000000 /* 0xd0000000-0xd7ffffff: QSPI0 Memory */
#define SAM_QSPI1_VSECTION       0xd8000000 /* 0xd8000000-0xdfffffff: QSPI1 Memory */
                                            /* 0x80000000-0xefffffff: Undefined */

/* If CONFIG_ARCH_LOWVECTORS is not defined, then move the system control
 * registers out of the way.
 */

#ifdef CONFIG_ARCH_LOWVECTORS
#define SAM_PERIPH_VSECTION      0xf0000000 /* 0xf0000000-0xffffffff: Internal Peripherals */
#  define SAM_PERIPHA_VSECTION   0xf0000000 /* 0xf0000000-0xf7ffffff: Internal Peripherals A */
#  define SAM_PERIPHB_VSECTION   0xf8000000 /* 0xf8000000-0xfbffffff: Internal Peripherals B */
#  define SAM_PERIPHC_VSECTION   0xfc000000 /* 0xfc000000-0xffffffff: Internal Peripherals C */
#  define SAM_SYSC_VSECTION      0xfff00000 /* 0xfff00000-0xffffbfff: System Controller */
#  define SAM_SYSC_VADDR         0xffffc000 /* 0xffffc000-0xffffffff: System Controller */
#else
#define SAM_PERIPH_VSECTION      0xf0000000 /* 0xf0000000-0xffffffff: Internal Peripherals */
#  define SAM_PERIPHA_VSECTION   0xf0000000 /* 0xf0000000-0xf00fffff: Internal Peripherals A */
#  define SAM_PERIPHB_VSECTION   0xf1000000 /* 0xf1000000-0xf10fffff: Internal Peripherals B */
#  define SAM_PERIPHC_VSECTION   0xf2000000 /* 0xf2000000-0xf20fffff: Internal Peripherals C */
#  define SAM_SYSC_VSECTION      0xf2000000 /* 0xf2000000-0xf20fffff: System Controller */
#  define SAM_SYSC_VADDR         0xf20fc000 /* 0xf20fc000-0xf20fffff: System Controller */
#endif
#endif

/* Peripheral virtual base addresses */

#define SAM_SDMMC0_VBASE         (SAM_SDMMC0_VSECTION)
#define SAM_SDMMC1_VBASE         (SAM_SDMMC1_VSECTION)
#define SAM_LCDC_VBASE           (SAM_PERIPHA_VSECTION+SAM_LCDC_OFFSET)
#define SAM_XDMAC1_VBASE         (SAM_PERIPHA_VSECTION+SAM_XDMAC1_OFFSET)
#define SAM_ISC_VBASE            (SAM_PERIPHA_VSECTION+SAM_ISC_OFFSET)
#define SAM_MPDDRC_VBASE         (SAM_PERIPHA_VSECTION+SAM_MPDDRC_OFFSET)
#define SAM_XDMAC0_VBASE         (SAM_PERIPHA_VSECTION+SAM_XDMAC0_OFFSET)
#define SAM_PMC_VBASE            (SAM_PERIPHA_VSECTION+SAM_PMC_OFFSET)
#define SAM_MATRIX64_VBASE       (SAM_PERIPHA_VSECTION+SAM_MATRIX0_OFFSET)
#define SAM_AESB_VBASE           (SAM_PERIPHA_VSECTION+SAM_AESB_OFFSET)
#define SAM_QSPI0_VBASE          (SAM_PERIPHA_VSECTION+SAM_QSPI0_OFFSET)
#define SAM_QSPI1_VBASE          (SAM_PERIPHA_VSECTION+SAM_QSPI1_OFFSET)
#define SAM_SHA_VBASE            (SAM_PERIPHA_VSECTION+SAM_SHA_OFFSET)
#define SAM_AES_VBASE            (SAM_PERIPHA_VSECTION+SAM_AES_OFFSET)

#define SAM_SPI0_VBASE           (SAM_PERIPHB_VSECTION+SAM_SPI0_OFFSET)
#define SAM_SSC0_VBASE           (SAM_PERIPHB_VSECTION+SAM_SSC0_OFFSET)
#define SAM_EMAC0_VBASE          (SAM_PERIPHB_VSECTION+SAM_EMAC0_OFFSET)
#define SAM_TC012_VBASE          (SAM_PERIPHB_VSECTION+SAM_TC012_OFFSET)
#define SAM_TC345_VBASE          (SAM_PERIPHB_VSECTION+SAM_TC345_OFFSET)
#define SAM_HSMC_VBASE           (SAM_PERIPHB_VSECTION+SAM_HSMC_OFFSET)
#define SAM_PDMIC_VBASE          (SAM_PERIPHB_VSECTION+SAM_PDMIC_OFFSET)
#define SAM_UART0_VBASE          (SAM_PERIPHB_VSECTION+SAM_UART0_OFFSET)
#define SAM_UART1_VBASE          (SAM_PERIPHB_VSECTION+SAM_UART1_OFFSET)
#define SAM_UART2_VBASE          (SAM_PERIPHB_VSECTION+SAM_UART2_OFFSET)
#define SAM_TWI0_VBASE           (SAM_PERIPHB_VSECTION+SAM_TWI0_OFFSET)
#define SAM_PWMC_VBASE           (SAM_PERIPHB_VSECTION+SAM_PWMC_OFFSET)
#define SAM_SFR_VBASE            (SAM_PERIPHB_VSECTION+SAM_SFR_OFFSET)
#define SAM_FLEXCOM0_VBASE       (SAM_PERIPHB_VSECTION+SAM_FLEXCOM0_OFFSET)
#define SAM_FLEXCOM1_VBASE       (SAM_PERIPHB_VSECTION+SAM_FLEXCOM1_OFFSET)
#define SAM_SAIC_VBASE           (SAM_PERIPHB_VSECTION+SAM_SAIC_OFFSET)
#define SAM_ICM_VBASE            (SAM_PERIPHB_VSECTION+SAM_ICM_OFFSET)
#define SAM_SECURAM_VBASE        (SAM_PERIPHB_VSECTION+SAM_SECURAM_OFFSET)
#define SAM_SYSC_VBASE           (SAM_PERIPHB_VSECTION+SAM_SYSC_OFFSET)
#define SAM_RSTC_VBASE           (SAM_PERIPHB_VSECTION+SAM_RSTC_OFFSET)
#define SAM_SHDWC_VBASE          (SAM_PERIPHB_VSECTION+SAM_SHDWC_OFFSET)
#define SAM_PITC_VBASE           (SAM_PERIPHB_VSECTION+SAM_PITC_OFFSET)
#define SAM_WDT_VBASE            (SAM_PERIPHB_VSECTION+SAM_WDT_OFFSET)
#define SAM_SCKCR_VBASE          (SAM_PERIPHB_VSECTION+SAM_SCKCR_OFFSET)
#define SAM_RTCC_VBASE           (SAM_PERIPHB_VSECTION+SAM_RTCC_OFFSET)
#define SAM_RXLP_VBASE           (SAM_PERIPHB_VSECTION+SAM_RXLP_OFFSET)
#define SAM_ACC_VBASE            (SAM_PERIPHB_VSECTION+SAM_ACC_OFFSET)
#define SAM_SFC_VBASE            (SAM_PERIPHB_VSECTION+SAM_SFC_OFFSET)
#define SAM_I2SC0_VBASE          (SAM_PERIPHB_VSECTION+SAM_I2SC0_OFFSET)
#define SAM_CAN0_VBASE           (SAM_PERIPHB_VSECTION+SAM_CAN0_OFFSET)

#define SAM_SPI1_VBASE           (SAM_PERIPHC_VSECTION+SAM_SPI1_OFFSET)
#define SAM_SSC1_VBASE           (SAM_PERIPHC_VSECTION+SAM_SSC1_OFFSET)
#define SAM_UART3_VBASE          (SAM_PERIPHC_VSECTION+SAM_UART3_OFFSET)
#define SAM_UART4_VBASE          (SAM_PERIPHC_VSECTION+SAM_UART4_OFFSET)
#define SAM_FLEXCOM2_VBASE       (SAM_PERIPHC_VSECTION+SAM_FLEXCOM2_OFFSET)
#define SAM_FLEXCOM3_VBASE       (SAM_PERIPHC_VSECTION+SAM_FLEXCOM3_OFFSET)
#define SAM_FLEXCOM4_VBASE       (SAM_PERIPHC_VSECTION+SAM_FLEXCOM4_OFFSET)
#define SAM_TRNG_VBASE           (SAM_PERIPHC_VSECTION+SAM_TRNG_OFFSET)
#define SAM_AIC_VBASE            (SAM_PERIPHC_VSECTION+SAM_AIC_OFFSET)
#define SAM_TWI1_VBASE           (SAM_PERIPHC_VSECTION+SAM_TWI1_OFFSET)
#define SAM_UDPHS_VBASE          (SAM_PERIPHC_VSECTION+SAM_UDPHS_OFFSET)
#define SAM_TSADC_VBASE          (SAM_PERIPHC_VSECTION+SAM_ADC_OFFSET)
#define SAM_PIO_VBASE            (SAM_PERIPHC_VSECTION+SAM_PIO_OFFSET)
#define SAM_MATRIX32_VBASE       (SAM_PERIPHC_VSECTION+SAM_MATRIX1_OFFSET)
#define SAM_SECUMOD_VBASE        (SAM_PERIPHC_VSECTION+SAM_SECUMOD_OFFSET)
#define SAM_TDES_VBASE           (SAM_PERIPHC_VSECTION+SAM_TDES_OFFSET)
#define SAM_CLASSD_VBASE         (SAM_PERIPHC_VSECTION+SAM_CLASSD_OFFSET)
#define SAM_I2SC1_VBASE          (SAM_PERIPHC_VSECTION+SAM_I2SC1_OFFSET)
#define SAM_CAN1_VBASE           (SAM_PERIPHC_VSECTION+SAM_CAN1_OFFSET)
#define SAM_UTMI_VBASE           (SAM_PERIPHC_VSECTION+SAM_UTMI_OFFSET)
#define SAM_SFRBU_VBASE          (SAM_PERIPHC_VSECTION+SAM_SFRBU_OFFSET)
#define SAM_CHIPID_VBASE         (SAM_PERIPHC_VSECTION+SAM_CHIPID_OFFSET)

#define SAM_PIOA_VBASE           SAM_PIO_IOGROUPA_VBASE
#define SAM_PIOB_VBASE           SAM_PIO_IOGROUPB_VBASE
#define SAM_PIOC_VBASE           SAM_PIO_IOGROUPC_VBASE
#define SAM_PIOD_VBASE           SAM_PIO_IOGROUPD_VBASE

/* NuttX virtual base address
 *
 * The boot logic will create a temporarily mapping based on where NuttX is
 * executing in memory.  In this case, NuttX could be running from NOR FLASH,
 * SDRAM, external SRAM, or internal SRAM.  If we are running from FLASH,
 * then we must have a separate mapping for the non-contiguous RAM region.
 */

#if defined(CONFIG_BOOT_RUNFROMFLASH)

/* Some sanity checks.  If we are running from FLASH, then one of the
 * external chip selects must be configured to boot from NOR flash.
 * And, if so, then its size must agree with the configured size.
 */

#  if defined(CONFIG_SAMA5_EBICS0) && defined(CONFIG_SAMA5_EBICS0_NOR) && \
      defined (CONFIG_SAMA5_BOOT_CS0FLASH)

#    if CONFIG_SAMA5_EBICS0_SIZE != CONFIG_FLASH_SIZE
#      error CS0 FLASH size disagreement
#    endif

#    undef CONFIG_SAMA5_BOOT_CS1FLASH
#    undef CONFIG_SAMA5_BOOT_CS2FLASH
#    undef CONFIG_SAMA5_BOOT_CS3FLASH

#  elif defined(CONFIG_SAMA5_EBICS1) && defined(CONFIG_SAMA5_EBICS1_NOR) && \
        defined (CONFIG_SAMA5_BOOT_CS1FLASH)

#    if CONFIG_SAMA5_EBICS1_SIZE != CONFIG_FLASH_SIZE
#      error CS1 FLASH size disagreement
#    endif

#    undef CONFIG_SAMA5_BOOT_CS0FLASH
#    undef CONFIG_SAMA5_BOOT_CS2FLASH
#    undef CONFIG_SAMA5_BOOT_CS3FLASH

#  elif defined(CONFIG_SAMA5_EBICS2) && defined(CONFIG_SAMA5_EBICS2_NOR) && \
        defined (CONFIG_SAMA5_BOOT_CS2FLASH)

#    if CONFIG_SAMA2_EBICS0_SIZE != CONFIG_FLASH_SIZE
#      error CS2 FLASH size disagreement
#    endif

#    undef CONFIG_SAMA5_BOOT_CS0FLASH
#    undef CONFIG_SAMA5_BOOT_CS1FLASH
#    undef CONFIG_SAMA5_BOOT_CS3FLASH

#  elif defined(CONFIG_SAMA5_EBICS3) && defined(CONFIG_SAMA5_EBICS3_NOR) && \
        defined (CONFIG_SAMA5_BOOT_CS3FLASH)

#    if CONFIG_SAMA5_EBICS3_SIZE != CONFIG_FLASH_SIZE
#      error CS3 FLASH size disagreement
#    endif

#    undef CONFIG_SAMA5_BOOT_CS0FLASH
#    undef CONFIG_SAMA5_BOOT_CS1FLASH
#    undef CONFIG_SAMA5_BOOT_CS2FLASH

#  else
#    error CONFIG_BOOT_RUNFROMFLASH=y, but no bootable NOR flash defined

#    undef CONFIG_SAMA5_BOOT_CS0FLASH
#    undef CONFIG_SAMA5_BOOT_CS1FLASH
#    undef CONFIG_SAMA5_BOOT_CS2FLASH
#    undef CONFIG_SAMA5_BOOT_CS3FLASH

#  endif

  /* Set up the NOR FLASH region as the NUTTX .text region */

#  define NUTTX_TEXT_VADDR       (CONFIG_FLASH_VSTART & 0xfff00000)
#  define NUTTX_TEXT_PADDR       (CONFIG_FLASH_START & 0xfff00000)
#  define NUTTX_TEXT_PEND        ((CONFIG_FLASH_END + 0x000fffff) & 0xfff00000)
#  define NUTTX_TEXT_SIZE        (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

  /* In the default configuration, the primary RAM used for .bss and .data
   * is the internal SRAM.
   */

#  define NUTTX_RAM_VADDR        (CONFIG_RAM_VSTART & 0xfff00000)
#  define NUTTX_RAM_PADDR        (CONFIG_RAM_START & 0xfff00000)
#  define NUTTX_RAM_PEND         ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#  define NUTTX_RAM_SIZE         (NUTTX_RAM_PEND - NUTTX_RAM_PADDR)

#else /* CONFIG_BOOT_RUNFROMFLASH */

  /* Otherwise we are running from some kind of RAM (ISRAM or SDRAM).
   * Setup the RAM region as the NUTTX .txt, .bss, and .data region.
   */

#  define NUTTX_TEXT_VADDR       (CONFIG_RAM_VSTART & 0xfff00000)
#  define NUTTX_TEXT_PADDR       (CONFIG_RAM_START & 0xfff00000)
#  define NUTTX_TEXT_PEND        ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#  define NUTTX_TEXT_SIZE        (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

#endif /* CONFIG_BOOT_RUNFROMFLASH */

/* MMU Page Table Location
 *
 * Determine the address of the MMU page table.  Regardless of the memory
 * configuration, we will keep the page table in the SAMA5's internal SRAM.
 * We will always attempt to use the bottom 16KB of internal SRAM for the
 * page table, but there are a few conditions that affect this:
 *
 * 1) If CONFIG_ARCH_ROMPGTABLE, then the page table resides in ROM and we
 *    will not use any page table in RAM.
 * 2) We are executing out of SRAM.  In this case, vectors will reside at
 *    the bottom of SRAM, following by .text, .data, .bss, and heap.  The
 *    page table will be squeezed into the end of internal SRAM in this
 *    case.
 *
 * Or...
 * the user may specify the address of the page table explicitly be defining
 * PGTABLE_BASE_VADDR and PGTABLE_BASE_PADDR in the board.h file.
 */

#undef PGTABLE_IN_HIGHSRAM
#undef PGTABLE_IN_LOWSRAM
#undef ARMV7A_PGTABLE_MAPPING

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
 * We must declare the page table at the bottom or at the top of internal
 * SRAM.  We pick the bottom of internal SRAM *unless* there are vectors
 * in the way at that position.
 */

#  if defined(CONFIG_SAMA5_BOOT_ISRAM) && defined(CONFIG_ARCH_LOWVECTORS)

  /* In this case, page table must lie at the top 16Kb of ISRAM1 (or ISRAM0
   * if ISRAM1 is not available in this architecture)
   *
   * If CONFIG_PAGING is defined, then mmu.h assign the virtual address
   * of the page table.
   */

#    if SAM_ISRAM1_SIZE > 0
#      define PGTABLE_BASE_PADDR (SAM_ISRAM1_PADDR+SAM_ISRAM1_SIZE-PGTABLE_SIZE)
#      ifndef CONFIG_PAGING
#        define PGTABLE_BASE_VADDR (SAM_ISRAM1_VADDR+SAM_ISRAM1_SIZE-PGTABLE_SIZE)
#      endif
#    else
#      define PGTABLE_BASE_PADDR (SAM_ISRAM0_PADDR+SAM_ISRAM0_SIZE-PGTABLE_SIZE)
#      ifndef CONFIG_PAGING
#        define PGTABLE_BASE_VADDR (SAM_ISRAM0_VADDR+SAM_ISRAM0_SIZE-PGTABLE_SIZE)
#      endif
#    endif
#    define PGTABLE_IN_HIGHSRAM   1

  /* If we execute from SRAM but keep data in SDRAM, then we will also have
   * to position the initial, IDLE stack in SRAM.  SDRAM will not be ready
   * soon enough to serve as the stack.
   *
   * In this case, the initial IDLE stack can just follow the vector table,
   * lying between the vector table and the page table.  We don't really
   * know how much memory to set aside for the vector table, but 4KiB should
   * be much more than enough
   */

#    ifdef CONFIG_BOOT_SDRAM_DATA
#      define IDLE_STACK_PBASE    (SAM_ISRAM0_PADDR + 0x0001000)
#      define IDLE_STACK_VBASE    (SAM_ISRAM0_VADDR + 0x0001000)
#    endif

#  else /* CONFIG_SAMA5_BOOT_ISRAM && CONFIG_ARCH_LOWVECTORS */

/* Otherwise, the vectors lie at another location (perhaps in NOR FLASH,
 * perhaps elsewhere in internal SRAM).  The page table will then be
 * positioned at the first 16Kb of ISRAM0.
 */

#    define PGTABLE_BASE_PADDR    SAM_ISRAM0_PADDR
#    ifndef CONFIG_PAGING
#      define PGTABLE_BASE_VADDR  SAM_ISRAM0_VADDR
#    endif
#    define PGTABLE_IN_LOWSRAM    1

#    ifdef CONFIG_BOOT_SDRAM_DATA
#      define IDLE_STACK_PBASE    (PGTABLE_BASE_PADDR + PGTABLE_SIZE)
#      define IDLE_STACK_VBASE    (PGTABLE_BASE_VADDR + PGTABLE_SIZE)
#    endif

#  endif /* CONFIG_SAMA5_BOOT_ISRAM && CONFIG_ARCH_LOWVECTORS */

/* In either case, the page table lies in ISRAM.  If ISRAM is not the
 * primary RAM region, then we will need to set-up a special mapping for
 * the page table at boot time.
 */

#  if defined(CONFIG_BOOT_RUNFROMFLASH)
/* If we are running from FLASH, then the primary memory region is
 * given by NUTTX_RAM_PADDR.
 */

#    if NUTTX_RAM_PADDR != SAM_ISRAM_PSECTION
#      define ARMV7A_PGTABLE_MAPPING 1
#    endif

/* Otherwise, we are running from RAM and that RAM is also the primary
 * RAM.
 */

#  elif !defined(CONFIG_SAMA5_BOOT_ISRAM)
#    define ARMV7A_PGTABLE_MAPPING 1
#  endif

#else /* !PGTABLE_BASE_PADDR || !PGTABLE_BASE_VADDR */

/* Sanity check.. if one is defined, both should be defined */

#  if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)
#    error "One of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is undefined"
#  endif

  /* If we execute from SRAM but keep data in SDRAM, then we will also have
   * to position the initial, IDLE stack in SRAM.  SDRAM will not be ready
   * soon enough to serve as the stack.
   *
   * In this case, the initial IDLE stack can just follow the page table
   * in ISRAM.
   */

#    ifdef CONFIG_BOOT_SDRAM_DATA
#      define IDLE_STACK_PBASE    (SAM_ISRAM0_PADDR + PGTABLE_SIZE)
#      define IDLE_STACK_VBASE    (SAM_ISRAM0_VADDR + PGTABLE_SIZE)
#    endif

#endif /* !PGTABLE_BASE_PADDR || !PGTABLE_BASE_VADDR */

/* Level 2 Page table start addresses.
 *
 * 16Kb of memory is reserved hold the page table for the virtual mappings.
 *  A portion of this table is not accessible in the virtual address space
 * (for normal operation).   There is this large whole in the physcal address
 * space for which there will never be level 1 mappings:
 *
 *   0x80000000-0xefffffff: Undefined (1.75 GB)
 *
 * That is the offset where the main L2 page tables will be positioned.  This
 * corresponds to page table offsets 0x000002000 up to 0x000003c00.  That
 * is 1792 entries, each mapping 4KB of address for a total of 7MB of virtual
 * address space)
 *
 * Up to two L2 page tables may be used:
 *
 * 1) One mapping the vector table.  However, L2 page tables must be aligned
 *    to 1KB address boundaries, so the minimum L2 page table size is then
 *    1KB, mapping up a full megabyte of virtual address space.
 *
 *    This L2 page table is only allocated if CONFIG_ARCH_LOWVECTORS is *not*
 *    defined.  The SAMA5 boot-up logic will map the beginning of the boot
 *    memory to address 0x0000:0000 using both the MMU and the AXI matrix
 *    REMAP register.  So no L2 page table is required.
 *
 * 2) If on-demand paging is supported (CONFIG_PAGING=y), than an additional
 *    L2 page table is needed.  This page table will use the remainder of
 *    the address space.
 */

#ifndef CONFIG_ARCH_LOWVECTORS
  /* Vector L2 page table offset/size */

#  define VECTOR_L2_OFFSET        0x000002000
#  define VECTOR_L2_SIZE          0x000000400

  /* Vector L2 page table base addresses */

#  define VECTOR_L2_PBASE         (PGTABLE_BASE_PADDR+VECTOR_L2_OFFSET)
#  define VECTOR_L2_VBASE         (PGTABLE_BASE_VADDR+VECTOR_L2_OFFSET)

  /* Vector L2 page table end addresses */

#  define VECTOR_L2_END_PADDR     (VECTOR_L2_PBASE+VECTOR_L2_SIZE)
#  define VECTOR_L2_END_VADDR     (VECTOR_L2_VBASE+VECTOR_L2_SIZE)

  /* Paging L2 page table offset/size */

#  define PGTABLE_L2_OFFSET       0x000002400
#  define PGTABLE_L2_SIZE         0x000001800

#else
  /* Paging L2 page table offset/size */

#  define PGTABLE_L2_OFFSET       0x000002000
#  define PGTABLE_L2_SIZE         0x000001c00
#endif

/* Paging L2 page table base addresses
 *
 * NOTE: If CONFIG_PAGING is defined, mmu.h will re-assign the virtual
 * address of the page table.
 */

#define PGTABLE_L2_PBASE          (PGTABLE_BASE_PADDR+PGTABLE_L2_OFFSET)
#define PGTABLE_L2_VBASE          (PGTABLE_BASE_VADDR+PGTABLE_L2_OFFSET)

/* Paging L2 page table end addresses */

#define PGTABLE_L2_END_PADDR      (PGTABLE_L2_PBASE+PGTABLE_L2_SIZE)
#define PGTABLE_L2_END_VADDR      (PGTABLE_L2_VBASE+PGTABLE_L2_SIZE)

/* Base address of the interrupt vector table.
 *
 *   SAM_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
 *   SAM_VECTOR_VSRAM - Virtual address of vector table in SRAM
 *   SAM_VECTOR_VADDR - Virtual address of vector table
 *                      (0x00000000 or 0xffff0000)
 */

#define VECTOR_TABLE_SIZE         0x00010000

#ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */

#  define SAM_VECTOR_PADDR        SAM_ISRAM0_PADDR
#  define SAM_VECTOR_VSRAM        SAM_ISRAM0_VADDR
#  define SAM_VECTOR_VADDR        0x00000000

#else  /* Vectors located at 0xffff:0000 -- this probably does not work */

#  if SAM_ISRAM1_SIZE >= VECTOR_TABLE_SIZE
#    define SAM_VECTOR_PADDR      (SAM_ISRAM1_PADDR+SAM_ISRAM1_SIZE-VECTOR_TABLE_SIZE)
#    define SAM_VECTOR_VSRAM      (SAM_ISRAM1_VADDR+SAM_ISRAM1_SIZE-VECTOR_TABLE_SIZE)
#  else
#    define SAM_VECTOR_PADDR      (SAM_ISRAM0_PADDR+SAM_ISRAM0_SIZE-VECTOR_TABLE_SIZE)
#    define SAM_VECTOR_VSRAM      (SAM_ISRAM0_VADDR+SAM_ISRAM0_SIZE-VECTOR_TABLE_SIZE)
#  endif
#  define SAM_VECTOR_VADDR        0xffff0000

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D2X_MEMORYMAP_H */

