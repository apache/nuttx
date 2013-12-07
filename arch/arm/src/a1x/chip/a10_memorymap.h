/************************************************************************************
 * arch/arm/src/a1x/a10_memorymap.h
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

#ifndef __ARCH_ARM_SRC_A1X_CHIP_A10_MEMORYMAP_H
#define __ARCH_ARM_SRC_A1X_CHIP_A10_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/a1x/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Decimal configuration values may exceed 2Gb and, hence, overflow to negative
 * values unless we force them to unsigned long:
 */

#define __CONCAT(a,b) a ## b
#define MKULONG(a) __CONCAT(a,ul)

/* A1X physical section base addresses (aligned to 1MB boundaries) */

#define A1X_INTMEM_PSECTION  0x00000000 /* Internal memory 0x0000:0000-0x0002:ffff */
#define A1X_PERIPH_PSECTION  0x01c00000 /* Peripherals     0x01c0:0000-0x01c4:ffff */
#define A1X_SRAMC_PSECTION   0x01d00000 /* SRAM C          0x01d0:0000-0x01df:ffff Module sram */
#define A1X_DE_PSECTION      0x01e00000 /* DE, MP, AVG     0x01e0:0000-0x01eb:ffff  */
#define A1X_DDR_PSECTION     0x40000000 /* DDR-II/DDR-III  0x4000:0000-0xbfff:ffff 2G */
#define A1X_BROM_PSECTION    0xfff00000 /* BROM            0xffff:0000—0xffff:7fff 32K */

/* A1X Offsets from the internal memory section base address */

#define A1X_SRAMA1_OFFSET    0x00000000 /* SRAM A1         0x0000:0000-0x0000:3fff 16K */
#define A1X_SRAMA2_OFFSET    0x00004000 /* SRAM A2         0x0000:4000-0x0000:7fff 16K */
#define A1X_SRAMA3_OFFSET    0x00008000 /* SRAM A3         0x0000:8000-0x0000:b3ff 13K */
#define A1X_SRAMA4_OFFSET    0x0000b400 /* SRAM A4         0x0000:b400-0x0000:bfff 3K */
#define A1X_SRAMNAND_OFFSET             /* SRAM Nand 2K */
#define A1X_SRAMD_OFFSET     0x00010000 /* SRAM D          0x0001:0000-0x0001:0fff 4K */
#define A1X_SRAMDSEC_OFFSET  0x00020000 /* SRAM D(Secure)  0x0002:0000-0x0002:ffff 64K */

/* A1X offsets from the peripheral section base address */

#define A1X_SRAMC_OFFSET     0x00000000 /* SRAM Controller 0x01c0:0000-0x01c0:0fff 4K */
#define A1X_DRAMC_OFFSET     0x00001000 /* DRAM Controller 0x01c0:1000-0x01c0:1fff 4K */
#define A1X_DMA_OFFSET       0x00002000 /* DMA             0x01c0:2000-0x01c0:2fff 4K */
#define A1X_NFC_OFFSET       0x00003000 /* NFC             0x01c0:3000-0x01c0:3fff 4K */
#define A1X_TS_OFFSET        0x00004000 /* TS              0x01c0:4000-0x01c0:4fff 4K */
#define A1X_SPI0_OFFSET      0x00005000 /* SPI 0           0x01c0:5000-0x01c0:5fff 4K */
#define A1X_SPI1_OFFSET      0x00006000 /* SPI 1           0x01c0:6000-0x01c0:6fff 4K */
#define A1X_MS_OFFSET        0x00007000 /* MS              0x01c0:7000-0x01c0:7fff 4K */
#define A1X_TVD_OFFSET       0x00008000 /* TVD             0x01c0:8000-0x01c0:8fff 4K */
#define A1X_CSI0_OFFSET      0x00009000 /* CSI 0           0x01c0:9000-0x01c0:9fff 4K */
#define A1X_TVE0_OFFSET      0x0000a000 /* TVE 0           0x01c0:a000-0x01c0:afff 4K */
#define A1X_EMAC_OFFSET      0x0000b000 /* EMAC            0x01c0:b000-0x01c0:bfff 4K */
#define A1X_LCD0_OFFSET      0x0000c000 /* LCD 0           0x01c0:c000-0x01c0:cfff 4K */
#define A1X_LCD1_OFFSET      0x0000d000 /* LCD 1           0x01c0:d000-0x01c0:dfff 4K */
#define A1X_VE_OFFSET        0x0000e000 /* VE              0x01c0:e000-0x01c0:efff 4K */
#define A1X_SDMMC0_OFFSET    0x0000f000 /* SD/MMC 0        0x01c0:f000-0x01c0:ffff 4K */
#define A1X_SDMMC1_OFFSET    0x00010000 /* SD/MMC 1        0x01c1:0000-0x01c1:0fff 4K */
#define A1X_SDMMC2_OFFSET    0x00011000 /* SD/MMC 2        0x01c1:1000-0x01c1:1fff 4K */
#define A1X_SDMMC3_OFFSET    0x00012000 /* SD/MMC 3        0x01c1:2000-0x01c1:2fff 4K */
#define A1X_USB0_OFFSET      0x00013000 /* USB 0           0x01c1:3000-0x01c1:3fff 4K */
#define A1X_USB1_OFFSET      0x00014000 /* USB 1           0x01c1:4000-0x01c1:4fff 4K */
#define A1X_SS_OFFSET        0x00015000 /* SS              0x01c1:5000-0x01c1:5fff 4K */
#define A1X_HDMI_OFFSET      0x00016000 /* HDMI            0x01c1:6000-0x01c1:6fff 4K */
#define A1X_SPI2_OFFSET      0x00017000 /* SPI 2           0x01c1:7000-0x01c1:7fff 4K */
#define A1X_PATA_OFFSET      0x00019000 /* PATA            0x01c1:9000-0x01c1:9fff 4K */
#define A1X_ACE_OFFSET       0x0001a000 /* ACE             0x01c1:A000-0x01c1:afff 4K */
#define A1X_TVE1_OFFSET      0x0001b000 /* TVE 1           0x01c1:B000-0x01c1:bfff 4K */
#define A1X_USB2_OFFSET      0x0001c000 /* USB 2           0x01c1:C000-0x01c1:cfff 4K */
#define A1X_CSI1_OFFSET      0x0001d000 /* CSI 1           0x01c1:D000-0x01c1:dfff 4K */
#define A1X_TZASC_OFFSET     0x0001e000 /* TZASC           0x01c1:E000-0x01c1:efff 4K */
#define A1X_SPI3_OFFSET      0x0001f000 /* SPI3            0x01c1:F000-0x01c1:ffff 4K */
#define A1X_CCM_OFFSET       0x00020000 /* CCM             0x01c2:0000-0x01c2:03ff 1K */
#define A1X_INTC_OFFSET      0x00020400 /* INTC            0x01c2:0400-0x01c2:07ff 1K */
#define A1X_PIO_OFFSET       0x00020800 /* PIO             0x01c2:0800-0x01c2:0bff 1K */
#define A1X_TIMER_OFFSET     0x00020c00 /* Timer           0x01c2:0C00-0x01c2:0fff 1K */
#define A1X_AC97_OFFSET      0x00021400 /* AC97            0x01c2:1400-0x01c2:17ff 1K */
#define A1X_IR0_OFFSET       0x00021800 /* IR 0            0x01c2:1800-0x01c2:1bff 1K */
#define A1X_IR1_OFFSET       0x00021c00 /* IR 1            0x01c2:1c00-0x01c2:1fff 1K */
#define A1X_IIS_OFFSET       0x00022400 /* IIS             0x01c2:2400-0x01c2:27ff 1K */
#define A1X_LRADC01_OFFSET   0x00022800 /* LRADC 0/1       0x01c2:2800-0x01c2:2bff 1K */
#define A1X_ADDA_OFFSET      0x00022c00 /* AD/DA           0x01c2:2c00-0x01c2:2fff 1K */
#define A1X_KEYPAD_OFFSET    0x00023000 /* KEYPAD          0x01c2:3000-0x01c2:33ff 1K */
#define A1X_TZPC_OFFSET      0x00023400 /* TZPC            0x01c2:3400-0x01c2:37ff 1K */
#define A1X_SID_OFFSET       0x00023800 /* SID             0x01c2:3800-0x01c2:3bff 1K */
#define A1X_SJTAG_OFFSET     0x00023c00 /* SJTAG           0x01c2:3c00-0x01c2:3fff 1K */
#define A1X_TP_OFFSET        0x00025000 /* TP              0x01c2:5000-0x01c2:53ff 1K */
#define A1X_PMU_OFFSET       0x00025400 /* PMU             0x01c2:5400-0x01c2:57ff 1K */
#define A1X_UART_OFFSET(n)   (0x00028000 + ((uint32_t)(n) << 10))
#define A1X_UART0_OFFSET     0x00028000 /* UART 0          0x01c2:8000-0x01c2:83ff 1K */
#define A1X_UART1_OFFSET     0x00028400 /* UART 1          0x01c2:8400-0x01c2:87ff 1K */
#define A1X_UART2_OFFSET     0x00028800 /* UART 2          0x01c2:8800-0x01c2:8bff 1K */
#define A1X_UART3_OFFSET     0x00028c00 /* UART 3          0x01c2:8C00-0x01c2:8fff 1K */
#define A1X_UART4_OFFSET     0x00029000 /* UART 4          0x01c2:9000-0x01c2:93ff 1K */
#define A1X_UART5_OFFSET     0x00029400 /* UART 5          0x01c2:9400-0x01c2:97ff 1K */
#define A1X_UART6_OFFSET     0x00029800 /* UART 6          0x01c2:9800-0x01c2:9bff 1K */
#define A1X_UART7_OFFSET     0x00029c00 /* UART 7          0x01c2:9c00-0x01c2:9fff 1K */
#define A1X_PS20_OFFSET      0x0002a000 /* PS2-0           0x01c2:a000-0x01c2:a3ff 1K */
#define A1X_PS21_OFFSET      0x0002a400 /* PS2-1           0x01c2:a400-0x01c2:a7ff 1K */
#define A1X_TWI0_OFFSET      0x0002ac00 /* TWI 0           0x01c2:ac00-0x01c2:afff 1K */
#define A1X_TWI1_OFFSET      0x0002b000 /* TWI 1           0x01c2:b000-0x01c2:B3ff 1K */
#define A1X_TWI2_OFFSET      0x0002b400 /* TWI 2           0x01c2:b400-0x01c2:b7ff 1K */
#define A1X_CAN_OFFSET       0x0002bc00 /* CAN             0x01c2:bc00-0x01c2:bfff 1K */
#define A1X_SCR_OFFSET       0x0002c400 /* SCR             0x01c2:c400-0x01c2:c7ff 1K */
#define A1X_MALI400_OFFSET   0x00040000 /* Mali400         0x01c4:0000-0x01c4:ffff 64K */

/* A1X offsets from the DE section base address */

#define A1X_DEFE0_OFFSET     0x00000000 /* DE_FE0          0x01e0:0000-0x01e1:ffff 128K */
#define A1X_DEFE1_OFFSET     0x00020000 /* DE_FE1          0x01e2:0000-0x01e3:ffff 128K */
#define A1X_DEBE0_OFFSET     0x00060000 /* DE_BE0          0x01e6:0000-0x01e7:ffff 128K */
#define A1X_DEBE1_OFFSET     0x00040000 /* DE_BE1          0x01e4:0000-0x01e5:ffff 128K */
#define A1X_MP_OFFSET        0x00080000 /* MP              0x01e8:0000-0x01e9:ffff 128K */
#define A1X_AVG_OFFSET       0x000a0000 /* AVG             0x01ea:0000-0x01eb:ffff 128K */

/* A1X offsets from the BRROM section base address */

#define A1X_BROM_OFFSET      0x000f0000 /* BROM            0xffff:0000—0xffff:7fff 32K */

/* A1X internal memory physical base addresses */

#define A1X_SRAMA1_PADDR     (A1X_INTMEM_PSECTION+A1X_SRAMA1_OFFSET)
#define A1X_SRAMA2_PADDR     (A1X_INTMEM_PSECTION+A1X_SRAMA2_OFFSET)
#define A1X_SRAMA3_PADDR     (A1X_INTMEM_PSECTION+A1X_SRAMA3_OFFSET)
#define A1X_SRAMA4_PADDR     (A1X_INTMEM_PSECTION+A1X_SRAMA4_OFFSET)
#define A1X_SRAMNAND_PADDR
#define A1X_SRAMD_PADDR      (A1X_INTMEM_PSECTION+A1X_SRAMD_OFFSET)
#define A1X_SRAMDSEC_PADDR   (A1X_INTMEM_PSECTION+A1X_SRAMDSEC_OFFSET)

/* Peripheral physical base addresses */

#define A1X_SRAMC_PADDR      (A1X_PERIPH_PSECTION+A1X_SRAMC_OFFSET)
#define A1X_DRAMC_PADDR      (A1X_PERIPH_PSECTION+A1X_DRAMC_OFFSET)
#define A1X_DMA_PADDR        (A1X_PERIPH_PSECTION+A1X_DMA_OFFSET)
#define A1X_NFC_PADDR        (A1X_PERIPH_PSECTION+A1X_NFC_OFFSET)
#define A1X_TS_PADDR         (A1X_PERIPH_PSECTION+A1X_TS_OFFSET)
#define A1X_SPI0_PADDR       (A1X_PERIPH_PSECTION+A1X_SPI0_OFFSET)
#define A1X_SPI1_PADDR       (A1X_PERIPH_PSECTION+A1X_SPI1_OFFSET)
#define A1X_MS_PADDR         (A1X_PERIPH_PSECTION+A1X_MS_OFFSET)
#define A1X_TVD_PADDR        (A1X_PERIPH_PSECTION+A1X_TVD_OFFSET)
#define A1X_CSI0_PADDR       (A1X_PERIPH_PSECTION+A1X_CSI0_OFFSET)
#define A1X_TVE0_PADDR       (A1X_PERIPH_PSECTION+A1X_TVE0_OFFSET)
#define A1X_EMAC_PADDR       (A1X_PERIPH_PSECTION+A1X_EMAC_OFFSET)
#define A1X_LCD0_PADDR       (A1X_PERIPH_PSECTION+A1X_LCD0_OFFSET)
#define A1X_LCD1_PADDR       (A1X_PERIPH_PSECTION+A1X_LCD1_OFFSET)
#define A1X_VE_PADDR         (A1X_PERIPH_PSECTION+A1X_VE_OFFSET)
#define A1X_SDMMC0_PADDR     (A1X_PERIPH_PSECTION+A1X_SDMMC0_OFFSET)
#define A1X_SDMMC1_PADDR     (A1X_PERIPH_PSECTION+A1X_SDMMC1_OFFSET)
#define A1X_SDMMC2_PADDR     (A1X_PERIPH_PSECTION+A1X_SDMMC2_OFFSET)
#define A1X_SDMMC3_PADDR     (A1X_PERIPH_PSECTION+A1X_SDMMC3_OFFSET)
#define A1X_USB0_PADDR       (A1X_PERIPH_PSECTION+A1X_USB0_OFFSET)
#define A1X_USB1_PADDR       (A1X_PERIPH_PSECTION+A1X_USB1_OFFSET)
#define A1X_SS_PADDR         (A1X_PERIPH_PSECTION+A1X_SS_OFFSET)
#define A1X_HDMI_PADDR       (A1X_PERIPH_PSECTION+A1X_HDMI_OFFSET)
#define A1X_SPI2_PADDR       (A1X_PERIPH_PSECTION+A1X_SPI2_OFFSET)
#define A1X_PATA_PADDR       (A1X_PERIPH_PSECTION+A1X_PATA_OFFSET)
#define A1X_ACE_PADDR        (A1X_PERIPH_PSECTION+A1X_ACE_OFFSET)
#define A1X_TVE1_PADDR       (A1X_PERIPH_PSECTION+A1X_TVE1_OFFSET)
#define A1X_USB2_PADDR       (A1X_PERIPH_PSECTION+A1X_USB2_OFFSET)
#define A1X_CSI1_PADDR       (A1X_PERIPH_PSECTION+A1X_CSI1_OFFSET)
#define A1X_TZASC_PADDR      (A1X_PERIPH_PSECTION+A1X_TZASC_OFFSET)
#define A1X_SPI3_PADDR       (A1X_PERIPH_PSECTION+A1X_SPI3_OFFSET)
#define A1X_CCM_PADDR        (A1X_PERIPH_PSECTION+A1X_CCM_OFFSET)
#define A1X_INTC_PADDR       (A1X_PERIPH_PSECTION+A1X_INTC_OFFSET)
#define A1X_PIO_PADDR        (A1X_PERIPH_PSECTION+A1X_PIO_OFFSET)
#define A1X_TIMER_PADDR      (A1X_PERIPH_PSECTION+A1X_TIMER_OFFSET)
#define A1X_AC97_PADDR       (A1X_PERIPH_PSECTION+A1X_AC97_OFFSET)
#define A1X_IR0_PADDR        (A1X_PERIPH_PSECTION+A1X_IR0_OFFSET)
#define A1X_IR1_PADDR        (A1X_PERIPH_PSECTION+A1X_IR1_OFFSET)
#define A1X_IIS_PADDR        (A1X_PERIPH_PSECTION+A1X_IIS_OFFSET)
#define A1X_LRADC01_PADDR    (A1X_PERIPH_PSECTION+A1X_LRADC01_OFFSET)
#define A1X_ADDA_PADDR       (A1X_PERIPH_PSECTION+A1X_ADDA_OFFSET)
#define A1X_KEYPAD_PADDR     (A1X_PERIPH_PSECTION+A1X_KEYPAD_OFFSET)
#define A1X_TZPC_PADDR       (A1X_PERIPH_PSECTION+A1X_TZPC_OFFSET)
#define A1X_SID_PADDR        (A1X_PERIPH_PSECTION+A1X_SID_OFFSET)
#define A1X_SJTAG_PADDR      (A1X_PERIPH_PSECTION+A1X_SJTAG_OFFSET)
#define A1X_TP_PADDR         (A1X_PERIPH_PSECTION+A1X_TP_OFFSET)
#define A1X_PMU_PADDR        (A1X_PERIPH_PSECTION+A1X_PMU_OFFSET)
#define A1X_UART_PADDR(n)    (A1X_PERIPH_PSECTION+A1X_UART_OFFSET(n))
#define A1X_UART0_PADDR      (A1X_PERIPH_PSECTION+A1X_UART0_OFFSET)
#define A1X_UART1_PADDR      (A1X_PERIPH_PSECTION+A1X_UART1_OFFSET)
#define A1X_UART2_PADDR      (A1X_PERIPH_PSECTION+A1X_UART2_OFFSET)
#define A1X_UART3_PADDR      (A1X_PERIPH_PSECTION+A1X_UART3_OFFSET)
#define A1X_UART4_PADDR      (A1X_PERIPH_PSECTION+A1X_UART4_OFFSET)
#define A1X_UART5_PADDR      (A1X_PERIPH_PSECTION+A1X_UART5_OFFSET)
#define A1X_UART6_PADDR      (A1X_PERIPH_PSECTION+A1X_UART6_OFFSET)
#define A1X_UART7_PADDR      (A1X_PERIPH_PSECTION+A1X_UART7_OFFSET)
#define A1X_PS20_PADDR       (A1X_PERIPH_PSECTION+A1X_PS20_OFFSET)
#define A1X_PS21_PADDR       (A1X_PERIPH_PSECTION+A1X_PS21_OFFSET)
#define A1X_TWI0_PADDR       (A1X_PERIPH_PSECTION+A1X_TWI0_OFFSET)
#define A1X_TWI1_PADDR       (A1X_PERIPH_PSECTION+A1X_TWI1_OFFSET)
#define A1X_TWI2_PADDR       (A1X_PERIPH_PSECTION+A1X_TWI2_OFFSET)
#define A1X_CAN_PADDR        (A1X_PERIPH_PSECTION+A1X_CAN_OFFSET)
#define A1X_SCR_PADDR        (A1X_PERIPH_PSECTION+A1X_SCR_OFFSET)
#define A1X_MALI400_PADDR    (A1X_PERIPH_PSECTION+A1X_MALI400_OFFSET)

/* A1X DE section physical base addresses */

#define A1X_DEFE0_PADDR      (A1X_DE_PSECTION+A1X_DEFE0_OFFSET)
#define A1X_DEFE1_PADDR      (A1X_DE_PSECTION+A1X_DEFE1_OFFSET)
#define A1X_DEBE0_PADDR      (A1X_DE_PSECTION+A1X_DEBE0_OFFSET)
#define A1X_DEBE1_PADDR      (A1X_DE_PSECTION+A1X_DEBE1_OFFSET)
#define A1X_MP_PADDR         (A1X_DE_PSECTION+A1X_MP_OFFSET)
#define A1X_AVG_PADDR        (A1X_DE_PSECTION+A1X_AVG_OFFSET)

/* A1X BRROM section physical base address */

#define A1X_BROM_PADDR       (A1X_BROM_PSECTION+A1X_BROM_OFFSET)

/* Sizes of memory regions in bytes.
 *
 * These sizes exclude the undefined addresses at the end of the memory
 * region.  The implemented sizes of the external memory regions are
 * not known apriori and must be specified with configuration settings.
 */

#define A1X_INTMEM_SIZE      0x00030000 /* Internal memory 0x0000:0000-0x0002:ffff */
#define A1X_PERIPH_SIZE      0x00050000 /* Peripherals     0x01c0:0000-0x01c4:ffff */
#define A1X_SRAMC_SIZE       0x00100000 /* SRAM C          0x01d0:0000-0x01df:ffff Module sram */
#define A1X_DE_SIZE          0x000c0000 /* DE, MP, AVG     0x01e0:0000-0x01eb:ffff  */
#define A1X_BROM_SIZE        0x000f8000 /* BROM            0xfff0:0000—0xffff:7fff 32K */

/* Force configured sizes that might exceed 2GB to be unsigned long */

#define A1X_DDR_MAPOFFSET    MKULONG(CONFIG_A1X_DDR_MAPOFFSET)
#define A1X_DDR_MAPSIZE      MKULONG(CONFIG_A1X_DDR_MAPSIZE)
#define A1X_DDR_HEAP_OFFSET  MKULONG(CONFIG_A1X_DDR_HEAP_OFFSET)
#define A1X_DDR_HEAP_SIZE    MKULONG(CONFIG_A1X_DDR_HEAP_SIZE)

/* Convert size in bytes to number of sections (in Mb). */

#define _NSECTIONS(b)        (((b)+0x000fffff) >> 20)

/* Sizes of memory regions in sections.
 *
 * The boot logic in A1X_boot.c, will select 1Mb level 1 MMU mappings to
 * span the entire physical address space.  The definitions below specify
 * the number of 1Mb entries that are required to span a particular address
 * region.
 *
 * NOTE: the size of the mapped SDRAM region depends on the configured size
 * of DRAM, not on the size of the address space assigned to DRAM.
 */

#define A1X_INTMEM_NSECTIONS _NSECTIONS(A1X_INTMEM_SIZE)
#define A1X_PERIPH_NSECTIONS _NSECTIONS(A1X_PERIPH_SIZE)
#define A1X_SRAMC_NSECTIONS  _NSECTIONS(A1X_SRAMC_SIZE)
#define A1X_DE_NSECTIONS     _NSECTIONS(A1X_DE_SIZE)
#define A1X_DDR_NSECTIONS    _NSECTIONS(A1X_DDR_MAPSIZE)
#define A1X_BROM_NSECTIONS   _NSECTIONS(A1X_BROM_SIZE)

/* Section MMU Flags */

#define A1X_INTMEM_MMUFLAGS  MMU_MEMFLAGS
#define A1X_PERIPH_MMUFLAGS  MMU_IOFLAGS
#define A1X_SRAMC_MMUFLAGS   MMU_MEMFLAGS
#define A1X_DE_MMUFLAGS      MMU_IOFLAGS
#define A1X_DDR_MMUFLAGS     MMU_MEMFLAGS
#define A1X_BROM_MMUFLAGS    MMU_ROMFLAGS

/* A1X Virtual (mapped) Memory Map
 *
 * board_memorymap.h contains special mappings that are needed when a ROM
 * memory map is used.  It is included in this odd location becaue it depends
 * on some the virtual address definitions provided above.
 */

#include <arch/board/board_memorymap.h>

/* A1X Virtual (mapped) Memory Map.  These are the mappings that will
 * be created if the page table lies in RAM.  If the platform has another,
 * read-only, pre-initialized page table (perhaps in ROM), then the board.h
 * file must provide these definitions.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE

/* Notice that these mappings are a simple 1-to-1 mappings */

#define A1X_INTMEM_VSECTION  0x00000000 /* Internal memory 0x0000:0000-0x0002:ffff */
#define A1X_PERIPH_VSECTION  0x01c00000 /* Peripherals     0x01c0:0000-0x01c4:ffff */
#define A1X_SRAMC_VSECTION   0x01d00000 /* SRAM C          0x01d0:0000-0x01df:ffff Module sram */
#define A1X_DE_VSECTION      0x01e00000 /* DE, MP, AVG     0x01e0:0000-0x01eb:ffff  */
#define A1X_DDR_VSECTION     0x40000000 /* DDR-II/DDR-III  0x4000:0000-0xbfff:ffff 2G */
#define A1X_BROM_VSECTION    0xfff00000 /* BROM            0xffff:0000—0xffff:7fff 32K */

#endif

/* A1X internal memory virtual base addresses */

#define A1X_SRAMA1_VADDR     (A1X_INTMEM_VSECTION+A1X_SRAMA1_OFFSET)
#define A1X_SRAMA2_VADDR     (A1X_INTMEM_VSECTION+A1X_SRAMA2_OFFSET)
#define A1X_SRAMA3_VADDR     (A1X_INTMEM_VSECTION+A1X_SRAMA3_OFFSET)
#define A1X_SRAMA4_VADDR     (A1X_INTMEM_VSECTION+A1X_SRAMA4_OFFSET)
#define A1X_SRAMNAND_VADDR
#define A1X_SRAMD_VADDR      (A1X_INTMEM_VSECTION+A1X_SRAMD_OFFSET)
#define A1X_SRAMDSEC_VADDR   (A1X_INTMEM_VSECTION+A1X_SRAMDSEC_OFFSET)

/* Peripheral virtual base addresses */

#define A1X_SRAMC_VADDR      (A1X_PERIPH_VSECTION+A1X_SRAMC_OFFSET)
#define A1X_DRAMC_VADDR      (A1X_PERIPH_VSECTION+A1X_DRAMC_OFFSET)
#define A1X_DMA_VADDR        (A1X_PERIPH_VSECTION+A1X_DMA_OFFSET)
#define A1X_NFC_VADDR        (A1X_PERIPH_VSECTION+A1X_NFC_OFFSET)
#define A1X_TS_VADDR         (A1X_PERIPH_VSECTION+A1X_TS_OFFSET)
#define A1X_SPI0_VADDR       (A1X_PERIPH_VSECTION+A1X_SPI0_OFFSET)
#define A1X_SPI1_VADDR       (A1X_PERIPH_VSECTION+A1X_SPI1_OFFSET)
#define A1X_MS_VADDR         (A1X_PERIPH_VSECTION+A1X_MS_OFFSET)
#define A1X_TVD_VADDR        (A1X_PERIPH_VSECTION+A1X_TVD_OFFSET)
#define A1X_CSI0_VADDR       (A1X_PERIPH_VSECTION+A1X_CSI0_OFFSET)
#define A1X_TVE0_VADDR       (A1X_PERIPH_VSECTION+A1X_TVE0_OFFSET)
#define A1X_EMAC_VADDR       (A1X_PERIPH_VSECTION+A1X_EMAC_OFFSET)
#define A1X_LCD0_VADDR       (A1X_PERIPH_VSECTION+A1X_LCD0_OFFSET)
#define A1X_LCD1_VADDR       (A1X_PERIPH_VSECTION+A1X_LCD1_OFFSET)
#define A1X_VE_VADDR         (A1X_PERIPH_VSECTION+A1X_VE_OFFSET)
#define A1X_SDMMC0_VADDR     (A1X_PERIPH_VSECTION+A1X_SDMMC0_OFFSET)
#define A1X_SDMMC1_VADDR     (A1X_PERIPH_VSECTION+A1X_SDMMC1_OFFSET)
#define A1X_SDMMC2_VADDR     (A1X_PERIPH_VSECTION+A1X_SDMMC2_OFFSET)
#define A1X_SDMMC3_VADDR     (A1X_PERIPH_VSECTION+A1X_SDMMC3_OFFSET)
#define A1X_USB0_VADDR       (A1X_PERIPH_VSECTION+A1X_USB0_OFFSET)
#define A1X_USB1_VADDR       (A1X_PERIPH_VSECTION+A1X_USB1_OFFSET)
#define A1X_SS_VADDR         (A1X_PERIPH_VSECTION+A1X_SS_OFFSET)
#define A1X_HDMI_VADDR       (A1X_PERIPH_VSECTION+A1X_HDMI_OFFSET)
#define A1X_SPI2_VADDR       (A1X_PERIPH_VSECTION+A1X_SPI2_OFFSET)
#define A1X_PATA_VADDR       (A1X_PERIPH_VSECTION+A1X_PATA_OFFSET)
#define A1X_ACE_VADDR        (A1X_PERIPH_VSECTION+A1X_ACE_OFFSET)
#define A1X_TVE1_VADDR       (A1X_PERIPH_VSECTION+A1X_TVE1_OFFSET)
#define A1X_USB2_VADDR       (A1X_PERIPH_VSECTION+A1X_USB2_OFFSET)
#define A1X_CSI1_VADDR       (A1X_PERIPH_VSECTION+A1X_CSI1_OFFSET)
#define A1X_TZASC_VADDR      (A1X_PERIPH_VSECTION+A1X_TZASC_OFFSET)
#define A1X_SPI3_VADDR       (A1X_PERIPH_VSECTION+A1X_SPI3_OFFSET)
#define A1X_CCM_VADDR        (A1X_PERIPH_VSECTION+A1X_CCM_OFFSET)
#define A1X_INTC_VADDR       (A1X_PERIPH_VSECTION+A1X_INTC_OFFSET)
#define A1X_PIO_VADDR        (A1X_PERIPH_VSECTION+A1X_PIO_OFFSET)
#define A1X_TIMER_VADDR      (A1X_PERIPH_VSECTION+A1X_TIMER_OFFSET)
#define A1X_AC97_VADDR       (A1X_PERIPH_VSECTION+A1X_AC97_OFFSET)
#define A1X_IR0_VADDR        (A1X_PERIPH_VSECTION+A1X_IR0_OFFSET)
#define A1X_IR1_VADDR        (A1X_PERIPH_VSECTION+A1X_IR1_OFFSET)
#define A1X_IIS_VADDR        (A1X_PERIPH_VSECTION+A1X_IIS_OFFSET)
#define A1X_LRADC01_VADDR    (A1X_PERIPH_VSECTION+A1X_LRADC01_OFFSET)
#define A1X_ADDA_VADDR       (A1X_PERIPH_VSECTION+A1X_ADDA_OFFSET)
#define A1X_KEYPAD_VADDR     (A1X_PERIPH_VSECTION+A1X_KEYPAD_OFFSET)
#define A1X_TZPC_VADDR       (A1X_PERIPH_VSECTION+A1X_TZPC_OFFSET)
#define A1X_SID_VADDR        (A1X_PERIPH_VSECTION+A1X_SID_OFFSET)
#define A1X_SJTAG_VADDR      (A1X_PERIPH_VSECTION+A1X_SJTAG_OFFSET)
#define A1X_TP_VADDR         (A1X_PERIPH_VSECTION+A1X_TP_OFFSET)
#define A1X_PMU_VADDR        (A1X_PERIPH_VSECTION+A1X_PMU_OFFSET)
#define A1X_UART_VADDR(n)    (A1X_PERIPH_VSECTION+A1X_UART_OFFSET(n))
#define A1X_UART0_VADDR      (A1X_PERIPH_VSECTION+A1X_UART0_OFFSET)
#define A1X_UART1_VADDR      (A1X_PERIPH_VSECTION+A1X_UART1_OFFSET)
#define A1X_UART2_VADDR      (A1X_PERIPH_VSECTION+A1X_UART2_OFFSET)
#define A1X_UART3_VADDR      (A1X_PERIPH_VSECTION+A1X_UART3_OFFSET)
#define A1X_UART4_VADDR      (A1X_PERIPH_VSECTION+A1X_UART4_OFFSET)
#define A1X_UART5_VADDR      (A1X_PERIPH_VSECTION+A1X_UART5_OFFSET)
#define A1X_UART6_VADDR      (A1X_PERIPH_VSECTION+A1X_UART6_OFFSET)
#define A1X_UART7_VADDR      (A1X_PERIPH_VSECTION+A1X_UART7_OFFSET)
#define A1X_PS20_VADDR       (A1X_PERIPH_VSECTION+A1X_PS20_OFFSET)
#define A1X_PS21_VADDR       (A1X_PERIPH_VSECTION+A1X_PS21_OFFSET)
#define A1X_TWI0_VADDR       (A1X_PERIPH_VSECTION+A1X_TWI0_OFFSET)
#define A1X_TWI1_VADDR       (A1X_PERIPH_VSECTION+A1X_TWI1_OFFSET)
#define A1X_TWI2_VADDR       (A1X_PERIPH_VSECTION+A1X_TWI2_OFFSET)
#define A1X_CAN_VADDR        (A1X_PERIPH_VSECTION+A1X_CAN_OFFSET)
#define A1X_SCR_VADDR        (A1X_PERIPH_VSECTION+A1X_SCR_OFFSET)
#define A1X_MALI400_VADDR    (A1X_PERIPH_VSECTION+A1X_MALI400_OFFSET)

/* A1X DE section virtual base addresses */

#define A1X_DEFE0_VADDR      (A1X_DE_VSECTION+A1X_DEFE0_OFFSET)
#define A1X_DEFE1_VADDR      (A1X_DE_VSECTION+A1X_DEFE1_OFFSET)
#define A1X_DEBE0_VADDR      (A1X_DE_VSECTION+A1X_DEBE0_OFFSET)
#define A1X_DEBE1_VADDR      (A1X_DE_VSECTION+A1X_DEBE1_OFFSET)
#define A1X_MP_VADDR         (A1X_DE_VSECTION+A1X_MP_OFFSET)
#define A1X_AVG_VADDR        (A1X_DE_VSECTION+A1X_AVG_OFFSET)

/* A1X BRROM section virtual base address */

#define A1X_BROM_VADDR       (A1X_BROM_VSECTION+A1X_BROM_OFFSET)

/* Offset SDRAM address */

#define A1X_DDR_MAPPADDR     (A1X_DDR_PSECTION+A1X_DDR_MAPOFFSET)
#define A1X_DDR_MAPVADDR     (A1X_DDR_VSECTION+A1X_DDR_MAPOFFSET)

/* NuttX virtual base address
 *
 * The boot logic will create a temporarily mapping based on where NuttX is
 * executing in memory.  In this case, NuttX will be running from either
 * internal SRAM or external SDRAM.
 *
 * Setup the RAM region as the NUTTX .txt, .bss, and .data region.
 */

#define NUTTX_TEXT_VADDR     (CONFIG_RAM_VSTART & 0xfff00000)
#define NUTTX_TEXT_PADDR     (CONFIG_RAM_START & 0xfff00000)
#define NUTTX_TEXT_PEND      ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#define NUTTX_TEXT_SIZE      (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

/* MMU Page Table
 *
 * Determine the address of the MMU page table.  Regardless of the memory
 * configuration, we will keep the page table in the A1X's internal SRAM.
 */

#if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)

  /* Sanity check.. if one is undefined, both should be undefined */

#  if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif

  /* A sanity check, if the configuration says that the page table is read-only
   * and pre-initialized (maybe ROM), then it should have also defined both of
   * the page table base addresses.
   */

#  ifdef CONFIG_ARCH_ROMPGTABLE
#    error "CONFIG_ARCH_ROMPGTABLE defined; PGTABLE_BASE_P/VADDR not defined"
#  endif

#else /* PGTABLE_BASE_PADDR || PGTABLE_BASE_VADDR */

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

  #if defined(CONFIG_ARCH_LOWVECTORS)
  /* In this case, table must lie in SRAM A2 after the vectors in SRAM A1 */

#    define PGTABLE_BASE_PADDR  A1X_SRAMA2_PADDR
#    define PGTABLE_BASE_VADDR  A1X_SRAMA2_VADDR

#  else /* CONFIG_ARCH_LOWVECTORS */

  /* Otherwise, the vectors lie at another location.  The page table will
   * then be positioned at the beginning of SRAM A1.
   */

#    define PGTABLE_BASE_PADDR  A1X_SRAMA1_PADDR
#    define PGTABLE_BASE_VADDR  A1X_SRAMA1_VADDR

#  endif /* CONFIG_ARCH_LOWVECTORS */

  /* Note that the page table does not lie in the same address space as does the
   * mapped RAM in either case.  So we will need to create a special mapping for
   * the page table at boot time.
   */

#  define ARMV7A_PGTABLE_MAPPING 1

#endif /* PGTABLE_BASE_PADDR || PGTABLE_BASE_VADDR */

/* Level 2 Page table start addresses.
 *
 * 16Kb of memory is reserved hold the page table for the virtual mappings.  A
 * portion of this table is not accessible in the virtual address space (for
 * normal operation).   There are several large holes in the physical address
 * space for which there will never be level 1 mappings:
 *
 *                                    LI PAGE TABLE
 *   ADDRESS RANGE           SIZE     ENTRIES       SECTIONS
 *   ----------------------- ------- -------------- ---------
 *   0x0003:0000-0x01eb:ffff 275MB   0x0004-0x006c 26
 *                                  *(none usable) 0
 *   0x01ec:0000-0x3fff:ffff 993MB   0x0078-0x0ffc 993
 *                                  *0x0400-0x0ffc 767
 *
 * And the largest is probably from the end of SDRAM through 0xfff0:0000.
 * But the size of that region varies with the size of the installed SDRAM.
 * It is at least:
 *
 *                                    LI PAGE TABLE
 *   ADDRESS RANGE           SIZE     ENTRIES       SECTIONS
 *   ----------------------- ------- -------------- ---------
 *   0xc000:0000-0xffef:ffff 1022MB  *0x3000-0x3ff8 1022
 *
 * And probably much larger.
 *
 *   * NOTE that the L2 page table entries must be aligned 1KB address
 *     boundaries.
 *
 * These two larger regions is where L2 page tables will positioned.  Up to
 * two L2 page tables may be used:
 *
 * 1) One mapping the vector table (only when CONFIG_ARCH_LOWVECTORS is not
 *    defined).
 * 2) If on-demand paging is supported (CONFIG_PAGING=y), than an additional
 *    L2 page table is needed.
 */

#ifndef CONFIG_ARCH_LOWVECTORS
/* Vector L2 page table offset/size */

#  define VECTOR_L2_OFFSET        0x000000400
#  define VECTOR_L2_SIZE          0x000000bfc

/* Vector L2 page table base addresses */

#  define VECTOR_L2_PBASE         (PGTABLE_BASE_PADDR+VECTOR_L2_OFFSET)
#  define VECTOR_L2_VBASE         (PGTABLE_BASE_VADDR+VECTOR_L2_OFFSET)

/* Vector L2 page table end addresses */

#  define VECTOR_L2_END_PADDR     (VECTOR_L2_PBASE+VECTOR_L2_SIZE)
#  define VECTOR_L2_END_VADDR     (VECTOR_L2_VBASE+VECTOR_L2_SIZE)

#endif /* !CONFIG_ARCH_LOWVECTORS */

/* Paging L2 page table offset/size */

#define PGTABLE_L2_START_PADDR    (A1X_DDR_PSECTION+A1X_DDR_MAPOFFSET+A1X_DDR_MAPSIZE)
#define PGTABLE_BROM_OFFSET       0x3ffc

#define PGTABLE_L2_OFFSET         ((PGTABLE_L2_START_PADDR >> 18) & ~3)
#define PGTABLE_L2_SIZE           (PGTABLE_BROM_OFFSET - PGTABLE_L2_OFFSET)

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
 *   A1X_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
 *   A1X_VECTOR_VSRAM - Virtual address of vector table in SRAM
 *   A1X_VECTOR_VADDR - Virtual address of vector table (0x00000000 or 0xffff0000)
 *
 * NOTE: When using LOWVECTORS, the actual base of the vectors appears to be
 * offset to address 0x0000:0040
 */

#define VECTOR_TABLE_SIZE         0x00010000
#define VECTOR_TABLE_OFFSET       0x00000040

#ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */

#  define A1X_VECTOR_PADDR        A1X_SRAMA1_PADDR
#  define A1X_VECTOR_VSRAM        A1X_SRAMA1_VADDR
#  define A1X_VECTOR_VADDR        0x00000000

#else  /* Vectors located at 0xffff:0000 -- this probably does not work */

#  ifdef A1X_ISRAM1_SIZE >= VECTOR_TABLE_SIZE
#    define A1X_VECTOR_PADDR      (A1X_SRAMA1_PADDR+A1X_ISRAM1_SIZE-VECTOR_TABLE_SIZE)
#    define A1X_VECTOR_VSRAM      (A1X_SRAMA1_VADDR+A1X_ISRAM1_SIZE-VECTOR_TABLE_SIZE)
#  else
#    define A1X_VECTOR_PADDR      (A1X_SRAMA1_PADDR+A1X_ISRAM0_SIZE-VECTOR_TABLE_SIZE)
#    define A1X_VECTOR_VSRAM      (A1X_SRAMA1_VADDR+A1X_ISRAM0_SIZE-VECTOR_TABLE_SIZE)
#  endif
#  define A1X_VECTOR_VADDR        0xffff0000

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

#endif /* __ARCH_ARM_SRC_A1X_CHIP_A10_MEMORYMAP_H */
