/************************************************************************************************
 * arch/arm/src/sam34/chip/sam3x_memorymap.h
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM3X_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM3X_MEMORYMAP_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Address regions */

#define SAM_CODE_BASE          0x00000000 /* 0x00000000-0x1fffffff: Code space */
#define SAM_INTSRAM_BASE       0x20000000 /* 0x20000000-0x3fffffff: Internal SRAM */
#define SAM_PERIPHERALS_BASE   0x40000000 /* 0x40000000-0x5fffffff: Peripherals */
#define SAM_EXTRAM_BASE        0x60000000 /* 0x60000000-0x9fffffff: External RAM */
#define SAM_SYSTEM_BASE        0xe0000000 /* 0xe0000000-0xffffffff: System */

/* Code memory region */

#define SAM_BOOTMEMORY_BASE    0x00000000 /* 0x00000000-0x003fffff: Boot Memory */
#define SAM_INTFLASH0_BASE     0x00080000 /* 0x00080000-0x000fffff: Internal FLASH 0 */
#define SAM_INTFLASH1_BASE     (0x00080000 + SAM34_FLASH_SIZE/2)
#define SAM_INTROM_BASE        0x00100000 /* 0x00100000-0x001fffff: Internal ROM */
                                          /* 0x00200000-0x1fffffff: Reserved */
/* Internal SRAM memory region */

#define SAM_INTSRAM0_BASE      0x20000000 /* 0x20000000-0x2007ffff: Internal SRAM 0 */
#define SAM_INTSRAM1_BASE      0x20080000 /* 0x20080000-0x200fffff: Internal SRAM 1 */
#define SAM_NFCSRAM_BASE       0x20100000 /* 0x20100000-0x2017ffff: NAND FLASH controller (SRAM) */
#define SAM_UOTGHSRAM_BASE     0x20180000 /* 0x20100000-0x201fffff: UOTGHS controller (DMA) */
                                          /* 0x20200000-0x201fffff: Undefined */
#define SAM_BBSRAM_BASE        0x22000000 /* 0x22000000-0x23ffffff: 32Mb bit-band alias */
                                          /* 0x24000000-0x3fffffff: Undefined */
/* Peripherals address region */

#define SAM_HSMCI_BASE         0x40000000 /* 0x40000000-0x400003ff: High Speed Multimedia Card Interface */
#define SAM_SSC_BASE           0x40004000 /* 0x40004000-0x40007fff: Synchronous Serial Controller */
#define SAM_SPI0_BASE          0x40008000 /* 0x40008000-0x4000bfff: Serial Peripheral Interface 0 */
#define SAM_SPI1_BASE          0x4000c000 /* 0x40008000-0x4007ffff: Serial Peripheral Interface 1 */
#define SAM_TC_BASE            0x40080000 /* 0x40080000-0x4008bfff: Timer Counters */
#  define SAM_TC0_BASE         0x40080000 /* 0x40080000-0x4008003f:   Timer Counter 0 */
#  define SAM_TC1_BASE         0x40080040 /* 0x40080040-0x4008007f:   Timer Counter 1 */
#  define SAM_TC2_BASE         0x40080080 /* 0x40080080-0x400800bf:   Timer Counter 2 */
                                          /* 0x400800c0-0x40083fff    Reserved */
#  define SAM_TC3_BASE         0x40084000 /* 0x40084000-0x4008403f:   Timer Counter 0 */
#  define SAM_TC4_BASE         0x40084040 /* 0x40084040-0x4008407f:   Timer Counter 1 */
#  define SAM_TC5_BASE         0x40084080 /* 0x40084080-0x400840bf:   Timer Counter 2 */
                                          /* 0x400840c0-0x40087fff    Reserved */
#  define SAM_TC6_BASE         0x40088000 /* 0x40088000-0x4008803f:   Timer Counter 3 */
#  define SAM_TC7_BASE         0x40088040 /* 0x40088040-0x4008807f:   Timer Counter 4 */
#  define SAM_TC8_BASE         0x40088080 /* 0x40088080-0x400880bf:   Timer Counter 5 */
                                          /* 0x400880c0-0x4008ffff    Reserved */
#define SAM_TWI_BASE           0x4008c000 /* 0x4008c000-0x4001ffff: Two-Wire Interface */
#  define SAM_TWI0_BASE        0x4008c000 /* 0x4008c000-0x4008ffff:   Two-Wire Interface 0 */
#  define SAM_TWI1_BASE        0x40090000 /* 0x40090000-0x40093fff:   Two-Wire Interface 1 */
#define SAM_PWM_BASE           0x40094000 /* 0x40020000-0x4003ffff: Pulse Width Modulation */
#define SAM_USART_BASE         0x40098000 /* 0x40098000-0x4002bfff: USART */
#  define SAM_USART0_BASE      0x40098000 /* 0x40098000-0x400a7fff:   USART0 */
#  define SAM_USART1_BASE      0x4009c000 /* 0x4009c000-0x400bffff:   USART1 */
#  define SAM_USART2_BASE      0x400a0000 /* 0x40028000-0x400a3fff:   USART2 */
#  define SAM_USART3_BASE      0x400a4000 /* 0x40028000-0x400a7fff:   USART3 */
                                          /* 0x400a8000-0x400ac000: Reserved */
#define SAM_UOTGHS_BASE        0x40034000 /* 0x40034000-0x400affff: USB OTG High Speed */
#define SAM_EMAC_BASE          0x400b0000 /* 0x400b0000-0x400b3fff: Ethernet MAC */
#define SAM_CAN_BASE           0x400b4000 /* 0x400b4000-0x400bbfff: CAN */
#define SAM_CAN0_BASE          0x400b4000 /* 0x400b4000-0x400b7fff:   CAN0 */
#define SAM_CAN1_BASE          0x400b8000 /* 0x400b8000-0x400bbfff:   CAN1 */
#define SAM_TRNG_BASE          0x400bc000 /* 0x400bc000-0x400bffff: True Random Number Generator */
#define SAM_ADC_BASE           0x400c0000 /* 0x400c0000-0x400c3fff: Analog To Digital Converter */
#define SAM_DMAC_BASE          0x400c4000 /* 0x400c4000-0x400c7fff: DMA controller */
#define SAM_DACC_BASE          0x400c8000 /* 0x400c8000-0x400cffff: Digital To Analog Converter */
                                          /* 0x400d0000-0x400dffff: Reserved */
#define SAM_SYSCTRLR_BASE      0x400e0000 /* 0x400e0000-0x4007ffff: System Controller */
                                          /* 0x40080000-0x40ffffff: Reserved */
                                          /* 0x41000000-0x41ffffff: Undefined */
#define SAM_BBPERIPH_BASE      0x42000000 /* 0x42000000-0x43ffffff: 32Mb bit-band alias */
                                          /* 0x44000000-0x5fffffff: Undefined */
/* System Controller Register Blocks:  0x400e0000-0x4007ffff */

#define SAM_SMC_BASE           0x400e0000 /* 0x400e0000-0x400e01ff: Static Memory Controller */
#define SAM_SDRAMC_BASE        0x400e0200 /* 0x400e0200-0x400e03ff: SDRAM Controller */
#define SAM_MATRIX_BASE        0x400e0400 /* 0x400e0400-0x400E05ff: MATRIX */
#define SAM_PMC_BASE           0x400e0600 /* 0x400e0600-0x400e07ff: Power Management Controller */
#define SAM_UART0_BASE         0x400e0800 /* 0x400e0800-0x400e093f: UART 0 */
#define SAM_CHIPID_BASE        0x400e0940 /* 0x400e0940-0x400e09ff: CHIP ID */
#define SAM_EEFC_BASE          0x400e0a00 /* 0x400e0a00-0x400e0bff: Enhanced Embedded Flash Controllers*/
#  define SAM_EEFC0_BASE       0x400e0a00 /* 0x400e0a00-0x400e0bff:   Enhanced Embedded Flash Controller 0 */
#  define SAM_EEFC1_BASE       0x400e0c00 /* 0x400e0c00-0x400e0dff:   Enhanced Embedded Flash Controller 1 */
#define SAM_PIO_BASE           0x400e0e00 /* 0x400e0e00-0x400e13ff: Parallel I/O Controllers */
#  define SAM_PION_BASE(n)     (0x400e0e00 + ((n) << 9))
#  define SAM_PIOA_BASE        0x400e0e00 /* 0x400e0e00-0x400e0fff:   Parallel I/O Controller A */
#  define SAM_PIOB_BASE        0x400e1000 /* 0x400e1000-0x400e11ff:   Parallel I/O Controller B */
#  define SAM_PIOC_BASE        0x400e1200 /* 0x400e1200-0x400e13ff:   Parallel I/O Controller C */
#  define SAM_PIOD_BASE        0x400e1400 /* 0x400e1400-0x400e15ff:   Parallel I/O Controller D */
#  define SAM_PIOE_BASE        0x400e1600 /* 0x400e1600-0x400e17ff:   Parallel I/O Controller E */
#  define SAM_PIOF_BASE        0x400e1800 /* 0x400e1800-0x400e19ff:   Parallel I/O Controller F */
#define SAM_RSTC_BASE          0x400e1a00 /* 0x400e1a00-0x400e1a0f: Reset Controller */
#define SAM_SUPC_BASE          0x400e1a10 /* 0x400e1a10-0x400e1a2f: Supply Controller */
#define SAM_RTT_BASE           0x400e1a30 /* 0x400e1a30-0x400e1a4f: Real Time Timer */
#define SAM_WDT_BASE           0x400e1a50 /* 0x400e1a50-0x400e1a5f: Watchdog Timer */
#define SAM_RTC_BASE           0x400e1a60 /* 0x400e1a60-0x400e1a8f: Real Time Clock */
#define SAM_GPBR_BASE          0x400e1a90 /* 0x400e1a90-0x400e1aaf: GPBR */
                                          /* 0x400e1ab0-0x4007ffff: Reserved */
/* External RAM memory region */

#define SAM_EXTCS_BASE         0x60000000 /* 0x60000000-0x63ffffff: Chip selects */
#  define SAM_EXTCSN_BASE(n)   (0x60000000*((n)<<24))
#  define SAM_EXTCS0_BASE      0x60000000 /* 0x60000000-0x60ffffff:   Chip select 0 */
#  define SAM_EXTCS1_BASE      0x61000000 /* 0x61000000-0x61ffffff:   Chip select 1 */
#  define SAM_EXTCS2_BASE      0x62000000 /* 0x62000000-0x62ffffff:   Chip select 2 */
#  define SAM_EXTCS3_BASE      0x63000000 /* 0x63000000-0x63ffffff:   Chip select 3 */
#  define SAM_EXTCS4_BASE      0x64000000 /* 0x64000000-0x64ffffff:   Chip select 3 */
#  define SAM_EXTCS5_BASE      0x65000000 /* 0x65000000-0x65ffffff:   Chip select 3 */
#  define SAM_EXTCS6_BASE      0x66000000 /* 0x66000000-0x66ffffff:   Chip select 3 */
#  define SAM_EXTCS7_BASE      0x67000000 /* 0x67000000-0x67ffffff:   Chip select 3 */
#define SAM_NFC_BASE           0x68000000 /* 0x68000000-0x68ffffff: NFC */
                                          /* 0x69000000-0x6fffffff: Reserved */
#define SAM_SDRAMCS_BASE       0x70000000 /* 0x70000000-0x7fffffff: SDRAM chip select */
                                          /* 0x80000000-0x9fffffff: Reserved */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM3X_MEMORYMAP_H */
