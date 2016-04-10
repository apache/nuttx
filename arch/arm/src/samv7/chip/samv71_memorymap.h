/************************************************************************************************
 * arch/arm/src/samv7/chip/samv71_memorymap.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMV7_CHIP_SAMV71_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAMV7_CHIP_SAMV71_MEMORYMAP_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Address regions */

#define SAM_CODE_BASE          0x00000000 /* 0x00000000-0x1fffffff: Code space */
#define SAM_INTSRAM_BASE       0x20000000 /* 0x20000000-0x3fffffff: Internal SRAM */
#define SAM_PERIPHERALS_BASE   0x40000000 /* 0x40000000-0x5fffffff: Peripherals */
#define SAM_MEMORY_BASE        0x60000000 /* 0x60000000-0x7fffffff: Memories */
#define SAM_QSPIMEM_BASE       0x80000000 /* 0x80000000-0x9fffffff: QSPI memory */
#define SAM_AXIMX_BASE         0xa0000000 /* 0xa0000000-0x9fffffff: AXIMX */
#define SAM_USBHSRAM_BASE      0xa0100000 /* 0xa0100000-0xa01fffff: USBHS RAM */
                                          /* 0xa0200000-0xdfffffff: Reserved */
#define SAM_SYSTEM_BASE        0xe0000000 /* 0xe0000000-0xffffffff: System */

/* Code memory region */

#define SAM_BOOTMEMORY_BASE    0x00000000 /* 0x00000000-0x00ffffff: ITCM or Boot Memory */
#define SAM_INTFLASH_BASE      0x00400000 /* 0x00400000-0x007fffff: Internal FLASH */
#define SAM_INTROM_BASE        0x00800000 /* 0x00800000-0x00bfffff: Internal ROM */
                                          /* 0x00c00000-0x1fffffff: Reserved */
/* Internal SRAM memory region */

#define SAM_DTCM_BASE          0x20000000 /* 0x20000000-0x203fffff: DTCM */
#define SAM_SRAM_BASE          0x20400000 /* 0x20400000-0x20bfffff: SRAM  */
                                          /* 0x20c00000-0x3fffffff: Reserved */
/* Peripherals address region */

#define SAM_HSMCI0_BASE        0x40000000 /* 0x40000000-0x40003fff: High Speed Multimedia Card Interface */
#define SAM_SSC0_BASE          0x40004000 /* 0x40004000-0x40007fff: Serial Synchronous Controller */
#define SAM_SPI0_BASE          0x40008000 /* 0x40008000-0x4000bfff: Serial Peripheral Interface 0 */
#define SAM_TC012_BASE         0x4000c000 /* 0x4000c000-0x4000ffff: Timer Counters 0-2 */
#  define SAM_TC0_BASE         0x4000c000 /* 0x4000c000-0x4000c03f:   Timer Counter 0 */
#  define SAM_TC1_BASE         0x4000c040 /* 0x4000c040-0x4000c07f:   Timer Counter 1 */
#  define SAM_TC2_BASE         0x4000c080 /* 0x4000c080-0x4000c0bf:   Timer Counter 2 */
                                          /* 0x4000c0c0-0x4000ffff    Reserved */
#define SAM_TC345_BASE         0x40010000 /* 0x40010000-0x40013fff: Timer Counters 3-5 */
#  define SAM_TC3_BASE         0x40010000 /* 0x40010000-0x4001003f:   Timer Counter 3 */
#  define SAM_TC4_BASE         0x40010040 /* 0x40010040-0x4001007f:   Timer Counter 4 */
#  define SAM_TC5_BASE         0x40010080 /* 0x40010080-0x400100bf:   Timer Counter 5 */
                                          /* 0x400100c0-0x40013fff    Reserved */
#define SAM_TC678_BASE         0x40014000 /* 0x40014000-0x40017fff: Timer Counters 6-8 */
#  define SAM_TC6_BASE         0x40014000 /* 0x40014000-0x4001403f:   Timer Counter 6 */
#  define SAM_TC7_BASE         0x40014040 /* 0x40014040-0x4001407f:   Timer Counter 7 */
#  define SAM_TC8_BASE         0x40014080 /* 0x40014080-0x400140bf:   Timer Counter 8 */
                                          /* 0x400140c0-0x40017fff: Reserved */
#define SAM_TWIHS0_BASE        0x40018000 /* 0x40018000-0x4001bfff: Two-Wire Interface 0 (TWIHS0) */
#define SAM_TWIHS1_BASE        0x4001c000 /* 0x4001c000-0x4001ffff: Two-Wire Interface 1 (TWIHS1) */
#define SAM_PWM0_BASE          0x40020000 /* 0x40020000-0x40023fff: Pulse Width Modulation 0 */
#define SAM_USART0_BASE        0x40024000 /* 0x40024000-0x40023fff:   USART0 */
#define SAM_USART1_BASE        0x40028000 /* 0x40028000-0x4002bfff:   USART1 */
#define SAM_USART2_BASE        0x4002c000 /* 0x4002c000-0x4002ffff:   USART12*/
#define SAM_MCAN0_BASE         0x40030000 /* 0x40030000-0x40033fff: Controller Area Network 0 (MCAN0) */
#define SAM_MCAN1_BASE         0x40034000 /* 0x40034000-0x40037fff: Controller Area Network 1 (MCAN1) */
#define SAM_USBHS_BASE         0x40038000 /* 0x40038000-0x4003bfff: USB High-Speed (USBHS) */
#define SAM_AFEC0_BASE         0x4003c000 /* 0x4003c000-0x4003ffff: Analog Front End 0 (AFEC0) */
#define SAM_DACC_BASE          0x40040000 /* 0x40040000-0x40043fff: Digital To Analog Converter (DACC) */
#define SAM_ACC_BASE           0x40044000 /* 0x40044000-0x40047fff: Analog Comparator (ACC) */
#define SAM_ICM_BASE           0x40048000 /* 0x40048000-0x4004bfff: Integrity Check Monitor (ICM) */
#define SAM_ISI_BASE           0x4004c000 /* 0x4004c000-0x4004ffff: Image Sensor Interface (ISI) */
#define SAM_EMAC0_BASE         0x40050000 /* 0x40050000-0x40053fff: EMAC0 (aka GMAC) */
#define SAM_TC901_BASE         0x40054000 /* 0x40054000-0x40054fff: Timer Counters 9-11 */
#  define SAM_TC9_BASE         0x40054000 /* 0x40054000-0x4005403f:   Timer Counter 9 */
#  define SAM_TC10_BASE        0x40054040 /* 0x40054040-0x4005407f:   Timer Counter 10 */
#  define SAM_TC11_BASE        0x40054080 /* 0x40054080-0x400540bf:   Timer Counter 11 */
                                          /* 0x400540c0-0x40057fff:   Reserved */
#define SAM_SPI1_BASE          0x40058000 /* 0x40058000-0x4005bfff: Serial Peripheral Interface 1 */
#define SAM_PWM1_BASE          0x4005c000 /* 0x4005c000-0x4005ffff: Pulse Width Modulation 1 */
#define SAM_TWIHS2_BASE        0x40060000 /* 0x40060000-0x40063fff: Two-Wire Interface 2 (TWIHS2) */
#define SAM_AFEC1_BASE         0x40064000 /* 0x40064000-0x40067fff: Analog Front End 1 (AFEC1) */
#define SAM_MLB_BASE           0x40068000 /* 0x40068000-0x4006bfff: Media LB Interface (MLB) */
#define SAM_AES_BASE           0x4006c000 /* 0x4006c000-0x4006ffff: AES */
#define SAM_TRNG_BASE          0x40070000 /* 0x40070000-0x40073fff: True Random Number Generator (TRNG) */
#define SAM_BRAM_BASE          0x40074000 /* 0x40074000-0x40077fff: Backup SRAM (BRAM) */
#define SAM_XDMAC_BASE         0x40078000 /* 0x40078000-0x4007bfff: Central DMA controller (XDMAC) */
#define SAM_QSPI_BASE          0x4007c000 /* 0x4007c000-0x4007ffff: Quad SPI (QSPI) */
#define SAM_SMC_BASE           0x40080000 /* 0x40080000-0x40083fff: Static Memory Controller (SMC) */
#define SAM_SDRAMC_BASE        0x40084000 /* 0x40084000-0x40087fff: SDRAM Controller (SDRAMC) */
#define SAM_MATRIX_BASE        0x40088000 /* 0x40088000-0x400e03ff: MATRIX */
#define SAM_UTMI_BASE          0x400e0400 /* 0x400e0400-0x400e05ff: USB Transmitter Macrocell Interface (UTMI) */
#define SAM_PMC_BASE           0x400e0600 /* 0x400e0600-0x400e07ff: Power Management Controller (PMC) */
#define SAM_UART0_BASE         0x400e0800 /* 0x400e0800-0x400e093f: UART 0 */
#define SAM_CHIPID_BASE        0x400e0940 /* 0x400e0940-0x400e09ff: CHIP ID */
#define SAM_UART1_BASE         0x400e0a00 /* 0x400e0a00-0x400e0bff: UART 1 */
#define SAM_EEFC_BASE          0x400e0c00 /* 0x400e0c00-0x400e0dff: Embedded Flash Controller (EEFC) */
#define SAM_PIO_BASE           0x400e0e00 /* 0x400e0e00-0x400e13ff: Parallel I/O Controllers */
#  define SAM_PION_BASE(n)     (0x400e0e00 + ((n) << 9))
#  define SAM_PIOA_BASE        0x400e0e00 /* 0x400e0e00-0x400e0fff:   Parallel I/O Controller A */
#  define SAM_PIOB_BASE        0x400e1000 /* 0x400e1000-0x400e11ff:   Parallel I/O Controller B */
#  define SAM_PIOC_BASE        0x400e1200 /* 0x400e1200-0x400e13ff:   Parallel I/O Controller C */
#  define SAM_PIOD_BASE        0x400e1400 /* 0x400e1400-0x400e15ff:   Parallel I/O Controller D */
#  define SAM_PIOE_BASE        0x400e1600 /* 0x400e1600-0x400e17ff:   Parallel I/O Controller E */
#define SAM_SYSCTRLR_BASE      0x400e0000 /* 0x400e0000-0x400e1bff: System Controller */
#  define SAM_RSTC_BASE        0x400e1800 /* 0x400e1800-0x400e180f: Reset Controller (RSTC) */
#  define SAM_SUPC_BASE        0x400e1810 /* 0x400e1810-0x400e182f: Supply Controller (SUPC) */
#  define SAM_RTT_BASE         0x400e1830 /* 0x400e1830-0x400e184f: Real Time Timer (RTT) */
#  define SAM_WDT_BASE         0x400e1850 /* 0x400e1850-0x400e185f: Watchdog Timer(WDT) */
#  define SAM_RTC_BASE         0x400e1860 /* 0x400e1860-0x400e188f: Real Time Clock (RTC) */
#  define SAM_GPBR_BASE        0x400e1890 /* 0x400e1890-0x400e18ff: GPBR */
#  define SAM_SYSC_BASE        0x400e18e0 /* 0x400e1890-0x400e18ff: System Controller Common */
#  define SAM_RSWDT_BASE       0x400e1900 /* 0x400e1900-0x400e19ff: Reinforced Safety Watchdog Timer (RSWDT) */
#define SAM_UART2_BASE         0x400e1a00 /* 0x400e1a00-0x400e1bff: UART 2 */
#define SAM_UART3_BASE         0x400e1c00 /* 0x400e1c00-0x400e1dff: UART 3 */
#define SAM_UART4_BASE         0x400e1e00 /* 0x400e1e00-0x400e1fff: UART 4 */
                                          /* 0x400e2000-0x5fffffff: Reserved */
/* External RAM memory region */

#define SAM_EXTCS_BASE         0x60000000 /* 0x60000000-0x7fffffff: EBI Chip selects */
#  define SAM_EXTCSn_BASE(n)   (0x60000000 + ((n) << 24))
#  define SAM_EXTCS0_BASE      0x60000000 /* 0x60000000-0x60ffffff:   EBI Chip select 0 */
#  define SAM_EXTCS1_BASE      0x61000000 /* 0x61000000-0x601fffff:   EBI Chip select 1 */
#  define SAM_EXTCS2_BASE      0x62000000 /* 0x62000000-0x62ffffff:   EBI Chip select 2 */
#  define SAM_EXTCS3_BASE      0x63000000 /* 0x63000000-0x6fffffff:   EBI Chip select 3 */
#define SAM_SDRAMCS_BASE       0x70000000 /* 0x70000000-0x7fffffff: SDRAM chip select */

/* QSPI memory region */

#define SAM_QSPIMEM_SIZE       0x20000000 /* 0x80000000-0x9fffffff: QSPI memory */

/* System memory region */

#define SAM_PRIVPERIPH_BASE    0xe0000000 /* 0xe0000000-0xe00fffff: Private peripheral bus */
#define SAM_VENDOR_BASE        0xe0100000 /* 0ex0100000-0xffffffff: Vendor-specific memory */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_SAMV7_CHIP_SAM4S_MEMORYMAP_H */
