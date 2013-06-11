/************************************************************************************************
 * arch/arm/src/sam34/chip/sam4s_memorymap.h
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4S_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4S_MEMORYMAP_H

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
#define SAM_EXTDEV_BASE        0xa0000000 /* 0xa0000000-0xdfffffff: External device */
#define SAM_SYSTEM_BASE        0xe0000000 /* 0xe0000000-0xffffffff: System */

/* Code memory region */

#define SAM_BOOTMEMORY_BASE    0x00000000 /* 0x00000000-0x003fffff: Boot Memory */
#define SAM_INTFLASH_BASE      0x00400000 /* 0x00400000-0x007fffff: Internal FLASH */
#define SAM_INTROM_BASE        0x00800000 /* 0x00180000-0x00bfffff: Internal ROM */
                                          /* 0x00c00000-0x1fffffff: Reserved */
/* Internal SRAM memory region */

#define SAM_INTSRAM0_BASE      0x20000000 /* For SAM3U compatibility */
#define SAM_BBSRAM_BASE        0x22000000 /* 0x22000000-0x23ffffff: 32MB bit-band region */
                                          /* 0x24000000-0x3fffffff: Undefined */
/* Peripherals address region */

#define SAM_HSMCI_BASE         0x40000000 /* 0x40000000-0x400003ff: High Speed Multimedia Card Interface */
#define SAM_SSC_BASE           0x40004000 /* 0x40004000-0x40007fff: Synchronous Serial Controller */
#define SAM_SPI_BASE           0x40008000 /* 0x40008000-0x4000bfff: Serial Peripheral Interface */
                                          /* 0x4000c000-0x4000ffff: Reserved */
#define SAM_TC_BASE            0x40010000 /* 0x40010000-0x40017fff: Timer Counters */
#  define SAM_TC0_BASE         0x40080000 /* 0x40010000-0x4001003f:   Timer Counter 0 */
#  define SAM_TC1_BASE         0x40080040 /* 0x40010040-0x4001007f:   Timer Counter 1 */
#  define SAM_TC2_BASE         0x40080080 /* 0x40010080-0x400100bf:   Timer Counter 2 */
                                          /* 0x400100c0-0x40013fff    Reserved */
#  define SAM_TC3_BASE         0x40080000 /* 0x40014000-0x4001403f:   Timer Counter 3 */
#  define SAM_TC4_BASE         0x40080040 /* 0x40014040-0x4001407f:   Timer Counter 4 */
#  define SAM_TC5_BASE         0x40080080 /* 0x40014080-0x400140bf:   Timer Counter 5 */
#define SAM_TWI_BASE           0x40018000 /* 0x40018000-0x4001ffff: Two-Wire Interface */
#  define SAM_TWI0_BASE        0x40018000 /* 0x40018000-0x4001bfff:   Two-Wire Interface 0 */
#  define SAM_TWI1_BASE        0x4001c000 /* 0x4001c000-0x4001ffff:   Two-Wire Interface 1 */
#define SAM_PWM_BASE           0x40020000 /* 0x40020000-0x4003ffff: Pulse Width Modulation */
#define SAM_USART_BASE         0x40024000 /* 0x40024000-0x4002bfff: USART */
#  define SAM_USART0_BASE      0x40024000 /* 0x40024000-0x40023fff:   USART0 */
#  define SAM_USART1_BASE      0x40028000 /* 0x40028000-0x4002bfff:   USART1 */
                                          /* 0x4002C000-0x4002ffff: Reserved */
                                          /* 0x40030000-0x40033fff: Reserved */
#define SAM_UDP_BASE           0x40034000 /* 0x40034000-0x40037fff: USB 2.0 Device */
#define SAM_ADC_BASE           0x40038000 /* 0x40038000-0x4003bfff: Analog To Digital Converter */
#define SAM_DACCBASE           0x400cC000 /* 0x4003c000-0x4003ffff: Digital To Analog Converter */
#define SAM_ACC_BASE           0x40040000 /* 0x40040000-0x40043fff: Analog Comparator */
#define SAM_CRCCU_BASE         0x40044000 /* 0x40040000-0x40047fff: CRC Calculation Unit */
                                          /* 0x40048000-0x400dffff: Reserved */
#define SAM_SYSCTRLR_BASE      0x400e0000 /* 0x400e0000-0x400e25ff: System Controller */
                                          /* 0x400e2600-0x400fffff: Reserved */
                                          /* 0x40100000-0x4002ffff: Reserved */
#define SAM_BBPERIPH_BASE      0x42000000 /* 0x42000000-0x43ffffff: 32MB bit-band region */
                                          /* 0x44000000-0x5fffffff: Reserved */
/* System Controller Register Blocks:  0x400e0000-0x4007ffff */

#define SAM_SMC_BASE           0x400e0000 /* 0x400e0000-0x400e01ff: Static Memory Controller */
#define SAM_MATRIX_BASE        0x400e0200 /* 0x400e0200-0x400e03ff: MATRIX */
#define SAM_PMC_BASE           0x400e0400 /* 0x400e0400-0x400e05ff: Power Management Controller */
#define SAM_UART0_BASE         0x400e0600 /* 0x400e0600-0x400e073f: UART 0 */
#define SAM_CHIPID_BASE        0x400e0740 /* 0x400e0740-0x400e07ff: CHIP ID */
#define SAM_UART1_BASE         0x400e0800 /* 0x400e0800-0x400e0bff: UART 1 */
#define SAM_EEFC_BASE          0x400e0a00 /* 0x400e0a00-0x400e0bff: Enhanced Embedded Flash Controllers*/
#  define SAM_EEFC0_BASE       0x400e0a00 /* 0x400e0a00-0x400e0bff:   Enhanced Embedded Flash Controller 0 */
#  define SAM_EEFC1_BASE       0x400e0c00 /* 0x400e0c00-0x400e0dff:   Enhanced Embedded Flash Controller 1 */
#define SAM_PIO_BASE           0x400e0e00 /* 0x400e0e00-0x400e13ff: Parallel I/O Controllers */
#  define SAM_PION_BASE(n)     (0x400e0e00 + ((n) << 9))
#  define SAM_PIOA_BASE        0x400e0e00 /* 0x400e0e00-0x400e0fff:   Parallel I/O Controller A */
#  define SAM_PIOB_BASE        0x400e1000 /* 0x400e1000-0x400e11ff:   Parallel I/O Controller B */
#  define SAM_PIOC_BASE        0x400e1200 /* 0x400e1200-0x400e13ff:   Parallel I/O Controller C */
#define SAM_RSTC_BASE          0x400e1400 /* 0x400e1400-0x400e140f: Reset Controller */
#define SAM_SUPC_BASE          0x400e1410 /* 0x400e1410-0x400e142f: Supply Controller */
#define SAM_RTT_BASE           0x400e1430 /* 0x400e1430-0x400e144f: Real Time Timer */
#define SAM_WDT_BASE           0x400e1450 /* 0x400e1250-0x400e145f: Watchdog Timer */
#define SAM_RTC_BASE           0x400e1460 /* 0x400e1460-0x400e148f: Real Time Clock */
#define SAM_GPBR_BASE          0x400e1490 /* 0x400e1490-0x400e15ff: GPBR */
                                          /* 0x400e1600-0x4007ffff: Reserved */
/* External RAM memory region */

#define SAM_EXTCS_BASE         0x60000000 /* 0x60000000-0x63ffffff: Chip selects */
#  define SAM_EXTCSN_BASE(n)   (0x60000000*((n)<<24))
#  define SAM_EXTCS0_BASE      0x60000000 /* 0x60000000-0x60ffffff:   Chip select 0 */
#  define SAM_EXTCS1_BASE      0x61000000 /* 0x61000000-0x601fffff:   Chip select 1 */
#  define SAM_EXTCS2_BASE      0x62000000 /* 0x62000000-0x62ffffff:   Chip select 2 */
#  define SAM_EXTCS3_BASE      0x63000000 /* 0x63000000-0x63ffffff:   Chip select 3 */
                                          /* 0x64000000-0x9fffffff: Reserved */
/* System memory region */

#define SAM_PRIVPERIPH_BASE    0xe0000000 /* 0xe0000000-0xe00fffff: Private peripheral bus */
#define SAM_VENDOR_BASE        0xe0100000 /* 0ex0100000-0xffffffff: Vendor-specific memory */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4S_MEMORYMAP_H */
