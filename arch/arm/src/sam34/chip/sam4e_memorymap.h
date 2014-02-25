/************************************************************************************************
 * arch/arm/src/sam34/chip/sam4e_memorymap.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4E_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4E_MEMORYMAP_H

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
                                          /* 0xa0000000-0xdfffffff: Reserved */
#define SAM_SYSTEM_BASE        0xe0000000 /* 0xe0000000-0xffffffff: System */

/* Code memory region */

#define SAM_BOOTMEMORY_BASE    0x00000000 /* 0x00000000-0x003fffff: Boot Memory */
#define SAM_INTFLASH_BASE      0x00400000 /* 0x00400000-0x007fffff: Internal FLASH */
#define SAM_INTROM_BASE        0x00800000 /* 0x00180000-0x00bfffff: Internal ROM */
                                          /* 0x00c00000-0x1fffffff: Reserved */
/* Internal SRAM memory region */

#define SAM_INTSRAM0_BASE      0x20000000 /* For SAM3U compatibility */
                                          /* 0x20400000-0x207fffff: Reserved */
                                          /* 0x20800000-0x3fffffff: Undefined (abort) */

#define SAM_BBSRAM_BASE        0x22000000 /* 0x22000000-0x23ffffff: 32MB bit-band region */

/* Peripherals address region */

#define SAM_PWM_BASE           0x40000000 /* 0x40000000-0x40003fff: Pulse Width Modulation */
#define SAM_AES_BASE           0x40004000 /* 0x40004000-0x40007fff: AES */
                                          /* 0x40008000-0x40010000: Reserved */
#define SAM_CAN0_BASE          0x40010000 /* 0x40010000-0x40013fff: CAN0 */
#define SAM_CAN1_BASE          0x40014000 /* 0x40014000-0x40017fff: CAN1 */
                                          /* 0x40018000-0x40033fff: Reserved */
#define SAM_EMAC_BASE          0x40034000 /* 0x40034000-0x40037fff: EMAC */
                                          /* 0x40038000-0x4005ffff: Reserved */
#define SAM_MPSYSCTRL_BASE     0x40060000 /* 0x40060000-0x4007ffff: MP Sys Controller */
#  define SAM_SMC_BASE         0x40060000 /* 0x40060000-0x400601ff: Static Memory Controller */
                                          /* 0x40060200-0x400605ff: Reserved */
#  define SAM_UART1_BASE       0x40060600 /* 0x40060600-0x400607ff: UART 1 */
                                          /* 0x40060900-0x4007ffff: Reserved */
#define SAM_HSMCI_BASE         0x40080000 /* 0x40080000-0x40083fff: High Speed Multimedia Card Interface */
#define SAM_UDP_BASE           0x40084000 /* 0x40084000-0x40087fff: USB device */
#define SAM_SPI0_BASE          0x40088000 /* 0x40088000-0x4008bfff: Serial Peripheral Interface */
                                          /* 0x4008c000-0x4008ffff: Reserved */
#define SAM_TC012_BASE         0x40090000 /* 0x40090000-0x40097fff: Timer Counters 0-2 */
#  define SAM_TC0_BASE         0x40090000 /* 0x40090000-0x4009003f:   Timer Counter 0 */
#  define SAM_TC1_BASE         0x40090040 /* 0x40090040-0x4009007f:   Timer Counter 1 */
#  define SAM_TC2_BASE         0x40090080 /* 0x40090080-0x400900bf:   Timer Counter 2 */
                                          /* 0x400900c0-0x40093fff    Reserved */
#define SAM_TC345_BASE         0x40094000 /* 0x40094000-0x40094fff: Timer Counters 3-5 */
#  define SAM_TC3_BASE         0x40094000 /* 0x40094000-0x4009403f:   Timer Counter 3 */
#  define SAM_TC4_BASE         0x40094040 /* 0x40094040-0x4009407f:   Timer Counter 4 */
#  define SAM_TC5_BASE         0x40094080 /* 0x40094080-0x400940bf:   Timer Counter 5 */
                                          /* 0x400940c0-0x40097fff    Reserved */
#define SAM_TC678_BASE         0x40098000 /* 0x40098000-0x40097fff: Timer Counters 6-8 */
#  define SAM_TC6_BASE         0x40098000 /* 0x40098000-0x4009003f:   Timer Counter 6 */
#  define SAM_TC7_BASE         0x40098040 /* 0x40098040-0x4009007f:   Timer Counter 7 */
#  define SAM_TC8_BASE         0x40098080 /* 0x40098080-0x400900bf:   Timer Counter 8 */
                                          /* 0x4009c000-0x4009ffff: Reserved */
#define SAM_USART_BASE         0x400a0000 /* 0x400a0000-0x400abfff: USART */
#  define SAM_USART0_BASE      0x400a0000 /* 0x400a0000-0x400a3fff:   USART0 */
#  define SAM_USART1_BASE      0x400a4000 /* 0x400a4000-0x400abfff:   USART1 */
#define SAM_TWI_BASE           0x400a8000 /* 0x400a8000-0x400affff: Two-Wire Interface */
#  define SAM_TWI0_BASE        0x400a8000 /* 0x400a8000-0x400abfff:   Two-Wire Interface 0 */
#  define SAM_TWI1_BASE        0x400ac000 /* 0x400ac000-0x400affff:   Two-Wire Interface 1 */
#define SAM_AFEC_BASE          0x400b0000 /* 0x400b0000-0x400b7fff: Analog Front End */
#  define SAM_AFEC0_BASE       0x400b0000 /* 0x400b0000-0x400b3fff:   Analog Front End 0 */
#  define SAM_AFEC1_BASE       0x400b4000 /* 0x400b4000-0x400b7fff:   Analog Front End 1 */
#define SAM_DACC_BASE          0x400b8000 /* 0x400b8000-0x400bbfff: Digital To Analog Converter */
#define SAM_ACC_BASE           0x400bc000 /* 0x400bc000-0x400bffff: Analog Comparator */
#define SAM_DMAC_BASE          0x400c0000 /* 0x400c0000-0x400c3fff: DMA controller */
#define SAM_CMCC_BASE          0x400c4000 /* 0x400c4000-0x400c7fff: Cortex-M Cache Controller */
                                          /* 0x400c8000-0x400dffff: Reserved */
#define SAM_SYSCTRLR_BASE      0x400e0000 /* 0x400e0000-0x400e25ff: System Controller */
                                          /* 0x400e2600-0x5fffffff: Reserved */

#define SAM_BBPERIPH_BASE      0x42000000 /* 0x42000000-0x43ffffff: 32MB bit-band region */

/* System Controller Register Blocks:  0x400e0000-0x4007ffff */

                                          /* 0x400e0000-0x400e01ff: Reserved */
#define SAM_MATRIX_BASE        0x400e0200 /* 0x400e0200-0x400e03ff: MATRIX */
#define SAM_PMC_BASE           0x400e0400 /* 0x400e0400-0x400e05ff: Power Management Controller */
#define SAM_UART0_BASE         0x400e0600 /* 0x400e0600-0x400e073f: UART 0 */
#define SAM_CHIPID_BASE        0x400e0740 /* 0x400e0740-0x400e07ff: CHIP ID */
                                          /* 0x400e0800-0x400e09ff: Reserved */
#define SAM_EEFC_BASE          0x400e0a00 /* 0x400e0a00-0x400e0bff: Enhanced Embedded Flash Controller */
#  define SAM_EEFC0_BASE       0x400e0a00 /* 0x400e0a00-0x400e0bff: (For compatibility) */
                                          /* 0x400e0c00-0x400e0dff: Reserved */
#define SAM_PIO_BASE           0x400e0e00 /* 0x400e0e00-0x400e13ff: Parallel I/O Controllers */
#  define SAM_PION_BASE(n)     (0x400e0e00 + ((n) << 9))
#  define SAM_PIOA_BASE        0x400e0e00 /* 0x400e0e00-0x400e0fff:   Parallel I/O Controller A */
#  define SAM_PIOB_BASE        0x400e1000 /* 0x400e1000-0x400e11ff:   Parallel I/O Controller B */
#  define SAM_PIOC_BASE        0x400e1200 /* 0x400e1200-0x400e13ff:   Parallel I/O Controller C */
#  define SAM_PIOD_BASE        0x400e1400 /* 0x400e1400-0x400e15ff:   Parallel I/O Controller C */
#  define SAM_PIOE_BASE        0x400e1600 /* 0x400e1600-0x400e17ff:   Parallel I/O Controller C */
#define SAM_RSTC_BASE          0x400e1800 /* 0x400e1800-0x400e180f: Reset Controller */
#define SAM_SUPC_BASE          0x400e1810 /* 0x400e1810-0x400e182f: Supply Controller */
#define SAM_RTT_BASE           0x400e1830 /* 0x400e1830-0x400e184f: Real Time Timer */
#define SAM_WDT_BASE           0x400e1850 /* 0x400e1850-0x400e185f: Watchdog Timer */
#define SAM_RTC_BASE           0x400e1860 /* 0x400e1860-0x400e188f: Real Time Clock */
#define SAM_GPBR_BASE          0x400e1890 /* 0x400e1890-0x400e18ff: GPBR */
#define SAM_RSWDT_BASE         0x400e1900 /* 0x400e1900-??????????: Reinforced Safety Watchdog Timer */

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

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4E_MEMORYMAP_H */
