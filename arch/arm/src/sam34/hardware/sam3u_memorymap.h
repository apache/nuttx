/****************************************************************************
 * arch/arm/src/sam34/hardware/sam3u_memorymap.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM3U_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM3U_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define   SAM_CODE_BASE        0x00000000 /* 0x00000000-0x1fffffff: Code space */
#  define SAM_BOOTMEMORY_BASE  0x00000000 /* 0x00000000-0x0007ffff:   Boot Memory */
#  define SAM_INTFLASH0_BASE   0x00080000 /* 0x00080000-0x000fffff:   Internal FLASH 0 */
#  define SAM_INTFLASH1_BASE   0x00100000 /* 0x00100000-0x0017ffff:   Internal FLASH 1 */
#  define SAM_INTROM_BASE      0x00180000 /* 0x00180000-0x001fffff:   Internal ROM */
                                          /* 0x00200000-0x1fffffff:   Reserved */
#define   SAM_INTSRAM_BASE     0x20000000 /* 0x20000000-0x3fffffff: Internal SRAM */
#  define SAM_INTSRAM0_BASE    0x20000000 /* 0x20000000-0x2007ffff:   SRAM0 (see chip.h) */
#  define SAM_INTSRAM1_BASE    0x20080000 /* 0x20080000-0x200fffff:   SRAM1 (see chip.h) */
#  define SAM_NFCSRAM_BASE     0x20100000 /* 0x20100000-0x207fffff:   NAND FLASH controller (SRAM) */
#  define SAM_UDPHPSDMS_BASE   0x20180000 /* 0x20180000-0x201fffff:   USB Device High Speed (DMA) */
                                          /* 0x20200000-0x2fffffff:   Undefined */
#  define SAM_BBSRAM_BASE      0x22000000 /* 0x22000000-0x23ffffff:   32Mb bit-band alias */
                                          /* 0x24000000-0x3fffffff:   Undefined */
#define SAM_PERIPHERALS_BASE   0x40000000 /* 0x40000000-0x5fffffff: Peripherals */
#  define SAM_HSMCI_BASE       0x40000000 /* 0x40000000-0x400003ff:   High Speed Multimedia Card Interface */
#  define SAM_SSC_BASE         0x40004000 /* 0x40004000-0x40007fff:   Synchronous Serial Controller */
#  define SAM_SPI0_BASE        0x40008000 /* 0x40008000-0x4000bfff:   Serial Peripheral Interface */
                                          /* 0x4000c000-0x4007ffff:   Reserved */
#  define SAM_TC_BASE          0x40080000 /* 0x40080000-0x40083fff:   Timer Counters */
#    define SAM_TCN_BASE(n)    (0x40080000+((n)<<6))
#    define SAM_TC0_BASE       0x40080000 /* 0x40080000-0x4008003f:     Timer Counter 0 */
#    define SAM_TC1_BASE       0x40080040 /* 0x40080040-0x4008007f:     Timer Counter 1 */
#    define SAM_TC2_BASE       0x40080080 /* 0x40080080-0x400800bf:     Timer Counter 2 */
#  define SAM_TWI_BASE         0x40084000 /* 0x40084000-0x4008ffff:   Two-Wire Interface */
#    define SAM_TWIN_BASE(n)   (0x40084000+((n)<<14))
#    define SAM_TWI0_BASE      0x40084000 /* 0x40084000-0x40087fff:     Two-Wire Interface 0 */
#    define SAM_TWI1_BASE      0x40088000 /* 0x40088000-0x4008bfff:     Two-Wire Interface 1 */
#  define SAM_PWM_BASE         0x4008c000 /* 0x4008c000-0x4008ffff:   Pulse Width Modulation Controller */
#  define SAM_USART_BASE       0x40090000 /* 0x40090000-0x4009ffff:   USART */
#    define SAM_USARTN_BASE(n) (0x40090000+((n)<<14))
#    define SAM_USART0_BASE    0x40090000 /* 0x40090000-0x40093fff:     USART0 */
#    define SAM_USART1_BASE    0x40094000 /* 0x40094000-0x40097fff:     USART1 */
#    define SAM_USART2_BASE    0x40098000 /* 0x40098000-0x4009bfff:     USART2 */
#    define SAM_USART3_BASE    0x4009c000 /* 0x4009c000-0x4009ffff:     USART3 */
                                          /* 0x400a0000-0x400a3fff:   Reserved */
#  define SAM_UDPHS_BASE       0x400a4000 /* 0x400a4000-0x400a7fff:   USB Device High Speed */
#  define SAM_ADC12B_BASE      0x400a8000 /* 0x400a8000-0x400abfff:   12-bit ADC Controller */
#  define SAM_ADC_BASE         0x400ac000 /* 0x400ac000-0x400affff:   10-bit ADC Controller */
#  define SAM_DMAC_BASE        0x400b0000 /* 0x400b0000-0x400b3fff:   DMA controller */
                                          /* 0x400b4000-0x400dffff:   Reserved */
#  define SAM_SYSCTRLR_BASE    0x400e0000 /* 0x400e0000-0x400e25ff:   System controller */
                                          /* 0x400e2600-0x400fffff:   Reserved */
                                          /* 0x40100000-0x41ffffff:   Reserved */
#  define SAM_BBPERIPH_BASE    0x42000000 /* 0x42000000-0x43ffffff:   32Mb bit-band alias */
                                          /* 0x44000000-0x5fffffff:   Reserved */
#define   SAM_EXTSRAM_BASE     0x60000000 /* 0x60000000-0x9fffffff: External SRAM */
#  define SAM_EXTCS_BASE       0x60000000 /* 0x60000000-0x63ffffff:   Chip selects */
#    define SAM_EXTCSN_BASE(n) (0x60000000 + ((n) << 24))
#    define SAM_EXTCS0_BASE    0x60000000 /* 0x60000000-0x60ffffff:     Chip select 0 */
#    define SAM_EXTCS1_BASE    0x61000000 /* 0x61000000-0x601fffff:     Chip select 1 */
#    define SAM_EXTCS2_BASE    0x62000000 /* 0x62000000-0x62ffffff:     Chip select 2 */
#    define SAM_EXTCS3_BASE    0x63000000 /* 0x63000000-0x63ffffff:     Chip select 3 */
                                          /* 0x64000000-0x67ffffff:   Reserved */
#  define SAM_NFC_BASE         0x68000000 /* 0x68000000-0x68ffffff:   NAND FLASH controller */
                                          /* 0x69000000-0x9fffffff:   Reserved */
                                          /* 0xa0000000-0xdfffffff:   Reserved */
#define   SAM_SYSTEM_BASE      0xe0000000 /* 0xe0000000-0xffffffff: System */

/* System Controller Register Blocks:  0x400e0000-0x4007ffff */

#define SAM_SMC_BASE           0x400e0000 /* 0x400e0000-0x400e01ff: Static Memory Controller */
#define SAM_MATRIX_BASE        0x400e0200 /* 0x400e0200-0x400e03ff: MATRIX */
#define SAM_PMC_BASE           0x400e0400 /* 0x400e0400-0x400e05ff: Power Management Controller */
#define SAM_UART0_BASE         0x400e0600 /* 0x400e0600-0x400e073f: UART 0 */
#define SAM_CHIPID_BASE        0x400e0740 /* 0x400e0740-0x400e07ff: CHIP ID */
#define SAM_EEFC_BASE          0x400e0800 /* 0x400e0800-0x400e0bff: Enhanced Embedded Flash Controllers*/
#  define SAM_EEFCN_BASE(n)    (0x400e0800 + ((n) << 9))
#  define SAM_EEFC0_BASE       0x400e0800 /* 0x400e0800-0x400e09ff:   Enhanced Embedded Flash Controller 0 */
#  define SAM_EEFC1_BASE       0x400e0a00 /* 0x400e0a00-0x400e0bff:   Enhanced Embedded Flash Controller 1 */
#define SAM_PIO_BASE           0x400e0c00 /* 0x400e0c00-0x400e11ff: Parallel I/O Controllers */
#  define SAM_PION_BASE(n)     (0x400e0c00 + ((n) << 9))
#  define SAM_PIOA_BASE        0x400e0c00 /* 0x400e0c00-0x400e0dff:   Parallel I/O Controller A */
#  define SAM_PIOB_BASE        0x400e0e00 /* 0x400e0e00-0x400e0fff:   Parallel I/O Controller B */
#  define SAM_PIOC_BASE        0x400e1000 /* 0x400e1000-0x400e11ff:   Parallel I/O Controller C */
#define SAM_RSTC_BASE          0x400e1200 /* 0x400e1200-0x400e120f: Reset Controller */
#define SAM_SUPC_BASE          0x400e1210 /* 0x400e1210-0x400e122f: Supply Controller */
#define SAM_RTT_BASE           0x400e1230 /* 0x400e1230-0x400e124f: Real Time Timer */
#define SAM_WDT_BASE           0x400e1250 /* 0x400e1250-0x400e125f: Watchdog Timer */
#define SAM_RTC_BASE           0x400e1260 /* 0x400e1260-0x400e128f: Real Time Clock */
#define SAM_GPBR_BASE          0x400e1290 /* 0x400e1290-0x400e13ff: GPBR */
                                          /* 0x490e1400-0x4007ffff: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM3U_MEMORYMAP_H */
