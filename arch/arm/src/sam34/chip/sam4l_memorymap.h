/************************************************************************************************
 * arch/arm/src/sam34/chip/sam4l_memorymap.h
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_MEMORYMAP_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Global Memory Space */

#define SAM_CODE_BASE        0x00000000 /* 0x00000000-0x1fffffff: Code space */
#define SAM_INTSRAM_BASE     0x20000000 /* 0x20000000-0x3fffffff: Internal SRAM */
                                        /* 0x22000000-0x3fffffff: Undefined */
#define SAM_PERIPHERALS_BASE 0x40000000 /* 0x40000000-0x5fffffff: Peripherals */
                                        /* 0x60000000-0xdfffffff: Reserved */
#define SAM_SYSTEM_BASE      0xe0000000 /* 0xe0000000-0xffffffff: System */

/* Code Space */

#define SAM_INTFLASH_BASE    0x00000000 /* 0x00000000-0x003fffff: Internal FLASH */
                                        /* 0x00400000-0x1fffffff: Reserved */
/* Internal SRAM Space */

#define SAM_INTSRAM0_BASE    0x20000000 /* 0x20000000-0x2007ffff: HRAMC0 (see chip.h) */
                                        /* 0x20008000-0x20ffffff: Reserved */
#define SAM_INTSRAM1_BASE    0x21000000 /* 0x21000000-0x210007ff: HRAMC1 (see chip.h) */
                                        /* 0x21000800-0x21ffffff: Reserved */
/* Peripherals Space */

#define SAM_PERIPHA_BASE     0x40000000 /* 0x40000000-0x4009ffff: Peripheral Bridge A */
#define SAM_PERIPHB_BASE     0x400a0000 /* 0x400a0000-0x400affff: Peripheral Bridge B */
#define SAM_AESA_BASE        0x400b0000 /* 0x400b0000-0x400b00ff: AESA */
                                        /* 0x400b0100-0x400dffff: Reserved */
#define SAM_PERIPHC_BASE     0x400e0000 /* 0x400e0000-0x400effff: Peripheral Bridge C */
#define SAM_PERIPHD_BASE     0x400e0000 /* 0x400f0000-0x400fffff: Peripheral Bridge D */
                                        /* 0x40100000-0x5fffffff: Reserved */
/* Peripheral Bridge A */
                                        /* 0x40000000-0x40003fff: Reserved */
#define SAM_I2SC_BASE        0x40004000 /* 0x40004000-0x40007fff: I2S Controller */
#define SAM_SPI0_BASE        0x40008000 /* 0x40008000-0x4000bfff: Serial Peripheral Interface */
                                        /* 0x4000c000-0x4000ffff: Reserved */
#define SAM_TC0_BASE         0x40100000 /* 0x40100000-0x4013ffff: Timer Counter 0 */
#define SAM_TC1_BASE         0x40140000 /* 0x40180000-0x4017ffff: Timer Counter 1 */

#define SAM_TWIMS_BASE       0x40180000 /* 0x40180000-0x401fffff: Two-wire Master/Slave */
#define SAM_TWIN_BASE(n)     (SAM_TWIMS_BASE + ((n) << 14))
#define SAM_TWIMS0_BASE      0x40180000 /* 0x40180000-0x401bffff: Two-wire Master/Slave Interface 0 */
#define SAM_TWIMS1_BASE      0x401c0000 /* 0x401c0000-0x401fffff: Two-wire Master/Slave Interface 1 */
                                        /* 0x40020000-0x40023fff: Reserved */
#define SAM_USARTN_BASE(n)   (0x40024000+((n)<<14))
#define SAM_USART0_BASE      0x40024000 /* 0x40024000-0x40027fff: USART0 */
#define SAM_USART1_BASE      0x40028000 /* 0x40028000-0x4002bfff: USART1 */
#define SAM_USART2_BASE      0x4002c000 /* 0x4002c000-0x4002ffff: USART2 */
#define SAM_USART3_BASE      0x40030000 /* 0x40030000-0x40033fff: USART3 */
                                        /* 0x40034000-0x40037fff: Reserved */
#define SAM_ADCIFE_BASE      0x40038000 /* 0x40038000-0x4003bfff: ADC controller interface */
#define SAM_DACC_BASE        0x4003c000 /* 0x4003c000-0x4003ffff: DAC Controller */
#define SAM_ACIF_BASE        0x40040000 /* 0x40040000-0x40043fff: Analog Comparator Interface */
                                        /* 0x40044000-0x4005ffff: Reserved */
#define SAM_GLOC_BASE        0x40040000 /* 0x40060000-0x40063fff: GLOC */
#define SAM_ABDACB_BASE      0x40040000 /* 0x40064000-0x40067fff: Audio Bitstream DAC */
#define SAM_TRNG_BASE        0x40068000 /* 0x40064000-0x4006bfff: True Random Number Generator */
#define SAM_PARC_BASE        0x4006c000 /* 0x4006c000-0x4006ffff: Parallel Capture */
#define SAM_CATB_BASE        0x40070000 /* 0x4006c000-0x40073fff: Capacitive Touch Module B */
                                        /* 0x40074000-0x40077fff: Reserved */
#define SAM_TWIM2_BASE       0x40078000 /* 0x40078000-0x4007bfff: Two-wire Master Interface 2 */
#define SAM_TWIM3_BASE       0x4007c000 /* 0x4007c000-0x4007ffff: Two-wire Master Interface 3 */
#define SAM_LCDCA_BASE       0x40080000 /* 0x40080000-0x40083fff: LCD Controller A */
                                        /* 0x40084000-0x4009ffff: Reserved */
/* Peripheral Bridge B */

#define SAM_FLASHCALW_BASE   0x400a0000 /* 0x400a0000-0x400a03ff: FLASHCALW */
#define SAM_PICOCACHE_BASE   0x400a0400 /* 0x400a0400-0x400a0fff: PICOCACHE */
#define SAM_HMATRIX_BASE     0x400a1000 /* 0x400a1000-0x400a1fff: HMATRIX */
#define SAM_PDCA_BASE        0x400a2000 /* 0x400a2000-0x400a2fff: Peripheral DMA Controller */
#define SAM_SMAP_BASE        0x400a3000 /* 0x400a3000-0x400a3fff: SMAP */
#define SAM_CRCCU_BASE       0x400a4000 /* 0x400a4000-0x400a4fff: CRC Calculation Unit */
#define SAM_USBC_BASE        0x400a5000 /* 0x400a5000-0x400a5fff: USB 2.0 Interface */
#define SAM_PEVC_BASE        0x400a6000 /* 0x400a6000-0x400a63ff: Peripheral Event Controller */
                                        /* 0x400a6400-0x400affff: Reserved */
/* Peripheral Bridge C */

#define SAM_PM_BASE          0x400e0000 /* 0x400e0000-0x400e073f: Power Manager */
#define SAM_CHIPID_BASE      0x400e0740 /* 0x400e0740-0x400e07ff: CHIPID */
#define SAM_SCIF_BASE        0x400e0800 /* 0x400e0800-0x400e0bff: System Control Interface  */
#define SAM_FREQM_BASE       0x400e0c00 /* 0x400e0c00-0x400e0fff: Frequency Meter */
#define SAM_GPIO_BASE        0x400e1000 /* 0x400e1000-0x400e17ff: GPIO */
                                        /* 0x400e1800-0x400effff: Reserved */
/* Peripheral Bridge D */

#define SAM_BPM_BASE         0x400f0000 /* 0x400f0000-0x400f03ff: Backup Power Manager */
#define SAM_BSCIF_BASE       0x400f0400 /* 0x400f0400-0x400f07ff: Backup System Control Interface */
#define SAM_AST_BASE         0x400f0800 /* 0x400f0800-0x400f0bff: Asynchronous Timer */
#define SAM_WDT_BASE         0x400f0c00 /* 0x400f0c00-0x400f0fff: Watchdog Timer */
#define SAM_EIC_BASE         0x400f1000 /* 0x400f1000-0x400f13ff: External Interrupt Controller */
#define SAM_PICOUART_BASE    0x400f1400 /* 0x400f1400-0x400f17ff: PICOUART */
                                        /* 0x400f1800-0x400fffff: Reserved */
/* System Space */

#define SAM_ITM_BASE         0xe0000000 /* 0xe0000000-0xe0000fff: ITM */
#define SAM_DWT_BASE         0xe0001000 /* 0xe0001000-0xe0001fff: DWT */
#define SAM_FPB_BASE         0xe0002000 /* 0xe0002000-0xe0002fff: FPB */
                                        /* 0xe0003000-0xe000dfff: Reserved */
#define SAM_SCS_BASE         0xe000e000 /* 0xe000e000-0xe000efff: SCS */
                                        /* 0xe000f000-0xe003ffff: Reserved */
#define SAM_TPIU_BASE        0xe0040000 /* 0xe0040000-0xe0040fff: TPIU */
                                        /* 0xe0041000-0xe0041fff: Reserved */
#define SAM_EXTPPB_BASE      0xe0042000 /* 0xe0042000-0xe00fefff: External PPB */
#define SAM_ROMTAB_BASE      0xe00ff000 /* 0xe00ff000-0xe00fffff: ROM Table */
                                        /* 0xe0100000-0xffffffff: Reserved */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_MEMORYMAP_H */
