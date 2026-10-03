/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mzw1_memorymap.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* The PIC32MZ-W1 / WFI32Exx family (used in the WFI32E01 modules) shares
 * the MIPS32 M-Class core and much of the peripheral IP found in the
 * PIC32MZ-EC/EF families, but the SFR layout was substantially reorganized
 * to make room for the integrated Wi-Fi subsystem, crypto engine, and RNG.
 * Do not assume offsets from pic32mzef_memorymap.h apply here.
 *
 * Offsets below were derived from the Microchip PIC32MZ-W_DFP device
 * family pack (Apache-2.0 licensed), specifically the WFI32E01 processor
 * header (proc/pwfi32e01.h) and WFI32E01.atdf, cross-checked against the
 * PIC32MZ W1 family datasheet.
 */

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_MEMORYMAP_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "mips32-memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Physical Memory Map ******************************************************/

/* Memory Regions */

#define PIC32MZ_DATAMEM_PBASE     0x00000000 /* Size depends on CHIP_DATAMEM_KB */
#define PIC32MZ_PROGFLASH_PBASE   0x10000000 /* Size depends on CHIP_PROGFLASH_KB -
                                               * NOTE: this differs from EC/EF's
                                               * 0x1d000000; verified against both
                                               * the DFP's ATDF memory-segment list
                                               * and Microchip's own WFI32E01
                                               * reference linker script
                                               * (kseg0_program_mem at KSEG0
                                               * 0x90000000). */
#define PIC32MZ_SFR_PBASE         0x1f800000 /* Special function registers */
#define PIC32MZ_BOOTFLASH_PBASE   0x1fc00000 /* Size depends on CHIP_BOOTFLASH_KB */
#define PIC32MZ_SQIMEM_PBASE      0x30000000 /* External memory via SQI (e.g. SPI flash) */

/* Boot FLASH */

#define PIC32MZ_LOWERBOOT_PBASE   0x1fc00000 /* Lower boot alias */
#define PIC32MZ_BOOTCFG_PBASE     0x1fc55e88 /* Configuration space (altConfig) */

/* Virtual Memory Map *******************************************************/

#define PIC32MZ_DATAMEM_K0BASE      (KSEG0_BASE + PIC32MZ_DATAMEM_PBASE)
#define PIC32MZ_PROGFLASH_K0BASE    (KSEG0_BASE + PIC32MZ_PROGFLASH_PBASE)
#define PIC32MZ_SFR_K0BASE          (KSEG0_BASE + PIC32MZ_SFR_PBASE)
#define PIC32MZ_BOOTFLASH_K0BASE    (KSEG0_BASE + PIC32MZ_BOOTFLASH_PBASE)
#define PIC32MZ_SQIMEM_K0BASE       (KSEG0_BASE + PIC32MZ_SQIMEM_PBASE)

#define PIC32MZ_DATAMEM_K1BASE      (KSEG1_BASE + PIC32MZ_DATAMEM_PBASE)
#define PIC32MZ_PROGFLASH_K1BASE    (KSEG1_BASE + PIC32MZ_PROGFLASH_PBASE)
#define PIC32MZ_SFR_K1BASE          (KSEG1_BASE + PIC32MZ_SFR_PBASE)
#define PIC32MZ_BOOTFLASH_K1BASE    (KSEG1_BASE + PIC32MZ_BOOTFLASH_PBASE)
#define PIC32MZ_SQIMEM_K1BASE       (KSEG1_BASE + PIC32MZ_SQIMEM_PBASE)

#define PIC32MZ_LOWERBOOT_K0BASE    (KSEG0_BASE + PIC32MZ_LOWERBOOT_PBASE)
#define PIC32MZ_BOOTCFG_K0BASE      (KSEG0_BASE + PIC32MZ_BOOTCFG_PBASE)
#define PIC32MZ_LOWERBOOT_K1BASE    (KSEG1_BASE + PIC32MZ_LOWERBOOT_PBASE)
#define PIC32MZ_BOOTCFG_K1BASE      (KSEG1_BASE + PIC32MZ_BOOTCFG_PBASE)

/* Register Base Addresses **************************************************
 *
 * Unlike the EC/EF families, PIC32MZ-W1 peripherals are not laid out in
 * one contiguous, cleanly-spaced table.  The blocks below are grouped by
 * the "near"/"far" SFR regions used by the silicon (periph_pb1..pb4,
 * periph_indep_a/b in the ATDF), each holding a handful of unrelated
 * peripherals.
 */

/* "Near" SFR region: system config, oscillator, timers, interrupt
 * controller, DMA, prefetch cache.
 */

#define PIC32MZ_CONFIG_K1BASE       (PIC32MZ_SFR_K1BASE + 0x00000000) /* CFGCONx, SYSKEY, PMDx */
#define PIC32MZ_WDT_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00000800) /* Watchdog Timer */
#define PIC32MZ_DMT_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00000a00) /* Deadman Timer */
#define PIC32MZ_OSC_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00001200) /* Oscillator, RCON, RSWRST */
#define PIC32MZ_UART3_REAL_K1BASE   (PIC32MZ_SFR_K1BASE + 0x00001600) /* UART3 (low-power/debug UART).
                                               * NOTE: named _REAL_ to avoid
                                               * colliding with (and being
                                               * silently shadowed by) the
                                               * generic, WRONG
                                               * PIC32MZ_UART3_K1BASE that
                                               * hardware/pic32mz_uart.h
                                               * computes generically as
                                               * UART_K1BASE+0x400 - see
                                               * note above. Not wired up
                                               * to any driver yet. */
#define PIC32MZ_PPS_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00001800) /* Peripheral Pin Select */
#define PIC32MZ_TIMER_K1BASE        (PIC32MZ_SFR_K1BASE + 0x00002000) /* Timer1-Timer7 (0x200 each) */

#define PIC32MZ_INT_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00010000) /* Interrupt Controller */
#define PIC32MZ_DMA_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00011000) /* DMA */
#define PIC32MZ_PREFETCH_K1BASE     (PIC32MZ_SFR_K1BASE + 0x00012400) /* Prefetch cache controller */

/* periph_pb2: GPIO ports, ADC, I2C1, CAN1, CAN-FD aux, CVD
 *
 * NOTE: I2C1 and I2C2 are NOT a contiguous pair on this family (I2C1
 * lives here in pb2, I2C2 lives in pb3 below), unlike EC/EF where
 * PIC32MZ_I2C_K1BASE + n*0x200 reaches every instance.  There is no
 * PIC32MZ_I2C_K1BASE here; hardware/pic32mz_i2c.h uses the per-instance
 * bases below.
 */

#define PIC32MZ_IOPORT_K1BASE       (PIC32MZ_SFR_K1BASE + 0x00020000) /* PORTA-C, PORTK (0x100 each) */
#define PIC32MZ_I2C1_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00020400) /* I2C1 */
#define PIC32MZ_ADC1_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00021000) /* ADC1 */
#define PIC32MZ_CAN1_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00022000) /* CAN1 */
#define PIC32MZ_CFD2_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00023000) /* CAN-FD auxiliary block */
#define PIC32MZ_CVD_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00024000) /* Capacitive voltage divider */

/* periph_pb3: I2C2, UART1, UART2, SPI1, SPI2, input capture, output
 * compare, Ethernet MAC, USB OTG.
 *
 * UART1/UART2 and SPI1/SPI2 ARE contiguous 0x200-apart pairs here, so
 * PIC32MZ_UART_K1BASE/PIC32MZ_SPI_K1BASE (base of instance 1) work with
 * the existing generic hardware/pic32mz_uart.h and pic32mz_spi.h offset
 * tables for instances 1-2.  UART3 (see PIC32MZ_UART3_K1BASE above) is a
 * separate, non-contiguous low-power UART and is NOT reachable through
 * that generic table.
 */

#define PIC32MZ_I2C2_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00040400) /* I2C2 */
#define PIC32MZ_UART_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00040600) /* UART1 (base), UART2 at +0x200 */
#define PIC32MZ_SPI_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00040c00) /* SPI1 (base), SPI2 at +0x200 */
#define PIC32MZ_IC_K1BASE           (PIC32MZ_SFR_K1BASE + 0x00041000) /* IC1-IC9 */
#define PIC32MZ_OC_K1BASE           (PIC32MZ_SFR_K1BASE + 0x00042000) /* OC1-OC9 */
#define PIC32MZ_ETH_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00043000) /* Ethernet MAC */
#define PIC32MZ_USB_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00044000) /* USB OTG */

/* periph_pb4: RTCC */

#define PIC32MZ_RTCC_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00070000) /* RTCC */

/* periph_indep_a: SQI (external QSPI flash), crypto, RNG */

#define PIC32MZ_SQI1_K1BASE         (PIC32MZ_SFR_K1BASE + 0x000e1000) /* SQI1 controller */
#define PIC32MZ_CRYPTO_K1BASE       (PIC32MZ_SFR_K1BASE + 0x000e4000) /* Crypto engine */
#define PIC32MZ_RNG_K1BASE          (PIC32MZ_SFR_K1BASE + 0x000e5000) /* RNG */

/* periph_indep_b: public-key crypto accelerator */

#define PIC32MZ_PKE_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00120000) /* Public key engine */

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_MEMORYMAP_H */
