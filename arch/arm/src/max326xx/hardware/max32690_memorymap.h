/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32690_memorymap.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32690_MEMORYMAP_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32690_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Memory Map */

#define MAX326_CODE_BASE         0x00000000 /* Code address space (not cached) */
#define MAX326_SRAM_BASE         0x20000000 /* SRAM address space (not cached) */
#define MAX326_PERIPH_BASE       0x40000000 /* Peripherals memory space (not cached) */
#define MAX326_APB_BASE          0x40000000 /* AHB-to-APB Bridge */
#define MAX326_SYSTEM_BASE       0xe0000000 /* System address space */
#define MAX326_SCS_BASE          0xe000e000 /* SCSS */

/* AHB-to-APB Bridge */

#define MAX326_GCR_BASE          0x40000000 /* Global Control Registers */
#define MAX326_SIR_BASE          0x40000400 /* SI Reisters */
#define MAX326_FCR_BASE          0x40000800 /* Function Control Registers */
#define MAX326_CRYPTO_BASE       0x40001000 /* Crypto Toolbox */
#define MAX326_WDT0_BASE         0x40003000 /* Watchdog Timer 0 */
#define MAX326_MP_KEY_BASE       0x40005000 /* Memory Protection Key */
#define MAX326_TRIM_BASE         0x40005400 /* Trim System Initialization */
#define MAX326_GC_BASE           0x40005800 /* General Control Function */
#define MAX326_RTC_BASE          0x40006000 /* RTC */
#define MAX326_WAKE_TMR0         0x40006400 /* Wakeup Timer 0 */
#define MAX326_WAKE_TMR1         0x40006600 /* Wakeup Timer 1 */
#define MAX326_PWRSEQ_BASE       0x40006800 /* Power Sequencer */
#define MAX326_MC_BASE           0x40006C00 /* Miscellaneous Control */
#define MAX326_PUF_BASE          0x40007000 /* PUF */
#define MAX326_GPIO0_BASE        0x40008000 /* GPIO Port 0 */
#define MAX326_GPIO1_BASE        0x40009000 /* GPIO Port 1 */
#define MAX326_GPIO2_BASE        0x4000A000 /* GPIO Port 2 */
#define MAX326_TMR0_BASE         0x40010000 /* TMR0 */
#define MAX326_TMR1_BASE         0x40011000 /* TMR1 */
#define MAX326_TMR2_BASE         0x40012000 /* TMR2 */
#define MAX326_TMR3_BASE         0x40013000 /* TMR3 */
#define MAX326_I2C0_BASE         0x4001D000 /* I2CM 0 Master/Slave */
#define MAX326_I2C1_BASE         0x4001E000 /* I2CM 1 Master/Slave */
#define MAX326_I2C2_BASE         0x4001F000 /* I2CM 2 Master/Slave */
#define MAX326_SPIXFM            0x40026000 /* SPIXF Controller */
#define MAX326_SPIXFC            0x40027000 /* SPIXF Host Controller */
#define MAX326_DMA_BASE          0x40028000 /* Standard DMA */
#define MAX326_FLC0_BASE         0x40029000 /* FLASH Controller 0 */
#define MAX326_FLC1_BASE         0x40029400 /* FLASH Controller 1 */
#define MAX326_ICC0_BASE         0x4002a000 /* Instruction Cache Controller 0 ( CM4 ) */
#define MAX326_ICC1_BASE         0x4002a800 /* Instruction Cache Controller 1 ( RV32 ) */
#define MAX326_SFCC_BASE         0x4002F000 /* SPIXF Cache Controller */
#define MAX326_EMCC_BASE         0x40033000 /* External Memory Cache Controller */
#define MAX326_ADC_BASE          0x40034000 /* Analog-to-Digital Converter */
#define MAX326_HPB_BASE          0x40039000 /* HyperBus/Xccela */
#define MAX326_SPIXR_BASE        0x4003A000 /* SPIXR Host Controller */
#define MAX326_PT_BASE           0x4003C000 /* Pulse Train Engine */
#define MAX326_OWM_BASE          0x4003D000 /* 1-Wire Controller */
#define MAX326_SEMA_BASE         0x4003E000 /* Semaphore */
#define MAX326_UART0_BASE        0x40042000 /* UART 0 */
#define MAX326_UART1_BASE        0x40043000 /* UART 1 */
#define MAX326_UART2_BASE        0x40044000 /* UART 2 */
#define MAX326_SPI0_BASE         0x40046000 /* SPIM 0 Master/Slave */
#define MAX326_SPI1_BASE         0x40047000 /* SPIM 1 Master/Slave */
#define MAX326_SPI2_BASE         0x40048000 /* SPIM 2 Master/Slave */
#define MAX326_TRNG_BASE         0x4004D000 /* TRNG */
#define MAX326_I2S_BASE          0x40060000 /* I2S */
#define MAX326_CAN0_BASE         0x40064000 /* CAN0 */
#define MAX326_CAN1_BASE         0x40065000 /* CAN1 */
#define MAX326_LPGCR_BASE        0x40080000 /* Low-Power General Control */
#define MAX326_GPIO3_BASE        0x40080400 /* Low-Power GPIO Port 3 (LPGPIO0) */
#define MAX326_WDT1_BASE         0x40080800 /* Low-Power Watchdog Timer 1 (LPWDT0) */
#define MAX326_TMR4_BASE         0x40080C00 /* Low-Power Timer 4 (LPTMR0) */
#define MAX326_TMR5_BASE         0x40081000 /* Low-Power Timer 5 (LPTMR1) */
#define MAX326_UART3_BASE        0x40081400 /* Low-Power UART 3 (LPUART0) */
#define MAX326_LPCMP0_BASE       0x40088000 /* Low-Power Comparator */
#define MAX326_USBHS_BASE        0x400B1000 /* USB Hi-Speed Host */
#define MAX326_SPIXFC_FIFO_BASE  0x400BC000 /* SPIXF Host Controller FIFO */
#define MAX326_SPI3_BASE         0x400BE000 /* SPIM 3 Master/Slave */
#define MAX326_SPI4_BASE         0x400BE400 /* SPIM 4 Master/Slave */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32690_MEMORYMAP_H */
