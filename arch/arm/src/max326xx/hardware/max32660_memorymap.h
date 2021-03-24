/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_memorymap.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_MEMORYMAP_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Memory Map */

#define MAX326_CODE_BASE       0x00000000 /* Code address space (not cached) */
#define MAX326_SRAM_BASE       0x20000000 /* SRAM address space (not cached) */
#define MAX326_PERIPH_BASE     0x40000000 /* Peripherals memory space (not cached) */
#  define MAX326_APB_BASE      0x40000000 /* AHB-to-APB Bridge */
#define MAX326_SYSTEM_BASE     0xe0000000 /* System address space */
#  define MAX326_SCS_BASE      0xe000e000 /* SCSS */

/* AHB-to-APB Bridge */

#define MAX326_GCR_BASE        0x40000000 /* Global Control Registers */
#define MAX326_SIR_BASE        0x40000400 /* SI Reisters */
#define MAX326_FCR_BASE        0x40000800 /* Function Control Registers */
#define MAX326_WDT0_BASE       0x40003000 /* Watchdog Timer 0 */
#define MAX326_RTC_BASE        0x40006000 /* RTC */
#define MAX326_PWRSEQ_BASE     0x40006800 /* Power Sequencer */
#define MAX326_GPIO_BASE       0x40008000 /* GPIO Port 0 */
#define MAX326_TMR0_BASE       0x40010000 /* TMR0 */
#define MAX326_TMR1_BASE       0x40011000 /* TMR1 */
#define MAX326_TMR2_BASE       0x40012000 /* TMR2 */
#define MAX326_SPIMSS_BASE     0x40019000 /* SPIMSS (I2S) */
#define MAX326_I2C0_BASE       0x4001d000 /* I2CM 0 Master/Slave */
#define MAX326_I2C1_BASE       0x4001e000 /* I2CM 1 Master/Slave */
#define MAX326_DMA_BASE        0x40028000 /* Standard DMA */
#define MAX326_FLC_BASE        0x40029000 /* FLASH Controller */
#define MAX326_ICC_BASE        0x4002a000 /* Internal Cache Controller */
#define MAX326_UART0_BASE      0x40042000 /* UART 0 */
#define MAX326_UART1_BASE      0x40043000 /* UART 1 */
#define MAX326_SPI0_BASE       0x40046000 /* SPIM 0 Master/Slave */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_MEMORYMAP_H */
