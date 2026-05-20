/****************************************************************************
 * arch/arm64/src/am62x/hardware/am62x_memorymap.h
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

/* Reference: AM62x TRM (SPRSP43), Chapter 2 - Memory Map */

#ifndef __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_MEMORYMAP_H
#define __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Main Domain Peripheral Base Addresses (TRM §2.1) ************************/

/* UART (16550-compatible, main domain) */

#define AM62X_UART0_BASE        0x02800000ul  /* UART0 main domain console */
#define AM62X_UART1_BASE        0x02810000ul
#define AM62X_UART2_BASE        0x02820000ul
#define AM62X_UART3_BASE        0x02830000ul
#define AM62X_UART4_BASE        0x02840000ul
#define AM62X_UART5_BASE        0x02850000ul
#define AM62X_UART6_BASE        0x02860000ul

/* MCU Domain UART (used by R5 firmware, do not use from A53 without care) */

#define AM62X_MCU_UART0_BASE    0x04a00000ul

/* GIC-600 (TRM §9.2) */

#define AM62X_GIC_BASE          0x01800000ul  /* GIC top-level           */
#define AM62X_GICD_BASE         0x01800000ul  /* Distributor             */
#define AM62X_GICR_BASE         0x01880000ul  /* Redistributor (4 cores) */
#define AM62X_GICR_STRIDE       0x00020000ul  /* Per-core stride         */

/* Timers */

#define AM62X_DMTIMER0_BASE     0x2400000ul
#define AM62X_DMTIMER1_BASE     0x2410000ul
#define AM62X_DMTIMER2_BASE     0x2420000ul
#define AM62X_DMTIMER3_BASE     0x2430000ul

/* GPIO */

#define AM62X_GPIO0_BASE        0x00600000ul
#define AM62X_GPIO1_BASE        0x00601000ul

/* I2C */

#define AM62X_I2C0_BASE         0x20000000ul
#define AM62X_I2C1_BASE         0x20010000ul
#define AM62X_I2C2_BASE         0x20020000ul
#define AM62X_I2C3_BASE         0x20030000ul

/* SPI (McSPI) */

#define AM62X_SPI0_BASE         0x20100000ul
#define AM62X_SPI1_BASE         0x20110000ul
#define AM62X_SPI2_BASE         0x20120000ul

/* USB */

#define AM62X_USB0_BASE         0x31100000ul
#define AM62X_USB1_BASE         0x31200000ul

/* MMC/SD */

#define AM62X_MMCSD0_BASE       0xfa10000ul
#define AM62X_MMCSD1_BASE       0xfa00000ul
#define AM62X_MMCSD2_BASE       0xfa20000ul

/* Watchdog */

#define AM62X_WDT0_BASE         0x23100000ul
#define AM62X_WDT1_BASE         0x23110000ul

/* CTRL_MMR (Pad config / system control) */

#define AM62X_CTRLMMR_BASE      0x000f0000ul
#define AM62X_PADCFG_BASE       0x000f0000ul

/* DDR Base (512 MB on PocketBeagle 2 / BeaglePlay) */

#define AM62X_DDR_BASE          0x80000000ul
#define AM62X_DDR_SIZE          0x20000000ul  /* 512 MB */

/* Device I/O region for MMU flat mapping */

#define AM62X_DEVICEIO_BASE     0x00000000ul
#define AM62X_DEVICEIO_SIZE     0x80000000ul  /* Peripheral space below DDR */

#endif /* __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_MEMORYMAP_H */
