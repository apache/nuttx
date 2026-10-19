/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_memorymap.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_GD32W51X_MEMORYMAP_H
#define __ARCH_ARM_SRC_GD32W51X_GD32W51X_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32W51x has TrustZone.  Every memory and peripheral is visible
 * through a non-secure alias and through a secure alias (bit 28 set:
 * 0x0c000000 flash, 0x30000000 SRAM, 0x5xxxxxxx peripherals).
 *
 * NuttX runs in secure state with the SAU disabled, and uses the
 * non-secure aliases below for everything: gd32w51x_tzinit() releases the
 * GPIO pins and the bus masters to the non-secure world at boot, the
 * peripherals already are non-secure out of reset.  That is the same view
 * as a part with TrustZone disabled, and the one the prebuilt Wi-Fi
 * libraries are built for.
 */

/* Memories */

#define GD32W51X_FLASH_BASE      0x08000000  /* SIP flash (1/2 MB) */
#define GD32W51X_FLASH_S_BASE    0x0c000000  /* SIP flash, secure alias */
#define GD32W51X_SRAM_BASE       0x20000000  /* SRAM0..3 (384/448 KB) */
#define GD32W51X_SRAM0_BASE      0x20000000  /*  64 KB */
#define GD32W51X_SRAM1_BASE      0x20010000  /*  64 KB */
#define GD32W51X_SRAM2_BASE      0x20020000  /* 128 KB */
#define GD32W51X_SRAM3_BASE      0x20040000  /* 128/192 KB */
#define GD32W51X_PSRAM_BASE      0x60000000  /* SQPI PSRAM window */
#define GD32W51X_QSPIMEM_BASE    0x90000000  /* QSPI flash XIP window */

/* Bus bases */

#define GD32W51X_APB1_BASE       0x40000000
#define GD32W51X_APB2_BASE       0x40010000
#define GD32W51X_AHB1_BASE       0x40020000
#define GD32W51X_AHB2_BASE       0x4c000000

/* APB1 peripherals */

#define GD32W51X_TIMER1_BASE     (GD32W51X_APB1_BASE + 0x0000)
#define GD32W51X_TIMER2_BASE     (GD32W51X_APB1_BASE + 0x0400)
#define GD32W51X_TIMER3_BASE     (GD32W51X_APB1_BASE + 0x0800)
#define GD32W51X_TIMER4_BASE     (GD32W51X_APB1_BASE + 0x0c00)
#define GD32W51X_TIMER5_BASE     (GD32W51X_APB1_BASE + 0x1000)
#define GD32W51X_RTC_BASE        (GD32W51X_APB1_BASE + 0x2800)
#define GD32W51X_WWDGT_BASE      (GD32W51X_APB1_BASE + 0x2c00)
#define GD32W51X_FWDGT_BASE      (GD32W51X_APB1_BASE + 0x3000)
#define GD32W51X_I2S1ADD_BASE    (GD32W51X_APB1_BASE + 0x3400)
#define GD32W51X_SPI1_BASE       (GD32W51X_APB1_BASE + 0x3800)
#define GD32W51X_UART1_BASE      (GD32W51X_APB1_BASE + 0x4400)  /* USART1 */
#define GD32W51X_USART0_BASE     (GD32W51X_APB1_BASE + 0x4800)
#define GD32W51X_I2C0_BASE       (GD32W51X_APB1_BASE + 0x5400)
#define GD32W51X_I2C1_BASE       (GD32W51X_APB1_BASE + 0x5800)
#define GD32W51X_PMU_BASE        (GD32W51X_APB1_BASE + 0x7000)

/* APB2 peripherals */

#define GD32W51X_TIMER0_BASE     (GD32W51X_APB2_BASE + 0x0000)
#define GD32W51X_UART2_BASE      (GD32W51X_APB2_BASE + 0x1000)  /* USART2 */
#define GD32W51X_ADC_BASE        (GD32W51X_APB2_BASE + 0x2000)
#define GD32W51X_SDIO_BASE       (GD32W51X_APB2_BASE + 0x2c00)
#define GD32W51X_SPI_BASE        (GD32W51X_APB2_BASE + 0x3000)  /* SPI0 */
#define GD32W51X_SYSCFG_BASE     (GD32W51X_APB2_BASE + 0x3800)
#define GD32W51X_EXTI_BASE       (GD32W51X_APB2_BASE + 0x3c00)
#define GD32W51X_HPDF_BASE       (GD32W51X_APB2_BASE + 0x6000)
#define GD32W51X_WIFIRF_BASE     (GD32W51X_APB2_BASE + 0x7800)
#define GD32W51X_TIMER15_BASE    (GD32W51X_APB2_BASE + 0x8000)
#define GD32W51X_TIMER16_BASE    (GD32W51X_APB2_BASE + 0x8400)

/* AHB1 peripherals */

#define GD32W51X_GPIOA_BASE      (GD32W51X_AHB1_BASE + 0x0000)
#define GD32W51X_GPIOB_BASE      (GD32W51X_AHB1_BASE + 0x0400)
#define GD32W51X_GPIOC_BASE      (GD32W51X_AHB1_BASE + 0x0800)
#define GD32W51X_FMC_BASE        (GD32W51X_AHB1_BASE + 0x2000)
#define GD32W51X_EFUSE_BASE      (GD32W51X_AHB1_BASE + 0x2800)
#define GD32W51X_CRC_BASE        (GD32W51X_AHB1_BASE + 0x3000)
#define GD32W51X_RCU_BASE        (GD32W51X_AHB1_BASE + 0x3800)
#define GD32W51X_TSI_BASE        (GD32W51X_AHB1_BASE + 0x4000)
#define GD32W51X_SQPI_BASE       (GD32W51X_AHB1_BASE + 0x5400)
#define GD32W51X_QSPI_BASE       (GD32W51X_AHB1_BASE + 0x5800)
#define GD32W51X_DMA_BASE        (GD32W51X_AHB1_BASE + 0x6000)  /* DMA0 */
#define GD32W51X_DMA1_BASE       (GD32W51X_AHB1_BASE + 0x6400)
#define GD32W51X_WIFI_BASE       (GD32W51X_AHB1_BASE + 0x10000)
#define GD32W51X_ICACHE_BASE     (GD32W51X_AHB1_BASE + 0x60000)
#define GD32W51X_TZSPC_BASE      (GD32W51X_AHB1_BASE + 0x80000)
#define GD32W51X_TZIAC_BASE      (GD32W51X_AHB1_BASE + 0x80400)
#define GD32W51X_TZBMPC0_BASE    (GD32W51X_AHB1_BASE + 0x80800)
#define GD32W51X_TZBMPC1_BASE    (GD32W51X_AHB1_BASE + 0x80c00)
#define GD32W51X_TZBMPC2_BASE    (GD32W51X_AHB1_BASE + 0x90000)
#define GD32W51X_TZBMPC3_BASE    (GD32W51X_AHB1_BASE + 0x90400)
#define GD32W51X_USBFS_BASE      (GD32W51X_AHB1_BASE + 0x8fe0000)

/* AHB2 peripherals */

#define GD32W51X_DCI_BASE        (GD32W51X_AHB2_BASE + 0x50000)
#define GD32W51X_CAU_BASE        (GD32W51X_AHB2_BASE + 0x60000)
#define GD32W51X_HAU_BASE        (GD32W51X_AHB2_BASE + 0x60400)
#define GD32W51X_TRNG_BASE       (GD32W51X_AHB2_BASE + 0x60800)
#define GD32W51X_PKCAU_BASE      (GD32W51X_AHB2_BASE + 0x61000)

/* Secure alias of a non-secure address */

#define GD32W51X_SECURE_ALIAS    0x10000000

#endif /* __ARCH_ARM_SRC_GD32W51X_GD32W51X_MEMORYMAP_H */
