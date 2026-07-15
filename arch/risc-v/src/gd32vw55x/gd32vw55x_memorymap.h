/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_MEMORYMAP_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory regions */

#define GD32VW55X_FLASH_BASE     0x08000000  /* Main flash (2/4 MB) */
#define GD32VW55X_SRAM_BASE      0x20000000  /* 320 KB SRAM (top 32 KB shared) */
#define GD32VW55X_BLERAM_BASE    0x21000000  /* 64 KB BLE RAM */
#define GD32VW55X_BOOTROM_BASE   0x0bf40000  /* 256 KB mask ROM (IBL) */
#define GD32VW55X_EFUSE_BASE     0x0ffc0000  /* 256 B EFUSE */
#define GD32VW55X_QSPIMEM_BASE   0x90000000  /* QSPI flash XIP window */

/* Core-local (Nuclei N307) */

#define GD32VW55X_SYSTIMER_BASE  0xd1000000  /* 64-bit SysTimer (mtime) */
#define GD32VW55X_ECLIC_BASE     0xd2000000  /* ECLIC */

/* AHB1/AHB2 peripherals */

#define GD32VW55X_GPIOA_BASE     0x40020000
#define GD32VW55X_GPIOB_BASE     0x40020400
#define GD32VW55X_GPIOC_BASE     0x40020800
#define GD32VW55X_FMC_BASE       0x40022000

/* Peripheral base addresses (from the vendor SDK gd32vw55x.h).  The family
 * has a single instance of SPI, ADC, QSPI, CRC, TRNG and DMA; there is no
 * I2S, USB, CAN, DAC, SDIO or Ethernet controller.
 */

#define GD32VW55X_APB1_BASE      0x40000000
#define GD32VW55X_APB2_BASE      0x40010000
#define GD32VW55X_AHB1_BASE      0x40020000
#define GD32VW55X_AHB2_BASE      0x4c000000

/* APB1 */

#define GD32VW55X_TIMER1_BASE    (GD32VW55X_APB1_BASE + 0x0000)
#define GD32VW55X_TIMER2_BASE    (GD32VW55X_APB1_BASE + 0x0400)
#define GD32VW55X_TIMER5_BASE    (GD32VW55X_APB1_BASE + 0x1000)
#define GD32VW55X_RTC_BASE       (GD32VW55X_APB1_BASE + 0x2800)
#define GD32VW55X_WWDGT_BASE     (GD32VW55X_APB1_BASE + 0x2c00)
#define GD32VW55X_FWDGT_BASE     (GD32VW55X_APB1_BASE + 0x3000)
#define GD32VW55X_UART1_BASE     (GD32VW55X_APB1_BASE + 0x4400)
#define GD32VW55X_USART0_BASE    (GD32VW55X_APB1_BASE + 0x4800)
#define GD32VW55X_I2C0_BASE      (GD32VW55X_APB1_BASE + 0x5400)
#define GD32VW55X_I2C1_BASE      (GD32VW55X_APB1_BASE + 0x5800)
#define GD32VW55X_PMU_BASE       (GD32VW55X_APB1_BASE + 0x7000)
#define GD32VW55X_UART2_BASE     (GD32VW55X_APB1_BASE + 0x11000)

/* APB2 */

#define GD32VW55X_TIMER0_BASE    (GD32VW55X_APB2_BASE + 0x0000)
#define GD32VW55X_ADC_BASE       (GD32VW55X_APB2_BASE + 0x2000)
#define GD32VW55X_SPI_BASE       (GD32VW55X_APB2_BASE + 0x3000)
#define GD32VW55X_SYSCFG_BASE    (GD32VW55X_APB2_BASE + 0x3800)
#define GD32VW55X_EXTI_BASE      (GD32VW55X_APB2_BASE + 0x3c00)
#define GD32VW55X_TIMER15_BASE   (GD32VW55X_APB2_BASE + 0x8000)
#define GD32VW55X_TIMER16_BASE   (GD32VW55X_APB2_BASE + 0x8400)

/* AHB1 */

#define GD32VW55X_CRC_BASE       (GD32VW55X_AHB1_BASE + 0x3000)
#define GD32VW55X_RCU_BASE       (GD32VW55X_AHB1_BASE + 0x3800)
#define GD32VW55X_QSPI_BASE      (GD32VW55X_AHB1_BASE + 0x5800)
#define GD32VW55X_DMA_BASE       (GD32VW55X_AHB1_BASE + 0x6000)

/* AHB2 (crypto accelerators) */

#define GD32VW55X_CAU_BASE       (GD32VW55X_AHB2_BASE + 0x60000)
#define GD32VW55X_HAU_BASE       (GD32VW55X_AHB2_BASE + 0x60400)
#define GD32VW55X_TRNG_BASE      (GD32VW55X_AHB2_BASE + 0x60800)
#define GD32VW55X_PKCAU_BASE     (GD32VW55X_AHB2_BASE + 0x61000)

/* Wi-Fi MAC (driven by the prebuilt libraries) */

#define GD32VW55X_WIFI_BASE      0x40030000

/* Idle thread stack starts at _ebss */

#ifndef __ASSEMBLY__
#define GD32VW55X_IDLESTACK_BASE (uintptr_t)_ebss
#else
#define GD32VW55X_IDLESTACK_BASE _ebss
#endif

#define GD32VW55X_IDLESTACK_TOP  (GD32VW55X_IDLESTACK_BASE + SMP_STACK_SIZE)

#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_MEMORYMAP_H */
