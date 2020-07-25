/****************************************************************************************************
 * arch/arm/src/lpc55xx/chip/lpc55s6x_memorymap.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55S6X_MEMORYMAP_H
#define __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55S6X_MEMORYMAP_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Memory Map */

#define LPC55_FLASH_BASE     0x00000000  /* Flash memory (630 KB) */
#define LPC55_BOOTROM_BASE   0x03000000  /* Boot ROM with flash services in a 128 KB space. */
#define LPC55_SRAMX_BASE     0x04000000  /* I&D SRAM bank (32 KB) */
#define LPC55_SRAM_BASE      0x20000000  /* SRAM banks (256+16 KB) */
#define LPC55_APB0_BASE      0x40000000  /* APB slave group 0 (128 KB) */
#define LPC55_APB1_BASE      0x40020000  /* APB slave group 1 (128 KB) */
#define LPC55_AHB_BASE       0x40080000  /* AHB peripherals (256 KB) */
#define LPC55_USBSRAM_BASE   0x40100000  /* USB SRAM (8 KB) */

/* ROM Driver Table */

#define LPC55_ROM_DRIVERTAB  0x03000200  /* Beginning of the ROM driver table */

/* AHB Peripherals */

#define LPC55_DMA_BASE       0x40082000  /* DMA registers */
#define LPC55_FSUSB_BASE     0x40084000  /* FS USB device registers */
#define LPC55_SCTPWM_BASE    0x40085000  /* SC Timer / PWM */
#define LPC55_FLEXCOMM0_BASE 0x40086000  /* Flexcomm 0 */
#define LPC55_FLEXCOMM1_BASE 0x40087000  /* Flexcomm 1 */
#define LPC55_FLEXCOMM2_BASE 0x40088000  /* Flexcomm 2 */
#define LPC55_FLEXCOMM3_BASE 0x40089000  /* Flexcomm 3 */
#define LPC55_FLEXCOMM4_BASE 0x4008a000  /* Flexcomm 4 */
#define LPC55_ICPUMB_BASE    0x4008b000  /* Inter-CPU Mailbox */
#define LPC55_GPIO_BASE      0x4008c000  /* High Speed GPIO */
#define LPC55_HSUSB_BASE     0x40094000  /* HS USB device */
#define LPC55_CRC_BASE       0x40095000  /* CRC engine */
#define LPC55_FLEXCOMM5_BASE 0x40096000  /* Flexcomm 5 */
#define LPC55_FLEXCOMM6_BASE 0x40097000  /* Flexcomm 6 */
#define LPC55_FLEXCOMM7_BASE 0x40098000  /* Flexcomm 7 */
#define LPC55_SDMMC_BASE     0x4009b000  /* SD/MMC */
#define LPC55_DMAP_BASE      0x4009c000  /* DM-AP interface */
#define LPC55_ADC_BASE       0x400a0000  /* ADC */
#define LPC55_FSUSBHOST_BASE 0x400a2000  /* FS USB host registers */
#define LPC55_HSUSBHOST_BASE 0x400a3000  /* HS USB host registers */
#define LPC55_HASHAES_BASE   0x400a4000  /* Hash-AES registers */

/* APB Bridge 0 */

#define LPC55_SYSCON_BASE    0x40000000  /* Syscon */
#define LPC55_IOCON_BASE     0x40001000  /* IOCON */
#define LPC55_GINT0_BASE     0x40002000  /* GINT0 */
#define LPC55_GINT1_BASE     0x40003000  /* GINT1 */
#define LPC55_PINT_BASE      0x40004000  /* Pin Interrupts (PINT) */
#define LPC55_SPINT_BASE     0x40005000  /* Secure Pin Interrupts (SPINT) */
#define LPC55_MUX_BASE       0x40006000  /* Input muxes */
#define LPC55_CTIMER0_BASE   0x40008000  /* CTIMER0 */
#define LPC55_CTIMER1_BASE   0x40009000  /* CTIMER1 */
#define LPC55_WWDT_BASE      0x4000c000  /* WDT */
#define LPC55_MRT_BASE       0x4000d000  /* MRT */
#define LPC55_MTICK_BASE     0x4000e000  /* Micro-Tick */
#define LPC55_ANALOG_BASE    0x40013000  /* Analog controller */

/* APB Bridge 1 */

#define LPC55_SYSCTL_BASE    0x40023000  /* Other system registers */
#define LPC55_CTIMER2_BASE   0x40028000  /* CTIMER2 */
#define LPC55_CTIMER3_BASE   0x40029000  /* CTIMER3 */
#define LPC55_CTIMER4_BASE   0x4002a000  /* CTIMER4 */
#define LPC55_RTC_BASE       0x4002c000  /* RTC */
#define LPC55_RIT_BASE       0x4002d000  /* RIT */
#define LPC55_FLASHC_BASE    0x40034000  /* Flash controller */
#define LPC55_PRINCE_BASE    0x40035000  /* PRINCE dynamic encrypt/decrypt */
#define LPC55_USBHSPHY_BASE  0x40038000  /* USB HS Phy */
#define LPC55_RNG_BASE       0x4003a000  /* RNG */
#define LPC55_PUF_BASE       0x4003b000  /* Physical Unclonable Function */
#define LPC55_PLU_BASE       0x4003d000  /* Programmable Logic Unit */

#endif /* __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55S6X_MEMORYMAP_H */
