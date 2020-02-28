/****************************************************************************************************
 * arch/arm/src/lpc54xx/chip/lpc546x_memorymap.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC546X_MEMORYMAP_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC546X_MEMORYMAP_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Memory Map */

#define LPC54_FLASH_BASE     0x00000000  /* Flash memory (512 KB) */
#define LPC54_BOOTROM_BASE   0x03000000  /* Boot ROM with flash services in a 64 KB space. */
#define LPC54_SRAMX_BASE     0x04000000  /* I&D SRAM bank (32 KB) */
#define LPC54_SPIFLASH_BASE  0x10000000  /* SPIFI memory mapped access space (128 MB). */
#define LPC54_SRAM_BASE      0x20000000  /* SRAM banks (160 KB) */
#define LPC54_SRAMBB_BASE    0x22000000  /* SRAM bit band alias addressing (32 MB) */
#define LPC54_APB0_BASE      0x40000000  /* APB slave group 0 (128 KB) */
#define LPC54_APB1_BASE      0x40020000  /* APB slave group 1 (128 KB) */
#define LPC54_APB2_BASE      0x40040000  /* APB slave group 2 (128 KB) */
#define LPC54_AHB_BASE       0x40080000  /* AHB peripherals (256 KB) */
#define LPC54_USBSRAM_BASE   0x40100000  /* USB SRAM (8 KB) */
#define LPC54_PERIPHBB_BASE  0x42000000  /* Peripheral bit band alias addressing (32 MB) */
#define LPC54_SRAMCS0_BASE   0x80000000  /* Static memory chip select 0 (<=64MB) */
#define LPC54_SRAMCS1_BASE   0x88000000  /* Static memory chip select 1 (<=64MB) */
#define LPC54_SRAMCS2_BASE   0x90000000  /* Static memory chip select 2 (<=64MB) */
#define LPC54_SRAMCS3_BASE   0x98000000  /* Static memory chip select 3 (<=64MB) */
#define LPC54_DRAMCS0_BASE   0xa0000000  /* Dynamic memory chip select 0 (<=256MB) */
#define LPC54_DRAMCS1_BASE   0xa8000000  /* Dynamic memory chip select 1 (<=256MB) */
#define LPC54_DRAMCS2_BASE   0xb0000000  /* Dynamic memory chip select 2 (<=256MB) */
#define LPC54_DRAMCS3_BASE   0xb8000000  /* Dynamic memory chip select 3 (<=256MB) */
#define LPC54_CORTEXM4_BASE  0xe0000000  /* Cortex-M4 Private Peripheral Bus */

/* ROM Driver Table */

#define LPC54_ROM_DRIVERTAB  0x03000200  /* Beginning of the ROM driver table */

/* AHB Peripherals */

#define LPC54_SPIFI_BASE     0x40080000  /* SPIFI registers */
#define LPC54_EMC_BASE       0x40081000  /* EMC registers */
#define LPC54_DMA_BASE       0x40082000  /* DMA registers */
#define LPC54_LCD_BASE       0x40083000  /* LCD registers */
#define LPC54_FSUSB_BASE     0x40084000  /* FS USB device registers */
#define LPC54_SCTPWM_BASE    0x40085000  /* SC Timer / PWM */
#define LPC54_FLEXCOMM0_BASE 0x40086000  /* Flexcomm 0 */
#define LPC54_FLEXCOMM1_BASE 0x40087000  /* Flexcomm 1 */
#define LPC54_FLEXCOMM2_BASE 0x40088000  /* Flexcomm 2 */
#define LPC54_FLEXCOMM3_BASE 0x40089000  /* Flexcomm 3 */
#define LPC54_FLEXCOMM4_BASE 0x4008a000  /* Flexcomm 4 */
#define LPC54_GPIO_BASE      0x4008c000  /* High Speed GPIO */
#define LPC54_DMIC_BASE      0x40090000  /* D-Mic interface */
#define LPC54_ETHERNET_BASE  0x40092000  /* Ethernet */
#define LPC54_HSUSB_BASE     0x40094000  /* HS USB device */
#define LPC54_CRC_BASE       0x40095000  /* CRC engine */
#define LPC54_FLEXCOMM5_BASE 0x40096000  /* Flexcomm 5 */
#define LPC54_FLEXCOMM6_BASE 0x40097000  /* Flexcomm 6 */
#define LPC54_FLEXCOMM7_BASE 0x40098000  /* Flexcomm 7 */
#define LPC54_FLEXCOMM8_BASE 0x40099000  /* Flexcomm 8 */
#define LPC54_FLEXCOMM9_BASE 0x4009a000  /* Flexcomm 9 */
#define LPC54_SDMMC_BASE     0x4009b000  /* SD/MMC */
#define LPC54_ISPAP_BASE     0x4009c000  /* ISP-AP interface */
#define LPC54_CAN0_BASE      0x4009d000  /* CAN 0 */
#define LPC54_CAN1_BASE      0x4009e000  /* CAN 1 */
#define LPC54_ADC_BASE       0x400a0000  /* ADC */
#define LPC54_SHA_BASE       0x400a1000  /* SHA registers */
#define LPC54_FSUSBHOST_BASE 0x400a2000  /* FS USB host registers */
#define LPC54_HSUSBHOST_BASE 0x400a3000  /* HS USB host registers */
#define LPC54_USBSRAM_BASE   0x40100000  /* USB SRAM (8 kB) */
#define LPC54_EPROM_BASE     0x40108000  /* EPROM (16 kB) */

/* APB Bridge 0 */

#define LPC54_SYSCON_BASE    0x40000000  /* Syscon */
#define LPC54_IOCON_BASE     0x40001000  /* IOCON */
#define LPC54_GINT0_BASE     0x40002000  /* GINT0 */
#define LPC54_GINT1_BASE     0x40003000  /* GINT1 */
#define LPC54_PINT_BASE      0x40004000  /* Pin Interrupts (PINT) */
#define LPC54_MUX_BASE       0x40005000  /* Input muxes */
#define LPC54_CTIMER0_BASE   0x40008000  /* CTIMER0 */
#define LPC54_CTIMER1_BASE   0x40009000  /* CTIMER1 */
#define LPC54_WWDT_BASE      0x4000c000  /* WDT */
#define LPC54_MRT_BASE       0x4000d000  /* MRT */
#define LPC54_MTICK_BASE     0x4000e000  /* Micro-Tick */
#define LPC54_EEPROMC_BASE   0x40014000  /* EEPROM controller */
#define LPC54_OTP_BASE       0x40016000  /* OTP controller */

/* APB Bridge 1 */

#define LPC54_OSYSCON_BASE   0x40020000  /* Other system registers */
#define LPC54_CTIMER2_BASE   0x40028000  /* CTIMER2 */
#define LPC54_RTC_BASE       0x4002c000  /* RTC */
#define LPC54_RIT_BASE       0x4002d000  /* RIT */
#define LPC54_FLASHC_BASE    0x40034000  /* Flash controller */
#define LPC54_SMARCARD0_BASE 0x40036000  /* Smart card 0 */
#define LPC54_SMARCARD1_BASE 0x40037000  /* Smart card 1 */
#define LPC54_RNG_BASE       0x4003a000  /* RNG */

/* Asynchronous APB bridge */

#define LPC54_ASYSCON_BASE   0x40040000  /* Asynchronous Syscon */
#define LPC54_CTIMER3_BASE   0x40048000  /* CTIMER3 */
#define LPC54_CTIMER4_BASE   0x40049000  /* CTIMER4 */

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC546X_MEMORYMAP_H */
