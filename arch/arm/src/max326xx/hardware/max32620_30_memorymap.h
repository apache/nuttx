/********************************************************************************************
 * arch/arm/src/max326xx/hardware/max32620_30_memorymap.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32620_30_MEMORYMAP_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32620_30_MEMORYMAP_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* System Memory Map */

#define MAX326_CODE_BASE       0x00000000 /* Code address space (not cached, not scrambled) */
#define MAX326_SPIXIP_BASE     0x10000000 /* SPI XIP Code address space (scrambled, not cached) */
#define MAX326_SRAM_BASE       0x20000000 /* SRAM address space (not cached) */
#define MAX326_PERIPH_BASE     0x40000000 /* Peripherals memory space (not cached) */
#  define MAX326_APB_BASE      0x40000000 /* AHB-to-APB Bridge */
#  define MAX326_AHB_BASE      0x41000000 /* AHB Peripherals */
#define MAX326_SYSTEM_BASE     0xe0000000 /* System address space */
#  define MAX326_SCS_BASE      0xe000e000 /* SCSS */

/* AHB-to-APB Bridge */

#define MAX326_SYSMAN_BASE     0x40000000 /* SYSMAN */
#define MAX326_CLKMAN_BASE     0x40000400 /* CLKMAN */
#define MAX326_PWRMAN_BASE     0x40000800 /* PWRMAN */
#define MAX326_RTC_BASE        0x40000a00 /* RTCTMR, PWRSEQ, RTCCFG */
#define MAX326_IOMAN_BASE      0x40000c00 /* IOMAN */
#define MAX326_FLC_BASE        0x40002000 /* FLC (Flash Controller) */
#define MAX326_ICC_BASE        0x40003000 /* ICC (Instruction Cache Controller) */
#define MAX326_SPIX_BASE       0x40004000 /* SPIX (SPI XPI Configuration) */
#define MAX326_PMU_BASE        0x40005000 /* PMU */
#define MAX326_CRC_BASE        0x40006000 /* CRC */
#define MAX326_PRNG_BASE       0x40007000 /* TPU PRNG */
#define MAX326_AES_BASE        0x40007400 /* TPU AES */
#define MAX326_MAA_BASE        0x40007800 /* TPU MAA */
#define MAX326_TPU_BASE        0x40007c00 /* TPU Secure Key Storage */
#define MAX326_WDT0_BASE       0x40008000 /* WDT0 (Watchdog Timer 0) */
#define MAX326_WDT1_BASE       0x40009000 /* WDT0 (Watchdog Timer 1) */
#define MAX326_GPIO_BASE       0x4000a000 /* GPIO */
#define MAX326_TMR0_BASE       0x4000b000 /* TMR0 */
#define MAX326_TMR1_BASE       0x4000c000 /* TMR1 */
#define MAX326_TMR2_BASE       0x4000d000 /* TMR2 */
#define MAX326_TMR3_BASE       0x4000e000 /* TMR3 */
#define MAX326_TMR4_BASE       0x4000f000 /* TMR4 */
#define MAX326_TMR5_BASE       0x40010000 /* TMR5 */
#define MAX326_PT_BASE         0x40011000 /* Pulse Trains */
#define MAX326_UART0_BASE      0x40012000 /* UART 0 */
#define MAX326_UART1_BASE      0x40013000 /* UART 1 */
#define MAX326_UART2_BASE      0x40014000 /* UART 2 */
#define MAX326_UART3_BASE      0x40015000 /* UART 3 */
#define MAX326_I2CM0_BASE      0x40016000 /* I2CM 0 (I2C Master 2) */
#define MAX326_I2CM1_BASE      0x40017000 /* I2CM 1 (I2C Master 2) */
#define MAX326_I2CM2_BASE      0x40018000 /* I2CM 2 (I2C Master 2) */
#define MAX326_I2CS_BASE       0x40019000 /* I2CS (I2C Mailbox Slave) */
#define MAX326_SPIM0_BASE      0x4001a000 /* SPIM 0 */
#define MAX326_SPIM1_BASE      0x4001b000 /* SPIM 1 */
#define MAX326_SPIM2_BASE      0x4001c000 /* SPIM 2 */
#define MAX326_OWM_BASE        0x4001e000 /* OWM (1-Wire Master) */
#define MAX326_ADC_BASE        0x4001f000 /* ADC */
#ifdef CONFIG_ARCH_FAMILY_MAX32630
#  define MAX326_SPIS_BASE     0x40020000 /* SPIS */
#endif

/* AHB Peripherals */

#define MAX326_USB_BASE        0x41000000 /* USB Registers */
#define MAX326_CRC_BASE        0x41001000 /* CRC */
#define MAX326_AESMEM_BASE     0x41002000 /* TPU AES Memory */
#define MAX326_MAAMEM_BASE     0x41002800 /* TPU MAA Memory */
#define MAX326_UART0_FIFO      0x41003000 /* UART 0 FIFOs */
#define MAX326_UART1_FIFO      0x41004000 /* UART 1 FIFOs */
#define MAX326_UART2_FIFO      0x41005000 /* UART 2 FIFOs */
#define MAX326_UART3_FIFO      0x41006000 /* UART 3 FIFOs */
#define MAX326_I2CM0_FIFO      0x41007000 /* I2C Master 0 FIFOs */
#define MAX326_I2CM1_FIFO      0x41008000 /* I2C Master 1 FIFOs */
#define MAX326_I2CM2_FIFO      0x41009000 /* I2C Master 2 FIFOs */
#define MAX326_SPIM0_FIFO      0x4100a000 /* SPI Master 0 FIFOs */
#define MAX326_SPIM1_FIFO      0x4100b000 /* SPI Master 1 FIFOs */
#define MAX326_SPIM2_FIFO      0x4100c000 /* SPI Master 2 FIFOs */
#ifdef CONFIG_ARCH_FAMILY_MAX32630
#  define MAX326_SPIS_FIFO    0x4100e000 /* SPI Slave FIFOs */
#endif

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32620_30_MEMORYMAP_H */
