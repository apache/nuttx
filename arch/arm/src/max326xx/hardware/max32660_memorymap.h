/********************************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_memorymap.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_MEMORYMAP_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_MEMORYMAP_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

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

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_MEMORYMAP_H */
