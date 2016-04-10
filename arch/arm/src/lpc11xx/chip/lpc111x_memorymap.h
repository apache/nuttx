/************************************************************************************
 * arch/arm/src/lpc11xx/lpc111x_memorymap.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC11XX_LPC111X_MEMORYMAP_H
#define __ARCH_ARM_SRC_LPC11XX_LPC111X_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory Map ***********************************************************************/

#define LPC11_FLASH_BASE    0x00000000 /* -0x1fffffff: On-chip non-volatile memory */
#define LPC11_SRAM_BASE     0x10000000 /* -0x10007fff: On-chip SRAM (devices <=16Kb) */
#define LPC11_ROM_BASE      0x1fff0000 /* -0x1fffffff: 16Kb Boot ROM with flash services */
#define LPC11_AHBSRAM_BASE  0x20000000 /* -0x3fffffff: On-chip AHB SRAM (devices >32Kb) */
#define LPC11_GPIO_BASE     0x50000000 /* -0x2009ffff: GPIO at AHB Peripherals */
#define LPC11_APB_BASE      0x40000000 /* -0x4007ffff: APB Peripherals */
#define LPC11_AHB_BASE      0x50000000 /* -0x501fffff: AHB Peripherals */
#define LPC11_CORTEXM3_BASE 0xe0000000 /* -0xe00fffff: (see armv7-m/nvic.h) */
#define LPC11_SCS_BASE      0xe000e000
#define LPC11_DEBUGMCU_BASE 0xe0042000

/* APB Peripherals *****************************************************************/

#define LPC11_I2C0_BASE     0x40000000 /* -0x40003fff: I2C-bus */
#define LPC11_WDT_BASE      0x40004000 /* -0x40007fff: Watchdog timer */
#define LPC11_UART0_BASE    0x40008000 /* -0x4000bfff: UART 0 */
#define LPC11_TMR0_BASE     0x4000c000 /* -0x4000ffff: Timer 0 */
#define LPC11_TMR1_BASE     0x40010000 /* -0x40013fff: Timer 1 */
#define LPC11_TMR2_BASE     0x40014000 /* -0x40017fff: Timer 0 */
#define LPC11_TMR3_BASE     0x40018000 /* -0x4001bfff: Timer 1 */
#define LPC11_ADC_BASE      0x4001c000 /* -0x4001ffff: ADC */
                                       /* -0x40037fff: Reserved */
#define LPC11_PMU_BASE      0x40038000 /* -0x4003bfff: PMU */
                                       /* -0x40017fff: Reserved */
#define LPC11_FLASHC_BASE   0x4003c000 /* -0x4003ffff: Flash Controller */
#define LPC11_SPI0_BASE     0x40040000 /* -0x40043fff: SPI0 */
#define LPC11_IOCON_BASE    0x40044000 /* -0x40047fff: IOCONFIG */
#define LPC11_SYSCON_BASE   0x40048000 /* -0x4004bfff: System Control */
                                       /* -0x4004ffff: Reserved */
#define LPC11_CAN0_BASE     0x40050000 /* -0x40053fff: CAN0 */
                                       /* -0x40057ffff: Reserved */
#define LPC11_SPI1_BASE     0x40058000 /* -0x4005bffff: SPI1 */
                                       /* -0x4007fffff: Reserved */

/* AHB Peripherals ******************************************************************/

#define LPC11_GPIO_PIO0     (LPC11_GPIO_BASE + 0)       /* -0x5000ffff: GPIO PIO0 */
#define LPC11_GPIO_PIO1     (LPC11_GPIO_BASE + 0x10000) /* -0x5001ffff: GPIO PIO1 */
#define LPC11_GPIO_PIO2     (LPC11_GPIO_BASE + 0x20000) /* -0x5002ffff: GPIO PIO1 */
#define LPC11_GPIO_PIO3     (LPC11_GPIO_BASE + 0x30000) /* -0x5003ffff: GPIO PIO1 */
                                                        /* -0x501fffff: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_LPC116X_MEMORYMAP_H */
