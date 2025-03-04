/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_resets.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_RESETS_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_RESETS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_RESETS_RESET_OFFSET       0x000000  /* Reset control. If a bit is set it means the peripheral is in reset. 0 means the peripheral's reset is deasserted. */
#define RP23XX_RESETS_WDSEL_OFFSET       0x000004  /* Watchdog select. If a bit is set then the watchdog will reset this peripheral when the watchdog fires. */
#define RP23XX_RESETS_RESET_DONE_OFFSET  0x000008  /* Reset done. If a bit is set then a reset done signal has been returned by the peripheral. This indicates that the peripheral's registers are ready to be accessed. */

/* Register definitions *****************************************************/

#define RP23XX_RESETS_RESET       (RP23XX_RESETS_BASE + RP23XX_RESETS_RESET_OFFSET)
#define RP23XX_RESETS_WDSEL       (RP23XX_RESETS_BASE + RP23XX_RESETS_WDSEL_OFFSET)
#define RP23XX_RESETS_RESET_DONE  (RP23XX_RESETS_BASE + RP23XX_RESETS_RESET_DONE_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_RESETS_RESET_MASK             (0x1fffffff)
#define RP23XX_RESETS_RESET_USBCTRL          (1 << 28)
#define RP23XX_RESETS_RESET_UART1            (1 << 27)
#define RP23XX_RESETS_RESET_UART0            (1 << 26)
#define RP23XX_RESETS_RESET_TRNG             (1 << 25)
#define RP23XX_RESETS_RESET_TIMER1           (1 << 24)
#define RP23XX_RESETS_RESET_TIMER0           (1 << 23)
#define RP23XX_RESETS_RESET_TBMAN            (1 << 22)
#define RP23XX_RESETS_RESET_SYSINFO          (1 << 21)
#define RP23XX_RESETS_RESET_SYSCFG           (1 << 20)
#define RP23XX_RESETS_RESET_SPI1             (1 << 19)
#define RP23XX_RESETS_RESET_SPI0             (1 << 18)
#define RP23XX_RESETS_RESET_SHA256           (1 << 17)
#define RP23XX_RESETS_RESET_PWM              (1 << 16)
#define RP23XX_RESETS_RESET_PLL_USB          (1 << 15)
#define RP23XX_RESETS_RESET_PLL_SYS          (1 << 14)
#define RP23XX_RESETS_RESET_PIO2             (1 << 13)
#define RP23XX_RESETS_RESET_PIO1             (1 << 12)
#define RP23XX_RESETS_RESET_PIO0             (1 << 11)
#define RP23XX_RESETS_RESET_PADS_QSPI        (1 << 10)
#define RP23XX_RESETS_RESET_PADS_BANK0       (1 << 9)
#define RP23XX_RESETS_RESET_JTAG             (1 << 8)
#define RP23XX_RESETS_RESET_IO_QSPI          (1 << 7)
#define RP23XX_RESETS_RESET_IO_BANK0         (1 << 6)
#define RP23XX_RESETS_RESET_I2C1             (1 << 5)
#define RP23XX_RESETS_RESET_I2C0             (1 << 4)
#define RP23XX_RESETS_RESET_HSTX             (1 << 3)
#define RP23XX_RESETS_RESET_DMA              (1 << 2)
#define RP23XX_RESETS_RESET_BUSCTRL          (1 << 1)
#define RP23XX_RESETS_RESET_ADC              (1 << 0)

#define RP23XX_RESETS_WDSEL_USBCTRL          (1 << 28)
#define RP23XX_RESETS_WDSEL_UART1            (1 << 27)
#define RP23XX_RESETS_WDSEL_UART0            (1 << 26)
#define RP23XX_RESETS_WDSEL_TRNG             (1 << 25)
#define RP23XX_RESETS_WDSEL_TIMER1           (1 << 24)
#define RP23XX_RESETS_WDSEL_TIMER0           (1 << 23)
#define RP23XX_RESETS_WDSEL_TBMAN            (1 << 22)
#define RP23XX_RESETS_WDSEL_SYSINFO          (1 << 21)
#define RP23XX_RESETS_WDSEL_SYSCFG           (1 << 20)
#define RP23XX_RESETS_WDSEL_SPI1             (1 << 19)
#define RP23XX_RESETS_WDSEL_SPI0             (1 << 18)
#define RP23XX_RESETS_WDSEL_SHA256           (1 << 17)
#define RP23XX_RESETS_WDSEL_PWM              (1 << 16)
#define RP23XX_RESETS_WDSEL_PLL_USB          (1 << 15)
#define RP23XX_RESETS_WDSEL_PLL_SYS          (1 << 14)
#define RP23XX_RESETS_WDSEL_PIO2             (1 << 13)
#define RP23XX_RESETS_WDSEL_PIO1             (1 << 12)
#define RP23XX_RESETS_WDSEL_PIO0             (1 << 11)
#define RP23XX_RESETS_WDSEL_PADS_QSPI        (1 << 10)
#define RP23XX_RESETS_WDSEL_PADS_BANK0       (1 << 9)
#define RP23XX_RESETS_WDSEL_JTAG             (1 << 8)
#define RP23XX_RESETS_WDSEL_IO_QSPI          (1 << 7)
#define RP23XX_RESETS_WDSEL_IO_BANK0         (1 << 6)
#define RP23XX_RESETS_WDSEL_I2C1             (1 << 5)
#define RP23XX_RESETS_WDSEL_I2C0             (1 << 4)
#define RP23XX_RESETS_WDSEL_HSTX             (1 << 3)
#define RP23XX_RESETS_WDSEL_DMA              (1 << 2)
#define RP23XX_RESETS_WDSEL_BUSCTRL          (1 << 1)
#define RP23XX_RESETS_WDSEL_ADC              (1 << 0)

#define RP23XX_RESETS_RESET_DONE_USBCTRL     (1 << 28)
#define RP23XX_RESETS_RESET_DONE_UART1       (1 << 27)
#define RP23XX_RESETS_RESET_DONE_UART0       (1 << 26)
#define RP23XX_RESETS_RESET_DONE_TRNG        (1 << 25)
#define RP23XX_RESETS_RESET_DONE_TIMER1      (1 << 24)
#define RP23XX_RESETS_RESET_DONE_TIMER0      (1 << 23)
#define RP23XX_RESETS_RESET_DONE_TBMAN       (1 << 22)
#define RP23XX_RESETS_RESET_DONE_SYSINFO     (1 << 21)
#define RP23XX_RESETS_RESET_DONE_SYSCFG      (1 << 20)
#define RP23XX_RESETS_RESET_DONE_SPI1        (1 << 19)
#define RP23XX_RESETS_RESET_DONE_SPI0        (1 << 18)
#define RP23XX_RESETS_RESET_DONE_SHA256      (1 << 17)
#define RP23XX_RESETS_RESET_DONE_PWM         (1 << 16)
#define RP23XX_RESETS_RESET_DONE_PLL_USB     (1 << 15)
#define RP23XX_RESETS_RESET_DONE_PLL_SYS     (1 << 14)
#define RP23XX_RESETS_RESET_DONE_PIO2        (1 << 13)
#define RP23XX_RESETS_RESET_DONE_PIO1        (1 << 12)
#define RP23XX_RESETS_RESET_DONE_PIO0        (1 << 11)
#define RP23XX_RESETS_RESET_DONE_PADS_QSPI   (1 << 10)
#define RP23XX_RESETS_RESET_DONE_PADS_BANK0  (1 << 9)
#define RP23XX_RESETS_RESET_DONE_JTAG        (1 << 8)
#define RP23XX_RESETS_RESET_DONE_IO_QSPI     (1 << 7)
#define RP23XX_RESETS_RESET_DONE_IO_BANK0    (1 << 6)
#define RP23XX_RESETS_RESET_DONE_I2C1        (1 << 5)
#define RP23XX_RESETS_RESET_DONE_I2C0        (1 << 4)
#define RP23XX_RESETS_RESET_DONE_HSTX        (1 << 3)
#define RP23XX_RESETS_RESET_DONE_DMA         (1 << 2)
#define RP23XX_RESETS_RESET_DONE_BUSCTRL     (1 << 1)
#define RP23XX_RESETS_RESET_DONE_ADC         (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_RESETS_H */
