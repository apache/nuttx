/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_resets.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_RESETS_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_RESETS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_RESETS_RESET_OFFSET       0x000000  /* Reset control. If a bit is set it means the peripheral is in reset. 0 means the peripheral's reset is deasserted. */
#define RP2040_RESETS_WDSEL_OFFSET       0x000004  /* Watchdog select. If a bit is set then the watchdog will reset this peripheral when the watchdog fires. */
#define RP2040_RESETS_RESET_DONE_OFFSET  0x000008  /* Reset done. If a bit is set then a reset done signal has been returned by the peripheral. This indicates that the peripheral's registers are ready to be accessed. */

/* Register definitions *****************************************************/

#define RP2040_RESETS_RESET       (RP2040_RESETS_BASE + RP2040_RESETS_RESET_OFFSET)
#define RP2040_RESETS_WDSEL       (RP2040_RESETS_BASE + RP2040_RESETS_WDSEL_OFFSET)
#define RP2040_RESETS_RESET_DONE  (RP2040_RESETS_BASE + RP2040_RESETS_RESET_DONE_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_RESETS_RESET_USBCTRL          (1 << 24)
#define RP2040_RESETS_RESET_UART1            (1 << 23)
#define RP2040_RESETS_RESET_UART0            (1 << 22)
#define RP2040_RESETS_RESET_TIMER            (1 << 21)
#define RP2040_RESETS_RESET_TBMAN            (1 << 20)
#define RP2040_RESETS_RESET_SYSINFO          (1 << 19)
#define RP2040_RESETS_RESET_SYSCFG           (1 << 18)
#define RP2040_RESETS_RESET_SPI1             (1 << 17)
#define RP2040_RESETS_RESET_SPI0             (1 << 16)
#define RP2040_RESETS_RESET_RTC              (1 << 15)
#define RP2040_RESETS_RESET_PWM              (1 << 14)
#define RP2040_RESETS_RESET_PLL_USB          (1 << 13)
#define RP2040_RESETS_RESET_PLL_SYS          (1 << 12)
#define RP2040_RESETS_RESET_PIO1             (1 << 11)
#define RP2040_RESETS_RESET_PIO0             (1 << 10)
#define RP2040_RESETS_RESET_PADS_QSPI        (1 << 9)
#define RP2040_RESETS_RESET_PADS_BANK0       (1 << 8)
#define RP2040_RESETS_RESET_JTAG             (1 << 7)
#define RP2040_RESETS_RESET_IO_QSPI          (1 << 6)
#define RP2040_RESETS_RESET_IO_BANK0         (1 << 5)
#define RP2040_RESETS_RESET_I2C1             (1 << 4)
#define RP2040_RESETS_RESET_I2C0             (1 << 3)
#define RP2040_RESETS_RESET_DMA              (1 << 2)
#define RP2040_RESETS_RESET_BUSCTRL          (1 << 1)
#define RP2040_RESETS_RESET_ADC              (1 << 0)

#define RP2040_RESETS_WDSEL_USBCTRL          (1 << 24)
#define RP2040_RESETS_WDSEL_UART1            (1 << 23)
#define RP2040_RESETS_WDSEL_UART0            (1 << 22)
#define RP2040_RESETS_WDSEL_TIMER            (1 << 21)
#define RP2040_RESETS_WDSEL_TBMAN            (1 << 20)
#define RP2040_RESETS_WDSEL_SYSINFO          (1 << 19)
#define RP2040_RESETS_WDSEL_SYSCFG           (1 << 18)
#define RP2040_RESETS_WDSEL_SPI1             (1 << 17)
#define RP2040_RESETS_WDSEL_SPI0             (1 << 16)
#define RP2040_RESETS_WDSEL_RTC              (1 << 15)
#define RP2040_RESETS_WDSEL_PWM              (1 << 14)
#define RP2040_RESETS_WDSEL_PLL_USB          (1 << 13)
#define RP2040_RESETS_WDSEL_PLL_SYS          (1 << 12)
#define RP2040_RESETS_WDSEL_PIO1             (1 << 11)
#define RP2040_RESETS_WDSEL_PIO0             (1 << 10)
#define RP2040_RESETS_WDSEL_PADS_QSPI        (1 << 9)
#define RP2040_RESETS_WDSEL_PADS_BANK0       (1 << 8)
#define RP2040_RESETS_WDSEL_JTAG             (1 << 7)
#define RP2040_RESETS_WDSEL_IO_QSPI          (1 << 6)
#define RP2040_RESETS_WDSEL_IO_BANK0         (1 << 5)
#define RP2040_RESETS_WDSEL_I2C1             (1 << 4)
#define RP2040_RESETS_WDSEL_I2C0             (1 << 3)
#define RP2040_RESETS_WDSEL_DMA              (1 << 2)
#define RP2040_RESETS_WDSEL_BUSCTRL          (1 << 1)
#define RP2040_RESETS_WDSEL_ADC              (1 << 0)

#define RP2040_RESETS_RESET_DONE_USBCTRL     (1 << 24)
#define RP2040_RESETS_RESET_DONE_UART1       (1 << 23)
#define RP2040_RESETS_RESET_DONE_UART0       (1 << 22)
#define RP2040_RESETS_RESET_DONE_TIMER       (1 << 21)
#define RP2040_RESETS_RESET_DONE_TBMAN       (1 << 20)
#define RP2040_RESETS_RESET_DONE_SYSINFO     (1 << 19)
#define RP2040_RESETS_RESET_DONE_SYSCFG      (1 << 18)
#define RP2040_RESETS_RESET_DONE_SPI1        (1 << 17)
#define RP2040_RESETS_RESET_DONE_SPI0        (1 << 16)
#define RP2040_RESETS_RESET_DONE_RTC         (1 << 15)
#define RP2040_RESETS_RESET_DONE_PWM         (1 << 14)
#define RP2040_RESETS_RESET_DONE_PLL_USB     (1 << 13)
#define RP2040_RESETS_RESET_DONE_PLL_SYS     (1 << 12)
#define RP2040_RESETS_RESET_DONE_PIO1        (1 << 11)
#define RP2040_RESETS_RESET_DONE_PIO0        (1 << 10)
#define RP2040_RESETS_RESET_DONE_PADS_QSPI   (1 << 9)
#define RP2040_RESETS_RESET_DONE_PADS_BANK0  (1 << 8)
#define RP2040_RESETS_RESET_DONE_JTAG        (1 << 7)
#define RP2040_RESETS_RESET_DONE_IO_QSPI     (1 << 6)
#define RP2040_RESETS_RESET_DONE_IO_BANK0    (1 << 5)
#define RP2040_RESETS_RESET_DONE_I2C1        (1 << 4)
#define RP2040_RESETS_RESET_DONE_I2C0        (1 << 3)
#define RP2040_RESETS_RESET_DONE_DMA         (1 << 2)
#define RP2040_RESETS_RESET_DONE_BUSCTRL     (1 << 1)
#define RP2040_RESETS_RESET_DONE_ADC         (1 << 0)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_RESETS_H */
