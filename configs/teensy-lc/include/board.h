/************************************************************************************
 * configs/teensy-lc/include/board.h
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The board has a 16MHz crystal. */

#undef  BOARD_EXTCLOCK                      /* Crystal */
#define BOARD_XTAL_FREQ      16000000       /* 16 MHz crystal frequency (REFCLK) */

/* PLL Configuration.
 *
 *   PLL Input frequency:   PLLIN     = REFCLK / PRDIV0 = 16MHz / 4 = 4 MHz
 *   PLL Output frequency:  PLLOUT    = PLLIN * VDIV0   = 4Mhz * 24 = 96 MHz
 *   MCGPLLCLK Frequency:   MCGPLLCLK = 96MHz
 */

#define BOARD_PRDIV0         4              /* PLL External Reference Divider */
#define BOARD_VDIV0          24             /* PLL VCO Divider (frequency multiplier) */

#define BOARD_PLLIN_FREQ     (BOARD_XTAL_FREQ / BOARD_PRDIV0)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV0)
#define BOARD_MCGPLLCLK_FREQ  BOARD_PLLOUT_FREQ

/* MCGOUTCLK: MCG output of either IRC, MCGFLLCLK, MCGPLLCLK, or MCG's external
 * reference clock that sources the core, system, bus, and flash clock.
 *
 * MCGOUTCLK = MCGPLLCLK = 96MHz
 */

#define BOARD_MCGOUTCLK_FREQ  BOARD_MCGPLLCLK_FREQ

/* SIM CLKDIV1 dividers.
 *
 * Core/system clock
 *   MCGOUTCLK divided by OUTDIV1, clocks the ARM Cortex-M0+ core
 *
 * Bus clock
 *   System clock divided by OUTDIV4, clocks the bus slaves and peripherals.
 */

#define BOARD_OUTDIV1        2              /* Core/system = MCGOUTCLK / 2 = 48 MHz */
#define BOARD_OUTDIV4        2              /* Bus clock   = System clock / 2 = 24 MHz */

#define BOARD_CORECLK_FREQ  (BOARD_MCGOUTCLK_FREQ / BOARD_OUTDIV1)
#define BOARD_BUSCLK_FREQ   (BOARD_CORECLK_FREQ / BOARD_OUTDIV4)

/* PWM Configuration */
/* TPM0 Channels */

#define GPIO_TPM0_CH0OUT PIN_TPM0_CH0_2  // Pin 22: PTC1
#define GPIO_TPM0_CH1OUT PIN_TPM0_CH1_2  // Pin 23: PTC2
#define GPIO_TPM0_CH2OUT PIN_TPM0_CH2_2  // Pin  9: PTC3
#define GPIO_TPM0_CH3OUT PIN_TPM0_CH3_1  // Pin 10: PTC4
#define GPIO_TPM0_CH4OUT PIN_TPM0_CH4_2  // Pin  6: PTD4
#define GPIO_TPM0_CH5OUT PIN_TPM0_CH5_3  // Pin 20: PTD5

/* TPM1 Channels */

#define GPIO_TPM1_CH0OUT PIN_TPM1_CH0_2  // Pin 16: PTB0
#define GPIO_TPM1_CH1OUT PIN_TPM1_CH1_2  // Pin 17: PTB1

/* TPM2 Channels */

#define GPIO_TPM2_CH0OUT PIN_TPM2_CH0_1  // Pin  3: PTA1
#define GPIO_TPM2_CH1OUT PIN_TPM2_CH1_1  // Pin  4: PTA2

/* LED definitions ******************************************************************/
/* The Teensy LC has a single LED. */
#define LED_STARTED       0  /* LED off */
#define LED_HEAPALLOCATE  0  /* LED off */
#define LED_IRQSENABLED   0  /* LED off */
#define LED_STACKCREATED  1  /* LED on */
#define LED_INIRQ         2  /* LED no change */
#define LED_SIGNAL        2  /* LED no change */
#define LED_ASSERTION     2  /* LED no change */
#define LED_PANIC         3  /* LED flashing */

/* SPI0 Pinout
 * ===========
 *
 */

/* Note that the Teensy maps SCK0 to pin 13 which conflicts with the
 * LED.  Use pin 14 instead. */
#define PIN_SPI0_SCK   (PIN_SPI0_SCK_3 | PIN_ALT2_PULLUP)  // Pin 14: PTD1
#define PIN_SPI0_MISO  (PIN_SPI0_MISO_4 | PIN_ALT2_PULLUP) // Pin 12: PTC7
#define PIN_SPI0_MOSI  (PIN_SPI0_MOSI_3 | PIN_ALT2_PULLUP) // Pin 11: PTC6

#define PIN_SPI1_SCK   (PIN_SPI1_SCK_2 | PIN_ALT2_PULLUP)  // Pin 20: PTD5
#define PIN_SPI1_MISO  (PIN_SPI1_MISO_2 | PIN_ALT2_PULLUP) // Pin 1: PTB17
#define PIN_SPI1_MOSI  (PIN_SPI0_MOSI_1 | PIN_ALT2_PULLUP) // Pin 0: PTB16

#endif  /* __ARCH_BOARD_BOARD_H */
