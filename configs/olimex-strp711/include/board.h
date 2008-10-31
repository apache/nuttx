/****************************************************************************
 * configs/olimex-strp711/include/board.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ****************************************************************************/
/****************************************************************************
 * Features:
 *
 * - MCU: STR711FR2T6 16/32 bit ARM7TDMI™ with 256K Bytes Program Flash,
 *   64K Bytes RAM, USB 2.0, RTC, 12 bit ADC, 4x UARTs, 2x I2C,2x SPI,
 *   5x 32bit TIMERS, 2x PWM, 2x CCR, WDT, up to 50MHz operation
 * - Standard JTAG connector with ARM 2x10 pin layout for programming/debugging
 *   with ARM-JTAG
 * - USB connector
 * - Two channel RS232 interface and drivers
 * - SD/MMC card connector
 * - Two buttons
 * - Trimpot connected to ADC
 * - Two status LEDs
 * - Buzzer
 * - UEXT - 10 pin extension connector for Olimex addon peripherials like MP3,
 *   RF2.4Ghz, RFID etc. modules
 * - 2x SPI connectors
 * - I2C connector
 * - On board voltage regulator 3.3V with up to 800mA current
 * - Single power supply: 6V AC or DC required, USB port can power the board
 * - Power supply LED
 * - Power supply filtering capacitor
 * - RESET circuit
 * - RESET button
 * - 4 Mhz crystal oscillator
 * - 32768 Hz crystal and RTC
 *
 ****************************************************************************/

#ifndef _CONFIGS_OLIMEX_STRP711_BOARD_H
#define _CONFIGS_OLIMEX_STRP711_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
# include <sys/types.h>
#endif
#include "chip.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Main Oscillator Frequency = 4MHz */

#define STR71X_RCCU_MAIN_OSC (4000000)

/* RTC Oscillator Frequency = 32,768 Hz */

#define STR71X_RCCU_RTC_OSC  (32768)

/* HCLK driving PLL2 */

#define STR71X_PCU_HCLK_OSC  (4000000)  /* ? */

/* PLL1 Setup:
 *
 * PLL1 input clock:      CLK2 = Main OSC = 4MHz
 * PLL1 output clock:     PLL1OUT = 16 * CLK2 / 2 = 32MHz
 * PLL1 output:           CLK3 = PLL1OUT = 32MHz (hard coded selection)
 *                        RCLK = CLK3  = 32MHz (hard coded selection)
 * APB1 peripheral clock: PCLK1 = RCLK = 32MHz
 * APB2 peripheral clock: PCLK2 = RCLK = 32MHz
 * Main system clock:     MCLK  = RCLK = 32MHz
 */

#undef  STR71X_PLL1IN_DIV2                            /* Don't divide main OSC by two */
#define STR71X_PLL1OUT_MUL   STR71X_RCCUPLL1CR_MUL16  /* PLL1OUT = 16 * CLK2 */
#define STR71X_PLL1OUT_DIV   STR71X_RCCUPLL1CR_DIV2   /* PLL1OUT = CLK2 / 2 */
#define STR71X_APB1_DIV      STR71X_PCUPDIVR_APB1DIV1 /* PCLK1 = RCLK */
#define STR71X_APB2_DIV      STR71X_PCUPDIVR_APB1DIV1 /* PCLK2 = RCLK */
#define STR71X_MCLK_DIV      STR71X_PCUMDIVR_DIV1     /* MCLK = RCLK */

/* PLL2 Setup -- only needed for HDLC or USB
 *
 * USB input: USB clock
 * HCLK = 4MHz?
 * USB clock = 12 * HCLK / 1 = 48 MHz
 */

#undef  STR71X_USBIN_PLL2                             /* USB input is USB clock */
#define STR71X_PLL2OUT_MUL   STR71X_PCUPPL2CR_MUL12   /* PLL2OUT = 12 * HCLK */
#define STR71X_PLL2OUT_DIV   STR71X_PCUPPL2CR_DIV1    /* PLL2OUT = HCLK / 1 */

/* LED definitions **********************************************************/

#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif  /* _CONFIGS_OLIMEX_STRP711_BOARD_H */
