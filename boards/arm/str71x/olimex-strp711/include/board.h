/****************************************************************************
 * boards/arm/str71x/olimex-strp711/include/board.h
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

/****************************************************************************
 * Features:
 *
 * - MCU: STR711FR2T6 16/32 bit ARM7TDMI™ with 256K Bytes Program Flash,
 *   64K Bytes RAM, USB 2.0, RTC, 12 bit ADC, 4x UARTs, 2x I2C,2x SPI,
 *   5x 32bit TIMERS, 2x PWM, 2x CCR, WDT, up to 50MHz operation
 * - Standard JTAG connector with ARM 2x10 pin layout for programming/
 *   debugging with ARM-JTAG
 * - USB connector
 * - Two channel RS232 interface and drivers
 * - SD/MMC card connector
 * - Two buttons
 * - Trimpot connected to ADC
 * - Two status LEDs
 * - Buzzer
 * - UEXT - 10 pin extension connector for Olimex addon peripherals like MP3,
 *   RF2.4Ghz, RFID etc. modules
 * - 2x SPI connectors
 * - I2C connector
 * - On board voltage regulator 3.3V with up to 800mA current
 * - Single power supply: 6V AC or DC required, USB port can power the board
 * - Power supply LED
 * - Power supply filtering capacitor
 * - RESET circuit
 * - RESET button
 * - 4 MHz crystal oscillator
 * - 32768 Hz crystal and RTC
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STR71X_OLIMEX_STRP711_INCLUDE_BOARD_H
#define __BOARDS_ARM_STR71X_OLIMEX_STRP711_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
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

#undef  STR71X_PLL1IN_DIV2         /* Don't divide main OSC by two */
#define STR71X_PLL1OUT_MUL   16    /* PLL1OUT = 16 * CLK2 */
#define STR71X_PLL1OUT_DIV   2     /* PLL1OUT = CLK2 / 2 */
#define STR71X_APB1_DIV      1     /* PCLK1 = RCLK */
#define STR71X_APB2_DIV      1     /* PCLK2 = RCLK */
#define STR71X_MCLK_DIV      1     /* MCLK = RCLK */

/* PLL2 Setup -- only needed for HDLC or USB
 *
 * USB input: USB clock
 * HCLK = 4MHz?
 * USB clock = 12 * HCLK / 1 = 48 MHz
 */

#undef  STR71X_USBIN_PLL2          /* USB input is USB clock */
#define STR71X_PLL2OUT_MUL   12    /* PLL2OUT = 12 * HCLK */
#define STR71X_PLL2OUT_DIV   1     /* PLL2OUT = HCLK / 1 */

/* LED definitions **********************************************************/

/* The Olimex board has only two LEDs, so following states are faked as
 * follows
 *
 *                       SET         CLEAR
 *  LED_STARTED          (none)      (n/a)
 *  LED_HEAPALLOCATE     LED1        (n/a)
 *  LED_IRQSENABLED      LED1        (n/a)
 *  LED_STACKCREATED     LED1        (n/a)
 *  LED_INIRQ            LED1+LED2   LED1
 *  LED_SIGNAL           LED1+LED2   LED1
 *  LED_ASSERTION        LED1+LED2   LED1
 *  LED_PANIC            LED1+LED2*  LED1
 *
 *                      *The previous state of LED2 will be retained
 */

#define LED_STARTED          0
#define LED_HEAPALLOCATE     1
#define LED_IRQSENABLED      2
#define LED_STACKCREATED     3
#define LED_INIRQ            4
#define LED_SIGNAL           5
#define LED_ASSERTION        6
#define LED_PANIC            7

/* Button definitions *******************************************************/

/* The Olimex board has two buttons,
 * one labeled "BUT" and the other "WAKEUP"
 */

#define BUT_BUTTON              1 /* Bit 0: BUT button is depressed */
#define WAKEUP_BUTTON           2 /* Bit 1: WAKEUP button is depressed */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __BOARDS_ARM_STR71X_OLIMEX_STRP711_INCLUDE_BOARD_H */
