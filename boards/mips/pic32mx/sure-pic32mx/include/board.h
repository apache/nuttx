/****************************************************************************
 * boards/mips/pic32mx/sure-pic32mx/include/board.h
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

#ifndef __BOARDS_MIPS_PIC32MX_SURE_PIC32MX_INCLUDE_BOARD_H
#define __BOARDS_MIPS_PIC32MX_SURE_PIC32MX_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Clocking *****************************************************************/

/* Crystal frequencies */

#define BOARD_POSC_FREQ        20000000  /* Primary OSC XTAL frequency (20MHz) */
#define BOARD_SOSC_FREQ        32768     /* Secondary OSC XTAL frequency (32.768KHz) */

/* Oscillator modes */

#define BOARD_FNOSC_POSCPLL    1        /* Use primary oscillator w/PLL */
#define BOARD_POSC_HSMODE      1        /* High-speed crystal (HS) mode */

/* PLL configuration and resulting CPU clock.
 * CPU_CLOCK = ((POSC_FREQ / IDIV) * MULT) / ODIV
 */

#define BOARD_PLL_INPUT        BOARD_POSC_FREQ
#define BOARD_PLL_IDIV         5         /* PLL input divider */
#define BOARD_PLL_MULT         15        /* PLL multiplier */
#define BOARD_PLL_ODIV         1         /* PLL output divider */

#define BOARD_CPU_CLOCK        60000000 /* CPU clock (60MHz = (20MHz / 5) * 15 / 1) */

/* USB PLL configuration.
 * USB_CLOCK = ((POSC_XTAL / IDIV) * 24) / 2
 */

#define BOARD_UPLL_IDIV        5         /* USB PLL divider */
#define BOARD_USB_CLOCK        48000000  /* USB clock ((20MHz / 5) * 24) / 2 */

/* Peripheral clock is *not* divided down from CPU clock.
 * PBCLOCK = CPU_CLOCK / PBDIV
 */

#define BOARD_PBDIV            1        /* Peripheral clock divisor (PBDIV) */
#define BOARD_PBCLOCK          60000000 /* Peripheral clock (PBCLK = 60MHz/1) */

/* Watchdog pre-scaler (re-visit) */

#define BOARD_WD_ENABLE        0        /* Watchdog is disabled */
#define BOARD_WD_PRESCALER     8        /* Watchdog pre-scaler */

/* Timer 1 is a type A timer and is used as system clock.  It can be clocked
 * with either the SOSC or the PBCLOCK.  We will use the PBCLOCK because it
 * is much more accurate.
 */

#undef BOARD_TIMER1_SOSC

/* LED definitions **********************************************************/

/* The Sure DB_DP11215 PIC32 Storage Demo Board board has five LEDs.  One
 * (D4, lablel "Power") is not controllable by software.  Four are
 * controllable by software:
 *
 * D7  "USB"    Yellow  RD7 Low illuminates
 * D8  "SD"     Yellow  RD6 Low illuminates
 * D9  "Flash"  Yellow  RF0 Low illuminates
 * D10 "Error"  Red     RF1 Low illuminates
 */

                                  /* ON                  OFF               */

                                /* USB SD  FLASH ERROR USB SD  FLASH ERROR */
#define LED_STARTED          0  /* OFF OFF OFF   OFF   --- --- ---   ---   */
#define LED_HEAPALLOCATE     1  /* ON  OFF N/C   N/C   --- --- ---   ---   */
#define LED_IRQSENABLED      2  /* OFF ON  N/C   N/C   --- --- ---   ---   */
#define LED_STACKCREATED     3  /* ON  ON  N/C   N/C   --- --- ---   ---   */
#define LED_INIRQ            4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_SIGNAL           4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_ASSERTION        4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_PANIC            5  /* N/C N/C N/C   ON    N/C N/C N/C   OFF   */
#define LED_NVALUES          6

/* The Sure DB-DP11212 PIC32 General Purpose Demo Board does not have any
 * user controllable LEDs, but does does have a segment LED display.
 * That display is however, obscured by the larger segment display attached
 * to the board and, so, is not supported.
 */

/* For distinguishing individual LEDs */

#define LED_USB                0
#define LED_SD                 1
#define LED_FLASH              2
#define LED_ERROR              3

/* Button Definitions *******************************************************/

/* The Sure PIC32MX board has three buttons.
 *
 * SW1  (SW_UP, left arrow)          RB3 Pulled high, GND/low when depressed
 * SW2  (SW_DOWN, down/right arrow)  RB2 Pulled high, GND/low when depressed
 * SW3  (SW_OK, right arrow)         RB4 Pulled high, GND/low when depressed
 */

#define BUTTON_SW1             0
#define BUTTON_SW2             1
#define BUTTON_SW3             2
#define NUM_BUTTONS            3

#define BUTTON_SW1_BIT         (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT         (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT         (1 << BUTTON_SW3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MX_SURE_PIC32MX_INCLUDE_BOARD_H */
