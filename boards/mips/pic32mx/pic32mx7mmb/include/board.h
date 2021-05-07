/****************************************************************************
 * boards/mips/pic32mx/pic32mx7mmb/include/board.h
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

#ifndef __BOARDS_MIPS_PIC32MX_PIC32MX7MMB_INCLUDE_BOARD_H
#define __BOARDS_MIPS_PIC32MX_PIC32MX7MMB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Clocking *****************************************************************/

/* Crystal frequencies */

#define BOARD_POSC_FREQ        8000000  /* Primary OSC XTAL frequency (8MHz) */
#define BOARD_SOSC_FREQ        32768    /* Secondary OSC XTAL frequency (32.768KHz) */

/* Oscillator modes */

#define BOARD_FNOSC_POSCPLL    1        /* Use primary oscillator w/PLL */
#define BOARD_POSC_XTMODE      1        /* Resonator, crystal or resonator (XT) mode */
#undef  BOARD_POSC_SWITCH               /* Use FRC until POSC stabilizes, then switch */
#undef  BOARD_POSC_FSCM                 /* Switch to FRC if POSC fails */
#define BOARD_SOSC_ENABLE      1        /* Enable Secondary Oscillator */
#define BOARD_SOSC_IESO        1        /* Internal External Switchover mode is enabled */

/* PLL configuration and resulting CPU clock.
 * CPU_CLOCK = ((POSC_FREQ / IDIV) * MULT) / ODIV
 */

#define BOARD_PLL_INPUT        BOARD_POSC_FREQ
#define BOARD_PLL_IDIV         2        /* PLL input divider */
#define BOARD_PLL_MULT         20       /* PLL multiplier */
#define BOARD_PLL_ODIV         1        /* PLL output divider */

#define BOARD_CPU_CLOCK        80000000 /* CPU clock (80MHz = 8MHz * 20 / 2) */

/* USB PLL configuration.
 * USB_CLOCK = ((POSC_XTAL / IDIV) * 24) / 2
 */

#define BOARD_UPLL_IDIV        2        /* USB PLL divider (revisit) */
#define BOARD_USB_CLOCK        48000000 /* USB clock (8MHz / 2) * 24 / 2) */

/* Peripheral clock is divided down from CPU clock.
 * PBCLOCK = CPU_CLOCK / PBDIV
 */

#define BOARD_PBDIV            2        /* Peripheral clock divisor (PBDIV) */
#define BOARD_PBCLOCK          40000000 /* Peripheral clock (PBCLK = 80MHz/2) */

/* Watchdog pre-scaler (re-visit) */

#define BOARD_WD_ENABLE        0        /* Watchdog is disabled */
#define BOARD_WD_PRESCALER     8        /* Watchdog pre-scaler */

/* Ethernet MII clocking.
 *
 * The clock divider used to create the MII Management Clock (MDC).  The MIIM
 * module uses the SYSCLK as an input clock.  According to the IEEE 802.3
 * Specification this should be no faster than 2.5 MHz. However, some PHYs
 * support clock rates up to 12.5 MHz.
 */

#define BOARD_EMAC_MIIM_DIV    32        /* Ideal: 80MHz/32 = 2.5MHz */

/* LED definitions **********************************************************/

/* LED Configuration ********************************************************/

/* The Mikroelektronika PIC32MX7 MMB has 3 user LEDs labeled LED0-2 in the
 * schematics:
 *
 * ---  ----- --------------------------------------------------------------
 * PIN  Board Notes
 * ---  ----- --------------------------------------------------------------
 * RA0  LED0  Pulled-up, low value illuminates
 * RA1  LED1  Pulled-up, low value illuminates
 * RD9  LED2  Pulled-up, low value illuminates
 * RA9  LED4  Not available for general use*, indicates MMC/SD activity
 * ---  LED5  Not controllable by software, indicates power-on
 *
 * * RA9 is also the SD chip select.  It will illuminate whenever the SD card
 *   is selected.  If SD is not used, then LED4 could also be used as a user-
 *   controlled LED.
 */

/* LED index values for use with board_userled() */

#define PIC32MX_PIC32MX7MMB_LED0     0
#define PIC32MX_PIC32MX7MMB_LED1     1
#define PIC32MX_PIC32MX7MMB_LED2     2
#define PIC32MX_PIC32MX7MMB_NLEDS    3

/* LED bits for use with board_userled_all() */

#define PIC32MX_PIC32MX7MMB_LED0_BIT (1 << PIC32MX_PIC32MX7MMB_LED0)
#define PIC32MX_PIC32MX7MMB_LED1_BIT (1 << PIC32MX_PIC32MX7MMB_LED1)
#define PIC32MX_PIC32MX7MMB_LED2_BIT (1 << PIC32MX_PIC32MX7MMB_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 3 LEDs
 * on board the Mikroelektronika PIC32MX7 MMB.  The following definitions
 * describe how NuttX controls the LEDs:
 *
 *                           ON                  OFF
 * ------------------------- ---- ---- ---- ---- ---- ----
 *                           LED0 LED1 LED2 LED0 LED1 LED2
 * ------------------------- ---- ---- ---- ---- ---- ----
 * LED_STARTED            0  OFF  OFF  OFF  ---  ---  ---
 * LED_HEAPALLOCATE       1  ON   OFF  N/C  ---  ---  ---
 * LED_IRQSENABLED        2  OFF  ON   N/C  ---  ---  ---
 * LED_STACKCREATED       3  ON   ON   N/C  ---  ---  ---
 * LED_INIRQ              4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_SIGNAL             4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_ASSERTION          4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_PANIC              5  ON   N/C  N/C  OFF  N/C  N/C
 */

#define LED_STARTED            0
#define LED_HEAPALLOCATE       1
#define LED_IRQSENABLED        2
#define LED_STACKCREATED       3
#define LED_INIRQ              4
#define LED_SIGNAL             4
#define LED_ASSERTION          4
#define LED_PANIC              5

#define LED_NVALUES            6

/* Switch definitions *******************************************************/

/* The Mikroelektronika PIC32MX7 MMB has a joystick:
 *
 * ------ -------- --------------------------
 *  GPIO   SIGNAL  BOARD CONNECTION
 * ------ -------- --------------------------
 *   RB0   JOY-A   Joystick A, HDR1 pin 24
 *   RB2   JOY-C   Joystick C, HDR1 pin 22
 *   RB1   JOY-B   Joystick B, HDR1 pin 23
 *   RB3   JOY-D   Joystick D, HDR1 pin 21
 *   RA10  JOY-CP  Joystick CP, HDR1 pin 25
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MX_PIC32MX7MMB_INCLUDE_BOARD_H */
