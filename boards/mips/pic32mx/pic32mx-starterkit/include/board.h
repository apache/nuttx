/****************************************************************************
 * boards/mips/pic32mx/pic32mx-starterkit/include/board.h
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

#ifndef __BOARDS_MIPS_PIC32MX_PIC32MX_STARTERKIT_INCLUDE_BOARD_H
#define __BOARDS_MIPS_PIC32MX_PIC32MX_STARTERKIT_INCLUDE_BOARD_H

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
#define BOARD_POSC_HSMODE      1        /* High-speed crystal (HS) mode */

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

/* The PIC32MX Ethernet Starter kit has 3 user LEDs labeled LED1-3 on the
 * board graphics (but referred to as LED4-6 in the schematic):
 *
 * PIN User's Guide  Board Stencil  Notes
 * --- ------------- -------------- -------------------------
 * RD0 "User LED D4" "LED1 (RD0")   High illuminates (RED)
 * RD2 "User LED D5" "LED3 (RD2)"   High illuminates (YELLOW)
 * RD1 "User LED D6" "LED2 (RD1)"   High illuminates (GREEN)
 *
 * We will use the labels on the board to identify LEDs
 *
 * There are 5 additional LEDs available on the MEB (but not used by NuttX):
 *
 *   RD1          LED1
 *   RD2          LED2
 *   RD3          LED3
 *   RC1          LED4
 *   RC2          LED5
 */

/* LED index values for use with board_userled() */

#define PIC32MX_STARTERKIT_LED1     0
#define PIC32MX_STARTERKIT_LED2     1
#define PIC32MX_STARTERKIT_LED3     2
#define PIC32MX_STARTERKIT_NLEDS    3

/* LED bits for use with board_userled_all() */

#define PIC32MX_STARTERKIT_LED1_BIT (1 << PIC32MX_STARTERKIT_LED1)
#define PIC32MX_STARTERKIT_LED2_BIT (1 << PIC32MX_STARTERKIT_LED2)
#define PIC32MX_STARTERKIT_LED3_BIT (1 << PIC32MX_STARTERKIT_LED3)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 3 LEDs
 * on board the Ethernet Starter Kit.  The following definitions
 * describe how NuttX controls the LEDs:
 *
 *                           ON                  OFF
 * ------------------------- ---- ---- ---- ---- ---- ----
 *                           LED1 LED2 LED3 LED1 LED2 LED3
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

/* The PIC32 starter kit has 3 switches:
 *
 *   RD7            Switch SW2 (low when closed)
 *   RD6            Switch SW1 (low when closed)
 *   RD13           Switch SW3 (low when closed)
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
#endif /* __BOARDS_MIPS_PIC32MX_PIC32MX-STARTERKIT_INCLUDE_BOARD_H */
