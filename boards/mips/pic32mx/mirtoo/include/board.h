/****************************************************************************
 * boards/mips/pic32mx/mirtoo/include/board.h
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

#ifndef __BOARDS_MIPS_PIC32MX_MIRTOO_INCLUDE_BOARD_H
#define __BOARDS_MIPS_PIC32MX_MIRTOO_INCLUDE_BOARD_H

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

/* The Mirtoo does not use an external crystal but relies instead on the
 * internal +/- 0.9% FRC clock.  That clock has a nomninal frequency of 8MHz.
 */

#define BOARD_FRC_FREQ         8000000  /* FRC nominal frequency(8MHz) */

/* Oscillator modes */

#define BOARD_FNOSC_FRCPLL     1        /* Use FRC w/PLL module */
#define BOARD_POSC_DISABLED    1        /* Disable primary oscillator */

/* PLL configuration and resulting CPU clock.
 * CPU_CLOCK = ((POSC_FREQ / IDIV) * MULT) / ODIV
 */

#define BOARD_PLL_INPUT        BOARD_FRC_FREQ
#define BOARD_PLL_IDIV         2        /* PLL input divider:  Input  = 4MHz */
#define BOARD_PLL_MULT         20       /* PLL multiplier:     PLL    = 80MHz */
#define BOARD_PLL_ODIV         2        /* PLL output divider: Output = 40MHz */

#define BOARD_CPU_CLOCK        40000000 /* CPU clock 40MHz = (((8MHz / 2) * 20) / 2) */

/* USB PLL configuration.
 * USB_CLOCK = ((POSC_XTAL / IDIV) * 24) / 2
 */

#define BOARD_UPLL_IDIV        2        /* USB PLL divider (revisit) */
#define BOARD_USB_CLOCK        48000000 /* USB clock (8MHz / 2) * 24 / 2) */

/* Peripheral clock is divided down from CPU clock.
 * PBCLOCK = CPU_CLOCK / PBDIV
 */

#define BOARD_PBDIV            1        /* Peripheral clock divisor (PBDIV) */
#define BOARD_PBCLOCK          40000000 /* Peripheral clock (PBCLK = 40MHz/1) */

/* Watchdog pre-scaler (re-visit) */

#define BOARD_WD_ENABLE        0        /* Watchdog is disabled */
#define BOARD_WD_PRESCALER     8        /* Watchdog pre-scaler */

/* LED definitions **********************************************************/

/* The Mirtoo module has 2 user LEDs labeled LED0 and LED1 in the schematics:
 *
 * ---  ----- --------------------------------------------------------------
 * PIN  Board Notes
 * ---  ----- --------------------------------------------------------------
 * RC8  LED0  Grounded, high value illuminates
 * RC9  LED1  Grounded, high value illuminates
 *
 * The Dimitech DTX1-4000L EV-kit1 supports 3 more LEDs, but there are not
 * controllable from software.
 */

/* LED index values for use with board_userled() */

#define PIC32MX_MIRTOO_LED0     0
#define PIC32MX_MIRTOO_LED1     1
#define PIC32MX_MIRTOO_NLEDS    2

/* LED bits for use with board_userled_all() */

#define PIC32MX_MIRTOO_LED0_BIT (1 << PIC32MX_MIRTOO_LED0)
#define PIC32MX_MIRTOO_LED1_BIT (1 << PIC32MX_MIRTOO_LED1)

/* If CONFIG_ARCH_LEDS is defined, then NuttX will control these LEDs as
 * follows:
 *                              ON        OFF
 * ------------------------- ---- ---- ---- ----
 *                           LED0 LED1 LED0 LED1
 * ------------------------- ---- ---- ---- ----
 * LED_STARTED            0  OFF  OFF  ---  ---
 * LED_HEAPALLOCATE       1  ON   OFF  ---  ---
 * LED_IRQSENABLED        2  OFF  ON   ---  ---
 * LED_STACKCREATED       3  ON   ON   ---  ---
 * LED_INIRQ              4  ON   N/C  OFF  N/C
 * LED_SIGNAL             4  ON   N/C  OFF  N/C
 * LED_ASSERTION          4  ON   N/C  OFF  N/C
 * LED_PANIC              4  ON   N/C  OFF  N/C
 */

#define LED_STARTED            0
#define LED_HEAPALLOCATE       1
#define LED_IRQSENABLED        2
#define LED_STACKCREATED       3
#define LED_INIRQ              4
#define LED_SIGNAL             4
#define LED_ASSERTION          4
#define LED_PANIC              4

#define LED_NVALUES            5

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
#endif /* __BOARDS_MIPS_PIC32MX_MIRTOO_INCLUDE_BOARD_H */
