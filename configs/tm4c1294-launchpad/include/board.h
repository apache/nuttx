/************************************************************************************
 * configs/tm4c1294-launchpad/include/board.h
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

#ifndef __CONFIGS_TM4C1294_LAUNCHPAD_INCLUDE_BOARD_H
#define __CONFIGS_TM4C1294_LAUNCHPAD_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* Crystals on-board the EK-TM4C1294XL include:
 *
 * 1. 25.0MHz (Y2) is connected to OSC0/1 pins and is used as the run mode input to
 *    the PLL.
 * 2. 32.768kHz (Y3) connected to XOSC0/1 and clocks the hibernation module.
 */

#define SYSCON_RCC_XTAL      SYSCON_RCC_XTAL16000KHZ /* On-board crystal is 25 MHz */
#define XTAL_FREQUENCY       25000000

/* Frequencies of other clock sources */

#define PIOSC_FREQUENCY      16000000 /* Precision internal oscillator */
#define RTCOSC_FREQUENCY     32768    /* Hibernation module RTC oscillator */
#define LFIOSC_FREQUENCY     33000    /* Low frequency internal oscillator */

/* The PLL generates Fvco according to the following formulae.  The input clock to
 * the PLL may be either the external crystal (Fxtal) or PIOSC (Fpiosc).  This
 * logic supports only the external crystal as the PLL source clock.
 *
 *   Fin  = Fxtal / (Q + 1 )(N + 1) -OR- Fpiosc / (Q + 1)(N + 1)
 *   Mdiv = Mint + (MFrac / 1024)
 *   Fvco = Fin * Mdiv
 *
 * Where the register fields Q and N actually hold (Q-1) and (N-1).   The following
 * setup then generates Fvco = 480MHz:
 *
 *   Fin  = 25 MHz / 1 / 5 = 5 MHz
 *   Mdiv = 96
 *   Fvco = 480
 */

#define BOARD_PLL_MINT       96        /* Integer part of PLL M value */
#define BOARD_PLL_MFRAC      0         /* Fractional part of PLL M value */
#define BOARD_PLL_N          5         /* PLL N value */
#define BOARD_PLL_Q          1         /* PLL Q value */

#define BOARD_FVCO_FREQUENCY 480000000 /* Resulting Fvco */

/* When the PLL is active, the system clock frequency (SysClk) is calculated using
 * the following equation:
 *
 *   SysClk = Fvco/ (sysdiv + 1)
 *
 * The following setup generates Sysclk = 120MHz:
 */

#define BOARD_PLL_SYSDIV     4         /* Sysclk = Fvco / 4 = 120MHz */
#define SYSCLK_FREQUENCY     120000000 /* Resulting SysClk frequency */

/* Alternate Clock (ALTCLK)
 *
 * The ALTCLK provides a clock source of numerous frequencies to the general-purpose
 * timer, SSI, and UART modules.  The default source for the ALTCLK is the Precision
 * Internal Oscillator (PIOSC).  The Hibernation Real-time Clock (RTCOSC) and Low
 * Frequency Internal Oscillator (LFIOSC) are alternatives.  If the RTCOSC Output is
 * selected, the clock source must also be enabled in the Hibernation module.
 */

#define BOARD_ALTCLKCFG      SYSCON_ALTCLKCFG_ALTCLK_PIOSC
#define ALTCLK_FREQUENCY     PIOSC_FREQUENCY

/* LED definitions ******************************************************************/
/* The EK-TM4C1294XL has 5 green LEDs.
 * LED D0 is lit when 3.3V power supply is available .
 * LEDs D1 and D2 are general purpose LEDs.
 * LEDs D3 and D4 can be used for Ethernet link and activity.
 *
 *   --- ------------
 *   Pin Pin Function
 *   --- ------------
 *   PN1 green LED D1
 *   PN0 green LED D2
 *   PF4 green LED D3
 *   PF0 green LED D4
 *   --- ------------
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_D1      0
#define BOARD_LED_D2      1
#define BOARD_LED_D3      2
#define BOARD_LED_D4      3
#define BOARD_NLEDS       4

/* LED bits for use with board_userled_all() */

#define BOARD_LED_D1_BIT  (1 << BOARD_LED_D1)
#define BOARD_LED_D2_BIT  (1 << BOARD_LED_D2)
#define BOARD_LED_D3_BIT  (1 << BOARD_LED_D3)
#define BOARD_LED_D4_BIT  (1 << BOARD_LED_D4)

/* If CONFIG_ARCH_LEDS is defined, then automated support for the EK-TM4C1294XL LED
 * will be included in the build:
 */
                                /* LED1  LED2  LED3  LED4 */
#define LED_STARTED       1     /* ON    OFF   NC    NC   */
#define LED_HEAPALLOCATE  0     /* NC    NC    NC    NC   */
#define LED_IRQSENABLED   0     /* NC    NC    NC    NC   */
#define LED_STACKCREATED  2     /* ON    ON    NC    NC   */
#define LED_INIRQ         0     /* NC    NC    NC    NC   */
#define LED_SIGNAL        0     /* NC    NC    NC    NC   */
#define LED_ASSERTION     0     /* NC    NC    NC    NC   */
#define LED_PANIC         3     /* OFF   ON    NC    NC   (flashing 2Hz) */

/* Button definitions ***************************************************************/
/* There are four push buttons on the board. Two of them are user controllable.
 * The others are RESET and WAKE
 *
 *   --- ------------
 *   Pin Pin Function
 *   --- ------------
 *   PJ0 USR_SW1
 *   PJ1 USR_SW2
 *   --- ------------
 */

#define BUTTON_SW1        0
#define BUTTON_SW2        1
#define NUM_BUTTONS       2

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)

/* Pin Multiplexing Disambiguation **************************************************/
/* Ethernet LEDs
 *
 *    PF0/PF4/PF1 are used for Ethernet LEDs.
 *      PF0/EN0LED0/LED4  Ethernet link OK
 *      PF4/EN0LED1/LED3  Ethernet TX/RX activity
 *      PF1/EN0LED2       Ethernet 100-base TX
 */

#define GPIO_EN0_LED0     GPIO_EN0_LED0_1
#define GPIO_EN0_LED1     GPIO_EN0_LED1_1
#define GPIO_EN0_LED2     GPIO_EN0_LED2_1

#endif  /* __CONFIGS_TM4C1294_LAUNCHPAD_INCLUDE_BOARD_H */
