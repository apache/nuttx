/************************************************************************************
 * configs/dk-tm4c129x/include/board.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_DK_TM4C129X_INCLUDE_BOARD_H
#define __CONFIGS_DK_TM4C129X_INCLUDE_BOARD_H

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

/* RCC settings.  Crystals on-board the TMC4C123G LaunchPad include:
 *
 *   16MHz connected to OSC0/1 (pins 40/41)
 *   32.768kHz connected to XOSC0/1 (pins 34/36)
 */

#define SYSCON_RCC_XTAL      SYSCON_RCC_XTAL16000KHZ /* On-board crystal is 16 MHz */
#define XTAL_FREQUENCY       16000000

/* Oscillator source is the main oscillator */

#define SYSCON_RCC_OSCSRC    SYSCON_RCC_OSCSRC_MOSC
#define SYSCON_RCC2_OSCSRC   SYSCON_RCC2_OSCSRC2_MOSC
#define OSCSRC_FREQUENCY     XTAL_FREQUENCY

/* Use system divider = 4; this corresponds to a system clock frequency
 * of (400 / 1) / 5 = 80MHz (Using RCC2 and DIV400).
 */

#define TIVA_SYSDIV          5
#define SYSCLK_FREQUENCY     80000000  /* 80MHz */

/* Other RCC settings:
 *
 * - Main and internal oscillators enabled.
 * - PLL and sys dividers not bypassed
 * - PLL not powered down
 * - No auto-clock gating reset
 */

#define TIVA_RCC_VALUE (SYSCON_RCC_OSCSRC | SYSCON_RCC_XTAL | \
                      SYSCON_RCC_USESYSDIV | SYSCON_RCC_SYSDIV(TIVA_SYSDIV))

/* RCC2 settings
 *
 * - PLL and sys dividers not bypassed.
 * - PLL not powered down
 * - Not using RCC2
 *
 * When SYSCON_RCC2_DIV400 is not selected, SYSDIV2 is the divisor-1.
 * When SYSCON_RCC2_DIV400 is selected, SYSDIV2 is the divisor-1)/2, plus
 * the LSB:
 *
 * SYSDIV2 SYSDIV2LSB DIVISOR
 *   0       N/A         2
 *   1       0           3
 *   "       1           4
 *   2       0           5
 *   "       1           6
 *   etc.
 */

#if (TIVA_SYSDIV & 1) == 0
#  define TIVA_RCC2_VALUE (SYSCON_RCC2_OSCSRC | SYSCON_RCC2_SYSDIV2LSB | \
                           SYSCON_RCC2_SYSDIV_DIV400(TIVA_SYSDIV) | \
                           SYSCON_RCC2_DIV400 | SYSCON_RCC2_USERCC2)
#else
#  define TIVA_RCC2_VALUE (SYSCON_RCC2_OSCSRC | SYSCON_RCC2_SYSDIV_DIV400(TIVA_SYSDIV) | \
                           SYSCON_RCC2_DIV400 | SYSCON_RCC2_USERCC2)
#endif

/* LED definitions ******************************************************************/
/* The TMC4C123G LaunchPad has a single RGB LED.  There is only one visible LED which
 * will vary in color.  But, from the standpoint of the firmware, this appears as
 * three LEDs:
 *
 *   --- ------------ -----------------
 *   Pin Pin Function Jumper
 *   --- ------------ -----------------
 *   PN5 Red LED      J36 pins 1 and 2
 *   PQ4 Blue LED     J36 pins 3 and 4
 *   PQ7 Green LED    J36 pins 5 and 6
 *   --- ------------ -----------------
 */

/* LED index values for use with tm4c_setled() */

#define BOARD_LED_R               0
#define BOARD_LED_G               1
#define BOARD_LED_B               2
#define BOARD_NLEDS               3

/* LED bits for use with tm4c_setleds() */

#define BOARD_LED_R_BIT           (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT           (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT           (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDS is defined, then automated support for the DK-TM4C129X LED
 * will be included in the build:
 */
                                /* RED  GREEN BLUE */
#define LED_STARTED       0     /* OFF  OFF   ON   */
#define LED_HEAPALLOCATE  1     /* NC   NC    NC   */
#define LED_IRQSENABLED   1     /* NC   NC    NC   */
#define LED_STACKCREATED  2     /* OFF  ON    OFF  */
#define LED_INIRQ         1     /* NC   NC    NC   */
#define LED_SIGNAL        1     /* NC   NC    NC   */
#define LED_ASSERTION     1     /* NC   NC    NC   */
#define LED_PANIC         3     /* ON   OFF   OFF (flashing 2Hz) */

/* Button definitions ***************************************************************/
/* There are three push buttons on the board.
 *
 *   --- ------------ -----------------
 *   Pin Pin Function Jumper
 *   --- ------------ -----------------
 *   PP1 Select SW4   J37 pins 1 and 2
 *   PN3 Up SW2       J37 pins 3 and 4
 *   PE5 Down SW3     J37 pins 5 and 6
 *   --- ------------ -----------------
 */

#define BUTTON_SW2        0
#define BUTTON_SW3        1
#define BUTTON_SW4        2
#define NUM_BUTTONS       3

#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)
#define BUTTON_SW4_BIT    (1 << BUTTON_SW4)

/* Pin Multiplexing Disambiguation **************************************************/

#define GPIO_UART1_CTS    GPIO_UART1_CTS_1
#define GPIO_UART1_RTS    GPIO_UART1_RTS_1
#define GPIO_UART1_RX     GPIO_UART1_RX_1
#define GPIO_UART1_TX     GPIO_UART1_TX_1

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: tiva_boardinitialize
 *
 * Description:
 *   All Tiva architectures must provide the following entry point.  This entry
 *   point is called early in the initialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void tiva_boardinitialize(void);

/************************************************************************************
 * Name:  tm4c_ledinit, tm4c_setled, and tm4c_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LED.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfaces are available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void tm4c_ledinit(void);
void tm4c_setled(int led, bool ledon);
void tm4c_setleds(uint8_t ledset);
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_DK_TM4C129X_INCLUDE_BOARD_H */
