/************************************************************************************
 * configs/tm4c123g-launchpad/include/board.h
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

#ifndef __CONFIGS_TMC4C123G_LAUNCHPAD_INCLUDE_BOARD_H
#define __CONFIGS_TMC4C123G_LAUNCHPAD_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Definitions
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
 *   BOARD_LED_R    -- Connected to PF1
 *   BOARD_LED_G    -- Connected to PF3
 *   BOARD_LED_B    -- Connected to PF2
 */

/* LED index values for use with tm4c_setled() */

#define BOARD_LED_R               0
#define BOARD_LED_G               1
#define BOARD_LED_B               2
#define BOARD_NLEDS               3

/* LED bits for use with tm4c_setleds() */

#define BOARD_LED1_BIT            (1 << BOARD_LED1)
#define BOARD_LED2_BIT            (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDS is defined, then automated support for the LaunchPad LEDs
 * will be included in the build:
 *
 * OFF:
 * - OFF means that the OS is still initializing. Initialization is very fast so
 *   if you see this at all, it probably means that the system is hanging up
 *   somewhere in the initialization phases.
 *
 * GREEN or GREEN-ish
 * - This means that the OS completed initialization.
 *
 * Bluish:
 * - Whenever and interrupt or signal handler is entered, the BLUE LED is
 *   illuminated and extinguished when the interrupt or signal handler exits.
 *   This will add a BLUE-ish tinge to the LED.
 *
 * Redish:
 * - If a recovered assertion occurs, the RED component will be illuminated
 *   briefly while the assertion is handled.  You will probably never see this.
 *
 * Flashing RED:
 * - In the event of a fatal crash, the BLUE and GREEN components will be
 *   extinguished and the RED component will FLASH at a 2Hz rate.
 */
                                /* RED  GREEN BLUE               */
#define LED_STARTED       0     /* OFF  OFF   OFF                */
#define LED_HEAPALLOCATE  0     /* OFF  OFF   OFF                */
#define LED_IRQSENABLED   0     /* OFF  OFF   OFF                */
#define LED_STACKCREATED  1     /* OFF  ON    OFF                */
#define LED_INIRQ         2     /* NC   NC    ON  (momentary)    */
#define LED_SIGNAL        2     /* NC   NC    ON  (momentary)    */
#define LED_ASSERTION     3     /* ON   NC    NC  (momentary)    */
#define LED_PANIC         4     /* ON   OFF   OFF (flashing 2Hz) */

/* LED definitions ******************************************************************/
/* The TMC4C123G LaunchPad has a two buttons:
 *
 *   BOARD_SW1    -- Connected to PF4
 *   BOARD_SW2    -- Connected to PF0
 */

#define BUTTON_SW1        0
#define BUTTON_SW2        1
#define NUM_BUTTONS       2

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)

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
 *   point is called early in the intitialization -- after all memory has been
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
#endif  /* __CONFIGS_TMC4C123G_LAUNCHPAD_INCLUDE_BOARD_H */
