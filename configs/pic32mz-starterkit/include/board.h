/****************************************************************************
 * configs/pic32mz-starterkit/include/board.h
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
 ****************************************************************************/

#ifndef __CONFIGS_SURE_PIC32MZ_INCLUDE_BOARD_H
#define __CONFIGS_SURE_PIC32MZ_INCLUDE_BOARD_H

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
/* Crystal frequencies
 *
 * - A 24 MHz oscillator circuit (Y4) is connected to the on-board
 *   microcontroller. This oscillator circuit functions as the controller’s
 *   primary oscillator. Depending on which is populated on the starter kit
 *   board, a 24 MHz crystal (Y1) may be used instead of Y4.
 * - The starter kit also has provisions for an external secondary 32 kHz
 *   oscillator (Y2); however, this is not populated.
 */

#define BOARD_POSC_FREQ        24000000  /* Primary OSC XTAL frequency (Y4, 24MHz) */
#define BOARD_SOSC_FREQ        32000     /* Secondary OSC XTAL frequency (Y2, 32KHz) */

/* Oscillator modes.
 *
 * - BOARD_POSC_ECMODE:  An external oscillator is connected to OSC1/OSC2
 * - BOARD_POSC_HSMODE:  An external crystal or resonator is connected to
 *                       OSC1/OSC2
 */

#define BOARD_FNOSC_SPLL       1         /* Use system PLL */
#define BOARD_POSC_ECMODE      1         /* External clock (EC) mode */
#define BOARD_POSC_SWITCH      1         /* Enable clock switching */
#undef  BOARD_POSC_FSCM                  /* Disable clock monitoring */

/* PLL configuration and resulting CPU clock.
 * CPU_CLOCK = ((POSC_FREQ / IDIV) * MULT) / ODIV
 */

#define BOARD_PLL_INPUT        BOARD_POSC_FREQ
#define BOARD_PLL_IDIV         3         /* PLL input divider */
#define BOARD_PLL_MULT         50        /* PLL multiplier */
#define BOARD_PLL_ODIV         2         /* PLL output divider */

#define BOARD_CPU_CLOCK        200000000 /* CPU clock: 200MHz = (24MHz / 3) * 50 / 2) */

/* Peripheral clocks */
/* PBCLK1
 *   Peripherals: OSC2 pin
 *
 * NOTES:
 *   - PBCLK1 is used by system modules and cannot be turned off
 *   - PBCLK1 divided by 2 is available on the OSC2 pin in certain clock
 *     modes.
 */

#define BOARD_PB1DIV           5         /* Divider = 5 */
#define BOARD_PBCLK1           40000000  /* PBCLK1 frequency = 200MHz/5 = 40MHz */

/* PBCLK2
 *   Peripherals: PMP, I2C, UART, SPI
 */

#define BOARD_PBCLK2_ENABLE    1         /* Enable PBCLK2 */
#define BOARD_PB2DIV           2         /* Divider = 2 */
#define BOARD_PBCLK2           100000000 /* PBCLK2 frequency = 200MHz/2 = 100MHz */

/* PBCLK3
 *   Peripherals: ADC, Comparator, Timers, Output Compare, Input Compare
 *
 * NOTES:
 *   - Timer 1 uses SOSC
 */

#define BOARD_PBCLK3_ENABLE    1         /* Enable PBCLK3 */
#define BOARD_PB3DIV           4         /* Divider = 4 */
#define BOARD_PBCLK3           50000000  /* PBCLK3 frequency = 200MHz/4 = 50MHz */

/* PBCLK4
 *   Peripherals: Ports
 */

#define BOARD_PBCLK4_ENABLE    1         /* Enable PBCLK4 */
#define BOARD_PB4DIV           2         /* Divider = 2 */
#define BOARD_PBCLK4           100000000 /* PBCLK4 frequency = 200MHz/2 = 100MHz */

/* PBCLK5
 *   Peripherals: Flash, Crypto, RND, USB, CAN, Ethernet, SQI
 *
 * NOTES:
 *   - PBCLK5 is used to fetch data from/to the Flash Controller, while the
 *     FRC clock is used for programming
 */

#define BOARD_PBCLK5_ENABLE    1         /* Enable PBCLK5 */
#define BOARD_PB5DIV           2         /* Divider = 2 */
#define BOARD_PBCLK5           100000000 /* PBCLK5 frequency = 200MHz/2 = 100MHz */


/* PBCLK6
 *   Peripherals:
 */

#undef BOARD_PBCLK6_ENABLE

/* PBCLK7
 *   Peripherals:  CPU, Deadman timer
 */

#undef BOARD_PBCLK7_ENABLE

/* PBCLK8
 *   Peripherals: EBI
 */

#undef BOARD_PBCLK8_ENABLE

/* Watchdog pre-scaler (re-visit) */

#define BOARD_WD_PRESCALER     1048576   /* Watchdog pre-scaler */

/* Ethernet MII clocking.
 *
 * The clock divider used to create the MII Management Clock (MDC).  The MIIM
 * module uses the PBCLK5 as an input clock.  According to the IEEE 802.3
 * Specification this should be no faster than 2.5 MHz. However, some PHYs
 * support clock rates up to 12.5 MHz.
 */

#define BOARD_EMAC_MIIM_DIV    40        /* Ideal: 100MHz/40 = 2.5MHz */

/* LED definitions **********************************************************/
/* LED Configuration ********************************************************/
/* The PIC32MZ Ethernet Starter kit has 3 user LEDs labelled LED1-3 on the
 * board:
 *
 *   PIN  LED   Notes
 *   ---  ----- -------------------------
 *   RH0  LED1  High illuminates (RED)
 *   RH1  LED3  High illuminates (YELLOW)
 *   RH2  LED2  High illuminates (GREEN)
 */

/* LED index values for use with pic32mz_setled() */

#define PIC32MZ_STARTERKIT_LED1     0
#define PIC32MZ_STARTERKIT_LED2     1
#define PIC32MZ_STARTERKIT_LED3     2
#define PIC32MZ_STARTERKIT_NLEDS    3

/* LED bits for use with pic32mz_setleds() */

#define PIC32MZ_STARTERKIT_LED1_BIT (1 << PIC32MZ_STARTERKIT_LED1)
#define PIC32MZ_STARTERKIT_LED2_BIT (1 << PIC32MZ_STARTERKIT_LED2)
#define PIC32MZ_STARTERKIT_LED3_BIT (1 << PIC32MZ_STARTERKIT_LED3)

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
/* The PIC32MZ Ethernet Starter kit has 3 user push buttons labelled SW1-3
 * on the board:
 *
 *   PIN   LED  Notes
 *   ----  ---- -------------------------
 *   RB12  SW1  Active-low
 *   RB13  SW2  Active-low
 *   RB14  SW3  Active-low
 *
 * The switches do not have any debounce circuitry and require internal pull-
 * up resistors. When Idle, the switches are pulled high (+3.3V), and they
 * are grounded when pressed.
 */

#define BUTTON_SW1             0
#define BUTTON_SW2             1
#define BUTTON_SW3             2
#define NUM_BUTTONS            3

#define BUTTON_SW1_BIT         (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT         (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT         (1 << BUTTON_SW3)

/* UARTS ********************************************************************/
/*  MEB-II
 *
 * By default, the UART1 is configured for the pins used by the MEB-II
 * board.  The UART1 signals are available at the MEB-II PICTail
 * connector:
 *
 *   --------------- --------- -------------- ------------
 *   PIC32MZ PIN     CONNECTOR MEB-II PIN     PICTAIL PIN
 *   FUNCTION        J1        NAME           J2
 *   --------------- --------- -------------- ------------
 *   RPA14/SCL1/RA14 124       SCL1/TOUCH_SCL 4
 *   RPA15/SDA1/RA15 126       SDA1/TOUCH_SDA 6
 *                             +3.3V          1,26
 *                             GND            28
 *   --------------- --------- -------------- ------------
 *
 * The following pin assignment is used with the MEB-II board.  If you are
 * using signals from PIC32MZEC Adaptor Board (as described in the the README
 * file), then UART1 signals are available at these locations on the adaptor
 * board:
 *
 *   JP7 Pin 2: RPC14
 *   JP8 Pin 2: RPB3
 *
 * And the following should be changed to:
 *
 *  #define BOARD_U1RX_PPS  U1RXR_RPC14
 *  #define BOARD_U1TX_PPS  U1TX_RPB3R
 */

#define BOARD_U1RX_PPS  U1RXR_RPA14
#define BOARD_U1TX_PPS  U1TX_RPA15R

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

/****************************************************************************
 * Name:  pic32mz_ledinit and pic32mz_setled
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the following interfaces
 *   are available to control the LEDs from user applications.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void pic32mz_ledinit(void);
void pic32mz_setled(int led, bool ledon);
void pic32mz_setleds(uint8_t ledset);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SURE_PIC32MZ_INCLUDE_BOARD_H */
