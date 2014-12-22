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

/* Crystals on-board the DK-TM4C129X include:
 *
 * 1. 25.0MHz (Y2) is connected to OSC0/1 pins and is used as the run mode input to
 *    the PLL.
 * 2. 32.768kHz (Y3) connected to XOSC0/1 and clocks the hibernation module.
 */

#define SYSCON_RCC_XTAL      SYSCON_RCC_XTAL16000KHZ /* On-board crystal is 25 MHz */
#define XTAL_FREQUENCY       25000000

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

/* LED definitions ******************************************************************/
/* The DK-TM4C129X has a single RGB LED.  There is only one visible LED which
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
