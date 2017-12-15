/****************************************************************************
 * configs/lpcxpresso-lpc54628/src/lpcxpresso-lpc54628.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef _CONFIGS_LPCXPRESSO_LPC54628_SRC_LPCXPRESSO_LPC54628_H
#define _CONFIGS_LPCXPRESSO_LPC54628_SRC_LPCXPRESSO_LPC54628_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "lpc54_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAVE_I2CTOOL  1

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER) || \
    !defined(HAVE_I2C_MASTER_DEVICE)
#  undef HAVE_I2CTOOL
#endif

/* LED definitions **********************************************************/
/* The LPCXpress-LPC54628 has three user LEDs: D9, D11, and D12.  These
 * LEDs are for application use. They are illuminated when the driving
 * signal from the LPC546xx is low. The LEDs are driven by ports P2-2 (D9),
 * P3-3 (D11) and P3-14 (D12).
 */

#define GPIO_LED_D9 \
  (GPIO_PORT2 | GPIO_PIN2 | GPIO_VALUE_ONE | GPIO_OUTPUT | \
   GPIO_MODE_DIGITAL | GPIO_FILTER_OFF | GPIO_PUSHPULL | GPIO_PULLUP)

#define GPIO_LED_D11 \
  (GPIO_PORT3 | GPIO_PIN3 | GPIO_VALUE_ONE | GPIO_OUTPUT | \
   GPIO_MODE_DIGITAL | GPIO_FILTER_OFF | GPIO_PUSHPULL | GPIO_PULLUP)

#define GPIO_LED_D12 \
  (GPIO_PORT3 | GPIO_PIN14 | GPIO_VALUE_ONE | GPIO_OUTPUT | \
   GPIO_MODE_DIGITAL | GPIO_FILTER_OFF | GPIO_PUSHPULL | GPIO_PULLUP)

/* Button definitions *******************************************************/
/* The LPCXpresso has four switches:
 *
 *   SW2 ISP2         P0.6
 *   SW3 ISP1         P0.5
 *   SW4 ISP0         P0.4
 *   SW5 User Button  P1.1
 *
 * In all cased, the signal is low when the button is pressed.
 *
 * SW2, SW3, SW4 can be used to force the LPC546xx in to ISP boot modes.
 * After boot these buttons could be used as user buttons.  However, they are
 * not available when the on-board SRDRAM is used because P0.4, P0.5, and
 * P0.6 are also used as EMC_D2, EMC_D3, and EMC_D4, respectively.
 *
 * So SW5 is really the only button that that is generally available for
 * software usage.
 */

/* LCD/TSC definitions ******************************************************/
/* The backlight is controlled by P3.31 and is intended to connect via PWM
 * to control the brightness level.  For simplicity here, it configured as a
 * simple GPIO output.
 *
 * The output goes to the enable (EN) pin of a AP5724 step-up DC/DC
 * converter designed to drive white LEDs with a constant current.  A high
 * input at EN turns the converter on, and a low input turns it off.
 */

#define GPIO_LCD_BL \
  (GPIO_PORT3 | GPIO_PIN31 | GPIO_VALUE_ZERO | GPIO_OUTPUT | \
   GPIO_MODE_DIGITAL | GPIO_FILTER_OFF | GPIO_PUSHPULL | GPIO_PULLUP)

/* The integrated touchscreen uses one GPIO out and one GPIO interrupting
 * GPIO input:
 *
 *   P2.27  CT_RSTn
 *   P4.0   INTR
 */

#define GPIO_LCD_CTRSTn \
  (GPIO_PORT2 | GPIO_PIN27 | GPIO_VALUE_ZERO | GPIO_OUTPUT | \
   GPIO_MODE_DIGITAL | GPIO_FILTER_OFF | GPIO_PUSHPULL | GPIO_PULLUP)

/* I2C addresses (7-bit): */

#define CODEC_I2C_ADDRESS   0x1a
#define ACCEL_I2C_ADDRESS   0x1d
#define TSC_I2C_ADDRESS     0x38

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int lpc54_bringup(void);

/****************************************************************************
 * Name: lpc54_sdram_initialize
 *
 * Description:
 *   Initialize external SDRAM
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_EMC
void lpc54_sdram_initialize(void);
#endif

/****************************************************************************
 * Name: lpc54_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ****************************************************************************/

void lpc54_lcd_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_LPCXPRESSO_LPC54628_SRC_LPCXPRESSO_LPC54628_H */
