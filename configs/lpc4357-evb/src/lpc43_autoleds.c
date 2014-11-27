/****************************************************************************
 * configs/lpc4357-evb/src/lpc43_autoleds.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "lpc4357-evb.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* LED definitions **********************************************************/
/* The LPC4357-EVB has one user-controllable LED labelled D6 controlled by
 * the signal LED_3V3:
 *
 *  ---- ------- -------------
 *  LED  SIGNAL  MCU
 *  ---- ------- -------------
 *   D6  LED_3V3 PE_7 GPIO7[7]
 *  ---- ------- -------------
 *
 * LED is grounded and a high output illuminates the LED.
 *
 * If CONFIG_ARCH_LEDS is defined, the LED will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change").
 *
 *   -------------------------- --------
 *                              LED
 *   -------------------------- --------
 *   LED_STARTED                OFF
 *   LED_HEAPALLOCATE           OFF
 *   LED_IRQSENABLED            OFF
 *   LED_STACKCREATED           ON
 *   LED_INIRQ                  NC
 *   LED_SIGNAL                 NC
 *   LED_ASSERTION              NC
 *   LED_PANIC                  Flashing
 *   -------------------------- --------
 */

/* Debug definitions ********************************************************/
/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg  lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define LED_VERBOSE 1
#    define ledvdbg lldbg
#  else
#    undef LED_VERBOSE
#    define ledvdbg(x...)
#  endif
#else
#  undef LED_VERBOSE
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(FAR const char *msg)
{
  lpc43_pin_dump(PINCONFIG_LED, msg);
  lpc43_gpio_dump(GPIO_LED, msg);
}
#else
#  define led_dumppins(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

void board_led_initialize(void)
{
  /* Configure LED pin as a GPIO outputs */

  led_dumppins("board_led_initialize() Entry)");

  /* Configure LED pin as a GPIO, then configure GPIO as an outputs */

  lpc43_pin_config(PINCONFIG_LED);
  lpc43_gpio_config(GPIO_LED);

  led_dumppins("board_led_initialize() Exit");
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
  switch (led)
    {
      default:
      case 0:
        lpc43_gpio_write(GPIO_LED, false);  /* LED OFF */
        break;

      case 2:                               /* LED no change */
        break;

      case 1:
      case 3:
        lpc43_gpio_write(GPIO_LED, true);   /* LED ON */
        break;
    }
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  switch (led)
    {
      default:
      case 0:
      case 1:
      case 2:
        break;                              /* LED no change */

      case 3:
        lpc43_gpio_write(GPIO_LED, false);  /* LED OFF */
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
