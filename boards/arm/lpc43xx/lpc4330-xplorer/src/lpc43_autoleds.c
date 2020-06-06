/****************************************************************************
 * boards/arm/lpc43xx/lpc4330-xplorer/src/lpc43_autoleds.c
 *
 *   Copyright (C) 2012-2013, 2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc4330-xplorer.h"

#ifdef CONFIG_ARCH_LEDS

/* LED definitions **********************************************************/

/* The LPC4330-Xplorer has 2 user-controllable LEDs labeled D2 an D3 in the
 * schematic and on but referred to has LED1 and LED2 here, respectively.
 *
 *  LED1   D2  GPIO1[12]
 *  LED2   D3  GPIO1[11]
 *
 * LEDs are pulled high to a low output illuminates the LED.
 *
 * If CONFIG_ARCH_LEDS is defined, the LEDs will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change").
 *
 *                                      ON            OFF
 *                                  LED1   LED2   LED1   LED2
 *   LED_STARTED                0   OFF    OFF     -      -
 *   LED_HEAPALLOCATE           1   ON     OFF     -      -
 *   LED_IRQSENABLED            1   ON     OFF     -      -
 *   LED_STACKCREATED           1   ON     OFF     -      -
 *   LED_INIRQ                  2   NC     ON      NC     OFF
 *   LED_SIGNAL                 2   NC     ON      NC     OFF
 *   LED_ASSERTION              2   NC     ON      NC     OFF
 *   LED_PANIC                  2   NC     ON      NC     OFF
 *
 * If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
 * control of the application.  The following interfaces are then available
 * for application control of the LEDs:
 *
 *  uint32_t board_userled_initialize(void);
 *  void board_userled(int led, bool ledon);
 *  void board_userled_all(uint32_t ledset);
 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(FAR const char *msg)
{
  lpc43_pin_dump(PINCONFIG_LED1, msg);
  lpc43_gpio_dump(GPIO_LED2, msg);
}
#else
#  define led_dumppins(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure all LED pins as GPIO outputs */

  led_dumppins("board_autoled_initialize() Entry)");

  /* Configure LED pins as GPIOs, then configure GPIOs as outputs */

  lpc43_pin_config(PINCONFIG_LED1);
  lpc43_gpio_config(GPIO_LED1);

  lpc43_pin_config(PINCONFIG_LED2);
  lpc43_gpio_config(GPIO_LED2);

  led_dumppins("board_autoled_initialize() Exit");
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      default:
      case 0:
        lpc43_gpio_write(GPIO_LED1, true);   /* LED1 OFF */
        lpc43_gpio_write(GPIO_LED2, true);   /* LED2 OFF */
        break;

      case 1:
        lpc43_gpio_write(GPIO_LED1, false);  /* LED1 ON */
        lpc43_gpio_write(GPIO_LED2, true);   /* LED2 OFF */
        break;

      case 2:
        lpc43_gpio_write(GPIO_LED2, false);  /* LED2 ON */
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      default:
      case 0:
      case 1:
        break;

      case 2:
        lpc43_gpio_write(GPIO_LED2, true);  /* LED2 OFF */
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
