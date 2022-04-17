/****************************************************************************
 * boards/arm/lpc43xx/bambino-200e/src/lpc43_autoleds.c
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
#include "arm_internal.h"
#include "bambino-200e.h"

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
static void led_dumppins(const char *msg)
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
