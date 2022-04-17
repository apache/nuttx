/****************************************************************************
 * boards/arm/lpc43xx/lpc4357-evb/src/lpc43_autoleds.c
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
#include "lpc4357-evb.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(const char *msg)
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
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED pin as a GPIO outputs */

  led_dumppins("board_autoled_initialize() Entry)");

  /* Configure LED pin as a GPIO, then configure GPIO as an outputs */

  lpc43_pin_config(PINCONFIG_LED);
  lpc43_gpio_config(GPIO_LED);

  led_dumppins("board_autoled_initialize() Exit");
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledon = true;   /* OFF. Low illuminates */

  switch (led)
    {
      default:
      case 0:
        break;          /* LED OFF until state 1 */

      case 2:
        return;         /* LED no change */

      case 1:
      case 3:
        ledon = false;  /* LED ON.  Low illuminates */
        break;
    }

  /* Turn LED on or off, depending on state */

  lpc43_gpio_write(GPIO_LED, ledon);
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
      case 3:
        break;  /* LED OFF */

      case 2:
        return; /* LED no change */
    }

  /* LED OFF, Low illuminates */

  lpc43_gpio_write(GPIO_LED, true);
}

#endif /* CONFIG_ARCH_LEDS */
