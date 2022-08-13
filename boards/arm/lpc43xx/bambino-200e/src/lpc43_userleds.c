/****************************************************************************
 * boards/arm/lpc43xx/bambino-200e/src/lpc43_userleds.c
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

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "bambino-200e.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The LPC4330-Xplorer has 2 user-controllable LEDs labeled D2 an D3 in the
 * schematic and on but referred to has LED1 and LED2 here, respectively.
 *
 *  LED1   D2  GPIO1[12]
 *  LED2   D3  GPIO1[11]
 *
 * LEDs are pulled high to a low output illuminates the LED.
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
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure all LED GPIO lines */

  led_dumppins("board_userled_initialize() Entry)");

  /* Configure LED pins as GPIOs, then configure GPIOs as outputs */

  lpc43_pin_config(PINCONFIG_LED1);
  lpc43_gpio_config(GPIO_LED1);

  lpc43_pin_config(PINCONFIG_LED2);
  lpc43_gpio_config(GPIO_LED2);

  led_dumppins("board_userled_initialize() Exit");
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint16_t gpiocfg = (led == BOARD_LED1 ? GPIO_LED1 : GPIO_LED2);
  lpc43_gpio_write(gpiocfg, !ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  lpc43_gpio_write(GPIO_LED1, (ledset & BOARD_LED1_BIT) == 0);
  lpc43_gpio_write(GPIO_LED2, (ledset & BOARD_LED2_BIT) == 0);
}

#endif /* !CONFIG_ARCH_LEDS */
