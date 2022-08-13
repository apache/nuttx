/****************************************************************************
 * boards/arm/lpc43xx/lpc4357-evb/src/lpc43_userleds.c
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
#include "lpc4357-evb.h"

#ifndef CONFIG_ARCH_LEDS

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
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED pin as a GPIO outputs */

  led_dumppins("board_userled_initialize() Entry)");

  /* Configure LED pin as a GPIO, then configure GPIO as an outputs */

  lpc43_pin_config(PINCONFIG_LED);
  lpc43_gpio_config(GPIO_LED);

  led_dumppins("board_userled_initialize() Exit");
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if (led == BOARD_LED)
    {
      lpc43_gpio_write(GPIO_LED, !ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  lpc43_gpio_write(GPIO_LED, (ledset & BOARD_LED_BIT) == 0);
}

#endif /* !CONFIG_ARCH_LEDS */
