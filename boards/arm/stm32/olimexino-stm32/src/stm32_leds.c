/****************************************************************************
 * boards/arm/stm32/olimexino-stm32/src/stm32_leds.c
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

#include "stm32.h"
#include "olimexino-stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) stm32_dumpgpio(GPIO_LED_GREEN, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
static bool g_initialized = false;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_led_initialize/board_userled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
uint32_t stm32_led_initialize(void)
#else
uint32_t board_userled_initialize(void)
#endif
{
  /* Configure all LED GPIO lines */

  led_dumpgpio("board_*led_initialize() Entry)");

  stm32_configgpio(GPIO_LED_YELLOW);
  stm32_configgpio(GPIO_LED_GREEN);

  led_dumpgpio("board_*led_initialize() Exit");
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled(int led, bool ledon)
{
  if (led == BOARD_LED_GREEN)
    {
      stm32_gpiowrite(GPIO_LED_GREEN, !ledon);
    }
  else if (led == BOARD_LED_YELLOW)
    {
      stm32_gpiowrite(GPIO_LED_YELLOW, !ledon);
    }
}
#endif

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled_all(uint32_t ledset)
{
  stm32_gpiowrite(GPIO_LED_GREEN, (ledset & BOARD_LED_YELLOW_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_YELLOW, (ledset & BOARD_LED_YELLOW_BIT) == 0);
}
#endif

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  switch (led)
    {
      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
        stm32_gpiowrite(GPIO_LED_GREEN, false);
        stm32_gpiowrite(GPIO_LED_YELLOW, false);
        break;

      case LED_STACKCREATED:
        stm32_gpiowrite(GPIO_LED_GREEN, true);
        stm32_gpiowrite(GPIO_LED_YELLOW, false);
        g_initialized = true;
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        stm32_gpiowrite(GPIO_LED_YELLOW, true);
        break;

      case LED_IDLE : /* IDLE */
        stm32_gpiowrite(GPIO_LED_GREEN, false);
        break;
    }
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  switch (led)
    {
      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
        stm32_gpiowrite(GPIO_LED_GREEN, false);

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        stm32_gpiowrite(GPIO_LED_YELLOW, false);
        break;

      case LED_IDLE: /* IDLE */
        stm32_gpiowrite(GPIO_LED_GREEN, g_initialized);
        break;
    }
}
#endif
