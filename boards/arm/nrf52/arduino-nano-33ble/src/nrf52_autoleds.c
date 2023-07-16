/****************************************************************************
 * boards/arm/nrf52/arduino-nano-33ble/src/nrf52_autoleds.c
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
#include "arduino-nano-33ble.h"

#ifdef CONFIG_ARCH_LEDS

#define LED_ON 1
#define LED_OFF 0

/* This array maps an LED number to GPIO pin configuration */

static const uint32_t g_ledcfg[BOARD_NLEDS] =
{
#if 0 < BOARD_NLEDS
  GPIO_LED1,
#endif
#if 1 < BOARD_NLEDS
  GPIO_LED2,
#endif
#if 2 < BOARD_NLEDS
  GPIO_LED3,
#endif
#if 3 < BOARD_NLEDS
  GPIO_LED4,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(const char *msg)
{
  nrf52_pin_dump(PINCONFIG_LED, msg);
  nrf52_gpio_dump(GPIO_LED, msg);
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
  int i;

  /* Configure LED pin as a GPIO outputs */

  led_dumppins("board_autoled_initialize() Entry)");

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      nrf52_gpio_config(g_ledcfg[i]);
      nrf52_gpio_write(g_ledcfg[i], LED_OFF);
    }

  led_dumppins("board_autoled_initialize() Exit");
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led) {
    case LED_STARTED: /* Power */
      nrf52_gpio_write(g_ledcfg[0], LED_ON);
      break;
    case LED_HEAPALLOCATE: /* Yellow (red + green) only */
      nrf52_gpio_write(g_ledcfg[1], LED_ON);
      nrf52_gpio_write(g_ledcfg[2], LED_ON);
      nrf52_gpio_write(g_ledcfg[3], LED_OFF);
      break;
    case LED_IRQSENABLED: /* Green only */
      nrf52_gpio_write(g_ledcfg[1], LED_OFF);
      nrf52_gpio_write(g_ledcfg[2], LED_ON);
      nrf52_gpio_write(g_ledcfg[3], LED_OFF);
      break;
    case LED_STACKCREATED: /* Blue only */
      nrf52_gpio_write(g_ledcfg[1], LED_OFF);
      nrf52_gpio_write(g_ledcfg[2], LED_OFF);
      nrf52_gpio_write(g_ledcfg[3], LED_ON);
      break;
    case LED_INIRQ: /* Red added to the blue for LED_STACKCREATED */
      nrf52_gpio_write(g_ledcfg[1], LED_ON);
      break;
    case LED_SIGNAL: /* Green added to the blue for LED_STACKCREATED */
      nrf52_gpio_write(g_ledcfg[1], LED_ON);
      break;
    case LED_ASSERTION: /* Red only */
    case LED_PANIC: /* Red only, system will cause blink */
      nrf52_gpio_write(g_ledcfg[1], LED_ON);
      nrf52_gpio_write(g_ledcfg[2], LED_OFF);
      nrf52_gpio_write(g_ledcfg[3], LED_OFF);
      break;
    default:
      break;
  }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led) {
    case LED_INIRQ: /* Red added to the blue for LED_STACKCREATED */
      nrf52_gpio_write(g_ledcfg[1], LED_OFF);
      break;
    case LED_SIGNAL: /* Green added to the blue for LED_STACKCREATED */
      nrf52_gpio_write(g_ledcfg[1], LED_OFF);
      break;
    case LED_ASSERTION: /* Red only */
    case LED_PANIC: /* Red only, system will cause blink */
      nrf52_gpio_write(g_ledcfg[1], LED_OFF);
      nrf52_gpio_write(g_ledcfg[2], LED_OFF);
      nrf52_gpio_write(g_ledcfg[3], LED_OFF);
      break;
    default: /* Others never turn off */
      break;
  }
}

#endif /* CONFIG_ARCH_LEDS */
