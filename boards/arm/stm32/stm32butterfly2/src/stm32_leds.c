/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_leds.c
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

#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz |\
                         GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN0)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz |\
                         GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN1)
#define GPIO_LED3       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz |\
                         GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN4)
#define GPIO_LED4       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz |\
                         GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN5)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Identifies led state */

enum led_state
{
  LED_ON = false,
  LED_OFF = true
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_state
 *
 * Description:
 *   Sets pack of leds to given state
 ****************************************************************************/

static void led_state(enum led_state state, unsigned int leds)
{
  if (leds & BOARD_LED1_BIT)
    {
      stm32_gpiowrite(GPIO_LED1, state);
    }

  if (leds & BOARD_LED2_BIT)
    {
      stm32_gpiowrite(GPIO_LED2, state);
    }

  if (leds & BOARD_LED3_BIT)
    {
      stm32_gpiowrite(GPIO_LED3, state);
    }

  if (leds & BOARD_LED4_BIT)
    {
      stm32_gpiowrite(GPIO_LED4, state);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_led_initialize
 *
 * Description:
 *   Initializes low level gpio pins for board LEDS
 ****************************************************************************/

void stm32_led_initialize(void)
{
  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
  stm32_configgpio(GPIO_LED3);
  stm32_configgpio(GPIO_LED4);
}

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Drives board leds when specific RTOS state led occurs.
 *
 * Input Parameters:
 *   led - This is actually RTOS state not led number of anything like that
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
    case LED_STARTED:
      led_state(LED_OFF, BOARD_LED2_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT);
      led_state(LED_ON,  BOARD_LED1_BIT);
      break;

    case LED_HEAPALLOCATE:
      led_state(LED_OFF, BOARD_LED1_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT);
      led_state(LED_ON,  BOARD_LED2_BIT);
      break;

    case LED_IRQSENABLED:
      led_state(LED_OFF, BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED4_BIT);
      led_state(LED_ON,  BOARD_LED3_BIT);
      break;

    case LED_STACKCREATED:
      led_state(LED_OFF, BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED3_BIT);
      led_state(LED_ON,  BOARD_LED4_BIT);
      break;

    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
    case LED_PANIC:
      led_state(LED_ON,  BOARD_LED4_BIT);
      break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Drives board leds when specific RTOS state led ends
 *
 * Input Parameters:
 *   led - This is actually RTOS state not led number of anything like that
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
    case LED_STARTED:
      led_state(LED_OFF, BOARD_LED1_BIT);
      break;

    case LED_HEAPALLOCATE:
      led_state(LED_OFF, BOARD_LED2_BIT);
      break;

    case LED_IRQSENABLED:
      led_state(LED_OFF, BOARD_LED3_BIT);
      break;

    case LED_STACKCREATED:
    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
    case LED_PANIC:
      led_state(LED_OFF,  BOARD_LED4_BIT);
      break;
    }
}
#endif

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   This function should initialize leds for user use, but on RTOS start we
 *   initialize every led for use by RTOS and at end, when RTOS is fully
 *   booted up, we give control of these specific leds for user. So that's
 *   why this function is empty.
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Already initialized by stm32_led_initialize. */

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 *
 * Description:
 *   Sets led to ledon state.
 *
 * Input Parameters:
 *   led - Led to be set, indexed from 0
 *   ledon - new state for the led.
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  unsigned int ledbit;

#ifndef CONFIG_ARCH_LEDS
  if (led == BOARD_LED4)
    {
      return;
    }
#endif

  ledbit = 1 << led;
  led_state(ledon, ledbit);
}

/****************************************************************************
 * Name: board_userled_all
 *
 * Description:
 *   Sets whole ledset to given state.
 *
 * Input Parameters:
 *   ledset - Led bits to be set on or off
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
#ifdef CONFIG_ARCH_LEDS
  led_state(LED_ON, ledset & ~BOARD_LED4_BIT);
  led_state(LED_OFF, ~(ledset | BOARD_LED4_BIT));
#else
  led_state(LED_ON, ledset);
  led_state(LED_OFF, ~ledset);
#endif
}
