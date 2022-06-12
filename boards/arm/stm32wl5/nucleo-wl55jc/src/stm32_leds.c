/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_leds.c
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
#include <stdbool.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32wl5.h"
#include "nucleo-wl55jc.h"

#include <stdio.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Identifies led state */

#define LED_OFF 0
#define LED_ON  1

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_state
 *
 * Description:
 *   Sets pack of leds to given state
 ****************************************************************************/

static void led_state(int state, unsigned int leds)
{
  if (leds & BOARD_LED_BLUE_BIT)
    {
      stm32wl5_gpiowrite(GPIO_LED_BLUE, state);
    }

  if (leds & BOARD_LED_RED_BIT)
    {
      stm32wl5_gpiowrite(GPIO_LED_RED, state);
    }

  if (leds & BOARD_LED_GREEN_BIT)
    {
      stm32wl5_gpiowrite(GPIO_LED_GREEN, state);
    }
}

/****************************************************************************
 * Name: button3_led
 *
 * Description:
 *   Toggles Red LED state from interrupt generated from button 3
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_NUCLEO_WL55JC_DEMO_LED_IRQ
static int button3_led(int irq, void *context, void *arg)
{
  (void)irq;
  (void)context;
  (void)arg;
  int state;

  state = stm32wl5_gpioread(GPIO_LED_RED);

  /* toggle state */

  state = !state;
  stm32wl5_gpiowrite(GPIO_LED_RED, state);
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_leds_initialize
 ****************************************************************************/

void board_leds_initialize(void)
{
  stm32wl5_configgpio(GPIO_LED_BLUE);
  stm32wl5_configgpio(GPIO_LED_RED);
  stm32wl5_configgpio(GPIO_LED_GREEN);
}

/****************************************************************************
 * Name: board_leds_on
 *
 * Description:
 *   Drives board leds when specific RTOS state led occurs.
 *
 * Input Parameters:
 *   state - RTOS state
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS

void board_autoled_on(int state)
{
  switch (state)
    {
    case LED_STARTED:
      led_state(LED_OFF, (unsigned int)-1);
      led_state(LED_ON,  BOARD_LED_BLUE_BIT);
      break;

    case LED_HEAPALLOCATE:
      led_state(LED_OFF, (unsigned int)-1);
      led_state(LED_ON,  BOARD_LED_GREEN_BIT);
      break;

    case LED_IRQSENABLED:
      led_state(LED_OFF, (unsigned int)-1);
      led_state(LED_ON,  BOARD_LED_BLUE_BIT | BOARD_LED_GREEN_BIT);
      break;

    case LED_STACKCREATED:
      led_state(LED_OFF, (unsigned int)-1);
      led_state(LED_ON,  BOARD_LED_RED_BIT);
      break;

    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
    case LED_PANIC:
      led_state(LED_ON,  BOARD_LED_BLUE_BIT);
      break;
    }
}

#endif /* CONFIG_ARCH_LEDS */

/****************************************************************************
 * Name: board_leds_off
 *
 * Description:
 *   Drives board leds when specific RTOS state led occurs.
 *
 * Input Parameters:
 *   state - RTOS state
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS

void board_autoled_off(int state)
{
  switch (state)
    {
    case LED_HEAPALLOCATE:
      led_state(LED_OFF, BOARD_LED_GREEN_BIT);
      break;

    case LED_IRQSENABLED:
      led_state(LED_OFF, BOARD_LED_BLUE_BIT | BOARD_LED_GREEN_BIT);
      break;

    case LED_STACKCREATED:
      led_state(LED_OFF, BOARD_LED_RED_BIT);
      break;

    case LED_STARTED:
    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
    case LED_PANIC:
      led_state(LED_OFF, BOARD_LED_BLUE_BIT);
      break;
    }
}

#endif /* CONFIG_ARCH_LEDS */

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
  /* Leds are already initialized by stm32_led_initialize. */

#ifdef CONFIG_ARCH_BOARD_NUCLEO_WL55JC_DEMO_LED_IRQ
  /* Configure B3 button to fire an interrupt on falling edge (on press)  */

  stm32wl5_gpiosetevent(GPIO_BUTTON3, false, true, false, button3_led, NULL);
#endif

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
  if (led == BOARD_LED_BLUE)
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
  led_state(LED_ON, ledset & ~BOARD_LED_BLUE_BIT);
  led_state(LED_OFF, ~(ledset | BOARD_LED_BLUE_BIT));
#else
  led_state(LED_ON, ledset);
  led_state(LED_OFF, ~ledset);
#endif
}
