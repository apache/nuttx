/****************************************************************************
 * boards/arm/stm32l4/stm32l4r9ai-disco/src/stm32_autoleds.c
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
#include "stm32l4.h"
#include "stm32l4r9ai-disco.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LD4,5 GPIO for output */

  stm32l4_configgpio(GPIO_LED_RED);
  stm32l4_configgpio(GPIO_LED_GRN);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED
       *
       * Since the LEDs were initially all OFF and since this state only
       * occurs one time, nothing need be done.
       */

      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
        break;

      /* 1: LED_STACKCREATED
       *
       * This case will also occur only once.
       */

      case LED_STACKCREATED:
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION
       *
       * This case will occur many times.
       */

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
        stm32l4_gpiowrite(GPIO_LED_RED, true);
        break;

      /* 3: LED_PANIC: GPIO_LED_GRN=OFF RX=ON
       *
       * This case will also occur many times.
       */

      case LED_PANIC:
        stm32l4_gpiowrite(GPIO_LED_GRN, false);
        stm32l4_gpiowrite(GPIO_LED_RED, true);
        break;

      case LED_IDLE:
        stm32l4_gpiowrite(GPIO_LED_GRN, true);
        stm32l4_gpiowrite(GPIO_LED_RED, false);
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
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED:
       * 1: LED_STACKCREATED:
       *
       * These cases should never happen.
       */

      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION:
       *
       * This case will occur many times.
       */

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
        stm32l4_gpiowrite(GPIO_LED_RED, false);
        break;

      /* 3: LED_PANIC: GPIO_LED_GRN=OFF RX=OFF
       *
       * This case will also occur many times.
       */

      case LED_PANIC:
        stm32l4_gpiowrite(GPIO_LED_GRN, false);
        stm32l4_gpiowrite(GPIO_LED_RED, false);
        break;

      case LED_IDLE:
        stm32l4_gpiowrite(GPIO_LED_GRN, false);
        stm32l4_gpiowrite(GPIO_LED_RED, false);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
