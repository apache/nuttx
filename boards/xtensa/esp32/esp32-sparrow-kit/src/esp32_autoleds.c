/****************************************************************************
 * boards/xtensa/esp32/esp32-sparrow-kit/src/esp32_autoleds.c
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

#include "esp32_gpio.h"
#include "esp32-sparrow-kit.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following definitions map the encoded LED setting to GPIO settings */

#ifndef CONFIG_ARCH_LEDS_CPU_ACTIVITY
#  define LED_STARTED_BITS         (BOARD_LED2_BIT)
#  define LED_HEAPALLOCATE_BITS    (BOARD_LED3_BIT)
#  define LED_IRQSENABLED_BITS     (BOARD_LED3_BIT | BOARD_LED2_BIT)
#  define LED_STACKCREATED_BITS    (BOARD_LED3_BIT)
#  define LED_INIRQ_BITS           (BOARD_LED1_BIT | BOARD_LED3_BIT)
#  define LED_SIGNAL_BITS          (BOARD_LED2_BIT | BOARD_LED3_BIT)
#  define LED_ASSERTION_BITS       (BOARD_LED1_BIT | BOARD_LED2_BIT |\
                                    BOARD_LED3_BIT)
#  define LED_PANIC_BITS           (BOARD_LED1_BIT)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS_CPU_ACTIVITY
static const unsigned int g_ledbits[8] =
{
  LED_STARTED_BITS,
  LED_HEAPALLOCATE_BITS,
  LED_IRQSENABLED_BITS,
  LED_STACKCREATED_BITS,
  LED_INIRQ_BITS,
  LED_SIGNAL_BITS,
  LED_ASSERTION_BITS,
  LED_PANIC_BITS
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & BOARD_LED1_BIT) != 0)
    {
      esp32_gpiowrite(GPIO_LED1, false);
    }

  if ((clrbits & BOARD_LED2_BIT) != 0)
    {
      esp32_gpiowrite(GPIO_LED2, false);
    }

  if ((clrbits & BOARD_LED3_BIT) != 0)
    {
      esp32_gpiowrite(GPIO_LED3, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & BOARD_LED1_BIT) != 0)
    {
      esp32_gpiowrite(GPIO_LED1, true);
    }

  if ((setbits & BOARD_LED2_BIT) != 0)
    {
      esp32_gpiowrite(GPIO_LED2, true);
    }

  if ((setbits & BOARD_LED3_BIT) != 0)
    {
      esp32_gpiowrite(GPIO_LED3, true);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED1-4 GPIOs for output */

  esp32_configgpio(GPIO_LED1, OUTPUT);
  esp32_configgpio(GPIO_LED2, OUTPUT);
  esp32_configgpio(GPIO_LED3, OUTPUT);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
  switch (led)
    {
      case LED_CPU0:
        esp32_gpiowrite(GPIO_LED1, true);
        break;
      case LED_CPU1:
        esp32_gpiowrite(GPIO_LED2, true);
        break;
      case LED_HEAPALLOCATE:
        esp32_gpiowrite(GPIO_LED3, true);
        break;
      default:
        break;
    }
#else
  led_clrbits(BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED3_BIT);
  led_setbits(g_ledbits[led]);
#endif
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
  switch (led)
    {
      case LED_CPU0:
        esp32_gpiowrite(GPIO_LED1, false);
        break;
      case LED_CPU1:
        esp32_gpiowrite(GPIO_LED2, false);
        break;
      case LED_HEAPALLOCATE:
        esp32_gpiowrite(GPIO_LED3, false);
        break;
      default:
        break;
    }
#else
  led_clrbits(g_ledbits[led]);
#endif
}

#endif /* CONFIG_ARCH_LEDS */
