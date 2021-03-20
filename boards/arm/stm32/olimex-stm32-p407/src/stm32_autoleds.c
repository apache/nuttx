/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p407/src/stm32_autoleds.c
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
#include "olimex-stm32-p407.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following definitions map the encoded LED setting to GPIO settings */

#define LED_STARTED_BITS             (BOARD_LED1_BIT)
#define LED_HEAPALLOCATE_BITS        (BOARD_LED2_BIT)
#define LED_IRQSENABLED_BITS         (BOARD_LED1_BIT | BOARD_LED2_BIT)
#define LED_STACKCREATED_BITS        (BOARD_LED3_BIT)
#define LED_INIRQ_BITS               (BOARD_LED1_BIT | BOARD_LED3_BIT)
#define LED_SIGNAL_BITS              (BOARD_LED2_BIT | BOARD_LED3_BIT)
#define LED_ASSERTION_BITS           (BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED3_BIT)
#define LED_PANIC_BITS               (BOARD_LED4_BIT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & BOARD_LED1_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, false);
    }

  if ((clrbits & BOARD_LED2_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, false);
    }

  if ((clrbits & BOARD_LED3_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, false);
    }

  if ((clrbits & BOARD_LED4_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED4, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & BOARD_LED1_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, true);
    }

  if ((setbits & BOARD_LED2_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, true);
    }

  if ((setbits & BOARD_LED3_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, true);
    }

  if ((setbits & BOARD_LED4_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED4, true);
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

  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
  stm32_configgpio(GPIO_LED3);
  stm32_configgpio(GPIO_LED4);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  led_clrbits(BOARD_LED1_BIT | BOARD_LED2_BIT |
              BOARD_LED3_BIT | BOARD_LED4_BIT);
  led_setbits(g_ledbits[led]);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  led_clrbits(g_ledbits[led]);
}

#endif /* CONFIG_ARCH_LEDS */
