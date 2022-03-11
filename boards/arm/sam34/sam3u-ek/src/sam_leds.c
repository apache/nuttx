/****************************************************************************
 * boards/arm/sam34/sam3u-ek/src/sam_leds.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "sam_gpio.h"
#include "sam3u-ek.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED_OFF        0
#define LED_ON         1
#define LED_NOCHANGE   2
#define LED_MASK       3

#define LED0_SHIFT     0
#define LED0_OFF       (LED_OFF << LED0_SHIFT)
#define LED0_ON        (LED_ON << LED0_SHIFT)
#define LED0_NOCHANGE  (LED_NOCHANGE << LED0_SHIFT)
#define LED1_SHIFT     2
#define LED1_OFF       (LED_OFF << LED1_SHIFT)
#define LED1_ON        (LED_ON << LED1_SHIFT)
#define LED1_NOCHANGE  (LED_NOCHANGE << LED1_SHIFT)
#define LED2_SHIFT     4
#define LED2_OFF       (LED_OFF << LED2_SHIFT)
#define LED2_ON        (LED_ON << LED2_SHIFT)
#define LED2_NOCHANGE  (LED_NOCHANGE << LED2_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_ledon[8] =
{
  (LED0_OFF     | LED1_OFF     | LED2_OFF),      /* LED_STARTED  */
  (LED0_ON      | LED1_OFF     | LED2_ON),       /* LED_HEAPALLOCATE */
  (LED0_OFF     | LED1_ON      | LED2_OFF),      /* LED_IRQSENABLED  */
  (LED0_ON      | LED1_ON      | LED2_ON),       /* LED_STACKCREATED  */

  (LED0_NOCHANGE | LED1_OFF      | LED2_NOCHANGE), /* LED_INIRQ  */
  (LED0_NOCHANGE | LED1_NOCHANGE | LED2_OFF),      /* LED_SIGNAL  */
  (LED0_ON       | LED1_NOCHANGE | LED2_NOCHANGE), /* LED_ASSERTION  */
  (LED0_ON       | LED1_NOCHANGE | LED2_NOCHANGE)  /* LED_PANIC */
};

static const uint8_t g_ledoff[8] =
{
  (LED0_OFF     | LED1_OFF     | LED2_OFF),      /* LED_STARTED (does not happen) */
  (LED0_ON      | LED1_OFF     | LED2_ON),       /* LED_HEAPALLOCATE (does not happen) */
  (LED0_OFF     | LED1_ON      | LED2_OFF),      /* LED_IRQSENABLED (does not happen) */
  (LED0_ON      | LED1_ON      | LED2_ON),       /* LED_STACKCREATED (does not happen) */

  (LED0_NOCHANGE | LED1_ON       | LED2_NOCHANGE), /* LED_INIRQ  */
  (LED0_NOCHANGE | LED1_NOCHANGE | LED2_ON),       /* LED_SIGNAL */
  (LED0_OFF      | LED1_NOCHANGE | LED2_NOCHANGE), /* LED_ASSERTION */
  (LED0_OFF      | LED1_NOCHANGE | LED2_NOCHANGE)  /* LED_PANIC */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_setled
 ****************************************************************************/

static void sam_setled(uint16_t pinset, uint8_t state)
{
  /* Assume active high.  Initial state == 0 means active high */

  bool polarity = ((pinset & GPIO_OUTPUT_SET) == 0);
  switch (state)
    {
      case LED_OFF:
        polarity = !polarity;

      case LED_ON:
        break;

      case LED_NOCHANGE:
      default:
        return;
    }

  sam_gpiowrite(pinset, polarity);
}

/****************************************************************************
 * Name: sam_setleds
 ****************************************************************************/

static void sam_setleds(uint8_t state)
{
  sam_setled(GPIO_LED0, (state >> LED0_SHIFT) & LED_MASK);
  sam_setled(GPIO_LED1, (state >> LED1_SHIFT) & LED_MASK);
  sam_setled(GPIO_LED2, (state >> LED2_SHIFT) & LED_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  sam_configgpio(GPIO_LED0);
  sam_configgpio(GPIO_LED1);
  sam_configgpio(GPIO_LED2);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  sam_setleds(g_ledon[led & 7]);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  sam_setleds(g_ledoff[led & 7]);
}

#endif /* CONFIG_ARCH_LEDS */
