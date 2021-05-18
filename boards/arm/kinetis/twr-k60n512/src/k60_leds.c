/****************************************************************************
 * boards/arm/kinetis/twr-k60n512/src/k60_leds.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>

#include "kinetis.h"
#include "twr-k60n512.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The TWR-K60N512 has four LEDs:
 *
 * 1. E1 / Orange LED   PTA11
 * 2. E2 / Yellow LED   PTA28
 * 3. E3 / Green LED    PTA29
 * 4  E4 / Blue LED     PTA10
 */

/* The following definitions map the encoded LED setting to GPIO settings */

#define K60_LED1          (1 << 0)
#define K60_LED2          (1 << 1)
#define K60_LED3          (1 << 2)
#define K60_LED4          (1 << 3)

#define ON_SETBITS_SHIFT  (0)
#define ON_CLRBITS_SHIFT  (4)
#define OFF_SETBITS_SHIFT (8)
#define OFF_CLRBITS_SHIFT (12)

#define ON_BITS(v)        ((v) & 0xff)
#define OFF_BITS(v)       (((v) >> 8) & 0x0ff)
#define SETBITS(b)        ((b) & 0x0f)
#define CLRBITS(b)        (((b) >> 4) & 0x0f)

#define ON_SETBITS(v)     (SETBITS(ON_BITS(v))
#define ON_CLRBITS(v)     (CLRBITS(ON_BITS(v))
#define OFF_SETBITS(v)    (SETBITS(OFF_BITS(v))
#define OFF_CLRBITS(v)    (CLRBITS(OFF_BITS(v))

#define LED_STARTED_ON_SETBITS       ((K60_LED1) << ON_SETBITS_SHIFT)
#define LED_STARTED_ON_CLRBITS       ((K60_LED2|K60_LED3|K60_LED4) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      ((K60_LED1|K60_LED2|K60_LED3|K60_LED4) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((K60_LED2) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((K60_LED1|K60_LED3|K60_LED4) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS ((K60_LED1) << OFF_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((K60_LED2|K60_LED3|K60_LED4) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((K60_LED1|K60_LED2) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((K60_LED3|K60_LED4) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((K60_LED2) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((K60_LED1|K60_LED3|K60_LED4) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  ((K60_LED3) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((K60_LED1|K60_LED2|K60_LED4) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((K60_LED1|K60_LED2) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((K60_LED3|K60_LED4) << OFF_CLRBITS_SHIFT)

#define LED_INIRQ_ON_SETBITS         ((K60_LED1) << ON_SETBITS_SHIFT)
#define LED_INIRQ_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_INIRQ_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_INIRQ_OFF_CLRBITS        ((K60_LED1) << OFF_CLRBITS_SHIFT)

#define LED_SIGNAL_ON_SETBITS        ((K60_LED2) << ON_SETBITS_SHIFT)
#define LED_SIGNAL_ON_CLRBITS        ((0) << ON_CLRBITS_SHIFT)
#define LED_SIGNAL_OFF_SETBITS       ((0) << OFF_SETBITS_SHIFT)
#define LED_SIGNAL_OFF_CLRBITS       ((K60_LED2) << OFF_CLRBITS_SHIFT)

#define LED_ASSERTION_ON_SETBITS     ((K60_LED4) << ON_SETBITS_SHIFT)
#define LED_ASSERTION_ON_CLRBITS     ((0) << ON_CLRBITS_SHIFT)
#define LED_ASSERTION_OFF_SETBITS    ((0) << OFF_SETBITS_SHIFT)
#define LED_ASSERTION_OFF_CLRBITS    ((K60_LED4) << OFF_CLRBITS_SHIFT)

#define LED_PANIC_ON_SETBITS         ((K60_LED4) << ON_SETBITS_SHIFT)
#define LED_PANIC_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_PANIC_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_PANIC_OFF_CLRBITS        ((K60_LED4) << OFF_CLRBITS_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_ledbits[8] =
{
  (LED_STARTED_ON_SETBITS       | LED_STARTED_ON_CLRBITS |
  LED_STARTED_OFF_SETBITS      | LED_STARTED_OFF_CLRBITS),

  (LED_HEAPALLOCATE_ON_SETBITS  | LED_HEAPALLOCATE_ON_CLRBITS |
  LED_HEAPALLOCATE_OFF_SETBITS | LED_HEAPALLOCATE_OFF_CLRBITS),

  (LED_IRQSENABLED_ON_SETBITS   | LED_IRQSENABLED_ON_CLRBITS |
  LED_IRQSENABLED_OFF_SETBITS  | LED_IRQSENABLED_OFF_CLRBITS),

  (LED_STACKCREATED_ON_SETBITS  | LED_STACKCREATED_ON_CLRBITS |
  LED_STACKCREATED_OFF_SETBITS | LED_STACKCREATED_OFF_CLRBITS),

  (LED_INIRQ_ON_SETBITS         | LED_INIRQ_ON_CLRBITS |
  LED_INIRQ_OFF_SETBITS        | LED_INIRQ_OFF_CLRBITS),

  (LED_SIGNAL_ON_SETBITS        | LED_SIGNAL_ON_CLRBITS |
  LED_SIGNAL_OFF_SETBITS       | LED_SIGNAL_OFF_CLRBITS),

  (LED_ASSERTION_ON_SETBITS     | LED_ASSERTION_ON_CLRBITS |
  LED_ASSERTION_OFF_SETBITS    | LED_ASSERTION_OFF_CLRBITS),

  (LED_PANIC_ON_SETBITS         | LED_PANIC_ON_CLRBITS |
  LED_PANIC_OFF_SETBITS        | LED_PANIC_OFF_CLRBITS)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & K60_LED1) != 0)
    {
      kinetis_gpiowrite(GPIO_LED1, false);
    }

  if ((clrbits & K60_LED2) != 0)
    {
      kinetis_gpiowrite(GPIO_LED2, false);
    }

  if ((clrbits & K60_LED3) != 0)
    {
      kinetis_gpiowrite(GPIO_LED3, false);
    }

  if ((clrbits & K60_LED4) != 0)
    {
      kinetis_gpiowrite(GPIO_LED4, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & K60_LED1) != 0)
    {
      kinetis_gpiowrite(GPIO_LED1, true);
    }

  if ((setbits & K60_LED2) != 0)
    {
      kinetis_gpiowrite(GPIO_LED2, true);
    }

  if ((setbits & K60_LED3) != 0)
    {
      kinetis_gpiowrite(GPIO_LED3, true);
    }

  if ((setbits & K60_LED4) != 0)
    {
      kinetis_gpiowrite(GPIO_LED4, true);
    }
}

static void led_setonoff(unsigned int bits)
{
  led_clrbits(CLRBITS(bits));
  led_setbits(SETBITS(bits));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Initialize LED GPIOs so that LEDs can be controlled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  /* Configure LED1-4 GPIOs for output */

  kinetis_pinconfig(GPIO_LED1);
  kinetis_pinconfig(GPIO_LED2);
  kinetis_pinconfig(GPIO_LED3);
  kinetis_pinconfig(GPIO_LED4);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  led_setonoff(ON_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  led_setonoff(OFF_BITS(g_ledbits[led]));
}

#endif /* CONFIG_ARCH_LEDS */
