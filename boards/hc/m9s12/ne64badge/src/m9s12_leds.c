/****************************************************************************
 * boards/hc/m9s12/ne64badge/src/m9s12_leds.c
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
#include <arch/board/board.h>

#include "ne64badge.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) m9s12_dumpgpio(m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Configure and initialize on-board LEDs
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure all LED GPIO lines */

  led_dumpgpio("board_autoled_initialize() Entry)");

  hcs12_configgpio(NE64BADGE_LED1);
  hcs12_configgpio(NE64BADGE_LED2);

  led_dumpgpio("board_autoled_initialize() Exit");
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
       default:
       case 0 : /* STARTED, HEAPALLOCATE, IRQSENABLED */
        hcs12_gpiowrite(NE64BADGE_LED1, true);
        hcs12_gpiowrite(NE64BADGE_LED2, true);
        break;

       case 1 : /* STACKCREATED */
        hcs12_gpiowrite(NE64BADGE_LED1, false);
        hcs12_gpiowrite(NE64BADGE_LED2, true);
        break;

       case 2 : /* INIRQ, SIGNAL, ASSERTION, PANIC */
        hcs12_gpiowrite(NE64BADGE_LED2, false);
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
       default:
       case 0 : /* STARTED, HEAPALLOCATE, IRQSENABLED */
       case 1 : /* STACKCREATED */
        hcs12_gpiowrite(NE64BADGE_LED1, true);
       case 2 : /* INIRQ, SIGNAL, ASSERTION, PANIC */
        hcs12_gpiowrite(NE64BADGE_LED2, true);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
