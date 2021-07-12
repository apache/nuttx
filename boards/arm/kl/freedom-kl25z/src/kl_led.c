/****************************************************************************
 * boards/arm/kl/freedom-kl25z/src/kl_led.c
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

/* The Freedom KL25Z has a single RGB LED driven by the KL25Z as follows:
 *
 *   ------------- --------
 *   RGB LED       KL25Z128
 *   ------------- --------
 *   Red Cathode   PTB18
 *   Green Cathode PTB19
 *   Blue Cathode  PTD1
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Freedom KL25Z.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 Initially all LED is OFF
 *   -------------------  -----------------------  --------------------------
 *   LED_STARTED          NuttX has been started   R=OFF G=OFF B=OFF
 *   LED_HEAPALLOCATE     Heap has been allocated  (no change)
 *   LED_IRQSENABLED      Interrupts enabled       (no change)
 *   LED_STACKCREATED     Idle stack created       R=OFF G=OFF B=ON
 *   LED_INIRQ            In an interrupt          (no change)
 *   LED_SIGNAL           In a signal handler      (no change)
 *   LED_ASSERTION        An assertion failed      (no change)
 *   LED_PANIC            The system has crashed   R=FLASHING G=OFF B=OFF
 *   LED_IDLE             K25Z1XX is in sleep mode (Optional, not used)
 */

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
#include "kl_gpio.h"
#include "freedom-kl25z.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) kl_dumpgpio(GPIO_LED_B, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kl_led_initialize
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************/

void kl_led_initialize(void)
{
  kl_configgpio(GPIO_LED_R);
  kl_configgpio(GPIO_LED_G);
  kl_configgpio(GPIO_LED_B);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == LED_STACKCREATED)
    {
      kl_gpiowrite(GPIO_LED_R, true);
      kl_gpiowrite(GPIO_LED_G, true);
      kl_gpiowrite(GPIO_LED_B, false);
    }
  else if (led == LED_PANIC)
    {
      kl_gpiowrite(GPIO_LED_R, false);
      kl_gpiowrite(GPIO_LED_G, true);
      kl_gpiowrite(GPIO_LED_B, true);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_PANIC)
    {
      kl_gpiowrite(GPIO_LED_R, true);
      kl_gpiowrite(GPIO_LED_G, true);
      kl_gpiowrite(GPIO_LED_B, true);
    }
}

#endif /* CONFIG_ARCH_LEDS */
