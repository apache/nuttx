/****************************************************************************
 * boards/arm/nuc1xx/nutiny-nuc120/src/nuc_led.c
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

/* The NuTiny has a single green LED that can be controlled from software.
 * This LED is connected to PIN17.  It is pulled high so a low value will
 * illuminate the LED.
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the NuTiny.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 Initially all LED is OFF
 *   -------------------  -----------------------  ------------- ------------
 *   LED_STARTED          NuttX has been started   LED ON
 *   LED_HEAPALLOCATE     Heap has been allocated  LED ON
 *   LED_IRQSENABLED      Interrupts enabled       LED ON
 *   LED_STACKCREATED     Idle stack created       LED ON
 *   LED_INIRQ            In an interrupt          LED should glow
 *   LED_SIGNAL           In a signal handler      LED might glow
 *   LED_ASSERTION        An assertion failed      LED ON while handling the
 *                                                        assertion
 *   LED_PANIC            The system has crashed   LED Blinking at 2Hz
 *   LED_IDLE             NUC1XX is is sleep mode   (Optional, not used)
 */

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
#include "nuc_gpio.h"
#include "nutiny-nuc120.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) nuc_dumpgpio(GPIO_LED, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuc_led_initialize
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************/

void nuc_led_initialize(void)
{
  led_dumpgpio("Before configuration");
  nuc_configgpio(GPIO_LED);
  led_dumpgpio("After configuration");
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  nuc_gpiowrite(GPIO_LED, false);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  nuc_gpiowrite(GPIO_LED, true);
}

#endif /* CONFIG_ARCH_LEDS */
