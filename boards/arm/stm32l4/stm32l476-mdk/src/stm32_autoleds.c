/****************************************************************************
 * boards/arm/stm32l4/stm32l476-mdk/src/stm32_autoleds.c
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

/* The Reference Moto Mod contains three LEDs.  Two LEDs, are by convention,
 * used to indicate the Reference Moto Mod battery state of charge, and the
 * other is available for you to use in your applications.
 *
 *   1. The red LED on PD7.  Part of the (rear-firing) red/green LED.
 *   2. The green LED on PE7.  Part of the (rear-firing) red/green LED.
 *   3. The white (top-firing) LED on PE8
 *
 * When the I/O is HIGH value, the LED is OFF.
 * When the I/O is LOW, the LED is ON.
 *
 * Following this convention, only the white LED is made available even
 * though they all could be user-application controlled if desired.
 *
 * None of the LEDs are used by the board port unless CONFIG_ARCH_LEDS is
 * defined. In that case, the white LED (only) will be controlled.  Usage
 * by the board port is defined in include/board.h and src/stm32_autoleds.c.
 *  The white LED will be used to encode OS-related events as follows:
 *
 *   ------------------ ------------------------ ------
 *   SYMBOL             Meaning                  LED
 *   ------------------ ------------------------ ------
 *
 *   LED_STARTED        NuttX has been started   OFF
 *   LED_HEAPALLOCATE   Heap has been allocated  OFF
 *   LED_IRQSENABLED    Interrupts enabled       OFF
 *   LED_STACKCREATED   Idle stack created       ON
 *   LED_INIRQ          In an interrupt          N/C
 *   LED_SIGNAL         In a signal handler      N/C
 *   LED_ASSERTION      An assertion failed      N/C
 *   LED_PANIC          The system has crashed   FLASH
 *   LED_IDLE           MCU is is sleep mode     Not used
 *
 * Thus if the white LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If white LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system has
 * halted.
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

#include "stm32l4_gpio.h"
#include "stm32l476-mdk.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIOs for output */

  stm32l4_configgpio(GPIO_LED_RED);
  stm32l4_configgpio(GPIO_LED_GREEN);
  stm32l4_configgpio(GPIO_LED_WHITE);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == 1 || led == 3)
    {
      stm32l4_gpiowrite(GPIO_LED_WHITE, false); /* Low illuminates */
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == 3)
    {
      stm32l4_gpiowrite(GPIO_LED_WHITE, true);  /* High extinguishes */
    }
}

#endif /* CONFIG_ARCH_LEDS */
