/****************************************************************************
 * boards/arm/max326xx/max32660-evsys/src/max326_autoleds.c
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

/* The MAX32660-EVSYS board has a single red LED is connected to GPIO P0.13
 * for general user indication.  A low value illuminates the LED.
 *
 * This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/max326_autoleds.c. The LEDs are used to
 * encode OS-related events as follows:
 *
 *   ------------------- ----------------------- ------
 *   SYMBOL                  Meaning             LED
 *   ------------------- ----------------------- ------
 *
 *   LED_STARTED      0  NuttX has been started  OFF
 *   LED_HEAPALLOCATE 0  Heap has been allocated OFF
 *   LED_IRQSENABLED  0  Interrupts enabled      OFF
 *   LED_STACKCREATED 1  Idle stack created      ON
 *   LED_INIRQ        2  In an interrupt         N/C
 *   LED_SIGNAL       2  In a signal handler     N/C
 *   LED_ASSERTION    2  An assertion failed     N/C
 *   LED_PANIC        3  The system has crashed  FLASH
 *   LED_IDLE            MCU is is sleep mode    Not used
 *
 * Thus is LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
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

#include "max326_gpio.h"
#include "max32660-evsys.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED PIOs for output */

  max326_gpio_config(GPIO_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == 1 || led == 3)
    {
      max326_gpio_write(GPIO_LED, true); /* High illuminates */
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == 3)
    {
      max326_gpio_write(GPIO_LED, false);  /* Low extinguishes */
    }
}

#endif /* CONFIG_ARCH_LEDS */
