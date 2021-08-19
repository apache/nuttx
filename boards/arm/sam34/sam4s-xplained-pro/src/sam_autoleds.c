/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam_autoleds.c
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
#include "sam_gpio.h"
#include "sam4s-xplained-pro.h"

#ifdef CONFIG_ARCH_LEDS

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on
 * board the SAM4S Xplained Pro.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                     LED state
 *                                                   D9     D10
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     OFF
 *   LED_IDLE             MCU is is sleep mode       Not used
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIO for output */

  sam_configgpio(GPIO_D301);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case 0:  /* LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED - off while initializing */
        sam_gpiowrite(GPIO_D301, LED_D301_OFF);
        break;

      case 1:  /* LED_STACKCREATED - turn on when ready */
        sam_gpiowrite(GPIO_D301, LED_D301_ON);
        break;

      case 2:  /* LED_INIRQ, LED_SIGNAL - turn off inside irqs/signal processing */
        sam_gpiowrite(GPIO_D301, LED_D301_OFF);
        return;

      case 3:  /* LED_PANIC - flash */
        sam_gpiowrite(GPIO_D301, LED_D301_ON);
        break;

      default:
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
        break;

      case 2:  /* LED_INIRQ, LED_SIGNAL - return to on after irq/signal processing */
        sam_gpiowrite(GPIO_D301, LED_D301_ON);
        return;

      case 3:  /* LED_PANIC - flashes */
        sam_gpiowrite(GPIO_D301, LED_D301_OFF);
        break;
  }
}

#endif /* CONFIG_ARCH_LEDS */
