/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_autoleds.c
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

#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>

#include "stm32_gpio.h"
#include "stm32f746g-disco.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure the LD1 GPIO for output. Initial state is OFF */

  stm32_configgpio(GPIO_LD1);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledstate = false;

  switch (led)
    {
    case 0:                   /* LED_STARTED:      NuttX has been started  STATUS LED=OFF */
                              /* LED_HEAPALLOCATE: Heap has been allocated STATUS LED=OFF */
                              /* LED_IRQSENABLED:  Interrupts enabled      STATUS LED=OFF */
      break;                  /* Leave ledstate == true to turn OFF */

    default:
    case 2:                   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C */
                              /* LED_SIGNAL:       In a signal handler     STATUS LED=N/C */
                              /* LED_ASSERTION:    An assertion failed     STATUS LED=N/C */
      return;                 /* Return to leave STATUS LED unchanged */

    case 3:                   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
    case 1:                   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */
      ledstate = true;        /* Set ledstate == false to turn ON */
      break;
    }

  stm32_gpiowrite(GPIO_LD1, ledstate);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
    /* These should not happen and are ignored */

    default:
    case 0:                   /* LED_STARTED:      NuttX has been started  STATUS LED=OFF */
                              /* LED_HEAPALLOCATE: Heap has been allocated STATUS LED=OFF */
                              /* LED_IRQSENABLED:  Interrupts enabled      STATUS LED=OFF */
    case 1:                   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */

    /* These result in no-change */

    case 2:                   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C */
                              /* LED_SIGNAL:       In a signal handler     STATUS LED=N/C */
                              /* LED_ASSERTION:    An assertion failed     STATUS LED=N/C */
      return;                 /* Return to leave STATUS LED unchanged */

    /* Turn STATUS LED off set driving the output high */

    case 3:                   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
      stm32_gpiowrite(GPIO_LD1, false);
      break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
