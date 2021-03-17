/****************************************************************************
 * boards/arm/tiva/launchxl-cc1312r1/src/cc1312_autoleds.c
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

#include "tiva_gpio.h"
#include "launchxl-cc1312r1.h"

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  tiva_configgpio(&g_gpio_gled);
  tiva_configgpio(&g_gpio_rled);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool gled_change = true;   /* True: Change GLED */
  bool gled_on     = false;  /* High output illuminates */
  bool rled_on     = false;

  /* SYMBOL           VAL MEANING                 GLED RLED
   * ---------------- --- ----------------------- ---- -----
   * LED_STARTED       0  NuttX has been started  OFF  OFF
   * LED_HEAPALLOCATE  1  Heap has been allocated OFF  ON
   * LED_IRQSENABLED   1  Interrupts enabled      OFF  ON
   * LED_STACKCREATED  2  Idle stack created      ON   OFF
   * LED_INIRQ         3  In an interrupt         N/C  GLOW
   * LED_SIGNAL        3  In a signal handler     N/C  GLOW
   * LED_ASSERTION     3  An assertion failed     N/C  GLOW
   * LED_PANIC         4  The system has crashed  OFF  BLINK
   */

  switch (led)
    {
      case 0:  /* GLED=OFF RLED=OFF */
        break;

      case 3:  /* GLED=N/C RLED=ON */
        gled_change = false;

        /* Fall through */

      case 1:  /* GLED=OFF RLED=ON */
      case 4:  /* GLED=OFF RLED=ON */
        rled_on = true;
        break;

      case 2:  /* GLED=ON  RLED=OFF */
        gled_on = true;
        break;

      default:
        return;
    }

  /* Set the new state of the GLED (unless is is N/C) */

  if (gled_change)
    {
      tiva_gpiowrite(&g_gpio_gled, gled_on);
    }

  /* Set the new state of the RLED */

  tiva_gpiowrite(&g_gpio_rled, rled_on);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* SYMBOL           VAL MEANING                 GLED RLED
   * ---------------- --- ----------------------- ---- -----
   * LED_STARTED       0  NuttX has been started  OFF  OFF
   * LED_HEAPALLOCATE  1  Heap has been allocated OFF  ON
   * LED_IRQSENABLED   1  Interrupts enabled      OFF  ON
   * LED_STACKCREATED  2  Idle stack created      ON   OFF
   * LED_INIRQ         3  In an interrupt         N/C  GLOW
   * LED_SIGNAL        3  In a signal handler     N/C  GLOW
   * LED_ASSERTION     3  An assertion failed     N/C  GLOW
   * LED_PANIC         4  The system has crashed  OFF  BLINK
   */

  switch (led)
    {
      case 4:  /* GLED=OFF RLED=OFF */
        tiva_gpiowrite(&g_gpio_gled, false);

        /* Fall through */

      case 3:  /* GLED=N/C RLED=OFF */
        tiva_gpiowrite(&g_gpio_rled, false);
        break;

      case 0:  /* Should not happen */
      case 1:  /* Should not happen */
      case 2:  /* Should not happen */
      default:
        return;
    }
}

#endif /* CONFIG_ARCH_LEDS */
