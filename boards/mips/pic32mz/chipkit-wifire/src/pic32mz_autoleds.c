/****************************************************************************
 * boards/mips/pic32mz/chipkit-wifire/src/pic32mz_autoleds.c
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
#include <arch/board/board.h>

#include "mips_internal.h"
#include "pic32mz_gpio.h"
#include "chipkit-wifire.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* There are four LEDs on the top side of the board:
 *
 *   LED LD1      - RG6
 *   LED LD2      - RD4
 *   LED LD3      - RB11
 *   LED LD4      - RG15
 *
 * A high output value illuminates the LEDs.
 *
 * These LEDs are available to the application and are all available to the
 * application unless CONFIG_ARCH_LEDS is defined.  In that case, the usage
 * by the board port is defined in include/board.h and
 * src/pic32mz_autoleds.c. The LEDs are used to encode OS-related events
 * as follows:
 * SYMBOL            MEANING                    LED STATE
 *                                            A   B   C   D
 * ----------------  ----------------------- --- --- --- ---
 * LED_STARTED       NuttX has been started  ON  OFF OFF OFF
 * LED_HEAPALLOCATE  Heap has been allocated OFF ON  OFF OFF
 * LED_IRQSENABLED   Interrupts enabled      OFF OFF ON  OFF
 * LED_STACKCREATED  Idle stack created      OFF OFF OFF ON
 * LED_INIRQ         In an interrupt         ON  ON  ON  ON
 * LED_SIGNAL        In a signal handler     ON  ON  ON  ON
 * LED_ASSERTION     An assertion failed     ON  ON  ON  ON
 * LED_PANIC         The system has crashed  ON  ON  ON  ON
 * LED_IDLE          MCU is is sleep mode    ---- Not used ----
 */

/* LED indices */

#define INDEX_LED_LD1   0
#define INDEX_LED_LD2   1
#define INDEX_LED_LD3   2
#define INDEX_LED_LD4   3
#define NLEDS           4

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void board_autoled_setone(int ledndx)
{
  bool ledon[NLEDS] =
    {
      false,
      false,
      false,
      false
    };

  ledon[ledndx] = true;
  pic32mz_gpiowrite(GPIO_LED_LD1, ledon[INDEX_LED_LD1]);
  pic32mz_gpiowrite(GPIO_LED_LD2, ledon[INDEX_LED_LD2]);
  pic32mz_gpiowrite(GPIO_LED_LD3, ledon[INDEX_LED_LD3]);
  pic32mz_gpiowrite(GPIO_LED_LD4, ledon[INDEX_LED_LD4]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_led_initialize
 ****************************************************************************/

void pic32mz_led_initialize(void)
{
  /* Configure LED GPIOs for output */

  pic32mz_configgpio(GPIO_LED_LD1);
  pic32mz_configgpio(GPIO_LED_LD2);
  pic32mz_configgpio(GPIO_LED_LD3);
  pic32mz_configgpio(GPIO_LED_LD4);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  /* SYMBOL               MEANING                    LED STATE
   *                                               A   B   C   D
   * -------------------  ----------------------- --- --- --- ---
   * LED_STARTED       0  NuttX has been started  ON  OFF OFF OFF
   * LED_HEAPALLOCATE  1  Heap has been allocated OFF ON  OFF OFF
   * LED_IRQSENABLED   2  Interrupts enabled      OFF OFF ON  OFF
   * LED_STACKCREATED  3  Idle stack created      OFF OFF OFF ON
   * LED_INIRQ         4  In an interrupt         ON  ON  ON  ON
   * LED_SIGNAL        4  In a signal handler     ON  ON  ON  ON
   * LED_ASSERTION     4  An assertion failed     ON  ON  ON  ON
   * LED_PANIC         4  The system has crashed  ON  ON  ON  ON
   */

  switch (led)
    {
      default:
      case 0:
        board_autoled_setone(INDEX_LED_LD1);
        break;

      case 1:
        board_autoled_setone(INDEX_LED_LD2);
        break;

      case 2:
        board_autoled_setone(INDEX_LED_LD3);
        break;

      case 3:
        board_autoled_setone(INDEX_LED_LD4);
        break;

      case 4:
        board_autoled_setone(INDEX_LED_LD1);
        board_autoled_setone(INDEX_LED_LD2);
        board_autoled_setone(INDEX_LED_LD3);
        board_autoled_setone(INDEX_LED_LD4);
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* SYMBOL               MEANING                    LED STATE
   *                                               A   B   C   D
   * -------------------  ----------------------- --- --- --- ---
   * LED_STARTED       0  NuttX has been started  ON  OFF OFF OFF
   * LED_HEAPALLOCATE  1  Heap has been allocated OFF ON  OFF OFF
   * LED_IRQSENABLED   2  Interrupts enabled      OFF OFF ON  OFF
   * LED_STACKCREATED  3  Idle stack created      OFF OFF OFF ON
   * LED_INIRQ         4  In an interrupt         ON  ON  ON  ON
   * LED_SIGNAL        4  In a signal handler     ON  ON  ON  ON
   * LED_ASSERTION     4  An assertion failed     ON  ON  ON  ON
   * LED_PANIC         4  The system has crashed  ON  ON  ON  ON
   */

  switch (led)
    {
      default:
        pic32mz_gpiowrite(GPIO_LED_LD1, false);
        pic32mz_gpiowrite(GPIO_LED_LD2, false);
        pic32mz_gpiowrite(GPIO_LED_LD3, false);
        pic32mz_gpiowrite(GPIO_LED_LD4, false);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
