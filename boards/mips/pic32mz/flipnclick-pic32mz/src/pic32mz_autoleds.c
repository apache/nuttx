/****************************************************************************
 * boards/mips/pic32mz/flipnclick-pic32mz/src/pic32mz_autoleds.c
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
#include "flipnclick-pic32mz.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* There are four LEDs on the top, red side of the board.  Only one can be
 * controlled by software:
 *
 *   LED L      - RB14 (SPI3_SCK)
 *
 * There are also four LEDs on the back, white side of the board:
 *
 *   LED A      - RA6
 *   LED B      - RA7
 *   LED C      - RE0
 *   LED D      - RE1
 *
 * A high output value illuminates the LEDs.
 *
 * These LEDs are available to the application and are all available to the
 * application unless CONFIG_ARCH_LEDS is defined.  In that case, the usage
 * by the board port is defined in include/board.h and src/sam_autoleds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *   SYMBOL           MEANING                        LED STATE
 *                                             L   A   B   C   D
 *   ---------------- ----------------------- --- --- --- --- ---
 *   LED_STARTED      NuttX has been started  OFF ON  OFF OFF OFF
 *   LED_HEAPALLOCATE Heap has been allocated OFF OFF ON  OFF OFF
 *   LED_IRQSENABLED  Interrupts enabled      OFF OFF OFF ON  OFF
 *   LED_STACKCREATED Idle stack created      OFF OFF OFF OFF ON
 *   LED_INIRQ        In an interrupt         GLO N/C N/C N/C N/C
 *   LED_SIGNAL       In a signal handler     GLO N/C N/C N/C N/C
 *   LED_ASSERTION    An assertion failed     GLO N/C N/C N/C N/C
 *   LED_PANIC        The system has crashed  2Hz N/C N/C N/C N/C
 *   LED_IDLE         MCU is is sleep mode    ---- Not used -----
 *
 * Thus if LED L is faintly glowing and all other LEDs are off (except LED
 * D which was left on but is no longer controlled by NuttX and so may be in
 * any state), NuttX has successfully booted and is, apparently, running
 * normally and taking interrupts.  If any of LEDs A-D are statically set,
 * then NuttX failed to boot and the LED indicates the initialization phase
 * where the failure occurred.  If LED L is flashing at approximately 2Hz,
 * then a fatal error has been detected and the system has halted.
 *
 * NOTE: After booting, LEDs A-D are no longer used by the system and may
 * be controlled the application.
 */

/* LED indices */

#define INDEX_LED_L     0
#define INDEX_LED_A     1
#define INDEX_LED_B     2
#define INDEX_LED_C     3
#define INDEX_LED_D     4
#define NLEDS           5

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
      false,
      false
    };

  ledon[ledndx] = true;
  pic32mz_gpiowrite(GPIO_LED_L, ledon[INDEX_LED_L]);
  pic32mz_gpiowrite(GPIO_LED_A, ledon[INDEX_LED_A]);
  pic32mz_gpiowrite(GPIO_LED_B, ledon[INDEX_LED_B]);
  pic32mz_gpiowrite(GPIO_LED_C, ledon[INDEX_LED_C]);
  pic32mz_gpiowrite(GPIO_LED_D, ledon[INDEX_LED_D]);
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

  pic32mz_configgpio(GPIO_LED_L);
  pic32mz_configgpio(GPIO_LED_A);
  pic32mz_configgpio(GPIO_LED_B);
  pic32mz_configgpio(GPIO_LED_C);
  pic32mz_configgpio(GPIO_LED_D);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  /* SYMBOL               MEANING                      LED STATE
   *                                               L   A   B   C   D
   * -------------------  ----------------------- --- --- --- --- ---
   * LED_STARTED       0  NuttX has been started  OFF ON  OFF OFF OFF
   * LED_HEAPALLOCATE  1  Heap has been allocated OFF OFF ON  OFF OFF
   * LED_IRQSENABLED   2  Interrupts enabled      OFF OFF OFF ON  OFF
   * LED_STACKCREATED  3  Idle stack created      OFF OFF OFF OFF ON
   * LED_INIRQ         4  In an interrupt         GLO N/C N/C N/C N/C
   * LED_SIGNAL        4  In a signal handler     GLO N/C N/C N/C N/C
   * LED_ASSERTION     4  An assertion failed     GLO N/C N/C N/C N/C
   * LED_PANIC         4  The system has crashed  2Hz N/C N/C N/C N/C
   */

  switch (led)
    {
      default:
      case 0:
        board_autoled_setone(INDEX_LED_A);
        break;

      case 1:
        board_autoled_setone(INDEX_LED_B);
        break;

      case 2:
        board_autoled_setone(INDEX_LED_C);
        break;

      case 3:
        board_autoled_setone(INDEX_LED_D);
        break;

      case 4:
        pic32mz_gpiowrite(GPIO_LED_L, true);
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* SYMBOL               MEANING                      LED STATE
   *                                               L   A   B   C   D
   * -------------------  ----------------------- --- --- --- --- ---
   * LED_STARTED       0  NuttX has been started  OFF ON  OFF OFF OFF
   * LED_HEAPALLOCATE  1  Heap has been allocated OFF OFF ON  OFF OFF
   * LED_IRQSENABLED   2  Interrupts enabled      OFF OFF OFF ON  OFF
   * LED_STACKCREATED  3  Idle stack created      OFF OFF OFF OFF ON
   * LED_INIRQ         4  In an interrupt         GLO N/C N/C N/C N/C
   * LED_SIGNAL        4  In a signal handler     GLO N/C N/C N/C N/C
   * LED_ASSERTION     4  An assertion failed     GLO N/C N/C N/C N/C
   * LED_PANIC         4  The system has crashed  2Hz N/C N/C N/C N/C
   */

  switch (led)
    {
      default:
        pic32mz_gpiowrite(GPIO_LED_L, false);
        pic32mz_gpiowrite(GPIO_LED_A, false);
        pic32mz_gpiowrite(GPIO_LED_B, false);
        pic32mz_gpiowrite(GPIO_LED_C, false);
        pic32mz_gpiowrite(GPIO_LED_D, false);
        break;

      case 4:
        pic32mz_gpiowrite(GPIO_LED_L, false);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
