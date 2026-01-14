/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-xiao/src/esp32s3_autoleds.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "esp32s3_gpio.h"
#include "esp32s3-xiao.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*  There is one user-controllable LED on the Seeed Studio XIAO RA4M1 board:
 *
 *     LED              GPIO
 *     ---------------- -----
 *     L   Yellow LED   P011
 *
 * The LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined. In that case, the usage by the board port is defined in
 * include/board.h and src/esp32s3_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   SYMBOL                MEANING               LED STATE
 *                                                   L
 *   -------------------  -----------------------  --------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            N/C
 *   LED_SIGNAL           In a signal handler        N/C
 *   LED_ASSERTION        An assertion failed        N/C
 *   LED_PANIC            The system has crashed     N/C
 *   LED_IDLE             MCU is is sleep mode      ------
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure RX and TX LED GPIOs for output */

  esp32s3_configgpio(GPIO_LED1, OUTPUT);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledon = true;

  switch (led)
    {
      case 0:  /* LED Off */
        ledon = false;
        break;

      case 2:  /* LED No change */
        return;

      case 1:  /* LED On */
      case 3:  /* LED On */
        break;
    }

  /* High illuminates */

  esp32s3_gpiowrite(GPIO_LED1, ledon);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case 0:  /* LED Off */
      case 1:  /* LED Off */
      case 3:  /* LED Off */
        break;

      case 2:  /* LED No change */
        return;
    }

  /* High illuminates */

  esp32s3_gpiowrite(GPIO_LED1, false);
}

#endif /* CONFIG_ARCH_LEDS */