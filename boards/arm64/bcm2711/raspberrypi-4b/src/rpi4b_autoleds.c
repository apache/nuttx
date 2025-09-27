/****************************************************************************
 * boards/arm64/bcm2711/raspberrypi-4b/src/rpi4b_autoleds.c
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

/* LEDs
 *
 * There are two LEDs on the Pi 4B:
 *
 * - Power LED, which is red
 * - Status LED, which is green
 *
 * These LEDs are under the control of the BCM2711 firmware when the board is
 * going through the proprietary boot process. Afterwards, they can be
 * accessed by NuttX through the mailbox API. NuttX will use the status LED
 * for indication of LED_STARTED, and the power LED for indication of
 * LED_PANIC (since it is red).
 *
 *   ------------------- ----------------------- ------ ------
 *   SYMBOL              Meaning                 STATUS POWER
 *   ------------------- ----------------------- ------ ------
 *   LED_STARTED         NuttX has been started  ON     OFF
 *   LED_HEAPALLOCATE    Heap has been allocated N/C    OFF
 *   LED_IRQSENABLED     Interrupts enabled      N/C    OFF
 *   LED_STACKCREATED    Idle stack created      N/C    OFF
 *   LED_INIRQ           In an interrupt         N/C    OFF
 *   LED_SIGNAL          In a signal handler     N/C    OFF
 *   LED_ASSERTION       An assertion failed     N/C    OFF
 *   LED_PANIC           The system has crashed  N/C    FLASH
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include <nuttx/config.h>

#include <arch/board/board.h>
#include <nuttx/board.h>

#include "bcm2711_mailbox.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Make sure that power LED is off and status LED is on.
   * NOTE: the POWER_LED is inverted (true -> off, false -> on)
   */

  bcm2711_mbox_ledset(STATUS_LED, true);
  bcm2711_mbox_ledset(POWER_LED, true);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
    case LED_STARTED:
      bcm2711_mbox_ledset(STATUS_LED, true);
      break;
    case LED_PANIC:
    case LED_ASSERTION:
      bcm2711_mbox_ledset(POWER_LED, false);
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
    case LED_STARTED:
      bcm2711_mbox_ledset(STATUS_LED, false);
      break;
    case LED_PANIC:
    case LED_ASSERTION:
      bcm2711_mbox_ledset(POWER_LED, true);
      break;
    }
}
