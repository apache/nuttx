/****************************************************************************
 * boards/arm64/am62x/beagleplay/src/beagleplay_autoleds.c
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

/* GPIO-backed LED support is not implemented yet for the AM62x boards.
 * Keep the ARCH_LEDS hooks as explicit no-ops until the GPIO work lands.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "beagleplay.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: beagleplay_led_initialize
 *
 * Description:
 *   Configure LED GPIO pins as outputs, all OFF initially.
 *   Stubbed until the GPIO driver is available.
 *
 ****************************************************************************/

void beagleplay_led_initialize(void)
{
  /* TODO: configure GPIO pins for the user LEDs once GPIO support exists. */
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turn ON the LED(s) associated with the OS event 'led'.
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  UNUSED(led);
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turn OFF the LED(s) associated with the OS event 'led'.
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  UNUSED(led);
}

#endif /* CONFIG_ARCH_LEDS */
