/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_userleds.c
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

/* There are two LED status indicators located on Teensy-4.x Board.  The
 * functions of these LEDs include:
 *
  *   - RED LED (loading status)
 *      - dim:    ready
 *      - bright: writing
 *      - blink:  no USB
 *   - USER LED (D3)
 *
 * Only a single LED, D3, is under software control.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include <arch/board/board.h>
#include "teensy-4.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED GPIO for output */

  imxrt_config_gpio(GPIO_LED);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  imxrt_gpio_write(GPIO_LED, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  imxrt_gpio_write(GPIO_LED, (ledset & BOARD_USERLED_BIT));
}

#endif                                 /* !CONFIG_ARCH_LEDS */
