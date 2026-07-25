/****************************************************************************
 * boards/arm/rp23xx/pimoroni-pico-plus-2-w/src/rp23xx_userleds.c
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
 * The single LED on this board is wired to GPIO 0 of the CYW43439 wireless
 * chip rather than to a pin of the RP2350, so setting it means sending an
 * iovar request over the gSPI bus.  Two consequences:
 *
 *   - The LED only responds once the wireless chip is running, which happens
 *     when wlan0 is brought up.  Before that every write fails with -EIO.
 *   - Writing the LED blocks, so it must not be done from interrupt context.
 *     That is why this board implements the user LED interface only and not
 *     the CONFIG_ARCH_LEDS auto-LED interface.
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
#include <arch/board/rp23xx_extra_gpio.h>

#if defined(CONFIG_RP23XX_INFINEON_CYW43439) && !defined(CONFIG_ARCH_LEDS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Nothing to configure -- the LED lives on the wireless chip, which the
   * board bringup has already set up.
   */

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if (led == BOARD_LED1)
    {
      int ret = rp23xx_extra_gpio_put(RP23XX_EXTRA_GPIO_LED, ledon);

      if (ret < 0)
        {
          lederr("ERROR: cannot reach the LED on the CYW43439: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  board_userled(BOARD_LED1, (ledset & BOARD_LED1_BIT) != 0);
}

#endif /* CONFIG_RP23XX_INFINEON_CYW43439 && !CONFIG_ARCH_LEDS */
