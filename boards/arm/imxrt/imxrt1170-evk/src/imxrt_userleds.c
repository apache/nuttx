/****************************************************************************
 * boards/arm/imxrt/imxrt1170-evk/src/imxrt_userleds.c
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

/* Copyright 2022 NXP */

/* There are five LED status indicators located on the EVK Board.
 * The functions of these LEDs include:
 *
 *   - Main Power Supply (D16)
 *     Green: DC 5V main supply is normal.
 *     Red:   J43 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset LED - Red (D7)
 *   - OpenSDA LED - Red (D5)
 *   - USER LED 1 - Green (D6)
 *   - USER LED 2 - Red (D34)
 *
 * Only two LEDs, D6 & D34, are under software control.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include <arch/board/board.h>
#include "imxrt1170-evk.h"

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

  imxrt_config_gpio(GPIO_LED1);
  imxrt_config_gpio(GPIO_LED2);

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

  if (led == BOARD_USERLED1)
    {
      ledcfg = GPIO_LED1;
    }
  else if (led == BOARD_USERLED2)
    {
      ledcfg = GPIO_LED2;
    }
  else
    {
      return;
    }

  imxrt_gpio_write(ledcfg, ledon);  /* High illuminates */
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  /* Low illuminates */

  imxrt_gpio_write(GPIO_LED1, ((ledset & BOARD_USERLED1_BIT) != 0));
  imxrt_gpio_write(GPIO_LED2, ((ledset & BOARD_USERLED2_BIT) != 0));
}

#endif /* !CONFIG_ARCH_LEDS */
