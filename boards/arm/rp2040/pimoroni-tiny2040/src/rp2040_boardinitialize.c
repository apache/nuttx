/****************************************************************************
 * boards/arm/rp2040/pimoroni-tiny2040/src/rp2040_boardinitialize.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "rp2040_gpio.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_initialize.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardearlyinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  rp2040_common_earlyinitialize();
  #endif

  /* --- Place any board specific early initialization here --- */

  /* Set all board RGB LED pins to HIGH:
   * LEDs turned off (driven from external pull-up)
   */

  rp2040_gpio_init(BOARD_GPIO_LED_PIN_R);
  rp2040_gpio_setdir(BOARD_GPIO_LED_PIN_R, true);
  rp2040_gpio_put(BOARD_GPIO_LED_PIN_R, true);

  rp2040_gpio_init(BOARD_GPIO_LED_PIN_G);
  rp2040_gpio_setdir(BOARD_GPIO_LED_PIN_G, true);
  rp2040_gpio_put(BOARD_GPIO_LED_PIN_G, true);

  rp2040_gpio_init(BOARD_GPIO_LED_PIN_B);
  rp2040_gpio_setdir(BOARD_GPIO_LED_PIN_B, true);
  rp2040_gpio_put(BOARD_GPIO_LED_PIN_B, true);
}

/****************************************************************************
 * Name: rp2040_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  rp2040_common_initialize();
  #endif

  /* --- Place any board specific initialization here --- */
}
