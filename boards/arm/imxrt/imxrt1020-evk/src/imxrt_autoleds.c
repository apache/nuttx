/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/src/imxrt_autoleds.c
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

/* There is one user accessible LED status indicator located on the 1020-EVK.
 * The function of the LEDs include:
 *
 * D3: Power (Green) & Overpower (Red)
 * D5: User LED (Green) GPIO_AD_B0_05
 * D15: RST LED (Red)
 *
 * This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as documented in board.h
 *
 * The intention is that if the LED is statically on, NuttX has successfully
 * booted and is, apparently, running normally.  If the LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system has
 * halted.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "imxrt1020-evk.h"

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt_autoled_initialize(void)
{
  /* Configure LED GPIO for output */

  imxrt_config_gpio(GPIO_USERLED);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turn on the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledoff = false;

  switch (led)
    {
      case 0:  /* LED Off */
        ledoff = true;
        break;

      case 2:  /* LED No change */
        return;

      case 1:  /* LED On */
      case 3:  /* LED On */
        break;
    }

  imxrt_gpio_write(GPIO_USERLED, ledoff); /* Low illuminates */
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turn off the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
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

  imxrt_gpio_write(GPIO_USERLED, true); /* Low illuminates */
}

#endif /* CONFIG_ARCH_LEDS */
