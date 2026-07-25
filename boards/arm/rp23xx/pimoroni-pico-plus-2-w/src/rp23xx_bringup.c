/****************************************************************************
 * boards/arm/rp23xx/pimoroni-pico-plus-2-w/src/rp23xx_bringup.c
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

#include <nuttx/debug.h>
#include <stddef.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "rp23xx_pico.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp23xx_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_RP23XX_INFINEON_CYW43439
#  include "rp23xx_cyw43439.h"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_RP23XX_INFINEON_CYW43439
gspi_dev_t *g_cyw43439 = NULL;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_bringup
 ****************************************************************************/

int rp23xx_bringup(void)
{
  int ret;

  UNUSED(ret);

#ifdef CONFIG_ARCH_BOARD_COMMON

  ret = rp23xx_common_bringup();
  if (ret < 0)
    {
      return ret;
    }

#endif /* CONFIG_ARCH_BOARD_COMMON */

  /* --- Place any board specific bringup code here --- */

#ifdef CONFIG_RP23XX_INFINEON_CYW43439
  /* Bring up the CYW43439 wireless chip.  This registers the wlan0 network
   * device; the chip's firmware is not downloaded until wlan0 is brought up.
   *
   * Do this before the LED driver, which drives a GPIO on this chip.
   */

  g_cyw43439 = rp23xx_cyw_setup(GPIO_CYW43439_ON,
                                GPIO_CYW43439_CS,
                                GPIO_CYW43439_DATA,
                                GPIO_CYW43439_CLOCK,
                                GPIO_CYW43439_DATA);

  if (g_cyw43439 == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: failed to initialize the cyw43439 (WiFi chip): %d\n",
             errno);
    }
#endif

#if defined(CONFIG_USERLED_LOWER) && defined(CONFIG_RP23XX_INFINEON_CYW43439)
  /* Register the LED driver.  The LED sits on the wireless chip, so there is
   * nothing to register without it.
   */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, \
      "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

  return OK;
}
