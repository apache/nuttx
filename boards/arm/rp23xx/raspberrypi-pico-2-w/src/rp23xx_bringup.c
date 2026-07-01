/****************************************************************************
 * boards/arm/rp23xx/raspberrypi-pico-2-w/src/rp23xx_bringup.c
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
#include <stddef.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include <nuttx/wireless/ieee80211/bcmf_gpio.h>

#include <arch/board/board.h>

#include "rp23xx_pico.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp23xx_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_RP23XX_INFINEON_CYW43439
#include "rp23xx_cyw43439.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RP23XX_INFINEON_CYW43439

/* CYW43439 wireless host-interface GPIO assignments.
 *
 * These match the Raspberry Pi Pico W (RP2040) wiring, which the Pico 2 W
 * carries over unchanged so that existing CYW43 software keeps working.
 * The data line is bidirectional (single-wire gSPI), so it is also used as
 * the host-wake interrupt line.
 *
 * VERIFY: Pico 2 W datasheet -- confirm these GPIO numbers against the
 * official Raspberry Pi Pico 2 W datasheet / schematic before relying on a
 * hardware bring-up.
 */

#  define CYW43439_POWER_ON_GPIO     23   /* WL_ON  - power/enable           */
#  define CYW43439_CHIP_SELECT_GPIO  25   /* WL_CS  - chip select (active 0) */
#  define CYW43439_DATA_GPIO         24   /* WL_D   - bidirectional gSPI data */
#  define CYW43439_CLOCK_GPIO        29   /* WL_CLK - gSPI clock             */

#endif

/****************************************************************************
 * Global Data
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
  int ret = OK;

#ifdef CONFIG_ARCH_BOARD_COMMON

  ret = rp23xx_common_bringup();
  if (ret < 0)
    {
      return ret;
    }

#endif /* CONFIG_ARCH_BOARD_COMMON */

  /* --- Place any board specific bringup code here --- */

#ifdef CONFIG_USERLED
  /* Register the LED driver */

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

#ifdef CONFIG_RP23XX_INFINEON_CYW43439
  /* Initialize the cyw43439 (onboard WiFi chip) over PIO-based gSPI. */

  g_cyw43439 = rp23xx_cyw_setup(CYW43439_POWER_ON_GPIO,
                                CYW43439_CHIP_SELECT_GPIO,
                                CYW43439_DATA_GPIO,
                                CYW43439_CLOCK_GPIO,
                                CYW43439_DATA_GPIO);

  if (g_cyw43439 == NULL)
    {
      ret = errno;

      syslog(LOG_ERR,
             "Failed to initialize cyw43439 (WiFi chip): %d\n",
             ret);

      return ret;
    }
#endif

  return ret;
}
