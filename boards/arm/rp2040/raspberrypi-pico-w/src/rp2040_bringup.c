/****************************************************************************
 * boards/arm/rp2040/raspberrypi-pico-w/src/rp2040_bringup.c
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

#include <nuttx/wireless/ieee80211/bcmf_gpio.h>

#include <arch/board/board.h>

#include "rp2040_pico.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#ifdef CONFIG_RP2040_INFINEON_CYW43439
#include "rp2040_cyw43439.h"
#endif

/****************************************************************************
 * Global Data
 ****************************************************************************/

#ifdef CONFIG_RP2040_INFINEON_CYW43439
gspi_dev_t *g_cyw43439 = NULL;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_bringup
 ****************************************************************************/

int rp2040_bringup(void)
{
#ifdef CONFIG_ARCH_BOARD_COMMON

  int ret = rp2040_common_bringup();
  if (ret < 0)
    {
      return ret;
    }

#endif /* CONFIG_ARCH_BOARD_COMMON */

  /* --- Place any board specific bringup code here --- */

#define CYW43439_POWER_ON_GPIO     23
#define CYW43439_CHIP_SELECT_GPIO  25
#define CYW43439_DATA_GPIO         24
#define CYW43439_CLOCK_GPIO        29

#ifdef CONFIG_RP2040_INFINEON_CYW43439

  g_cyw43439 = rp2040_cyw_setup(CYW43439_POWER_ON_GPIO,
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

  return OK;
}
