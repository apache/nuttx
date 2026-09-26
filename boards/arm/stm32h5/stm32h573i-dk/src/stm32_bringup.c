/****************************************************************************
 * boards/arm/stm32h5/stm32h573i-dk/src/stm32_bringup.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <syslog.h>
#include <nuttx/board.h>
#include <nuttx/input/buttons.h>
#include <nuttx/leds/userled.h>

#include "stm32h573i-dk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform board initialization from board_late_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs: %d\n", ret);
    }
#endif

#if defined(CONFIG_USERLED_LOWER) && !defined(CONFIG_ARCH_LEDS)
  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register LEDs: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register buttons: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
