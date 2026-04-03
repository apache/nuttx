/****************************************************************************
 * boards/risc-v/k210/maix-bit/src/k210_bringup.c
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

#include <stdbool.h>
#include <stdio.h>
#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_USERLED_LOWER
#  include <nuttx/leds/userled.h>
#endif

#include "k210.h"
#include "k210_wdt.h"
#include "maix-bit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_bringup
 ****************************************************************************/

int k210_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
    }
#endif

#ifdef CONFIG_USERLED_LOWER
  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_K210_WDT0
  ret = k210_wdt_initialize(CONFIG_WATCHDOG_DEVPATH, K210_WDT_DEVICE0);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WARNING: Failed to initialize WDT0: %d\n", ret);
    }
#endif

#ifdef CONFIG_K210_WDT1
  ret = k210_wdt_initialize(
#ifdef CONFIG_K210_WDT0
      "/dev/watchdog1",
#else
      CONFIG_WATCHDOG_DEVPATH,
#endif
      K210_WDT_DEVICE1);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WARNING: Failed to initialize WDT1: %d\n", ret);
    }
#endif

  return ret;
}
