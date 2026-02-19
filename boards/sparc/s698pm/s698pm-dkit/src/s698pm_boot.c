/****************************************************************************
 * boards/sparc/s698pm/s698pm-dkit/src/s698pm_boot.c
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

#include <debug.h>
#include <sys/mount.h>
#include <stdio.h>
#include <syslog.h>

#include <arch/board/board.h>
#include <nuttx/board.h>

#include "sparc_internal.h"
#include "s698pm.h"
#include "s698pm-dkit.h"

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
 * Name: up_boardinitialize
 *
 * Description:
 *   All s698pm architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void s698pm_boardinitialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#ifdef CONFIG_S698PM_WDG
  /* Initialize the watchdog timer */

  s698pm_wdginitialize("/dev/watchdog0");
#endif

#ifdef CONFIG_S698PM_DKIT_WDG
  /* Start WDG kicker thread */

  ret = s698pm_dkit_watchdog_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to start watchdog thread: %d\n", ret);
      return;
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, S698PM_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to mount procfs at %s: %d\n",
             S698PM_PROCFS_MOUNTPOINT, ret);
    }
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
