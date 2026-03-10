/****************************************************************************
 * boards/sparc/bm3803/xx3803/src/bm3803_boot.c
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
#include <stdio.h>
#include <syslog.h>

#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/timers/oneshot.h>

#include "sparc_internal.h"
#include "bm3803_wdg.h"
#include "bm3803.h"
#include "xx3803.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we support everything until convinced otherwise */

#define HAVE_AM29LV      1

/* Can't support the AM29LV device if it AM29LV support is not enabled */

#if !defined(CONFIG_MTD_AM29LV)
#  undef HAVE_AM29LV
#endif

/* Can't support AM29LV features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_AM29LV
#endif

/* Default AM29LV minor number */

#if defined(HAVE_AM29LV) && !defined(CONFIG_NSH_AM29LVMINOR)
#  define CONFIG_NSH_AM29LVMINOR 0
#endif

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
 *   All bm3803 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void bm3803_boardinitialize(void)
{
  BM3803_REG.mecfg2 = 0x0;
  BM3803_REG.mecfg3 = 0x0;
  BM3803_REG.mecfg1 = 0xf0038000;

  BM3803_REG.mem_cfg1 = 0x14f9f91f;
  BM3803_REG.mem_cfg2 = 0x00078c67;
  BM3803_REG.mem_cfg3 = 0x0;

  /* BM3803_REG.CacheCtrl = 0x0; */

  BM3803_REG.timer_ctrl1 = 0;
  BM3803_REG.timer_cnt1 = 0;
  BM3803_REG.timer_load1 = 0;
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
#ifdef CONFIG_ONESHOT
  struct oneshot_lowerhalf_s *os = NULL;
#endif
  int ret;

  /* Initialize and register the AM29LV FLASH file system. */

#ifdef HAVE_AM29LV
  ret = bm3803_am29lv_initialize(CONFIG_NSH_AM29LVMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize AM29LV minor %d: %d\n",
             CONFIG_NSH_AM29LVMINOR, ret);
      return;
    }
#endif

#ifdef CONFIG_ONESHOT
  os = oneshot_initialize(1, 10);
  if (os)
    {
      ret = oneshot_register("/dev/oneshot", os);
    }
#endif

#ifdef CONFIG_BM3803_WDG
  /* Initialize the watchdog timer */

  bm3803_wdginitialize("/dev/watchdog0");
#endif

#ifdef CONFIG_XX3803_WDG
  /* Start WDG kicker thread */

  ret = xx3803_watchdog_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to start watchdog thread: %d\n", ret);
      return;
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, BM3803_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to mount procfs at %s: %d\n",
           BM3803_PROCFS_MOUNTPOINT, ret);
    }
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
