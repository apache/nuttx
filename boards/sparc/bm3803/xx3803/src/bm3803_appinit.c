/****************************************************************************
 * boards/sparc/bm3803/xx3803/src/bm3803_appinit.c
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

#include <nuttx/board.h>
#include <nuttx/timers/oneshot.h>
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
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
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
      return ret;
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
      return ret;
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

  return ret;
}
