/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam_wdt.c
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
#include <sys/types.h>
#include <sys/ioctl.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <sched.h>
#include <stdio.h>
#include <fcntl.h>

#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include <nuttx/kthread.h>

#include "sam_wdt.h"
#include <nuttx/clock.h>

#ifdef CONFIG_WATCHDOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Watchdog hardware should be enabled */

#if !defined(CONFIG_SAM34_WDT)
#  warning "CONFIG_SAM34_WDT must be defined"
#endif

/* Select the path to the registered watchdog timer device */

#ifndef CONFIG_WATCHDOG_DEVPATH
#  ifdef CONFIG_EXAMPLES_WATCHDOG_DEVPATH
#    define CONFIG_WATCHDOG_DEVPATH CONFIG_EXAMPLES_WATCHDOG_DEVPATH
#  else
#    define CONFIG_WATCHDOG_DEVPATH "/dev/watchdog0"
#  endif
#endif

#if (CONFIG_WDT_THREAD_INTERVAL < CONFIG_WDT_MINTIME)
#  error "WDT_THREAD_INTERVAL must be greater than or equal to WDT_MINTIME"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Watchdog kicker task */

#if defined(CONFIG_WDT_THREAD)
static int wdog_daemon(int argc, char *argv[])
{
  struct file filestruct;
  int ret;

  /* Open the watchdog device for reading */

  wdinfo("Opening.\n");
  ret = file_open(&filestruct, CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: open %s failed: %d\n", CONFIG_WATCHDOG_DEVPATH, ret);
      goto errout;
    }

  /* Start the watchdog timer. */

  wdinfo("Starting.\n");
  ret = file_ioctl(&filestruct, WDIOC_START, 0);
  if (ret < 0)
    {
      wderr("ERROR: file_ioctl(WDIOC_START) failed: %d\n", ret);
      goto errout_with_dev;
    }

  nxsig_usleep(200);
  while (1)
    {
      nxsig_usleep((CONFIG_WDT_THREAD_INTERVAL)*1000);

      wdinfo("ping\n");
      ret = file_ioctl(&filestruct, WDIOC_KEEPALIVE, 0);
      if (ret < 0)
        {
          wderr("ERROR: file_ioctl(WDIOC_KEEPALIVE) failed: %d\n", ret);
          goto errout_with_dev;
        }
    }

errout_with_dev:
  file_close(&filestruct);
errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: sam_watchdog_initialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware.
 *   This interface must be provided by all configurations using
 *   apps/examples/watchdog
 *
 ****************************************************************************/

int sam_watchdog_initialize(void)
{
#if (defined(CONFIG_SAM34_WDT) && !defined(CONFIG_WDT_DISABLE_ON_RESET))
  struct file filestruct;
  int ret;

  /* Initialize the watchdog timer device */

  wdinfo("Initializing Watchdog driver...\n");

  sam_wdtinitialize(CONFIG_WATCHDOG_DEVPATH);

  /* Open the watchdog device */

  wdinfo("Opening.\n");

  ret = file_open(&filestruct, CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: open %s failed: %d\n", CONFIG_WATCHDOG_DEVPATH, ret);
      goto errout;
    }

  /* Set the watchdog timeout */

  wdinfo("Timeout = %d.\n", CONFIG_WDT_TIMEOUT);

  ret = file_ioctl(&filestruct, WDIOC_SETTIMEOUT,
                   (unsigned long)CONFIG_WDT_TIMEOUT);
  if (ret < 0)
    {
      wderr("ERROR: file_ioctl(WDIOC_SETTIMEOUT) failed: %d\n", ret);
      goto errout_with_dev;
    }

  /* Set the watchdog minimum time */

  wdinfo("MinTime = %d.\n", CONFIG_WDT_MINTIME);
  ret = file_ioctl(&filestruct, WDIOC_MINTIME,
                   (unsigned long)CONFIG_WDT_MINTIME);
  if (ret < 0)
    {
      wderr("ERROR: file_ioctl(WDIOC_MINTIME) failed: %d\n", ret);
      goto errout_with_dev;
    }

  /* Start Kicker task */

#if defined(CONFIG_WDT_THREAD)
  sched_lock();

  int taskid = kthread_create(CONFIG_WDT_THREAD_NAME,
                              CONFIG_WDT_THREAD_PRIORITY,
                              CONFIG_WDT_THREAD_STACKSIZE,
                              (main_t)wdog_daemon, (char * const *)NULL);

  DEBUGASSERT(taskid > 0);
  UNUSED(taskid);

  sched_unlock();
#endif
  return OK;
errout_with_dev:
  file_close(&filestruct);
errout:
  return ret;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_WATCHDOG */
