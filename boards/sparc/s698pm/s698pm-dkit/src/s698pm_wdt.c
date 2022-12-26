/****************************************************************************
 * boards/sparc/s698pm/s698pm-dkit/src/s698pm_wdt.c
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

#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>
#include <stdio.h>
#include <fcntl.h>

#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include <nuttx/kthread.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Watchdog daemon thread */

#if defined(CONFIG_S698PM_DKIT_WDG_THREAD)

static int wdog_daemon(int argc, char *argv[])
{
  struct file filestruct;
  int ret;

  /* Open watchdog device */

  ret = file_open(&filestruct, CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: open %s failed: %d\n", CONFIG_WATCHDOG_DEVPATH, ret);
      return ret;
    }

  /* Start watchdog timer */

  ret = file_ioctl(&filestruct, WDIOC_START, 0);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_START) failed: %d\n", errno);
      goto exit_close_dev;
    }

  while (1)
    {
      nxsig_usleep((CONFIG_S698PM_DKIT_WDG_THREAD_INTERVAL)*1000);

      /* Send keep alive ioctl */

      ret = file_ioctl(&filestruct, WDIOC_KEEPALIVE, 0);
      if (ret < 0)
        {
          wderr("ERROR: ioctl(WDIOC_KEEPALIVE) failed: %d\n", errno);
          break;
        }
    }

exit_close_dev:

  /* Close watchdog device and exit. */

  file_close(&filestruct);
  return ret;
}

#endif /* CONFIG_S698PM_DKIT_WDG_THREAD */

/****************************************************************************
 * Name: s698pm_dkit_watchdog_initialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware.
 *   This interface must be provided by all configurations using
 *   apps/examples/watchdog
 *
 ****************************************************************************/

int s698pm_dkit_watchdog_initialize(void)
{
  struct file filestruct;
  int ret = 0;

  /* Open the watchdog device */

  ret = file_open(&filestruct, CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: open %s failed: %d\n", CONFIG_WATCHDOG_DEVPATH, ret);
      return ret;
    }

  /* Set the watchdog timeout */

#ifdef CONFIG_S698PM_DKIT_WDG
  wdinfo("Timeout = %d.\n", CONFIG_S698PM_DKIT_WDG_TIMEOUT);
  ret = file_ioctl(&filestruct, WDIOC_SETTIMEOUT,
                   (unsigned long)CONFIG_S698PM_DKIT_WDG_TIMEOUT);
#else
# error "No watchdog configured"
#endif

  /*  Close watchdog as it is not needed here anymore */

  file_close(&filestruct);

  if (ret < 0)
    {
      wderr("ERROR: watchdog configuration failed: %d\n", errno);
      return ret;
    }

#if defined(CONFIG_S698PM_DKIT_WDG_THREAD)

  sched_lock();

  /* Spawn wdog daemon thread */

  int taskid = kthread_create(CONFIG_S698PM_DKIT_WDG_THREAD_NAME,
                              CONFIG_S698PM_DKIT_WDG_THREAD_PRIORITY,
                              CONFIG_S698PM_DKIT_WDG_THREAD_STACKSIZE,
                              wdog_daemon, NULL);

  DEBUGASSERT(taskid > 0);
  UNUSED(taskid);

  sched_unlock();

#endif /* CONFIG_S698PM_DKIT_WDG_THREAD */

  return OK;
}
