/****************************************************************************
 * boards/arm/stm32/photon/src/stm32_wdt.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>

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
#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Watchdog daemon thread */

#if defined(CONFIG_PHOTON_WDG_THREAD)

static int wdog_daemon(int argc, char *argv[])
{
  FAR struct file filestruct;
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
      nxsig_usleep((CONFIG_PHOTON_WDG_THREAD_INTERVAL)*1000);

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

#endif /* CONFIG_PHOTON_WDG_THREAD */

/****************************************************************************
 * Name: photon_watchdog_initialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware.
 *   This interface must be provided by all configurations using
 *   apps/examples/watchdog
 *
 ****************************************************************************/

int photon_watchdog_initialize(void)
{
  FAR struct file filestruct;
  int ret = 0;

  /* Open the watchdog device */

  ret = file_open(&filestruct, CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: open %s failed: %d\n", CONFIG_WATCHDOG_DEVPATH, ret);
      return ret;
    }

  /* Set the watchdog timeout */

#ifdef CONFIG_PHOTON_IWDG
  wdinfo("Timeout = %d.\n", CONFIG_PHOTON_IWDG_TIMEOUT);
  ret = file_ioctl(&filestruct, WDIOC_SETTIMEOUT,
                   (unsigned long)CONFIG_PHOTON_IWDG_TIMEOUT);
#else
# error "No watchdog configured"
#endif

  /* Close watchdog as it is not needed here anymore */

  file_close(&filestruct);

  if (ret < 0)
    {
      wderr("ERROR: watchdog configuration failed: %d\n", errno);
      return ret;
    }

#if defined(CONFIG_PHOTON_WDG_THREAD)

  /* Spawn wdog daemon thread */

  int taskid = kthread_create(CONFIG_PHOTON_WDG_THREAD_NAME,
                              CONFIG_PHOTON_WDG_THREAD_PRIORITY,
                              CONFIG_PHOTON_WDG_THREAD_STACKSIZE,
                              (main_t)wdog_daemon, (FAR char * const *)NULL);

  if (taskid <= 0)
    {
      wderr("ERROR: cannot spawn wdog_daemon thread\n");
      return ERROR;
    }

#endif /* CONFIG_PHOTON_WDG_THREAD */

  return OK;
}
