/************************************************************************************
 * configs/sam4s-xplained-pro/src/up_wdt.c
 *
 *   Copyright (C) 2014, 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <errno.h>
#include <debug.h>
#include <sched.h>
#include <stdio.h>
#include <fcntl.h>

#include <nuttx/signal.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include <nuttx/kthread.h>

#include "sam_wdt.h"
#include <nuttx/clock.h>

#ifdef CONFIG_WATCHDOG

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration *******************************************************************/
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

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Watchdog kicker task */

#if defined(CONFIG_WDT_THREAD)
static int wdog_daemon(int argc, char *argv[])
{
  int fd;
  int ret;

  /* Open the watchdog device for reading */

  wdinfo("Opening.\n");
  fd = open(CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      wderr("ERROR: open %s failed: %d\n", CONFIG_WATCHDOG_DEVPATH, errno);
      goto errout;
    }

  /* Start the watchdog timer. */

  wdinfo("Starting.\n");
  ret = ioctl(fd, WDIOC_START, 0);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_START) failed: %d\n", errno);
      goto errout_with_dev;
    }

  nxsig_usleep(200);
  while(1)
    {
      nxsig_usleep((CONFIG_WDT_THREAD_INTERVAL)*1000);

      wdinfo("ping\n");
      ret = ioctl(fd, WDIOC_KEEPALIVE, 0);
      if (ret < 0)
        {
          wderr("ERROR: ioctl(WDIOC_KEEPALIVE) failed: %d\n", errno);
          goto errout_with_dev;
        }
    }

errout_with_dev:
  close(fd);
errout:
  return ERROR;
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
  int fd;
  int ret;

  /* Initialize tha register the watchdog timer device */

  wdinfo("Initializing Watchdog driver...\n");
  sam_wdtinitialize(CONFIG_WATCHDOG_DEVPATH);

  /* Open the watchdog device */

  wdinfo("Opening.\n");
  fd = open(CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      wderr("ERROR: open %s failed: %d\n", CONFIG_WATCHDOG_DEVPATH, errno);
      goto errout;
    }

  /* Set the watchdog timeout */

  wdinfo("Timeout = %d.\n", CONFIG_WDT_TIMEOUT);
  ret = ioctl(fd, WDIOC_SETTIMEOUT, (unsigned long)CONFIG_WDT_TIMEOUT);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_SETTIMEOUT) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Set the watchdog minimum time */

  wdinfo("MinTime = %d.\n", CONFIG_WDT_MINTIME);
  ret = ioctl(fd, WDIOC_MINTIME, (unsigned long)CONFIG_WDT_MINTIME);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_MINTIME) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Start Kicker task */

#if defined(CONFIG_WDT_THREAD)
  sched_lock();

  int taskid = kthread_create(CONFIG_WDT_THREAD_NAME,
                              CONFIG_WDT_THREAD_PRIORITY,
                              CONFIG_WDT_THREAD_STACKSIZE,
                              (main_t)wdog_daemon, (FAR char * const *)NULL);

  DEBUGASSERT(taskid > 0);
  UNUSED(taskid);

  sched_unlock();
#endif
  return OK;
errout_with_dev:
  close(fd);
errout:
  return ERROR;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_WATCHDOG */
