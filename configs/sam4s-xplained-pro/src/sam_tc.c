/****************************************************************************
 * configs/sam4s-xplained-pro/src/sam_tc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Bob Doiron
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

#include <nuttx/arch.h>
#include <nuttx/timers/timer.h>
#include <nuttx/clock.h>
#include <nuttx/kthread.h>

#include <arch/board/board.h>

#include "sam_lowputc.h"
#include "sam_tc.h"
#include "sam_rtt.h"

#ifdef CONFIG_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#if !(defined(CONFIG_SAM34_TC0) || defined(CONFIG_SAM34_TC1) || defined(CONFIG_SAM34_TC2) \
     || defined(CONFIG_SAM34_TC3) || defined(CONFIG_SAM34_TC4) || defined(CONFIG_SAM34_RTT) )
#  warning "CONFIG_SAM34_TCx or CONFIG_SAM34_RTT must be defined"
#endif

/* Select the path to the registered watchdog timer device */

#ifndef CONFIG_TIMER0_DEVPATH
#  define CONFIG_TIMER0_DEVPATH "/dev/tc0"
#endif
#ifndef CONFIG_TIMER1_DEVPATH
#  define CONFIG_TIMER1_DEVPATH "/dev/tc1"
#endif
#ifndef CONFIG_TIMER2_DEVPATH
#  define CONFIG_TIMER2_DEVPATH "/dev/tc2"
#endif
#ifndef CONFIG_TIMER3_DEVPATH
#  define CONFIG_TIMER3_DEVPATH "/dev/tc3"
#endif
#ifndef CONFIG_TIMER4_DEVPATH
#  define CONFIG_TIMER4_DEVPATH "/dev/tc4"
#endif
#ifndef CONFIG_TIMER5_DEVPATH
#  define CONFIG_TIMER5_DEVPATH "/dev/tc5"
#endif
#ifndef CONFIG_RTT_DEVPATH
#  define CONFIG_RTT_DEVPATH "/dev/rtt0"
#endif

/* Timer Definitions ********************************************************/

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog
 * timer
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_TIMER
#endif

#ifdef CONFIG_DEBUG_TIMER
#  define tcdbg                 dbg
#  define tclldbg               lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define tcvdbg              vdbg
#    define tcllvdbg            llvdbg
#  else
#    define tcvdbg(x...)
#    define tcllvdbg(x...)
#  endif
#else
#  define tcdbg(x...)
#  define tclldbg(x...)
#  define tcvdbg(x...)
#  define tcllvdbg(x...)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_SYSTEMTICK_EXTCLK) && !defined(CONFIG_SUPPRESS_INTERRUPTS) && \
    !defined(CONFIG_SUPPRESS_TIMER_INTS)

static bool systemtick(FAR uint32_t *next_interval_us)
{
  sched_process_timer();
  return true; // reload, no change to interval
}

#endif /* CONFIG_SYSTEMTICK_EXTCLK && !CONFIG_SUPPRESS_INTERRUPTS && !CONFIG_SUPPRESS_TIMER_INTS */

#if defined(CONFIG_SCHED_CPULOAD) && defined(CONFIG_SCHED_CPULOAD_EXTCLK)

static bool calc_cpuload(FAR uint32_t *next_interval_us)
{
  sched_process_cpuload();
  return TRUE; /* Reload, no change to interval */
}

#endif /* CONFIG_SCHED_CPULOAD && CONFIG_SCHED_CPULOAD_EXTCLK */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_timerinitialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the timer hardware.
 *
 ****************************************************************************/

int sam_timerinitialize(void)
{
  int fd;
  int ret;

  /* Initialize and register the timer devices */

#if defined(CONFIG_SAM34_TC0)
  tcvdbg("Initializing %s...\n", CONFIG_TIMER0_DEVPATH);
  sam_tcinitialize(CONFIG_TIMER0_DEVPATH, SAM_IRQ_TC0);
#endif

#if defined(CONFIG_SAM34_TC1)
  tcvdbg("Initializing %s...\n", CONFIG_TIMER1_DEVPATH);
  sam_tcinitialize(CONFIG_TIMER1_DEVPATH, SAM_IRQ_TC1);
#endif

#if defined(CONFIG_SAM34_TC2)
  tcvdbg("Initializing %s...\n", CONFIG_TIMER2_DEVPATH);
  sam_tcinitialize(CONFIG_TIMER2_DEVPATH, SAM_IRQ_TC2);
#endif

#if defined(CONFIG_SAM34_TC3)
  tcvdbg("Initializing %s...\n", CONFIG_TIMER3_DEVPATH);
  sam_tcinitialize(CONFIG_TIMER3_DEVPATH, SAM_IRQ_TC3);
#endif

#if defined(CONFIG_SAM34_TC4)
  tcvdbg("Initializing %s...\n", CONFIG_TIMER4_DEVPATH);
  sam_tcinitialize(CONFIG_TIMER4_DEVPATH, SAM_IRQ_TC4);
#endif

#if defined(CONFIG_SAM34_TC5)
  tcvdbg("Initializing %s...\n", CONFIG_TIMER5_DEVPATH);
  sam_tcinitialize(CONFIG_TIMER5_DEVPATH, SAM_IRQ_TC5);
#endif

#if defined(CONFIG_SAM34_RTT)
  tcvdbg("Initializing %s...\n", CONFIG_RTT_DEVPATH);
  sam_rttinitialize(CONFIG_RTT_DEVPATH);
#endif

#if defined(CONFIG_SYSTEMTICK_EXTCLK) && !defined(CONFIG_SUPPRESS_INTERRUPTS) && \
    !defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* System Timer Initialization */

  tcvdbg("Opening %s\n", CONFIG_SAM4S_XPLAINED_PRO_SCHED_TIMER_DEVPATH);

  fd = open(CONFIG_SAM4S_XPLAINED_PRO_SCHED_TIMER_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      tcdbg("open %s failed: %d\n",
            CONFIG_SAM4S_XPLAINED_PRO_SCHED_TIMER_DEVPATH, errno);
      goto errout;
    }

  /* Set the timeout */

  tcvdbg("Interval = %d us.\n",  (unsigned long)USEC_PER_TICK);
  ret = ioctl(fd, TCIOC_SETTIMEOUT, (unsigned long)USEC_PER_TICK);
  if (ret < 0)
    {
      tcdbg("ioctl(TCIOC_SETTIMEOUT) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* install user callback */
  {
    struct timer_sethandler_s tccb;
    tccb.newhandler = systemtick;
    tccb.oldhandler = NULL;

    ret = ioctl(fd, TCIOC_SETHANDLER, (unsigned long)&tccb);
    if (ret < 0)
      {
        tcdbg("ioctl(TCIOC_SETHANDLER) failed: %d\n", errno);
        goto errout_with_dev;
      }
  }

  /* Start the timer */

  tcvdbg("Starting.\n");
  ret = ioctl(fd, TCIOC_START, 0);
  if (ret < 0)
    {
      tcdbg("ioctl(TCIOC_START) failed: %d\n", errno);
      goto errout_with_dev;
    }
#endif

#if defined(CONFIG_SCHED_CPULOAD) && defined(CONFIG_SCHED_CPULOAD_EXTCLK)
  /* CPU Load initialization */

  tcvdbg("Opening %s\n", CONFIG_SAM4S_XPLAINED_PRO_CPULOAD_TIMER_DEVPATH);

  fd = open(CONFIG_SAM4S_XPLAINED_PRO_CPULOAD_TIMER_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      tcdbg("open %s failed: %d\n",
            CONFIG_SAM4S_XPLAINED_PRO_CPULOAD_TIMER_DEVPATH, errno);
      goto errout;
    }

  /* Set the timeout */

  tcvdbg("Interval = %d us.\n",  (unsigned long)1000000 / CONFIG_SCHED_CPULOAD_TICKSPERSEC);

  ret = ioctl(fd, TCIOC_SETTIMEOUT,
             (unsigned long)1000000 / CONFIG_SCHED_CPULOAD_TICKSPERSEC);
  if (ret < 0)
    {
      tcdbg("ioctl(TCIOC_SETTIMEOUT) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Install user callback */

  {
    struct timer_sethandler_s tccb;
    tccb.newhandler = calc_cpuload;
    tccb.oldhandler = NULL;

    ret = ioctl(fd, TCIOC_SETHANDLER, (unsigned long)&tccb);
    if (ret < 0)
      {
        tcdbg("ioctl(TCIOC_SETHANDLER) failed: %d\n", errno);
        goto errout_with_dev;
      }
  }

  /* Start the timer */

  tcvdbg("Starting.\n");
  ret = ioctl(fd, TCIOC_START, 0);
  if (ret < 0)
    {
      tcdbg("ioctl(TCIOC_START) failed: %d\n", errno);
      goto errout_with_dev;
    }
#endif

  goto success;
errout_with_dev:
  close(fd);
errout:
  return ERROR;

success:
  return OK;

}

#endif /* CONFIG_TIMER */
