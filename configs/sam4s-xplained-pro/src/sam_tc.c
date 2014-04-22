/****************************************************************************
 * configs/sam4s-xplained-pro/src/sam_tc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/timer.h>
#include <nuttx/clock.h>
#include <nuttx/kthread.h>

#include <arch/board/board.h>

#include "sam_lowputc.h"
#include "sam_tc.h"

#ifdef CONFIG_TIMER

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Watchdog hardware should be enabled */

#if !defined(CONFIG_SAM34_TC0)
#  warning "CONFIG_SAM34_TC0 must be defined"
#endif

/* Select the path to the registered watchdog timer device */

#ifndef CONFIG_TIMER0_DEVPATH
#  define CONFIG_TIMER0_DEVPATH "/dev/tc0"
#  define CONFIG_TIMER1_DEVPATH "/dev/tc1"
#  define CONFIG_TIMER2_DEVPATH "/dev/tc2"
#  define CONFIG_TIMER3_DEVPATH "/dev/tc3"
#  define CONFIG_TIMER4_DEVPATH "/dev/tc4"
#  define CONFIG_TIMER5_DEVPATH "/dev/tc5"
#endif

/* Timer Definitions ********************************************************/

#define TINTERVAL (3042)

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

  /* Open the timer device */

  tcvdbg("Opening.\n");
  fd = open(CONFIG_TIMER0_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      tcdbg("open %s failed: %d\n", CONFIG_TIMER0_DEVPATH, errno);
      goto errout;
    }

  /* Set the timeout */

  tcvdbg("Timeout = %d.\n", TINTERVAL);
  ret = ioctl(fd, TCIOC_SETTIMEOUT, (unsigned long)TINTERVAL);
  if (ret < 0)
    {
      tcdbg("ioctl(TCIOC_SETTIMEOUT) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Start the timer */

  tcvdbg("Starting.\n");
  ret = ioctl(fd, TCIOC_START, 0);
  if (ret < 0)
    {
      tcdbg("ioctl(TCIOC_START) failed: %d\n", errno);
      goto errout_with_dev;
    }
  
  return OK;

errout_with_dev:
  close(fd);
errout:
  return ERROR;
}

#endif /* CONFIG_TIMER */
