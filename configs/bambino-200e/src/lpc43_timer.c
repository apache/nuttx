/****************************************************************************
 * configs/bambino-200e/src/lpc43_timer.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
#include "lpc43_timer.h"

#ifdef CONFIG_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#if !(defined(CONFIG_LPC43_TMR0) || defined(CONFIG_LPC43_TMR1) || defined(CONFIG_LPC43_TMR2) \
     || defined(CONFIG_LPC43_TMR3) )
#  warning "CONFIG_LPC43_TMRx must be defined"
#endif

/* Select the path to the registered watchdog timer device */

#ifndef CONFIG_TIMER0_DEVPATH
#  define CONFIG_TIMER0_DEVPATH "/dev/timer0"
#endif
#ifndef CONFIG_TIMER1_DEVPATH
#  define CONFIG_TIMER1_DEVPATH "/dev/timer1"
#endif
#ifndef CONFIG_TIMER2_DEVPATH
#  define CONFIG_TIMER2_DEVPATH "/dev/timer2"
#endif
#ifndef CONFIG_TIMER3_DEVPATH
#  define CONFIG_TIMER3_DEVPATH "/dev/timer3"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_timerinitialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the timer hardware.
 *
 ****************************************************************************/

int lpc43_timerinitialize(void)
{
  /* Initialize and register the timer devices */

#if defined(CONFIG_LPC43_TMR0)
  tmrinfo("Initializing %s...\n", CONFIG_TIMER0_DEVPATH);
  lpc43_tmrinitialize(CONFIG_TIMER0_DEVPATH, LPC43M4_IRQ_TIMER0);
#endif

#if defined(CONFIG_LPC43_TMR1)
  tmrinfo("Initializing %s...\n", CONFIG_TIMER1_DEVPATH);
  lpc43_tmrinitialize(CONFIG_TIMER1_DEVPATH, LPC43M4_IRQ_TIMER1);
#endif

#if defined(CONFIG_LPC43_TMR2)
  tmrinfo("Initializing %s...\n", CONFIG_TIMER2_DEVPATH);
  lpc43_tmrinitialize(CONFIG_TIMER2_DEVPATH, LPC43M4_IRQ_TIMER2);
#endif

#if defined(CONFIG_LPC43_TMR3)
  tmrinfo("Initializing %s...\n", CONFIG_TIMER3_DEVPATH);
  lpc43_tmrinitialize(CONFIG_TIMER3_DEVPATH, LPC43M4_IRQ_TIMER3);
#endif

  return OK;
}

#endif /* CONFIG_TIMER */
