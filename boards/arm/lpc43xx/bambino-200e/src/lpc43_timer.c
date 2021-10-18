/****************************************************************************
 * boards/arm/lpc43xx/bambino-200e/src/lpc43_timer.c
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
