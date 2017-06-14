/****************************************************************************
 * arch/sim/src/up_idle.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <pthread.h>
#include <time.h>

#include <nuttx/arch.h>

#ifdef CONFIG_PM
#  include <nuttx/power/pm.h>
#endif

#ifdef CONFIG_SMP
#  include <nuttx/spinlock.h>
#endif

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PM_IDLE_DOMAIN 0 /* Revisit */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SIM_X11FB
static int g_x11refresh = 0;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_SIM_WALLTIME) || defined(CONFIG_SIM_X11FB)
extern int up_hostusleep(unsigned int usec);
#ifdef CONFIG_SIM_X11FB
extern void up_x11update(void);
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their
 *   is no other ready-to-run task.  This is processor idle
 *   time and will continue until some interrupt occurs to
 *   cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g.,
 *   this is where power management operations might be
 *   performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#ifdef CONFIG_SMP
  /* In the SMP configuration, only one CPU should do these operations.  It
   * should not matter which, however.
   */

  static volatile spinlock_t lock SP_SECTION = SP_UNLOCKED;

  /* The one that gets the lock is the one that executes the IDLE operations */

  if (up_testset(&lock) != SP_UNLOCKED)
    {
      /* We didn't get it... Give other pthreads/CPUs a shot and try again
       * later.
       */

      pthread_yield();
      return;
    }
#endif

#ifdef CONFIG_SCHED_TICKLESS
  /* Driver the simulated interval timer */

  up_timer_update();
#else
  /* If the system is idle, then process "fake" timer interrupts.
   * Hopefully, something will wake up.
   */

  sched_process_timer();
#endif

#if defined(CONFIG_DEV_CONSOLE) && !defined(CONFIG_SIM_UART_DATAPOST)
  /* Handle UART data availability */

  if (g_uart_data_available)
    {
      g_uart_data_available = 0;
      simuart_post();
    }
#endif

#if defined(CONFIG_NET_ETHERNET) && defined(CONFIG_SIM_NETDEV)
  /* Run the network if enabled */

  netdriver_loop();
#endif

#ifdef CONFIG_PM
  /* Fake some power management stuff for testing purposes */

  {
    static enum pm_state_e state = PM_NORMAL;
    enum pm_state_e newstate;

    newstate = pm_checkstate(PM_IDLE_DOMAIN);
    if (newstate != state)
      {
        if (pm_changestate(PM_IDLE_DOMAIN, newstate) == OK)
          {
            state = newstate;
          }
      }
  }
#endif

#if defined(CONFIG_SIM_WALLTIME) || defined(CONFIG_SIM_X11FB)
  /* Wait a bit so that the sched_process_timer() is called close to the
   * correct rate.
   */

  (void)up_hostusleep(1000000 / CLK_TCK);

  /* Handle X11-related events */

#ifdef CONFIG_SIM_X11FB
  if (g_x11initialized)
    {
#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK)
       /* Drive the X11 event loop */

      if (g_eventloop)
        {
          up_x11events();
        }
#endif

      /* Update the display periodically */

      g_x11refresh += 1000000 / CLK_TCK;
      if (g_x11refresh > 500000)
        {
          up_x11update();
        }
    }
#endif
#endif

#ifdef CONFIG_SMP
  /* Release the spinlock */

  lock = SP_UNLOCKED;

  /* Give other pthreads/CPUs a shot */

  pthread_yield();
#endif
}
