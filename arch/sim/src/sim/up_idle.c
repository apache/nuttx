/****************************************************************************
 * arch/sim/src/sim/up_idle.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2014, 2016, 2020 Gregory Nutt. All
 *     rights reserved.
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

#include <nuttx/arch.h>
#include <nuttx/power/pm.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PM_IDLE_DOMAIN 0 /* Revisit */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there
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
#ifdef CONFIG_PM
  static enum pm_state_e state = PM_NORMAL;
  enum pm_state_e newstate;

  /* Fake some power management stuff for testing purposes */

  newstate = pm_checkstate(PM_IDLE_DOMAIN);
  if (newstate != state)
    {
      if (pm_changestate(PM_IDLE_DOMAIN, newstate) == OK)
        {
          state = newstate;
          pwrinfo("IDLE: switching to new state %i\n", state);
        }
    }
#endif

  /* Handle UART data availability */

  up_uartloop();

#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK)
  /* Drive the X11 event loop */

  up_x11events();
#endif

#ifdef CONFIG_SIM_NETDEV
  /* Run the network if enabled */

  netdriver_loop();
#endif

#ifdef CONFIG_RPTUN
  up_rptun_loop();
#endif

#ifdef CONFIG_SIM_HCISOCKET
  bthcisock_loop();
#endif

#ifdef CONFIG_ONESHOT
  /* Driver the simulated interval timer */

  up_timer_update();
#endif
}
