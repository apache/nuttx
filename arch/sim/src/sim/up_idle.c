/****************************************************************************
 * arch/sim/src/sim/up_idle.c
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
#include <debug.h>
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

#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK) || \
    defined(CONFIG_SIM_BUTTONS)
  /* Drive the X11 event loop */

  up_x11events();
#endif

#ifdef CONFIG_SIM_NETDEV
  /* Run the network if enabled */

  netdriver_loop();
#endif

#ifdef CONFIG_SIM_NETUSRSOCK
  usrsock_loop();
#endif

#ifdef CONFIG_RPTUN
  up_rptun_loop();
#endif

#ifdef CONFIG_SIM_HCISOCKET
  bthcisock_loop();
#endif

#ifdef CONFIG_SIM_SOUND
  sim_audio_loop();
#endif

#ifdef CONFIG_ONESHOT
  /* Driver the simulated interval timer */

  up_timer_update();
#endif

#ifdef CONFIG_SIM_MOTOR_FOC
  /* Update simulated FOC device */

  sim_foc_update();
#endif
}
