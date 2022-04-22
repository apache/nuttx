/****************************************************************************
 * arch/ceva/src/common/up_idle.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/power/pm.h>

#include "up_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_idlepm(void)
{
  enum pm_state_e newstate;

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate(PM_IDLE_DOMAIN);

  /* Then force the global state change */

  pm_changestate(PM_IDLE_DOMAIN, newstate);

  /* The change may fail, let's get the final state from power manager */

  newstate = pm_querystate(PM_IDLE_DOMAIN);

#ifdef CONFIG_PM_KEEPBUSY
  /* Check whether need keep CPU busy */

  if (pm_keepbusy(PM_IDLE_DOMAIN, newstate))
    {
      return;
    }
#endif

  /* MCU-specific power management logic */

  switch (newstate)
    {
    case PM_NORMAL:
      up_cpu_doze();
      break;

    case PM_IDLE:
      up_cpu_idle();
      break;

    case PM_STANDBY:
      up_cpu_standby();
      break;

    case PM_SLEEP:
      up_cpu_sleep();
      break;

    default:
      break;
    }
}
#else
#  define up_idlepm() up_cpu_idle()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed
 *   when their is no other ready-to-run task.  This is processor
 *   idle time and will continue until some interrupt occurs to
 *   cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g.,
 *   this is where power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
  irqstate_t flags;

  flags = up_irq_save();
  sched_lock();

  /* Perform IDLE mode power management */

  up_idlepm();

  /* Quit lower power mode, restore to PM_NORMAL */

  up_cpu_normal();
  pm_changestate(PM_IDLE_DOMAIN, PM_RESTORE);

  sched_unlock();
  up_irq_restore(flags);
}

/****************************************************************************
 * Power callback default implementation
 ****************************************************************************/

void weak_function up_cpu_normal(void)
{
}

/****************************************************************************
 * Name: up_pminitialize
 *
 * Description:
 *   This function is called by MCU-specific logic at power-on reset in
 *   order to provide one-time initialization the power management subsystem.
 *   This function must be called *very* early in the initialization sequence
 *   *before* any other device drivers are initialized (since they may
 *   attempt to register with the power management subsystem).
 *
 * Input parameters:
 *   None.
 *
 * Returned value:
 *    None.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void up_pminitialize(void)
{
  /* Initialize the NuttX power management subsystem proper */

  pm_initialize();
}
#endif
