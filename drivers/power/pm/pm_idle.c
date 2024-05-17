/****************************************************************************
 * drivers/power/pm/pm_idle.c
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
#include <nuttx/power/pm.h>
#include <sched/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_idle
 *
 * Description:
 *   Standard pm idle work flow for up_idle, for not smp case.
 *
 * Input Parameters:
 *   handler - The execution after PM_IDLE_DOMAIN state changed.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pm_idle(pm_idle_handler_t handler)
{
  enum pm_state_e newstate;
  irqstate_t flags;
  int ret;

  /* If sched lock before irq save, and irq handler do post, scheduler will
   * be delayed after WFI until next sched unlock. which is not acceptable.
   */

  flags = up_irq_save();
  sched_lock();

  newstate = pm_checkstate(PM_IDLE_DOMAIN);
  ret      = pm_changestate(PM_IDLE_DOMAIN, newstate);
  if (ret < 0)
    {
      newstate = PM_NORMAL;
    }

  handler(newstate);

  pm_changestate(PM_IDLE_DOMAIN, PM_RESTORE);

  /* If there is pending irq, enable irq make handlers finish all execution
   * will be better decrease scheduler context switch times.
   */

  up_irq_restore(flags);
  sched_unlock();
}
