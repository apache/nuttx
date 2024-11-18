/****************************************************************************
 * drivers/power/pm/pm_idle.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SMP

#  define PM_SMP_ALL_CPUS        ((1 << CONFIG_SMP_NCPUS) - 1)
#  define PM_SMP_CPU_DOMAIN(cpu) (CONFIG_PM_NDOMAINS - CONFIG_SMP_NCPUS + (cpu))

static_assert(CONFIG_PM_NDOMAINS >= (CONFIG_SMP_NCPUS + 1),
              "No enough domain for PM SMP to occupy");

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct pm_idle_s
{
  spinlock_t lock;
  cpu_set_t running;
  cpu_set_t firstdone;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pm_idle_s g_pm_idle =
{
  SP_UNLOCKED,
  PM_SMP_ALL_CPUS,
  0,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_idle_unlock
 *
 * Description:
 *   Release SMP idle cpus lock, allow other cpu continue idle process.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pm_idle_unlock(void)
{
  spin_unlock(&g_pm_idle.lock);
}

/****************************************************************************
 * Name: pm_idle_lock
 *
 * Description:
 *   Claim SMP idle cpus lock, other cpu have to wait until released.
 *
 * Input Parameters:
 *   cpu - The current CPU, used to update cpu_set_t
 *
 * Returned Value:
 *   true  - Current CPU is the first one woken from sleep, should handle
 *           system domain restore process also.
 *   false - Current CPU is not the first one woken from sleep, should only
 *           handle cpu domain restore process.
 *
 * Assumptions:
 *   Restore operation pm_changestate(, PM_RESTORE) will done inside pm_idle.
 *   Handler don't have to care about it.
 *
 ****************************************************************************/

bool pm_idle_lock(int cpu)
{
  bool first;
  spin_lock(&g_pm_idle.lock);
  first = (g_pm_idle.running == 0);
  CPU_SET(cpu, &g_pm_idle.running);
  return first;
}

/****************************************************************************
 * Name: pm_idle
 *
 * Description:
 *   Standard pm idle work flow for up_idle.
 *   pm_idle_handler_t will be different prototype when SMP.
 *
 * Input Parameters:
 *   handler - The execution after cpu and system domain state changed.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pm_idle(pm_idle_handler_t handler)
{
  enum pm_state_e systemstate;
  enum pm_state_e oldstate;
  enum pm_state_e newstate;
  irqstate_t flags;
  int domain;
  bool first;
  bool last;
  int cpu;
  int ret;

  systemstate = PM_RESTORE;
  cpu         = this_cpu();
  domain      = PM_SMP_CPU_DOMAIN(cpu);

  /* If sched lock before irq save, and irq handler do post, scheduler will
   * be delayed after WFI until next sched unlock. which is not acceptable.
   */

  flags = up_irq_save();
  sched_lock();

  oldstate = pm_querystate(domain);
  newstate = pm_checkstate(domain);

  ret = pm_changestate(domain, newstate);
  if (ret < 0)
    {
      newstate = oldstate;
    }

  if (oldstate != newstate)
    {
      pm_stay(PM_IDLE_DOMAIN, newstate);
      if (CPU_ISSET(cpu, &g_pm_idle.firstdone))
        {
          pm_relax(PM_IDLE_DOMAIN, oldstate);
        }
      else
        {
          spin_lock(&g_pm_idle.lock);
          CPU_SET(cpu, &g_pm_idle.firstdone);
          spin_unlock(&g_pm_idle.lock);
        }
    }

  spin_lock(&g_pm_idle.lock);
  CPU_CLR(cpu, &g_pm_idle.running);
  last = (g_pm_idle.running == 0);
  if (last)
    {
      systemstate = pm_checkstate(PM_IDLE_DOMAIN);
      ret         = pm_changestate(PM_IDLE_DOMAIN, systemstate);
      if (ret < 0)
        {
          systemstate = PM_NORMAL;
        }
    }

  first = handler(cpu, newstate, systemstate);

  if (first)
    {
      pm_changestate(PM_IDLE_DOMAIN, PM_RESTORE);
    }

  spin_unlock(&g_pm_idle.lock);
  pm_changestate(domain, PM_RESTORE);

  /* If there is pending irq, enable irq make handlers finish all
   * execution will be better decrease scheduler context switch times.
   */

  up_irq_restore(flags);
  sched_unlock();
}

#else

/****************************************************************************
 * Name: pm_idle
 *
 * Description:
 *   Standard pm idle work flow for up_idle.
 *   pm_idle_handler_t will be different prototype when SMP.
 *
 * Input Parameters:
 *   handler - The execution after cpu and system domain state changed.
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

#endif
