/****************************************************************************
 * sched/sched/sched_smp.c
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

#include <nuttx/arch.h>
#include <nuttx/nuttx.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct smp_call_cookie_s
{
  sem_t       sem;
  int         error;
};

struct smp_call_data_s
{
  sq_entry_t                    node[CONFIG_SMP_NCPUS];
  nxsched_smp_call_t            func;
  FAR void                     *arg;
  FAR struct smp_call_cookie_s *cookie;
  spinlock_t                    lock;
  volatile int                  refcount;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sq_queue_t g_smp_call_queue[CONFIG_SMP_NCPUS];
static struct smp_call_data_s g_smp_call_data;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_smp_call_add
 *
 * Description:
 *   Add call data to other processors
 *
 * Input Parameters:
 *   cpu        - Target cpu id
 *   call_data  - Call data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxsched_smp_call_add(int cpu,
                                 FAR struct smp_call_data_s *call_data)
{
  irqstate_t flags;

  flags = enter_critical_section();
  sq_addlast(&call_data->node[cpu], &g_smp_call_queue[cpu]);
  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_smp_call_handler
 *
 * Description:
 *   SMP function call handler
 *
 * Input Parameters:
 *   irq     - Interrupt id
 *   context - Regs context before irq
 *   arg     - Interrupt arg
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

int nxsched_smp_call_handler(int irq, FAR void *context,
                             FAR void *arg)
{
  FAR sq_queue_t *call_queue;
  FAR sq_entry_t *curr;
  FAR sq_entry_t *next;
  int cpu = this_cpu();

  irqstate_t flags = enter_critical_section();

  call_queue = &g_smp_call_queue[cpu];

  up_cpu_paused_save();
  sq_for_every_safe(call_queue, curr, next)
    {
      FAR struct smp_call_data_s *call_data =
        container_of(curr, struct smp_call_data_s, node[cpu]);
      int ret;

      sq_rem(&call_data->node[cpu], call_queue);

      leave_critical_section(flags);

      ret = call_data->func(call_data->arg);

      flags = enter_critical_section();
      if (call_data->cookie != NULL)
        {
          if (ret < 0)
            {
              call_data->cookie->error = ret;
            }

          nxsem_post(&call_data->cookie->sem);
        }

      if (spin_is_locked(&call_data->lock))
        {
          if (--call_data->refcount == 0)
            {
              spin_unlock(&call_data->lock);
            }
        }
    }

  up_cpu_paused_restore();
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nxsched_smp_call_single
 *
 * Description:
 *   Call function on single processor
 *
 * Input Parameters:
 *   cpuid - Target cpu id
 *   func  - Function
 *   arg   - Function args
 *   wait  - Wait function callback or not
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

int nxsched_smp_call_single(int cpuid, nxsched_smp_call_t func,
                            FAR void *arg, bool wait)
{
  cpu_set_t cpuset;

  CPU_ZERO(&cpuset);
  CPU_SET(cpuid, &cpuset);
  return nxsched_smp_call(cpuset, func, arg, wait);
}

/****************************************************************************
 * Name: nxsched_smp_call
 *
 * Description:
 *   Call function on multi processors
 *
 * Input Parameters:
 *   cpuset - Target cpuset
 *   func   - Function
 *   arg    - Function args
 *   wait   - Wait function callback or not
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

int nxsched_smp_call(cpu_set_t cpuset, nxsched_smp_call_t func,
                     FAR void *arg, bool wait)
{
  struct smp_call_data_s call_data_stack =
    {
      0
    };

  struct smp_call_cookie_s cookie =
    {
      0
    };

  FAR struct smp_call_data_s *call_data;
  int remote_cpus = 0;
  int ret = OK;
  int i;

  /* Prevent reschedule on another processor */

  sched_lock();

  if (CPU_ISSET(this_cpu(), &cpuset))
    {
      ret = func(arg);
      if (ret < 0)
        {
          goto out;
        }

      CPU_CLR(this_cpu(), &cpuset);
    }

  remote_cpus = CPU_COUNT(&cpuset);
  if (remote_cpus == 0)
    {
      goto out;
    }

  /* If waiting is necessary, initialize and wait for the cookie. */

  if (wait)
    {
      nxsem_init(&cookie.sem, 0, 0);

      call_data = &call_data_stack;
      call_data->cookie = &cookie;
    }
  else
    {
      call_data = &g_smp_call_data;
      spin_lock(&call_data->lock);
    }

  call_data->func = func;
  call_data->arg  = arg;
  call_data->refcount = remote_cpus;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      if (CPU_ISSET(i, &cpuset))
        {
          nxsched_smp_call_add(i, call_data);
        }
    }

  up_send_smp_call(cpuset);

  if (wait)
    {
      for (i = 0; i < remote_cpus; i++)
        {
          int wait_ret = nxsem_wait_uninterruptible(&cookie.sem);
          if (wait_ret < 0)
            {
              ret = wait_ret;
            }
        }

      if (cookie.error < 0)
        {
          ret = cookie.error;
        }

      nxsem_destroy(&cookie.sem);
    }

out:
  sched_unlock();
  return ret;
}
