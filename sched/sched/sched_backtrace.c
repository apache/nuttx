/****************************************************************************
 * sched/sched/sched_backtrace.c
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
#include <nuttx/sched.h>
#include <nuttx/init.h>

#include "sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_backtrace
 *
 * Description:
 *  Get thread backtrace from specified tid.
 *  Store up to SIZE return address of the current program state in
 *  ARRAY and return the exact number of values stored.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_BACKTRACE
int sched_backtrace(pid_t tid, FAR void **buffer, int size, int skip)
{
  FAR struct tcb_s *tcb = this_task();
  int ret = 0;

  if (tcb->pid == tid)
    {
      ret = up_backtrace(tcb, buffer, size, skip);
    }
  else
    {
      irqstate_t flags = enter_critical_section();

      tcb = nxsched_get_tcb(tid);
      if (tcb != NULL)
        {
#ifdef CONFIG_SMP
          if (tcb->task_state == TSTATE_TASK_RUNNING &&
              g_nx_initstate != OSINIT_PANIC)
            {
              up_cpu_pause(tcb->cpu);
            }
#endif

          ret = up_backtrace(tcb, buffer, size, skip);
#ifdef CONFIG_SMP
          if (tcb->task_state == TSTATE_TASK_RUNNING &&
              g_nx_initstate != OSINIT_PANIC)
            {
              up_cpu_resume(tcb->cpu);
            }
#endif
        }

      leave_critical_section(flags);
    }

  return ret;
}
#endif
