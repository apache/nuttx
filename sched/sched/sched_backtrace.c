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

#ifdef CONFIG_ARCH_HAVE_BACKTRACE

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

#ifdef CONFIG_SMP
struct backtrace_arg_s
{
  pid_t pid;
  FAR void **buffer;
  int size;
  int skip;
  cpu_set_t saved_affinity;
  bool need_restore;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sched_backtrace_handler(FAR void *cookie)
{
  FAR struct backtrace_arg_s *arg = cookie;
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  flags = enter_critical_section();

  tcb = nxsched_get_tcb(arg->pid);

  if (!tcb || tcb->task_state == TSTATE_TASK_INVALID ||
      (tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      /* There is no TCB with this pid or, if there is, it is not a task. */

      leave_critical_section(flags);
      return -ESRCH;
    }

  if (arg->need_restore)
    {
      tcb->affinity = arg->saved_affinity;
      tcb->flags &= ~TCB_FLAG_CPU_LOCKED;
    }

  leave_critical_section(flags);

  return up_backtrace(tcb, arg->buffer, arg->size, arg->skip);
}
#endif

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
              struct backtrace_arg_s arg;

              if ((tcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
                {
                  arg.pid = tcb->pid;
                  arg.need_restore = false;
                }
              else
                {
                  arg.pid = tcb->pid;
                  arg.saved_affinity = tcb->affinity;
                  arg.need_restore = true;

                  tcb->flags |= TCB_FLAG_CPU_LOCKED;
                  CPU_SET(tcb->cpu, &tcb->affinity);
                }

              arg.buffer = buffer;
              arg.size = size;
              arg.skip = skip;
              ret = nxsched_smp_call_single(tcb->cpu,
                                            sched_backtrace_handler,
                                            &arg);
            }
          else
#endif
            {
              ret = up_backtrace(tcb, buffer, size, skip);
            }
        }

      leave_critical_section(flags);
    }

  return ret;
}
#endif
