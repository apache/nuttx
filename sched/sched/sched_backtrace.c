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

#include <sys/param.h>

#include <string.h>

#include "sched.h"

#ifdef CONFIG_ARCH_HAVE_BACKTRACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SMP) && defined(CONFIG_ARCH_ADDRENV)

/* Depth of the scratch buffer used to relay a remote backtrace back to
 * the caller, since the caller's own buffer may not be mapped in the
 * address environment active on the target CPU.  Requests larger than
 * this are serviced over multiple round trips.
 */

#define BACKTRACE_SCRATCH_DEPTH 32

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_SMP

struct backtrace_arg_s
{
  pid_t pid;
  FAR void **buffer;
  int size;
  int skip;
  bool need_restore;

  /* The return value of up_backtrace() */

  int stacksize;
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
      tcb->flags &= ~TCB_FLAG_CPU_LOCKED;
    }

  leave_critical_section(flags);

  arg->stacksize = up_backtrace(tcb, arg->buffer, arg->size, arg->skip);

  return OK;
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

  if (size <= 0 || buffer == NULL)
    {
      return 0;
    }

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
          if (!OSINIT_IS_PANIC() &&
              tcb->task_state == TSTATE_TASK_RUNNING)
            {
              struct backtrace_arg_s arg;
              bool need_restore;
#ifdef CONFIG_ARCH_ADDRENV
              FAR void *scratch[BACKTRACE_SCRATCH_DEPTH];
#endif

              if ((tcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
                {
                  need_restore = false;
                }
              else
                {
                  need_restore = true;
                  tcb->flags |= TCB_FLAG_CPU_LOCKED;
                }

              arg.pid = tcb->pid;

#ifdef CONFIG_ARCH_ADDRENV
              arg.buffer = scratch;

              while (ret < size)
                {
                  arg.size = MIN(size - ret, BACKTRACE_SCRATCH_DEPTH);
                  arg.skip = skip + ret;

                  /* If this round's request covers all remaining
                   * frames (i.e. it was not capped by the scratch
                   * buffer), there is nothing left to loop for
                   * afterwards, so it is safe to have it release the
                   * pin as well and skip the data-less round below.
                   */

                  arg.need_restore = need_restore &&
                                      arg.size == size - ret;

                  if (nxsched_smp_call_single(tcb->cpu,
                                              sched_backtrace_handler,
                                              &arg) < 0)
                    {
                      break;
                    }

                  if (arg.need_restore)
                    {
                      need_restore = false;
                    }

                  memcpy(&buffer[ret], scratch,
                         arg.stacksize * sizeof(FAR void *));
                  ret += arg.stacksize;

                  if (arg.stacksize < arg.size)
                    {
                      /* Reached the bottom of the target's stack. */

                      break;
                    }
                }

              /* The loop above may have exited without a round
               * releasing the pin (e.g. it broke out on reaching the
               * bottom of the stack before a "guaranteed last round"
               * occurred).  Send one more, data-less round purely to
               * release it.
               */

              if (need_restore)
                {
                  arg.size = 0;
                  arg.need_restore = true;
                  nxsched_smp_call_single(tcb->cpu, sched_backtrace_handler,
                                          &arg);
                }
#else
              arg.need_restore = need_restore;
              arg.buffer = buffer;
              arg.size = size;
              arg.skip = skip;
              ret = nxsched_smp_call_single(tcb->cpu,
                                            sched_backtrace_handler,
                                            &arg) < 0 ? 0 : arg.stacksize;
#endif
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
