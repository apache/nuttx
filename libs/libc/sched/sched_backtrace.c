/****************************************************************************
 * libs/libc/sched/sched_backtrace.c
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

#ifndef CONFIG_ARCH_HAVE_BACKTRACE

#include <sys/types.h>
#include <unistd.h>
#include <execinfo.h>
#include <unwind.h>

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct trace_arg
{
  FAR void    **array;
  _Unwind_Word  cfa;
  int           cnt;
  int           size;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

weak_function
FAR void *unwind_arch_adjustment(FAR void *prev, FAR void *addr)
{
  return addr;
}

static _Unwind_Reason_Code
backtrace_helper(FAR struct _Unwind_Context *ctx, FAR void *a)
{
  FAR struct trace_arg *arg = a;

  /* We are first called with address in the backtrace function.
   * Skip it.
   */

  if (arg->cnt >= 0)
    {
      arg->array[arg->cnt] = (FAR void *)_Unwind_GetIP(ctx);
      if (arg->cnt > 0)
        {
          arg->array[arg->cnt]
            = unwind_arch_adjustment(arg->array[arg->cnt - 1],
                arg->array[arg->cnt]);
        }

      /* Check whether we make any progress. */

      _Unwind_Word cfa = _Unwind_GetCFA(ctx);

      if (arg->cnt > 0 &&
          arg->array[arg->cnt - 1] == arg->array[arg->cnt] &&
          cfa == arg->cfa)
        {
          return _URC_END_OF_STACK;
        }

      arg->cfa = cfa;
    }

  if (++arg->cnt >= arg->size)
    {
      return _URC_END_OF_STACK;
    }

  return _URC_NO_REASON;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_backtrace
 *
 * Description:
 *  Get thread backtrace from specified tid.
 *
 ****************************************************************************/

int sched_backtrace(pid_t tid, FAR void **buffer, int size, int skip)
{
  struct trace_arg arg;

  if (tid != gettid())
    {
      return 0;
    }

  arg.array = buffer;
  arg.cfa = 0;
  arg.size = size;
  arg.cnt = -skip - 1;

  if (size <= 0)
    {
      return 0;
    }

  _Unwind_Backtrace(backtrace_helper, &arg);

  /* _Unwind_Backtrace seems to put NULL address above
   * _start.  Fix it up here.
   */

  if (arg.cnt > 1 && arg.array[arg.cnt - 1] == NULL)
    {
      --arg.cnt;
    }

  return arg.cnt > 0 ? arg.cnt : 0;
}

#endif /* !CONFIG_ARCH_HAVE_BACKTRACE */
