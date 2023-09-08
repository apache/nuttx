/****************************************************************************
 * libs/libc/signal/sig_altstack.c
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

#include <signal.h>
#include <errno.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigaltstack
 *
 * Description:
 * The sigaltstack() function allows a process to define and examine the
 * state of an alternate stack for signal handlers for the current thread.
 * Signals that have been explicitly declared to execute on the alternate
 * stack shall be delivered on the alternate stack.
 *
 ****************************************************************************/

int sigaltstack(FAR const stack_t *ss, FAR stack_t *oss)
{
  if (ss)
    {
      if (ss->ss_flags & SS_DISABLE)
        {
          goto out;
        }

      if (ss->ss_size < MINSIGSTKSZ)
        {
          set_errno(ENOMEM);
          return ERROR;
        }

      /* not support SS_ONSTACK now */

      set_errno(EINVAL);
      return ERROR;
    }

out:
  if (oss)
    {
      memset(oss, 0, sizeof(*oss));
      oss->ss_flags = SS_DISABLE;
    }

  return OK;
}
