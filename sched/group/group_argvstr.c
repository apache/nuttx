/****************************************************************************
 * sched/group/group_argvstr.c
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

#include <sched.h>
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

#include <nuttx/addrenv.h>
#include <nuttx/tls.h>

#include "sched/sched.h"
#include "group/group.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_argvstr
 *
 * Description:
 *   Safely read the contents of a task's argument vector, into a a safe
 *   buffer. Function skips the process's name.
 *
 * Input Parameters:
 *   tcb  - tcb of the task.
 *   args - Output buffer for the argument vector.
 *   size - Size of the buffer.
 *
 * Returned Value:
 *   The actual string length that was written.
 *
 ****************************************************************************/

size_t group_argvstr(FAR struct tcb_s *tcb, FAR char *args, size_t size)
{
  size_t n = 0;
#ifdef CONFIG_ARCH_ADDRENV
  bool saved = false;
#endif

  /* Perform sanity checks */

  if (!tcb || !tcb->group || !tcb->group->tg_info)
    {
      /* Something is very wrong -> get out */

      *args = '\0';
      return 0;
    }

#ifdef CONFIG_ARCH_ADDRENV
  if (tcb->addrenv_own != NULL)
    {
      addrenv_select(tcb->addrenv_own);
      saved = true;
    }
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
    {
      FAR struct pthread_tcb_s *ptcb = (FAR struct pthread_tcb_s *)tcb;

      n += snprintf(args, size, " %p %p", ptcb->cmn.entry.main, ptcb->arg);
    }
  else
#endif
    {
      FAR char **argv = tcb->group->tg_info->argv + 1;

      while (*argv != NULL && n < size)
        {
          n += snprintf(args + n, size - n, " %s", *argv++);
        }
    }

#ifdef CONFIG_ARCH_ADDRENV
  if (saved)
    {
      addrenv_restore();
    }
#endif

  return n < size - 1 ? n : size - 1;
}
