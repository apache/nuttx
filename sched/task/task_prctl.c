/****************************************************************************
 * sched/task/task_prctl.c
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

#include <sys/prctl.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/sched.h>

#include "sched/sched.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: prctl
 *
 * Description:
 *   prctl() is called with a first argument describing what to do (with
 *   values PR_* defined above) and with additional arguments depending on
 *   the specific command.
 *
 * Returned Value:
 *   The returned value may depend on the specific command.  For PR_SET_NAME
 *   and PR_GET_NAME, the returned value of 0 indicates successful operation.
 *   On any failure, -1 is retruend and the errno value is set appropriately.
 *
 *     EINVAL The value of 'option' is not recognized.
 *     EFAULT optional arg1 is not a valid address.
 *     ESRCH  No task/thread can be found corresponding to that specified
 *       by optional arg1.
 *
 ****************************************************************************/

int prctl(int option, ...)
{
  va_list ap;
  int errcode;

  va_start(ap, option);
  switch (option)
    {
      case PR_SET_NAME:
      case PR_GET_NAME:
      case PR_SET_NAME_EXT:
      case PR_GET_NAME_EXT:
#if CONFIG_TASK_NAME_SIZE > 0
        {
          /* Get the prctl arguments */

          FAR char *name = va_arg(ap, FAR char *);
          FAR struct tcb_s *tcb;
          int pid = 0;

          if (option == PR_SET_NAME_EXT ||
              option == PR_GET_NAME_EXT)
            {
              pid = va_arg(ap, int);
            }

          /* Get the TCB associated with the PID (handling the special case
           * of pid==0 meaning "this thread")
           */

          if (pid == 0)
            {
              tcb = this_task();
            }
          else
            {
              tcb = nxsched_get_tcb(pid);
            }

          /* An invalid pid will be indicated by a NULL TCB returned from
           * nxsched_get_tcb()
           */

          if (tcb == NULL)
            {
              serr("ERROR: Pid does not correspond to a task: %d\n", pid);
              errcode = ESRCH;
              goto errout;
            }

          /* A pointer to the task name storage must also be provided */

          if (name == NULL)
            {
              serr("ERROR: No name provide\n");
              errcode = EFAULT;
              goto errout;
            }

          /* Now get or set the task name */

          if (option == PR_SET_NAME || option == PR_SET_NAME_EXT)
            {
              /* Ensure that tcb->name will be null-terminated, truncating if
               * necessary.
               */

              strlcpy(tcb->name, name, sizeof(tcb->name));
              tcb->name[CONFIG_TASK_NAME_SIZE] = '\0';
            }
          else
            {
              /* The returned value will be null-terminated, truncating if
               * necessary.
               */

              strlcpy(name, tcb->name, sizeof(tcb->name));
              name[CONFIG_TASK_NAME_SIZE - 1] = '\0';
            }
        }
        break;
#else
        serr("ERROR: Option not enabled: %d\n", option);
        errcode = ENOSYS;
        goto errout;
#endif

      case PR_SET_DUMPABLE:
      case PR_GET_DUMPABLE:
#ifdef CONFIG_SCHED_USER_IDENTITY
        {
          FAR struct tcb_s *tcb = this_task();

          if (tcb == NULL || tcb->group == NULL)
            {
              errcode = ESRCH;
              goto errout;
            }

          if (option == PR_GET_DUMPABLE)
            {
              va_end(ap);
              return (tcb->group->tg_flags & GROUP_FLAG_DUMPABLE) != 0;
            }

          if (va_arg(ap, int) != 0)
            {
              if (tcb->group->tg_euid != 0)
                {
                  errcode = EPERM;
                  goto errout;
                }

              tcb->group->tg_flags |= GROUP_FLAG_DUMPABLE;
            }
          else
            {
              tcb->group->tg_flags &= ~GROUP_FLAG_DUMPABLE;
            }
        }
        break;
#else
        serr("ERROR: Option not enabled: %d\n", option);
        errcode = ENOSYS;
        goto errout;
#endif

      default:
        serr("ERROR: Unrecognized option: %d\n", option);
        errcode = EINVAL;
        goto errout;
    }

  /* Not reachable unless CONFIG_TASK_NAME_SIZE is > 0.  NOTE: This might
   * change if additional commands are supported.
   */

#if CONFIG_TASK_NAME_SIZE > 0
  va_end(ap);
  return OK;
#endif

errout:
  va_end(ap);
  set_errno(errcode);
  return ERROR;
}
