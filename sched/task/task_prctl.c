/****************************************************************************
 * sched/task/task_prctl.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <debug.h>

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

              strncpy(tcb->name, name, CONFIG_TASK_NAME_SIZE);
              tcb->name[CONFIG_TASK_NAME_SIZE] = '\0';
            }
          else
            {
              /* The returned value will be null-terminated, truncating if
               * necessary.
               */

              strncpy(name, tcb->name, CONFIG_TASK_NAME_SIZE - 1);
              name[CONFIG_TASK_NAME_SIZE - 1] = '\0';
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
