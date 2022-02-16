/****************************************************************************
 * sched/task/task_setcancelstate.c
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

#include <stdlib.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>

#include "sched/sched.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_setcancelstate
 *
 * Description:
 *   The task_setcancelstate() function atomically both sets the calling
 *   task's cancellability state to the indicated state and returns the
 *   previous cancellability state at the location referenced by oldstate.
 *   Legal values for state are TASK_CANCEL_ENABLE and TASK_CANCEL_DISABLE.
 *
 *   The cancellability state and type of any newly created tasks are
 *   TASK_CANCEL_ENABLE and TASK_CANCEL_DEFERRED respectively.
 *
 * Input Parameters:
 *   state    - the new cancellability state, either TASK_CANCEL_ENABLE or
 *              TASK_CANCEL_DISABLE
 *   oldstate - The location to return the old cancellability state.
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR is returned on any failure with the
 *   errno value set appropriately.
 *
 ****************************************************************************/

int task_setcancelstate(int state, FAR int *oldstate)
{
  FAR struct tcb_s *tcb = this_task();
  int ret = OK;

  /* Suppress context changes for a bit so that the flags are stable. (the
   * flags should not change in interrupt handling).
   */

  sched_lock();

  /* Return the current state if so requested */

  if (oldstate != NULL)
    {
      if ((tcb->flags & TCB_FLAG_NONCANCELABLE) != 0)
        {
          *oldstate = TASK_CANCEL_DISABLE;
        }
      else
        {
          *oldstate = TASK_CANCEL_ENABLE;
        }
    }

  /* Set the new cancellation state */

  if (state == TASK_CANCEL_ENABLE)
    {
      /* Clear the non-cancelable flag */

      tcb->flags &= ~TCB_FLAG_NONCANCELABLE;

      /* Check if a cancellation was pending */

      if ((tcb->flags & TCB_FLAG_CANCEL_PENDING) != 0)
        {
#ifdef CONFIG_CANCELLATION_POINTS
          /* If we are using deferred cancellation? */

          if ((tcb->flags & TCB_FLAG_CANCEL_DEFERRED) == 0)
#endif
            {
              /* No.. We are using asynchronous cancellation.  If the
               * cancellation was pending in this case, then just exit.
               */

              tcb->flags &= ~TCB_FLAG_CANCEL_PENDING;

#ifndef CONFIG_DISABLE_PTHREAD
              if ((tcb->flags & TCB_FLAG_TTYPE_MASK) ==
                  TCB_FLAG_TTYPE_PTHREAD)
                {
                  pthread_exit(PTHREAD_CANCELED);
                }
              else
#endif
                {
                  exit(EXIT_FAILURE);
                }
            }
        }
    }
  else if (state == TASK_CANCEL_DISABLE)
    {
      /* Set the non-cancelable state */

      tcb->flags |= TCB_FLAG_NONCANCELABLE;
    }
  else
    {
      set_errno(EINVAL);
      ret = ERROR;
    }

  sched_unlock();
  return ret;
}
