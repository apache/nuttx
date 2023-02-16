/****************************************************************************
 * sched/task/task_setcanceltype.c
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
 * Name: task_setcanceltype
 *
 * Description:
 *   The task_setcanceltype() function atomically both sets the calling
 *   thread's cancellability type to the indicated type and returns the
 *   previous cancellability type at the location referenced by oldtype
 *   Legal values for type are TASK_CANCEL_DEFERRED and
 *   TASK_CANCEL_ASYNCHRONOUS.
 *
 *   The cancellability state and type of any newly created threads,
 *   including the thread in which main() was first invoked, are
 *   TASK_CANCEL_ENABLE and TASK_CANCEL_DEFERRED respectively.
 *
 ****************************************************************************/

int task_setcanceltype(int type, FAR int *oldtype)
{
  FAR struct tcb_s *tcb = this_task();
  int ret = OK;

  /* Suppress context changes for a bit so that the flags are stable. (the
   * flags should not change in interrupt handling).
   */

  sched_lock();

  /* Return the current type if so requested */

  if (oldtype != NULL)
    {
      if ((tcb->flags & TCB_FLAG_CANCEL_DEFERRED) != 0)
        {
          *oldtype = TASK_CANCEL_DEFERRED;
        }
      else
        {
          *oldtype = TASK_CANCEL_ASYNCHRONOUS;
        }
    }

  /* Set the new cancellation type */

  if (type == TASK_CANCEL_ASYNCHRONOUS)
    {
      /* Clear the deferred cancellation bit */

      tcb->flags &= ~TCB_FLAG_CANCEL_DEFERRED;

#ifdef CONFIG_CANCELLATION_POINTS
      /* If we just switched from deferred to asynchronous type and if a
       * cancellation is pending, then exit now.
       */

      if ((tcb->flags & TCB_FLAG_CANCEL_PENDING) != 0 &&
          (tcb->flags & TCB_FLAG_NONCANCELABLE) == 0)
        {
          tcb->flags &= ~TCB_FLAG_CANCEL_PENDING;

          /* Exit according to the type of the thread */

#ifndef CONFIG_DISABLE_PTHREAD
          if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
            {
              pthread_exit(PTHREAD_CANCELED);
            }
          else
#endif
            {
              _exit(EXIT_FAILURE);
            }
        }
#endif
    }
#ifdef CONFIG_CANCELLATION_POINTS
  else if (type == TASK_CANCEL_DEFERRED)
    {
      /* Set the deferred cancellation type */

      tcb->flags |= TCB_FLAG_CANCEL_DEFERRED;
    }
#endif
  else
    {
      ret = EINVAL;
    }

  sched_unlock();
  return ret;
}
