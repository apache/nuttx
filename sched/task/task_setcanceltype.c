/****************************************************************************
 * sched/task/task_setcanceltype.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
              exit(EXIT_FAILURE);
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
