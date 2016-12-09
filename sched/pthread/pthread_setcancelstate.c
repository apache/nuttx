/****************************************************************************
 * sched/pthread/pthread_setcancelstate.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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

#include <pthread.h>
#include <errno.h>

#include "task/task.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_setcancelstate
 *
 * Description:
 *   The pthread_setcancelstate() function atomically both sets the calling
 *   thread's cancelability state to the indicated state and returns the
 *   previous cancelability state at the location referenced by oldstate.
 *   Legal values for state are PTHREAD_CANCEL_ENABLE and
 *   PTHREAD_CANCEL_DISABLE.
 *
 *   The cancelability state and type of any newly created threads,
 *   including the thread in which main() was first invoked, are
 *   PTHREAD_CANCEL_ENABLE and PTHREAD_CANCEL_DEFERRED respectively.
 *
 ****************************************************************************/

int pthread_setcancelstate(int state, FAR int *oldstate)
{
  FAR struct tcb_s *tcb = this_task();
  int ret = OK;

  /* Suppress context changes for a bit so that the flags are stable. (the
   * flags should not change in interrupt handling).
   */

  sched_lock();

  /* Return the current state if so requrested */

  if (oldstate)
    {
      if ((tcb->flags & TCB_FLAG_NONCANCELABLE) != 0)
        {
          *oldstate = PTHREAD_CANCEL_DISABLE;
        }
      else
        {
          *oldstate = PTHREAD_CANCEL_ENABLE;
        }
    }

  /* Set the new cancellation state */

  if (state == PTHREAD_CANCEL_ENABLE)
    {
      /* Clear the non-cancelable flag */

      tcb->flags &= ~TCB_FLAG_NONCANCELABLE;

      /* Check if a cancellation was pending */

      if ((tcb->flags & TCB_FLAG_CANCEL_PENDING) != 0)
        {
#ifdef CONFIG_CANCELLATION_POINTS
          /* If we are using deferred cancellation? */

          if ((tcb->flags & TCB_FLAG_CANCEL_DEFERRED) != 0)
            {
              /* Yes.. If we are within a cancellation point, then
               * notify of the cancellation.
               */

              if (tcb->cpcount > 0)
                {
                  notify_cancellation(tcb);
                }
            }
          else
#endif
           {
             /* No.. We are using asynchronous cancellation.  If the
              * cancellation was pending in this case, then just exit.
              */

             tcb->flags &= ~TCB_FLAG_CANCEL_PENDING;
             pthread_exit(PTHREAD_CANCELED);
           }
        }
    }
  else if (state == PTHREAD_CANCEL_DISABLE)
    {
      /* Set the non-cancelable state */

      tcb->flags |= TCB_FLAG_NONCANCELABLE;
    }
  else
    {
      ret = EINVAL;
    }

  sched_unlock();
  return ret;
}
