/************************************************************
 * pthread_join.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>
#include "os_internal.h"
#include "pthread_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  pthread_join
 *
 * Description:
 *    A thread can await termination of another thread and
 *    retrieve the return value of the thread.
 *
 * Parameters:
 *   thread
 *   pexit_value
 *
 * Return Value:
 *   0 if successful.  Otherwise, one of the following error codes:
 *       EINVAL  The value specified by thread does not refer to a
 *               joinable thread.
 *       ESRCH   No thread could be found corresponding to that
 *               specified by the given thread ID.
 *       EDEADLK A deadlock was detected or the value of thread
 *               specifies the calling thread.
 *
 * Assumptions:
 *
 ************************************************************/

int pthread_join(pthread_t thread, pthread_addr_t *pexit_value)
{
  join_t *pjoin;
  int ret;

  dbg("%s: thread=%d\n", __FUNCTION__, thread.pid);

  /* First make sure that this is not an attempt to join to
   * ourself.
   */

  if (thread.pid == getpid())
    {
      return EDEADLK;
    }

  /* Make sure no other task is mucking with the data structures
   * while we are performing the following operations.  NOTE:
   * we can be also sure that pthread_exit() will not execute
   * because it will also attempt to get this semaphore.
   */

  (void)pthread_takesemaphore(&g_join_semaphore);

  /* Find the join information associated with this thread.
   * This can fail for one of three reasons:  (1) There is no
   * thread associated with 'thread,' (2) the thread is a task
   * and does not have join information, or (3) the thread
   * was detached and has exitted.
   */

  pjoin = pthread_findjoininfo(thread.pid);
  if (!pjoin)
    {
      /* Determine what kind of error to return */

      _TCB *tcb = sched_gettcb(thread.pid);

      dbg("%s: Could not find thread data\n", __FUNCTION__);

      /* Case (1) or (3) -- we can't tell which.  Assume (3) */

      if (!tcb)
        {
          ret = ESRCH;
        }

      /* The thread is still active but has no join info.  In that
       * case, it must be a task and not a pthread.
       */

      else /* if ((tcb->flags & EDEADLK) == 0) */
        {
          ret = EINVAL;
        }

      (void)pthread_givesemaphore(&g_join_semaphore);
    }
  else if (pjoin->terminated)
    {
      dbg("%s: Thread has terminated\n", __FUNCTION__);

      /* Get the thread exit value from the terminated thread. */

      if (pexit_value)
        {
          dbg("%s: exit_value=0x%p\n", __FUNCTION__, pjoin->exit_value);
          *pexit_value = pjoin->exit_value;
        }

      /* Then remove and deallocate the thread entry. */

      (void)pthread_removejoininfo(thread.pid);
      (void)pthread_givesemaphore(&g_join_semaphore);
      sched_free(pjoin);
      ret = OK;
    }
  else
    {
      dbg("%s: Thread is still running\n", __FUNCTION__);

      /* Relinquish the data set semaphore, making certain that
       * no task has the opportunity to run between the time
       * we relinquish the data set semaphore and the time that
       * we wait on the join semaphore.
       */

      sched_lock();
      (void)pthread_givesemaphore(&g_join_semaphore);

      /* Take the thread's join semaphore */

      (void)pthread_takesemaphore(&pjoin->exit_sem);

      /* Get the thread exit value */

      if (pexit_value)
        {
          *pexit_value = pjoin->exit_value;
          dbg("%s: exit_value=0x%p\n", __FUNCTION__, pjoin->exit_value);
        }

      /* Post the thread's join semaphore so that exitting thread
       * will know that we have received the data.
       */

      (void)pthread_givesemaphore(&pjoin->data_sem);

      /* Pre-emption is okay now. */

      sched_unlock();

      ret = OK;
    }

  dbg("%s: Returning %d\n", __FUNCTION__, ret);
  return ret;
}
