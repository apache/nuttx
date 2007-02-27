/************************************************************
 * pthread_completejoin.c
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
 * Function:  pthread_destroyjoininfo
 *
 * Description:
 *   Destroy a join_t structure.  This must
 *   be done by the child thread at child thread destruction
 *   time.
 *
 ************************************************************/

static void pthread_destroyjoininfo(FAR join_t *pjoin)
{
  int ntasks_waiting;
  int status;

  dbg("pjoin=0x%p\n", pjoin);

  /* Are any tasks waiting for our exit value? */

  status = sem_getvalue(&pjoin->exit_sem, &ntasks_waiting);
  if (status == OK && ntasks_waiting < 0)
    {
      /* Set the data semaphore so that this thread will be
       * awakened when all waiting tasks receive the data
       */

      (void)sem_init(&pjoin->data_sem, 0, (ntasks_waiting+1));

      /* Post the semaphore to restart each thread that is waiting
       * on the semaphore
       */

      do
        {
          status = pthread_givesemaphore(&pjoin->exit_sem);
          if (status == OK)
            {
              status = sem_getvalue(&pjoin->exit_sem, &ntasks_waiting);
            }
        }
      while (ntasks_waiting < 0 && status == OK);

      /* Now wait for all these restarted tasks to obtain the return
       * value.
       */

      (void)pthread_takesemaphore(&pjoin->data_sem);
      (void)sem_destroy(&pjoin->data_sem);
    }

  /* All of the joined threads have had received the exit value.
   * Now we can destroy this thread's exit semaphore
   */

  (void)sem_destroy(&pjoin->exit_sem);
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  pthread_completejoin
 *
 * Description:
 *   A thread has been terminated -- either by returning,
 *   calling pthread_exit(), or through pthread_cancel().
 *   In any event, we must complete any pending join events.
 *
 * Parameters:
 *   exit_value
 *
 * Returned Value:
 *   OK unless there is no join information associated with
 *   the pid.  This could happen, for example, if a task
 *   started with task_create() calls pthread_exit().
 *
 * Assumptions:
 *
 ************************************************************/

int pthread_completejoin(pid_t pid, FAR void *exit_value)
{
  FAR join_t *pjoin;
  boolean detached = FALSE;

  dbg("process_id=%d exit_value=%p\n", pid, exit_value);

  /* First, find thread's structure in the private data set. */

  (void)pthread_takesemaphore(&g_join_semaphore);
  pjoin = pthread_findjoininfo(pid);
  if (!pjoin)
    {

      (void)pthread_givesemaphore(&g_join_semaphore);
      return ERROR;
    }
  else
    {
      /* Has the thread been marked as detached? */

      pjoin->terminated = TRUE;
      detached = pjoin->detached;
      if (detached)
        {
          dbg("Detaching\n");

          /* If so, then remove the thread's structure from the private
           * data set. After this point, no other thread can perform a join
           * operation.
           */

          (void)pthread_removejoininfo(pid);
          (void)pthread_givesemaphore(&g_join_semaphore);

          /* Destroy this thread data structure. */

          pthread_destroyjoininfo(pjoin);

          /* Deallocate the join entry if it was detached. */

          sched_free((FAR void*)pjoin);
        }

     /* No, then we can assume that some other thread is waiting for the join info */

      else
        {
          /* Save the return exit value in the thread structure. */

          pjoin->exit_value = exit_value;

          /* Destroy this thread data structure. */

          pthread_destroyjoininfo(pjoin);

          /* pthread_join may now access the thread entry structure. */

          (void)pthread_givesemaphore(&g_join_semaphore);
        }
    }

  return OK;
}
