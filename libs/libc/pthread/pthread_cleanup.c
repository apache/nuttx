/****************************************************************************
 * libs/libc/pthread/pthread_cleanup.c
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

#include <pthread.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/tls.h>
#include <nuttx/pthread.h>

#if defined(CONFIG_PTHREAD_CLEANUP_STACKSIZE) && CONFIG_PTHREAD_CLEANUP_STACKSIZE > 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cleanup_pop_tls
 *
 * Description:
 *   The pthread_cleanup_pop_tcb() function will remove the routine at the
 *   top of the calling thread's cancellation cleanup stack and optionally
 *   invoke it (if 'execute' is non-zero).
 *
 * Input Parameters:
 *   tcb - The TCB of the pthread that is exiting or being canceled.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The scheduler is locked.
 *
 ****************************************************************************/

static void pthread_cleanup_pop_tls(FAR struct tls_info_s *tls, int execute)
{
  if (tls->tos > 0)
    {
      unsigned int ndx;

      /* Get the index to the last cleaner function pushed onto the stack */

      ndx = tls->tos - 1;
      DEBUGASSERT(ndx >= 0 && ndx < CONFIG_PTHREAD_CLEANUP_STACKSIZE);

      /* Should we execute the cleanup routine at the top of the stack? */

      if (execute != 0)
        {
          FAR struct pthread_cleanup_s *cb;

          /* Yes..  Execute the clean-up routine. */

          cb  = &tls->stack[ndx];
          cb->pc_cleaner(cb->pc_arg);
        }

      tls->tos = ndx;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cleanup_push
 *       pthread_cleanup_pop
 *
 * Description:
 *   The pthread_cleanup_pop() function will remove the routine at the top
 *   of the calling thread's cancellation cleanup stack and optionally
 *   invoke it (if 'execute' is non-zero).
 *
 *   The pthread_cleanup_push() function will push the specified cancellation
 *   cleanup handler routine onto the calling thread's cancellation cleanup
 *   stack. The cancellation cleanup handler will be popped from the
 *   cancellation cleanup stack and invoked with the argument arg when:
 *
 *   - The thread exits (that is, calls pthread_exit()).
 *   - The thread acts upon a cancellation request.
 *   - The thread calls pthread_cleanup_pop() with non-zero execute argument.
 *
 * Input Parameters:
 *   routine - The cleanup routine to be pushed on the cleanup stack.
 *   arg     - An argument that will accompany the callback.
 *   execute - Execute the popped cleanup function immediately.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pthread_cleanup_pop(int execute)
{
  FAR struct tls_info_s *tls = tls_get_info();

  DEBUGASSERT(tls != NULL);

  /* sched_lock() should provide sufficient protection.  We only need to
   * have this TCB stationary; the pthread cleanup stack should never be
   * modified by interrupt level logic.
   */

  sched_lock();
  pthread_cleanup_pop_tls(tls, execute);
  sched_unlock();
}

void pthread_cleanup_push(pthread_cleanup_t routine, FAR void *arg)
{
  FAR struct tls_info_s *tls = tls_get_info();

  DEBUGASSERT(tls != NULL);
  DEBUGASSERT(tls->tos < CONFIG_PTHREAD_CLEANUP_STACKSIZE);

  /* sched_lock() should provide sufficient protection.  We only need to
   * have this TCB stationary; the pthread cleanup stack should never be
   * modified by interrupt level logic.
   */

  sched_lock();
  if (tls->tos < CONFIG_PTHREAD_CLEANUP_STACKSIZE)
    {
      unsigned int ndx = tls->tos;

      tls->tos++;
      tls->stack[ndx].pc_cleaner = routine;
      tls->stack[ndx].pc_arg = arg;
    }

  sched_unlock();
}

/****************************************************************************
 * Name: pthread_cleanup_popall
 *
 * Description:
 *   The pthread_cleanup_popall() is an internal function that will pop and
 *   execute all clean-up functions.  This function is only called from
 *   within the pthread_exit() and pthread_cancellation() logic
 *
 * Input Parameters:
 *   tls - The local storage info of the exiting thread
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pthread_cleanup_popall(FAR struct tls_info_s *tls)
{
  DEBUGASSERT(tls != NULL);

  sched_lock();
  while (tls->tos > 0)
    {
      pthread_cleanup_pop_tls(tls, 1);
    }

  sched_unlock();
}

#endif /* defined(CONFIG_PTHREAD_CLEANUP_STACKSIZE) && CONFIG_PTHREAD_CLEANUP_STACKSIZE > 0 */
