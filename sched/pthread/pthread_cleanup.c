/****************************************************************************
 * sched/pthread/pthread_cleanup.c
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

#include <nuttx/sched.h>

#include "sched/sched.h"
#include "pthread/pthread.h"

#ifdef CONFIG_PTHREAD_CLEANUP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cleanup_pop_tcb
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

static void pthread_cleanup_pop_tcb(FAR struct tcb_s *tcb, int execute)
{
  if (tcb->tos > 0)
    {
      unsigned int ndx;

      /* Get the index to the last cleaner function pushed onto the stack */

      ndx = tcb->tos - 1;
      DEBUGASSERT(ndx >= 0 && ndx < CONFIG_PTHREAD_CLEANUP_STACKSIZE);

      /* Should we execute the cleanup routine at the top of the stack? */

      if (execute != 0)
        {
          FAR struct pthread_cleanup_s *cb;

          /* Yes..  Execute the clean-up routine.
           *
           * REVISIT: This is a security problem In the PROTECTED and KERNEL
           * builds:  We must not call the registered function in supervisor
           * mode!  See also on_exit() and atexit() callbacks.
           */

          cb  = &tcb->stack[ndx];
          cb->pc_cleaner(cb->pc_arg);
        }

      tcb->tos = ndx;
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
  FAR struct tcb_s *tcb = this_task();

  DEBUGASSERT(tcb != NULL);

  /* sched_lock() should provide sufficient protection.  We only need to
   * have this TCB stationary; the pthread cleanup stack should never be
   * modified by interrupt level logic.
   */

  sched_lock();
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      pthread_cleanup_pop_tcb(tcb, execute);
    }

  sched_unlock();
}

void pthread_cleanup_push(pthread_cleanup_t routine, FAR void *arg)
{
  FAR struct tcb_s *tcb = this_task();

  DEBUGASSERT(tcb != NULL);
  DEBUGASSERT(tcb->tos < CONFIG_PTHREAD_CLEANUP_STACKSIZE);

  /* sched_lock() should provide sufficient protection.  We only need to
   * have this TCB stationary; the pthread cleanup stack should never be
   * modified by interrupt level logic.
   */

  sched_lock();
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL &&
      tcb->tos < CONFIG_PTHREAD_CLEANUP_STACKSIZE)
    {
      unsigned int ndx = tcb->tos;

      tcb->tos++;
      tcb->stack[ndx].pc_cleaner = routine;
      tcb->stack[ndx].pc_arg = arg;
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
 *   tcb - The TCB of the pthread that is exiting or being canceled.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pthread_cleanup_popall(FAR struct tcb_s *tcb)
{
  DEBUGASSERT(tcb != NULL);

  /* Kernel threads do not support pthread APIs */

  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      /* Pop and execute each cleanup routine/
       *
       * sched_lock() should provide sufficient protection.  We only need to
       * have this TCB stationary; the pthread cleanup stack should never be
       * modified by interrupt level logic.
       */

      sched_lock();
      while (tcb->tos > 0)
        {
          pthread_cleanup_pop_tcb(tcb, 1);
        }

      sched_unlock();
    }
}

#endif /* CONFIG_PTHREAD_CLEANUP */
