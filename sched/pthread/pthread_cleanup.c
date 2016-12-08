/****************************************************************************
 * sched/pthread/pthread_cleanup.c
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

#include <pthread.h>

#include <nuttx/irq.h>
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
 *   The pthread_cleanup_pop_tcb() function will remove the routine at the top
 *   of the calling thread's cancellation cleanup stack and optionally
 *   invoke it (if 'execute' is non-zero).
 *
 * Input Parameters:
 *   tcb - The TCB of the pthread that is exiting or being canceled.
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

static void pthread_cleanup_pop_tcb(FAR struct pthread_tcb_s *tcb, int execute)
{
  if (tcb->tos > 0)
    {
      unsigned int ndx;

      /* Get the index to the last cleaner function pushed onto the stack */

      ndx = tcb->tos - 1;
      DEBUGASSERT(ndx < CONFIG_PTHREAD_CLEANUP_STACKSIZE);

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
 *   - The thread calls pthread_cleanup_pop() with a non-zero execute argument.
 *
 * Input Parameters:
 *   routine - The cleanup routine to be pushed on the the cleanup stack.
 *   arg     - An argument that will accompany the callback.
 *   execute - Execute the popped cleanup function immediately.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pthread_cleanup_pop(int execute)
{
  FAR struct pthread_tcb_s *tcb = (FAR struct pthread_tcb_s *)this_task();
  irqstate_t flags;

  /* We don't assert if called from a non-pthread; we just don't do anything */

  DEBUGASSERT(tcb != NULL);

  flags = enter_critical_section();
  if ((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
    {
      pthread_cleanup_pop_tcb(tcb, execute);
    }

  leave_critical_section(flags);
}

void pthread_cleanup_push(pthread_cleanup_t routine, FAR void *arg)
{
  FAR struct pthread_tcb_s *tcb = (FAR struct pthread_tcb_s *)this_task();
  irqstate_t flags;

  /* We don't assert if called from a non-pthread; we just don't do anything */

  DEBUGASSERT(tcb != NULL);
  DEBUGASSERT(tcb->tos < CONFIG_PTHREAD_CLEANUP_STACKSIZE);

  flags = enter_critical_section();
  if ((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD &&
       tcb->tos < CONFIG_PTHREAD_CLEANUP_STACKSIZE)
    {
      unsigned int ndx = tcb->tos;

      tcb->tos++;
      tcb->stack[ndx].pc_cleaner = routine;
      tcb->stack[ndx].pc_arg = arg;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: pthread_cleanup_popall
 *
 * Description:
 *   The pthread_cleanup_popall() is an internal function that will pop and
 *   execute all clean-up functions.  This function is only called from within
 *   the pthread_exit() and pthread_cancellation() logic
 *
 * Input Parameters:
 *   tcb - The TCB of the pthread that is exiting or being canceled.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pthread_cleanup_popall(FAR struct pthread_tcb_s *tcb)
{
  irqstate_t flags;

  DEBUGASSERT(tcb != NULL);
  DEBUGASSERT((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD);

  /* Pop and execute each cleanup routine */

  flags = enter_critical_section();
  while (tcb->tos > 0)
    {
      pthread_cleanup_pop_tcb(tcb, 1);
    }

  leave_critical_section(flags);
}

#endif /* CONFIG_PTHREAD_CLEANUP */
