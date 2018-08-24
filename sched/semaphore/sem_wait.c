/****************************************************************************
 * sched/semaphore/sem_wait.c
 *
 *   Copyright (C) 2007-2013, 2016 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_wait
 *
 * Description:
 *   This function attempts to lock the semaphore referenced by 'sem'.  If
 *   the semaphore value is (<=) zero, then the calling task will not return
 *   until it successfully acquires the lock.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellaction point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of a signal.
 *
 ****************************************************************************/

int nxsem_wait(FAR sem_t *sem)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  int ret = -EINVAL;

  /* This API should not be called from interrupt handlers */

  DEBUGASSERT(sem != NULL && up_interrupt_context() == false);

  /* The following operations must be performed with interrupts
   * disabled because nxsem_post() may be called from an interrupt
   * handler.
   */

  flags = enter_critical_section();

  /* Make sure we were supplied with a valid semaphore. */

  if (sem != NULL)
    {
      /* Check if the lock is available */

      if (sem->semcount > 0)
        {
          /* It is, let the task take the semaphore. */

          sem->semcount--;
          nxsem_addholder(sem);
          rtcb->waitsem = NULL;
          ret = OK;
        }

      /* The semaphore is NOT available, We will have to block the
       * current thread of execution.
       */

      else
        {
          int saved_errno;

          /* First, verify that the task is not already waiting on a
           * semaphore
           */

          DEBUGASSERT(rtcb->waitsem == NULL);

          /* Handle the POSIX semaphore (but don't set the owner yet) */

          sem->semcount--;

          /* Save the waited on semaphore in the TCB */

          rtcb->waitsem = sem;

          /* If priority inheritance is enabled, then check the priority of
           * the holder of the semaphore.
           */

#ifdef CONFIG_PRIORITY_INHERITANCE
          /* Disable context switching.  The following operations must be
           * atomic with regard to the scheduler.
           */

          sched_lock();

          /* Boost the priority of any threads holding a count on the
           * semaphore.
           */

          nxsem_boostpriority(sem);
#endif
          /* Set the errno value to zero (preserving the original errno)
           * value).  We reuse the per-thread errno to pass information
           * between sem_waitirq() and this functions.
           */

          saved_errno   = rtcb->pterrno;
          rtcb->pterrno = OK;

          /* Add the TCB to the prioritized semaphore wait queue */

          up_block_task(rtcb, TSTATE_WAIT_SEM);

          /* When we resume at this point, either (1) the semaphore has been
           * assigned to this thread of execution, or (2) the semaphore wait
           * has been interrupted by a signal or a timeout.  We can detect these
           * latter cases be examining the per-thread errno value.
           *
           * In the event that the semaphore wait was interrupted by a signal or
           * a timeout, certain semaphore clean-up operations have already been
           * performed (see sem_waitirq.c).  Specifically:
           *
           * - nxsem_canceled() was called to restore the priority of all
           *   threads that hold a reference to the semaphore,
           * - The semaphore count was decremented, and
           * - tcb->waitsem was nullifed.
           *
           * It is necesaary to do these things in sem_waitirq.c because a long
           * time may elapse between the time that the signal was issued and
           * this thread is awakened and this leaves a door open to several
           * race conditions.
           */

          /* Check if an error occurred while we were sleeping.  Expected
           * errors include EINTR meaning that we were awakened by a signal
           * or ETIMEDOUT meaning that the timer expired for the case of
           * sem_timedwait().
           *
           * If we were not awakened by a signal or a timeout, then
           * nxsem_addholder() was called by logic in sem_wait() fore this
           * thread was restarted.
           */

          ret           = rtcb->pterrno != OK ? -rtcb->pterrno : OK;
          rtcb->pterrno = saved_errno;

#ifdef CONFIG_PRIORITY_INHERITANCE
          sched_unlock();
#endif
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sem_wait
 *
 * Description:
 *   This function attempts to lock the semaphore referenced by 'sem'.  If
 *   the semaphore value is (<=) zero, then the calling task will not return
 *   until it successfully acquires the lock.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This function is a standard, POSIX application interface.  It returns
 *   zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.  Possible errno values include:
 *
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of a signal.
 *
 ****************************************************************************/

int sem_wait(FAR sem_t *sem)
{
  int errcode;
  int ret;

  /* sem_wait() is a cancellation point */

  if (enter_cancellation_point())
    {
#ifdef CONFIG_CANCELLATION_POINTS
      /* If there is a pending cancellation, then do not perform
       * the wait.  Exit now with ECANCELED.
       */

      errcode = ECANCELED;
      goto errout_with_cancelpt;
#endif
    }

  /* Let nxsem_wait() do the real work */

  ret = nxsem_wait(sem);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_cancelpt;
    }

  leave_cancellation_point();
  return OK;

errout_with_cancelpt:
  set_errno(errcode);
  leave_cancellation_point();
  return ERROR;
}
