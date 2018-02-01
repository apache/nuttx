/****************************************************************************
 * sched/task/task_cancelpt.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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
 * Cancellation Points.
 *
 * Cancellation points shall occur when a thread is executing the following
 * functions:
 *
 * accept()          mq_timedsend()           putpmsg()       sigtimedwait()
 * aio_suspend()     msgrcv()                 pwrite()        sigwait()
 * clock_nanosleep() msgsnd()                 read()          sigwaitinfo()
 * close()           msync()                  readv()         sleep()
 * connect()         nanosleep()              recv()          system()
 * creat()           open()                   recvfrom()      tcdrain()
 * fcntl()           pause()                  recvmsg()       usleep()
 * fdatasync()       poll()                   select()        wait()
 * fsync()           pread()                  sem_timedwait() waitid()
 * getmsg()          pselect()                sem_wait()      waitpid()
 * getpmsg()         pthread_cond_timedwait() send()          write()
 * lockf()           pthread_cond_wait()      sendmsg()       writev()
 * mq_receive()      pthread_join()           sendto()
 * mq_send()         pthread_testcancel()     sigpause()
 * mq_timedreceive() putmsg()                 sigsuspend()
 *
 * Each of the above function must call enter_cancellation_point() on entry
 * in order to establish the cancellation point and leave_cancellation_point()
 * on exit.  These functions are described below.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"
#include "signal/signal.h"
#include "mqueue/mqueue.h"
#include "task/task.h"

#ifdef CONFIG_CANCELLATION_POINTS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enter_cancellation_point
 *
 * Description:
 *   Called at the beginning of the cancellation point to establish the
 *   cancellation point.  This function does the following:
 *
 *   1. If deferred cancellation does not apply to this thread, nothing is
 *      done, otherwise, it
 *   2. Sets state information in the caller's TCB and increments a nesting
 *      count.
 *   3. If this is the outermost nesting level, it checks if there is a
 *      pending cancellation and, if so, calls either exit() or
 *      pthread_exit(), depending upon the type of the thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true is returned if a cancellation is pending but cannot be performed
 *   now due to the nesting level.
 *
 ****************************************************************************/

bool enter_cancellation_point(void)
{
  FAR struct tcb_s *tcb = this_task();
  bool ret = false;

  /* Disabling pre-emption should provide sufficient protection.  We only
   * need the TCB to be stationary (no interrupt level modification is
   * anticipated).
   *
   * REVISIT: is locking the scheduler sufficent in SMP mode?
   */

  sched_lock();

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then do nothing.
   *
   * Special case: if the cpcount count is greater than zero, then we are
   * nested and the above condition was certainly true at the outermost
   * nesting level.
   */

  if (((tcb->flags & TCB_FLAG_NONCANCELABLE) == 0 &&
       (tcb->flags & TCB_FLAG_CANCEL_DEFERRED) != 0) ||
      tcb->cpcount > 0)
    {
      /* Check if there is a pending cancellation */

      if ((tcb->flags & TCB_FLAG_CANCEL_PENDING) != 0)
        {
          /* Yes... return true (if we don't exit here) */

          ret = true;

          /* If there is a pending cancellation and we are at the outermost
           * nesting level of cancellation function calls, then exit
           * according to the type of the thread.
           */

          if (tcb->cpcount == 0)
            {
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
        }

      /* Otherwise, indicate that we are at a cancellation point by
       * incrementing the nesting level of the cancellation point
       * functions.
       */

      DEBUGASSERT(tcb->cpcount < INT16_MAX);
      tcb->cpcount++;
    }

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: leave_cancellation_point
 *
 * Description:
 *   Called at the end of the cancellation point.  This function does the
 *   following:
 *
 *   1. If deferred cancellation does not apply to this thread, nothing is
 *      done, otherwise, it
 *   2. Clears state information in the caller's TCB and decrements a
 *      nesting count.
 *   3. If this is the outermost nesting level, it checks if there is a
 *      pending cancellation and, if so, calls either exit() or
 *      pthread_exit(), depending upon the type of the thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void leave_cancellation_point(void)
{
  FAR struct tcb_s *tcb = this_task();

  /* Disabling pre-emption should provide sufficient protection.  We only
   * need the TCB to be stationary (no interrupt level modification is
   * anticipated).
   *
   * REVISIT: is locking the scheduler sufficent in SMP mode?
   */

  sched_lock();

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then do nothing.  Here we check only the
   * nesting level: if the cpcount count is greater than zero, then the
   * required condition was certainly true at the outermost nesting level.
   */

  if (tcb->cpcount > 0)
    {
      /* Decrement the nesting level.  If if would decrement to zero, then
       * we are at the outermost nesting level and may need to do more.
       */

      if (tcb->cpcount == 1)
        {
          /* We are no longer at the cancellation point */

          tcb->cpcount = 0;

          /* If there is a pending cancellation then just exit according to
           * the type of the thread.
           */

          if ((tcb->flags & TCB_FLAG_CANCEL_PENDING) != 0)
            {
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
        }
      else
        {
          /* We are not at the outermost nesting level.  Just decrment the
           * nesting level count.
           */

          tcb->cpcount--;
        }
    }

  sched_unlock();
}

/****************************************************************************
 * Name: check_cancellation_point
 *
 * Description:
 *   Returns true if:
 *
 *   1. Deferred cancellation does applies to this thread,
 *   2. We are within a cancellation point (i.e., the nesting level in the
 *      TCB is greater than zero).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true is returned if a cancellation is pending but cannot be performed
 *   now due to the nesting level.
 *
 ****************************************************************************/

bool check_cancellation_point(void)
{
  FAR struct tcb_s *tcb = this_task();
  bool ret = false;

  /* Disabling pre-emption should provide sufficient protection.  We only
   * need the TCB to be stationary (no interrupt level modification is
   * anticipated).
   *
   * REVISIT: is locking the scheduler sufficent in SMP mode?
   */

  sched_lock();

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then return false.
   *
   * If the cpcount count is greater than zero, then we within a
   * cancellation and will true if there is a pending cancellation.
   */

  if (((tcb->flags & TCB_FLAG_NONCANCELABLE) == 0 &&
       (tcb->flags & TCB_FLAG_CANCEL_DEFERRED) != 0) ||
      tcb->cpcount > 0)
    {
      /* Check if there is a pending cancellation.  If so, return true. */

      ret = ((tcb->flags & TCB_FLAG_CANCEL_PENDING) != 0);
    }

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: notify_cancellation
 *
 * Description:
 *   Called by task_delete() or pthread_cancel() if the cancellation occurs
 *   while we the thread is within the cancellation point.  This logic
 *   behaves much like sending a signal:  It will cause waiting threads
 *   to wake up and terminated with ECANCELED.  A call to
 *   leave_cancellation_point() whould then follow, causing the thread to
 *   exit.
 *
 ****************************************************************************/

void notify_cancellation(FAR struct tcb_s *tcb)
{
  irqstate_t flags;

  /* We need perform the following operations from within a critical section
   * because it can compete with interrupt level activity.
   */

  flags = enter_critical_section();

  /* Make sure that the cancellation pending indication is set. */

  tcb->flags |= TCB_FLAG_CANCEL_PENDING;

  /* We only notify the cancellation if (1) the thread has not disabled
   * cancellation, (2) the thread uses the deffered cancellation mode,
   * (3) the thread is waiting within a cancellation point.
   */

  if (((tcb->flags & TCB_FLAG_NONCANCELABLE) == 0 &&
       (tcb->flags & TCB_FLAG_CANCEL_DEFERRED) != 0) ||
      tcb->cpcount > 0)
    {
      /* If the thread is blocked waiting for a semaphore, then the thread
       * must be unblocked to handle the cancellation.
       */

      if (tcb->task_state == TSTATE_WAIT_SEM)
        {
          nxsem_wait_irq(tcb, ECANCELED);
        }

#ifndef CONFIG_DISABLE_SIGNALS
      /* If the thread is blocked waiting on a signal, then the
       * thread must be unblocked to handle the cancellation.
       */

      else if (tcb->task_state == TSTATE_WAIT_SIG)
        {
          nxsig_wait_irq(tcb, ECANCELED);
        }
#endif

#ifndef CONFIG_DISABLE_MQUEUE
      /* If the thread is blocked waiting on a message queue, then the
       * thread must be unblocked to handle the cancellation.
       */

      else if (tcb->task_state == TSTATE_WAIT_MQNOTEMPTY ||
               tcb->task_state == TSTATE_WAIT_MQNOTFULL)
        {
          nxmq_wait_irq(tcb, ECANCELED);
        }
#endif
    }

  leave_critical_section(flags);
}

#endif /* CONFIG_CANCELLATION_POINTS */
