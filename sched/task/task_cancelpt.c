/****************************************************************************
 * sched/task/task_cancelpt.c
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
 * in order to establish the cancellation point and
 * leave_cancellation_point() on exit.  These functions are described below.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <assert.h>
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
 *   2. Sets state information in the caller's TCB.
 *   3. Checks if there is a pending cancellation and, if so, calls either
 *      exit() or pthread_exit(), depending upon the type of the thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void enter_cancellation_point(void)
{
  FAR struct tcb_s *tcb = this_task();

  /* Disabling pre-emption should provide sufficient protection.  We only
   * need the TCB to be stationary (no interrupt level modification is
   * anticipated).
   *
   * REVISIT: is locking the scheduler sufficient in SMP mode?
   */

  sched_lock();

  /* Do not allow nested cancellation points */

  DEBUGASSERT(!tcb->cpentered);

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then do nothing.
   */

  if (((tcb->flags & TCB_FLAG_NONCANCELABLE) == 0 &&
       (tcb->flags & TCB_FLAG_CANCEL_DEFERRED) != 0))
    {
      /* Check if there is a pending cancellation */

      if ((tcb->flags & TCB_FLAG_CANCEL_PENDING) != 0)
        {
          /* Yes, exit according to the type of the thread. */

#ifndef CONFIG_DISABLE_PTHREAD
          if ((tcb->flags & TCB_FLAG_TTYPE_MASK) ==
              TCB_FLAG_TTYPE_PTHREAD)
            {
              pthread_exit(PTHREAD_CANCELED);
            }
          else
#endif
            {
              exit(EXIT_FAILURE);
            }
        }

      /* Otherwise, indicate that we are at a cancellation point. */

      tcb->cpentered = true;
    }

  sched_unlock();
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
 *   2. Clears state information in the caller's TCB.
 *   3. Checks if there is a pending cancellation and, if so, calls either
 *      exit() or pthread_exit(), depending upon the type of the thread.
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
   * REVISIT: is locking the scheduler sufficient in SMP mode?
   */

  sched_lock();

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then do nothing.  If there is a pending
   * cancellation then just exit according to the type of the thread.
   */

  if (tcb->cpentered)
    {
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

      tcb->cpentered = false;
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
 *   2. We are within a cancellation point.
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
   * REVISIT: is locking the scheduler sufficient in SMP mode?
   */

  sched_lock();

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then return false.
   *
   * If the cpentered is true, then we within a cancellation and will true if
   * there is a pending cancellation.
   */

  if (((tcb->flags & TCB_FLAG_NONCANCELABLE) == 0 &&
       (tcb->flags & TCB_FLAG_CANCEL_DEFERRED) != 0) ||
      tcb->cpentered)
    {
      /* Check if there is a pending cancellation.  If so, return true. */

      ret = ((tcb->flags & TCB_FLAG_CANCEL_PENDING) != 0);
    }

  sched_unlock();
  return ret;
}

#endif /* CONFIG_CANCELLATION_POINTS */

/****************************************************************************
 * Name: nxnotify_cancellation
 *
 * Description:
 *   Called by task_delete() or pthread_cancel() if the cancellation occurs
 *   while we the thread is within the cancellation point.  This logic
 *   behaves much like sending a signal:  It will cause waiting threads
 *   to wake up and terminated with ECANCELED.  A call to
 *   leave_cancellation_point() would then follow, causing the thread to
 *   exit.
 *
 * Returned Value:
 *   Indicate whether the notification delivery to the target
 *
 ****************************************************************************/

bool nxnotify_cancellation(FAR struct tcb_s *tcb)
{
  irqstate_t flags;

  /* We need perform the following operations from within a critical section
   * because it can compete with interrupt level activity.
   */

  flags = enter_critical_section();

  /* We only notify the cancellation if (1) the thread has not disabled
   * cancellation, (2) the thread uses the deferred cancellation mode,
   * (3) the thread is waiting within a cancellation point.
   */

  /* Check to see if this task has the non-cancelable bit set. */

  if ((tcb->flags & TCB_FLAG_NONCANCELABLE) != 0)
    {
      /* Then we cannot cancel the thread now.  Here is how this is
       * supposed to work:
       *
       * "When cancellability is disabled, all cancels are held pending
       *  in the target thread until the thread changes the cancellability.
       *  When cancellability is deferred, all cancels are held pending in
       *  the target thread until the thread changes the cancellability,
       *  calls a function which is a cancellation point or calls
       *  pthread_testcancel(), thus creating a cancellation point.  When
       *  cancellability is asynchronous, all cancels are acted upon
       *  immediately, interrupting the thread with its processing."
       */

      tcb->flags |= TCB_FLAG_CANCEL_PENDING;
      leave_critical_section(flags);
      return true;
    }

#ifdef CONFIG_CANCELLATION_POINTS
  /* Check if this task supports deferred cancellation */

  if ((tcb->flags & TCB_FLAG_CANCEL_DEFERRED) != 0)
    {
      /* Then we cannot cancel the task asynchronously.
       * Mark the cancellation as pending.
       */

      tcb->flags |= TCB_FLAG_CANCEL_PENDING;

      /* If the task is waiting at a cancellation point, then notify of the
       * cancellation thereby waking the task up with an ECANCELED error.
       */

      if (tcb->cpentered)
        {
          /* If the thread is blocked waiting for a semaphore, then the
           * thread must be unblocked to handle the cancellation.
           */

          if (tcb->task_state == TSTATE_WAIT_SEM)
            {
              nxsem_wait_irq(tcb, ECANCELED);
            }

          /* If the thread is blocked waiting on a signal, then the
           * thread must be unblocked to handle the cancellation.
           */

          else if (tcb->task_state == TSTATE_WAIT_SIG)
            {
              nxsig_wait_irq(tcb, ECANCELED);
            }

#ifndef CONFIG_DISABLE_MQUEUE
          /* If the thread is blocked waiting on a message queue, then
           * the thread must be unblocked to handle the cancellation.
           */

          else if (tcb->task_state == TSTATE_WAIT_MQNOTEMPTY ||
                   tcb->task_state == TSTATE_WAIT_MQNOTFULL)
            {
              nxmq_wait_irq(tcb, ECANCELED);
            }
#endif
        }

      leave_critical_section(flags);
      return true;
    }
#endif

  leave_critical_section(flags);
  return false;
}
