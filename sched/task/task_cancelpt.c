/****************************************************************************
 * sched/task/task_cancelpt.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <pthread.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/cancelpt.h>
#include <nuttx/pthread.h>
#include <nuttx/tls.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"
#include "signal/signal.h"
#include "mqueue/mqueue.h"
#include "task/task.h"
#include "event/event.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxnotify_kcancel
 *
 * Description:
 *   Mark a thread that the end of its process cancels in a system call:
 *   nxtask_kcancel() ends it when the call is over.  End its wait if a
 *   signal ends it or if it is for a mutex of user code.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread to cancel.
 *
 * Assumptions:
 *   Called in a critical section.
 *
 ****************************************************************************/

#ifdef HAVE_TASK_KCANCEL
static void nxnotify_kcancel(FAR struct tcb_s *tcb)
{
  tcb->kcancel |= TASK_KCANCEL;

  switch (tcb->task_state)
    {
      case TSTATE_WAIT_SEM:

        /* No signal ends a mutex wait.  A mutex of the kernel has a holder
         * that gives it up, but a mutex of user code can have a holder
         * that has ended with the process: that wait ends.
         */

        if (NXSEM_IS_MUTEX((FAR sem_t *)tcb->waitobj) ?
            nxtask_kcancel_uwait(tcb) :
            nxtask_kcancel_wait(tcb, tcb->waitobj))
          {
            nxsem_wait_irq(tcb, ECANCELED);
          }
        break;

      case TSTATE_WAIT_SIG:
        nxsig_wait_irq(tcb, SIG_CANCEL_TIMEOUT, SI_USER, ECANCELED);
        break;

#if !defined(CONFIG_DISABLE_MQUEUE) || !defined(CONFIG_DISABLE_MQUEUE_SYSV)
      case TSTATE_WAIT_MQNOTEMPTY:
      case TSTATE_WAIT_MQNOTFULL:
        nxmq_wait_irq(tcb, ECANCELED);
        break;
#endif

#ifdef CONFIG_SIG_SIGSTOP_ACTION
      /* A thread that SIGSTOP stopped in its system call must run to its
       * end.
       */

      case TSTATE_TASK_STOPPED:
        {
          FAR struct tcb_s *rtcb = this_task();

          nxsched_remove_blocked(tcb);
          if (nxsched_add_readytorun(tcb))
            {
              up_switch_context(this_task(), rtcb);
            }
        }
        break;
#endif

      default:
        break;
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  FAR struct tls_info_s *tls = nxsched_get_tls(tcb);
  irqstate_t flags;
  bool ret = true;

  /* We need perform the following operations from within a critical section
   * because it can compete with interrupt level activity.
   */

  flags = enter_critical_section();

  /* A thread that ends with its process (a forced cancellation) and is in a
   * system call ends when the call is over, whatever its cancellation state
   * is.  This is also so in a cancellation point: there, a wait that no
   * signal ends must not end early either.
   */

  if (nxtask_kcancel_notify(tcb))
    {
      leave_critical_section(flags);
      return true;
    }

  /* We only notify the cancellation if (1) the thread has not disabled
   * cancellation, (2) the thread uses the deferred cancellation mode,
   * and is waiting within a cancellation point, (3) the thread uses the
   * asynchronous cancellation mode and currently blocked.
   */

  /* Check to see if this task has the non-cancelable bit set. */

  if ((tcb->flags & TCB_FLAG_FORCED_CANCEL) == 0 &&
      (tls->tl_cpstate & CANCEL_FLAG_NONCANCELABLE) != 0u)
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

      tls->tl_cpstate |= CANCEL_FLAG_CANCEL_PENDING;
    }

#ifdef CONFIG_CANCELLATION_POINTS
  /* Handle deferred cancellation, and asynchronous cancellation of a
   * blocked thread.  The caller cancels a thread with asynchronous
   * cancellation that is not blocked.
   */

  else if (((tls->tl_cpstate & CANCEL_FLAG_CANCEL_ASYNC) == 0u ||
           (tcb->task_state >= FIRST_BLOCKED_STATE &&
            tcb->task_state <= LAST_BLOCKED_STATE)))
    {
      /* Mark the cancellation as pending. */

      tls->tl_cpstate |= CANCEL_FLAG_CANCEL_PENDING;

      /* Treat a blocked thread with asynchronous cancellation as if it
       * waited at a cancellation point, so that it is woken below.
       */

      if ((tls->tl_cpstate & CANCEL_FLAG_CANCEL_ASYNC) != 0u)
        {
          tls->tl_cpcount = 1;
        }

      /* If the task is within a cancellation point, wake it up with
       * ECANCELED.
       */

      if (tls->tl_cpcount > 0)
        {
          /* If the thread is blocked waiting for a semaphore, then the
           * thread must be unblocked to handle the cancellation.  As for a
           * signal, not an uninterruptible wait in a system call: its
           * memory can still be in use.  The wait of pthread_join() still
           * ends.  Else the cancellation acts when the thread leaves the
           * cancellation point.
           */

          if (tcb->task_state == TSTATE_WAIT_SEM)
            {
              if ((tcb->flags & TCB_FLAG_SYSCALL) == 0 ||
                  (tcb->flags & TCB_FLAG_UNINTERRUPTIBLE) == 0 ||
                  tcb->waitobj == &tcb->join_sem)
                {
                  nxsem_wait_irq(tcb, ECANCELED);
                }
            }

          /* If the thread is blocked waiting on a signal, then the
           * thread must be unblocked to handle the cancellation.
           */

          else if (tcb->task_state == TSTATE_WAIT_SIG)
            {
              nxsig_wait_irq(tcb, SIG_CANCEL_TIMEOUT, SI_USER, ECANCELED);
            }

#ifdef CONFIG_SCHED_EVENTS
          /* If the thread is blocked waiting on a event, then the
           * thread must be unblocked to handle the cancellation.
           */

          else if (tcb->task_state == TSTATE_WAIT_EVENT)
            {
              nxevent_wait_irq(tcb, ECANCELED);
            }
#endif

#if !defined(CONFIG_DISABLE_MQUEUE) || !defined(CONFIG_DISABLE_MQUEUE_SYSV)
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

#ifdef HAVE_GROUP_MEMBERS
      else if (tcb->group && (tcb->group->tg_flags & GROUP_FLAG_EXITING))
        {
          /* Exit in progress, do asynchronous cancel instead */

          ret = false;
        }
#endif
    }
#endif

  else
    {
      /* None of the conditions are met: The thread is not non-cancelable,
       * and it's not in deferred cancellation mode (or in asynchronous
       * cancellation mode but not blocked).
       * We don't need to deliver the cancellation notification at this time.
       * Return false to indicate that the cancellation notification
       * was not needed.
       */

      ret = false;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: nxtask_kcancel_notify
 *
 * Description:
 *   Mark the target of a forced cancellation (the end of its process) if
 *   it is in a system call (nxnotify_kcancel()).  A thread that runs on
 *   another CPU is left to the SMP call of nxtask_terminate().
 *
 * Input Parameters:
 *   tcb - The TCB of the thread to cancel.
 *
 * Returned Value:
 *   true if the thread is marked, or if it is in its own end: then the
 *   caller must not end it.
 *
 * Assumptions:
 *   Called in a critical section.
 *
 ****************************************************************************/

#ifdef HAVE_TASK_KCANCEL
bool nxtask_kcancel_notify(FAR struct tcb_s *tcb)
{
  if ((tcb->flags & TCB_FLAG_FORCED_CANCEL) == 0 ||
      (tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      return false;
    }

  /* A thread in its own end (_exit(), pthread_exit()) ends by itself.  It
   * is not marked: its waits there must not end early.
   */

  if (nxtask_kcancel_exiting(tcb))
    {
      return true;
    }

  if ((tcb->flags & TCB_FLAG_SYSCALL) == 0)
    {
      return false;
    }

#ifdef CONFIG_SMP
  /* It reads the mark without the critical section (nxtask_kcancel()) */

  if (tcb->task_state == TSTATE_TASK_RUNNING && tcb->cpu != this_cpu())
    {
      return false;
    }
#endif

  nxnotify_kcancel(tcb);
  return true;
}

/****************************************************************************
 * Name: nxtask_kcancel_wait
 *
 * Description:
 *   Return true if nxnotify_kcancel() ends the wait of tcb on sem: a wait
 *   that a signal ends, or the wait of pthread_join().
 *
 ****************************************************************************/

bool nxtask_kcancel_wait(FAR struct tcb_s *tcb, FAR sem_t *sem)
{
  if ((tcb->flags & TCB_FLAG_UNINTERRUPTIBLE) == 0)
    {
      return true;
    }

#ifndef CONFIG_DISABLE_PTHREAD
  return sem == &tcb->join_sem;
#else
  return false;
#endif
}

/****************************************************************************
 * Name: nxtask_kcancel
 *
 * Description:
 *   End the calling thread if nxnotify_kcancel() marked it and its system
 *   call is over.  nxsig_unmask_pendingsignal() calls it at the end of
 *   each system call.  No cleanup handler runs, as for _exit().
 *
 * Returned Value:
 *   None.  It does not return if it ends the thread.
 *
 ****************************************************************************/

void nxtask_kcancel(void)
{
  FAR struct tcb_s *tcb = this_task();
  irqstate_t flags;
  bool cancel;

  /* The mark is set only while TCB_FLAG_SYSCALL is set and this thread
   * does not run, or on its CPU: no critical section is needed to see it.
   * It goes before the end: nx_pthread_exit() comes here again.
   */

  if ((tcb->kcancel & TASK_KCANCEL) == 0 || up_interrupt_context())
    {
      return;
    }

  flags  = enter_critical_section();
  cancel = (tcb->kcancel & TASK_KCANCEL) != 0 &&
           (tcb->flags & TCB_FLAG_SYSCALL) == 0;
  if (cancel)
    {
      tcb->kcancel = 0;
    }

  leave_critical_section(flags);

  if (!cancel)
    {
      return;
    }

#ifndef CONFIG_DISABLE_PTHREAD
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
    {
      nx_pthread_exit(PTHREAD_CANCELED);
    }
#endif

  /* As _exit(), but the other threads of the group stay: a task that
   * nxtask_delete() ends does not end its group.
   */

  enter_critical_section();
  tcb->flags |= TCB_FLAG_EXIT_PROCESSING;
  nxtask_exithook(tcb, EXIT_SUCCESS);
  up_exit(EXIT_SUCCESS);
}
#endif
