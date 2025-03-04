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

#include <nuttx/irq.h>
#include <nuttx/cancelpt.h>
#include <nuttx/tls.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"
#include "signal/signal.h"
#include "mqueue/mqueue.h"
#include "task/task.h"

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
  bool ret = false;

  /* We need perform the following operations from within a critical section
   * because it can compete with interrupt level activity.
   */

  flags = enter_critical_section();

  /* We only notify the cancellation if (1) the thread has not disabled
   * cancellation, (2) the thread uses the deferred cancellation mode,
   * (3) the thread is waiting within a cancellation point.
   */

  /* Check to see if this task has the non-cancelable bit set. */

  if ((tcb->flags & TCB_FLAG_FORCED_CANCEL) == 0 &&
      (tls->tl_cpstate & CANCEL_FLAG_NONCANCELABLE) != 0)
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
      leave_critical_section(flags);
      return true;
    }

#ifdef CONFIG_CANCELLATION_POINTS
  /* Check if this task supports deferred cancellation */

  if ((tls->tl_cpstate & CANCEL_FLAG_CANCEL_ASYNC) == 0)
    {
      /* Then we cannot cancel the task asynchronously. */

      ret = true;

      /* Mark the cancellation as pending. */

      tls->tl_cpstate |= CANCEL_FLAG_CANCEL_PENDING;

      /* If the task is waiting at a cancellation point, then notify of the
       * cancellation thereby waking the task up with an ECANCELED error.
       */

      if (tls->tl_cpcount > 0)
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

  leave_critical_section(flags);
  return ret;
}
