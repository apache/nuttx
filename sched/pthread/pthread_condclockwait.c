/****************************************************************************
 * sched/pthread/pthread_condclockwait.c
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/signal.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "pthread/pthread.h"
#include "clock/clock.h"
#include "signal/signal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_condtimedout
 *
 * Description:
 *   This function is called if the timeout elapses before
 *   the condition is signaled.
 *
 * Input Parameters:
 *   arg   - the argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void pthread_condtimedout(wdparm_t arg)
{
  pid_t pid = arg;

#ifdef HAVE_GROUP_MEMBERS
    {
      FAR struct tcb_s *tcb;
      siginfo_t info;

      /* The logic below if equivalent to nxsig_queue(), but uses
       * nxsig_tcbdispatch() instead of nxsig_dispatch().  This avoids the
       * group signal deliver logic and assures, instead, that the signal is
       * delivered specifically to this thread that is known to be waiting on
       * the signal.
       */

      /* Get the waiting TCB.  nxsched_get_tcb() might return NULL if the
       * task has exited for some reason.
       */

      tcb = nxsched_get_tcb(pid);
      if (tcb)
        {
          /* Create the siginfo structure */

          info.si_signo           = SIGCONDTIMEDOUT;
          info.si_code            = SI_QUEUE;
          info.si_errno           = ETIMEDOUT;
          info.si_value.sival_ptr = NULL;
#ifdef CONFIG_SCHED_HAVE_PARENT
          info.si_pid             = pid;
          info.si_status          = OK;
#endif

          /* Process the receipt of the signal.  The scheduler is not locked
           * as is normally the case when this function is called because we
           * are in a watchdog timer interrupt handler.
           */

          nxsig_tcbdispatch(tcb, &info);
        }
    }
#else /* HAVE_GROUP_MEMBERS */
    {
      /* Things are a little easier if there are not group members.  We can
       *  just use nxsig_queue().
       */

      union sigval value;

      /* Send the specified signal to the specified task. */

      value.sival_ptr = NULL;
      nxsig_queue(pid, SIGCONDTIMEDOUT, value);
    }

#endif /* HAVE_GROUP_MEMBERS */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cond_clockwait
 *
 * Description:
 *   A thread can perform a timed wait on a condition variable.
 *
 * Input Parameters:
 *   cond    - the condition variable to wait on
 *   mutex   - the mutex that protects the condition variable
 *   clockid - The timing source to use in the conversion
 *   abstime - wait until this absolute time
 *
 * Returned Value:
 *   OK (0) on success; A non-zero errno value is returned on failure.
 *
 * Assumptions:
 *   Timing is of resolution 1 msec, with +/-1 millisecond accuracy.
 *
 ****************************************************************************/

int pthread_cond_clockwait(FAR pthread_cond_t *cond,
                           FAR pthread_mutex_t *mutex,
                           clockid_t clockid,
                           FAR const struct timespec *abstime)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  sclock_t ticks;
  int mypid = getpid();
  int ret = OK;
  int status;

  sinfo("cond=0x%p mutex=0x%p abstime=0x%p\n", cond, mutex, abstime);

  /* pthread_cond_clockwait() is a cancellation point */

  enter_cancellation_point();

  /* Make sure that non-NULL references were provided. */

  if (!cond || !mutex)
    {
      ret = EINVAL;
    }

  /* Make sure that the caller holds the mutex */

  else if (mutex->pid != mypid)
    {
      ret = EPERM;
    }

  /* If no wait time is provided, this function degenerates to
   * the same behavior as pthread_cond_wait().
   */

  else if (!abstime)
    {
      ret = pthread_cond_wait(cond, mutex);
    }

  else
    {
      sinfo("Give up mutex...\n");

      /* We must disable pre-emption and interrupts here so that
       * the time stays valid until the wait begins.   This adds
       * complexity because we assure that interrupts and
       * pre-emption are re-enabled correctly.
       */

      sched_lock();
      flags = enter_critical_section();

      /* Convert the timespec to clock ticks.  We must disable pre-
       * emption here so that this time stays valid until the wait
       * begins.
       */

      ret = clock_abstime2ticks(clockid, abstime, &ticks);
      if (ret)
        {
          /* Restore interrupts  (pre-emption will be enabled when
           * we fall through the if/then/else)
           */

          leave_critical_section(flags);
        }
      else
        {
          /* Check the absolute time to wait.  If it is now or in the
           * past, then just return with the timedout condition.
           */

          if (ticks <= 0)
            {
              /* Restore interrupts and indicate that we have already
               * timed out. (pre-emption will be enabled when we fall
               * through the if/then/else
               */

              leave_critical_section(flags);
              ret = ETIMEDOUT;
            }
          else
            {
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
              uint8_t mflags;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
              uint8_t type;
              int16_t nlocks;
#endif
              /* Give up the mutex */

              mutex->pid = -1;
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
              mflags     = mutex->flags;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
              type       = mutex->type;
              nlocks     = mutex->nlocks;
#endif
              ret        = pthread_mutex_give(mutex);
              if (ret != 0)
                {
                  /* Restore interrupts  (pre-emption will be enabled
                   * when we fall through the if/then/else)
                   */

                  leave_critical_section(flags);
                }
              else
                {
                  /* Start the watchdog */

                  wd_start(&rtcb->waitdog, ticks,
                           pthread_condtimedout, mypid);

                  /* Take the condition semaphore.  Do not restore
                   * interrupts until we return from the wait.  This is
                   * necessary to make sure that the watchdog timer and
                   * the condition wait are started atomically.
                   */

                  status = nxsem_wait((FAR sem_t *)&cond->sem);

                  /* We no longer need the watchdog */

                  wd_cancel(&rtcb->waitdog);

                  /* Did we get the condition semaphore. */

                  if (status < 0)
                    {
                      /* NO.. Handle the special case where the semaphore
                       * wait was awakened by the receipt of a signal --
                       * presumably the signal posted by
                       * pthread_condtimedout().
                       */

                      if (status == -EINTR)
                        {
                          swarn("WARNING: Timedout!\n");
                          ret = ETIMEDOUT;
                        }
                      else
                        {
                          ret = status;
                        }
                    }

                  /* The interrupts stay disabled until after we sample
                   * the errno.  This is because when debug is enabled
                   * and the console is used for debug output, then the
                   * errno can be altered by interrupt handling! (bad)
                   */

                  leave_critical_section(flags);
                }

              /* Reacquire the mutex (retaining the ret). */

              sinfo("Re-locking...\n");

              status = pthread_mutex_take(mutex, NULL, false);
              if (status == OK)
                {
                  mutex->pid    = mypid;
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
                  mutex->flags  = mflags;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
                  mutex->type   = type;
                  mutex->nlocks = nlocks;
#endif
                }
              else if (ret == 0)
                {
                  ret           = status;
                }
            }
        }

      /* Re-enable pre-emption (It is expected that interrupts
       * have already been re-enabled in the above logic)
       */

      sched_unlock();
    }

  leave_cancellation_point();
  sinfo("Returning %d\n", ret);
  return ret;
}
