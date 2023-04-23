/****************************************************************************
 * sched/pthread/pthread_barrierwait.c
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

#include <nuttx/irq.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_barrier_wait
 *
 * Description:
 *   The pthread_barrier_wait() function synchronizse participating threads
 *   at the barrier referenced by 'barrier'.  The calling thread is blocked
 *   until the required number of threads have called pthread_barrier_wait()
 *   specifying the same 'barrier'.  When the required number of threads
 *   have called pthread_barrier_wait() specifying the 'barrier', the
 *   constant PTHREAD_BARRIER_SERIAL_THREAD will be returned to one
 *   unspecified thread and zero will be returned to each of the remaining
 *   threads. At this point, the barrier will be reset to the state it had
 *   as a result of the most recent pthread_barrier_init() function that
 *   referenced it.
 *
 *   The constant PTHREAD_BARRIER_SERIAL_THREAD is defined in pthread.h and
 *   its value must be distinct from any other value returned by
 *   pthread_barrier_wait().
 *
 *   The results are undefined if this function is called with an
 *   uninitialized barrier.
 *
 *   If a signal is delivered to a thread blocked on a barrier, upon return
 *   from the signal handler the thread will resume waiting at the barrier
 *   if the barrier wait has not completed; otherwise, the thread will
 *   continue as normal from the completed barrier wait. Until the thread in
 *   the signal handler returns from it, it is unspecified whether other
 *   threads may proceed past the barrier once they have all reached it.
 *
 *   A thread that has blocked on a barrier will not prevent any unblocked
 *   thread that is eligible to use the same processing resources from
 *   eventually making forward progress in its execution.  Eligibility for
 *   processing resources will be determined by the scheduling policy.
 *
 * Input Parameters:
 *   barrier - the barrier to wait on
 *
 * Returned Value:
 *   0 (OK) on success or EINVAL if the barrier is not valid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_barrier_wait(FAR pthread_barrier_t *barrier)
{
  int semcount;
  int ret = OK;
  irqstate_t flags;

  if (!barrier)
    {
      return EINVAL;
    }

  /* Disable pre-emption throughout the following */

  flags = enter_critical_section();

  /* Find out how many threads are already waiting at the barrier */

  ret = sem_getvalue(&barrier->sem, &semcount);
  if (ret != OK)
    {
      leave_critical_section(flags);
      return get_errno();
    }

  /* If the number of waiters would be equal to the count, then we are done */

  if ((1 - semcount) >= (int)barrier->count)
    {
      /* Free all of the waiting threads */

      while (semcount < 0)
        {
          sem_post(&barrier->sem);
          sem_getvalue(&barrier->sem, &semcount);
        }

      /* Then return PTHREAD_BARRIER_SERIAL_THREAD to the final thread */

      leave_critical_section(flags);
      return PTHREAD_BARRIER_SERIAL_THREAD;
    }
  else
    {
      /* Otherwise, this thread must wait as well */

      while (sem_wait(&barrier->sem) != OK)
        {
          /* If the thread is awakened by a signal, just continue to wait */

          int errornumber = get_errno();
          if (errornumber != EINTR)
            {
              /* If it is awakened by some other error, then there is a
               * problem
               */

              leave_critical_section(flags);
              return errornumber;
            }
        }

      /* We will only get here when we are one of the N-1 threads that were
       * waiting for the final thread at the barrier.  We just need to return
       * zero.
       */

      leave_critical_section(flags);
      return 0;
    }
}
