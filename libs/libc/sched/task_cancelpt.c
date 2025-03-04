/****************************************************************************
 * libs/libc/sched/task_cancelpt.c
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

#include <nuttx/cancelpt.h>
#include <nuttx/tls.h>

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
  FAR struct tls_info_s *tls = tls_get_info();
  bool ret = false;

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then do nothing.
   *
   * Special case: if the cpcount count is greater than zero, then we are
   * nested and the above condition was certainly true at the outermost
   * nesting level.
   */

  if (((tls->tl_cpstate & CANCEL_FLAG_NONCANCELABLE) == 0 &&
       (tls->tl_cpstate & CANCEL_FLAG_CANCEL_ASYNC) == 0) ||
      tls->tl_cpcount > 0)
    {
      /* Check if there is a pending cancellation */

      if ((tls->tl_cpstate & CANCEL_FLAG_CANCEL_PENDING) != 0)
        {
          /* Yes... return true (if we don't exit here) */

          ret = true;

          /* If there is a pending cancellation and we are at the outermost
           * nesting level of cancellation function calls, then exit
           * according to the type of the thread.
           */

          if (tls->tl_cpcount == 0)
            {
#ifndef CONFIG_DISABLE_PTHREAD
              pthread_exit(PTHREAD_CANCELED);
#else
              exit(EXIT_FAILURE);
#endif
            }
        }

      /* Otherwise, indicate that we are at a cancellation point by
       * incrementing the nesting level of the cancellation point
       * functions.
       */

      DEBUGASSERT(tls->tl_cpcount < INT16_MAX);
      tls->tl_cpcount++;
    }

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
  FAR struct tls_info_s *tls = tls_get_info();

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then do nothing.  Here we check only the
   * nesting level: if the cpcount count is greater than zero, then the
   * required condition was certainly true at the outermost nesting level.
   */

  if (tls->tl_cpcount > 0)
    {
      /* Decrement the nesting level.  If if would decrement to zero, then
       * we are at the outermost nesting level and may need to do more.
       */

      if (tls->tl_cpcount == 1)
        {
          /* We are no longer at the cancellation point */

          tls->tl_cpcount = 0;

          /* If there is a pending cancellation then just exit according to
           * the type of the thread.
           */

          if ((tls->tl_cpstate & CANCEL_FLAG_CANCEL_PENDING) != 0)
            {
#ifndef CONFIG_DISABLE_PTHREAD
              pthread_exit(PTHREAD_CANCELED);
#else
              exit(EXIT_FAILURE);
#endif
            }
        }
      else
        {
          /* We are not at the outermost nesting level.  Just decrment the
           * nesting level count.
           */

          tls->tl_cpcount--;
        }
    }
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
  FAR struct tls_info_s *tls = tls_get_info();
  bool ret = false;

  /* If cancellation is disabled on this thread or if this thread is using
   * asynchronous cancellation, then return false.
   *
   * If the cpcount count is greater than zero, then we within a
   * cancellation and will true if there is a pending cancellation.
   */

  if (((tls->tl_cpstate & CANCEL_FLAG_NONCANCELABLE) == 0 &&
       (tls->tl_cpstate & CANCEL_FLAG_CANCEL_ASYNC) == 0) ||
      tls->tl_cpcount > 0)
    {
      /* Check if there is a pending cancellation.  If so, return true. */

      ret = ((tls->tl_cpstate & CANCEL_FLAG_CANCEL_PENDING) != 0);
    }

  return ret;
}

#endif /* CONFIG_CANCELLATION_POINTS */
