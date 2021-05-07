/****************************************************************************
 * include/nuttx/cancelpt.h
 * Definitions related to cancellation points
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

#ifndef __INCLUDE_NUTTX_CANCELPT_H
#define __INCLUDE_NUTTX_CANCELPT_H

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
 * leave_cancellation_point() on exit. These functions are described below.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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

#ifdef CONFIG_CANCELLATION_POINTS
bool enter_cancellation_point(void);
#else
#  define enter_cancellation_point() false
#endif

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

#ifdef CONFIG_CANCELLATION_POINTS
void leave_cancellation_point(void);
#else
#  define leave_cancellation_point()
#endif

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

#ifdef CONFIG_CANCELLATION_POINTS
bool check_cancellation_point(void);
#else
#  define check_cancellation_point() false
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CANCELPT_H */
