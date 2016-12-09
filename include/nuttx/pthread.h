/****************************************************************************
 * include/nuttx/pthread.h
 * Non-standard, NuttX-specific pthread-related declarations.
 *
 *   Copyright (C) 2011, 2015-2016 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_PTHREAD_H
#define __INCLUDE_NUTTX_PTHREAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <pthread.h>
#include <sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default pthread attribute initializer */

#if CONFIG_RR_INTERVAL == 0
#  define PTHREAD_DEFAULT_POLICY SCHED_FIFO
#else
#  define PTHREAD_DEFAULT_POLICY SCHED_RR
#endif

/* A lot of hassle to use the old-fashioned struct initializers.  But this
 * gives us backward compatibility with some very old compilers.
 */

#if defined(CONFIG_SCHED_SPORADIC) && defined(CONFIG_SMP)
#  define PTHREAD_ATTR_INITIALIZER \
  { \
    PTHREAD_DEFAULT_PRIORITY, /* priority */ \
    PTHREAD_DEFAULT_POLICY,   /* policy */ \
    PTHREAD_EXPLICIT_SCHED,   /* inheritsched */ \
    0,                        /* low_priority */ \
    0,                        /* max_repl */ \
    0,                        /* affinity */ \
    PTHREAD_STACK_DEFAULT,    /* stacksize */ \
    {0, 0},                   /* repl_period */ \
    {0, 0}                    /* budget */ \
  }
#elif defined(CONFIG_SCHED_SPORADIC)
#  define PTHREAD_ATTR_INITIALIZER \
  { \
    PTHREAD_DEFAULT_PRIORITY, /* priority */ \
    PTHREAD_DEFAULT_POLICY,   /* policy */ \
    PTHREAD_EXPLICIT_SCHED,   /* inheritsched */ \
    0,                        /* low_priority */ \
    0,                        /* max_repl */ \
    PTHREAD_STACK_DEFAULT,    /* stacksize */ \
    {0, 0},                   /* repl_period */ \
    {0, 0},                   /* budget */ \
  }
#elif defined(CONFIG_SMP)
#  define PTHREAD_ATTR_INITIALIZER \
  { \
    PTHREAD_DEFAULT_PRIORITY, /* priority */ \
    PTHREAD_DEFAULT_POLICY,   /* policy */ \
    PTHREAD_EXPLICIT_SCHED,   /* inheritsched */ \
    0,                        /* affinity */ \
    PTHREAD_STACK_DEFAULT,    /* stacksize */ \
  }
#else
#  define PTHREAD_ATTR_INITIALIZER \
  { \
    PTHREAD_DEFAULT_PRIORITY, /* priority */ \
    PTHREAD_DEFAULT_POLICY,   /* policy */ \
    PTHREAD_EXPLICIT_SCHED,   /* inheritsched */ \
    PTHREAD_STACK_DEFAULT,    /* stacksize */ \
  }
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Default pthread attributes.  This global can only be shared within the
 * kernel- or within the user- address space.
 */

EXTERN const pthread_attr_t g_default_pthread_attr;

/****************************************************************************
 * Public Function Prototypes
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
 ****************************************************************************/

#ifdef CONFIG_CANCELLATION_POINTS
void enter_cancellation_point(void);
#else
#  define enter_cancellation_point()
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
 ****************************************************************************/

#ifdef CONFIG_CANCELLATION_POINTS
void leave_cancellation_point(void);
#else
#  define leave_cancellation_point()
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_PTHREAD_H */
