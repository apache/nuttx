/********************************************************************************
 * pthread.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ********************************************************************************/

#ifndef __PTHREAD_H
#define __PTHREAD_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>   /* Default settings */
#include <nuttx/compiler.h> /* Compiler settings */
#include <sys/types.h>      /* Needed for general types */
#include <semaphore.h>      /* Needed for sem_t */
#include <signal.h>         /* Needed for sigset_t */
#include <time.h>           /* Needed for struct timespec */
#include <nuttx/compiler.h> /* For noreturn_function */

/********************************************************************************
 * Compilation Switches
 ********************************************************************************/

/* Standard POSIX switches */

#ifndef _POSIX_THREADS
#define _POSIX_THREADS
#endif
#ifndef _POSIX_THREAD_ATTR_STACKSIZE
#define _POSIX_THREAD_ATTR_STACKSIZE
#endif

/********************************************************************************
 * Definitions
 ********************************************************************************/

/* Values for the process shared (pshared) attribute */

#define PTHREAD_PROCESS_PRIVATE       0
#define PTHREAD_PROCESS_SHARED        1

/* Valid ranges for the pthread stacksize attribute */

#define PTHREAD_STACK_MIN             CONFIG_PTHREAD_STACK_MIN
#define PTHREAD_STACK_DEFAULT         CONFIG_PTHREAD_STACK_DEFAULT

/* Values for the pthread inheritsched attribute */

#define PTHREAD_INHERIT_SCHED         0
#define PTHREAD_EXPLICIT_SCHED        1

#define PTHREAD_PRIO_NONE             0
#define PTHREAD_PRIO_INHERIT          1
#define PTHREAD_PRIO_PROTECT          2

#define PTHREAD_DEFAULT_PRIORITY      100

/* Cancellation states returned by pthread_cancelstate() */

#define PTHREAD_CANCEL_ENABLE         (0)
#define PTHREAD_CANCEL_DISABLE        (1)

/* Thread return value when a pthread is canceled */

#define PTHREAD_CANCELED              ((FAR void*)ERROR)

/* Used to initialize a pthread_once_t */

#define PTHREAD_ONCE_INIT             (FALSE)

/* This is returned by pthread_wait.  It must not match any errno in errno.h */

#define PTHREAD_BARRIER_SERIAL_THREAD 0x1000

/********************************************************************************
 * Global Type Declarations
 ********************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/*----------------------------------------------------------*
  PTHREAD-SPECIFIC TYPES
 *----------------------------------------------------------*/

typedef int             pthread_key_t;
typedef FAR void       *pthread_addr_t;
typedef pthread_addr_t  any_t;

typedef pthread_addr_t (*pthread_startroutine_t)(pthread_addr_t);
typedef pthread_startroutine_t	pthread_func_t;

struct pthread_addr_s
{
  size_t stacksize;    /* Size of the stack allocated for the pthead */
  short  priority;     /* Priority of the pthread */
  ubyte  policy;       /* Pthread scheduler policy */
  ubyte  inheritsched; /* Inherit parent prio/policy? */
};
typedef struct pthread_addr_s pthread_attr_t;

typedef pid_t pthread_t;

typedef int pthread_condattr_t;

struct pthread_cond_s
{
  sem_t sem;
};
typedef struct pthread_cond_s pthread_cond_t;
#define PTHREAD_COND_INITIALIZER {{0, 0xffff}}

struct pthread_mutexattr_s
{
  int pshared;
};
typedef struct pthread_mutexattr_s pthread_mutexattr_t;

struct pthread_mutex_s
{
  int   pid;
  sem_t sem;
};
typedef struct pthread_mutex_s pthread_mutex_t;
#define PTHREAD_MUTEX_INITIALIZER {0, {1, 0xffff}}

struct pthread_barrierattr_s
{
  int pshared;
};
typedef struct pthread_barrierattr_s pthread_barrierattr_t;

struct pthread_barrier_s
{
  sem_t        sem;
  unsigned int count;
};
typedef struct pthread_barrier_s pthread_barrier_t;

typedef boolean pthread_once_t;

/* Forware references */

struct sched_param; /* Defined in sched.h */

/********************************************************************************
 * Global Variables
 ********************************************************************************/

/********************************************************************************
 * Global Function Prototypes
 ********************************************************************************/

/* Initializes a thread attributes object (attr) with default values for all of
 * the individual attributes used by a given implementation.
 */

EXTERN int pthread_attr_init(pthread_attr_t *attr);

/* An attributes object can be deleted when it is no longer needed. */

EXTERN int pthread_attr_destroy(pthread_attr_t *attr);

/* Set or obtain the default scheduling algorithm */

EXTERN int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);
EXTERN int pthread_attr_getschedpolicy(pthread_attr_t *attr, int *policy);
EXTERN int pthread_attr_setschedparam(pthread_attr_t *attr,
                                      const struct sched_param *param);
EXTERN int pthread_attr_getschedparam(pthread_attr_t *attr,
                                      struct sched_param *param);
EXTERN int pthread_attr_setinheritsched(pthread_attr_t *attr, int inheritsched);
EXTERN int pthread_attr_getinheritsched(const pthread_attr_t *attr,
                                        int *inheritsched);

/* Set or obtain the default stack size */

EXTERN int pthread_attr_setstacksize(pthread_attr_t *attr, long stacksize);
EXTERN int pthread_attr_getstacksize(pthread_attr_t *attr, long *stackaddr);

/* To create a thread object and runnable thread, a routine must be specified
 * as the new thread's start routine.  An argument may be passed to this
 * routine, as an untyped address; an untyped address may also be returned as
 * the routine's value.  An attributes object may be used to specify details
 * about the kind of thread being created.
 */

EXTERN int pthread_create(pthread_t *thread, pthread_attr_t *attr,
                          pthread_startroutine_t startroutine,
                          pthread_addr_t arg);

/* A thread object may be "detached" to specify that the return value and
 * completion status will not be requested.
 */

EXTERN int pthread_detach(pthread_t thread);

/* A thread may terminate it's own execution or the execution of another
 * thread.
 */

EXTERN void pthread_exit(pthread_addr_t value) noreturn_function;
EXTERN int  pthread_cancel(pthread_t thread);
EXTERN int  pthread_setcancelstate(int state, int *oldstate);
EXTERN void pthread_testcancel(void);

/* A thread can await termination of another thread and retrieve the return
 * value of the thread.
 */

EXTERN int pthread_join(pthread_t thread, pthread_addr_t *value);

/* A thread may tell the scheduler that its processor can be made available. */

EXTERN void pthread_yield(void);

/* A thread may obtain a copy of its own thread handle. */

#define pthread_self() ((pthread_t)getpid())

/* Compare two thread IDs. */

#define pthread_equal(t1,t2) (t1 == t2)

/* Thread scheduling parameters */

EXTERN int pthread_getschedparam(pthread_t thread, int *policy,
                                 struct sched_param *param);
EXTERN int pthread_setschedparam(pthread_t thread, int policy,
                                 const struct sched_param *param);
EXTERN int pthread_setschedprio(pthread_t thread, int prio);

/* Thread-specific Data Interfaces */

EXTERN int pthread_key_create(pthread_key_t *key,
			      FAR void (*destructor)(FAR void*));
EXTERN int pthread_setspecific(pthread_key_t key, FAR void *value);
EXTERN FAR void *pthread_getspecific(pthread_key_t key);
EXTERN int pthread_key_delete(pthread_key_t key);

/* Create, operate on, and destroy mutex attributes. */

EXTERN int pthread_mutexattr_init(pthread_mutexattr_t *attr);
EXTERN int pthread_mutexattr_destroy(pthread_mutexattr_t *attr);
EXTERN int pthread_mutexattr_getpshared(pthread_mutexattr_t *attr, int *pshared);
EXTERN int pthread_mutexattr_setpshared(pthread_mutexattr_t *attr, int pshared);

/* The following routines create, delete, lock and unlock mutexes. */

EXTERN int pthread_mutex_init(pthread_mutex_t *mutex, pthread_mutexattr_t *attr);
EXTERN int pthread_mutex_destroy(pthread_mutex_t *mutex);
EXTERN int pthread_mutex_lock(pthread_mutex_t *mutex);
EXTERN int pthread_mutex_trylock(pthread_mutex_t *mutex);
EXTERN int pthread_mutex_unlock(pthread_mutex_t *mutex);

/* Operations on condition variables */

EXTERN int pthread_condattr_init(pthread_condattr_t *attr);
EXTERN int pthread_condattr_destroy(pthread_condattr_t *attr);

/* A thread can create and delete condition variables. */

EXTERN int pthread_cond_init(pthread_cond_t *cond, pthread_condattr_t *attr);
EXTERN int pthread_cond_destroy(pthread_cond_t *cond);

/* A thread can signal to and broadcast on a condition variable. */

EXTERN int pthread_cond_broadcast(pthread_cond_t *cond);
EXTERN int pthread_cond_signal(pthread_cond_t *cond);

/* A thread can wait for a condition variable to be signalled or broadcast. */

EXTERN int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);

/* A thread can perform a timed wait on a condition variable. */

EXTERN int pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex,
                                  const struct timespec *abstime);

/* Barrier attributes */

EXTERN int pthread_barrierattr_destroy(FAR pthread_barrierattr_t *attr);
EXTERN int pthread_barrierattr_init(FAR pthread_barrierattr_t *attr);
EXTERN int pthread_barrierattr_getpshared(FAR const pthread_barrierattr_t *attr,
                                          FAR int *pshared);
EXTERN int pthread_barrierattr_setpshared(FAR pthread_barrierattr_t *attr,
                                          int pshared);

/* Barriers */

EXTERN int pthread_barrier_destroy(FAR pthread_barrier_t *barrier);
EXTERN int pthread_barrier_init(FAR pthread_barrier_t *barrier,
                                FAR const pthread_barrierattr_t *attr,
                                unsigned int count);
EXTERN int pthread_barrier_wait(FAR pthread_barrier_t *barrier);

/* Pthread initialization */

EXTERN int pthread_once(FAR pthread_once_t *once_control,
                        CODE void (*init_routine)(void));

/* Pthread signal management APIs */

EXTERN int pthread_kill(pthread_t thread, int sig);
EXTERN int pthread_sigmask(int how, FAR const sigset_t *set, FAR sigset_t *oset);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __PTHREAD_H */

