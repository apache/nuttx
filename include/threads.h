/****************************************************************************
 * include/threads.h
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

#ifndef __INCLUDE_THREADS_H
#define __INCLUDE_THREADS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Indicates thread error status */

#define thrd_success  (OK)
#define thrd_timedout (ETIMEDOUT)
#define thrd_busy     (EBUSY)
#define thrd_nomem    (ENOMEM)
#define thrd_error    (ERROR)

/* Defines the type of a mutex */

#define mtx_plain     0
#define mtx_recursive 1
#define mtx_timed     2

/* ONCE_FLAG_INIT: initializes a once_flag */

#define ONCE_FLAG_INIT PTHREAD_ONCE_INIT

/* thread_local: thread local type macro */

#ifndef __cplusplus
#define thread_local _Thread_local
#endif

/* tss_t: thread-specific storage pointer */

#define tss_t pthread_key_t

/* TSS_DTOR_ITERATIONS: maximum number of times destructors are called */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* thrd_t: a type identifying a thread */

#define thrd_t pthread_t

/* thrd_start_t: function pointer type passed to thrd_create */

typedef CODE int (*thrd_start_t)(FAR void *arg);

/* mtx_t : mutex identifier */

#define mtx_t pthread_mutex_t

/* once_flag: the type of the flag used by call_once */

#define once_flag pthread_once_t

/* cnd_t: condition variable identifier */

#define cnd_t pthread_cond_t

/* tss_dtor_t: function pointer type used for TSS destructor */

typedef CODE void (*tss_dtor_t)(FAR void *);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Threads ******************************************************************/

/* thrd_create: creates a thread
 *
 * int thrd_create(FAR thrd_t *thr, thrd_start_t func, FAR void *arg);
 */

#define thrd_create(thr,func,arg) \
  pthread_create(thr,NULL,(pthread_startroutine_t)(func),arg)

/* thrd_equal: checks if two identifiers refer to the same thread
 *
 * int thrd_equal(thrd_t lhs, thrd_t rhs);
 */

#define thrd_equal(lhs,rhs) ((lhs) == (rhs))

/* thrd_current: obtains the current thread identifier
 *
 * thrd_t thrd_current(void);
 */

#define thrd_current() ((thrd_t)getpid())

/* thrd_sleep: suspends execution of the calling thread for the given
 * period of time
 *
 * int thrd_sleep(FAR const struct timespec *time_point,
 *                FAR struct timespec *remaining);
 */

#define thrd_sleep(rqtp,rmtp) nanosleep(rqtp,rmtp)

/* thrd_yield: yields the current time slice
 *
 * void thrd_yield(void);
 */

#define thrd_yield() pthread_yield()

/* thrd_exit: terminates the calling thread
 *
 * _Noreturn void thrd_exit(int res);
 */

#define thrd_exit(res) pthread_exit((pthread_addr_t)(res))

/* thrd_detach: detaches a thread
 *
 * int thrd_detach(thrd_t thr);
 */

#define thrd_detach(thr) pthread_detach(thr)

/* thrd_join: blocks until a thread terminates
 *
 * int thrd_join(thrd_t thr, int *res);
 */

static inline int thrd_join(thrd_t thr, int *res)
{
  pthread_addr_t value;
  int ret = pthread_join(thr, &value);
  if (res)
    {
      *res = (int)value;
    }

  return ret;
}

/* Mutual exclusion *********************************************************/

/* mtx_init: creates a mutex
 *
 * int mtx_init(FAR mtx_t *mutex, int type);
 */

static inline int mtx_init(FAR mtx_t *mutex, int type)
{
  FAR pthread_mutexattr_t *pattr = NULL;
  pthread_mutexattr_t attr;

  if (type & mtx_recursive)
    {
      pthread_mutexattr_init(&attr);
      pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
      pattr = &attr;
    }

  return pthread_mutex_init(mutex, pattr);
}

/* mtx_lock: blocks until locks a mutex
 *
 * int mtx_lock(FAR mtx_t* mutex);
 */

#define mtx_lock(mutex) pthread_mutex_lock(mutex)

/* mtx_timedlock: blocks until locks a mutex or times out
 *
 * int mtx_timedlock(FAR mtx_t *mutex, FAR const struct timespec *tp);
 */

#define mtx_timedlock(mutex,tp) pthread_mutex_timedwait(mutex,tp)

/* mtx_trylock: locks a mutex or returns without blocking if already locked
 *
 * int mtx_trylock(FAR mtx_t *mutex);
 */

#define mtx_trylock(mutex) pthread_mutex_trylock(mutex)

/* mtx_unlock: unlocks a mutex
 *
 * int mtx_unlock(FAR mtx_t *mutex);
 */

#define mtx_unlock(mutex) pthread_mutex_unlock(mutex)

/* mtx_destroy: destroys a mutex
 *
 * void mtx_destroy(FAR mtx_t *mutex);
 */

#define mtx_destroy(mutex) pthread_mutex_destroy(mutex)

/* Call once ****************************************************************/

/* call_once: calls a function exactly once
 *
 * void call_once(FAR once_flag *flag, CODE void (*func)(void));
 */

#define call_once(flag,func) pthread_once(flag,func)

/* Condition variables ******************************************************/

/* cnd_init: creates a condition variable
 *
 * int cnd_init(FAR cnd_t *cond);
 */

#define cnd_init(cond) pthread_cond_init(cond,NULL)

/* cnd_signal: unblocks one thread blocked on a condition variable
 *
 * int cnd_signal(FAR cnd_t *cond);
 */

#define cnd_signal(cond) pthread_cond_signal(cond)

/* cnd_broadcast: unblocks all threads blocked on a condition variable
 *
 * int cnd_broadcast(FAR cnd_t *cond);
 */

#define cnd_broadcast(cond) pthread_cond_broadcast(cond)

/* cnd_wait: blocks on a condition variable
 *
 * int cnd_wait(FAR cnd_t *cond, FAR mtx_t *mutex);
 */

#define cnd_wait(cond,mutex) pthread_cond_wait(cond,mutex)

/* cnd_timedwait: blocks on a condition variable, with a timeout
 *
 * int cnd_timedwait(FAR cnd_t *cond, FAR mtx_t *mutex,
 *                   FAR const struct timespec* tp);
 */

#define cnd_timedwait(cond,mutex,tp) pthread_cond_timedwait(cond,mutex,tp)

/* cnd_destroy: destroys a condition variable
 *
 * void cnd_destroy(FAR cnd_t *cond);
 */

#define cnd_destroy(cond) pthread_cond_destroy(cond)

/* Thread-local storage *****************************************************/

/* tss_create: creates thread-specific storage pointer with a destructor
 *
 * int tss_create(FAR tss_t *tss_id, tss_dtor_t destructor);
 */

#define tss_create(tss_id,destructor) pthread_key_create(tss_id,destructor)

/* tss_get: reads from thread-specific storage
 *
 * FAR void *tss_get(tss_t tss_id);
 */

#define tss_get(tss_id) pthread_getspecific(tss_id)

/* tss_set: write to thread-specific storage
 *
 * int tss_set(tss_t tss_id, FAR void *value);
 */

#define tss_set(tss_id,value) pthread_setspecific(tss_id,value)

/* tss_delete: releases the resources held by a given thread-specific pointer
 *
 * void tss_delete(tss_t tss_id);
 */

#define tss_delete(tss_id) pthread_key_delete(tss_id)

#endif /* __INCLUDE_THREADS_H */
