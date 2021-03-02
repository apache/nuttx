/****************************************************************************
 * libs/libc/pthread/pthread_mutex_lock.c
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

#include <pthread.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_lock
 *
 * Description:
 *   The mutex object referenced by mutex is locked by calling
 *   pthread_mutex_lock(). If the mutex is already locked, the calling thread
 *   blocks until the mutex becomes available. This operation returns with
 *   the mutex object referenced by mutex in the locked state with the
 *   calling thread as its owner.
 *
 *   If the mutex type is PTHREAD_MUTEX_NORMAL, deadlock detection is not
 *   provided. Attempting to relock the mutex causes deadlock. If a thread
 *   attempts to unlock a mutex that it has not locked or a mutex which is
 *   unlocked, undefined behavior results.
 *
 *   If the mutex type is PTHREAD_MUTEX_ERRORCHECK, then error checking is
 *   provided. If a thread attempts to relock a mutex that it has already
 *   locked, an error will be returned. If a thread attempts to unlock a
 *   mutex that it has not locked or a mutex which is unlocked, an error will
 *   be returned.
 *
 *   If the mutex type is PTHREAD_MUTEX_RECURSIVE, then the mutex maintains
 *   the concept of a lock count. When a thread successfully acquires a mutex
 *   for the first time, the lock count is set to one. Every time a thread
 *   relocks this mutex, the lock count is incremented by one. Each time the
 *   thread unlocks the mutex, the lock count is decremented by one. When the
 *   lock count reaches zero, the mutex becomes available for other threads
 *   to acquire. If a thread attempts to unlock a mutex that it has not
 *   locked or a mutex which is unlocked, an error will be returned.
 *
 *   If a signal is delivered to a thread waiting for a mutex, upon return
 *   from the signal handler the thread resumes waiting for the mutex as if
 *   it was not interrupted.
 *
 * Input Parameters:
 *   mutex - A reference to the mutex to be locked.
 *
 * Returned Value:
 *   0 on success or an errno value on failure.  Note that the errno EINTR
 *   is never returned by pthread_mutex_lock().
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   - This implementation does not return EAGAIN when the mutex could not be
 *     acquired because the maximum number of recursive locks for mutex has
 *     been exceeded.
 *
 ****************************************************************************/

int pthread_mutex_lock(FAR pthread_mutex_t *mutex)
{
  /* pthread_mutex_lock() is equivalent to pthread_mutex_timedlock() when
   * the absolute time delay is a NULL value.
   */

  return pthread_mutex_timedlock(mutex, NULL);
}
