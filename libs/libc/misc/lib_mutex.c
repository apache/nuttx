/****************************************************************************
 * libs/libc/misc/lib_mutex.c
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
 * Included Files
 ****************************************************************************/

#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmutex_add_backtrace
 *
 * Description:
 *   This function add the backtrace of the holder of the mutex.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

#if CONFIG_LIBC_MUTEX_BACKTRACE > 0
void nxmutex_add_backtrace(FAR mutex_t *mutex)
{
  int n;

  n = sched_backtrace(nxmutex_get_holder(&mutex), mutex->backtrace,
                      CONFIG_LIBC_MUTEX_BACKTRACE, 0);
  if (n < CONFIG_LIBC_MUTEX_BACKTRACE)
    {
      mutex->backtrace[n] = NULL;
    }
}
#endif

/****************************************************************************
 * Name: nxmutex_init
 *
 * Description:
 *   This function initializes the UNNAMED mutex. Following a
 *   successful call to nxmutex_init(), the mutex may be used in subsequent
 *   calls to nxmutex_lock(), nxmutex_unlock(), and nxmutex_trylock().  The
 *   mutex remains usable until it is destroyed.
 *
 * Parameters:
 *   mutex - Semaphore to be initialized
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxmutex_init(FAR mutex_t *mutex)
{
  int ret = nxsem_init(&mutex->sem, 0, NXSEM_NO_MHOLDER);

  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_PRIORITY_INHERITANCE
  nxsem_set_protocol(&mutex->sem, SEM_TYPE_MUTEX | SEM_PRIO_INHERIT);
#else
  nxsem_set_protocol(&mutex->sem, SEM_TYPE_MUTEX);
#endif
  return ret;
}

/****************************************************************************
 * Name: nxmutex_is_hold
 *
 * Description:
 *   This function check whether the calling thread hold the mutex
 *   referenced by 'mutex'.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

bool nxmutex_is_hold(FAR mutex_t *mutex)
{
  return nxmutex_get_holder(mutex) == _SCHED_GETTID();
}

/****************************************************************************
 * Name: nxmutex_ticklock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If the
 *   mutex value is (<=) zero, then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   mutex   - Mutex object
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to sem_trywait().
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

int nxmutex_ticklock(FAR mutex_t *mutex, uint32_t delay)
{
  int ret;

  /* Wait until we get the lock or until the timeout expires */

  if (delay)
    {
      ret = nxsem_tickwait(&mutex->sem, delay);
    }
  else
    {
      ret = nxsem_trywait(&mutex->sem);
    }

  if (ret >= 0)
    {
      nxmutex_add_backtrace(mutex);
    }

  return ret;
}

/****************************************************************************
 * Name: nxmutex_clocklock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If the
 *   mutex value is (<=) zero, then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   mutex   - Mutex object
 *   clockid - The clock to be used as the time base
 *   abstime - The absolute time when the mutex lock timed out
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

int nxmutex_clocklock(FAR mutex_t *mutex, clockid_t clockid,
                      FAR const struct timespec *abstime)
{
  int ret;

  /* Wait until we get the lock or until the timeout expires */

  if (abstime)
    {
      ret = nxsem_clockwait(&mutex->sem, clockid, abstime);
    }
  else
    {
      ret = nxsem_wait(&mutex->sem);
    }

  if (ret >= 0)
    {
      nxmutex_add_backtrace(mutex);
    }

  return ret;
}

/****************************************************************************
 * Name: nxmutex_timedlock
 *
 * Description:
 *   This function attempts to lock the mutex .  If the mutex value
 *   is (<=) zero,then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   mutex   - Mutex object
 *   timeout - The time when mutex lock timed out
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

int nxmutex_timedlock(FAR mutex_t *mutex, unsigned int timeout)
{
  struct timespec now;
  struct timespec delay;
  struct timespec rqtp;

  clock_gettime(CLOCK_MONOTONIC, &now);
  clock_ticks2time(&delay, MSEC2TICK(timeout));
  clock_timespec_add(&now, &delay, &rqtp);

  /* Wait until we get the lock or until the timeout expires */

  return nxmutex_clocklock(mutex, CLOCK_MONOTONIC, &rqtp);
}

/****************************************************************************
 * Name: nrxmutex_lock
 *
 * Description:
 *   This function attempts to lock the recursive mutex referenced by
 *   'rmutex'.The recursive mutex can be locked multiple times in the same
 *   thread.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

int nxrmutex_lock(FAR rmutex_t *rmutex)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_lock(&rmutex->mutex);
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}

/****************************************************************************
 * Name: nxrmutex_trylock
 *
 * Description:
 *   This function locks the recursive mutex if the recursive mutex is
 *   currently not locked or the same thread call.
 *   If the recursive mutex is locked and other thread call it,
 *   the call returns without blocking.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     -EINVAL - Invalid attempt to lock the recursive mutex
 *     -EAGAIN - The recursive mutex is not available.
 *
 ****************************************************************************/

int nxrmutex_trylock(FAR rmutex_t *rmutex)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_trylock(&rmutex->mutex);
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}

/****************************************************************************
 * Name: nxrmutex_ticklock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If the
 *   mutex value is (<=) zero, then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   rmutex  - Rmutex object
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to nxrmutex_trylock().
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

int nxrmutex_ticklock(FAR rmutex_t *rmutex, uint32_t delay)
{
  int ret = 0;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_ticklock(&rmutex->mutex, delay);
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}

/****************************************************************************
 * Name: nxrmutex_clocklock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If the
 *   mutex value is (<=) zero, then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   rmutex  - Rmutex object
 *   clockid - The clock to be used as the time base
 *   abstime - The absolute time when the mutex lock timed out
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

int nxrmutex_clocklock(FAR rmutex_t *rmutex, clockid_t clockid,
                       FAR const struct timespec *abstime)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_clocklock(&rmutex->mutex, clockid, abstime);
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}

/****************************************************************************
 * Name: nxrmutex_timedlock
 *
 * Description:
 *   This function attempts to lock the mutex .  If the mutex value
 *   is (<=) zero,then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   rmutex  - Rmutex object
 *   timeout - The time when mutex lock timed out
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int nxrmutex_timedlock(FAR rmutex_t *rmutex, unsigned int timeout)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_timedlock(&rmutex->mutex, timeout);
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}

/****************************************************************************
 * Name: nxrmutex_unlock
 *
 * Description:
 *   This function attempts to unlock the recursive mutex
 *   referenced by 'rmutex'.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int nxrmutex_unlock(FAR rmutex_t *rmutex)
{
  int ret = OK;

  DEBUGASSERT(rmutex->count > 0);

  if (--rmutex->count == 0)
    {
      ret = nxmutex_unlock(&rmutex->mutex);
      if (ret < 0)
        {
          ++rmutex->count;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrxmutex_breaklock
 *
 * Description:
 *   This function attempts to break the recursive mutex
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

int nxrmutex_breaklock(FAR rmutex_t *rmutex, FAR unsigned int *count)
{
  int ret = OK;

  *count = 0;
  if (nxrmutex_is_hold(rmutex))
    {
      *count = rmutex->count;
      rmutex->count = 0;
      ret = nxmutex_unlock(&rmutex->mutex);
      if (ret < 0)
        {
          rmutex->count = *count;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nxrmutex_restorelock
 *
 * Description:
 *   This function attempts to restore the recursive mutex.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

int nxrmutex_restorelock(FAR rmutex_t *rmutex, unsigned int count)
{
  int ret = OK;

  if (count != 0)
    {
      ret = nxmutex_lock(&rmutex->mutex);
      if (ret >= 0)
        {
          rmutex->count = count;
        }
    }

  return ret;
}
