/****************************************************************************
 * include/nuttx/mutex.h
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

#ifndef __INCLUDE_NUTTX_MUTEX_H
#define __INCLUDE_NUTTX_MUTEX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <stdbool.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXMUTEX_NO_HOLDER      ((pid_t)-1)
#define NXMUTEX_INITIALIZER    {NXSEM_INITIALIZER(1, SEM_TYPE_MUTEX | \
                                SEM_PRIO_INHERIT), NXMUTEX_NO_HOLDER}
#define NXRMUTEX_INITIALIZER   {NXMUTEX_INITIALIZER, 0}

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct mutex_s
{
  sem_t sem;
  pid_t holder;
#if CONFIG_LIBC_MUTEX_BACKTRACE > 0
  FAR void *backtrace[CONFIG_LIBC_MUTEX_BACKTRACE];
#endif
};

typedef struct mutex_s mutex_t;

struct rmutex_s
{
  mutex_t mutex;
  unsigned int count;
};

typedef struct rmutex_s rmutex_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
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

int nxmutex_init(FAR mutex_t *mutex);

/****************************************************************************
 * Name: nxmutex_destroy
 *
 * Description:
 *   This function initializes the UNNAMED mutex. Following a
 *   successful call to nxmutex_init(), the mutex may be used in subsequent
 *   calls to nxmutex_lock(), nxmutex_unlock(), and nxmutex_trylock().  The
 *   mutex remains usable until it is destroyed.
 *
 * Parameters:
 *   mutex - Semaphore to be destroyed
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxmutex_destroy(FAR mutex_t *mutex);

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

bool nxmutex_is_hold(FAR mutex_t *mutex);

/****************************************************************************
 * Name: nxmutex_get_holder
 *
 * Description:
 *   This function get the holder of the mutex referenced by 'mutex'.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

int nxmutex_get_holder(FAR mutex_t *mutex);

/****************************************************************************
 * Name: nxmutex_is_locked
 *
 * Description:
 *   This function get the lock state the mutex referenced by 'mutex'.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

bool nxmutex_is_locked(FAR mutex_t *mutex);

/****************************************************************************
 * Name: nxmutex_lock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  The
 *   mutex is implemented with a semaphore, so if the semaphore value is
 *   (<=) zero, then the calling task will not return until it successfully
 *   acquires the lock.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

int nxmutex_lock(FAR mutex_t *mutex);

/****************************************************************************
 * Name: nxmutex_trylock
 *
 * Description:
 *   This function locks the mutex only if the mutex is currently not locked.
 *   If the mutex has been locked already, the call returns without blocking.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     -EINVAL - Invalid attempt to lock the mutex
 *     -EAGAIN - The mutex is not available.
 *
 ****************************************************************************/

int nxmutex_trylock(FAR mutex_t *mutex);

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
                      FAR const struct timespec *abstime);

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

int nxmutex_timedlock(FAR mutex_t *mutex, unsigned int timeout);

/****************************************************************************
 * Name: nxmutex_unlock
 *
 * Description:
 *   This function attempts to unlock the mutex referenced by 'mutex'.
 *
 * Parameters:
 *   mutex - mutex descriptor.
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

int nxmutex_unlock(FAR mutex_t *mutex);

/****************************************************************************
 * Name: nxmutex_reset
 *
 * Description:
 *   This function reset lock state.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
void nxmutex_reset(FAR mutex_t *mutex);
#endif

/****************************************************************************
 * Name: nxmutex_breaklock
 *
 * Description:
 *   This function attempts to break the mutex
 *
 * Parameters:
 *   mutex   - Mutex descriptor.
 *   locked  - Is the mutex break success
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

int nxmutex_breaklock(FAR mutex_t *mutex, FAR unsigned int *locked);

/****************************************************************************
 * Name: nxmutex_restorelock
 *
 * Description:
 *   This function attempts to restore the mutex.
 *
 * Parameters:
 *   mutex   - mutex descriptor.
 *   locked  - true: it's mean that the mutex is broke success
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

int nxmutex_restorelock(FAR mutex_t *mutex, unsigned int locked);

/****************************************************************************
 * Name: nxmutex_set_protocol
 *
 * Description:
 *   This function attempts to set the priority protocol of a mutex.
 *
 * Parameters:
 *   mutex        - mutex descriptor.
 *   protocol     - mutex protocol value to set.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

int nxmutex_set_protocol(FAR mutex_t *mutex, int protocol);

/****************************************************************************
 * Name: nxmutex_getprioceiling
 *
 * Description:
 *   This function attempts to get the priority ceiling of a mutex.
 *
 * Parameters:
 *   mutex        - mutex descriptor.
 *   prioceiling  - location to return the mutex priority ceiling.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

int nxmutex_getprioceiling(FAR const mutex_t *mutex, FAR int *prioceiling);

/****************************************************************************
 * Name: nxmutex_setprioceiling
 *
 * Description:
 *   This function attempts to set the priority ceiling of a mutex.
 *
 * Parameters:
 *   mutex        - mutex descriptor.
 *   prioceiling  - mutex priority ceiling value to set.
 *   old_ceiling  - location to return the mutex ceiling priority set before.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

int nxmutex_setprioceiling(FAR mutex_t *mutex, int prioceiling,
                           FAR int *old_ceiling);

/****************************************************************************
 * Name: nxrmutex_init
 *
 * Description:
 *   This function initializes the UNNAMED recursive mutex. Following a
 *   successful call to nxrmutex_init(), the recursive mutex may be used in
 *   subsequent calls to nxrmutex_lock(), nxrmutex_unlock(),
 *   and nxrmutex_trylock(). The recursive mutex remains usable
 *   until it is destroyed.
 *
 * Parameters:
 *   rmutex - Recursive mutex to be initialized
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxrmutex_init(FAR rmutex_t *rmutex);

/****************************************************************************
 * Name: nxrmutex_destroy
 *
 * Description:
 *   This function destroy the UNNAMED recursive mutex.
 *
 * Parameters:
 *   rmutex - Recursive mutex to be destroyed
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxrmutex_destroy(FAR rmutex_t *rmutex);

/****************************************************************************
 * Name: nxrmutex_is_hold
 *
 * Description:
 *   This function check whether the calling thread hold the recursive mutex
 *   referenced by 'rmutex'.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

bool nxrmutex_is_hold(FAR rmutex_t *rmutex);

/****************************************************************************
 * Name: nxrmutex_is_recursive
 *
 * Description:
 *   This function check whether the recursive mutex is currently held
 *   recursively. That is, whether it's locked more than once by the
 *   current holder.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *  If rmutex has returned to True recursively, otherwise returns false.
 *
 ****************************************************************************/

bool nxrmutex_is_recursive(FAR rmutex_t *rmutex);

/****************************************************************************
 * Name: nxrmutex_get_holder
 *
 * Description:
 *   This function get the holder of the mutex referenced by 'mutex'.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   rmutex - Rmutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

int nxrmutex_get_holder(FAR rmutex_t *rmutex);

/****************************************************************************
 * Name: nxrmutex_is_locked
 *
 * Description:
 *   This function get the lock state the recursive mutex
 *   referenced by 'rmutex'.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

bool nxrmutex_is_locked(FAR rmutex_t *rmutex);

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

int nxrmutex_lock(FAR rmutex_t *rmutex);

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

int nxrmutex_trylock(FAR rmutex_t *rmutex);

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
                       FAR const struct timespec *abstime);

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

int nxrmutex_timedlock(FAR rmutex_t *rmutex, unsigned int timeout);

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

int nxrmutex_unlock(FAR rmutex_t *rmutex);

/****************************************************************************
 * Name: nxrmutex_reset
 *
 * Description:
 *   This function reset lock state.
 *
 * Parameters:
 *   rmutex - rmutex descriptor.
 *
 ****************************************************************************/
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
void nxrmutex_reset(FAR rmutex_t *rmutex);
#endif

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

int nxrmutex_breaklock(FAR rmutex_t *rmutex, FAR unsigned int *count);

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

int nxrmutex_restorelock(FAR rmutex_t *rmutex, unsigned int count);

#define nxrmutex_set_protocol(rmutex, protocol) \
        nxmutex_set_protocol(&(rmutex)->mutex, protocol)
#define nxrmutex_getprioceiling(rmutex, prioceiling) \
        nxmutex_getprioceiling(&(rmutex)->mutex, prioceiling)
#define nxrmutex_setprioceiling(rmutex, prioceiling, old_ceiling) \
        nxmutex_setprioceiling(&(rmutex)->mutex, prioceiling, old_ceiling)

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MUTEX_H */
