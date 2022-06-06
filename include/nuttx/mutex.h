/****************************************************************************
 * include/nuttx/mutex.h
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

#include <nuttx/config.h>

#include <errno.h>
#include <mutex.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Initializers */

#define NXMUTEX_INITIALIZER  {NXSEM_INITIALIZER(1,0),MUTEX_NO_HOLDER}
#define NXRMUTEX_INITIALIZER {NXMUTEX_INITIALIZER,0}

/* Most internal nxsem_* interfaces are not available in the user space in
 * PROTECTED and KERNEL builds.  In that context, the application semaphore
 * interfaces must be used.  The differences between the two sets of
 * interfaces are:  (1) the nxsem_* interfaces do not cause cancellation
 * points and (2) they do not modify the errno variable.
 *
 * This is only important when compiling libraries (libc or libnx) that are
 * used both by the OS (libkc.a and libknx.a) or by the applications
 * (libc.a and libnx.a).  In that case, the correct interface must be
 * used for the build context.
 *
 * REVISIT:  In the flat build, the same functions must be used both by
 * the OS and by applications.  We have to use the normal user functions
 * in this case or we will fail to set the errno or fail to create the
 * cancellation point.
 */

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
#  define _MUTEX_INIT(m)       nxmutex_init(m)
#  define _MUTEX_DESTROY(m)    nxmutex_destroy(m)
#  define _MUTEX_LOCK(m)       nxmutex_lock(m)
#  define _MUTEX_TRYLOCK(m)    nxmutex_trylock(m)
#  define _MUTEX_IS_LOCKED(m)  nxmutex_is_locked(m)
#  define _MUTEX_UNLOCK(m)     nxmutex_unlock(m)
#  define _MUTEX_ERRNO(r)      (-(r))
#  define _MUTEX_ERRVAL(r)     (r)

#  define _RMUTEX_INIT(m)      nxrmutex_init(m)
#  define _RMUTEX_DESTROY(m)   nxrmutex_destroy(m)
#  define _RMUTEX_LOCK(m)      nxrmutex_lock(m)
#  define _RMUTEX_TRYLOCK(m)   nxrmutex_trylock(m)
#  define _RMUTEX_IS_LOCKED(m) nxrmutex_is_locked(m)
#  define _RMUTEX_UNLOCK(m)    nxrmutex_unlock(m)
#  define _RMUTEX_ERRNO(r)     (-(r))
#  define _RMUTEX_ERRVAL(r)    (r)
#else
#  define _MUTEX_INIT(m)       mutex_init(m)
#  define _MUTEX_DESTROY(m)    mutex_destroy(m)
#  define _MUTEX_LOCK(m)       mutex_lock(m)
#  define _MUTEX_TRYLOCK(m)    mutex_trylock(m)
#  define _MUTEX_IS_LOCKED(m)  mutex_is_locked(m)
#  define _MUTEX_UNLOCK(m)     mutex_unlock(m)
#  define _MUTEX_ERRNO(r)      errno
#  define _MUTEX_ERRVAL(r)     (-errno)

#  define _RMUTEX_INIT(m)      rmutex_init(m)
#  define _RMUTEX_DESTROY(m)   rmutex_destroy(m)
#  define _RMUTEX_LOCK(m)      rmutex_lock(m)
#  define _RMUTEX_TRYLOCK(m)   rmutex_trylock(m)
#  define _RMUTEX_IS_LOCKED(m) rmutex_is_locked(m)
#  define _RMUTEX_UNLOCK(m)    rmutex_unlock(m)
#  define _RMUTEX_ERRNO(r)     errno
#  define _RMUTEX_ERRVAL(r)    (-errno)
#endif

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

static inline int nxmutex_init(FAR mutex_t *mutex)
{
  DEBUGASSERT(mutex != NULL);

  mutex->holder = MUTEX_NO_HOLDER;
  return nxsem_init(&mutex->sem, 0, 1);
}

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

static inline int nxmutex_destroy(FAR mutex_t *mutex)
{
  DEBUGASSERT(mutex != NULL);

  return nxsem_destroy(&mutex->sem);
}

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

static inline int nxmutex_lock(FAR mutex_t *mutex)
{
  int ret;

  DEBUGASSERT(mutex != NULL);

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait_uninterruptible(&mutex->sem);
    }
  while (ret == -ECANCELED);

  if (ret == OK)
    {
      mutex->holder = gettid();
    }

  return ret;
}

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

static inline int nxmutex_trylock(FAR mutex_t *mutex)
{
  int ret;

  DEBUGASSERT(mutex != NULL);

  ret = nxsem_trywait(&mutex->sem);
  if (ret == OK)
    {
      mutex->holder = gettid();
    }

  return ret;
}

/****************************************************************************
 * Name: nxmutex_is_locked
 *
 * Description:
 *   This function get the lock state the mutex referenced by 'mutex'.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

static inline bool nxmutex_is_locked(FAR mutex_t *mutex)
{
  int cnt;
  int ret;

  DEBUGASSERT(mutex != NULL);

  ret = nxsem_get_value(&mutex->sem, &cnt);

  DEBUGASSERT(ret == OK);

  return cnt < 1;
}

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

static inline int nxmutex_unlock(FAR mutex_t *mutex)
{
  int ret;

  DEBUGASSERT(mutex != NULL);

  if (mutex->holder == gettid())
    {
      mutex->holder = MUTEX_NO_HOLDER;
      ret = nxsem_post(&mutex->sem);
    }
  else
    {
      ret = -EPERM;
    }

  return ret;
}

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

static inline int nxrmutex_init(FAR rmutex_t *rmutex)
{
  DEBUGASSERT(rmutex != NULL);

  rmutex->count = 0;
  return nxmutex_init(&rmutex->mutex);
}

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

static inline int nxrmutex_destroy(FAR rmutex_t *rmutex)
{
  DEBUGASSERT(rmutex != NULL);

  return nxmutex_destroy(&rmutex->mutex);
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

static inline int nxrmutex_lock(FAR rmutex_t *rmutex)
{
  int ret;

  DEBUGASSERT(rmutex != NULL);

  if (rmutex->mutex.holder == gettid())
    {
      DEBUGASSERT(rmutex->count < UINT16_MAX);
      rmutex->count++;
      ret = OK;
    }
  else
    {
      ret = nxmutex_lock(&rmutex->mutex);
      if (ret == OK)
        {
          rmutex->count = 1;
        }
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

static inline int nxrmutex_trylock(FAR rmutex_t *rmutex)
{
  int ret;

  DEBUGASSERT(rmutex != NULL);

  if (rmutex->mutex.holder == gettid())
    {
      DEBUGASSERT(rmutex->count < UINT16_MAX);
      rmutex->count++;
      ret = OK;
    }
  else
    {
      ret = nxmutex_trylock(&rmutex->mutex);
      if (ret == OK)
        {
          rmutex->count = 1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nxrmutex_is_locked
 *
 * Description:
 *   This function get the lock state the recursive mutex
 *   referenced by 'rmutex'.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

static inline bool nxrmutex_is_locked(FAR rmutex_t *rmutex)
{
  DEBUGASSERT(rmutex != NULL);

  return nxmutex_is_locked(&rmutex->mutex);
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

static inline int nxrmutex_unlock(FAR rmutex_t *rmutex)
{
  int ret;

  DEBUGASSERT(rmutex != NULL);

  if (rmutex->mutex.holder == gettid())
    {
      DEBUGASSERT(rmutex->count > 0);
      if (rmutex->count == 1)
        {
          rmutex->count = 0;
          ret = nxmutex_unlock(&rmutex->mutex);
        }
      else
        {
          rmutex->count--;
          ret = OK;
        }
    }
  else
    {
      ret = -EPERM;
    }

  return ret;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MUTEX_H */
