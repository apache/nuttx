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

#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXRMUTEX_NO_HOLDER     (pid_t)-1
#define NXMUTEX_INITIALIZER    SEM_INITIALIZER(1)
#define NXRMUTEX_INITIALIZER   {SEM_INITIALIZER(1), NXRMUTEX_NO_HOLDER, 0}

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef sem_t mutex_t;

struct rmutex_s
{
  mutex_t mutex;
  pid_t holder;
  uint16_t count;
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

static inline int nxmutex_init(FAR mutex_t *mutex)
{
  int ret = _SEM_INIT(mutex, 0, 1);

  if (ret < 0)
    {
      return _SEM_ERRVAL(ret);
    }

  return ret;
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
  int ret = _SEM_DESTROY(mutex);

  if (ret < 0)
    {
      return _SEM_ERRVAL(ret);
    }

  return ret;
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

  for (; ; )
    {
      /* Take the semaphore (perhaps waiting) */

      ret = _SEM_WAIT(mutex);
      if (ret >= 0)
        {
          break;
        }

      ret = _SEM_ERRVAL(ret);
      if (ret != -EINTR && ret != -ECANCELED)
        {
          break;
        }
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
  int ret = _SEM_TRYWAIT(mutex);

  if (ret < 0)
    {
      return _SEM_ERRVAL(ret);
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

  ret = _SEM_GETVALUE(mutex, &cnt);

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

  ret = _SEM_POST(mutex);
  if (ret < 0)
    {
      return _SEM_ERRVAL(ret);
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
  rmutex->count = 0;
  rmutex->holder = NXRMUTEX_NO_HOLDER;
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
  pid_t tid = gettid();
  int ret;

  if (rmutex->holder == tid)
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
          rmutex->holder = tid;
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
  pid_t tid = gettid();
  int ret;

  if (rmutex->holder == tid)
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
          rmutex->holder = tid;
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
  return rmutex->count > 0;
}

/****************************************************************************
 * Name: nxrmutex_is_hold
 *
 * Description:
 *   This function check whether the caller hold the recursive mutex
 *   referenced by 'rmutex'.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

static inline bool nxrmutex_is_hold(FAR rmutex_t *rmutex)
{
  return rmutex->holder == gettid();
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
  pid_t tid = gettid();
  int ret = OK;

  DEBUGASSERT(rmutex->holder == tid);
  DEBUGASSERT(rmutex->count > 0);

  if (rmutex->count == 1)
    {
      rmutex->count = 0;
      rmutex->holder = NXRMUTEX_NO_HOLDER;
      ret = nxmutex_unlock(&rmutex->mutex);
    }
  else
    {
      rmutex->count--;
    }

  return ret;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MUTEX_H */
