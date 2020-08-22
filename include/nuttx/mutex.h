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
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MUTEX_INITIALIZER    SEM_INITIALIZER(1);

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef sem_t mutex_t;

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
  return nxsem_init(mutex, 0, 1);
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
  return nxsem_destroy(mutex);
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
  return nxsem_wait_uninterruptible(mutex);
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
  return nxsem_trywait(mutex);
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

  ret = nxsem_get_value(mutex, &cnt);

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
  return nxsem_post(mutex);
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MUTEX_H */
