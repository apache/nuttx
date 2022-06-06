/****************************************************************************
 * include/mutex.h
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

#ifndef __INCLUDE_MUTEX_H
#define __INCLUDE_MUTEX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MUTEX_NO_HOLDER      (pid_t)-1

/* Initializers */

#define MUTEX_INITIALIZER    {SEM_INITIALIZER(1),MUTEX_NO_HOLDER}
#define RMUTEX_INITIALIZER   {MUTEX_INITIALIZER,0}

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

struct mutex_s
{
  sem_t sem;
  pid_t holder;
};

typedef struct mutex_s mutex_t;

struct rmutex_s
{
  mutex_t mutex;
  uint16_t count;
};

typedef struct rmutex_s rmutex_t;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mutex_init
 *
 * Description:
 *   This function initializes the UNNAMED mutex 'mutex'. Following a
 *   successful call to mutex_init(), the mutex may be used in subsequent
 *   calls to mutex_lock(), mutex_unlock(), and mutex_trymutex->sem().  The mutex
 *   remains usable until it is destroyed.
 *
 *   Only 'mutex' itself may be used for performing synchronization. The result
 *   of referring to copies of 'mutex' in calls to mutex_lock(),
 *   mutex_trylock(), mutex_unlock(), and mutex_destroy() is undefined.
 *
 * Input Parameters:
 *   mutex - Mutex to be initialized
 *
 * Returned Value:
 *   This returns zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.
 *
 ****************************************************************************/

static inline int mutex_init(FAR mutex_t *mutex)
{
  DEBUGASSERT(mutex != NULL);

  mutex->holder = MUTEX_NO_HOLDER;
  return sem_init(&mutex->sem, 0, 1);
}

/****************************************************************************
 * Name: mutex_destroy
 *
 * Description:
 *   This function is used to destroy the un-named mutex indicated by
 *   'mutex'.  Only a mutex that was created using mutex_init() may be
 *   destroyed using mutex_destroy(); the effect of calling mutex_destroy()
 *   with a named mutex is undefined.  The effect of subsequent use of
 *   the mutex 'mutex' is undefined until 'mutex' is re-initialized by another
 *   call to mutex_init().
 *
 *   The effect of destroying a mutex upon which other processes are
 *   currently blocked is undefined.
 *
 * Input Parameters:
 *   mutex - Mutex to be destroyed.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

static inline int mutex_destroy(FAR mutex_t *mutex)
{
  DEBUGASSERT(mutex != NULL);

  return sem_destroy(&mutex->sem);
}

/****************************************************************************
 * Name: mutex_lock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If
 *   the mutex value is (<=) zero, then the calling task will not return
 *   until it successfully acquires the lock.
 *
 * Input Parameters:
 *   mutex - Mutex descriptor.
 *
 * Returned Value:
 *   This function returns zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.  Possible errno
 *   values include:
 *
 *   - EINVAL:  Invalid attempt to get the mutex
 *
 ****************************************************************************/

static inline int mutex_lock(FAR mutex_t *mutex)
{
  int ret;

  DEBUGASSERT(mutex != NULL);

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = sem_wait(&mutex->sem);
    }
  while (errno == -EINTR);

  if (ret == OK)
    {
      mutex->holder = gettid();
    }

  return ret;
}

/****************************************************************************
 * Name: mutex_trylock
 *
 * Description:
 *   This function locks the specified mutex only if the mutex is
 *   currently not locked.  In either case, the call returns without
 *   blocking.
 *
 * Input Parameters:
 *   mutex - Mutex descriptor
 *
 * Returned Value:
 *   Zero (OK) on success or -1 (ERROR) if unsuccessful. If this function
 *   returns -1(ERROR), then the cause of the failure will be reported in
 *   errno variable as:
 *
 *     EINVAL - Invalid attempt to get the mutex
 *     EAGAIN - The mutex is not available.
 *
 ****************************************************************************/

static inline int mutex_trylock(FAR mutex_t *mutex)
{
  int ret;

  DEBUGASSERT(mutex != NULL);

  ret = sem_trywait(&mutex->sem);
  if (ret == OK)
    {
      mutex->holder = gettid();
    }

  return ret;
}

/****************************************************************************
 * Name: mutex_is_locked
 *
 * Description:
 *   This function get the lock state the mutex referenced by 'mutex'.
 *
 * Parameters:
 *   mutex - Mutex descriptor.
 *
 * Return Value:
 *   true if mutex is locked and false is mutex is free
 *
 ****************************************************************************/

static inline bool mutex_is_locked(FAR mutex_t *mutex)
{
  int cnt;
  int ret;

  DEBUGASSERT(mutex != NULL);

  ret = sem_getvalue(&mutex->sem, &cnt);

  DEBUGASSERT(ret == OK);

  return cnt < 1;
}

/****************************************************************************
 * Name: mutex_unlock
 *
 * Description:
 *   When a task has finished with a mutex, it will call mutex_unlock().
 *   This function unlocks the mutex referenced by 'mutex' by performing the
 *   mutex unlock operation on that mutex.
 *
 *   If the mutex value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the mutex to become unlocked; the
 *   mutex is simply incremented.
 *
 *   If the value of the mutex resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the mutex shall be
 *   allowed to return successfully from its call to mutex_lock().
 *
 * Input Parameters:
 *   mutex - Mutex descriptor
 *
 * Returned Value:
 *   This function return zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.
 *
 ****************************************************************************/

static inline int mutex_unlock(FAR mutex_t *mutex)
{
  int ret = ERROR;

  DEBUGASSERT(mutex != NULL);

  if (mutex->holder == gettid())
    {
      mutex->holder = MUTEX_NO_HOLDER;
      ret = sem_post(&mutex->sem);
    }
  else
    {
      set_errno(EPERM);
    }

  return ret;
}

/****************************************************************************
 * Name: rmutex_init
 *
 * Description:
 *   This function initializes the UNNAMED recursive mutex 'mutex'. Following
 *   a successful call to rmutex_init(), the recursive mutex may be used in
 *   subsequent calls to rmutex_lock(), rmutex_unlock(), and rmutex_trylock().
 *   The recursive mutex remains usable until it is destroyed.
 *
 *   Only 'rmutex' itself may be used for performing synchronization.
 *   The result of referring to copies of 'rmutex' in calls to rmutex_lock(),
 *   rmutex_trylock(), rmutex_unlock(), and rmutex_destroy() is undefined.
 *
 * Input Parameters:
 *   rmutex - Recursive mutex to be initialized
 *
 * Returned Value:
 *   This returns zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.
 *
 ****************************************************************************/

static inline int rmutex_init(FAR rmutex_t *rmutex)
{
  DEBUGASSERT(rmutex != NULL);

  rmutex->count = 0;
  return mutex_init(&rmutex->mutex);
}

/****************************************************************************
 * Name: rmutex_destroy
 *
 * Description:
 *   This function is used to destroy the un-named recursive mutex indicated
 *   by 'mutex'.  Only a recursive mutex that was created using rmutex_init()
 *   may be destroyed using rmutex_destroy(); the effect of calling
 *   rmutex_destroy() with a named recursive mutex is undefined.  The effect
 *   of subsequent use of the recursive mutex 'rmutex' is undefined until
 *   'rmutex' is re-initialized by another call to rmutex_init().
 *
 *   The effect of destroying a recursive mutex upon which other processes are
 *   currently blocked is undefined.
 *
 * Input Parameters:
 *   rmutex - Recursive mutex to be destroyed.
 *
 * Returned Value:
 *   This function is a application interface.  It returns zero (OK) if
 *   successful.  Otherwise, -1 (ERROR) is returned and the errno value is
 *   set appropriately.
 *
 ****************************************************************************/

static inline int rmutex_destroy(FAR rmutex_t *rmutex)
{
  DEBUGASSERT(rmutex != NULL);

  return mutex_destroy(&rmutex->mutex);
}

/****************************************************************************
 * Name: rmutex_lock
 *
 * Description:
 *   This function attempts to lock the recursive mutex referenced by
 *   'rmutex'.  If the recursive mutex value is (<=) zero, then the calling
 *   task will not return until it successfully acquires the lock.  If the
 *   recursive mutex is already locked by the calling thread then only the
 *   nested lock counter is incremented.
 *
 * Input Parameters:
 *   rmutex - Recurseive mutex descriptor.
 *
 * Returned Value:
 *   This function returns zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.  Possible errno
 *   values include:
 *
 *   - EINVAL:  Invalid attempt to get the recursive mutex
 *
 ****************************************************************************/

static inline int rmutex_lock(FAR rmutex_t *rmutex)
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
      ret = mutex_lock(&rmutex->mutex);
      if (ret == OK)
        {
          rmutex->count = 1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rmutex_trylock
 *
 * Description:
 *   This function locks the specified recursive mutex only if the recursive
 *   mutex is currently not locked.  In either case, the call returns without
 *   blocking.
 *
 * Input Parameters:
 *   rmutex - Recursive mutex descriptor
 *
 * Returned Value:
 *   Zero (OK) on success or -1 (ERROR) if unsuccessful. If this function
 *   returns -1(ERROR), then the cause of the failure will be reported in
 *   errno variable as:
 *
 *     EINVAL - Invalid attempt to get the recursive mutex
 *     EAGAIN - The recursive mutex is not available.
 *
 ****************************************************************************/

static inline int rmutex_trylock(FAR rmutex_t *rmutex)
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
      ret = mutex_trylock(&rmutex->mutex);
      if (ret == OK)
        {
          rmutex->count = 1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rmutex_is_locked
 *
 * Description:
 *   This function get the lock state the recursive mutex referenced by
 *   'rmutex'.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   'true' if recursive mutex is locked and 'false' is recursive mutex is
 *   free
 *
 ****************************************************************************/

static inline bool rmutex_is_locked(FAR rmutex_t *rmutex)
{
  DEBUGASSERT(rmutex != NULL);

  return mutex_is_locked(&rmutex->mutex);
}

/****************************************************************************
 * Name: rmutex_unlock
 *
 * Description:
 *   When a task has finished with a recursive mutex, it will call
 *   rmutex_unlock().  This function unlocks the recursive mutex referenced
 *   by 'mutex' by performing the recursive mutex unlock operation on that
 *   recursive mutex.
 *
 *   If the recursive mutex value resulting from this operation is positive,
 *   then no tasks were blocked waiting for the recursive mutex to become
 *   unlocked; the recursive mutex is simply incremented.
 *
 *   If the value of the recursive mutex resulting from this operation is
 *   zero, then one of the tasks blocked waiting for the recursive mutex
 *   shall be allowed to return successfully from its call to rmutex_lock().
 *
 * Input Parameters:
 *   rmutex - Recursive mutex descriptor
 *
 * Returned Value:
 *   This function return zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.
 *
 ****************************************************************************/

static inline int rmutex_unlock(FAR rmutex_t *rmutex)
{
  int ret = ERROR;

  DEBUGASSERT(rmutex != NULL);

  if (rmutex->mutex.holder == gettid())
    {
      DEBUGASSERT(rmutex->count > 0);
      if (rmutex->count == 1)
        {
          rmutex->count = 0;
          ret = mutex_unlock(&rmutex->mutex);
        }
      else
        {
          rmutex->count--;
          ret = OK;
        }
    }
  else
    {
      set_errno(EPERM);
    }

  return ret;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_MUTEX_H */
