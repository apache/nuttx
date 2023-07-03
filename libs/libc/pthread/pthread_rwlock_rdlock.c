/****************************************************************************
 * libs/libc/pthread/pthread_rwlock_rdlock.c
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

#include <stdint.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_PTHREAD_CLEANUP_STACKSIZE) && CONFIG_PTHREAD_CLEANUP_STACKSIZE > 0
static void rdlock_cleanup(FAR void *arg)
{
  FAR pthread_rwlock_t *rw_lock = (FAR pthread_rwlock_t *)arg;

  pthread_mutex_unlock(&rw_lock->lock);
}
#endif

static int tryrdlock(FAR pthread_rwlock_t *rw_lock)
{
  int err;

  if (rw_lock->num_writers > 0 || rw_lock->write_in_progress)
    {
      err = EBUSY;
    }
  else if (rw_lock->num_readers == UINT_MAX)
    {
      err = EAGAIN;
    }
  else
    {
      rw_lock->num_readers++;
      err = OK;
    }

  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_rwlock_rdlock
 *
 * Description:
 *   Locks a read/write lock for reading
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_rwlock_tryrdlock(FAR pthread_rwlock_t *rw_lock)
{
  int err = pthread_mutex_trylock(&rw_lock->lock);

  if (err != 0)
    {
      return err;
    }

  err = tryrdlock(rw_lock);

  pthread_mutex_unlock(&rw_lock->lock);
  return err;
}

int pthread_rwlock_clockrdlock(FAR pthread_rwlock_t *rw_lock,
                               clockid_t clockid,
                               FAR const struct timespec *ts)
{
  int err = pthread_mutex_lock(&rw_lock->lock);

  if (err != 0)
    {
      return err;
    }

#if defined(CONFIG_PTHREAD_CLEANUP_STACKSIZE) && CONFIG_PTHREAD_CLEANUP_STACKSIZE > 0
  pthread_cleanup_push(&rdlock_cleanup, rw_lock);
#endif
  while ((err = tryrdlock(rw_lock)) == EBUSY)
    {
      if (ts != NULL)
        {
          err = pthread_cond_clockwait(&rw_lock->cv, &rw_lock->lock,
                                       clockid, ts);
        }
      else
        {
          err = pthread_cond_wait(&rw_lock->cv, &rw_lock->lock);
        }

      if (err != 0)
        {
          break;
        }
    }

#if defined(CONFIG_PTHREAD_CLEANUP_STACKSIZE) && CONFIG_PTHREAD_CLEANUP_STACKSIZE > 0
  pthread_cleanup_pop(0);
#endif

  pthread_mutex_unlock(&rw_lock->lock);
  return err;
}

int pthread_rwlock_timedrdlock(FAR pthread_rwlock_t *rw_lock,
                               FAR const struct timespec *ts)
{
  return pthread_rwlock_clockrdlock(rw_lock, CLOCK_REALTIME, ts);
}

int pthread_rwlock_rdlock(FAR pthread_rwlock_t * rw_lock)
{
  return pthread_rwlock_timedrdlock(rw_lock, NULL);
}
