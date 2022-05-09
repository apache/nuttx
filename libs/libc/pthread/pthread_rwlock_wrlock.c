/****************************************************************************
 * libs/libc/pthread/pthread_rwlock_wrlock.c
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

#ifdef CONFIG_PTHREAD_CLEANUP
static void wrlock_cleanup(FAR void *arg)
{
  FAR pthread_rwlock_t *rw_lock = (FAR pthread_rwlock_t *)arg;

  rw_lock->num_writers--;
  pthread_mutex_unlock(&rw_lock->lock);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_rwlock_wrlock
 *
 * Description:
 *   Locks a read/write lock for writing
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

int pthread_rwlock_trywrlock(FAR pthread_rwlock_t *rw_lock)
{
  int err = pthread_mutex_trylock(&rw_lock->lock);

  if (err != 0)
    {
      return err;
    }

  if (rw_lock->num_readers > 0 || rw_lock->write_in_progress)
    {
      err = EBUSY;
    }
  else
    {
      rw_lock->write_in_progress = true;
    }

  pthread_mutex_unlock(&rw_lock->lock);
  return err;
}

int pthread_rwlock_clockwrlock(FAR pthread_rwlock_t *rw_lock,
                               clockid_t clockid,
                               FAR const struct timespec *ts)
{
  int err = pthread_mutex_lock(&rw_lock->lock);

  if (err != 0)
    {
      return err;
    }

  if (rw_lock->num_writers == UINT_MAX)
    {
      err = EAGAIN;
      goto exit_with_mutex;
    }

  rw_lock->num_writers++;

#ifdef CONFIG_PTHREAD_CLEANUP
  pthread_cleanup_push(&wrlock_cleanup, rw_lock);
#endif
  while (rw_lock->write_in_progress || rw_lock->num_readers > 0)
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

#ifdef CONFIG_PTHREAD_CLEANUP
  pthread_cleanup_pop(0);
#endif

  if (err == 0)
    {
      rw_lock->write_in_progress = true;
    }
  else
    {
      /* In case of error, notify any blocked readers. */

      pthread_cond_broadcast(&rw_lock->cv);
    }

  rw_lock->num_writers--;

exit_with_mutex:
  pthread_mutex_unlock(&rw_lock->lock);
  return err;
}

int pthread_rwlock_timedwrlock(FAR pthread_rwlock_t *rw_lock,
                               FAR const struct timespec *ts)
{
  return pthread_rwlock_clockwrlock(rw_lock, CLOCK_REALTIME, ts);
}

int pthread_rwlock_wrlock(FAR pthread_rwlock_t *rw_lock)
{
  return pthread_rwlock_timedwrlock(rw_lock, NULL);
}
