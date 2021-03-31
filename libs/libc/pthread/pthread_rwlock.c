/****************************************************************************
 * libs/libc/pthread/pthread_rwlock.c
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
 * Public Functions
 ****************************************************************************/

int pthread_rwlock_init(FAR pthread_rwlock_t *lock,
                        FAR const pthread_rwlockattr_t *attr)
{
  int err;

  if (attr != NULL)
    {
      return ENOSYS;
    }

  lock->num_readers       = 0;
  lock->num_writers       = 0;
  lock->write_in_progress = false;

  err = pthread_cond_init(&lock->cv, NULL);
  if (err != 0)
    {
      return err;
    }

  err = pthread_mutex_init(&lock->lock, NULL);
  if (err != 0)
    {
      pthread_cond_destroy(&lock->cv);
      return err;
    }

  return err;
}

int pthread_rwlock_destroy(FAR pthread_rwlock_t *lock)
{
  int cond_err  = pthread_cond_destroy(&lock->cv);
  int mutex_err = pthread_mutex_destroy(&lock->lock);

  if (mutex_err)
    {
      return mutex_err;
    }

  return cond_err;
}

int pthread_rwlock_unlock(FAR pthread_rwlock_t *rw_lock)
{
  int err;

  err = pthread_mutex_lock(&rw_lock->lock);
  if (err != 0)
    {
      return err;
    }

  if (rw_lock->num_readers > 0)
    {
      rw_lock->num_readers--;

      if (rw_lock->num_readers == 0)
        {
          err = pthread_cond_broadcast(&rw_lock->cv);
        }
    }
  else if (rw_lock->write_in_progress)
    {
      rw_lock->write_in_progress = false;

      err = pthread_cond_broadcast(&rw_lock->cv);
    }
  else
    {
      err = EINVAL;
    }

  pthread_mutex_unlock(&rw_lock->lock);
  return err;
}
