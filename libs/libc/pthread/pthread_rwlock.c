/****************************************************************************
 * libs/libc/pthread/pthread_rwlock.c
 *
 *   Copyright (C) 2017 Mark Schulte. All rights reserved.
 *   Author: Mark Schulte <mark@mjs.pw>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pthread_rwlock_init(FAR pthread_rwlock_t *lock,
                        FAR const pthread_rwlockattr_t *attr)
{
  int err;

  if (attr != NULL)
    {
      return -ENOSYS;
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
