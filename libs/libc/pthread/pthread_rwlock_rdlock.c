/****************************************************************************
 * libs/libc/pthread/pthread_rwlock_rdlock.c
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
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PTHREAD_CLEANUP
static void rdlock_cleanup(FAR void *arg)
{
  FAR pthread_rwlock_t *rw_lock = (FAR pthread_rwlock_t *)arg;

  (void)pthread_mutex_unlock(&rw_lock->lock);
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

int pthread_rwlock_timedrdlock(FAR pthread_rwlock_t *rw_lock,
                               FAR const struct timespec *ts)
{
  int err = pthread_mutex_lock(&rw_lock->lock);

  if (err != 0)
    {
      return err;
    }

#ifdef CONFIG_PTHREAD_CLEANUP
  pthread_cleanup_push(&rdlock_cleanup, rw_lock);
#endif
  while ((err = tryrdlock(rw_lock)) == EBUSY)
    {
      if (ts != NULL)
        {
          err = pthread_cond_timedwait(&rw_lock->cv, &rw_lock->lock, ts);
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

  pthread_mutex_unlock(&rw_lock->lock);
  return err;
}

int pthread_rwlock_rdlock(FAR pthread_rwlock_t * rw_lock)
{
  return pthread_rwlock_timedrdlock(rw_lock, NULL);
}
