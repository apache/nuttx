/****************************************************************************
 * libs/libc/pthread/pthread_spinlock.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <pthread.h>
#include <sched.h>

#include <nuttx/atomic.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMPOSSIBLE_THREAD ((pthread_t)-1)

/* Number of tests of a busy lock before the thread yields the CPU.  With
 * one CPU the holder cannot run while this thread spins: yield at once.
 */

#ifdef CONFIG_SMP
#  define SPIN_TRIES      100
#else
#  define SPIN_TRIES      1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_spin_init
 *
 * Description:
 *   Initialize the spin lock to the unlocked state.  The lock is an atomic
 *   variable, so it also works in memory that processes share
 *   (PTHREAD_PROCESS_SHARED).
 *
 * Input Parameters:
 *   lock    - A reference to the spinlock object to be initialized.
 *   pshared - PTHREAD_PROCESS_PRIVATE or PTHREAD_PROCESS_SHARED
 *
 * Returned Value:
 *   Zero (OK) on success, EINVAL if an argument is not valid.
 *
 ****************************************************************************/

int pthread_spin_init(FAR pthread_spinlock_t *lock, int pshared)
{
  if (lock == NULL || (pshared != PTHREAD_PROCESS_PRIVATE &&
                       pshared != PTHREAD_PROCESS_SHARED))
    {
      return EINVAL;
    }

  atomic_set(&lock->sp_lock, 0);
  lock->sp_holder = IMPOSSIBLE_THREAD;
  return OK;
}

/****************************************************************************
 * Name: pthread_spin_destroy
 *
 * Description:
 *   Destroy the spin lock.  A spin lock has no resources to release.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object.
 *
 * Returned Value:
 *   Zero (OK) is always returned.
 *
 ****************************************************************************/

int pthread_spin_destroy(FAR pthread_spinlock_t *lock)
{
  return OK;
}

/****************************************************************************
 * Name: pthread_spin_trylock
 *
 * Description:
 *   Lock the spin lock if no thread holds it.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   Zero if the lock was acquired; EINVAL if 'lock' is not valid; EBUSY if
 *   a thread (also the caller) holds the lock.
 *
 ****************************************************************************/

int pthread_spin_trylock(FAR pthread_spinlock_t *lock)
{
  int32_t expected = 0;

  if (lock == NULL)
    {
      return EINVAL;
    }

  if (!atomic_cmpxchg_acquire(&lock->sp_lock, &expected, 1))
    {
      return EBUSY;
    }

  lock->sp_holder = pthread_self();
  return OK;
}

/****************************************************************************
 * Name: pthread_spin_lock
 *
 * Description:
 *   Lock the spin lock.  The thread spins while another thread holds the
 *   lock, and yields the CPU after SPIN_TRIES tests, so that a holder that
 *   was preempted can run.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   Zero if the lock was acquired; EINVAL if 'lock' is not valid; EDEADLK
 *   if the caller already holds the lock.
 *
 ****************************************************************************/

int pthread_spin_lock(FAR pthread_spinlock_t *lock)
{
  int tries = 0;

  if (lock == NULL)
    {
      return EINVAL;
    }

  if (lock->sp_holder == pthread_self())
    {
      return EDEADLK;
    }

  while (pthread_spin_trylock(lock) != OK)
    {
      /* Wait with plain reads, not with atomic writes */

      while (atomic_read(&lock->sp_lock) != 0)
        {
          if (++tries >= SPIN_TRIES)
            {
              sched_yield();
              tries = 0;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pthread_spin_unlock
 *
 * Description:
 *   Release the spin lock that the caller holds.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   Zero if the lock was released; EINVAL if 'lock' is not valid; EPERM if
 *   the caller does not hold the lock.
 *
 ****************************************************************************/

int pthread_spin_unlock(FAR pthread_spinlock_t *lock)
{
  if (lock == NULL)
    {
      return EINVAL;
    }

  if (atomic_read(&lock->sp_lock) == 0 ||
      lock->sp_holder != pthread_self())
    {
      return EPERM;
    }

  lock->sp_holder = IMPOSSIBLE_THREAD;
  atomic_set_release(&lock->sp_lock, 0);
  return OK;
}
