/****************************************************************************
 * libs/libc/misc/lib_rwlock.c
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
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/rwlock.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxrwlock_init
 *
 * Description:
 *   Initialize a rwlock object to it's initial, both write semaphore and
 *   read semaphore was set to 1, the num of readers  was set to 0
 *
 * Input Parameters:
 *   lock  - read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxrwlock_init(FAR rwlock_t *lock)
{
  int ret;

  ret = nxmutex_init(&lock->read);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_init(&lock->write);
  if (ret < 0)
    {
      return ret;
    }

  lock->reads = 0;
  return ret;
}

/****************************************************************************
 * Name: nxrwlock_destroy
 *
 * Description:
 *   rwlock_destroy a rwlock object, both write semaphore and read semaphore
 *   was destroied, the num of readers  was set to 0.
 *
 * Input Parameters:
 *   lock  - read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxrwlock_destroy(FAR rwlock_t *lock)
{
  int ret;

  ret = nxmutex_destroy(&lock->read);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_destroy(&lock->write);
  if (ret < 0)
    {
      return ret;
    }

  lock->reads = 0;
  return ret;
}

/****************************************************************************
 * Name: nxrwlock_read_lock
 *
 * Description:
 *   This function attempts to get the rwlock. If thers's none other write
 *   task hold the write semaphore of rwlock, the task can successful get
 *   rwlock, otherwise the task would be blocked.
 *
 * Input Parameters:
 *   lock - read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

int nxrwlock_read_lock(FAR rwlock_t *lock)
{
  irqstate_t flags;
  int ret;

  /* The following operations must be performed with interrupts
   * disabled because it may be interrupted between rdlock and wrlock
   */

  flags = spin_lock_irqsave(NULL);
  ret = nxmutex_lock(&lock->read);
  if (ret < 0)
    {
      goto errout;
    }

  lock->reads++;

  if (lock->reads == 1)
    {
      ret = nxmutex_lock(&lock->write);
    }

  nxmutex_unlock(&lock->read);
errout:
  spin_unlock_irqrestore(NULL, flags);
  return ret;
}

/****************************************************************************
 * Name: nxrwlock_read_unlock
 *
 * Description:
 *   This function attempts to give the rwlock. If thers's none other read
 *   task hold the rwlock, the task can successful give the rwlock by give
 *   the write semaphore of rwlock.
 *
 * Input Parameters:
 *   lock - read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

int nxrwlock_read_unlock(FAR rwlock_t *lock)
{
  irqstate_t flags;
  int ret;

  /* The following operations must be performed with interrupts
   * disabled because it may be interrupted between rdlock and wrlock
   */

  flags = spin_lock_irqsave(NULL);
  ret = nxmutex_lock(&lock->read);
  if (ret < 0)
    {
      goto errout;
    }

  lock->reads--;

  if (lock->reads == 0)
    {
      ret = nxmutex_unlock(&lock->write);
    }

  nxmutex_unlock(&lock->read);
errout:
  spin_unlock_irqrestore(NULL, flags);
  return ret;
}

/****************************************************************************
 * Name: nxrwlock_write_lock
 *
 * Description:
 *   When a task try to write with a rwlock, it will call rwlock_write_lock.
 *   This function try to get write semaphore of rwlock. If the write
 *   semaphore does already hold by other task, this tasks will be blocked,
 *   which is uninterruptible.
 *
 * Input Parameters:
 *   lock - read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

int nxrwlock_write_lock(FAR rwlock_t *lock)
{
  return nxmutex_lock(&lock->write);
}

/****************************************************************************
 * Name: nxrwlock_write_unlock
 *
 * Description:
 *   When a task has finished write with a rwlock, it will call
 *   rwlock_write_unlock(). This function unlocks the write semaphore
 *   referenced by rwlock by performing the semaphore unlock operation on
 *   write semaphore of rwlock.
 *
 * Input Parameters:
 *   lock - read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

int nxrwlock_write_unlock(FAR rwlock_t *lock)
{
  return nxmutex_unlock(&lock->write);
}
