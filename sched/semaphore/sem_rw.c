/****************************************************************************
 * sched/semaphore/sem_rw.c
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

#include <nuttx/rwsem.h>
#include <nuttx/irq.h>
#include <assert.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void up_wait(FAR rw_semaphore_t *rwsem)
{
  int i;

  for (i = 0; i < rwsem->waiter; i++)
    {
      /* If there are some waiter for unlock, then post the lock wait queue.
       */

      nxsem_post(&rwsem->waiting);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: down_read_trylock
 *
 * Description:
 *   Acquire a read lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 * Returned Value:
 *   Return 1 if successful, 0 if failed
 *
 ****************************************************************************/

int down_read_trylock(FAR rw_semaphore_t *rwsem)
{
  irqstate_t flags = spin_lock_irqsave(&rwsem->protected);

  if (rwsem->writer > 0)
    {
      spin_unlock_irqrestore(&rwsem->protected, flags);
      return 0;
    }

  /* In a scenario where there is no write lock, we just need to make the
   * read base +1.
   */

  rwsem->reader++;

  spin_unlock_irqrestore(&rwsem->protected, flags);

  return 1;
}

/****************************************************************************
 * Name: down_read
 *
 * Description:
 *   Acquire a read lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void down_read(FAR rw_semaphore_t *rwsem)
{
  /* we have to check if there is a write-lock scenario, if there is then we
   * block and wait for the write-lock to be unlocked.
   */

  irqstate_t flags = spin_lock_irqsave(&rwsem->protected);

  while (rwsem->writer > 0)
    {
      rwsem->waiter++;
      spin_unlock_irqrestore(&rwsem->protected, flags);
      nxsem_wait(&rwsem->waiting);
      flags = spin_lock_irqsave(&rwsem->protected);
      rwsem->waiter--;
    }

  /* In a scenario where there is no write lock, we just need to make the
   * read base +1.
   */

  rwsem->reader++;

  spin_unlock_irqrestore(&rwsem->protected, flags);
}

/****************************************************************************
 * Name: up_read
 *
 * Description:
 *   Unlock a read lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void up_read(FAR rw_semaphore_t *rwsem)
{
  irqstate_t flags = spin_lock_irqsave(&rwsem->protected);

  DEBUGASSERT(rwsem->reader > 0);

  rwsem->reader--;

  if (rwsem->waiter > 0)
    {
      up_wait(rwsem);
    }

  spin_unlock_irqrestore(&rwsem->protected, flags);
}

/****************************************************************************
 * Name: down_write_trylock
 *
 * Description:
 *   Acquire a write lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 * Returned Value:
 *   Return 1 if successful, 0 if failed
 *
 ****************************************************************************/

int down_write_trylock(FAR rw_semaphore_t *rwsem)
{
  irqstate_t flags = spin_lock_irqsave(&rwsem->protected);

  if (rwsem->writer > 0 || rwsem->reader > 0)
    {
      spin_unlock_irqrestore(&rwsem->protected, flags);
      return 0;
    }

  /* The check passes, then we just need the writer reference + 1 */

  rwsem->writer++;

  spin_unlock_irqrestore(&rwsem->protected, flags);

  return 1;
}

/****************************************************************************
 * Name: down_write
 *
 * Description:
 *   Acquire a write lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void down_write(FAR rw_semaphore_t *rwsem)
{
  irqstate_t flags = spin_lock_irqsave(&rwsem->protected);

  while (rwsem->reader > 0 || rwsem->writer > 0)
    {
      rwsem->waiter++;
      spin_unlock_irqrestore(&rwsem->protected, flags);
      nxsem_wait(&rwsem->waiting);
      flags = spin_lock_irqsave(&rwsem->protected);
      rwsem->waiter--;
    }

  /* The check passes, then we just need the writer reference + 1 */

  rwsem->writer++;

  spin_unlock_irqrestore(&rwsem->protected, flags);
}

/****************************************************************************
 * Name: up_write
 *
 * Description:
 *   Unlock a write lock on a read-write-lock object.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void up_write(FAR rw_semaphore_t *rwsem)
{
  irqstate_t flags = spin_lock_irqsave(&rwsem->protected);

  DEBUGASSERT(rwsem->writer > 0);

  rwsem->writer--;

  up_wait(rwsem);

  spin_unlock_irqrestore(&rwsem->protected, flags);
}

/****************************************************************************
 * Name: init_rwsem
 *
 * Description:
 *   Initialize a read-write-lock object, setting its initial state.
 *
 * Input Parameters:
 *   rwsem  - Pointer to the read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy: Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int init_rwsem(FAR rw_semaphore_t *rwsem)
{
  int ret;

  /* Initialize structure information */

  spin_lock_init(&rwsem->protected);

  ret = nxsem_init(&rwsem->waiting, 0, 0);
  if (ret < 0)
    {
      return ret;
    }

  rwsem->reader = 0;
  rwsem->writer = 0;
  rwsem->waiter = 0;

  return OK;
}

/****************************************************************************
 * Name: destroy_rwsem
 *
 * Description:
 *   Destroy a read-write-lock object, freeing any resources associated with
 *   it.
 *
 * Input Parameters:
 *   rwsem - Pointer to the read-write-lock descriptor.
 *
 ****************************************************************************/

void destroy_rwsem(FAR rw_semaphore_t *rwsem)
{
  /* Need to check if there is still an unlocked or waiting state */

  DEBUGASSERT(rwsem->waiter == 0 && rwsem->reader == 0 &&
              rwsem->writer == 0);

  nxsem_destroy(&rwsem->waiting);
}
