/****************************************************************************
 * libs/libc/semaphore/sem_post.c
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
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/atomic.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_post
 *
 * Description:
 *   When a task has finished with a semaphore, it will call sem_post().
 *   This function unlocks the semaphore referenced by sem by performing the
 *   semaphore unlock operation on that semaphore.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to nxsem_wait().
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *
 * Returned Value:
 *   This function is a standard, POSIX application interface.  It will
 *   return zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int sem_post(FAR sem_t *sem)
{
  int ret;

  /* Make sure we were supplied with a valid semaphore. */

  if (sem == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  ret = nxsem_post(sem);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nxsem_post
 *
 * Description:
 *   When a kernel thread has finished with a semaphore, it will call
 *   nxsem_post().  This function unlocks the semaphore referenced by sem
 *   by performing the semaphore unlock operation on that semaphore.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to nxsem_wait().
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int nxsem_post(FAR sem_t *sem)
{
  bool fastpath = true;
  bool mutex;

  DEBUGASSERT(sem != NULL);

  mutex = NXSEM_IS_MUTEX(sem);

  /* Disable fast path if priority protection is enabled on the semaphore */

#ifdef CONFIG_PRIORITY_PROTECT
  if ((sem->flags & SEM_PRIO_MASK) == SEM_PRIO_PROTECT)
    {
      fastpath = false;
    }
#endif

  /* Disable fast path on a counting semaphore with priority inheritance */

#ifdef CONFIG_PRIORITY_INHERITANCE
  if (!mutex && (sem->flags & SEM_PRIO_MASK) != SEM_PRIO_NONE)
    {
      fastpath = false;
    }
#endif

  while (fastpath)
    {
      FAR atomic_t *val = mutex ? NXSEM_MHOLDER(sem) : NXSEM_COUNT(sem);
      int32_t old = atomic_read(val);
      int32_t new;

      if (mutex)
        {
          if (NXSEM_MBLOCKING(old))
            {
              break;
            }

          new = NXSEM_NO_MHOLDER;
        }
      else
        {
          if (old < 0)
            {
              break;
            }

          new = old + 1;
        }

      if (atomic_try_cmpxchg_release(val, &old, new))
        {
          return OK;
        }
    }

  return nxsem_post_slow(sem);
}
