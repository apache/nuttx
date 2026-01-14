/****************************************************************************
 * libs/libc/semaphore/sem_trywait.c
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
#include <sched.h>

#include <nuttx/sched.h>
#include <nuttx/init.h>
#include <nuttx/semaphore.h>
#include <nuttx/atomic.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_trywait
 *
 * Description:
 *   This function locks the specified semaphore only if the semaphore is
 *   currently not locked.  In either case, the call returns without
 *   blocking.
 *
 * Input Parameters:
 *   sem - the semaphore descriptor
 *
 * Returned Value:
 *   Zero (OK) on success or -1 (ERROR) if unsuccessful. If this function
 *   returns -1(ERROR), then the cause of the failure will be reported in
 *   errno variable as:
 *
 *     EINVAL - Invalid attempt to get the semaphore
 *     EAGAIN - The semaphore is not available.
 *
 ****************************************************************************/

int sem_trywait(FAR sem_t *sem)
{
  int ret;

  if (sem == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Let nxsem_trywait do the real work */

  ret = nxsem_trywait(sem);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nxsem_trywait
 *
 * Description:
 *   This function locks the specified semaphore only if the semaphore is
 *   currently not locked.  In either case, the call returns without
 *   blocking.
 *
 * Input Parameters:
 *   sem - the semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     - EINVAL - Invalid attempt to get the semaphore
 *     - EAGAIN - The semaphore is not available.
 *
 ****************************************************************************/

int nxsem_trywait(FAR sem_t *sem)
{
  bool fastpath = true;
  bool mutex;

  DEBUGASSERT(sem != NULL);

  /* This API should not be called from the idleloop or interrupt */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  DEBUGASSERT(!OSINIT_IDLELOOP() || !sched_idletask() ||
              up_interrupt_context());
#endif

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
          if (old != NXSEM_NO_MHOLDER)
            {
              return -EAGAIN;
            }

          new = _SCHED_GETTID();
        }
      else
        {
          if (old < 1)
            {
              return -EAGAIN;
            }

          new = old - 1;
        }

      if (atomic_try_cmpxchg_acquire(val, &old, new))
        {
          return OK;
        }
    }

  return nxsem_trywait_slow(sem);
}
