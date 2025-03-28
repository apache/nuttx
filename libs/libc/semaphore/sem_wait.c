/****************************************************************************
 * libs/libc/semaphore/sem_wait.c
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

#include <nuttx/init.h>
#include <nuttx/cancelpt.h>
#include <nuttx/semaphore.h>
#include <nuttx/atomic.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_wait
 *
 * Description:
 *   This function attempts to lock the semaphore referenced by 'sem'.  If
 *   the semaphore value is (<=) zero, then the calling task will not return
 *   until it successfully acquires the lock.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This function is a standard, POSIX application interface.  It returns
 *   zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.  Possible errno values include:
 *
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of a signal.
 *
 ****************************************************************************/

int sem_wait(FAR sem_t *sem)
{
  int errcode;
  int ret;

  if (sem == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* sem_wait() is a cancellation point */

  if (enter_cancellation_point())
    {
#ifdef CONFIG_CANCELLATION_POINTS
      /* If there is a pending cancellation, then do not perform
       * the wait.  Exit now with ECANCELED.
       */

      errcode = ECANCELED;
      goto errout_with_cancelpt;
#endif
    }

  /* Let nxsem_wait() do the real work */

  ret = nxsem_wait(sem);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_cancelpt;
    }

  leave_cancellation_point();
  return OK;

errout_with_cancelpt:
  set_errno(errcode);
  leave_cancellation_point();
  return ERROR;
}

/****************************************************************************
 * Name: nxsem_wait
 *
 * Description:
 *   This function attempts to lock the semaphore referenced by 'sem'.  If
 *   the semaphore value is (<=) zero, then the calling task will not return
 *   until it successfully acquires the lock.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of a signal.
 *
 ****************************************************************************/

int nxsem_wait(FAR sem_t *sem)
{
  DEBUGASSERT(sem != NULL);

  /* This API should not be called from the idleloop or interrupt */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  DEBUGASSERT(!OSINIT_IDLELOOP() || !sched_idletask() ||
              up_interrupt_context());
#endif

  /* We don't do atomic fast path in case of LIBC_ARCH_ATOMIC because that
   * uses spinlocks, which can't be called from userspace. Also in the kernel
   * taking the slow path directly is faster than locking first in here
   */

#ifndef CONFIG_LIBC_ARCH_ATOMIC

  if ((sem->flags & SEM_TYPE_MUTEX)
#  if defined(CONFIG_PRIORITY_PROTECT) || defined(CONFIG_PRIORITY_INHERITANCE)
      && (sem->flags & SEM_PRIO_MASK) == SEM_PRIO_NONE
#  endif
      )
    {
      int32_t old = 1;
      if (atomic_try_cmpxchg_acquire(NXSEM_COUNT(sem), &old, 0))
        {
          return OK;
        }
    }

#endif

  return nxsem_wait_slow(sem);
}
