/****************************************************************************
 * net/utils/net_lock.c
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

#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>

#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NO_HOLDER (INVALID_PROCESS_ID)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static rmutex_t g_netlock = NXRMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _net_timedwait
 ****************************************************************************/

static int
_net_timedwait(sem_t *sem, bool interruptible, unsigned int timeout)
{
  unsigned int count;
  irqstate_t   flags;
  int          blresult;
  int          ret;

  flags = enter_critical_section(); /* No interrupts */
  sched_lock();                     /* No context switches */

  /* Release the network lock, remembering my count.  net_breaklock will
   * return a negated value if the caller does not hold the network lock.
   */

  blresult = net_breaklock(&count);

  /* Now take the semaphore, waiting if so requested. */

  if (timeout != UINT_MAX)
    {
      /* Wait until we get the lock or until the timeout expires */

      if (interruptible)
        {
          ret = nxsem_tickwait(sem, MSEC2TICK(timeout));
        }
      else
        {
          ret = nxsem_tickwait_uninterruptible(sem, MSEC2TICK(timeout));
        }
    }
  else
    {
      /* Wait as long as necessary to get the lock */

      if (interruptible)
        {
          ret = nxsem_wait(sem);
        }
      else
        {
          ret = nxsem_wait_uninterruptible(sem);
        }
    }

  /* Recover the network lock at the proper count (if we held it before) */

  if (blresult >= 0)
    {
      net_restorelock(count);
    }

  sched_unlock();
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_lock
 *
 * Description:
 *   Take the network lock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failured (probably -ECANCELED).
 *
 ****************************************************************************/

int net_lock(void)
{
  return nxrmutex_lock(&g_netlock);
}

/****************************************************************************
 * Name: net_trylock
 *
 * Description:
 *   Try to take the network lock only when it is currently not locked.
 *   Otherwise, it locks the semaphore.  In either
 *   case, the call returns without blocking.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failured (probably -EAGAIN).
 *
 ****************************************************************************/

int net_trylock(void)
{
  return nxrmutex_trylock(&g_netlock);
}

/****************************************************************************
 * Name: net_unlock
 *
 * Description:
 *   Release the network lock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net_unlock(void)
{
  nxrmutex_unlock(&g_netlock);
}

/****************************************************************************
 * Name: net_breaklock
 *
 * Description:
 *   Break the lock, return information needed to restore re-entrant lock
 *   state.
 *
 ****************************************************************************/

int net_breaklock(FAR unsigned int *count)
{
  DEBUGASSERT(count != NULL);
  return nxrmutex_breaklock(&g_netlock, count);
}

/****************************************************************************
 * Name: net_restorelock
 *
 * Description:
 *   Restore the locked state
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failured (probably -ECANCELED).
 *
 ****************************************************************************/

int net_restorelock(unsigned int count)
{
  return nxrmutex_restorelock(&g_netlock, count);
}

/****************************************************************************
 * Name: net_timedwait
 *
 * Description:
 *   Atomically wait for sem (or a timeout) while temporarily releasing
 *   the lock on the network.
 *
 *   Caution should be utilized.  Because the network lock is relinquished
 *   during the wait, there could be changes in the network state that occur
 *   before the lock is recovered.  Your design should account for this
 *   possibility.
 *
 * Input Parameters:
 *   sem     - A reference to the semaphore to be taken.
 *   timeout - The relative time to wait until a timeout is declared.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int net_timedwait(sem_t *sem, unsigned int timeout)
{
  return _net_timedwait(sem, true, timeout);
}

/****************************************************************************
 * Name: net_lockedwait
 *
 * Description:
 *   Atomically wait for sem while temporarily releasing the network lock.
 *
 *   Caution should be utilized.  Because the network lock is relinquished
 *   during the wait, there could be changes in the network state that occur
 *   before the lock is recovered.  Your design should account for this
 *   possibility.
 *
 * Input Parameters:
 *   sem - A reference to the semaphore to be taken.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int net_lockedwait(sem_t *sem)
{
  return net_timedwait(sem, UINT_MAX);
}

/****************************************************************************
 * Name: net_timedwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of net_timedwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem     - A reference to the semaphore to be taken.
 *   timeout - The relative time to wait until a timeout is declared.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int net_timedwait_uninterruptible(sem_t *sem, unsigned int timeout)
{
  return _net_timedwait(sem, false, timeout);
}

/****************************************************************************
 * Name: net_lockedwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of net_lockedwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem - A reference to the semaphore to be taken.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int net_lockedwait_uninterruptible(sem_t *sem)
{
  return net_timedwait_uninterruptible(sem, UINT_MAX);
}

#ifdef CONFIG_MM_IOB

/****************************************************************************
 * Name: net_timedalloc
 *
 * Description:
 *   Allocate an IOB.  If no IOBs are available, then atomically wait for
 *   for the IOB while temporarily releasing the lock on the network.
 *   This function is wrapped version of nxsem_tickwait(), this wait will
 *   be terminated when the specified timeout expires.
 *
 *   Caution should be utilized.  Because the network lock is relinquished
 *   during the wait, there could be changes in the network state that occur
 *   before the lock is recovered.  Your design should account for this
 *   possibility.
 *
 * Input Parameters:
 *   throttled  - An indication of the IOB allocation is "throttled"
 *   timeout    - The relative time to wait until a timeout is declared.
 *
 * Returned Value:
 *   A pointer to the newly allocated IOB is returned on success.  NULL is
 *   returned on any allocation failure.
 *
 ****************************************************************************/

FAR struct iob_s *net_iobtimedalloc(bool throttled, unsigned int timeout)
{
  FAR struct iob_s *iob;

  iob = iob_tryalloc(throttled);
  if (iob == NULL && timeout != 0)
    {
      unsigned int count;
      int blresult;

      /* There are no buffers available now.  We will have to wait for one to
       * become available. But let's not do that with the network locked.
       */

      blresult = net_breaklock(&count);
      iob      = iob_timedalloc(throttled, timeout);
      if (blresult >= 0)
        {
          net_restorelock(count);
        }
    }

  return iob;
}

/****************************************************************************
 * Name: net_ioballoc
 *
 * Description:
 *   Allocate an IOB.  If no IOBs are available, then atomically wait for
 *   for the IOB while temporarily releasing the lock on the network.
 *
 *   Caution should be utilized.  Because the network lock is relinquished
 *   during the wait, there could be changes in the network state that occur
 *   before the lock is recovered.  Your design should account for this
 *   possibility.
 *
 * Input Parameters:
 *   throttled  - An indication of the IOB allocation is "throttled"
 *
 * Returned Value:
 *   A pointer to the newly allocated IOB is returned on success.  NULL is
 *   returned on any allocation failure.
 *
 ****************************************************************************/

FAR struct iob_s *net_ioballoc(bool throttled)
{
  return net_iobtimedalloc(throttled, UINT_MAX);
}
#endif
