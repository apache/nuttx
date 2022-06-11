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

static sem_t        g_netlock = SEM_INITIALIZER(1);
static pid_t        g_holder  = NO_HOLDER;
static unsigned int g_count;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _net_takesem
 *
 * Description:
 *   Take the semaphore, waiting indefinitely.
 *   REVISIT: Should this return if -EINTR?
 *
 ****************************************************************************/

static int _net_takesem(void)
{
  return nxsem_wait_uninterruptible(&g_netlock);
}

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
  pid_t me = getpid();
  int ret = OK;

  /* Does this thread already hold the semaphore? */

  if (g_holder == me)
    {
      /* Yes.. just increment the reference count */

      g_count++;
    }
  else
    {
      /* No.. take the semaphore (perhaps waiting) */

      ret = _net_takesem();
      if (ret >= 0)
        {
          /* Now this thread holds the semaphore */

          g_holder = me;
          g_count  = 1;
        }
    }

  return ret;
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
  pid_t me = getpid();
  int ret = OK;

  /* Does this thread already hold the semaphore? */

  if (g_holder == me)
    {
      /* Yes.. just increment the reference count */

      g_count++;
    }
  else
    {
      ret = nxsem_trywait(&g_netlock);
      if (ret >= 0)
        {
          /* Now this thread holds the semaphore */

          g_holder = me;
          g_count  = 1;
        }
    }

  return ret;
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
  DEBUGASSERT(g_holder == getpid() && g_count > 0);

  /* If the count would go to zero, then release the semaphore */

  if (g_count == 1)
    {
      /* We no longer hold the semaphore */

      g_holder = NO_HOLDER;
      g_count  = 0;
      nxsem_post(&g_netlock);
    }
  else
    {
      /* We still hold the semaphore. Just decrement the count */

      g_count--;
    }
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
  irqstate_t flags;
  pid_t me = getpid();
  int ret = -EPERM;

  DEBUGASSERT(count != NULL);

  flags = enter_critical_section(); /* No interrupts */
  if (g_holder == me)
    {
      /* Return the lock setting */

      *count   = g_count;

      /* Release the network lock  */

      g_holder = NO_HOLDER;
      g_count  = 0;

      nxsem_post(&g_netlock);
      ret      = OK;
    }

  leave_critical_section(flags);
  return ret;
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
  pid_t me = getpid();
  int ret;

  DEBUGASSERT(g_holder != me);

  /* Recover the network lock at the proper count */

  ret = _net_takesem();
  if (ret >= 0)
    {
      g_holder = me;
      g_count  = count;
    }

  return ret;
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
 *   consumerid - id representing who is consuming the IOB
 *
 * Returned Value:
 *   A pointer to the newly allocated IOB is returned on success.  NULL is
 *   returned on any allocation failure.
 *
 ****************************************************************************/

FAR struct iob_s *net_iobtimedalloc(bool throttled, unsigned int timeout,
                                    enum iob_user_e consumerid)
{
  FAR struct iob_s *iob;

  iob = iob_tryalloc(throttled, consumerid);
  if (iob == NULL && timeout != 0)
    {
      unsigned int count;
      int blresult;

      /* There are no buffers available now.  We will have to wait for one to
       * become available. But let's not do that with the network locked.
       */

      blresult = net_breaklock(&count);
      iob      = iob_timedalloc(throttled, timeout, consumerid);
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
 *   consumerid - id representing who is consuming the IOB
 *
 * Returned Value:
 *   A pointer to the newly allocated IOB is returned on success.  NULL is
 *   returned on any allocation failure.
 *
 ****************************************************************************/

FAR struct iob_s *net_ioballoc(bool throttled, enum iob_user_e consumerid)
{
  return net_iobtimedalloc(throttled, UINT_MAX, consumerid);
}
#endif
