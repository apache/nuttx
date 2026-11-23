/****************************************************************************
 * sched/semaphore/sem_holder.c
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

#include <sched.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/mm/kmap.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

#ifdef CONFIG_PRIORITY_INHERITANCE

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_allocholder
 *
 * Description:
 *   Priority inheritance is supported only for mutexes.  A mutex is held by
 *   exactly one thread, so the built-in holder slot embedded in the
 *   semaphore is always sufficient.
 *
 ****************************************************************************/

static inline FAR struct semholder_s *
nxsem_allocholder(FAR sem_t *sem, FAR struct tcb_s *htcb)
{
  FAR struct semholder_s *pholder;

  if (sem->holder.htcb == NULL)
    {
      pholder = &sem->holder;
    }
  else
    {
      serr("ERROR: Mutex already has a holder\n");
      PANIC();
    }

#ifdef CONFIG_MM_KMAP
  sem = kmm_map_user(this_task(), sem, sizeof(*sem));
#endif

  pholder->sem  = sem;
  pholder->htcb = htcb;

  /* Put it into the task's list */

  pholder->tlink = htcb->holdsem;
  htcb->holdsem  = pholder;

  return pholder;
}

/****************************************************************************
 * Name: nxsem_findholder
 *
 * NOTE: htcb may be used only as a look-up key.  It certain cases, the task
 * may have exited and htcb may refer to a stale memory.  It must not be
 * dereferenced.
 *
 ****************************************************************************/

static FAR struct semholder_s *
nxsem_findholder(FAR sem_t *sem, FAR struct tcb_s *htcb)
{
  FAR struct semholder_s *pholder = &sem->holder;

  if (pholder->htcb == htcb)
    {
      /* Got it! */

      return pholder;
    }

  /* The holder does not appear in the list */

  return NULL;
}

/****************************************************************************
 * Name: nxsem_findorallocateholder
 ****************************************************************************/

static inline FAR struct semholder_s *
nxsem_findorallocateholder(FAR sem_t *sem, FAR struct tcb_s *htcb)
{
  FAR struct semholder_s *pholder = nxsem_findholder(sem, htcb);

  if (pholder == NULL)
    {
      pholder = nxsem_allocholder(sem, htcb);
    }

  return pholder;
}

/****************************************************************************
 * Name: nxsem_freeholder
 ****************************************************************************/

static inline void nxsem_freeholder(FAR sem_t *sem,
                                    FAR struct semholder_s *pholder)
{
  FAR struct semholder_s * FAR *curr;

  /* Remove the holder from the task's list */

  for (curr = &pholder->htcb->holdsem;
       *curr != NULL;
       curr = &(*curr)->tlink)
    {
      if (*curr == pholder)
        {
          *curr = pholder->tlink;
          break;
        }
    }

#ifdef CONFIG_MM_KMAP
  kmm_unmap(pholder->sem);
#endif

  /* Release the holder */

  pholder->tlink = NULL;
  pholder->sem   = NULL;
  pholder->htcb  = NULL;
}

/****************************************************************************
 * Name: nxsem_boostholderprio
 ****************************************************************************/

static void nxsem_boostholderprio(FAR struct tcb_s *htcb,
                                  FAR struct tcb_s *rtcb)
{
  /* If the priority of the thread that is waiting for the mutex is less than
   * or equal to the priority of the thread holding it, then do nothing
   * because the holder is already running at a sufficient priority.
   */

  if (rtcb && htcb && rtcb->sched_priority > htcb->sched_priority)
    {
      /* Raise the priority of the holder of the semaphore.  This
       * cannot cause a context switch because we have preemption
       * disabled.  The task will be marked "pending" and the switch
       * will occur during up_contex_switch() processing.
       */

      nxsched_set_priority(htcb, rtcb->sched_priority);
    }
}

/****************************************************************************
 * Name: nxsem_dumpholder
 ****************************************************************************/

#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_SEM_PHDEBUG)
static void nxsem_dumpholder(FAR sem_t *sem)
{
  FAR struct semholder_s *pholder = &sem->holder;

  if (pholder->htcb != NULL)
    {
      _info("  %p: %p %p %p\n",
            pholder, pholder->tlink, pholder->sem, pholder->htcb);
    }
}
#endif

/****************************************************************************
 * Name: nxsem_restore_priority
 ****************************************************************************/

static void nxsem_restore_priority(FAR struct tcb_s *htcb)
{
  int hpriority;

  /* We attempt to restore thread priority to its base priority.  If
   * there is any thread with the higher priority waiting for the
   * semaphore held by htcb then this value will be overwritten.
   */

  hpriority = htcb->boost_priority > htcb->base_priority ?
              htcb->boost_priority : htcb->base_priority;

  /* Was the priority of the holder thread boosted? If so, then drop its
   * priority back to the correct level.  What is the correct level?
   */

  if (htcb->sched_priority != hpriority)
    {
      FAR struct semholder_s *pholder;

      /* Try to find the highest priority across all the threads that are
       * waiting for any semaphore held by htcb.
       */

      for (pholder = htcb->holdsem; pholder != NULL;
           pholder = pholder->tlink)
        {
          FAR struct tcb_s *stcb;

          stcb = (FAR struct tcb_s *)dq_peek(SEM_WAITLIST(pholder->sem));

          if (stcb != NULL && stcb->sched_priority > hpriority)
            {
              hpriority = stcb->sched_priority;
            }
        }

      /* Apply the selected priority to the thread (hopefully back to the
       * threads base_priority).
       */

      nxsched_set_priority(htcb, hpriority);
    }
}

/****************************************************************************
 * Name: nxsem_restoreholderprio
 ****************************************************************************/

static void nxsem_restoreholderprio(FAR sem_t *sem)
{
  FAR struct tcb_s *htcb = sem->holder.htcb;

  if (htcb != NULL)
    {
      /* A mutex is held by only one thread, so release the holder before
       * reprioritizing causes a context switch.
       */

      nxsem_freeholder(sem, &sem->holder);
      nxsem_restore_priority(htcb);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_destroyholder
 *
 * Description:
 *   Called from nxsem_destroy() to handle any holder of a semaphore
 *   when it is destroyed.
 *
 * Input Parameters:
 *   sem - A reference to the semaphore being destroyed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_destroyholder(FAR sem_t *sem)
{
  irqstate_t flags = enter_critical_section();

  /* It might be an error if a semaphore is destroyed while there is a
   * holder of the semaphore (except perhaps the thread that release the
   * semaphore itself).  We actually have to assume that the caller knows
   * what it is doing because could have killed another thread that is the
   * actual holder of the semaphore.
   *
   * It is also a standard practice to destroy the semaphore while the
   * caller holds it.  Of course, the caller MUST assure that there are no
   * other holders of the semaphore in this case.  This occurs, for example,
   * when a driver is unlink'ed and the driver instance must be destroyed.
   *
   * Therefore, we cannot make any assumptions about the state of the
   * semaphore or the state of any of the holder threads.  So just recover
   * any stranded holder and hope the task knows what it is doing.
   */

  DEBUGASSERT(sem->holder.htcb == NULL || sem->holder.htcb == this_task());

  if (sem->holder.htcb != NULL)
    {
      nxsem_freeholder(sem, &sem->holder);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nxsem_add_holder_tcb
 *
 * Description:
 *   Called from nxsem_wait() when the calling thread obtains the mutex;
 *   Called from sem_post() when the waiting thread obtains the mutex.
 *
 * Input Parameters:
 *   htcb - TCB of the thread that just obtained the mutex
 *   sem  - A reference to the mutex
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

void nxsem_add_holder_tcb(FAR struct tcb_s *htcb, FAR sem_t *sem)
{
  uint8_t prioinherit = sem->flags & SEM_PRIO_MASK;

  /* If priority inheritance is disabled for this semaphore or the holder is
   * the IDLE thread, then do not add the holder.
   */

  if (!is_idle_task(htcb) && prioinherit == SEM_PRIO_INHERIT)
    {
      /* Find or allocate a container for this holder */

      nxsem_findorallocateholder(sem, htcb);
    }
}

/****************************************************************************
 * Name: void nxsem_boost_priority(sem_t *sem)
 *
 * Description:
 *   Boost the priority of the thread holding the mutex if it is lower than
 *   the priority of the current thread that is waiting for the mutex.
 *
 * Input Parameters:
 *   sem - A reference to the mutex
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_boost_priority(FAR sem_t *sem)
{
  FAR struct tcb_s *rtcb = this_task();

  nxsem_boostholderprio(sem->holder.htcb, rtcb);
}

/****************************************************************************
 * Name: nxsem_release_holder
 *
 * Description:
 *   Called from sem_post() after a thread releases the mutex.
 *
 * Input Parameters:
 *   sem - A reference to the semaphore being posted
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_release_holder(FAR sem_t *sem)
{
  FAR struct tcb_s *rtcb = this_task();
  uint8_t prioinherit = sem->flags & SEM_PRIO_MASK;

  /* If priority inheritance is disabled for this semaphore or the running
   * task is the IDLE thread, then there is no holder to release.
   */

  if (!is_idle_task(rtcb) && prioinherit == SEM_PRIO_INHERIT)
    {
      FAR struct semholder_s *pholder = &sem->holder;

      DEBUGASSERT(!up_interrupt_context());

      if (pholder->htcb != NULL)
        {
          DEBUGASSERT(pholder->htcb == rtcb);
          nxsem_freeholder(sem, pholder);
        }
    }
}

/****************************************************************************
 * Name: nxsem_restore_baseprio
 *
 * Description:
 *   This function is called after the current running task releases the
 *   mutex.  It will check if we need to drop the priority of the thread
 *   holding the mutex.  Its priority could have been boosted while it held
 *   the mutex.
 *
 * Input Parameters:
 *   stcb - The TCB of the task that was just started (if any).  If the
 *     post action caused a thread to be given the mutex, then stcb is that
 *     TCB.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The scheduler is locked.
 *
 ****************************************************************************/

void nxsem_restore_baseprio(FAR struct tcb_s *stcb)
{
  DEBUGASSERT(!up_interrupt_context());

  /* The mutex is held by a single thread.  When it is posted and handed off
   * to a new owner (stcb), the new owner is already the highest priority
   * waiter (the wait queue is priority-based), so no adjustment is needed
   * for it.  Only restore the priority of the previous owner (the currently
   * running task) that may have been boosted while it held the mutex.
   */

  if (stcb != NULL)
    {
      nxsem_restore_priority(this_task());
    }
}

/****************************************************************************
 * Name: nxsem_canceled
 *
 * Description:
 *   Called from nxsem_wait_irq() after a thread that was waiting for a
 *   mutex was awakened because of a signal and the wait has been canceled.
 *   This function restores the correct thread priority of the holder of
 *   the mutex.
 *
 * Input Parameters:
 *   sem  - A reference to the semaphore no longer being waited for
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_canceled(FAR sem_t *sem)
{
  /* Check our assumptions */

  DEBUGASSERT(NXSEM_IS_MUTEX(sem) || atomic_read(NXSEM_COUNT(sem)) <= 0);
  DEBUGASSERT(!NXSEM_IS_MUTEX(sem) ||
              NXSEM_MACQUIRED(atomic_read(NXSEM_MHOLDER(sem))));

  /* Adjust the priority of the holder as necessary */

  nxsem_restoreholderprio(sem);
}

/****************************************************************************
 * Name: sem_enumholders
 *
 * Description:
 *   Show information about the thread currently holding this semaphore
 *
 * Input Parameters:
 *   sem - A reference to the semaphore
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_SEM_PHDEBUG)
void sem_enumholders(FAR sem_t *sem)
{
#ifdef CONFIG_DEBUG_INFO
  nxsem_dumpholder(sem);
#endif
}
#endif

/****************************************************************************
 * Name: nxsem_release_all
 *
 * Description:
 *   Release all mutex holders for the task.  Priority inheritance is
 *   supported only for mutexes, so only mutexes are ever recorded as being
 *   held by the task.
 *
 * Input Parameters:
 *   htcb - TCB of the task
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_release_all(FAR struct tcb_s *htcb)
{
  FAR struct semholder_s *pholder;

  while ((pholder = htcb->holdsem) != NULL)
    {
      nxsem_freeholder(pholder->sem, pholder);
    }
}

#endif /* CONFIG_PRIORITY_INHERITANCE */
