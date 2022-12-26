/****************************************************************************
 * sched/semaphore/sem_holder.c
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
#include <debug.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

#ifdef CONFIG_PRIORITY_INHERITANCE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SEM_PREALLOCHOLDERS
#  define CONFIG_SEM_PREALLOCHOLDERS 0
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

typedef int (*holderhandler_t)(FAR struct semholder_s *pholder,
                               FAR sem_t *sem, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Preallocated holder structures */

#if CONFIG_SEM_PREALLOCHOLDERS > 0
static struct semholder_s g_holderalloc[CONFIG_SEM_PREALLOCHOLDERS];
static FAR struct semholder_s *g_freeholders;
#endif

/****************************************************************************
 * Name: nxsem_allocholder
 ****************************************************************************/

static inline FAR struct semholder_s *
nxsem_allocholder(FAR sem_t *sem, FAR struct tcb_s *htcb)
{
  FAR struct semholder_s *pholder;

  /* Check if the "built-in" holder is being used.  We have this built-in
   * holder to optimize for the simplest case where semaphores are only
   * used to implement mutexes.
   */

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  pholder = g_freeholders;
  if (pholder != NULL)
    {
      /* Remove the holder from the free list and
       * put it into the semaphore's holder list
       */

      g_freeholders    = pholder->flink;
      pholder->flink   = sem->hhead;
      sem->hhead       = pholder;
    }
#else
  if (sem->holder[0].htcb == NULL)
    {
      pholder          = &sem->holder[0];
    }
  else if (sem->holder[1].htcb == NULL)
    {
      pholder          = &sem->holder[1];
    }
#endif
  else
    {
      serr("ERROR: Insufficient pre-allocated holders\n");
      pholder          = NULL;
      DEBUGPANIC();
    }

  if (pholder != NULL)
    {
      pholder->sem    = sem;
      pholder->htcb   = htcb;
      pholder->counts = 0;

      /* Put it into the task's list */

      pholder->tlink  = htcb->holdsem;
      htcb->holdsem   = pholder;
    }

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
  FAR struct semholder_s *pholder;

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  /* Try to find the holder in the list of holders associated with this
   * semaphore
   */

  for (pholder = sem->hhead; pholder != NULL; pholder = pholder->flink)
    {
      if (pholder->htcb == htcb)
        {
          /* Got it! */

          return pholder;
        }
    }
#else
  int i;

  /* We have two hard-allocated holder structures in sem_t */

  for (i = 0; i < 2; i++)
    {
      pholder = &sem->holder[i];
      if (pholder->htcb == htcb)
        {
          /* Got it! */

          return pholder;
        }
    }
#endif

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
  if (!pholder)
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

  /* Release the holder and counts */

  pholder->tlink  = NULL;
  pholder->sem    = NULL;
  pholder->htcb   = NULL;
  pholder->counts = 0;

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  /* Remove the holder from the semaphore's list */

  for (curr = &sem->hhead;
       *curr != NULL;
       curr = &(*curr)->flink)
    {
      if (*curr == pholder)
        {
          *curr = pholder->flink;
          break;
        }
    }

  /* And put it in the free list */

  pholder->flink = g_freeholders;
  g_freeholders  = pholder;
#endif
}

/****************************************************************************
 * Name: nxsem_freecount0holder
 ****************************************************************************/

static int nxsem_freecount0holder(FAR struct semholder_s *pholder,
                                  FAR sem_t *sem, FAR void *arg)
{
  /* When no more counts are held, remove the holder from the list.  The
   * count was decremented in nxsem_release_holder.
   */

  if (pholder->counts <= 0)
    {
      nxsem_freeholder(sem, pholder);
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: nxsem_foreachholder
 ****************************************************************************/

static int nxsem_foreachholder(FAR sem_t *sem, holderhandler_t handler,
                               FAR void *arg)
{
  FAR struct semholder_s *pholder;
  int ret = 0;

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  FAR struct semholder_s *next;

  for (pholder = sem->hhead; pholder && ret == 0; pholder = next)
    {
      /* In case this holder gets deleted */

      next = pholder->flink;

      DEBUGASSERT(pholder->htcb != NULL);

      /* Call the handler */

      ret = handler(pholder, sem, arg);
    }
#else
  int i;

  /* We have two hard-allocated holder structures in sem_t */

  for (i = 0; i < 2 && ret == 0; i++)
    {
      pholder = &sem->holder[i];

      /* The hard-allocated containers may hold a NULL holder */

      if (pholder->htcb != NULL)
        {
          /* Call the handler */

          ret = handler(pholder, sem, arg);
        }
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nxsem_recoverholders
 ****************************************************************************/

static int nxsem_recoverholders(FAR struct semholder_s *pholder,
                                FAR sem_t *sem, FAR void *arg)
{
  nxsem_freeholder(sem, pholder);
  return 0;
}

/****************************************************************************
 * Name: nxsem_boostholderprio
 ****************************************************************************/

static int nxsem_boostholderprio(FAR struct semholder_s *pholder,
                                 FAR sem_t *sem, FAR void *arg)
{
  FAR struct tcb_s *htcb = pholder->htcb;
  FAR struct tcb_s *rtcb = (FAR struct tcb_s *)arg;

  /* If the priority of the thread that is waiting for a count is less than
   * or equal to the priority of the thread holding a count, then do nothing
   * because the thread is already running at a sufficient priority.
   */

  if (rtcb->sched_priority > htcb->sched_priority)
    {
      /* Raise the priority of the holder of the semaphore.  This
       * cannot cause a context switch because we have preemption
       * disabled.  The task will be marked "pending" and the switch
       * will occur during up_contex_switch() processing.
       */

      nxsched_set_priority(htcb, rtcb->sched_priority);
    }

  return 0;
}

/****************************************************************************
 * Name: nxsem_verifyholder
 ****************************************************************************/

#ifdef CONFIG_DEBUG_ASSERTIONS
static int nxsem_verifyholder(FAR struct semholder_s *pholder,
                              FAR sem_t *sem, FAR void *arg)
{
  /* Need to revisit this, but these assumptions seem to be untrue -- OR
   * there is a bug???
   */

#if 0
  FAR struct tcb_s *htcb = pholder->htcb;

  /* Called after a semaphore has been released (incremented), the semaphore
   * could be non-negative, and there is no thread waiting for the count.
   * In this case, the priority of the holder should not be boosted.
   */

  DEBUGASSERT(htcb->sched_priority == htcb->base_priority);
#endif

  return 0;
}
#endif

/****************************************************************************
 * Name: nxsem_dumpholder
 ****************************************************************************/

#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_SEM_PHDEBUG)
static int nxsem_dumpholder(FAR struct semholder_s *pholder, FAR sem_t *sem,
                            FAR void *arg)
{
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  _info("  %08x: %08x %08x %08x %08x %04x\n",
        pholder, pholder->flink,
#else
  _info("  %08x: %08x %08x %08x %04x\n",
        pholder,
#endif
        pholder->tlink, pholder->sem, pholder->htcb, pholder->counts);
  return 0;
}
#endif

/****************************************************************************
 * Name: nxsem_restoreholderprio
 ****************************************************************************/

static int nxsem_restoreholderprio(FAR struct semholder_s *pholder,
                                   FAR sem_t *sem, FAR void *arg)
{
  FAR struct tcb_s *htcb = pholder->htcb;
  int hpriority;

  /* Release the holder if all counts have been given up
   * before reprioritizing causes a context switch.
   */

  if (pholder->counts <= 0)
    {
      nxsem_freeholder(sem, pholder);
    }

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

  return 0;
}

/****************************************************************************
 * Name: nxsem_restoreholderprio_others
 *
 * Description:
 *   Reprioritize all holders except the currently executing task
 *
 ****************************************************************************/

static int nxsem_restoreholderprio_others(FAR struct semholder_s *pholder,
                                          FAR sem_t *sem, FAR void *arg)
{
  FAR struct tcb_s *rtcb = this_task();
  if (pholder->htcb != rtcb)
    {
      return nxsem_restoreholderprio(pholder, sem, arg);
    }

  return 0;
}

/****************************************************************************
 * Name: nxsem_restoreholderprio_self
 *
 * Description:
 *   Reprioritize only the currently executing task
 *
 ****************************************************************************/

static int nxsem_restoreholderprio_self(FAR struct semholder_s *pholder,
                                        FAR sem_t *sem, FAR void *arg)
{
  FAR struct tcb_s *rtcb = this_task();

  if (pholder->htcb == rtcb)
    {
      /* The running task has given up a count on the semaphore */

      nxsem_restoreholderprio(pholder, sem, arg);
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: nxsem_restore_baseprio_irq
 *
 * Description:
 *   This function is called after an interrupt handler posts a count on
 *   the semaphore.  It will check if we need to drop the priority of any
 *   threads holding a count on the semaphore.  Their priority could have
 *   been boosted while they held the count.
 *
 * Input Parameters:
 *   stcb - The TCB of the task that was just started (if any).  If the
 *     post action caused a count to be given to another thread, then stcb
 *     is the TCB that received the count.  Note, just because stcb received
 *     the count, it does not mean that it is higher priority than other
 *     threads.
 *   sem - A reference to the semaphore being posted.
 *     - If the semaphore count is <0 then there are still threads waiting
 *       for a count.  stcb should be non-null and will be higher priority
 *       than all of the other threads still waiting.
 *     - If it is ==0 then stcb refers to the thread that got the last count;
 *       no other threads are waiting.
 *     - If it is >0 then there should be no threads waiting for counts and
 *       stcb should be null.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The scheduler is locked.
 *
 ****************************************************************************/

static inline void nxsem_restore_baseprio_irq(FAR struct tcb_s *stcb,
                                              FAR sem_t *sem)
{
  /* Drop the priority of all holder threads */

  nxsem_foreachholder(sem, nxsem_restoreholderprio, stcb);
}

/****************************************************************************
 * Name: nxsem_restore_baseprio_task
 *
 * Description:
 *   This function is called after the current running task releases a
 *   count on the semaphore.  It will check if we need to drop the priority
 *   of any threads holding a count on the semaphore.  Their priority could
 *   have been boosted while they held the count.
 *
 * Input Parameters:
 *   stcb - The TCB of the task that was just started (if any).  If the
 *     post action caused a count to be given to another thread, then stcb
 *     is the TCB that received the count.  Note, just because stcb received
 *     the count, it does not mean that it is higher priority than other
 *     threads.
 *   sem - A reference to the semaphore being posted.
 *     - If the semaphore count is <0 then there are still threads waiting
 *       for a count.  stcb should be non-null and will be higher priority
 *       than all of the other threads still waiting.
 *     - If it is ==0 then stcb refers to the thread that got the last count;
 *       no other threads are waiting.
 *     - If it is >0 then there should be no threads waiting for counts and
 *       stcb should be null.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The scheduler is locked.
 *
 ****************************************************************************/

static inline void nxsem_restore_baseprio_task(FAR struct tcb_s *stcb,
                                               FAR sem_t *sem)
{
  /* The currently executed thread should be the lower priority
   * thread that just posted the count and caused this action.
   * However, we cannot drop the priority of the currently running
   * thread -- because that will cause it to be suspended.
   *
   * So, do this in two passes.  First, reprioritizing all holders
   * except for the running thread.
   */

  nxsem_foreachholder(sem, nxsem_restoreholderprio_others, stcb);

  /* Now, find an reprioritize only the ready to run task */

  nxsem_foreachholder(sem, nxsem_restoreholderprio_self, stcb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_initialize_holders
 *
 * Description:
 *   Called from nxsem_initialize() to set up semaphore holder information.
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

void nxsem_initialize_holders(void)
{
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  int i;

  /* Put all of the pre-allocated holder structures into the free list */

  g_freeholders = g_holderalloc;
  for (i = 0; i < (CONFIG_SEM_PREALLOCHOLDERS - 1); i++)
    {
      g_holderalloc[i].flink = &g_holderalloc[i + 1];
    }

  g_holderalloc[CONFIG_SEM_PREALLOCHOLDERS - 1].flink = NULL;
#endif
}

/****************************************************************************
 * Name: nxsem_destroyholder
 *
 * Description:
 *   Called from nxsem_destroy() to handle any holders of a semaphore
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
  /* It might be an error if a semaphore is destroyed while there are any
   * holders of the semaphore (except perhaps the thread that release the
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
   * any stranded holders and hope the task knows what it is doing.
   */

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  if (sem->hhead != NULL)
    {
      /* There may be an issue if there are multiple holders of
       * the semaphore.
       */

      DEBUGASSERT(sem->hhead->flink == NULL);
    }

#else
  /* There may be an issue if there are multiple holders of the semaphore. */

  DEBUGASSERT(sem->holder[0].htcb == NULL || sem->holder[1].htcb == NULL);

#endif

  nxsem_foreachholder(sem, nxsem_recoverholders, NULL);
}

/****************************************************************************
 * Name: nxsem_add_holder_tcb
 *
 * Description:
 *   Called from nxsem_wait() when the calling thread obtains the semaphore;
 *   Called from sem_post() when the waiting thread obtains the semaphore.
 *
 * Input Parameters:
 *   htcb - TCB of the thread that just obtained the semaphore
 *   sem  - A reference to the incremented semaphore
 *
 * Returned Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

void nxsem_add_holder_tcb(FAR struct tcb_s *htcb, FAR sem_t *sem)
{
  FAR struct semholder_s *pholder;

  /* If priority inheritance is disabled for this thread or it is IDLE
   * thread, then do not add the holder.
   * If there are never holders of the semaphore, the priority
   * inheritance is effectively disabled.
   */

  if (!is_idle_task(htcb) && (sem->flags & PRIOINHERIT_FLAGS_ENABLE) != 0)
    {
      /* Find or allocate a container for this new holder */

      pholder = nxsem_findorallocateholder(sem, htcb);
      if (pholder != NULL && pholder->counts < SEM_VALUE_MAX)
        {
          /* Increment the number of counts held by this holder */

          pholder->counts++;
        }
    }
}

/****************************************************************************
 * Name: nxsem_add_holder
 *
 * Description:
 *   Called from nxsem_wait() when the calling thread obtains the semaphore
 *
 * Input Parameters:
 *   sem - A reference to the incremented semaphore
 *
 * Returned Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

void nxsem_add_holder(FAR sem_t *sem)
{
  nxsem_add_holder_tcb(this_task(), sem);
}

/****************************************************************************
 * Name: void nxsem_boost_priority(sem_t *sem)
 *
 * Description:
 *
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_boost_priority(FAR sem_t *sem)
{
  FAR struct tcb_s *rtcb = this_task();

  /* Boost the priority of every thread holding counts on this semaphore
   * that are lower in priority than the new thread that is waiting for a
   * count.
   */

  nxsem_foreachholder(sem, nxsem_boostholderprio, rtcb);
}

/****************************************************************************
 * Name: nxsem_release_holder
 *
 * Description:
 *   Called from sem_post() after a thread releases one count on the
 *   semaphore.
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
  FAR struct semholder_s *pholder;
  FAR struct semholder_s *candidate = NULL;
  unsigned int total = 0;

  /* Find the container for this holder */

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  for (pholder = sem->hhead; pholder != NULL; pholder = pholder->flink)
#else
  int i;

  /* We have two hard-allocated holder structures in sem_t */

  for (i = 0; i < 2; i++)
#endif
    {
#if CONFIG_SEM_PREALLOCHOLDERS == 0
      pholder = &sem->holder[i];
      if (pholder->htcb == NULL)
        {
          continue;
        }
#endif

      DEBUGASSERT(pholder->counts > 0);

      if (pholder->htcb == rtcb)
        {
          /* Decrement the counts on this holder -- the holder will be freed
           * later in nxsem_restore_baseprio.
           */

          pholder->counts--;
          return;
        }

      total++;
      candidate = pholder;
    }

  /* The current task is not a holder */

  if (total == 1)
    {
      /* If the semaphore has only one holder, we can decrement the counts
       * simply.
       */

      candidate->counts--;
      return;
    }

  /* TODO:
   *   How do we choose the holder to decrement it's counts?
   */
}

/****************************************************************************
 * Name: nxsem_restore_baseprio
 *
 * Description:
 *   This function is called after the current running task releases a
 *   count on the semaphore or an interrupt handler posts a new count.  It
 *   will check if we need to drop the priority of any threads holding a
 *   count on the semaphore.  Their priority could have been boosted while
 *   they held the count.
 *
 * Input Parameters:
 *   stcb - The TCB of the task that was just started (if any).  If the
 *     post action caused a count to be given to another thread, then stcb
 *     is the TCB that received the count.  Note, just because stcb received
 *     the count, it does not mean that it is higher priority than other
 *     threads.
 *   sem - A reference to the semaphore being posted.
 *     - If the semaphore count is <0 then there are still threads waiting
 *       for a count.  stcb should be non-null and will be higher priority
 *       than all of the other threads still waiting.
 *     - If it is ==0 then stcb refers to the thread that got the last count;
 *       no other threads are waiting.
 *     - If it is >0 then there should be no threads waiting for counts and
 *       stcb should be null.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The scheduler is locked.
 *
 ****************************************************************************/

void nxsem_restore_baseprio(FAR struct tcb_s *stcb, FAR sem_t *sem)
{
#if 0  /* DSA: sometimes crashes when Telnet calls external cmd (i.e. 'i2c') */
  /* Check our assumptions */

  DEBUGASSERT((sem->semcount > 0  && stcb == NULL) ||
              (sem->semcount <= 0 && stcb != NULL));
#endif

  /* Perform the following actions only if a new thread was given a count.
   * The thread that received the count should be the highest priority
   * of all threads waiting for a count from the semaphore.  So in that
   * case, the priority of all holder threads should be dropped to the
   * next highest pending priority.
   */

  if (stcb != NULL)
    {
      /* Handler semaphore counts posted from an interrupt handler
       * differently from interrupts posted from threads.  The primary
       * difference is that if the semaphore is posted from a thread, then
       * the poster thread is a player in the priority inheritance scheme.
       * The interrupt handler externally injects the new count without
       * otherwise participating itself.
       */

      if (up_interrupt_context())
        {
          nxsem_restore_baseprio_irq(stcb, sem);
        }
      else
        {
          nxsem_restore_baseprio_task(stcb, sem);
        }
    }
  else
    {
      /* Remove the holder from the list if it's counts is zero. */

      nxsem_foreachholder(sem, nxsem_freecount0holder, NULL);

      /* If there are no tasks waiting for available counts, then all holders
       * should be at their base priority.
       */

#ifdef CONFIG_DEBUG_ASSERTIONS
      nxsem_foreachholder(sem, nxsem_verifyholder, NULL);
#endif
    }
}

/****************************************************************************
 * Name: nxsem_canceled
 *
 * Description:
 *   Called from nxsem_wait_irq() after a thread that was waiting for a
 *   semaphore count was awakened because of a signal and the semaphore wait
 *   has been canceled.  This function restores the correct thread priority
 *   of each holder of the semaphore.
 *
 * Input Parameters:
 *   sem - A reference to the semaphore no longer being waited for
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_canceled(FAR struct tcb_s *stcb, FAR sem_t *sem)
{
  /* Check our assumptions */

  DEBUGASSERT(sem->semcount <= 0);

  /* Adjust the priority of every holder as necessary */

  nxsem_foreachholder(sem, nxsem_restoreholderprio, stcb);
}

/****************************************************************************
 * Name: sem_enumholders
 *
 * Description:
 *   Show information about threads currently waiting on this semaphore
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
  nxsem_foreachholder(sem, nxsem_dumpholder, NULL);
#endif
}
#endif

/****************************************************************************
 * Name: nxsem_nfreeholders
 *
 * Description:
 *   Return the number of available holder containers.  This is a good way
 *   to find out which threads are not calling sem_destroy.
 *
 * Input Parameters:
 *   sem - A reference to the semaphore
 *
 * Returned Value:
 *   The number of available holder containers
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_SEM_PHDEBUG)
int nxsem_nfreeholders(void)
{
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  FAR struct semholder_s *pholder;
  int n;

  for (pholder = g_freeholders, n = 0; pholder; pholder = pholder->flink)
    {
      n++;
    }

  return n;
#else
  return 0;
#endif
}
#endif

/****************************************************************************
 * Name: nxsem_release_all
 *
 * Description:
 *   Release all semaphore holders for the task.
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
      FAR sem_t *sem = pholder->sem;

      nxsem_freeholder(sem, pholder);
    }
}

#endif /* CONFIG_PRIORITY_INHERITANCE */
