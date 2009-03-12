/****************************************************************************
 * sched/sem_holder.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <semaphore.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "sem_internal.h"

#ifdef CONFIG_PRIORITY_INHERITANCE

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SEM_PREALLOCHOLDERS
#  define CONFIG_SEM_PREALLOCHOLDERS 0
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

typedef int (*holderhandler_t)(struct semholder_s *pholder, FAR sem_t *sem, FAR void *arg);

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* Preallocated holder structures */

#if CONFIG_SEM_PREALLOCHOLDERS > 0
static struct semholder_s g_holderalloc[CONFIG_SEM_PREALLOCHOLDERS];
static FAR struct semholder_s *g_freeholders;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sem_allocholder
 ****************************************************************************/

static inline FAR struct semholder_s *sem_allocholder(sem_t *sem)
{
  FAR struct semholder_s *pholder;

  /* Check if the "built-in" holder is being used.  We have this built-in
   * holder to optimize for the simplest case where semaphores are only
   * used to implement mutexes.
   */

  if (!sem->hlist.holder)
    {
      pholder          = &sem->hlist;
      pholder->counts  = 0;
    }
  else
    {
#if CONFIG_SEM_PREALLOCHOLDERS > 0
      pholder = g_freeholders;
      if (pholder)
        {
          /* Remove the holder from the free list an put it into the semaphore's holder list */

          g_freeholders    = pholder->flink;
          pholder->flink   = sem->hlist.flink;
          sem->hlist.flink = pholder;

          /* Make sure the the initial count is zero */

          pholder->counts  = 0;
        }
      else
#else
      pholder = NULL;
#endif
      sdbg("Insufficient pre-allocated holders\n");
    }
  return pholder;
}

/****************************************************************************
 * Function:  sem_findholder
 ****************************************************************************/

static FAR struct semholder_s *sem_findholder(sem_t *sem, FAR _TCB *htcb)
{
  FAR struct semholder_s *pholder;

  /* Try to find the holder in the list of holders associated with this semaphore */

  pholder = &sem->hlist;
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  for (; pholder; pholder = pholder->flink)
#endif
    {
      if (pholder->holder == htcb)
        {
          /* Got it! */

          return pholder;
        }
    }

  /* The holder does not appear in the list */

  return NULL;
}

/****************************************************************************
 * Function:  sem_findorallocateholder
 ****************************************************************************/

static inline FAR struct semholder_s *sem_findorallocateholder(sem_t *sem, FAR _TCB *htcb)
{
  FAR struct semholder_s *pholder = sem_findholder(sem, htcb);
  if (!pholder)
    {
      pholder = sem_allocholder(sem);
    }
  return pholder;
}

/****************************************************************************
 * Function:  sem_freeholder
 ****************************************************************************/

static inline void sem_freeholder(sem_t *sem, FAR struct semholder_s *pholder)
{
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  FAR struct semholder_s *curr;
  FAR struct semholder_s *prev;
#endif

  /* Release the holder and counts */

  pholder->holder = 0;
  pholder->counts = 0;

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  /* If this is the holder inside the semaphore, then do nothing more */

  if (pholder != &sem->hlist)
    {
      /* Otherwise, search the list for the matching holder */

      for (prev = &sem->hlist, curr = sem->hlist.flink;
           curr && curr != pholder;
           prev = curr, curr = curr->flink);

      if (curr)
        {
          /* Remove the holder from the list */

          prev->flink = pholder->flink;

          /* And put it in the free list */

          pholder->flink = g_freeholders;
          g_freeholders  = pholder;
        }
    }
#endif
}

/****************************************************************************
 * Name: sem_foreachholder
 ****************************************************************************/

static int sem_foreachholder(FAR sem_t *sem, holderhandler_t handler, FAR void *arg)
{
  struct semholder_s *pholder = &sem->hlist;
  int ret = 0;

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  for (; pholder && ret == 0; pholder = next)
#endif
    {
#if CONFIG_SEM_PREALLOCHOLDERS > 0
      /* In case this holder gets deleted */

      next = pholder->flink;
#endif
      /* The initial "built-in" container may hold a NULL holder */

      if (pholder->holder)
        {
          /* Call the handler */

          ret = handler(pholder, sem, arg);
        }
    }
  return ret;
}

/****************************************************************************
 * Name: sem_boostholderprio
 ****************************************************************************/

static int sem_boostholderprio(struct semholder_s *pholder, FAR sem_t *sem, FAR void *arg)
{
  FAR _TCB *htcb = (FAR _TCB *)pholder->holder;
  FAR _TCB *rtcb = (FAR _TCB*)arg;
#if CONFIG_SEM_NNESTPRIO > 0
  int i;
#endif

  /* Make sure that the thread is still active.  If it exited without releasing
   * its counts, then that would be a bad thing.  But we can take no real
   * action because we don't know know that the program is doing.  Perhaps its
   * plan is to kill a thread, then destroy the semaphore.
   */

  if (!sched_verifytcb(htcb))
   {
      sdbg("TCB 0x%08x is a stale handle, counts lost\n", htcb);
      sem_freeholder(sem, pholder);
   }

#if CONFIG_SEM_NNESTPRIO > 0

  /* If the priority of the thread that is waiting for a count is greater than
   * the base priority of the thread holding a count, then we may need to
   * adjust the holder's priority now or later to that priority.
   */

  else if (rtcb->sched_priority > htcb->base_priority)
    {
      /* If the new priority is greater than the current, possibly already
       * boosted priority of the holder thread, then we will have to raise
       * the holder's priority now.
       */

      if (rtcb->sched_priority > htcb->sched_priority)
        {
          /* If the current priority has already been boosted, then add the
           * boost priority to the list of restoration priorities.  When the
           * higher priority thread gets its count, then we need to revert
           * to this saved priority, not to the base priority.
           */

          if (htcb->sched_priority > htcb->base_priority)
            {
              /* Save the current, boosted priority */

              if (htcb->npend_reprio < CONFIG_SEM_NNESTPRIO)
                {
                  htcb->pend_reprios[htcb->npend_reprio] = htcb->sched_priority;
                  htcb->npend_reprio++;
                }
              else
                {
                  sdgb("CONFIG_SEM_NNESTPRIO exceeded\n");
                }  
            }

          /* Raise the priority of the holder of the semaphore.  This
           * cannot cause a context switch because we have preemption
           * disabled.  The task will be marked "pending" and the switch
           * will occur during up_block_task() processing.
           */

          (void)sched_setpriority(htcb, rtcb->sched_priority);
        }
      else
        {
          /* The new priority is above the base priority of the holder,
           * but not as high as its current working priority.  Just put it
           * in the list of pending restoration priorities so that when the
           * higher priority thread gets its count, we can revert to this
           * saved priority and not to the base priority.
           */

          htcb->pend_reprios[htcb->npend_reprio] = rtcb->sched_priority;
          htcb->npend_reprio++;
        }
    }

#else
  /* If the priority of the thread that is waiting for a count is less than
   * of equal to the priority of the thread holding a count, then do nothing
   * because the thread is already running at a sufficient priority.
   */

   else if (rtcb->sched_priority > htcb->sched_priority)
     {
       /* Raise the priority of the holder of the semaphore.  This
        * cannot cause a context switch because we have preemption
        * disabled.  The task will be marked "pending" and the switch
        * will occur during up_block_task() processing.
        */

       (void)sched_setpriority(htcb, rtcb->sched_priority);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: sem_verifyholder
 ****************************************************************************/

#ifdef CONFIG_DEBUG
static int sem_verifyholder(struct semholder_s *pholder, FAR sem_t *sem, FAR void *arg)
{
  FAR _TCB *htcb = (FAR _TCB *)pholder->holder;

#if CONFIG_SEM_NNESTPRIO > 0
  DEBUGASSERT(htcb->npend_repri == 0);
#endif
  DEBUGASSERT(htcb->sched_priority == htcb->base_priority);
  return 0;
}
#endif

/****************************************************************************
 * Name: sem_restoreholderprio
 ****************************************************************************/

static int sem_restoreholderprio(struct semholder_s *pholder, FAR sem_t *sem, FAR void *arg)
{
  FAR _TCB *htcb = (FAR _TCB *)pholder->holder;
#if CONFIG_SEM_NNESTPRIO > 0
  FAR _TCB *stcb = (FAR _TCB *)arg;
  int rpriority;
  int i;
  int j;
#endif

  /* Make sure that the thread is still active.  If it exited without releasing
   * its counts, then that would be a bad thing.  But we can take no real
   * action because we don't know know that the program is doing.  Perhaps its
   * plan is to kill a thread, then destroy the semaphore.
   */

  if (!sched_verifytcb(htcb))
   {
      sdbg("TCB 0x%08x is a stale handle, counts lost\n", htcb);
      sem_freeholder(sem, pholder);
   }

  /* Was the priority of this thread boosted? If so, then drop its priority
   * back to the correct level.
   */

  else if (htcb->sched_priority != htcb->base_priority)
    {
#if CONFIG_SEM_NNESTPRIO > 0
      /* Are there other, pending priority levels to revert to? */

      if (htcb->npend_reprio < 1)
        {
          /* No... the thread has only been boosted once */

          DEBUGASSERT(hctb->sched_priority == stcb->sched_priority && npend_reprio == 0);
          rpriority = htcb->base_priority;
        }

      /* There are multiple pending priority levels. The thread's "boosted"
       * priority could greater than or equal to "stcb->sched_priority" (it could be
       * greater if its priority we boosted becuase it also holds another semaphore.
       */

      else if (htcb->sched_priority <= stcb->sched_priority)
        {
          /* The thread has been boosted to the same priority as the task
           * that just received the count.  We will simply reprioritized
           * to the next highest priority that we have in rpriority.
           */

          /* Find the highest pending priority and remove it from the list */

          for (i = 1, j = 0; i < htcb->npend_reprio; i++)
            {
              if (htcb->pend_reprios[i] > htcb->pend_reprios[j])
                {
                  j = i;
                }
            }

          /* Remove the highest priority pending priority from the list */

          rpriority = htcb->pend_reprios[j];
          i = htcb->npend_reprio - 1;
          if (i > 0)
            {
              htcb->pend_reprios[j] = htcb->pend_reprios[i];
            }
          htcb->npend_reprio = i;

          /* And apply that priority to the thread */

          sched_reprioritize(htcb, rpriority);
        }                    
      else
        {
          /* The thread has been boosted to a higher priority than the task.  The
           * pending priority should be in he list (unless it was lost because of
           * of list overflow).
           *
           * Search the list for the matching priority.
           */

          for (i = 0; i < htcb->npend_reprio; i++)
            {
              /* Does this pending priority match the priority of the thread
               * that just received the count? 
               */

              if (htcb->pend_reprios[i] == stcb->sched_priority)
                {
                  /* Yes, remove it from the list */

                  j = htcb->npend_reprio - 1;
                  if (j > 0)
                    {
                      htcb->pend_reprios[i] = htcb->pend_reprios[j];
                    }
                   htcb->npend_reprio = j;
                   break;
                }
            }
        }
#else
      /* There is no alternative restore priorities, drop the priority
       * all the way back to the threads "base" priority.
       */

      sched_reprioritize(htcb, htcb->base_priority);
#endif
    }
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sem_initholders
 *
 * Description:
 *   Called from sem_initialize() to set up semaphore holder information.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void sem_initholders(void)
{
#if CONFIG_SEM_PREALLOCHOLDERS > 0
 int i;

  /* Put all of the pre-allocated holder structures into  free list */

  g_freeholders = g_holderalloc;
  for (i = 0; i < (CONFIG_SEM_PREALLOCHOLDERS-1); i++)
    {
      g_holderalloc[i].flink = &g_holderalloc[i+1];
    }
  g_holderalloc[CONFIG_SEM_PREALLOCHOLDERS-1].flink = NULL;
#endif
}

/****************************************************************************
 * Function:  sem_destroyholder
 *
 * Description:
 *   Called from sem_destroy() to handle any holders of a semaphore when
 *   it is destroyed.
 *
 * Parameters:
 *   sem - A reference to the semaphore being destroyed
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void sem_destroyholder(FAR sem_t *sem)
{
#if 0
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;

  /* It is an error is a semaphore is destroyed while there are any holders
   * (except perhaps the thread releas the semaphore itself).  Hmmm.. but
   * we actually have to assume that the caller knows what it is doing because
   * could have killed another thread that is the actual holder of the semaphore.
   */

#if CONFIG_SEM_PREALLOCHOLDERS > 0
  DEBUGASSERT((!sem->hlist.holder || sem->hlist.holder == rtcb) && !sem->hlist.flink);
#else
  DEBUGASSERT(!sem->hlist.holder || sem->hlist.holder == rtcb);
#endif
#endif

  sem->hlist.holder = NULL;
}

/****************************************************************************
 * Function:  sem_addholder
 *
 * Description:
 *   Called from sem_wait() when the calling thread obtains the semaphore
 *
 * Parameters:
 *   sem - A reference to the incremented semaphore
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *
 ****************************************************************************/

void sem_addholder(FAR sem_t *sem)
{
  FAR _TCB               *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR struct semholder_s *pholder;

  /* Find or allocate a container for this new holder */

  pholder = sem_findorallocateholder(sem, rtcb);
  if (pholder)
    {
      /* Then set the holder and increment the number of counts held by this holder */

      pholder->holder = rtcb;
      pholder->counts++;
    }
}

/****************************************************************************
 * Function:  void sem_boostpriority(sem_t *sem)
 *
 * Description:
 *   
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *
 ****************************************************************************/

void sem_boostpriority(FAR sem_t *sem)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;

  /* Boost the priority of every thread holding counts on this semaphore
   * that are lower in priority than the new thread that is waiting for a
   * count.
   */

   (void)sem_foreachholder(sem, sem_boostholderprio, rtcb);
}

/****************************************************************************
 * Function:  sem_releaseholder
 *
 * Description:
 *   Called from sem_post() after a thread releases one count on the
 *   semaphore.
 *
 * Parameters:
 *   sem - A reference to the semaphore being posted
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void sem_releaseholder(FAR sem_t *sem)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR struct semholder_s *pholder;

  /* Find the container for this holder */

  pholder = sem_findholder(sem, rtcb);
  if (pholder && pholder->counts > 0)
    {
      /* Decrement the counts on this holder -- the holder will be freed
       * later in sem_restorebaseprio.
       */

      pholder->counts--;
    }
}

/****************************************************************************
 * Function:  sem_restorebaseprio
 *
 * Description:
 *   This function is called after the current running task releases a
 *   count on the semaphore.  It will check if we need to drop the priority
 *   of any threads holding a count on the semaphore.  Their priority could
 *   have been boosted while they held the count.
 *
 * Parameters:
 *   stcb - The TCB of the task that was just started (if any).  If the
 *     post action caused a count to be given to another thread, then stcb
 *     is the TCB that received the count.  Note, just because stcb received
 *     the count, it does not mean that it it is higher priority than other threads.
 *   sem - A reference to the semaphore being posted.
 *     - If the semaphore count is <0 then there are still threads waiting
 *       for a count.  stcb should be non-null and will be higher priority than
 *       all of the other threads still waiting.
 *     - If it is ==0 then stcb refers to the thread that got the last count; no
 *       other threads are waiting.
 *     - If it is >0 then there should be no threads waiting for counts and
 *       stcb should be null.
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *
 ****************************************************************************/

void sem_restorebaseprio(FAR _TCB *stcb, FAR sem_t *sem)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  struct semholder_s *pholder;

  /* Check our assumptions */

  DEBUGASSERT((sem->semcount > 0  && stcb == NULL) ||
              (sem->semcount <= 0 && stcb != NULL));

  /* Perfom the following actions only if a new thread was given a count. */

  if (stcb)
    {
      /* Adjust the priority of every holder as necessary */

      (void)sem_foreachholder(sem, sem_restoreholderprio, stcb);
    }

  /* If there are no tasks waiting for available counts, then all holders
   * should be at their base priority.
   */

#ifdef CONFIG_DEBUG
  else
    {
      (void)sem_foreachholder(sem, sem_verifyholder, NULL);
    }
#endif

  /* In any case, the currently execuing task should have an entry in the 
   * list and we need to decrement the number of counts that it holds.  When it
   * holds no further counts, it must be removed from the list of holders.
   */

  pholder = sem_findholder(sem, rtcb);
  if (pholder)
    {
      /* When no more counts are held, remove the holder from the list.  The
       * count was decremented in sem_releaseholder.
       */

      if (pholder->counts <= 0)
        {
          sem_freeholder(sem, pholder);
        }
    }
}

/****************************************************************************
 * Function:  sem_canceled
 *
 * Description:
 *   Called from sem_post() after a thread that was waiting for a semaphore
 *   count was awakened because of a signal and the semaphore wait has been
 *   canceld.
 *
 * Parameters:
 *   sem - A reference to the semaphore no longer being waited for
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void sem_canceled(FAR sem_t *sem)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;

  /* Check our assumptions */

  DEBUGASSERT(sem->semcount <= 0);

  /* Adjust the priority of every holder as necessary */

  (void)sem_foreachholder(sem, sem_restoreholderprio, rtcb);
}

#endif /* CONFIG_PRIORITY_INHERITANCE */
