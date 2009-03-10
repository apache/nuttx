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
#  define CONFIG_SEM_PREALLOCHOLDERS (4*CONFIG_MAX_TASKS)
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

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
      pholder = &sem->hlist;
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
#ifdef CONFIG_SEM_PREALLOCHOLDERS
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

void sem_destroyholder(sem_t *sem)
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

void sem_addholder(sem_t *sem)
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

void sem_boostpriority(sem_t *sem)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR _TCB *htcb;
  struct semholder_s *pholder;

  /* Traverse the list of holders */

  pholder = &sem->hlist;
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  for (; pholder; pholder = pholder->flink)
#endif
    {
      /* As an artifact, there will be holders that have no counts.  This
       * because they have posted the semaphore and there count was decrement
       * by sem_releaseholder(), but they are still being retained in the
       * list to be harvested by sem_restorebaseprio.
       */

      if (pholder->counts > 0)
        {
          htcb = pholder->holder;
          if (htcb && htcb->sched_priority < rtcb->sched_priority)
            {
              /* Raise the priority of the holder of the semaphore.  This
               * cannot cause a context switch because we have preemption
               * disabled.  The task will be marked "pending" and the switch
               * will occur during up_block_task() processing.
               *
               * NOTE that we have to restore base_priority because
               * sched_setparam() should set both.
               */

              int base_priority = htcb->base_priority;
              (void)sched_settcbprio(htcb, rtcb->sched_priority);
              htcb->base_priority = base_priority;
            }
        }
    }
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

void sem_releaseholder(sem_t *sem)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR struct semholder_s *pholder;

  /* Find the container for this holder */

  pholder = sem_findholder(sem, rtcb);
  if (pholder && pholder->counts > 0)
    {
      /* Decrement the counts on this holder */

      pholder->counts--;
    }
}

/****************************************************************************
 * Function:  sem_restorebaseprio
 *
 * Description:
 *   Check if we need to drop the priority of any threads holding the
 *   semaphore.  The priority could have been boosted while they held the
 *   semaphore.
 *
 * Parameters:
 *   sem - A reference to the semaphore being posted
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *
 ****************************************************************************/

void sem_restorebaseprio(sem_t *sem)
{
  FAR _TCB  *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR _TCB *htcb;
  struct semholder_s *pholder;
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  struct semholder_s *next;
#endif

  /* Check if the semaphore is still available */

  if (sem->semcount > 0)
    {
      /* Traverse the list of holders */

      pholder = &sem->hlist;
#if CONFIG_SEM_PREALLOCHOLDERS > 0
      for (; pholder; pholder = next)
#endif
        {
#if CONFIG_SEM_PREALLOCHOLDERS > 0
          next = pholder->flink; /* In case pholder gets deleted */
#endif
          /* The initial "built-in" container may hold a NULL holder */

          htcb = pholder->holder;
          if (htcb)
            {
              /* Was the priority of this thread boosted? NOTE:  There is
               * a logical flaw here:  If the thread holds multiple semaphore
               * and has been boosted multiple times, then there is no mechanism
               * to know the correct priority to restore and we may error in
               * prematurely lowering the priority.
               */

              if (htcb->sched_priority != htcb->base_priority)
                {
                  up_reprioritize_rtr(rtcb, htcb->base_priority);
                }

              /* When no more counts are held, remove the holder from the list */

              if (pholder->counts <= 0)
                {
	          sem_freeholder(sem, pholder);
                }
            }
        }
    }
}

#endif /* CONFIG_PRIORITY_INHERITANCE */
