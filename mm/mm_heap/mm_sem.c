/****************************************************************************
 * mm/mm_heap/mm_sem.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is a special value that indicates that there is no holder of the
 * semaphore.  The valid range of PIDs is 0-32767 and any value outside of
 * that range could be used (except -ESRCH which is a special return value
 * from getpid())
 */

#define NO_HOLDER ((pid_t)-1)

/* Define MONITOR_MM_SEMAPHORE to enable semaphore state monitoring */

#ifdef MONITOR_MM_SEMAPHORE
#  define msemerr  _err
#  define msemwarn _warn
#  define mseminfo _info
#else
#  define msemerr  _none
#  define msemwarn _none
#  define mseminfo _none
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_seminitialize
 *
 * Description:
 *   Initialize the MM mutex
 *
 ****************************************************************************/

void mm_seminitialize(FAR struct mm_heap_s *heap)
{
  FAR struct mm_heap_impl_s *heap_impl;

  DEBUGASSERT(MM_IS_VALID(heap));
  heap_impl = heap->mm_impl;

  /* Initialize the MM semaphore to one (to support one-at-a-time access to
   * private data sets).
   */

  _SEM_INIT(&heap_impl->mm_semaphore, 0, 1);

  heap_impl->mm_holder      = NO_HOLDER;
  heap_impl->mm_counts_held = 0;
}

/****************************************************************************
 * Name: mm_trysemaphore
 *
 * Description:
 *   Try to take the MM mutex.  This is called only from the OS in certain
 *   conditions when it is necessary to have exclusive access to the memory
 *   manager but it is impossible to wait on a semaphore (e.g., the idle
 *   process when it performs its background memory cleanup).
 *
 ****************************************************************************/

int mm_trysemaphore(FAR struct mm_heap_s *heap)
{
  FAR struct mm_heap_impl_s *heap_impl;
  pid_t my_pid = getpid();
  int ret;

  /* getpid() returns the task ID of the task at the head of the ready-to-
   * run task list.  mm_trysemaphore() may be called during context
   * switches.  There are certain situations during context switching when
   * the OS data structures are in flux and where the current task (i.e.,
   * the task at the head of the ready-to-run task list) is not actually
   * running. Granting the semaphore access in that case is known to result
   * in heap corruption as in the following failure scenario.
   *
   * ----------------------------    -------------------------------
   * TASK A                          TASK B
   * ----------------------------    -------------------------------
   *                                 Begins memory allocation.
   *                                 - Holder is set to TASK B
   *                             <---- Task B preempted, Task A runs
   * Task A exits
   * - Current task set to Task B
   * Free tcb and stack memory
   * - Since holder is Task B,
   *   memory manager is re-
   *   entered, and
   * - Heap is corrupted.
   * ----------------------------    -------------------------------
   *
   * This is handled by getpid():  If the case where Task B is not actually
   * running, then getpid() will return the special value -ESRCH.  That will
   * avoid taking the fatal 'if' logic and will fall through to use the
   * 'else', albeit with a nonsensical PID value.
   */

  DEBUGASSERT(MM_IS_VALID(heap));
  heap_impl = heap->mm_impl;

  if (my_pid < 0)
    {
      ret = my_pid;
      goto errout;
    }

  /* Does the current task already hold the semaphore?  Is the current
   * task actually running?
   */

  if (heap_impl->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references held by the current
       * task.
       */

      heap_impl->mm_counts_held++;
      ret = OK;
    }
  else
    {
      /* Try to take the semaphore */

      ret = _SEM_TRYWAIT(&heap_impl->mm_semaphore);
      if (ret < 0)
        {
          ret = _SEM_ERRVAL(ret);
          goto errout;
        }

      /* We have it.  Claim the heap for the current task and return */

      heap_impl->mm_holder      = my_pid;
      heap_impl->mm_counts_held = 1;
      ret = OK;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: mm_takesemaphore
 *
 * Description:
 *   Take the MM mutex.  This is the normal action before all memory
 *   management actions.
 *
 ****************************************************************************/

void mm_takesemaphore(FAR struct mm_heap_s *heap)
{
  FAR struct mm_heap_impl_s *heap_impl;
  pid_t my_pid = getpid();

  DEBUGASSERT(MM_IS_VALID(heap));
  heap_impl = heap->mm_impl;

  /* Does the current task already hold the semaphore? */

  if (heap_impl->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references held by the current
       * task.
       */

      heap_impl->mm_counts_held++;
    }
  else
    {
      int ret;

      /* Take the semaphore (perhaps waiting) */

      mseminfo("PID=%d taking\n", my_pid);
      do
        {
          ret = _SEM_WAIT(&heap_impl->mm_semaphore);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          if (ret < 0)
            {
              ret = _SEM_ERRVAL(ret);
              DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
            }
        }
      while (ret == -EINTR);

      /* We have it (or some awful, unexpected error occurred).  Claim
       * the semaphore for the current task and return.
       */

      heap_impl->mm_holder      = my_pid;
      heap_impl->mm_counts_held = 1;
    }

  mseminfo("Holder=%d count=%d\n", heap_impl->mm_holder,
            heap_impl->mm_counts_held);
}

/****************************************************************************
 * Name: mm_givesemaphore
 *
 * Description:
 *   Release the MM mutex when it is not longer needed.
 *
 ****************************************************************************/

void mm_givesemaphore(FAR struct mm_heap_s *heap)
{
  FAR struct mm_heap_impl_s *heap_impl;

  DEBUGASSERT(MM_IS_VALID(heap));
  heap_impl = heap->mm_impl;

  /* The current task should be holding at least one reference to the
   * semaphore.
   */

  DEBUGASSERT(heap_impl->mm_holder == getpid());

  /* Does the current task hold multiple references to the semaphore */

  if (heap_impl->mm_counts_held > 1)
    {
      /* Yes, just release one count and return */

      heap_impl->mm_counts_held--;
      mseminfo("Holder=%d count=%d\n", heap_impl->mm_holder,
               heap_impl->mm_counts_held);
    }
  else
    {
      /* Nope, this is the last reference held by the current task. */

      mseminfo("PID=%d giving\n", getpid());

      heap_impl->mm_holder      = NO_HOLDER;
      heap_impl->mm_counts_held = 0;
      DEBUGVERIFY(_SEM_POST(&heap_impl->mm_semaphore));
    }
}
