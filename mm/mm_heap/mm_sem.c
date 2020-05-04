/****************************************************************************
 * mm/mm_heap/mm_sem.c
 *
 *   Copyright (C) 2007-2009, 2013, 2017-2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/config.h>

#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/mm/mm.h>

#ifdef CONFIG_SMP
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Internal nxsem_* interfaces are not available in the user space in
 * PROTECTED and KERNEL builds.  In that context, the application semaphore
 * interfaces must be used.  The differences between the two sets of
 * interfaces are:  (1) the nxsem_* interfaces do not cause cancellation
 * points and (2) they do not modify the errno variable.
 *
 * See additional definitions in include/nuttx/semaphore.h
 *
 * REVISIT:  The fact that sem_wait() is a cancellation point is an issue
 * and does cause a violation:  It makes all of the memory management
 * interfaces into cancellation points when used from user space in the
 * PROTECTED and KERNEL builds.
 */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
#  define _SEM_GETERROR(r)
#else
#  define _SEM_GETERROR(r)  (r) = -errno
#endif

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
  /* Initialize the MM semaphore to one (to support one-at-a-time access to
   * private data sets).
   */

  nxsem_init(&heap->mm_semaphore, 0, 1);

  heap->mm_holder      = NO_HOLDER;
  heap->mm_counts_held = 0;
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
#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif
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

  if (my_pid < 0)
    {
      ret = my_pid;
      goto errout;
    }

  /* Does the current task already hold the semaphore?  Is the current
   * task actually running?
   */

  if (heap->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references held by the current
       * task.
       */

      heap->mm_counts_held++;
      ret = OK;
    }
  else
    {
      /* Try to take the semaphore */

      ret = _SEM_TRYWAIT(&heap->mm_semaphore);
      if (ret < 0)
        {
          _SEM_GETERROR(ret);
          goto errout;
        }

      /* We have it.  Claim the heap for the current task and return */

      heap->mm_holder      = my_pid;
      heap->mm_counts_held = 1;
      ret = OK;
    }

errout:
#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
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
#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif
  pid_t my_pid = getpid();

  /* Does the current task already hold the semaphore? */

  if (heap->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references held by the current
       * task.
       */

      heap->mm_counts_held++;
    }
  else
    {
      int ret;

      /* Take the semaphore (perhaps waiting) */

      mseminfo("PID=%d taking\n", my_pid);
      do
        {
          ret = _SEM_WAIT(&heap->mm_semaphore);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          if (ret < 0)
            {
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
              DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
#else
              int errcode = get_errno();
              DEBUGASSERT(errcode == EINTR || errcode == ECANCELED);
              ret = -errcode;
#endif
            }
        }
      while (ret == -EINTR);

      /* We have it (or some awful, unexpected error occurred).  Claim
       * the semaphore for the current task and return.
       */

      heap->mm_holder      = my_pid;
      heap->mm_counts_held = 1;
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
  mseminfo("Holder=%d count=%d\n", heap->mm_holder, heap->mm_counts_held);
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
#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif

  /* The current task should be holding at least one reference to the
   * semaphore.
   */

  DEBUGASSERT(heap->mm_holder == getpid());

  /* Does the current task hold multiple references to the semaphore */

  if (heap->mm_counts_held > 1)
    {
      /* Yes, just release one count and return */

      heap->mm_counts_held--;
      mseminfo("Holder=%d count=%d\n", heap->mm_holder,
               heap->mm_counts_held);
    }
  else
    {
      /* Nope, this is the last reference held by the current task. */

      mseminfo("PID=%d giving\n", getpid());

      heap->mm_holder      = NO_HOLDER;
      heap->mm_counts_held = 0;
      DEBUGVERIFY(_SEM_POST(&heap->mm_semaphore));
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}
