/****************************************************************************
 * mm/mm_heap/mm_sem.c
 *
 *   Copyright (C) 2007-2009, 2013, 2017 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <errno.h>
#include <assert.h>

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

/* Define the following to enable semaphore state monitoring */
//#define MONITOR_MM_SEMAPHORE 1

#ifdef MONITOR_MM_SEMAPHORE
#  include <debug.h>
#  define msemerr  _err
#  define msemwarn _warn
#  define mseminfo _info
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define msemerr(x...)
#    define msemwarn(x...)
#    define mseminfo(x...)
#  else
#    define msemerr  (void)
#    define msemwarn (void)
#    define mseminfo (void)
#  endif
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

  (void)nxsem_init(&heap->mm_semaphore, 0, 1);

  heap->mm_holder      = -1;
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

  /* Do I already have the semaphore? */

  if (heap->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references that I have */

      heap->mm_counts_held++;
      ret = OK;
    }
  else
    {
      /* Try to take the semaphore (perhaps waiting) */

      ret = _SEM_TRYWAIT(&heap->mm_semaphore);
      if (ret < 0)
       {
         _SEM_GETERROR(ret);
         goto errout;
       }

      /* We have it.  Claim the heap and return */

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

  /* Do I already have the semaphore? */

  if (heap->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references that I have */

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
       * the semaphore and return.
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
#if defined(CONFIG_DEBUG_ASSERTIONS) || \
   (defined(MONITOR_MM_SEMAPHORE) && defined(CONFIG_DEBUG_INFO))
  pid_t my_pid = getpid();
#endif

  /* I better be holding at least one reference to the semaphore */

  DEBUGASSERT(heap->mm_holder == my_pid);

  /* Do I hold multiple references to the semphore */

  if (heap->mm_counts_held > 1)
    {
      /* Yes, just release one count and return */

      heap->mm_counts_held--;
      mseminfo("Holder=%d count=%d\n", heap->mm_holder,
               heap->mm_counts_held);
    }
  else
    {
      /* Nope, this is the last reference I have */

      mseminfo("PID=%d giving\n", my_pid);

      heap->mm_holder      = -1;
      heap->mm_counts_held = 0;
      DEBUGVERIFY(_SEM_POST(&heap->mm_semaphore));
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}
