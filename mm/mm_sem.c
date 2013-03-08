/****************************************************************************
 * mm/mm_sem.c
 *
 *   Copyright (C) 2007-2009, 2013 Gregory Nutt. All rights reserved.
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

#include <nuttx/mm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define the following to enable semaphore state monitoring */
//#define MONITOR_MM_SEMAPHORE 1

#ifdef MONITOR_MM_SEMAPHORE
#  ifdef CONFIG_DEBUG
#    include <debug.h>
#    define msemdbg dbg
#  else
#    define msemdbg printf
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define msemdbg(x...)
#  else
#    define msemdbg (void)
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
   * private data sets.
   */

  (void)sem_init(&heap->mm_semaphore, 0, 1);

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
  pid_t my_pid = getpid();

  /* Do I already have the semaphore? */

  if (heap->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references that I have */

      heap->mm_counts_held++;
      return OK;
    }
  else
    {
      /* Try to take the semaphore (perhaps waiting) */

      if (sem_trywait(&heap->mm_semaphore) != 0)
       {
         return ERROR;
       }

      /* We have it.  Claim the stak and return */

      heap->mm_holder      = my_pid;
      heap->mm_counts_held = 1;
      return OK;
    }
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
  pid_t my_pid = getpid();

  /* Do I already have the semaphore? */

  if (heap->mm_holder == my_pid)
    {
      /* Yes, just increment the number of references that I have */

      heap->mm_counts_held++;
    }
  else
    {
      /* Take the semaphore (perhaps waiting) */

      msemdbg("PID=%d taking\n", my_pid);
      while (sem_wait(&heap->mm_semaphore) != 0)
        {
          /* The only case that an error should occur here is if
           * the wait was awakened by a signal.
           */

          ASSERT(errno == EINTR);
        }

      /* We have it.  Claim the stake and return */

      heap->mm_holder      = my_pid;
      heap->mm_counts_held = 1;
    }

  msemdbg("Holder=%d count=%d\n", heap->mm_holder, heap->mm_counts_held);
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
#ifdef CONFIG_DEBUG
  pid_t my_pid = getpid();
#endif

  /* I better be holding at least one reference to the semaphore */

  DEBUGASSERT(heap->mm_holder == my_pid);

  /* Do I hold multiple references to the semphore */

  if (heap->mm_counts_held > 1)
    {
      /* Yes, just release one count and return */

      heap->mm_counts_held--;
      msemdbg("Holder=%d count=%d\n", heap->mm_holder, heap->mm_counts_held);
    }
  else
    {
      /* Nope, this is the last reference I have */

      msemdbg("PID=%d giving\n", my_pid);
      heap->mm_holder      = -1;
      heap->mm_counts_held = 0;
      ASSERT(sem_post(&heap->mm_semaphore) == 0);
    }
}

