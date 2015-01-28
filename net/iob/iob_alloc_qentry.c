/****************************************************************************
 * net/iob/iob_alloc_qentry.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#if defined(CONFIG_DEBUG) && defined(CONFIG_IOB_DEBUG)
/* Force debug output (from this file only) */

#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <semaphore.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/net/iob.h>

#include "iob.h"

#if CONFIG_IOB_NCHAINS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_allocwait_qentry
 *
 * Description:
 *   Allocate an I/O buffer chain container by taking the buffer at the head
 *   of the free list.  This function is intended only for internal use by
 *   the IOB module.
 *
 ****************************************************************************/

static FAR struct iob_qentry_s *iob_allocwait_qentry(void)
{
  FAR struct iob_qentry_s *qentry;
  irqstate_t flags;
  int ret;

  /* The following must be atomic; interrupt must be disabled so that there
   * is no conflict with interrupt level I/O buffer chain container
   * allocations.  This is not as bad as it sounds because interrupts will be
   * re-enabled while we are waiting for I/O buffers to become free.
   */

  flags = irqsave();
  do
    {
      /* Try to get an I/O buffer chain container.  If successful, the
       * semaphore count will be decremented atomically.
       */

      qentry = iob_tryalloc_qentry();
      if (!qentry)
        {
          /* If not successful, then the semaphore count was less than or
           * equal to zero (meaning that there are no free buffers).  We
           * need to wait for an I/O buffer chain container to be released
           * when the semaphore count will be incremented.
           */

          ret = sem_wait(&g_qentry_sem);

          /* When we wake up from wait, an I/O buffer chain container was
           * returned to the free list.  However, if there are concurrent
           * allocations from interrupt handling, then I suspect that there
           * is a race condition.  But no harm, we will just wait again in
           * that case.
           */
        }
    }
  while (ret == OK && !qentry);

  irqrestore(flags);
  return qentry;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_alloc_qentry
 *
 * Description:
 *   Allocate an I/O buffer chain container by taking the buffer at the head
 *   of the free list. This function is intended only for internal use by
 *   the IOB module.
 *
 ****************************************************************************/

FAR struct iob_qentry_s *iob_alloc_qentry(void)
{
  /* Were we called from the interrupt level? */

  if (up_interrupt_context())
    {
      /* Yes, then try to allocate an I/O buffer without waiting */

      return iob_tryalloc_qentry();
    }
  else
    {
      /* Then allocate an I/O buffer, waiting as necessary */

      return iob_allocwait_qentry();
    }
}

/****************************************************************************
 * Name: iob_tryalloc_qentry
 *
 * Description:
 *   Try to allocate an I/O buffer chain container by taking the buffer at
 *   the head of the free list without waiting for the container to become
 *   free. This function is intended only for internal use by the IOB module.
 *
 ****************************************************************************/

FAR struct iob_qentry_s *iob_tryalloc_qentry(void)
{
  FAR struct iob_qentry_s *iobq;
  irqstate_t flags;

  /* We don't know what context we are called from so we use extreme measures
   * to protect the free list:  We disable interrupts very briefly.
   */

  flags = irqsave();
  iobq  = g_iob_freeqlist;
  if (iobq)
    {
      /* Remove the I/O buffer chain container from the free list and
       * decrement the counting semaphore that tracks the number of free
       * containers.
       */

      g_iob_freeqlist = iobq->qe_flink;

      /* Take a semaphore count.  Note that we cannot do this in
       * in the orthodox way by calling sem_wait() or sem_trywait()
       * because this function may be called from an interrupt
       * handler. Fortunately we know at at least one free buffer
       * so a simple decrement is all that is needed.
       */

      g_qentry_sem.semcount--;
      DEBUGASSERT(g_qentry_sem.semcount >= 0);

      /* Put the I/O buffer in a known state */

      iobq->qe_head = NULL; /* Nothing is contained */
    }

  irqrestore(flags);
  return iobq;
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
