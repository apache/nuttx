/****************************************************************************
 * mm/iob/iob_free_qentry.c
 *
 *   Copyright (C) 2014, 2016-2017 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

#if CONFIG_IOB_NCHAINS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_free_qentry
 *
 * Description:
 *   Free the I/O buffer chain container by returning it to the free list.
 *   The link to  the next I/O buffer in the chain is return.
 *
 ****************************************************************************/

FAR struct iob_qentry_s *iob_free_qentry(FAR struct iob_qentry_s *iobq)
{
  FAR struct iob_qentry_s *nextq = iobq->qe_flink;
  irqstate_t flags;

  /* Free the I/O buffer chain container by adding it to the head of the
   * free or the committed list. We don't know what context we are called
   * from so we use extreme measures to protect the free list:  We disable
   * interrupts very briefly.
   */

  flags = enter_critical_section();

  /* Which list?  If there is a task waiting for an IOB chain, then put
   * the IOB chain on either the free list or on the committed list where
   * it is reserved for that allocation (and not available to
   * iob_tryalloc_qentry()).
   */

  if (g_qentry_sem.semcount < 0)
    {
      iobq->qe_flink   = g_iob_qcommitted;
      g_iob_qcommitted = iobq;
    }
  else
    {
      iobq->qe_flink   = g_iob_freeqlist;
      g_iob_freeqlist  = iobq;
    }

  /* Signal that an I/O buffer chain container is available.  If there
   * is a thread waiting for an I/O buffer chain container, this will
   * wake up exactly one thread.  The semaphore count will correctly
   * indicated that the awakened task owns an I/O buffer chain container
   * and should find it in the committed list.
   */

  nxsem_post(&g_qentry_sem);
  leave_critical_section(flags);

  /* And return the I/O buffer chain container after the one that was freed */

  return nextq;
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
