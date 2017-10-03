/****************************************************************************
 * mm/iob/iob_free.c
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

#include <semaphore.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_free
 *
 * Description:
 *   Free the I/O buffer at the head of a buffer chain returning it to the
 *   free list.  The link to  the next I/O buffer in the chain is return.
 *
 ****************************************************************************/

FAR struct iob_s *iob_free(FAR struct iob_s *iob)
{
  FAR struct iob_s *next = iob->io_flink;
  irqstate_t flags;

  iobinfo("iob=%p io_pktlen=%u io_len=%u next=%p\n",
          iob, iob->io_pktlen, iob->io_len, next);

  /* Copy the data that only exists in the head of a I/O buffer chain into
   * the next entry.
   */

  if (next != NULL)
    {
      /* Copy and decrement the total packet length, being careful to
       * do nothing too crazy.
       */

      if (iob->io_pktlen > iob->io_len)
        {
          /* Adjust packet length and move it to the next entry */

          next->io_pktlen = iob->io_pktlen - iob->io_len;
          DEBUGASSERT(next->io_pktlen >= next->io_len);
        }
      else
        {
          /* This can only happen if the next entry is last entry in the
           * chain... and if it is empty
           */

          next->io_pktlen = 0;
          DEBUGASSERT(next->io_len == 0 && next->io_flink == NULL);
        }

      iobinfo("next=%p io_pktlen=%u io_len=%u\n",
              next, next->io_pktlen, next->io_len);
    }

  /* Free the I/O buffer by adding it to the head of the free or the
   * committed list. We don't know what context we are called from so
   * we use extreme measures to protect the free list:  We disable
   * interrupts very briefly.
   */

  flags = enter_critical_section();

  /* Which list?  If there is a task waiting for an IOB, then put
   * the IOB on either the free list or on the committed list where
   * it is reserved for that allocation (and not available to
   * iob_tryalloc()).
   */

  if (g_iob_sem.semcount < 0)
    {
      iob->io_flink   = g_iob_committed;
      g_iob_committed = iob;
    }
  else
    {
      iob->io_flink   = g_iob_freelist;
      g_iob_freelist  = iob;
    }

  /* Signal that an IOB is available.  If there is a thread waiting
   * for an IOB, this will wake up exactly one thread.  The semaphore
   * count will correctly indicated that the awakened task owns an
   * IOB and should find it in the committed list.
   */

  nxsem_post(&g_iob_sem);
#if CONFIG_IOB_THROTTLE > 0
  nxsem_post(&g_throttle_sem);
#endif
  leave_critical_section(flags);

  /* And return the I/O buffer after the one that was freed */

  return next;
}
