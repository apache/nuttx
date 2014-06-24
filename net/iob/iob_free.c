/****************************************************************************
 * net/iob/iob_free.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/net/iob.h>

#include "iob.h"

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

  nllvdbg("iob=%p io_pktlen=%u io_len=%u next=%p\n",
          iob, iob->io_pktlen, iob->io_len, next);

  /* Copy the data that only exists in the head of a I/O buffer chain into
   * the next entry.
   */

  if (next)
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

      nllvdbg("next=%p io_pktlen=%u io_len=%u\n",
               next, next->io_pktlen, next->io_len);
    }

  /* Free the I/O buffer by adding it to the head of the free list. We don't
   * know what context we are called from so we use extreme measures to
   * protect the free list:  We disable interrupts very briefly.
   */

  flags = irqsave();
  iob->io_flink = g_iob_freelist;
  g_iob_freelist = iob;

  /* Signal that an IOB is available */

  sem_post(&g_iob_sem);
  irqrestore(flags);

  /* And return the I/O buffer after the one that was freed */

  return next;
}
