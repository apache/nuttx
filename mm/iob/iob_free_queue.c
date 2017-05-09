/****************************************************************************
 * mm/iob/iob_free_queue.c
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

#include <assert.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

#if CONFIG_IOB_NCHAINS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL ((FAR void *)0)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_free_queue
 *
 * Description:
 *   Free an entire queue of I/O buffer chains.
 *
 ****************************************************************************/

void iob_free_queue(FAR struct iob_queue_s *qhead)
{
  FAR struct iob_qentry_s *iobq;
  FAR struct iob_qentry_s *nextq;
  FAR struct iob_s *iob;

  /* Detach the list from the queue head so first for safety (should be safe
   * anyway).
   */

  iobq           = qhead->qh_head;
  qhead->qh_head = NULL;

  /* Remove each I/O buffer chain from the queue */

  while (iobq)
    {
      /* Remove the I/O buffer chain from the head of the queue and
       * discard the queue container.
       */

      iob = iobq->qe_head;
      DEBUGASSERT(iob);

      /* Remove the queue container from the list and discard it */

      nextq = iobq->qe_flink;
      iob_free_qentry(iobq);
      iobq = nextq;

      /* Free the I/O chain */

      iob_free_chain(iob);
    }
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
