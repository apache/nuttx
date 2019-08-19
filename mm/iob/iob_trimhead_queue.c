/****************************************************************************
 * mm/iob/iob_trimhead_queue.c
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
#include <debug.h>

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
 * Name: iob_trimhead_queue
 *
 * Description:
 *   Remove bytes from the beginning of an I/O chain at the head of the
 *   queue.  Emptied I/O buffers are freed and, hence, the head of the
 *   queue may change.
 *
 *   This function is just a wrapper around iob_trimhead() that assures that
 *   the I/O buffer chain at the head of queue is modified with the trimming
 *   operation.
 *
 * Returned Value:
 *   The new I/O buffer chain at the head of the queue is returned.
 *
 ****************************************************************************/

FAR struct iob_s *iob_trimhead_queue(FAR struct iob_queue_s *qhead,
                                     unsigned int trimlen,
                                     enum iob_user_e producerid)
{
  FAR struct iob_qentry_s *qentry;
  FAR struct iob_s *iob = NULL;

  /* Peek at the I/O buffer chain container at the head of the queue */

  qentry = qhead->qh_head;
  if (qentry)
    {
      /* Verify that the queue entry contains an I/O buffer chain */

      iob = qentry->qe_head;
      if (iob)
        {
          /* Trim the I/Buffer chain and update the queue head */

          iob = iob_trimhead(iob, trimlen, producerid);
          qentry->qe_head = iob;
        }
    }

  /* Return the new I/O buffer chain at the head of the queue */

  return iob;
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
