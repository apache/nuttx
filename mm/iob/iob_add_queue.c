/****************************************************************************
 * mm/iob/iob_add_queue.c
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
#include <errno.h>
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_add_queue_internal
 *
 * Description:
 *   Add one I/O buffer chain to the end of a queue.  May fail due to lack
 *   of resources.
 *
 ****************************************************************************/

static int iob_add_queue_internal(FAR struct iob_s *iob,
                                  FAR struct iob_queue_s *iobq,
                                  FAR struct iob_qentry_s *qentry)
{
  /* Add the I/O buffer chain to the container */

  qentry->qe_head = iob;

  /* Add the container to the end of the queue */

  qentry->qe_flink = NULL;
  if (!iobq->qh_head)
    {
      iobq->qh_head = qentry;
      iobq->qh_tail = qentry;
    }
  else
    {
      DEBUGASSERT(iobq->qh_tail);
      iobq->qh_tail->qe_flink = qentry;
      iobq->qh_tail = qentry;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_add_queue
 *
 * Description:
 *   Add one I/O buffer chain to the end of a queue.  May fail due to lack
 *   of resources.
 *
 ****************************************************************************/

int iob_add_queue(FAR struct iob_s *iob, FAR struct iob_queue_s *iobq)
{
  FAR struct iob_qentry_s *qentry;

  /* Allocate a container to hold the I/O buffer chain */

  qentry = iob_alloc_qentry();
  if (!qentry)
    {
      ioberr("ERROR: Failed to allocate a container\n");
      return -ENOMEM;
    }

  return iob_add_queue_internal(iob, iobq, qentry);
}

/****************************************************************************
 * Name: iob_tryadd_queue
 *
 * Description:
 *   Add one I/O buffer chain to the end of a queue without waiting for
 *   resources to become free.
 *
 ****************************************************************************/

int iob_tryadd_queue(FAR struct iob_s *iob, FAR struct iob_queue_s *iobq)
{
  FAR struct iob_qentry_s *qentry;

  /* Allocate a container to hold the I/O buffer chain */

  qentry = iob_tryalloc_qentry();
  if (!qentry)
    {
      ioberr("ERROR: Failed to allocate a container\n");
      return -ENOMEM;
    }

  return iob_add_queue_internal(iob, iobq, qentry);
}
#endif /* CONFIG_IOB_NCHAINS > 0 */
