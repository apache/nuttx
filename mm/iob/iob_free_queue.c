/****************************************************************************
 * mm/iob/iob_free_queue.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
