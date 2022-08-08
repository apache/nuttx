/****************************************************************************
 * mm/iob/iob_trimhead_queue.c
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
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

#if CONFIG_IOB_NCHAINS > 0

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
                                     unsigned int trimlen)
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

          iob = iob_trimhead(iob, trimlen);
          qentry->qe_head = iob;
        }
    }

  /* Return the new I/O buffer chain at the head of the queue */

  return iob;
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
