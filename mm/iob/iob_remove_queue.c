/****************************************************************************
 * mm/iob/iob_remove_queue.c
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

#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
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
 * Name: iob_remove_queue
 *
 * Description:
 *   Remove and return one I/O buffer chain from the head of a queue.
 *
 * Returned Value:
 *   Returns a reference to the I/O buffer chain at the head of the queue.
 *
 ****************************************************************************/

FAR struct iob_s *iob_remove_queue(FAR struct iob_queue_s *iobq)
{
  FAR struct iob_qentry_s *qentry;
  FAR struct iob_s *iob = NULL;

  /* Remove the I/O buffer chain from the head of the queue */

  irqstate_t flags = enter_critical_section();
  qentry = iobq->qh_head;
  if (qentry)
    {
      iobq->qh_head = qentry->qe_flink;
      if (!iobq->qh_head)
        {
          iobq->qh_tail = NULL;
        }

      /* Extract the I/O buffer chain from the container and free the
       * container.
       */

      iob = qentry->qe_head;
      iob_free_qentry(qentry);
    }

  leave_critical_section(flags);
  return iob;
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
