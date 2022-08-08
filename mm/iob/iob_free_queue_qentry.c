/****************************************************************************
 * mm/iob/iob_free_queue_qentry.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

#if CONFIG_IOB_NCHAINS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_free_queue_qentry
 *
 * Description:
 *   Free an iob entire queue of I/O buffer chains.
 *
 ****************************************************************************/

void iob_free_queue_qentry(FAR struct iob_s *iob,
                           FAR struct iob_queue_s *iobq)
{
  FAR struct iob_qentry_s *prev = NULL;
  FAR struct iob_qentry_s *qentry;

  irqstate_t flags = enter_critical_section();
  for (qentry = iobq->qh_head; qentry != NULL;
       prev = qentry, qentry = qentry->qe_flink)
    {
      /* Find head of the I/O buffer chain */

      if (qentry->qe_head == iob)
        {
          if (prev == NULL)
            {
              iobq->qh_head = qentry->qe_flink;
            }
          else
            {
              prev->qe_flink = qentry->qe_flink;
            }

          if (iobq->qh_tail == qentry)
            {
              iobq->qh_tail = prev;
            }

          /* Remove the queue container */

          iob_free_qentry(qentry);

          /* Free the I/O chain */

          iob_free_chain(iob);

          break;
        }
    }

  leave_critical_section(flags);
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
