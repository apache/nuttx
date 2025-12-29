/****************************************************************************
 * mm/iob/iob_free_queue_qentry.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_free_queue_qentry
 *
 * Description:
 *   Free an IOB chain and remove it from the queue.
 *
 ****************************************************************************/

void iob_free_queue_qentry(FAR struct iob_s *iob,
                           FAR struct iob_queue_s *iobq)
{
  FAR struct iob_s *prev = NULL;
  FAR struct iob_s *cur;
  FAR struct iob_s *last;

  /* Iterate through all chains in the queue */

  cur = iobq->qh_head;
  while (cur != NULL)
    {
      /* Check if this is the target chain */

      if (cur == iob)
        {
          /* Find the last IOB in this chain */

          last = cur;
          while (!IOB_FLAG_TEST(last, IOB_CHAIN_END))
            {
              last = IOB_LINK_GET(last);
            }

          /* Unlink this chain from the queue */

          if (prev == NULL)
            {
              /* Removing from head */

              iobq->qh_head = IOB_LINK_GET(last);
            }
          else
            {
              /* Link previous chain to next chain */

              IOB_FLAG_SET(prev, IOB_LINK_GET(last), IOB_CHAIN_END);
            }

          /* Update tail if necessary */

          if (iobq->qh_tail == last)
            {
              iobq->qh_tail = prev;
            }

          /* Detach this chain */

          last->io_flink = NULL;

          /* Free the I/O buffer chain */

          iob_free_chain(iob);

          return;
        }

      /* Skip to the end of this chain */

      while (!IOB_FLAG_TEST(cur, IOB_CHAIN_END))
        {
          cur = IOB_LINK_GET(cur);
        }

      /* Save as prev for next iteration */

      prev = cur;

      /* Move to next chain */

      cur = IOB_LINK_GET(cur);
    }
}
