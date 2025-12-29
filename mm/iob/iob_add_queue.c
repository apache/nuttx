/****************************************************************************
 * mm/iob/iob_add_queue.c
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

#include <assert.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_add_queue
 *
 * Description:
 *   Add one I/O buffer chain to the end of a queue.
 *
 ****************************************************************************/

int iob_add_queue(FAR struct iob_s *iob, FAR struct iob_queue_s *iobq)
{
  FAR struct iob_s *last;

  DEBUGASSERT(iob != NULL && iobq != NULL);

  /* Find the last IOB in this chain */

  last = iob;
  while (IOB_LINK_GET(last) != NULL)
    {
      last = IOB_LINK_GET(last);
    }

  /* Mark this IOB as the end of a chain in the queue using
   * io_flink low bits
   */

  IOB_FLAG_SET(last, NULL, IOB_CHAIN_END);

  /* Add to the end of the queue */

  if (iobq->qh_head == NULL)
    {
      /* Empty queue */

      iobq->qh_head = iob;
      iobq->qh_tail = last;
    }
  else
    {
      /* Link after current tail, setting the chain end flag */

      DEBUGASSERT(iobq->qh_tail != NULL);
      IOB_FLAG_SET(iobq->qh_tail, iob, IOB_CHAIN_END);
      iobq->qh_tail = last;
    }

  return 0;
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
  /* No resource allocation needed, just call iob_add_queue */

  return iob_add_queue(iob, iobq);
}

/****************************************************************************
 * Name: iob_concat_queue
 *
 * Description:
 *   Concatenate iob_s queue src to dest
 *
 ****************************************************************************/

int iob_concat_queue(FAR struct iob_queue_s *dest,
                     FAR struct iob_queue_s *src)
{
  FAR struct iob_s *iob;
  FAR struct iob_s *tail;

  /* Detach the list from the source queue */

  iob = src->qh_head;
  tail = src->qh_tail;
  src->qh_head = NULL;
  src->qh_tail = NULL;

  if (iob != NULL)
    {
      if (dest->qh_head == NULL)
        {
          /* Destination queue is empty */

          dest->qh_head = iob;
          dest->qh_tail = tail;
        }
      else
        {
          /* Link source to destination tail, preserving CHAIN_END flag */

          DEBUGASSERT(dest->qh_tail != NULL);
          IOB_FLAG_SET(dest->qh_tail, iob, IOB_CHAIN_END);
          dest->qh_tail = tail;
        }
    }

  return 0;
}
