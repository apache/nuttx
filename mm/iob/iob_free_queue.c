/****************************************************************************
 * mm/iob/iob_free_queue.c
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
 * Name: iob_free_queue
 *
 * Description:
 *   Free an entire queue of I/O buffer chains.
 *
 ****************************************************************************/

void iob_free_queue(FAR struct iob_queue_s *qhead)
{
  FAR struct iob_s *iob;
  FAR struct iob_s *next_chain;
  FAR struct iob_s *last;

  /* Detach the list from the queue head */

  iob = qhead->qh_head;
  qhead->qh_head = NULL;
  qhead->qh_tail = NULL;

  /* Free each I/O buffer chain */

  while (iob != NULL)
    {
      /* Find the last IOB in this chain */

      last = iob;
      while (!IOB_FLAG_TEST(last, IOB_CHAIN_END))
        {
          last = IOB_LINK_GET(last);
        }

      /* Save next chain head before freeing */

      next_chain = IOB_LINK_GET(last);

      /* Detach this chain */

      last->io_flink = NULL;

      /* Free this chain */

      iob_free_chain(iob);

      /* Move to next chain */

      iob = next_chain;
    }
}

