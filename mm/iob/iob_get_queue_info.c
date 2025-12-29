/****************************************************************************
 * mm/iob/iob_get_queue_info.c
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
 * Name: iob_get_queue_size
 *
 * Description:
 *   Queue helper for get the iob queue buffer size.
 *
 ****************************************************************************/

unsigned int iob_get_queue_size(FAR struct iob_queue_s *queue)
{
  FAR struct iob_s *iob;
  unsigned int total = 0;

  /* Iterate through all chains in the queue */

  iob = queue->qh_head;
  while (iob != NULL)
    {
      /* Add this chain's packet length */

      total += iob->io_pktlen;

      /* Move to next chain (skip to chain boundary) */

      while (!IOB_FLAG_TEST(iob, IOB_CHAIN_END))
        {
          iob = IOB_LINK_GET(iob);
        }

      /* Move to next chain head */

      iob = IOB_LINK_GET(iob);
    }

  return total;
}

/****************************************************************************
 * Name: iob_get_queue_entry_count
 *
 * Description:
 *   Queue helper for get the iob queue entry count.
 *
 ****************************************************************************/

unsigned int iob_get_queue_entry_count(FAR struct iob_queue_s *queue)
{
  FAR struct iob_s *iob;
  unsigned int count = 0;

  /* Iterate through all chains in the queue */

  iob = queue->qh_head;
  while (iob != NULL)
    {
      /* Count this chain */

      count++;

      /* Skip to chain boundary */

      while (!IOB_FLAG_TEST(iob, IOB_CHAIN_END))
        {
          iob = IOB_LINK_GET(iob);
        }

      /* Move to next chain head */

      iob = IOB_LINK_GET(iob);
    }

  return count;
}
