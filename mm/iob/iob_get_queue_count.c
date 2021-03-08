/****************************************************************************
 * mm/iob/iob_get_queue_count.c
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
#include <nuttx/mm/iob.h>
#include "iob.h"

#if CONFIG_IOB_NCHAINS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_get_queue_count
 *
 * Description:
 *   Queue helper for get the iob entry count.
 *
 ****************************************************************************/

int iob_get_queue_count(FAR struct iob_queue_s *queue)
{
  FAR struct iob_qentry_s *iobq;
  FAR struct iob_s *iob;
  int count = 0;

  for (iobq = queue->qh_head; iobq != NULL; iobq = iobq->qe_flink)
    {
      for (iob = iobq->qe_head; iob; iob = iob->io_flink)
        {
          count++;
        }
    }

  return count;
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
