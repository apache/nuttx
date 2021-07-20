/****************************************************************************
 * mm/iob/iob_check.c
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

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_check
 *
 * Description:
 *   Perform sanity checks on the IOB chain.
 *
 ****************************************************************************/

void iob_check(FAR struct iob_s *head)
{
  FAR struct iob_s *iob;
  FAR struct iob_s *next;
  unsigned int pktlen = 0;

  for (iob = head; iob != NULL; iob = next)
    {
      next = iob->io_flink;

      DEBUGASSERT(iob->io_offset + iob->io_len <= CONFIG_IOB_BUFSIZE);
      DEBUGASSERT(iob->io_offset < CONFIG_IOB_BUFSIZE);

      /* it's unusual that an IOB within a chain has 0 length */

      DEBUGASSERT(iob->io_len > 0 || (head == iob && next == NULL));
      pktlen += iob->io_len;
    }

  DEBUGASSERT(head == NULL || head->io_pktlen == pktlen);
}
