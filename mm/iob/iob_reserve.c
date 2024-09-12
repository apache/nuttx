/****************************************************************************
 * mm/iob/iob_reserve.c
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

#include <nuttx/config.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_reserve
 *
 * Description:
 *   Adjust headroom offset of iobs by reducing the tail room.
 *
 ****************************************************************************/

void iob_reserve(FAR struct iob_s *iob, unsigned int reserved)
{
  FAR struct iob_s *head = iob;
  unsigned int offset;
  int trimlen;

  /* Update offset and adjust packet length. */

  while (iob != NULL && reserved > 0)
    {
      if (reserved > IOB_BUFSIZE(iob))
        {
          offset = IOB_BUFSIZE(iob);
        }
      else
        {
          offset = reserved;
        }

      trimlen = offset - iob->io_offset;

      /* At most trim iob->io_len to 0. */

      if ((int)(iob->io_len - trimlen) < 0)
        {
          trimlen = iob->io_len;
        }

      head->io_pktlen -= trimlen;
      iob->io_len     -= trimlen;
      iob->io_offset   = offset;

      iob = iob->io_flink;
      reserved -= offset;
    }
}
