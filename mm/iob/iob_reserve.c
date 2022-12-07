/****************************************************************************
 * mm/iob/iob_reserve.c
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
  unsigned int offset;

  /* Empty iob buffer is allowed, update packet length. */

  if (iob->io_pktlen > reserved)
    {
      iob->io_pktlen -= reserved;
    }
  else
    {
      iob->io_pktlen = 0;
    }

  /* Update offset and reducing the tail room */

  while (iob != NULL && reserved > 0)
    {
      if (reserved > CONFIG_IOB_BUFSIZE)
        {
          offset = CONFIG_IOB_BUFSIZE;
        }
      else
        {
          offset = reserved;
        }

      if (iob->io_len > offset)
        {
          iob->io_len -= offset;
        }
      else
        {
          iob->io_len = 0;
        }

      iob->io_offset = offset;
      iob = iob->io_flink;
      reserved -= offset;
    }
}
