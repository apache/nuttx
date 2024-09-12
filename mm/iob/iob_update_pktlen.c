/****************************************************************************
 * mm/iob/iob_update_pktlen.c
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
 * Name: iob_update_pktlen
 *
 * Description:
 *   This function will update packet length of the iob, it will be
 *   trimmed if the current length of the iob chain is greater than the
 *   new length, and will be grown if less than new length.
 *
 * Returned Value:
 *   The new effective iob packet length, or a negated errno value on error.
 *
 ****************************************************************************/

int iob_update_pktlen(FAR struct iob_s *iob, unsigned int pktlen,
                      bool throttled)
{
  FAR struct iob_s *penultimate;
  FAR struct iob_s *next;
  int remain = pktlen;
  int ninqueue = 0;
  int nrequire = 0;
  uint16_t len;

  /* The data offset must be less than CONFIG_IOB_BUFSIZE */

  if (iob == NULL)
    {
      return -EINVAL;
    }

  /* Calculate the total entries of the data in the I/O buffer chain */

  next = iob;
  while (next != NULL)
    {
      ninqueue++;
      penultimate = next;
      if (remain > 0)
        {
          nrequire++;
          remain -= IOB_BUFSIZE(next) - next->io_offset;
        }

      next = next->io_flink;
    }

  if (remain > 0)
    {
      nrequire += (remain + CONFIG_IOB_BUFSIZE - 1) / CONFIG_IOB_BUFSIZE;
    }

  if (nrequire == 0)
    {
      nrequire = 1;
    }

  /* Trim inqueue entries if needed */

  if (nrequire < ninqueue)
    {
      /* Loop until complete the trim */

      next = iob;
      penultimate = NULL;

      while (next != NULL)
        {
          if (nrequire-- <= 0)
            {
              if (penultimate != NULL)
                {
                  penultimate->io_flink = NULL;
                }

              iob_free_chain(next);
              break;
            }
          else
            {
              penultimate = next;
              next = next->io_flink;
            }
        }
    }
  else if (nrequire > ninqueue)
    {
      /* Start from the last IOB */

      next = penultimate;

      /* Loop to extend the link */

      while (next != NULL && nrequire > ninqueue)
        {
          next->io_flink = iob_tryalloc(throttled);
          next = next->io_flink;
          ninqueue++;
        }
    }

  iob->io_pktlen = pktlen;

  /* Update size of each iob */

  next = iob;
  while (next != NULL && pktlen > 0)
    {
      if (pktlen + next->io_offset > IOB_BUFSIZE(next))
        {
          len = IOB_BUFSIZE(next) - next->io_offset;
        }
      else
        {
          len = pktlen;
        }

      next->io_len = len;
      pktlen      -= len;
      next         = next->io_flink;
    }

  /* Adjust final pktlen if it's not fully increased (e.g. alloc fail) */

  iob->io_pktlen -= pktlen;

  return iob->io_pktlen;
}
