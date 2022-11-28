/****************************************************************************
 * mm/iob/iob_update_pktlen.c
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
 *   trimmed if the length of the iob chain is greater than the current
 *   length.
 *   This function will not grow the iob link, any grow operation should
 *   be implemented through iob_copyin()/iob_trycopyin().
 *
 ****************************************************************************/

void iob_update_pktlen(FAR struct iob_s *iob, unsigned int pktlen)
{
  FAR struct iob_s *penultimate;
  FAR struct iob_s *next;
  uint16_t offset = 0;
  int ninqueue = 0;
  int nrequire;
  uint16_t len;

  /* The data offset must be less than CONFIG_IOB_BUFSIZE */

  if (iob == NULL)
    {
      return;
    }

  /* Calculate the total entries of the data in the I/O buffer chain */

  next = iob;
  while (next != NULL)
    {
      ninqueue++;
      offset += next->io_offset;
      next    = next->io_flink;
    }

  /* Trim inqueue entries if needed */

  nrequire = (pktlen + offset + CONFIG_IOB_BUFSIZE - 1) /
             CONFIG_IOB_BUFSIZE;
  if (nrequire == 0)
    {
      nrequire = 1;
    }

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

  iob->io_pktlen = pktlen;

  /* Update size of each iob */

  next = iob;
  while (next != NULL && pktlen > 0)
    {
      if (pktlen + next->io_offset > CONFIG_IOB_BUFSIZE)
        {
          len = CONFIG_IOB_BUFSIZE - next->io_offset;
        }
      else
        {
          len = pktlen;
        }

      next->io_len = len;
      pktlen      -= len;
      next         = next->io_flink;
    }
}
