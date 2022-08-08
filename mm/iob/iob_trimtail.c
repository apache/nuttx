/****************************************************************************
 * mm/iob/iob_trimtail.c
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

#include <string.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_trimtail
 *
 * Description:
 *   Remove bytes from the end of an I/O chain
 *
 ****************************************************************************/

FAR struct iob_s *iob_trimtail(FAR struct iob_s *iob, unsigned int trimlen)
{
  FAR struct iob_s *entry;
  FAR struct iob_s *penultimate;
  FAR struct iob_s *last;
  int len;

  iobinfo("iob=%p pktlen=%d trimlen=%d\n", iob, iob->io_pktlen, trimlen);

  if (iob && trimlen > 0)
    {
      len = trimlen;

      /* Loop until complete the trim */

      while (len > 0)
        {
          /* Calculate the total length of the data in the I/O buffer
           * chain and find the last entry in the chain.
           */

          penultimate = NULL;
          last = NULL;

          for (entry = iob; entry; entry = entry->io_flink)
            {
              /* Remember the last and the next to the last in the chain */

              penultimate = last;
              last = entry;
            }

          /* Trim from the last entry in the chain.  Do we trim this entire
           * I/O buffer away?
           */

          iobinfo("iob=%p len=%d vs %d\n", last, last->io_len, len);
          if (last->io_len <= len)
            {
              /* Yes.. Consume the entire buffer */

              iob->io_pktlen -= last->io_len;
              len            -= last->io_len;
              last->io_len    = 0;

              /* Free the last, empty buffer in the list */

              iob_free(last);

              /* There should be a buffer before this one */

              if (!penultimate)
                {
                  /* No.. we just freed the head of the chain */

                  return NULL;
                }

              /* Unlink the penultimate from the freed buffer */

              penultimate->io_flink = NULL;
            }

          else
            {
              /* No, then just take what we need from this I/O buffer and
               * stop the trim.
               */

              iob->io_pktlen -= len;
              last->io_len   -= len;
              len             = 0;
            }
        }
    }

  return iob;
}
