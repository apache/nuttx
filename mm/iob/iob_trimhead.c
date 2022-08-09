/****************************************************************************
 * mm/iob/iob_trimhead.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_trimhead
 *
 * Description:
 *   Remove bytes from the beginning of an I/O chain.  Emptied I/O buffers
 *   are freed and, hence, the beginning of the chain may change.
 *
 ****************************************************************************/

FAR struct iob_s *iob_trimhead(FAR struct iob_s *iob, unsigned int trimlen,
                               enum iob_user_e producerid)
{
  unsigned int pktlen;

  iobinfo("iob=%p trimlen=%d\n", iob, trimlen);

  if (iob && trimlen > 0)
    {
      /* Trim from the head of the I/O buffer chain */

      pktlen = iob->io_pktlen;
      while (trimlen > 0 && iob != NULL)
        {
          /* Do we trim this entire I/O buffer away? */

          iobinfo("iob=%p io_len=%d pktlen=%d trimlen=%d\n",
                  iob, iob->io_len, pktlen, trimlen);

          if (iob->io_len <= trimlen)
            {
              FAR struct iob_s *next;

              /* Decrement the trim length and packet length by the full
               * data size.
               */

              pktlen  -= iob->io_len;
              trimlen -= iob->io_len;

              /* Check if this was the last entry in the chain */

              next = iob->io_flink;
              if (next == NULL)
                {
                  /* Yes.. break out of the loop returning the empty
                   * I/O buffer chain containing only one empty entry.
                   */

                  DEBUGASSERT(pktlen == 0);
                  iob->io_len    = 0;
                  iob->io_offset = 0;
                  break;
                }

              /* Free this entry and set the next I/O buffer as the head */

              iobinfo("iob=%p: Freeing\n", iob);
              iob_free(iob, producerid);
              iob = next;
            }
          else
            {
              /* No, then just take what we need from this I/O buffer and
               * stop the trim.
               */

              pktlen         -= trimlen;
              iob->io_len    -= trimlen;
              iob->io_offset += trimlen;
              trimlen         = 0;
            }
        }

      /* Adjust the pktlen by the number of bytes removed from the head
       * of the I/O buffer chain.
       */

      iob->io_pktlen = pktlen;
    }

  return iob;
}
