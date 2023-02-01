/****************************************************************************
 * mm/iob/iob_contig.c
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
#include <sys/param.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_contig
 *
 * Description:
 *   Ensure that there is'len' bytes of contiguous space at the beginning
 *   of the I/O buffer chain starting at 'iob'.
 *
 ****************************************************************************/

int iob_contig(FAR struct iob_s *iob, unsigned int len)
{
  FAR struct iob_s *next;
  unsigned int ncopy;

  /* We can't make more contiguous space that the size of one I/O buffer.
   * If you get this assertion and really need that much contiguous data,
   * then you will need to increase CONFIG_IOB_BUFSIZE.
   */

  DEBUGASSERT(len <= CONFIG_IOB_BUFSIZE);

  /* Check if there is already sufficient, contiguous space at the beginning
   * of the packet
   */

  if (len <= iob->io_len)
    {
      /* Yes we are good */

      return 0;
    }

  /* Can we get the required amount of contiguous data by just packing the
   * head I/0 buffer?
   */

  else if (len <= iob->io_pktlen)
    {
      /* Yes.. First eliminate any leading offset */

      if (iob->io_offset > 0)
        {
          memcpy(iob->io_data, &iob->io_data[iob->io_offset], iob->io_len);
          iob->io_offset = 0;
        }

      /* Then move what we need from the next I/O buffer(s) */

      do
        {
          /* Get the next I/O buffer in the chain */

          next = iob->io_flink;
          DEBUGASSERT(next != NULL);

          /* Copy what we need or what we can from the next buffer */

          ncopy = len - iob->io_len;
          ncopy = MIN(ncopy, next->io_len);
          memcpy(&iob->io_data[iob->io_len],
                 &next->io_data[next->io_offset], ncopy);

          /* Adjust counts and offsets */

          iob->io_len     += ncopy;
          next->io_offset += ncopy;
          next->io_len    -= ncopy;

          /* Handle a (improbable) case where we just emptied the second
           * buffer in the chain.
           */

          if (next->io_len == 0)
            {
              iob->io_flink = iob_free(next);
            }
        }
      while (len > iob->io_len);

      /* This should always succeed because we know that:
       *
       *   pktlen >= CONFIG_IOB_BUFSIZE >= len
       */

      return 0;
    }

  /* Otherwise, the request for contiguous data is larger then the entire
   * packet.  We can't do that without extending the I/O buffer chain with
   * garbage (which would probably not be what the caller wants).
   */

  else
    {
      ioberr("ERROR: pktlen=%u < requested len=%u\n", iob->io_pktlen, len);
      return -ENOSPC;
    }
}
