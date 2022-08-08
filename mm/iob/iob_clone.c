/****************************************************************************
 * mm/iob/iob_clone.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_clone
 *
 * Description:
 *   Duplicate (and pack) the data in iob1 in iob2.  iob2 must be empty.
 *
 ****************************************************************************/

int iob_clone(FAR struct iob_s *iob1, FAR struct iob_s *iob2, bool throttled)
{
  FAR uint8_t *src;
  FAR uint8_t *dest;
  unsigned int ncopy;
  unsigned int avail1;
  unsigned int avail2;
  unsigned int offset1;
  unsigned int offset2;

  DEBUGASSERT(iob2->io_len == 0 && iob2->io_offset == 0 &&
              iob2->io_pktlen == 0 && iob2->io_flink == NULL);

  /* Copy the total packet size from the I/O buffer at the head of the
   * chain.
   */

  iob2->io_pktlen = iob1->io_pktlen;

  /* Handle special case where there are empty buffers at the head
   * the list.
   */

  while (iob1->io_len <= 0)
    {
      iob1 = iob1->io_flink;
    }

  /* Pack each entry from iob1 to iob2 */

  offset1 = 0;
  offset2 = 0;

  while (iob1)
    {
      /* Get the source I/O buffer pointer and the number of bytes to copy
       * from this address.
       */

      src    = &iob1->io_data[iob1->io_offset + offset1];
      avail1 = iob1->io_len - offset1;

      /* Get the destination I/O buffer pointer and the number of bytes to
       * copy to that address.
       */

      dest   = &iob2->io_data[offset2];
      avail2 = CONFIG_IOB_BUFSIZE - offset2;

      /* Copy the smaller of the two and update the srce and destination
       * offsets.
       */

      ncopy = MIN(avail1, avail2);
      memcpy(dest, src, ncopy);

      offset1 += ncopy;
      offset2 += ncopy;

      /* Have we taken all of the data from the source I/O buffer? */

      if (offset1 >= iob1->io_len)
        {
          /* Skip over empty entries in the chain (there should not be any
           * but just to be safe).
           */

          do
            {
              /* Yes.. move to the next source I/O buffer */

              iob1 = iob1->io_flink;
            }
          while (iob1 && iob1->io_len <= 0);

          /* Reset the offset to the beginning of the I/O buffer */

          offset1 = 0;
        }

      /* Have we filled the destination I/O buffer? Is there more data to be
       * transferred?
       */

      if (offset2 >= CONFIG_IOB_BUFSIZE && iob1 != NULL)
        {
          FAR struct iob_s *next;

          /* Allocate new destination I/O buffer and hook it into the
           * destination I/O buffer chain.
           */

          next = iob_alloc(throttled);
          if (!next)
            {
              ioberr("ERROR: Failed to allocate an I/O buffer\n");
              return -ENOMEM;
            }

          iob2->io_flink = next;
          iob2 = next;
          offset2 = 0;
        }
    }

  return 0;
}
