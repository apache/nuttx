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
#include <sys/param.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Static Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_next
 *
 * Description:
 *   Allocate or reinitialize the next node
 *
 ****************************************************************************/

static int iob_next(FAR struct iob_s *iob, bool throttled, bool block)
{
  FAR struct iob_s *next = iob->io_flink;

  /* Allocate new destination I/O buffer and hook it into the
   * destination I/O buffer chain.
   */

  if (next == NULL)
    {
      if (block)
        {
          next = iob_alloc(throttled);
        }
      else
        {
          next = iob_tryalloc(throttled);
        }

      if (next == NULL)
        {
          ioberr("ERROR: Failed to allocate an I/O buffer\n");
          return -ENOMEM;
        }

      iob->io_flink = next;
    }
  else
    {
      next->io_len    = 0;
      next->io_offset = 0;
      next->io_pktlen = 0;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_clone_partial
 *
 * Description:
 *   Duplicate the data from partial bytes of iob1 to iob2
 *
 * Input Parameters:
 *   iob1      - Pointer to source iob_s
 *   len       - Number of bytes to copy
 *   offset1   - Offset of source iobs_s
 *   iob2      - Pointer to destination iob_s
 *   offset2   - Offset of destination iobs_s
 *   throttled - An indication of the IOB allocation is "throttled"
 *   block     - Flag of Enable/Disable nonblocking operation
 *
 * Returned Value:
 *   == 0  - Partial clone successfully.
 *   < 0   - No available to clone to destination iob.
 *
 ****************************************************************************/

int iob_clone_partial(FAR struct iob_s *iob1, unsigned int len,
                      unsigned int offset1, FAR struct iob_s *iob2,
                      unsigned int offset2, bool throttled, bool block)
{
  FAR uint8_t *src;
  FAR uint8_t *dest;
  unsigned int ncopy;
  unsigned int avail1;
  unsigned int avail2;
  int ret;

  /* Copy the total packet size from the I/O buffer at the head of the
   * chain.
   */

  iob2->io_pktlen = len + offset2;

  /* Handle special case where there are empty buffers at the head
   * the list, Skip I/O buffer containing the data offset.
   */

  while (iob1 != NULL && offset1 >= iob1->io_len)
    {
      offset1 -= iob1->io_len;
      iob1     = iob1->io_flink;
    }

  /* Skip requested offset from the destination iob */

  while (iob2 != NULL)
    {
      avail2 = CONFIG_IOB_BUFSIZE - iob2->io_offset;
      if (offset2 < avail2)
        {
          break;
        }

      iob2->io_len = avail2;
      offset2     -= iob2->io_len;

      ret = iob_next(iob2, throttled, block);
      if (ret < 0)
        {
          return ret;
        }

      iob2 = iob2->io_flink;
    }

  /* Pack each entry from iob1 to iob2 */

  while (iob1 && len > 0)
    {
      /* Get the source I/O buffer pointer and the number of bytes to copy
       * from this address.
       */

      src    = &iob1->io_data[iob1->io_offset + offset1];
      avail1 = iob1->io_len - offset1;

      /* Get the destination I/O buffer pointer and the number of bytes to
       * copy to that address.
       */

      dest   = &iob2->io_data[iob2->io_offset + offset2];
      avail2 = CONFIG_IOB_BUFSIZE - iob2->io_offset - offset2;

      /* Copy the smaller of the two and update the srce and destination
       * offsets.
       */

      ncopy = MIN(avail1, avail2);
      if (ncopy > len)
        {
          ncopy = len;
        }

      len -= ncopy;

      memcpy(dest, src, ncopy);

      offset1      += ncopy;
      offset2      += ncopy;
      iob2->io_len  = offset2;

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

      if (offset2 >= (CONFIG_IOB_BUFSIZE - iob2->io_offset) &&
          iob1 != NULL)
        {
          ret = iob_next(iob2, throttled, block);
          if (ret < 0)
            {
              return ret;
            }

          iob2 = iob2->io_flink;
          offset2 = 0;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: iob_clone
 *
 * Description:
 *   Duplicate (and pack) the data in iob1 in iob2.  iob2 must be empty.
 *
 ****************************************************************************/

int iob_clone(FAR struct iob_s *iob1, FAR struct iob_s *iob2,
              bool throttled, bool block)
{
  DEBUGASSERT(iob2->io_len == 0 && iob2->io_offset == 0 &&
            iob2->io_pktlen == 0 && iob2->io_flink == NULL);

  return iob_clone_partial(iob1, iob1->io_pktlen, 0,
                           iob2, 0, throttled, block);
}
