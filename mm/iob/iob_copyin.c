/****************************************************************************
 * mm/iob/iob_copyin.c
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_copyin_internal
 *
 * Description:
 *  Copy data 'len' bytes from a user buffer into the I/O buffer chain,
 *  starting at 'offset', extending the chain as necessary.
 *
 * Returned Value:
 *  The number of uncopied bytes left if >= 0 OR a negative error code.
 *
 ****************************************************************************/

static int iob_copyin_internal(FAR struct iob_s *iob, FAR const uint8_t *src,
                               unsigned int len, int offset,
                               bool throttled, bool can_block)
{
  FAR struct iob_s *head = iob;
  FAR struct iob_s *next;
  FAR uint8_t *dest;
  unsigned int ncopy;
  unsigned int avail;
  unsigned int total = len;

  iobinfo("iob=%p len=%u offset=%d\n", iob, len, offset);
  DEBUGASSERT(iob && src);

  /* The offset must applied to data that is already in the I/O buffer
   * chain
   */

  if ((int)(offset - iob->io_pktlen) > 0)
    {
      ioberr("ERROR: offset is past the end of data: %d > %u\n",
             offset, iob->io_pktlen);
      return -ESPIPE;
    }

  if ((int)(offset + iob->io_offset) < 0)
    {
      ioberr("ERROR: offset is before the start of data: %d < %d\n",
             offset, -(int)iob->io_offset);
      return -ESPIPE;
    }

  /* Skip to the I/O buffer containing the data offset */

  while ((int)(offset - iob->io_len) > 0)
    {
      offset -= iob->io_len;
      iob     = iob->io_flink;
    }

  /* Then loop until all of the I/O data is copied from the user buffer */

  while (len > 0)
    {
      next = iob->io_flink;

      /* Get the destination I/O buffer address and the amount of data
       * available from that address.
       */

      dest  = &iob->io_data[iob->io_offset + offset];
      avail = iob->io_len - offset;

      iobinfo("iob=%p avail=%u len=%u next=%p\n", iob, avail, len, next);

      /* Will the rest of the copy fit into this buffer, overwriting
       * existing data.
       */

      if (len > avail)
        {
          /* No.. Is this the last buffer in the chain? */

          if (next)
            {
              /* No.. clip to size that will overwrite.  We cannot
               * extend the length of an I/O block in mid-chain.
               */

              ncopy = avail;
            }
          else
            {
              unsigned int maxlen;
              unsigned int newlen;

              /* Yes.. We can extend this buffer to the up to the very end. */

              maxlen = CONFIG_IOB_BUFSIZE - iob->io_offset;

              /* This is the new buffer length that we need.  Of course,
               * clipped to the maximum possible size in this buffer.
               */

              newlen = len + offset;
              if (newlen > maxlen)
                {
                  newlen = maxlen;
                }

              /* Set the new length and increment the packet length */

              head->io_pktlen += (newlen - iob->io_len);
              iob->io_len      = newlen;

              /* Set the new number of bytes to copy */

              ncopy = newlen - offset;
            }
        }
      else
        {
          /* Yes.. Copy all of the remaining bytes */

          ncopy = len;
        }

      /* Copy from the user buffer to the I/O buffer.  */

      memcpy(dest, src, ncopy);
      iobinfo("iob=%p Copy %u bytes new len=%u\n",
              iob, ncopy, iob->io_len);

      /* Adjust the total length of the copy and the destination address in
       * the user buffer.
       */

      len -= ncopy;
      src += ncopy;

      /* Skip to the next I/O buffer in the chain.  First, check if we
       * are at the end of the buffer chain.
       */

      if (len > 0 && !next)
        {
          /* Yes.. allocate a new buffer.
           *
           * Copy as many bytes as possible. Block if we're allowed.
           */

          if (can_block)
            {
              next = iob_alloc(throttled);
            }
          else
            {
              next = iob_tryalloc(throttled);
            }

          if (next == NULL)
            {
              ioberr("ERROR: Failed to allocate I/O buffer\n");
              return -ENOMEM;
            }

          /* Add the new, empty I/O buffer to the end of the buffer chain. */

          iob->io_flink = next;
          iobinfo("iob=%p added to the chain\n", iob);
        }

      iob = next;
      offset = 0;
    }

  return total;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_copyin
 *
 * Description:
 *  Copy data 'len' bytes from a user buffer into the I/O buffer chain,
 *  starting at 'offset', extending the chain as necessary.
 *
 ****************************************************************************/

int iob_copyin(FAR struct iob_s *iob, FAR const uint8_t *src,
               unsigned int len, int offset, bool throttled)
{
  return iob_copyin_internal(iob, src, len, offset, throttled, true);
}

/****************************************************************************
 * Name: iob_trycopyin
 *
 * Description:
 *  Copy data 'len' bytes from a user buffer into the I/O buffer chain,
 *  starting at 'offset', extending the chain as necessary BUT without
 *  waiting if buffers are not available.
 *
 ****************************************************************************/

int iob_trycopyin(FAR struct iob_s *iob, FAR const uint8_t *src,
                  unsigned int len, int offset, bool throttled)
{
  return iob_copyin_internal(iob, src, len, offset, throttled, false);
}
