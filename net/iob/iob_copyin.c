/****************************************************************************
 * net/iob/iob_copyin.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <nuttx/net/iob.h>

#include "iob.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
               unsigned int len, unsigned int offset)
{
  FAR struct iob_s *head = iob;
  FAR struct iob_s *next;
  FAR uint8_t *dest;
  unsigned int ncopy;
  unsigned int avail;

  DEBUGASSERT(iob && src);

  /* The offset must applied to data that is already in the I/O buffer chain */

  if (offset > iob->io_pktlen)
    {
      ndbg("ERROR: offset is past the end of data: %d > %d\n",
           offset, iob->io_pktlen);
      return -ESPIPE;
    }

  /* Skip to the I/O buffer containing the data offset */

  while (offset > iob->io_len)
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

              maxlen  = CONFIG_IOB_BUFSIZE - iob->io_offset;

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
          /* Yes.. allocate a new buffer */

          next = iob_alloc();
          if (next == NULL)
            {
              ndbg("ERROR: Failed to allocate I/O buffer\n");
              return -ENOMEM;
            }

          /* Add the new, empty I/O buffer to the end of the buffer chain. */

          iob->io_flink = next;
        }

      iob = next;
      offset = 0;
    }

  return 0;
}
