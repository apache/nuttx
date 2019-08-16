/****************************************************************************
 * mm/iob/iob_contig.c
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
 * Name: iob_contig
 *
 * Description:
 *   Ensure that there is'len' bytes of contiguous space at the beginning
 *   of the I/O buffer chain starting at 'iob'.
 *
 ****************************************************************************/

int iob_contig(FAR struct iob_s *iob, unsigned int len,
               enum iob_user_e producerid)
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
              iob->io_flink = iob_free(next, producerid);
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
