/****************************************************************************
 * mm/iob/iob_clone.c
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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
 * Name: iob_clone
 *
 * Description:
 *   Duplicate (and pack) the data in iob1 in iob2.  iob2 must be empty.
 *
 ****************************************************************************/

int iob_clone(FAR struct iob_s *iob1, FAR struct iob_s *iob2, bool throttled,
              enum iob_user_e consumerid)
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

          next = iob_alloc(throttled, consumerid);
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
