/****************************************************************************
 * mm/iob/iob_trimhead.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL ((FAR void *)0)
#endif

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
  uint16_t pktlen;

  iobinfo("iob=%p trimlen=%d\n", iob, trimlen);

  if (iob && trimlen > 0)
    {
      /* Trim from the head of the I/IO buffer chain */

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
