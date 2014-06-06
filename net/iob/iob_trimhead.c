/****************************************************************************
 * net/iob/iob_trimhead.c
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

#include <assert.h>

#include <nuttx/net/iob.h>

#include "iob.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: iob_trimhead
 *
 * Description:
 *   Remove bytes from the beginning of an I/O chain.  Emptied I/O buffers
 *   are freed and, hence, the beginning of the chain may change.
 *
 ****************************************************************************/

FAR struct iob_s *iob_trimhead(FAR struct iob_s *iob, unsigned int trimlen)
{
  uint16_t pktlen;
  unsigned int len;

  if (iob && trimlen > 0)
    {
      /* Trim from the head of the I/IO buffer chain */

      pktlen = iob->io_pktlen;
      len    = trimlen;

      while (len > 0 && iob != NULL)
        {
          /* Do we trim this entire I/O buffer away? */

          if (iob->io_len <= len)
            {
              FAR struct iob_s *next;

              /* Decrement the trim length and packet length by the full
               * data size.
               */

              pktlen        -= iob->io_len;
              len           -= iob->io_len;
              iob->io_len    = 0;
              iob->io_offset = 0;

              /* Check if this was the last entry in the chain */

              next = (FAR struct iob_s *)iob->io_link.flink;
              if (!next)
                {
                  /* Yes.. break out of the loop returning the empty
                   * I/O buffer chain containing only one empty entry.
                   */

                  DEBUGASSERT(pktlen == 0);
                  break;
                }

              /* Free this entry and set the next I/O buffer as the head */

              (void)iob_free(iob);
              iob   = next;
            }
          else
            {
              /* No, then just take what we need from this I/O buffer and
               * stop the trim.
               */

              pktlen         -= len;
              iob->io_len    -= len;
              iob->io_offset += len;
              len             = 0;
            }
        }

      /* Adjust the pktlen by the number of bytes removed from the head
       * of the I/O buffer chain.
       */

      iob->io_pktlen = pktlen;
    }

  return iob;
}
