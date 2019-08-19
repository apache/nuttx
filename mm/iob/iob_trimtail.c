/****************************************************************************
 * mm/iob/iob_trimtail.c
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
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_trimtail
 *
 * Description:
 *   Remove bytes from the end of an I/O chain
 *
 ****************************************************************************/

FAR struct iob_s *iob_trimtail(FAR struct iob_s *iob, unsigned int trimlen,
                               enum iob_user_e producerid)
{
  FAR struct iob_s *entry;
  FAR struct iob_s *penultimate;
  FAR struct iob_s *last;
  int len;

  iobinfo("iob=%p pktlen=%d trimlen=%d\n", iob, iob->io_pktlen, trimlen);

  if (iob && trimlen > 0)
    {
      len = trimlen;

      /* Loop until complete the trim */

      while (len > 0)
        {
          /* Calculate the total length of the data in the I/O buffer
           * chain and find the last entry in the chain.
           */

          penultimate = NULL;
          last = NULL;

          for (entry = iob; entry; entry = entry->io_flink)
            {
              /* Remember the last and the next to the last in the chain */

              penultimate = last;
              last = entry;
            }

          /* Trim from the last entry in the chain.  Do we trim this entire
           * I/O buffer away?
           */

          iobinfo("iob=%p len=%d vs %d\n", last, last->io_len, len);
          if (last->io_len <= len)
            {
              /* Yes.. Consume the entire buffer */

              iob->io_pktlen -= last->io_len;
              len            -= last->io_len;
              last->io_len    = 0;

              /* Free the last, empty buffer in the list */

              iob_free(last, producerid);

              /* There should be a buffer before this one */

              if (!penultimate)
                {
                  /* No.. we just freed the head of the chain */

                  return NULL;
                }

              /* Unlink the penultimate from the freed buffer */

              penultimate->io_flink = NULL;
            }

          else
            {
              /* No, then just take what we need from this I/O buffer and
               * stop the trim.
               */

              iob->io_pktlen -= len;
              last->io_len   -= len;
              len             = 0;
            }
        }
    }

  return iob;
}
