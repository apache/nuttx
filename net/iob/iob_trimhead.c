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
  FAR struct iob_s *entry;
  uint8_t flags;
  uint16_t pktlen;
  uint16_t vtag;
  void *priv;
  unsigned int len;

  if (iob && trimlen > 0)
    {
      /* Save information from the head of the chain (in case the
       * head is removed).
       */

      flags  = iob->io_flags;
      pktlen = iob->io_pktlen;
      vtag   = iob->io_vtag;
      priv   = iob->io_priv;

      /* Trim from the head of the I/IO buffer chain */

      entry  = iob;
      len    = trimlen;

      while (entry != NULL)
        {
          /* Do we trim this entire I/O buffer away? */

          if (entry->io_len <= len)
            {
              /* Decrement the trim length by the full data size */

              pktlen -= entry->io_len;
              len    -= entry->io_len;

              /* Free this one and set the next I/O buffer as the head */

              iob =  (FAR struct iob_s *)entry->io_link.flink;
              iob_free(entry);

              /* Continue with the new buffer head */

              entry = iob;
            }
          else
            {
              /* No, then just take what we need from this I/O buffer and
               * stop the trim.
               */

              pktlen           -= len;
              entry->io_len    -= len;
              entry->io_offset += len;
              len = 0;
            }
        }

      /* Restore the state to the head of the chain (which may not be
       * the same I/O buffer chain head that we started with).
       *
       * Adjust the pktlen by the number of bytes removed from the head
       * of the I/O buffer chain.  A special case is where we delete the
       * entire chain:  len > 0 and iob == NULL.
       */

      iob->io_flags  = flags;
      iob->io_pktlen = pktlen;
      iob->io_vtag   = vtag;
      iob->io_priv   = priv;
    }

  return iob;
}
