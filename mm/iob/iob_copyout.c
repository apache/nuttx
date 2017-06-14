/****************************************************************************
 * mm/iob/iob_copyout.c
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
#include <assert.h>

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
 * Name: iob_copyout
 *
 * Description:
 *  Copy data 'len' bytes of data into the user buffer starting at 'offset'
 *  in the I/O buffer, returning that actual number of bytes copied out.
 *
 ****************************************************************************/

int iob_copyout(FAR uint8_t *dest, FAR const struct iob_s *iob,
                unsigned int len, unsigned int offset)
{
  FAR const uint8_t *src;
  unsigned int ncopy;
  unsigned int avail;
  unsigned int remaining;

  /* Skip to the I/O buffer containing the offset */

  while (offset >= iob->io_len)
    {
      offset -= iob->io_len;
      iob     = iob->io_flink;
      if (iob == NULL)
        {
          /* We have no requested data in iob chain */

          return 0;
        }
    }

  /* Then loop until all of the I/O data is copied to the user buffer */

  remaining = len;
  while (iob && remaining > 0)
    {
      /* Get the source I/O buffer offset address and the amount of data
       * available from that address.
       */

      src   = &iob->io_data[iob->io_offset + offset];
      avail = iob->io_len - offset;

      /* Copy the from the I/O buffer in to the user buffer */

      ncopy = MIN(avail, remaining);
      memcpy(dest, src, ncopy);

      /* Adjust the total length of the copy and the destination address in
       * the user buffer.
       */

      remaining -= ncopy;
      dest += ncopy;

      /* Skip to the next I/O buffer in the chain */

      iob = iob->io_flink;
      offset = 0;
    }

  return len - remaining;
}
