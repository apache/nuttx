/****************************************************************************
 * mm/iob/iob_pack.c
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

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_pack
 *
 * Description:
 *   Pack all data in the I/O buffer chain so that the data offset is zero
 *   and all but the final buffer in the chain are filled.  Any emptied
 *   buffers at the end of the chain are freed.
 *
 ****************************************************************************/

FAR struct iob_s *iob_pack(FAR struct iob_s *iob,
                           enum iob_user_e producerid)
{
  FAR struct iob_s *head;
  FAR struct iob_s *next;
  unsigned int ncopy;
  unsigned int navail;

  /* Handle special cases */

  while (iob->io_len <= 0)
    {
      iob = iob_free(iob, producerid);
      if (iob == NULL)
        {
          return NULL;
        }
    }

  /* Now remember the head of the chain (for the return value) */

  head = iob;

  /* Pack each entry in the list */

  while (iob)
    {
      next = iob->io_flink;

      /* Eliminate the data offset in this entry */

      if (iob->io_offset > 0)
        {
          memcpy(iob->io_data, &iob->io_data[iob->io_offset], iob->io_len);
          iob->io_offset = 0;
        }

      /* Is there a buffer after this one? */

      if (next)
        {
          /* How many bytes can we copy from the next I/O buffer.  Limit the
           * size of the copy to the amount of free space in current I/O
           * buffer
           */

          ncopy  = next->io_len;
          navail = CONFIG_IOB_BUFSIZE - iob->io_len;
          if (ncopy > navail)
            {
              ncopy = navail;
            }

          if (ncopy > 0)
            {
              /* Copy the data from the next into the current I/O buffer iob */

              memcpy(&iob->io_data[iob->io_len],
                     &next->io_data[next->io_offset],
                     ncopy);

              /* Adjust lengths and offsets */

              iob->io_len     += ncopy;
              next->io_len    -= ncopy;
              next->io_offset += ncopy;
            }

          /* Have we consumed all of the data in the next entry? */

          if (next->io_len <= 0)
            {
              /* Yes.. free the next entry in I/O buffer chain */

              next          = iob_free(next, producerid);
              iob->io_flink = next;
            }
        }

      /* Set up to pack the next entry in the chain */

      iob = next;
    }

  return head;
}
