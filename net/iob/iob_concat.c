/****************************************************************************
 * net/iob/iob_concat.c
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
#include <queue.h>

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
 * Name: iob_concat
 *
 * Description:
 *   Concatenate iob_s chain iob2 to iob1.
 *
 ****************************************************************************/

void iob_concat(FAR struct iob_s *iob1, FAR struct iob_s *iob2)
{
  unsigned int offset2;
  unsigned int ncopy;
  unsigned int navail;

  /* Find the last buffer in the iob1 buffer chain */
 
  while (iob1->io_link.flink)
    {
      iob1 = (FAR struct iob_s *)iob1->io_link.flink;
    }

  /* Then add data to the end of iob1 */

  offset2 = 0;
  while (iob2)
    {
      /* Is the iob1 tail buffer full? */

      if (iob1->io_len >= CONFIG_IOB_BUFSIZE)
        {
          /* Yes.. Just connect the chains */

          iob1->io_link.flink = iob2->io_link.flink;

          /* Has the data offset in iob2? */

          if (offset2 > 0)
            {
              /* Yes, move the data down and adjust the size */

              iob2->io_len -= offset2;
              memcpy(iob2->io_data, &iob2->io_data[offset2], iob2->io_len);

              /* Set up to continue packing, but now into iob2 */

              iob1 = iob2;
              iob2 = (FAR struct iob_s *)iob2->io_link.flink;

              iob1->io_link.flink = NULL;
              offset2 = 0;
            }
          else
            {
              /* Otherwise, we are done */

              return;
            }
        }

      /* How many bytes can we copy from the source (iob2) */

      ncopy = iob2->io_len - offset2;

      /* Limit the size of the copy to the amount of free space in iob1 */

      navail = CONFIG_IOB_BUFSIZE - iob1->io_len;
      if (ncopy > navail)
        {
          ncopy = navail;
        }

      /* Copy the data from iob2 into iob1 */

      memcpy(iob1->io_data + iob1->io_len, iob2->io_data, ncopy);
      iob1->io_len += ncopy;
      offset2 += ncopy;

      /* Have we consumed all of the data in the iob2 entry? */

      if (offset2 >= iob2->io_len)
        {
          /* Yes.. free the iob2 entry and start processing the next I/O
           * buffer in the iob2 chain.
           */

          iob2 = iob_free(iob2);
          offset2 = 0;
        }
    }
}
