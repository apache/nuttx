/****************************************************************************
 * mm/iob/iob_copyout.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
