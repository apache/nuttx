/****************************************************************************
 * mm/iob/iob_pack.c
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

FAR struct iob_s *iob_pack(FAR struct iob_s *iob)
{
  FAR struct iob_s *head;
  FAR struct iob_s *next;
  unsigned int ncopy;
  unsigned int navail;

  /* Handle special cases, preserve at least one iob. */

  while (iob->io_len <= 0 && iob->io_flink != NULL)
    {
      iob = iob_free(iob);
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
              /* Copy the data from the next into
               * the current I/O buffer iob
               */

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

              next          = iob_free(next);
              iob->io_flink = next;
            }
        }

      /* Set up to pack the next entry in the chain */

      iob = next;
    }

  return head;
}
