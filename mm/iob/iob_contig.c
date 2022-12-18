/****************************************************************************
 * mm/iob/iob_contig.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_contig
 *
 * Description:
 *   Merge an iob chain into a continuous space, thereby reducing iob
 *   consumption
 *
 ****************************************************************************/

FAR struct iob_s *iob_contig(FAR struct iob_s *iob)
{
  FAR struct iob_s *head;
  FAR struct iob_s *next;
  unsigned int ncopy;

  /* Trim empty iob header */

  while (iob != NULL && iob->io_len == 0)
    {
      iob = iob_free(iob);
    }

  if (iob == NULL)
    {
      return NULL;
    }

  /* Remember the chain header */

  head = iob;

  while (iob != NULL && iob->io_flink != NULL)
    {
      next = iob->io_flink;

      /* Skip and free the empty next node */

      if (next->io_len == 0)
        {
          next = iob_free(next);
          iob->io_flink = next;
          continue;
        }

      /* Skip filled node */

      if (iob->io_len == CONFIG_IOB_BUFSIZE)
        {
          iob = iob->io_flink;
          continue;
        }

      /* Eliminate any leading offset */

      if (iob->io_offset > 0)
        {
          memcpy(iob->io_data, &iob->io_data[iob->io_offset], iob->io_len);
          iob->io_offset = 0;
        }

      /* Copy what we need or what we can from the next buffer */

      ncopy = CONFIG_IOB_BUFSIZE - iob->io_len;
      ncopy = MIN(ncopy, next->io_len);
      memcpy(&iob->io_data[iob->io_len],
             &next->io_data[next->io_offset], ncopy);

      /* Adjust counts and offsets */

      iob->io_len     += ncopy;
      next->io_offset += ncopy;
      next->io_len    -= ncopy;
    }

  return head;
}
