/****************************************************************************
 * mm/iob/iob_share.c
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

#include <nuttx/irq.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Static Functions
 ****************************************************************************/

/****************************************************************************
 * Name: alloc_share
 *
 * Description:
 *   Allocate shared iob node
 *
 ****************************************************************************/

static FAR struct iob_s *alloc_share(void)
{
  FAR struct iob_s *iob;

  iob = g_iob_sharelist;
  if (iob != NULL)
    {
      g_iob_sharelist = iob->io_flink;

      memset(iob, 0, sizeof(*iob));
    }

  return iob;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_share_partial
 *
 * Description:
 *   Create shared chain to share the partial data from input iob.
 *
 * Input Parameters:
 *   iob      - Pointer to source iob_s
 *   len      - Number of bytes to share
 *   offset   - Offset of source iobs_s
 *
 * Returned Value:
 *   Return iob share chain if success.
 *   Return NULL if the shared pool cannot be allocated
 *
 ****************************************************************************/

FAR struct iob_s *iob_share_partial(FAR struct iob_s *iob, unsigned int len,
                                    unsigned int offset)
{
  FAR struct iob_s *share = NULL;
  FAR struct iob_s *head = NULL;
  unsigned int total = len;
  FAR struct iob_s *next;
  irqstate_t flags;

  /* Handle special case where there are empty buffers at the head
   * the list, Skip I/O buffer containing the data offset.
   */

  while (iob != NULL && offset >= iob->io_len)
    {
      offset -= iob->io_len;
      iob     = iob->io_flink;
    }

  if (iob == NULL)
    {
      return NULL;
    }

  flags = enter_critical_section();

  while (len > 0)
    {
      next = alloc_share();
      if (next == NULL)
        {
          if (head != NULL)
            {
              iob_free(head);
            }

          leave_critical_section(flags);
          return NULL;
        }

      next->io_parent = iob;
      next->io_data   = iob->io_data;
      next->io_offset = iob->io_offset + offset;
      next->io_len    = iob->io_len - offset >= len ?
                        len : iob->io_len - offset;
      len            -= next->io_len;
      offset          = 0;
      iob->io_refs++;
      iob             = iob->io_flink;

      if (head == NULL)
        {
          head = next;
        }

      if (share != NULL)
        {
          share->io_flink = next;
        }

      share = next;
    }

  leave_critical_section(flags);

  head->io_pktlen = total;

  return head;
}

/****************************************************************************
 * Name: iob_share
 *
 * Description:
 *   Create share chain to share the data from input iob.
 *
 * Input Parameters:
 *   iob      - Pointer to source iob_s
 *
 * Returned Value:
 *   Return iob share chain if success.
 *   Return NULL if the shared pool cannot be allocated
 *
 ****************************************************************************/

FAR struct iob_s *iob_share(FAR struct iob_s *iob)
{
  return iob_share_partial(iob, iob->io_pktlen, 0);
}
