/****************************************************************************
 * net/utils/net_iob_concat.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/types.h>

#include <nuttx/mm/iob.h>

#ifdef CONFIG_MM_IOB

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_iob_concat
 *
 * Description:
 *   Concatenate iob_s chain iob2 to iob1, if CONFIG_NET_RECV_PACK is
 *   endabled, pack all data in the I/O buffer chain.
 *
 * Returned Value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to iob->io_pktlen.
 *
 ****************************************************************************/

uint16_t net_iob_concat(FAR struct iob_s **iob1, FAR struct iob_s **iob2)
{
  if (*iob1 == NULL)
    {
      *iob1 = *iob2;
    }
  else
    {
      iob_concat(*iob1, *iob2);
    }

  *iob2 = NULL;

#ifdef CONFIG_NET_RECV_PACK
  /* Merge an iob chain into a continuous space, thereby reducing iob
   * consumption.
   */

  *iob1 = iob_pack(*iob1);
#endif

  return (*iob1)->io_pktlen;
}

#endif /* CONFIG_MM_IOB */
