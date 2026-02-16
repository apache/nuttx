/****************************************************************************
 * net/can/can_bufpool.c
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

#include <debug.h>

#include <nuttx/net/can.h>

#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_TIMESTAMP
#  define CAN_BUFFER_SIZE ALIGN_UP(sizeof(struct iob_s) + NET_CAN_PKTSIZE + \
                                   CONFIG_NET_LL_GUARDSIZE + IOB_ALIGNMENT - \
                                   1, IOB_ALIGNMENT)
#else
#  define CAN_BUFFER_SIZE ALIGN_UP(sizeof(struct iob_s) + NET_CAN_PKTSIZE + \
                                   IOB_ALIGNMENT - 1, IOB_ALIGNMENT)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all CAN buffers */

NET_BUFPOOL_DECLARE(g_can_buffer, CAN_BUFFER_SIZE,
                    CONFIG_NET_CAN_NBUFFERS, 0, 0);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_buf_free
 *
 * Description:
 *   Free the CAN buffer to the buffer pool
 *
 * Input Parameters:
 *   data - The CAN buffer to be freed
 *
 ****************************************************************************/

static void can_buf_free(FAR void *buf)
{
  NET_BUFPOOL_FREE(g_can_buffer, buf);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_iob_timedalloc
 *
 * Description:
 *  Allocate an CAN I/O buffer from the CAN buffer pool.
 *
 ****************************************************************************/

FAR struct iob_s *can_iob_timedalloc(unsigned int timeout)
{
  FAR void *buf = NET_BUFPOOL_TIMEDALLOC(g_can_buffer, timeout);

  if (buf == NULL)
    {
      nwarn("Failed to allocate CAN buffer\n");
      return NULL;
    }

  return iob_init_with_data(buf, CAN_BUFFER_SIZE, can_buf_free);
}

/****************************************************************************
 * Name: can_iob_clone
 *
 * Description:
 *  Clone an I/O buffer from the CAN buffer pool.
 *
 ****************************************************************************/

FAR struct iob_s *can_iob_clone(FAR struct net_driver_s *dev)
{
  FAR struct iob_s *iob;

  iob = can_iob_timedalloc(0);
  if (iob == NULL)
    {
      return NULL;
    }

#ifdef CONFIG_NET_TIMESTAMP
  iob_reserve(iob, CONFIG_NET_LL_GUARDSIZE);
#endif

  /* CAN data length is fixed, So when we use iob_clone_partial to copy
   * data, we don't have to worry about distributing other iob.
   */

  iob_clone_partial(dev->d_iob, dev->d_iob->io_pktlen, 0, iob, 0,
                    false, false);

  return iob;
}

/****************************************************************************
 * Name: can_iob_navail
 *
 * Description:
 *   Return the number of available CAN I/O buffers.
 *
 ****************************************************************************/

int can_iob_navail(void)
{
  return NET_BUFPOOL_NAVAIL(g_can_buffer);
}
