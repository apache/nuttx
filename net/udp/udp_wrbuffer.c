/****************************************************************************
 * net/udp/udp_wrbuffer.c
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

#include <nuttx/net/netconfig.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP) && defined(CONFIG_NET_UDP_WRITE_BUFFERS)

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_NET_UDP_WRBUFFER_DEBUG)
/* Force debug output (from this file only) */

#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/mm/iob.h>

#include "utils/utils.h"
#include "udp/udp.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the state of the global write buffer resource */

NET_BUFPOOL_DECLARE(g_wrbuffer, sizeof(struct udp_wrbuffer_s),
                    CONFIG_NET_UDP_NWRBCHAINS,
                    CONFIG_NET_UDP_ALLOC_WRBCHAINS, 0);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_wrbuffer_initialize
 *
 * Description:
 *   Initialize the list of free write buffers
 *
 * Assumptions:
 *   Called once early initialization.
 *
 ****************************************************************************/

void udp_wrbuffer_initialize(void)
{
  NET_BUFPOOL_INIT(g_wrbuffer);
}

/****************************************************************************
 * Name: udp_wrbuffer_alloc
 *
 * Description:
 *   Allocate a UDP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from UDP logic when a buffer
 *   of UDP data is about to sent
 *
 * Input Parameters:
 *   None
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

FAR struct udp_wrbuffer_s *udp_wrbuffer_alloc(void)
{
  FAR struct udp_wrbuffer_s *wrb;

  /* We need to allocate two things:  (1) A write buffer structure and (2)
   * at least one I/O buffer to start the chain.
   *
   * Allocate the write buffer structure first then the IOBG.  In order to
   * avoid deadlocks, we will need to free the IOB first, then the write
   * buffer
   */

  wrb = NET_BUFPOOL_ALLOC(g_wrbuffer);
  DEBUGASSERT(wrb);

  /* Now get the first I/O buffer for the write buffer structure */

  wrb->wb_iob = net_ioballoc(false);
  if (!wrb->wb_iob)
    {
      nerr("ERROR: Failed to allocate I/O buffer\n");
      udp_wrbuffer_release(wrb);
      return NULL;
    }

  return wrb;
}

/****************************************************************************
 * Name: udp_wrbuffer_timedalloc
 *
 * Description:
 *   Allocate a UDP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from udp logic when a buffer
 *   of udp data is about to sent
 *   This function is wrapped version of udp_wrbuffer_alloc(),
 *   this wait will be terminated when the specified timeout expires.
 *
 * Input Parameters:
 *   timeout   - The relative time to wait until a timeout is declared.
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

FAR struct udp_wrbuffer_s *udp_wrbuffer_timedalloc(unsigned int timeout)
{
  FAR struct udp_wrbuffer_s *wrb;

  /* We need to allocate two things:  (1) A write buffer structure and (2)
   * at least one I/O buffer to start the chain.
   *
   * Allocate the write buffer structure first then the IOB.  In order to
   * avoid deadlocks, we will need to free the IOB first, then the write
   * buffer
   */

  wrb = NET_BUFPOOL_TIMEDALLOC(g_wrbuffer, timeout);
  if (wrb == NULL)
    {
      return NULL;
    }

  /* Now get the first I/O buffer for the write buffer structure */

  wrb->wb_iob = net_iobtimedalloc(true, timeout);

  /* Did we get an IOB?  We should always get one except under some really
   * weird error conditions.
   */

  if (wrb->wb_iob == NULL)
    {
      nerr("ERROR: Failed to allocate I/O buffer\n");
      udp_wrbuffer_release(wrb);
      return NULL;
    }

  return wrb;
}

/****************************************************************************
 * Name: udp_wrbuffer_tryalloc
 *
 * Description:
 *   Try to allocate a UDP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from UDP logic when a buffer
 *   of UDP data is about to be sent on a non-blocking socket. Returns
 *   immediately if the allocation failed.
 *
 * Input parameters:
 *   None
 *
 * Assumptions:
 *   Called from user logic with the network locked. Will return if no buffer
 *   is available.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_JUMBO_FRAME
FAR struct udp_wrbuffer_s *udp_wrbuffer_tryalloc(int len)
#else
FAR struct udp_wrbuffer_s *udp_wrbuffer_tryalloc(void)
#endif
{
  FAR struct udp_wrbuffer_s *wrb;

  /* We need to allocate two things:  (1) A write buffer structure and (2)
   * at least one I/O buffer to start the chain.
   *
   * Allocate the write buffer structure first then the IOB.  In order to
   * avoid deadlocks, we will need to free the IOB first, then the write
   * buffer
   */

  wrb = NET_BUFPOOL_TRYALLOC(g_wrbuffer);
  if (wrb == NULL)
    {
      return NULL;
    }

  /* Now get the first I/O buffer for the write buffer structure */

  wrb->wb_iob =
#ifdef CONFIG_NET_JUMBO_FRAME
    iob_alloc_dynamic(len);
#else
    iob_tryalloc(false);
#endif
  if (!wrb->wb_iob)
    {
      nerr("ERROR: Failed to allocate I/O buffer\n");
      udp_wrbuffer_release(wrb);
      return NULL;
    }

  return wrb;
}

/****************************************************************************
 * Name: udp_wrbuffer_release
 *
 * Description:
 *   Release a UDP write buffer by returning the buffer to the free list.
 *   This function is called from user logic after it is consumed the
 *   buffered data.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void udp_wrbuffer_release(FAR struct udp_wrbuffer_s *wrb)
{
  DEBUGASSERT(wrb);

  /* To avoid deadlocks, we must following this ordering:  Release the I/O
   * buffer chain first, then the write buffer structure.
   */

  if (wrb->wb_iob)
    {
      iob_free_chain(wrb->wb_iob);
    }

  /* Then free the write buffer structure */

  NET_BUFPOOL_FREE(g_wrbuffer, wrb);
}

/****************************************************************************
 * Name: udp_wrbuffer_inqueue_size
 *
 * Description:
 *   Get the in-queued write buffer size from connection
 *
 * Input Parameters:
 *   conn - The UDP connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
uint32_t udp_wrbuffer_inqueue_size(FAR struct udp_conn_s *conn)
{
  FAR struct udp_wrbuffer_s *wrb;
  FAR sq_entry_t *entry;
  uint32_t total = 0;

  if (conn)
    {
      for (entry = sq_peek(&conn->write_q); entry; entry = sq_next(entry))
        {
          wrb = (FAR struct udp_wrbuffer_s *)entry;
          total += wrb->wb_iob->io_pktlen;
        }
    }

  return total;
}
#endif /* CONFIG_NET_SEND_BUFSIZE */

/****************************************************************************
 * Name: udp_wrbuffer_test
 *
 * Description:
 *   Check if there is room in the write buffer.  Does not reserve any space.
 *
 * Assumptions:
 *   None.
 *
 ****************************************************************************/

int udp_wrbuffer_test(void)
{
  return NET_BUFPOOL_TEST(g_wrbuffer);
}

#endif /* CONFIG_NET && CONFIG_NET_UDP && CONFIG_NET_UDP_WRITE_BUFFERS */
