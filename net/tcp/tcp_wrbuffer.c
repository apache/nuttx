/****************************************************************************
 * net/tcp/tcp_wrbuffer.c
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

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_NET_TCP_WRBUFFER_DEBUG)
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
#include "tcp/tcp.h"

#if defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCP_WRITE_BUFFERS)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the state of the global write buffer resource */

NET_BUFPOOL_DECLARE(g_wrbuffer, sizeof(struct tcp_wrbuffer_s),
                    CONFIG_NET_TCP_NWRBCHAINS,
                    CONFIG_NET_TCP_ALLOC_WRBCHAINS, 0);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_wrbuffer_timedalloc
 *
 * Description:
 *   Allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to sent
 *   This function is wrapped version of tcp_wrbuffer_alloc(),
 *   this wait will be terminated when the specified timeout expires.
 *
 * Input Parameters:
 *   timeout   - The relative time to wait until a timeout is declared.
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_timedalloc(unsigned int timeout)
{
  FAR struct tcp_wrbuffer_s *wrb;

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
      tcp_wrbuffer_release(wrb);
      return NULL;
    }

  return wrb;
}

/****************************************************************************
 * Name: tcp_wrbuffer_alloc
 *
 * Description:
 *   Allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to sent
 *
 * Input Parameters:
 *   None
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_alloc(void)
{
  return tcp_wrbuffer_timedalloc(UINT_MAX);
}

/****************************************************************************
 * Name: tcp_wrbuffer_tryalloc
 *
 * Description:
 *   Try to allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to be sent on a non-blocking socket. Returns
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

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_tryalloc(void)
{
  return tcp_wrbuffer_timedalloc(0);
}

/****************************************************************************
 * Name: tcp_wrbuffer_release
 *
 * Description:
 *   Release a TCP write buffer by returning the buffer to the free list.
 *   This function is called from user logic after it is consumed the
 *   buffered data.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void tcp_wrbuffer_release(FAR struct tcp_wrbuffer_s *wrb)
{
  DEBUGASSERT(wrb != NULL);

  /* To avoid deadlocks, we must following this ordering:  Release the I/O
   * buffer chain first, then the write buffer structure.
   */

  if (wrb->wb_iob != NULL)
    {
      iob_free_chain(wrb->wb_iob);
    }

#if defined(CONFIG_NET_TCP_FAST_RETRANSMIT) && !defined(CONFIG_NET_TCP_CC_NEWRENO)
  /* Reset the ack counter */

  TCP_WBNACK(wrb) = 0;
#endif

  /* Then free the write buffer structure */

  NET_BUFPOOL_FREE(g_wrbuffer, wrb);
}

/****************************************************************************
 * Name: tcp_wrbuffer_inqueue_size
 *
 * Description:
 *   Get the in-queued write buffer size from connection
 *
 * Input Parameters:
 *   conn - The TCP connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
uint32_t tcp_wrbuffer_inqueue_size(FAR struct tcp_conn_s *conn)
{
  FAR struct tcp_wrbuffer_s *wrb;
  FAR sq_entry_t *entry;
  uint32_t total = 0;

  if (conn)
    {
      for (entry = sq_peek(&conn->unacked_q); entry; entry = sq_next(entry))
        {
          wrb = (FAR struct tcp_wrbuffer_s *)entry;
          total += TCP_WBPKTLEN(wrb);
        }

      for (entry = sq_peek(&conn->write_q); entry; entry = sq_next(entry))
        {
          wrb = (FAR struct tcp_wrbuffer_s *)entry;
          total += TCP_WBPKTLEN(wrb);
        }
    }

  return total;
}
#endif /* CONFIG_NET_SEND_BUFSIZE */

/****************************************************************************
 * Name: tcp_wrbuffer_test
 *
 * Description:
 *   Check if there is room in the write buffer.  Does not reserve any space.
 *
 * Assumptions:
 *   None.
 *
 ****************************************************************************/

int tcp_wrbuffer_test(void)
{
  return NET_BUFPOOL_TEST(g_wrbuffer);
}

#endif /* CONFIG_NET_TCP && CONFIG_NET_TCP_WRITE_BUFFERS */
