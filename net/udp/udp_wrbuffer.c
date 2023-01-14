/****************************************************************************
 * net/udp/udp_wrbuffer.c
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
 * Private Types
 ****************************************************************************/

/* Package all globals used by this logic into a structure */

struct wrbuffer_s
{
  /* The semaphore to protect the buffers */

  sem_t sem;

  /* This is the list of available write buffers */

  sq_queue_t freebuffers;

  /* These are the pre-allocated write buffers */

  struct udp_wrbuffer_s buffers[CONFIG_NET_UDP_NWRBCHAINS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the state of the global write buffer resource */

static struct wrbuffer_s g_wrbuffer =
{
  SEM_INITIALIZER(CONFIG_NET_UDP_NWRBCHAINS)
};

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
  int i;

  sq_init(&g_wrbuffer.freebuffers);

  for (i = 0; i < CONFIG_NET_UDP_NWRBCHAINS; i++)
    {
      sq_addfirst(&g_wrbuffer.buffers[i].wb_node, &g_wrbuffer.freebuffers);
    }
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

  net_sem_wait_uninterruptible(&g_wrbuffer.sem);

  /* Now, we are guaranteed to have a write buffer structure reserved
   * for us in the free list.
   */

  wrb = (FAR struct udp_wrbuffer_s *)sq_remfirst(&g_wrbuffer.freebuffers);
  DEBUGASSERT(wrb);
  memset(wrb, 0, sizeof(struct udp_wrbuffer_s));

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
  int ret;

  /* We need to allocate two things:  (1) A write buffer structure and (2)
   * at least one I/O buffer to start the chain.
   *
   * Allocate the write buffer structure first then the IOB.  In order to
   * avoid deadlocks, we will need to free the IOB first, then the write
   * buffer
   */

  ret = net_sem_timedwait_uninterruptible(&g_wrbuffer.sem, timeout);
  if (ret != OK)
    {
      return NULL;
    }

  /* Now, we are guaranteed to have a write buffer structure reserved
   * for us in the free list.
   */

  wrb = (FAR struct udp_wrbuffer_s *)sq_remfirst(&g_wrbuffer.freebuffers);
  DEBUGASSERT(wrb);
  memset(wrb, 0, sizeof(struct udp_wrbuffer_s));

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

FAR struct udp_wrbuffer_s *udp_wrbuffer_tryalloc(void)
{
  FAR struct udp_wrbuffer_s *wrb;

  /* We need to allocate two things:  (1) A write buffer structure and (2)
   * at least one I/O buffer to start the chain.
   *
   * Allocate the write buffer structure first then the IOB.  In order to
   * avoid deadlocks, we will need to free the IOB first, then the write
   * buffer
   */

  if (nxsem_trywait(&g_wrbuffer.sem) != OK)
    {
      return NULL;
    }

  /* Now, we are guaranteed to have a write buffer structure reserved
   * for us in the free list.
   */

  wrb = (FAR struct udp_wrbuffer_s *)sq_remfirst(&g_wrbuffer.freebuffers);
  DEBUGASSERT(wrb);
  memset(wrb, 0, sizeof(struct udp_wrbuffer_s));

  /* Now get the first I/O buffer for the write buffer structure */

  wrb->wb_iob = iob_tryalloc(false);
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

  sq_addlast(&wrb->wb_node, &g_wrbuffer.freebuffers);
  nxsem_post(&g_wrbuffer.sem);
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
  int val = 0;
  nxsem_get_value(&g_wrbuffer.sem, &val);
  return val > 0 ? OK : -ENOSPC;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP && CONFIG_NET_UDP_WRITE_BUFFERS */
