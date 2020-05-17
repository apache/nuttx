/****************************************************************************
 * net/tcp/tcp_wrbuffer.c
 *
 *   Copyright (C) 2007-2009, 2013-2014, 2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Jason Jiang  <jasonj@live.cn>
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

#include <nuttx/net/netconfig.h>

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_NET_TCP_WRBUFFER_DEBUG)
/* Force debug output (from this file only) */

#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <queue.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/mm/iob.h>

#include "utils/utils.h"
#include "tcp/tcp.h"

#if defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCP_WRITE_BUFFERS)

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

  struct tcp_wrbuffer_s buffers[CONFIG_NET_TCP_NWRBCHAINS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the state of the global write buffer resource */

static struct wrbuffer_s g_wrbuffer;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_wrbuffer_initialize
 *
 * Description:
 *   Initialize the list of free write buffers
 *
 * Assumptions:
 *   Called once early initialization.
 *
 ****************************************************************************/

void tcp_wrbuffer_initialize(void)
{
  int i;

  sq_init(&g_wrbuffer.freebuffers);

  for (i = 0; i < CONFIG_NET_TCP_NWRBCHAINS; i++)
    {
      sq_addfirst(&g_wrbuffer.buffers[i].wb_node, &g_wrbuffer.freebuffers);
    }

  nxsem_init(&g_wrbuffer.sem, 0, CONFIG_NET_TCP_NWRBCHAINS);
  nxsem_set_protocol(&g_wrbuffer.sem, SEM_PRIO_NONE);
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
  FAR struct tcp_wrbuffer_s *wrb;

  /* We need to allocate two things:  (1) A write buffer structure and (2)
   * at least one I/O buffer to start the chain.
   *
   * Allocate the write buffer structure first then the IOB.  In order to
   * avoid deadlocks, we will need to free the IOB first, then the write
   * buffer
   */

  net_lockedwait_uninterruptible(&g_wrbuffer.sem);

  /* Now, we are guaranteed to have a write buffer structure reserved
   * for us in the free list.
   */

  wrb = (FAR struct tcp_wrbuffer_s *)sq_remfirst(&g_wrbuffer.freebuffers);
  DEBUGASSERT(wrb);
  memset(wrb, 0, sizeof(struct tcp_wrbuffer_s));

  /* Now get the first I/O buffer for the write buffer structure */

  wrb->wb_iob = net_ioballoc(false, IOBUSER_NET_TCP_WRITEBUFFER);

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
  FAR struct tcp_wrbuffer_s *wrb;

  /* We need to allocate two things:  (1) A write buffer structure and (2)
   * at least one I/O buffer to start the chain.
   *
   * Allocate the write buffer structure first then the IOBG.  In order to
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

  wrb = (FAR struct tcp_wrbuffer_s *)sq_remfirst(&g_wrbuffer.freebuffers);
  DEBUGASSERT(wrb);
  memset(wrb, 0, sizeof(struct tcp_wrbuffer_s));

  /* Now get the first I/O buffer for the write buffer structure */

  wrb->wb_iob = iob_tryalloc(false, IOBUSER_NET_TCP_WRITEBUFFER);
  if (!wrb->wb_iob)
    {
      nerr("ERROR: Failed to allocate I/O buffer\n");
      tcp_wrbuffer_release(wrb);
      return NULL;
    }

  return wrb;
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
      iob_free_chain(wrb->wb_iob, IOBUSER_NET_TCP_WRITEBUFFER);
    }

  /* Then free the write buffer structure */

  sq_addlast(&wrb->wb_node, &g_wrbuffer.freebuffers);
  nxsem_post(&g_wrbuffer.sem);
}

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
  int val = 0;
  int ret;

  ret = nxsem_get_value(&g_wrbuffer.sem, &val);
  if (ret >= 0)
    {
      ret = val > 0 ? OK : -ENOSPC;
    }

  return ret;
}

#endif /* CONFIG_NET_TCP && CONFIG_NET_TCP_WRITE_BUFFERS */
