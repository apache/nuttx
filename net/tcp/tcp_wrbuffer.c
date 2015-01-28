/****************************************************************************
 * net/tcp/tcp_wrbuffer.c
 *
 *   Copyright (C) 2007-2009, 2013-2014 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCP_WRITE_BUFFERS)

#if defined(CONFIG_DEBUG) && defined(CONFIG_NET_TCP_WRBUFFER_DEBUG)
/* Force debug output (from this file only) */

#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <queue.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/iob.h>

#include "tcp/tcp.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tcp_wrbuffer_initialize
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

  sem_init(&g_wrbuffer.sem, 0, CONFIG_NET_TCP_NWRBCHAINS);
}

/****************************************************************************
 * Function: tcp_wrbuffer_alloc
 *
 * Description:
 *   Allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to sent
 *
 * Input parameters:
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
   * Allocate the write buffer structure first then the IOBG.  In order to
   * avoid deadlocks, we will need to free the IOB first, then the write
   * buffer
   */

  DEBUGVERIFY(net_lockedwait(&g_wrbuffer.sem));

  /* Now, we are guaranteed to have a write buffer structure reserved
   * for us in the free list.
   */

  wrb = (FAR struct tcp_wrbuffer_s *)sq_remfirst(&g_wrbuffer.freebuffers);
  DEBUGASSERT(wrb);
  memset(wrb, 0, sizeof(struct tcp_wrbuffer_s));

  /* Now get the first I/O buffer for the write buffer structure */

  wrb->wb_iob = iob_alloc(false);
  if (!wrb->wb_iob)
    {
      ndbg("ERROR: Failed to allocate I/O buffer\n");
      tcp_wrbuffer_release(wrb);
      return NULL;
    }

  return wrb;
}

/****************************************************************************
 * Function: tcp_wrbuffer_release
 *
 * Description:
 *   Release a TCP write buffer by returning the buffer to the free list.
 *   This function is called from user logic after it is consumed the
 *   buffered data.
 *
 * Assumptions:
 *   Called from interrupt level with interrupts disabled.
 *
 ****************************************************************************/

void tcp_wrbuffer_release(FAR struct tcp_wrbuffer_s *wrb)
{
  DEBUGASSERT(wrb && wrb->wb_iob);

  /* To avoid deadlocks, we must following this ordering:  Release the I/O
   * buffer chain first, then the write buffer structure.
   */

  iob_free_chain(wrb->wb_iob);

  /* Then free the write buffer structure */

  sq_addlast(&wrb->wb_node, &g_wrbuffer.freebuffers);
  sem_post(&g_wrbuffer.sem);
}

#endif /* CONFIG_NET && CONFIG_NET_TCP && CONFIG_NET_TCP_WRITE_BUFFERS */
