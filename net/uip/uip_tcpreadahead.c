/****************************************************************************
 * net/uip/uip_tcpreadahead.c
 *
 *   Copyright (C) 2007-2009, 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/net/uip/uipopt.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCP_READAHEAD)

#include <queue.h>
#include <debug.h>

#include <nuttx/net/uip/uip.h>

#include "uip_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Package all globals used by this logic into a structure */

struct readahead_s
{
  /* This is the list of available write buffers */

  sq_queue_t freebuffers;

  /* These are the pre-allocated write buffers */

  struct uip_readahead_s buffers[CONFIG_NET_NTCP_READAHEAD_BUFFERS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the state of the global read-ahead resource */

static struct readahead_s g_readahead;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: uip_tcpreadahead_init
 *
 * Description:
 *   Initialize the list of free read-ahead buffers
 *
 * Assumptions:
 *   Called once early initialization.
 *
 ****************************************************************************/

void uip_tcpreadahead_init(void)
{
  int i;

  sq_init(&g_readahead.freebuffers);
  for (i = 0; i < CONFIG_NET_NTCP_READAHEAD_BUFFERS; i++)
    {
      sq_addfirst(&g_readahead.buffers[i].rh_node, &g_readahead.freebuffers);
    }
}

/****************************************************************************
 * Function: uip_tcpreadahead_alloc
 *
 * Description:
 *   Allocate a TCP read-ahead buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when new,
 *   incoming TCP data is received but there is no user logic receiving the
 *   the data.  Note: kmalloc() cannot be used because this function is
 *   called from interrupt level.
 *
 * Assumptions:
 *   Called from interrupt level with interrupts disabled.
 *
 ****************************************************************************/

FAR struct uip_readahead_s *uip_tcpreadahead_alloc(void)
{
  return (FAR struct uip_readahead_s*)sq_remfirst(&g_readahead.freebuffers);
}

/****************************************************************************
 * Function: uip_tcpreadahead_release
 *
 * Description:
 *   Release a TCP read-ahead buffer by returning the buffer to the free list.
 *   This function is called from user logic after it is consumed the buffered
 *   data.
 *
 * Assumptions:
 *   Called from user logic BUT with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcpreadahead_release(FAR struct uip_readahead_s *readahead)
{
  sq_addfirst(&readahead->rh_node, &g_readahead.freebuffers);
}

#endif /* CONFIG_NET && CONFIG_NET_TCP && CONFIG_NET_TCP_READAHEAD */
