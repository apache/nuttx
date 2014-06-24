/****************************************************************************
 * net/pkt/pkt_conn.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Large parts of this file were leveraged from uIP logic:
 *
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/uip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>

#include "uip/uip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all packet socket connections */

static struct uip_pkt_conn g_pkt_connections[CONFIG_NET_PKT_CONNS];

/* A list of all free packet socket connections */

static dq_queue_t g_free_pkt_connections;
static sem_t g_free_sem;

/* A list of all allocated packet scoket connections */

static dq_queue_t g_active_pkt_connections;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _uip_semtake() and _uip_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static inline void _uip_semtake(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (uip_lockedwait(sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

#define _uip_semgive(sem) sem_post(sem)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_pktinit()
 *
 * Description:
 *   Initialize the packet socket connection structures.  Called once and
 *   only from the UIP layer.
 *
 ****************************************************************************/

void uip_pktinit(void)
{
  int i;

  /* Initialize the queues */

  dq_init(&g_free_pkt_connections);
  dq_init(&g_active_pkt_connections);
  sem_init(&g_free_sem, 0, 1);

  for (i = 0; i < CONFIG_NET_PKT_CONNS; i++)
    {
      /* Mark the connection closed and move it to the free list */
      g_pkt_connections[i].ifindex = 0;
      dq_addlast(&g_pkt_connections[i].node, &g_free_pkt_connections);
    }
}

/****************************************************************************
 * Name: uip_pktpalloc()
 *
 * Description:
 *   Alloc a new, uninitialized packet socket connection structure.
 *
 ****************************************************************************/

struct uip_pkt_conn *uip_pktalloc(void)
{
  struct uip_pkt_conn *conn;

  /* The free list is only accessed from user, non-interrupt level and
   * is protected by a semaphore (that behaves like a mutex).
   */

  _uip_semtake(&g_free_sem);
  conn = (struct uip_pkt_conn *)dq_remfirst(&g_free_pkt_connections);
  if (conn)
    {
      /* Make sure that the connection is marked as uninitialized */

      conn->ifindex = 0;

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->node, &g_active_pkt_connections);
    }

  _uip_semgive(&g_free_sem);
  return conn;
}

/****************************************************************************
 * Name: uip_pktfree()
 *
 * Description:
 *   Free a packet socket connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void uip_pktfree(struct uip_pkt_conn *conn)
{
  /* The free list is only accessed from user, non-interrupt level and
   * is protected by a semaphore (that behaves like a mutex).
   */

  DEBUGASSERT(conn->crefs == 0);

  _uip_semtake(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->node, &g_active_pkt_connections);

  /* Free the connection */

  dq_addlast(&conn->node, &g_free_pkt_connections);
  _uip_semgive(&g_free_sem);
}

/****************************************************************************
 * Name: uip_pktactive()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided Ethernet header
 *
 * Assumptions:
 *   This function is called from UIP logic at interrupt level
 *
 ****************************************************************************/

struct uip_pkt_conn *uip_pktactive(struct uip_eth_hdr *buf)
{
  #define uip_ethaddr_cmp(addr1, addr2) \
  ((addr1[0] == addr2[0]) && (addr1[1] == addr2[1]) && \
   (addr1[2] == addr2[2]) && (addr1[3] == addr2[3]) && \
   (addr1[4] == addr2[4]) && (addr1[5] == addr2[5]))

  FAR struct uip_pkt_conn *conn =
    (struct uip_pkt_conn *)g_active_pkt_connections.head;

  while (conn)
    {
      /* FIXME lmac in conn should have been set by pkt_rawbind() */

      if (uip_ethaddr_cmp(buf->dest, conn->lmac))
        {
          /* Matching connection found.. return a reference to it */

          break;
        }

      /* Look at the next active connection */

      conn = (struct uip_pkt_conn *)conn->node.flink;
    }

  return conn;
}

/****************************************************************************
 * Name: uip_nextpktconn()
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from UIP logic at interrupt level (or with
 *   interrupts disabled).
 *
 ****************************************************************************/

struct uip_pkt_conn *uip_nextpktconn(struct uip_pkt_conn *conn)
{
  if (!conn)
    {
      return (struct uip_pkt_conn *)g_active_pkt_connections.head;
    }
  else
    {
      return (struct uip_pkt_conn *)conn->node.flink;
    }
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
