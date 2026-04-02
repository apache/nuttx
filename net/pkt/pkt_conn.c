/****************************************************************************
 * net/pkt/pkt_conn.c
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

#include <string.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <nuttx/debug.h>

#include <arch/irq.h>
#include <netinet/if_ether.h>
#include <netpacket/packet.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/mm/mempool.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "pkt/pkt.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define eth_addr_cmp(addr1, addr2) \
  ((addr1[0] == addr2[0]) && (addr1[1] == addr2[1]) && \
   (addr1[2] == addr2[2]) && (addr1[3] == addr2[3]) && \
   (addr1[4] == addr2[4]) && (addr1[5] == addr2[5]))

#ifndef CONFIG_NET_PKT_MAX_CONNS
#  define CONFIG_NET_PKT_MAX_CONNS 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all packet socket connections */

MEMPOOL_DEFINE(g_pkt_connections, sizeof(struct pkt_conn_s),
               CONFIG_NET_PKT_PREALLOC_CONNS, CONFIG_NET_PKT_MAX_CONNS,
               CONFIG_NET_PKT_ALLOC_CONNS);

/* The list of all allocated packet socket connections is protected by this
 * rmutex.
 */

static rmutex_t g_pkt_connections_lock = NXRMUTEX_INITIALIZER;

/* A list of all allocated packet socket connections */

static dq_queue_t g_active_pkt_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized packet socket connection structure. This
 *   is normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_alloc(void)
{
  FAR struct pkt_conn_s *conn;

  /* The free list is protected by a mutex. */

  pkt_conn_list_lock();

  conn = mempool_allocate(&g_pkt_connections, 0);
  if (conn)
    {
      memset(conn, 0, sizeof(*conn));

      /* Use conn_init to initialize the connection structure */

      conn_init(&conn->sconn);

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->sconn.s_node, &g_active_pkt_connections);
    }

  pkt_conn_list_unlock();
  return conn;
}

/****************************************************************************
 * Name: pkt_free()
 *
 * Description:
 *   Free a packet socket connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void pkt_free(FAR struct pkt_conn_s *conn)
{
  /* The free list is protected by a mutex. */

  DEBUGASSERT(conn->crefs == 0);

  pkt_conn_list_lock();

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.s_node, &g_active_pkt_connections);

  /* Use conn_uninit to release all connection resources */

  conn_uninit(&conn->sconn);

  /* Free the read-ahead data.  pkt_in_() queues frames with the list
   * locked, so no new frame can come in after dq_rem().
   */

  iob_free_queue(&conn->readahead);

  /* Free the read-ahead data.  pkt_in_() queues frames with the list
   * locked, so no new frame can come in after dq_rem().
   */

  iob_free_queue(&conn->readahead);

#ifdef CONFIG_NET_PKT_WRITE_BUFFERS
  /* Free the write queue */

  iob_free_queue(&conn->write_q);
#endif

  /* Free the connection. */

  mempool_release(&g_pkt_connections, conn);

  pkt_conn_list_unlock();
}

/****************************************************************************
 * Name: pkt_nextconn()
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_nextconn(FAR struct pkt_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct pkt_conn_s *)g_active_pkt_connections.head;
    }
  else
    {
      return (FAR struct pkt_conn_s *)conn->sconn.s_node.flink;
    }
}

/****************************************************************************
 * Name: pkt_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided network device
 *
 * Input Parameters:
 *   dev  - The device driver structure containing the received packet
 *   conn - A pointer to the PKT connection structure
 *   loopback - Indicate whether it is a loopback packet
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_active(FAR struct net_driver_s *dev,
                                  FAR struct pkt_conn_s *conn,
                                  bool loopback)
{
  uint16_t ethertype = 0;

  if (dev->d_lltype == NET_LL_ETHERNET || dev->d_lltype == NET_LL_IEEE80211)
    {
      FAR struct eth_hdr_s *ethhdr = NETLLBUF;

      ethertype = ethhdr->type;
    }

#ifdef CONFIG_NET_IPv4
  else if ((IPv4BUF->vhl & IP_VERSION_MASK) == IPv4_VERSION)
    {
      ethertype = HTONS(ETH_P_IP);
    }
#endif
#ifdef CONFIG_NET_IPv6
  else if ((IPv6BUF->vtc & IP_VERSION_MASK) == IPv6_VERSION)
    {
      ethertype = HTONS(ETH_P_IPV6);
    }
#endif

  while ((conn = pkt_nextconn(conn)) != NULL)
    {
      if (dev->d_ifindex != conn->ifindex)
        {
          continue;
        }

      if (conn->type == HTONS(ETH_P_ALL) ||
          (!loopback && conn->type == ethertype))
        {
          /* Matching connection found.. return a reference to it */

          break;
        }
    }

  return conn;
}

/****************************************************************************
 * Name: pkt_sendmsg_is_valid
 *
 * Description:
 *   Validate the sendmsg() parameters for a packet socket.
 *
 * Input Parameters:
 *   psock - The socket structure to validate
 *   msg   - The message header containing the data to be sent
 *   dev   - The network device to be used to send the packet
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pkt_sendmsg_is_valid(FAR struct socket *psock,
                         FAR const struct msghdr *msg,
                         FAR struct net_driver_s **dev)
{
  FAR struct sockaddr_ll *addr = msg->msg_name;

  /* POSIX: fail with EMSGSIZE if msg_iovlen <= 0 or > IOV_MAX */

  if (msg->msg_iovlen == 0 || msg->msg_iovlen > IOV_MAX)
    {
      return -EMSGSIZE;
    }

  /* Only single iov supported */

  if (msg->msg_iovlen != 1)
    {
      return -ENOTSUP;
    }

  /* An iov must be provided */

  if (msg->msg_iov == NULL)
    {
      return -EINVAL;
    }

  /* Elements of the iov must not be NULL if length is non-zero */

  if (msg->msg_iov[0].iov_len > 0 && msg->msg_iov[0].iov_base == NULL)
    {
      return -EINVAL;
    }

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  if (psock->s_type == SOCK_DGRAM)
    {
      if (msg->msg_name == NULL ||
          msg->msg_namelen < sizeof(struct sockaddr_ll) ||
          addr->sll_halen < ETHER_ADDR_LEN)
        {
          return -EINVAL;
        }

      /* Get the device driver that will service this transfer */

      *dev = netdev_findbyindex(addr->sll_ifindex);
    }
  else if (psock->s_type == SOCK_RAW)
    {
      if (msg->msg_name != NULL)
        {
          return -EAFNOSUPPORT;
        }

      /* Get the device driver that will service this transfer */

      *dev = pkt_find_device(psock->s_conn);
    }
  else
    {
      return -ENOTSUP;
    }

  if (*dev == NULL)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: pkt_conn_list_lock()
 *
 * Description:
 *   Lock the packet connection list
 *
 * Assumptions:
 *   This function must be called by driver thread.
 *
 ****************************************************************************/

void pkt_conn_list_lock(void)
{
  nxrmutex_lock(&g_pkt_connections_lock);
}

/****************************************************************************
 * Name: pkt_conn_list_unlock()
 *
 * Description:
 *   Unlock the packet connection list
 *
 * Assumptions:
 *   This function must be called by driver thread.
 *
 ****************************************************************************/

void pkt_conn_list_unlock(void)
{
  nxrmutex_unlock(&g_pkt_connections_lock);
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
