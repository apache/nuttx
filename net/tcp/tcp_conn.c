/****************************************************************************
 * net/tcp/tcp_conn.c
 *
 *   Copyright (C) 2007-2011, 2013-2015, 2018 Gregory Nutt. All rights
 *     reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <sys/random.h>

#include <netinet/in.h>

#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "inet/inet.h"
#include "tcp/tcp.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF ((FAR struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF ((FAR struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all TCP connections. */

#ifndef CONFIG_NET_ALLOC_CONNS
static struct tcp_conn_s g_tcp_connections[CONFIG_NET_TCP_CONNS];
#endif

/* A list of all free TCP connections */

static dq_queue_t g_free_tcp_connections;

/* A list of all connected TCP connections */

static dq_queue_t g_active_tcp_connections;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_listener
 *
 * Description:
 *   Given a local port number (in network byte order), find the TCP
 *   connection that listens on this port.
 *
 *   Primary uses: (1) to determine if a port number is available, (2) to
 *   To identify the socket that will accept new connections on a local port.
 *
 ****************************************************************************/

static FAR struct tcp_conn_s *
  tcp_listener(uint8_t domain, FAR const union ip_addr_u *ipaddr,
               uint16_t portno)
{
  FAR struct tcp_conn_s *conn = NULL;

  /* Check if this port number is in use by any active UIP TCP connection */

  while ((conn = tcp_nextconn(conn)) != NULL)
    {
      /* Check if this connection is open and the local port assignment
       * matches the requested port number.
       */

      if (conn->tcpstateflags != TCP_CLOSED && conn->lport == portno
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
          && domain == conn->domain
#endif
         )
        {
          /* If there are multiple interface devices, then the local IP
           * address of the connection must also match.  INADDR_ANY is a
           * special case:  There can only be instance of a port number
           * with INADDR_ANY.
           */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
          if (domain == PF_INET)
#endif /* CONFIG_NET_IPv6 */
            {
              if (net_ipv4addr_cmp(conn->u.ipv4.laddr, ipaddr->ipv4) ||
                  net_ipv4addr_cmp(conn->u.ipv4.laddr, INADDR_ANY))
                {
                  /* The port number is in use, return the connection */

                  return conn;
                }
            }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
          else
#endif /* CONFIG_NET_IPv4 */
            {
              if (net_ipv6addr_cmp(conn->u.ipv6.laddr, ipaddr->ipv6) ||
                  net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr))
                {
                  /* The port number is in use, return the connection */

                  return conn;
                }
            }
#endif /* CONFIG_NET_IPv6 */
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: tcp_selectport
 *
 * Description:
 *   If the port number is zero; select an unused port for the connection.
 *   If the port number is non-zero, verify that no other connection has
 *   been created with this port number.
 *
 * Input Parameters:
 *   portno -- the selected port number in network order. Zero means no port
 *     selected.
 *
 * Returned Value:
 *   Selected or verified port number in network order on success, a negated
 *   errno on failure:
 *
 *   EADDRINUSE
 *     The given address is already in use.
 *   EADDRNOTAVAIL
 *     Cannot assign requested address (unlikely)
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int tcp_selectport(uint8_t domain,
                          FAR const union ip_addr_u *ipaddr,
                          uint16_t portno)
{
  static uint16_t g_last_tcp_port;
  ssize_t ret;

  /* Generate port base dynamically */

  if (g_last_tcp_port == 0)
    {
      ret = getrandom(&g_last_tcp_port, sizeof(uint16_t), 0);
      if (ret < 0)
        {
          ret = getrandom(&g_last_tcp_port, sizeof(uint16_t), GRND_RANDOM);
        }

      if (ret != sizeof(uint16_t))
        {
          g_last_tcp_port = clock_systime_ticks() % 32000;
        }
      else
        {
          g_last_tcp_port = g_last_tcp_port % 32000;
        }

      if (g_last_tcp_port < 4096)
        {
          g_last_tcp_port += 4096;
        }
    }

  if (portno == 0)
    {
      /* No local port assigned. Loop until we find a valid listen port
       * number that is not being used by any other connection. NOTE the
       * following loop is assumed to terminate but could not if all
       * 32000-4096+1 ports are in used (unlikely).
       */

      do
        {
          /* Guess that the next available port number will be the one after
           * the last port number assigned. Make sure that the port number
           * is within range.
           */

          if (++g_last_tcp_port >= 32000)
            {
              g_last_tcp_port = 4096;
            }

          portno = HTONS(g_last_tcp_port);
        }
      while (tcp_listener(domain, ipaddr, portno));
    }
  else
    {
      /* A port number has been supplied.  Verify that no other TCP/IP
       * connection is using this local port.
       */

      if (tcp_listener(domain, ipaddr, portno))
        {
          /* It is in use... return EADDRINUSE */

          return -EADDRINUSE;
        }
    }

  /* Return the selected or verified port number (host byte order) */

  return portno;
}

/****************************************************************************
 * Name: tcp_ipv4_active
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided TCP/IP header
 *
 * Assumptions:
 *   This function is called from network logic with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static inline FAR struct tcp_conn_s *
  tcp_ipv4_active(FAR struct net_driver_s *dev, FAR struct tcp_hdr_s *tcp)
{
  FAR struct ipv4_hdr_s *ip = IPv4BUF;
  FAR struct tcp_conn_s *conn;
  in_addr_t srcipaddr;
  in_addr_t destipaddr;

  conn       = (FAR struct tcp_conn_s *)g_active_tcp_connections.head;
  srcipaddr  = net_ip4addr_conv32(ip->srcipaddr);
  destipaddr = net_ip4addr_conv32(ip->destipaddr);

  while (conn)
    {
      /* Find an open connection matching the TCP input. The following
       * checks are performed:
       *
       * - The local port number is checked against the destination port
       *   number in the received packet.
       * - The remote port number is checked if the connection is bound
       *   to a remote port.
       * - Insist that the destination IP matches the bound address. If
       *   a socket is bound to INADDRY_ANY, then it should receive all
       *   packets directed to the port.
       * - Finally, if the connection is bound to a remote IP address,
       *   the source IP address of the packet is checked.
       *
       * If all of the above are true then the newly received TCP packet
       * is destined for this TCP connection.
       */

      if (conn->tcpstateflags != TCP_CLOSED &&
          tcp->destport == conn->lport &&
          tcp->srcport  == conn->rport &&
          (net_ipv4addr_cmp(conn->u.ipv4.laddr, INADDR_ANY) ||
           net_ipv4addr_cmp(destipaddr, conn->u.ipv4.laddr)) &&
          net_ipv4addr_cmp(srcipaddr, conn->u.ipv4.raddr))
        {
          /* Matching connection found.. break out of the loop and return a
           * reference to it.
           */

          break;
        }

      /* Look at the next active connection */

      conn = (FAR struct tcp_conn_s *)conn->sconn.node.flink;
    }

  return conn;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: tcp_ipv6_active
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided TCP/IP header
 *
 * Assumptions:
 *   This function is called from network logic with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static inline FAR struct tcp_conn_s *
  tcp_ipv6_active(FAR struct net_driver_s *dev, FAR struct tcp_hdr_s *tcp)
{
  FAR struct ipv6_hdr_s *ip = IPv6BUF;
  FAR struct tcp_conn_s *conn;
  net_ipv6addr_t *srcipaddr;
  net_ipv6addr_t *destipaddr;

  conn       = (FAR struct tcp_conn_s *)g_active_tcp_connections.head;
  srcipaddr  = (net_ipv6addr_t *)ip->srcipaddr;
  destipaddr = (net_ipv6addr_t *)ip->destipaddr;

  while (conn)
    {
      /* Find an open connection matching the TCP input. The following
       * checks are performed:
       *
       * - The local port number is checked against the destination port
       *   number in the received packet.
       * - The remote port number is checked if the connection is bound
       *   to a remote port.
       * - Insist that the destination IP matches the bound address. If
       *   a socket is bound to the IPv6 unspecified address, then it
       *   should receive all packets directed to the port.
       * - Finally, if the connection is bound to a remote IP address,
       *   the source IP address of the packet is checked.
       *
       * If all of the above are true then the newly received TCP packet
       * is destined for this TCP connection.
       */

      if (conn->tcpstateflags != TCP_CLOSED &&
          tcp->destport == conn->lport &&
          tcp->srcport  == conn->rport &&
          (net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr) ||
           net_ipv6addr_cmp(*destipaddr, conn->u.ipv6.laddr)) &&
          net_ipv6addr_cmp(*srcipaddr, conn->u.ipv6.raddr))
        {
          /* Matching connection found.. break out of the loop and return a
           * reference to it.
           */

          break;
        }

      /* Look at the next active connection */

      conn = (FAR struct tcp_conn_s *)conn->sconn.node.flink;
    }

  return conn;
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: tcp_ipv4_bind
 *
 * Description:
 *   This function implements the lower level parts of the standard TCP
 *   bind() operation.
 *
 * Returned Value:
 *   0 on success or -EADDRINUSE on failure
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static inline int tcp_ipv4_bind(FAR struct tcp_conn_s *conn,
                                FAR const struct sockaddr_in *addr)
{
  int port;
  int ret;

  /* Verify or select a local port and address */

  net_lock();

  /* Verify or select a local port (network byte order) */

  port = tcp_selectport(PF_INET,
                       (FAR const union ip_addr_u *)&addr->sin_addr.s_addr,
                       addr->sin_port);
  if (port < 0)
    {
      nerr("ERROR: tcp_selectport failed: %d\n", port);
      net_unlock();
      return port;
    }

  /* Save the local address in the connection structure (network order). */

  conn->lport = port;
  net_ipv4addr_copy(conn->u.ipv4.laddr, addr->sin_addr.s_addr);

  /* Find the device that can receive packets on the network associated with
   * this local address.
   */

  ret = tcp_local_ipv4_device(conn);
  if (ret < 0)
    {
      /* If no device is found, then the address is not reachable */

      nerr("ERROR: tcp_local_ipv4_device failed: %d\n", ret);

      /* Back out the local address setting */

      conn->lport = 0;
      net_ipv4addr_copy(conn->u.ipv4.laddr, INADDR_ANY);
    }

  net_unlock();
  return ret;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: tcp_ipv6_bind
 *
 * Description:
 *   This function implements the lower level parts of the standard TCP
 *   bind() operation.
 *
 * Returned Value:
 *   0 on success or -EADDRINUSE on failure
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static inline int tcp_ipv6_bind(FAR struct tcp_conn_s *conn,
                                FAR const struct sockaddr_in6 *addr)
{
  int port;
  int ret;

  /* Verify or select a local port and address */

  net_lock();

  /* Verify or select a local port (network byte order) */

  /* The port number must be unique for this address binding */

  port = tcp_selectport(PF_INET6,
                (FAR const union ip_addr_u *)addr->sin6_addr.in6_u.u6_addr16,
                addr->sin6_port);
  if (port < 0)
    {
      nerr("ERROR: tcp_selectport failed: %d\n", port);
      net_unlock();
      return port;
    }

  /* Save the local address in the connection structure (network order). */

  conn->lport = port;
  net_ipv6addr_copy(conn->u.ipv6.laddr, addr->sin6_addr.in6_u.u6_addr16);

  /* Find the device that can receive packets on the network
   * associated with this local address.
   */

  ret = tcp_local_ipv6_device(conn);
  if (ret < 0)
    {
      /* If no device is found, then the address is not reachable */

      nerr("ERROR: tcp_local_ipv6_device failed: %d\n", ret);

      /* Back out the local address setting */

      conn->lport = 0;
      net_ipv6addr_copy(conn->u.ipv6.laddr, g_ipv6_unspecaddr);
    }

  net_unlock();
  return ret;
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: tcp_alloc_conn
 *
 * Description:
 *   Find or allocate a free TCP/IP connection structure for use.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ALLOC_CONNS
FAR struct tcp_conn_s *tcp_alloc_conn(void)
{
  FAR struct tcp_conn_s *conn;
  int i;

  /* Return the entry from the head of the free list */

  if (dq_peek(&g_free_tcp_connections) == NULL)
    {
      conn = kmm_zalloc(sizeof(struct tcp_conn_s) *
                        CONFIG_NET_TCP_CONNS);
      if (conn == NULL)
        {
          return conn;
        }

      /* Now initialize each connection structure */

      for (i = 0; i < CONFIG_NET_TCP_CONNS; i++)
        {
          /* Mark the connection closed and move it to the free list */

          conn[i].tcpstateflags = TCP_CLOSED;
          dq_addlast(&conn[i].sconn.node, &g_free_tcp_connections);
        }
    }

  return (FAR struct tcp_conn_s *)dq_remfirst(&g_free_tcp_connections);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_initialize
 *
 * Description:
 *   Initialize the TCP/IP connection structures.  Called only once and only
 *   from the network layer at start-up.
 *
 ****************************************************************************/

void tcp_initialize(void)
{
#ifndef CONFIG_NET_ALLOC_CONNS
  int i;

  for (i = 0; i < CONFIG_NET_TCP_CONNS; i++)
    {
      /* Mark the connection closed and move it to the free list */

      g_tcp_connections[i].tcpstateflags = TCP_CLOSED;
      dq_addlast(&g_tcp_connections[i].sconn.node, &g_free_tcp_connections);
    }
#endif
}

/****************************************************************************
 * Name: tcp_alloc
 *
 * Description:
 *   Find a free TCP/IP connection structure and allocate it
 *   for use.  This is normally something done by the implementation of the
 *   socket() API but is also called from the event processing logic when a
 *   TCP packet is received while "listening"
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_alloc(uint8_t domain)
{
  FAR struct tcp_conn_s *conn;

  /* Because this routine is called from both event processing (with the
   * network locked) and and from user level.  Make sure that the network
   * locked in any cased while accessing g_free_tcp_connections[];
   */

  net_lock();

  /* Return the entry from the head of the free list */

  conn = (FAR struct tcp_conn_s *)dq_remfirst(&g_free_tcp_connections);

#ifndef CONFIG_NET_SOLINGER
  /* Is the free list empty? */

  if (!conn)
    {
      /* As a fall-back, check for connection structures which can be
       * stalled.
       * Search the active connection list for the oldest connection
       * that is about to be closed anyway.
       */

      FAR struct tcp_conn_s *tmp =
        (FAR struct tcp_conn_s *)g_active_tcp_connections.head;

      while (tmp)
        {
          ninfo("conn: %p state: %02x\n", tmp, tmp->tcpstateflags);

          /* Is this connection in a state we can sacrifice. */

          if ((tmp->crefs == 0) &&
              (tmp->tcpstateflags == TCP_CLOSED    ||
              tmp->tcpstateflags == TCP_CLOSING    ||
              tmp->tcpstateflags == TCP_FIN_WAIT_1 ||
              tmp->tcpstateflags == TCP_FIN_WAIT_2 ||
              tmp->tcpstateflags == TCP_TIME_WAIT  ||
              tmp->tcpstateflags == TCP_LAST_ACK))
            {
              /* Yes.. Is it the oldest one we have seen so far? */

              if (!conn || tmp->timer < conn->timer)
                {
                  /* Yes.. remember it */

                  conn = tmp;
                }
            }

          /* Look at the next active connection */

          tmp = (FAR struct tcp_conn_s *)tmp->sconn.node.flink;
        }

      /* Did we find a connection that we can re-use? */

      if (conn != NULL)
        {
          nwarn("WARNING: Closing unestablished connection: %p\n", conn);

          /* Yes... free it.  This will remove the connection from the list
           * of active connections and release all resources held by the
           * connection.
           *
           * REVISIT:  Could there be any higher level, socket interface
           * that needs to be informed that we did this to them?
           *
           * Actually yes. When CONFIG_NET_SOLINGER is enabled there is a
           * pending callback in netclose_disconnect waiting for getting
           * woken up.  Otherwise there's the callback too, but no one is
           * waiting for it.
           */

          tcp_free(conn);

          /* Now there is guaranteed to be one free connection.  Get it! */

          conn = (FAR struct tcp_conn_s *)
            dq_remfirst(&g_free_tcp_connections);
        }
    }
#endif

  /* Allocate the connect entry from heap */

#ifdef CONFIG_NET_ALLOC_CONNS
  if (conn == NULL)
    {
      conn = tcp_alloc_conn();
    }
#endif

  net_unlock();

  /* Mark the connection allocated */

  if (conn)
    {
      memset(conn, 0, sizeof(struct tcp_conn_s));
      conn->tcpstateflags = TCP_ALLOCATED;
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
      conn->domain        = domain;
#endif
#ifdef CONFIG_NET_TCP_KEEPALIVE
      conn->keepidle      = 2 * DSEC_PER_HOUR;
      conn->keepintvl     = 2 * DSEC_PER_SEC;
      conn->keepcnt       = 3;
#endif
#if CONFIG_NET_RECV_BUFSIZE > 0
      conn->rcv_bufs      = CONFIG_NET_RECV_BUFSIZE;
#endif
#if CONFIG_NET_SEND_BUFSIZE > 0
      conn->snd_bufs      = CONFIG_NET_SEND_BUFSIZE;

      nxsem_init(&conn->snd_sem, 0, 0);
      nxsem_set_protocol(&conn->snd_sem, SEM_PRIO_NONE);
#endif
    }

  return conn;
}

/****************************************************************************
 * Name: tcp_free
 *
 * Description:
 *   Free a connection structure that is no longer in use. This should be
 *   done by the implementation of close()
 *
 ****************************************************************************/

void tcp_free(FAR struct tcp_conn_s *conn)
{
  FAR struct devif_callback_s *cb;
  FAR struct devif_callback_s *next;
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  FAR struct tcp_wrbuffer_s *wrbuffer;
#endif

  /* Because g_free_tcp_connections is accessed from user level and event
   * processing logic, it is necessary to keep the network locked during this
   * operation.
   */

  net_lock();

  DEBUGASSERT(conn->crefs == 0);

  /* Cancel the close work */

  work_cancel(LPWORK, &conn->work);

  tcp_stop_timer(conn);

  /* Free remaining callbacks, actually there should be only the send
   * callback for CONFIG_NET_TCP_WRITE_BUFFERS is left.
   */

  for (cb = conn->sconn.list; cb; cb = next)
    {
      next = cb->nxtconn;
      tcp_callback_free(conn, cb);
    }

  /* TCP_ALLOCATED means that that the connection is not in the active list
   * yet.
   */

  if (conn->tcpstateflags != TCP_ALLOCATED)
    {
      /* Remove the connection from the active list */

      dq_rem(&conn->sconn.node, &g_active_tcp_connections);
    }

  /* Release any read-ahead buffers attached to the connection */

  iob_free_chain(conn->readahead);
  conn->readahead = NULL;

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  /* Release any write buffers attached to the connection */

  while ((wrbuffer = (struct tcp_wrbuffer_s *)
                     sq_remfirst(&conn->write_q)) != NULL)
    {
      tcp_wrbuffer_release(wrbuffer);
    }

  while ((wrbuffer = (struct tcp_wrbuffer_s *)
                     sq_remfirst(&conn->unacked_q)) != NULL)
    {
      tcp_wrbuffer_release(wrbuffer);
    }

#if CONFIG_NET_SEND_BUFSIZE > 0
  /* Notify the send buffer available */

  tcp_sendbuffer_notify(conn);
#endif /* CONFIG_NET_SEND_BUFSIZE */

#endif

#ifdef CONFIG_NET_TCPBACKLOG
  /* Remove any backlog attached to this connection */

  if (conn->backlog)
    {
      tcp_backlogdestroy(conn);
    }

  /* If this connection is, itself, backlogged, then remove it from the
   * parent connection's backlog list.
   */

  if (conn->blparent)
    {
      tcp_backlogdelete(conn->blparent, conn);
    }
#endif

  /* Mark the connection available and put it into the free list */

  conn->tcpstateflags = TCP_CLOSED;
  dq_addlast(&conn->sconn.node, &g_free_tcp_connections);
  net_unlock();
}

/****************************************************************************
 * Name: tcp_active
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided TCP/IP header
 *
 * Assumptions:
 *   This function is called from network logic with the network locked.
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_active(FAR struct net_driver_s *dev,
                                  FAR struct tcp_hdr_s *tcp)
{
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      return tcp_ipv6_active(dev, tcp);
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      return tcp_ipv4_active(dev, tcp);
    }
#endif /* CONFIG_NET_IPv4 */
}

/****************************************************************************
 * Name: tcp_nextconn
 *
 * Description:
 *   Traverse the list of active TCP connections
 *
 * Assumptions:
 *   This function is called from network logic with the network locked.
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_nextconn(FAR struct tcp_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct tcp_conn_s *)g_active_tcp_connections.head;
    }
  else
    {
      return (FAR struct tcp_conn_s *)conn->sconn.node.flink;
    }
}

/****************************************************************************
 * Name: tcp_alloc_accept
 *
 * Description:
 *    Called when driver event processing matches the incoming packet
 *    with a connection in LISTEN. In that case, this function will create
 *    a new connection and initialize it to send a SYNACK in return.
 *
 * Assumptions:
 *   This function is called from network logic with the network locked.
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_alloc_accept(FAR struct net_driver_s *dev,
                                        FAR struct tcp_hdr_s *tcp)
{
  FAR struct tcp_conn_s *conn;
  uint8_t domain;
  int ret;

  /* Get the appropriate IP domain */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  bool ipv6 = IFF_IS_IPv6(dev->d_flags);
  domain = ipv6 ? PF_INET6 : PF_INET;
#elif defined(CONFIG_NET_IPv4)
  domain = PF_INET;
#else /* defined(CONFIG_NET_IPv6) */
  domain = PF_INET6;
#endif

  /* Allocate the connection structure */

  conn = tcp_alloc(domain);
  if (conn)
    {
      /* Set up the local address (laddr) and the remote address (raddr)
       * that describes the TCP connection.
       */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      if (ipv6)
#endif
        {
          FAR struct ipv6_hdr_s *ip = IPv6BUF;

          /* Set the IPv6 specific MSS and the IPv6 locally bound address */

          conn->mss = TCP_IPv6_INITIAL_MSS(dev);
          net_ipv6addr_copy(conn->u.ipv6.raddr, ip->srcipaddr);
          net_ipv6addr_copy(conn->u.ipv6.laddr, ip->destipaddr);

          /* We now have to filter all outgoing transfers so that they use
           * only the MSS of this device.
           */

          DEBUGASSERT(conn->dev == NULL || conn->dev == dev);
          conn->dev = dev;

          /* Find the device that can receive packets on the network
           * associated with this local address.
           */

          ret = tcp_remote_ipv6_device(conn);
        }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      else
#endif
        {
          FAR struct ipv4_hdr_s *ip = IPv4BUF;

          /* Set the IPv6 specific MSS and the IPv4 bound remote address. */

          conn->mss = TCP_IPv4_INITIAL_MSS(dev);
          net_ipv4addr_copy(conn->u.ipv4.raddr,
                            net_ip4addr_conv32(ip->srcipaddr));

          /* Set the local address as well */

          net_ipv4addr_copy(conn->u.ipv4.laddr,
                            net_ip4addr_conv32(ip->destipaddr));

          /* We now have to filter all outgoing transfers so that they use
           * only the MSS of this device.
           */

          DEBUGASSERT(conn->dev == NULL || conn->dev == dev);
          conn->dev = dev;

          /* Find the device that can receive packets on the network
           * associated with this local address.
           */

          ret = tcp_remote_ipv4_device(conn);
        }
#endif /* CONFIG_NET_IPv4 */

      /* Verify that a network device that can provide packets to this
       * local address was found.
       */

      if (ret < 0)
        {
          /* If no device is found, then the address is not reachable.
           * That should be impossible in this context and we should
           * probably really just assert here.
           */

          nerr("ERROR: Failed to find network device: %d\n", ret);
          tcp_free(conn);
          return NULL;
        }

      /* Fill in the necessary fields for the new connection. */

      conn->rto           = TCP_RTO;
      conn->sa            = 0;
      conn->sv            = 4;
      conn->nrtx          = 0;
      conn->lport         = tcp->destport;
      conn->rport         = tcp->srcport;
      conn->tcpstateflags = TCP_SYN_RCVD;

      tcp_initsequence(conn->sndseq);
      conn->tx_unacked    = 1;
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
      conn->expired       = 0;
      conn->isn           = 0;
      conn->sent          = 0;
      conn->sndseq_max    = 0;
#endif

      /* rcvseq should be the seqno from the incoming packet + 1. */

      memcpy(conn->rcvseq, tcp->seqno, 4);
      conn->rcv_adv = tcp_getsequence(conn->rcvseq);

      /* Initialize the list of TCP read-ahead buffers */

      conn->readahead = NULL;

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
      /* Initialize the write buffer lists */

      sq_init(&conn->write_q);
      sq_init(&conn->unacked_q);
#endif

      /* And, finally, put the connection structure into the active list.
       * Interrupts should already be disabled in this context.
       */

      dq_addlast(&conn->sconn.node, &g_active_tcp_connections);
      tcp_update_retrantimer(conn, TCP_RTO);
    }

  return conn;
}

/****************************************************************************
 * Name: tcp_bind
 *
 * Description:
 *   This function implements the lower level parts of the standard TCP
 *   bind() operation.
 *
 * Returned Value:
 *   0 on success or -EADDRINUSE on failure
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

int tcp_bind(FAR struct tcp_conn_s *conn, FAR const struct sockaddr *addr)
{
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
      FAR const struct sockaddr_in *inaddr =
       (FAR const struct sockaddr_in *)addr;

      return tcp_ipv4_bind(conn, inaddr);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      FAR const struct sockaddr_in6 *inaddr =
       (FAR const struct sockaddr_in6 *)addr;

      return tcp_ipv6_bind(conn, inaddr);
    }
#endif /* CONFIG_NET_IPv6 */
}

/****************************************************************************
 * Name: tcp_connect
 *
 * Description:
 *   This function implements the lower level parts of the standard
 *   TCP connect() operation:  It connects to a remote host using TCP.
 *
 *   This function is used to start a new connection to the specified
 *   port on the specified host. It uses the connection structure that was
 *   allocated by a preceding socket() call.  It sets the connection to
 *   the SYN_SENT state and sets the retransmission timer to 0. This will
 *   cause a TCP SYN segment to be sent out the next time this connection
 *   is periodically processed, which usually is done within 0.5 seconds
 *   after the call to tcp_connect().
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

int tcp_connect(FAR struct tcp_conn_s *conn, FAR const struct sockaddr *addr)
{
  int port;
  int ret;

  /* The connection is expected to be in the TCP_ALLOCATED state.. i.e.,
   * allocated via up_tcpalloc(), but not yet put into the active connections
   * list.
   */

  if (!conn || conn->tcpstateflags != TCP_ALLOCATED)
    {
      return -EISCONN;
    }

  /* If the TCP port has not already been bound to a local port, then select
   * one now. We assume that the IP address has been bound to a local device,
   * but the port may still be INPORT_ANY.
   */

  net_lock();

  /* Check if the local port has been bind() */

  port = conn->lport;

  if (port == 0)
    {
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          /* Select a port that is unique for this IPv4 local address
           * (network order).
           */

          port = tcp_selectport(PF_INET,
                                (FAR const union ip_addr_u *)
                                &conn->u.ipv4.laddr, 0);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          /* Select a port that is unique for this IPv6 local address
           * (network order).
           */

          port = tcp_selectport(PF_INET6,
                                (FAR const union ip_addr_u *)
                                conn->u.ipv6.laddr, 0);
        }
#endif /* CONFIG_NET_IPv6 */

      /* Did we have a port assignment? */

      if (port < 0)
        {
          ret = port;
          goto errout_with_lock;
        }
    }

  /* Set up the local address (laddr) and the remote address (raddr) that
   * describes the TCP connection.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
      FAR const struct sockaddr_in *inaddr =
        (FAR const struct sockaddr_in *)addr;

      /* Save MSS and the port from the sockaddr (already in network order) */

      conn->mss    = MIN_IPv4_TCP_INITIAL_MSS;
      conn->rport  = inaddr->sin_port;

      /* The sockaddr address is 32-bits in network order. */

      net_ipv4addr_copy(conn->u.ipv4.raddr, inaddr->sin_addr.s_addr);

      /* Find the device that can receive packets on the network associated
       * with this remote address.
       */

      ret = tcp_remote_ipv4_device(conn);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      FAR const struct sockaddr_in6 *inaddr =
        (FAR const struct sockaddr_in6 *)addr;

      /* Save MSS and the port from the sockaddr (already in network order) */

      conn->mss     = MIN_IPv6_TCP_INITIAL_MSS;
      conn->rport   = inaddr->sin6_port;

      /* The sockaddr address is 128-bits in network order. */

      net_ipv6addr_copy(conn->u.ipv6.raddr, inaddr->sin6_addr.s6_addr16);

      /* Find the device that can receive packets on the network associated
       * with this local address.
       */

      ret = tcp_remote_ipv6_device(conn);
    }
#endif /* CONFIG_NET_IPv6 */

  /* Verify that a network device that can provide packets to this local
   * address was found.
   */

  if (ret < 0)
    {
      /* If no device is found, then the address is not reachable.  That
       * should be impossible in this context and we should probably really
       * just assert here.
       */

      nerr("ERROR: Failed to find network device: %d\n", ret);
      goto errout_with_lock;
    }

#if defined(CONFIG_NET_ARP_SEND) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
#ifdef CONFIG_NET_ARP_SEND
#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  if (conn->domain == PF_INET)
#endif
    {
      /* Make sure that the IP address mapping is in the ARP table */

      ret = arp_send(conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_ARP_SEND */

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
#ifdef CONFIG_NET_ARP_SEND
  else
#endif
    {
      /* Make sure that the IP address mapping is in the Neighbor Table */

      ret = icmpv6_neighbor(conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Did we successfully get the address mapping? */

  if (ret < 0)
    {
      ret = -ENETUNREACH;
      goto errout_with_lock;
    }
#endif /* CONFIG_NET_ARP_SEND || CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Initialize and return the connection structure, bind it to the port
   * number.  At this point, we do not know the size of the initial MSS We
   * know the total size of the packet buffer, but we don't yet know the
   * size of link layer header.
   */

  conn->tcpstateflags = TCP_SYN_SENT;
  tcp_initsequence(conn->sndseq);

  conn->tx_unacked = 1;    /* TCP length of the SYN is one. */
  conn->nrtx       = 0;
  conn->timeout    = true; /* Send the SYN immediately. */
  conn->rto        = TCP_RTO;
  conn->sa         = 0;
  conn->sv         = 16;   /* Initial value of the RTT variance. */
  conn->lport      = (uint16_t)port;
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  conn->expired    = 0;
  conn->isn        = 0;
  conn->sent       = 0;
  conn->sndseq_max = 0;
#endif

  /* Initialize the list of TCP read-ahead buffers */

  conn->readahead = NULL;

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  /* Initialize the TCP write buffer lists */

  sq_init(&conn->write_q);
  sq_init(&conn->unacked_q);
#endif

  /* And, finally, put the connection structure into the active list. */

  dq_addlast(&conn->sconn.node, &g_active_tcp_connections);
  ret = OK;

errout_with_lock:
  net_unlock();
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
