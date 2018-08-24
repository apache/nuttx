/****************************************************************************
 * net/udp/udp_conn.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2016, 2018 Gregory Nutt. All rights
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "inet/inet.h"
#include "udp/udp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all UDP connections. */

struct udp_conn_s g_udp_connections[CONFIG_NET_UDP_CONNS];

/* A list of all free UDP connections */

static dq_queue_t g_free_udp_connections;
static sem_t g_free_sem;

/* A list of all allocated UDP connections */

static dq_queue_t g_active_udp_connections;

/* Last port used by a UDP connection connection. */

static uint16_t g_last_udp_port;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _udp_semtake() and _udp_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static inline void _udp_semtake(FAR sem_t *sem)
{
  int ret;

  /* Take the semaphore (perhaps waiting) */

  while ((ret = net_lockedwait(sem)) < 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
    }

  UNUSED(ret);
}

#define _udp_semgive(sem) nxsem_post(sem)

/****************************************************************************
 * Name: udp_find_conn()
 *
 * Description:
 *   Find the UDP connection that uses this local port number.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

static FAR struct udp_conn_s *udp_find_conn(uint8_t domain,
                                            FAR union ip_binding_u *ipaddr,
                                            uint16_t portno)
{
  FAR struct udp_conn_s *conn;
  int i;

  /* Now search each connection structure. */

  for (i = 0; i < CONFIG_NET_UDP_CONNS; i++)
    {
      conn = &g_udp_connections[i];

      /* If the port local port number assigned to the connections matches
       * AND the IP address of the connection matches, then return a
       * reference to the connection structure.  INADDR_ANY is a special
       * case:  There can only be instance of a port number with INADDR_ANY.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (domain == PF_INET)
#endif
        {
          if (conn->lport == portno &&
              (net_ipv4addr_cmp(conn->u.ipv4.laddr, ipaddr->ipv4.laddr) ||
               net_ipv4addr_cmp(conn->u.ipv4.laddr, INADDR_ANY)))
            {
              return conn;
            }
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          if (conn->lport == portno &&
              (net_ipv6addr_cmp(conn->u.ipv6.laddr, ipaddr->ipv6.laddr) ||
               net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr)))
            {
              return conn;
            }
        }
#endif /* CONFIG_NET_IPv6 */
    }

  return NULL;
}

/****************************************************************************
 * Name: udp_select_port
 *
 * Description:
 *   Select an unused port number.
 *
 *   NOTE that in principle this function could fail if there is no available
 *   port number.  There is no check for that case and it would actually
 *   in an infinite loop if that were the case.  In this simple, small UDP
 *   implementation, it is reasonable to assume that that error cannot happen
 *   and that a port number will always be available.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Next available port number
 *
 ****************************************************************************/

static uint16_t udp_select_port(uint8_t domain, FAR union ip_binding_u *u)
{
  uint16_t portno;

  /* Find an unused local port number.  Loop until we find a valid
   * listen port number that is not being used by any other connection.
   */

  net_lock();
  do
    {
      /* Guess that the next available port number will be the one after
       * the last port number assigned.
       */

      ++g_last_udp_port;

      /* Make sure that the port number is within range */

      if (g_last_udp_port >= 32000)
        {
          g_last_udp_port = 4096;
        }
    }
  while (udp_find_conn(domain, u, htons(g_last_udp_port)) != NULL);

  /* Initialize and return the connection structure, bind it to the
   * port number
   */

  portno = g_last_udp_port;
  net_unlock();

  return portno;
}

/****************************************************************************
 * Name: udp_ipv4_active
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used within the provided UDP header
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static inline FAR struct udp_conn_s *
  udp_ipv4_active(FAR struct net_driver_s *dev, FAR struct udp_hdr_s *udp)
{
#ifdef CONFIG_NET_BROADCAST
  static const in_addr_t bcast = INADDR_BROADCAST;
#endif
  FAR struct ipv4_hdr_s *ip = IPv4BUF;
  FAR struct udp_conn_s *conn;

  conn = (FAR struct udp_conn_s *)g_active_udp_connections.head;
  while (conn)
    {
      /* If the local UDP port is non-zero, the connection is considered
       * to be used. If so, then the following checks are performed:
       *
       * 1. The destination address is verified against the bound address
       *    of the connection.
       *
       *   - The local port number is checked against the destination port
       *     number in the received packet.
       *   - If multiple network interfaces are supported, then the local
       *     IP address is available and we will insist that the
       *     destination IP matches the bound address (or the destination
       *     IP address is a broadcast address). If a socket is bound to
       *     INADDRY_ANY (laddr), then it should receive all packets
       *     directed to the port.
       *
       * 2. If this is a connection mode UDP socket, then the source address
       *    is verified against the connected remote address.
       *
       *   - The remote port number is checked if the connection is bound
       *     to a remote port.
       *   - Finally, if the connection is bound to a remote IP address,
       *     the source IP address of the packet is checked. Broadcast
       *     addresses are also accepted.
       *
       * If all of the above are true then the newly received UDP packet
       * is destined for this UDP connection.
       *
       * To send and receive multicast packets, the application should:
       *
       *   - Bind socket to INADDR6_ANY (for the all-nodes multicast address)
       *     or to a specific <multicast-address>
       *   - setsockopt to SO_BROADCAST (for all-nodes address)
       *
       * For connection-less UDP sockets:
       *
       *   - call sendto with sendaddr.sin_addr.s_addr = <multicast-address>
       *   - call recvfrom.
       *
       * For connection-mode UDP sockets:
       *
       *   - call connect() to connect the UDP socket to a specific remote
       *     address, then
       *   - Call send() with no address address information
       *   - call recv() (from address information should not be needed)
       *
       * REVIST: SO_BROADCAST flag is currently ignored.
       */

      /* Check that there is a local port number and this is matches
       * the port number in the destination address.
       */

      if (conn->lport != 0 && udp->destport == conn->lport &&

          /* Local port accepts any address on this port or there
           * is an exact match in destipaddr and the bound local
           * address.  This catches the receipt of a broadcast when
           * the socket is bound to INADDR_ANY.
           */

          (net_ipv4addr_cmp(conn->u.ipv4.laddr, INADDR_ANY) ||
           net_ipv4addr_hdrcmp(ip->destipaddr, &conn->u.ipv4.laddr)))
        {
          /* Check if the socket is connection mode.  In this case, only
           * packets with source addresses from the connected remote peer
           * will be accepted.
           */

          if (_UDP_ISCONNECTMODE(conn->flags))
            {
              /* Check if the UDP connection is either (1) accepting packets
               * from any port or (2) the packet srcport matches the local
               * bound port number.
               */

              if ((conn->rport == 0 || udp->srcport == conn->rport) &&

              /* If (1) not connected to a remote address, or (2) a
               * broadcast destipaddr was received, or (3) there is an
               * exact match between the srcipaddr and the bound remote IP
               * address, then accept the packet.
               */

                  (net_ipv4addr_cmp(conn->u.ipv4.raddr, INADDR_ANY) ||
#ifdef CONFIG_NET_BROADCAST
                   net_ipv4addr_hdrcmp(ip->destipaddr, &bcast) ||
#endif
                   net_ipv4addr_hdrcmp(ip->srcipaddr, &conn->u.ipv4.raddr)))
                {
                  /* Matching connection found.. Break out of the loop and
                   * return this reference to it.
                   */

                  break;
                }
            }
          else
            {
              /* This UDP socket is not connected.  We need to match only
               * the destination address with the bound socket address.
               * Break out out of the loop and return this reference to
               * the matching connection structure.
               */

              break;
            }
        }

      /* Look at the next active connection */

      conn = (FAR struct udp_conn_s *)conn->node.flink;
    }

  return conn;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: udp_ipv6_active
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used within the provided UDP header
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static inline FAR struct udp_conn_s *
  udp_ipv6_active(FAR struct net_driver_s *dev, FAR struct udp_hdr_s *udp)
{
  FAR struct ipv6_hdr_s *ip = IPv6BUF;
  FAR struct udp_conn_s *conn;

  conn = (FAR struct udp_conn_s *)g_active_udp_connections.head;
  while (conn != NULL)
    {
      /* If the local UDP port is non-zero, the connection is considered
       * to be used. If so, then the following checks are performed:
       *
       * 1. The destination address is verified against the bound address
       *    of the connection.
       *
       *    - The local port number is checked against the destination port
       *      number in the received packet.
       *    - If multiple network interfaces are supported, then the local
       *      IP address is available and we will insist that the
       *      destination IP matches the bound address. If a socket is bound
       *      to INADDR6_ANY (laddr), then it should receive all packets
       *      directed to the port. REVISIT: Should also depend on SO_BROADCAST.
       *
       * 2. If this is a connection mode UDP socket, then the source address
       *    is verified against the connected remote address.
       *
       *    - The remote port number is checked if the connection is bound
       *      to a remote port.
       *    - Finally, if the connection is bound to a remote IP address,
       *      the source IP address of the packet is checked.
       *
       * If all of the above are true then the newly received UDP packet
       * is destined for this UDP connection.
       *
       * To send and receive multicast packets, the application should:
       *
       *   - Bind socket to INADDR6_ANY (for the all-nodes multicast address)
       *     or to a specific <multicast-address>
       *   - setsockopt to SO_BROADCAST (for all-nodes address)
       *
       * For connection-less UDP sockets:
       *
       *   - call sendto with sendaddr.sin_addr.s_addr = <multicast-address>
       *   - call recvfrom.
       *
       * For connection-mode UDP sockets:
       *
       *   - call connect() to connect the UDP socket to a specific remote
       *     address, then
       *   - Call send() with no address address information
       *   - call recv() (from address information should not be needed)
       *
       * REVIST: SO_BROADCAST flag is currently ignored.
       */

      /* Check that there is a local port number and this is matches
       * the port number in the destination address.
       */

      if ((conn->lport != 0 && udp->destport == conn->lport &&

          /* Check if the local port accepts any address on this port or
           * that there is an exact match between the destipaddr and the
           * bound local address.  This catches the case of the all nodes
           * multicast when the socket is bound to the IPv6 unspecified
           * address.
           */

          (net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr) ||
           net_ipv6addr_hdrcmp(ip->destipaddr, conn->u.ipv6.laddr))))
        {
          /* Check if the socket is connection mode.  In this case, only
           * packets with source addresses from the connected remote peer
           * will be accepted.
           */

          if (_UDP_ISCONNECTMODE(conn->flags))
            {
              /* Check if the UDP connection is either (1) accepting packets
               * from any port or (2) the packet srcport matches the local
               * bound port number.
               */

              if ((conn->rport == 0 || udp->srcport == conn->rport) &&

              /* If (1) not connected to a remote address, or (2) a all-
               * nodes multicast destipaddr was received, or (3) there is an
               * exact match between the srcipaddr and the bound remote IP
               * address, then accept the packet.
               */

                  (net_ipv6addr_cmp(conn->u.ipv6.raddr, g_ipv6_unspecaddr) ||
#ifdef CONFIG_NET_BROADCAST
                   net_ipv6addr_hdrcmp(ip->destipaddr, g_ipv6_allnodes) ||
#endif
                   net_ipv6addr_hdrcmp(ip->srcipaddr, conn->u.ipv6.raddr)))
                {
                  /* Matching connection found.. Break out of the loop and
                   * return this reference to it.
                   */

                  break;
                }
            }
          else
            {
              /* This UDP socket is not connected.  We need to match only
               * the destination address with the bound socket address.
               * Break out out of the loop and return this reference to
               * the matching connection structure.
               */

              break;
            }
        }

      /* Look at the next active connection */

      conn = (FAR struct udp_conn_s *)conn->node.flink;
    }

  return conn;
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_initialize
 *
 * Description:
 *   Initialize the UDP connection structures.  Called once and only from
 *   the UIP layer.
 *
 ****************************************************************************/

void udp_initialize(void)
{
  int i;

  /* Initialize the queues */

  dq_init(&g_free_udp_connections);
  dq_init(&g_active_udp_connections);
  nxsem_init(&g_free_sem, 0, 1);

  for (i = 0; i < CONFIG_NET_UDP_CONNS; i++)
    {
      /* Mark the connection closed and move it to the free list */

      g_udp_connections[i].lport = 0;
      dq_addlast(&g_udp_connections[i].node, &g_free_udp_connections);
    }

  g_last_udp_port = 1024;
}

/****************************************************************************
 * Name: udp_alloc
 *
 * Description:
 *   Allocate a new, uninitialized UDP connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct udp_conn_s *udp_alloc(uint8_t domain)
{
  FAR struct udp_conn_s *conn;

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _udp_semtake(&g_free_sem);
  conn = (FAR struct udp_conn_s *)dq_remfirst(&g_free_udp_connections);
  if (conn)
    {
      /* Make sure that the connection is marked as uninitialized */

      conn->flags   = 0;
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
      conn->domain  = domain;
#endif
#ifdef CONFIG_NET_UDP_BINDTODEVICE
      conn->boundto = 0;  /* Not bound to any interface */
#endif
      conn->lport   = 0;
      conn->ttl     = IP_TTL;

#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
      /* Initialize the write buffer lists */

      sq_init(&conn->write_q);
#endif
      /* Enqueue the connection into the active list */

      dq_addlast(&conn->node, &g_active_udp_connections);
    }

  _udp_semgive(&g_free_sem);
  return conn;
}

/****************************************************************************
 * Name: udp_free
 *
 * Description:
 *   Free a UDP connection structure that is no longer in use. This should be
 *   done by the implementation of close().
 *
 ****************************************************************************/

void udp_free(FAR struct udp_conn_s *conn)
{
#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
  FAR struct udp_wrbuffer_s *wrbuffer;
#endif

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  _udp_semtake(&g_free_sem);
  conn->lport = 0;

  /* Remove the connection from the active list */

  dq_rem(&conn->node, &g_active_udp_connections);

#ifdef CONFIG_NET_UDP_READAHEAD
  /* Release any read-ahead buffers attached to the connection */

  iob_free_queue(&conn->readahead);
#endif

#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
  /* Release any write buffers attached to the connection */

  while ((wrbuffer = (struct udp_wrbuffer_s *)sq_remfirst(&conn->write_q)) != NULL)
    {
      udp_wrbuffer_release(wrbuffer);
    }
#endif

  /* Free the connection */

  dq_addlast(&conn->node, &g_free_udp_connections);
  _udp_semgive(&g_free_sem);
}

/****************************************************************************
 * Name: udp_active
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used within the provided UDP header
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct udp_conn_s *udp_active(FAR struct net_driver_s *dev,
                                  FAR struct udp_hdr_s *udp)
{
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      return udp_ipv6_active(dev, udp);
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      return udp_ipv4_active(dev, udp);
    }
#endif /* CONFIG_NET_IPv4 */
}

/****************************************************************************
 * Name: udp_nextconn
 *
 * Description:
 *   Traverse the list of allocated UDP connections
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct udp_conn_s *udp_nextconn(FAR struct udp_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct udp_conn_s *)g_active_udp_connections.head;
    }
  else
    {
      return (FAR struct udp_conn_s *)conn->node.flink;
    }
}

/****************************************************************************
 * Name: udp_bind
 *
 * Description:
 *   This function implements the low level parts of the standard UDP
 *   bind() operation.
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

int udp_bind(FAR struct udp_conn_s *conn, FAR const struct sockaddr *addr)
{
  uint16_t portno;
  int ret;

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
      FAR const struct sockaddr_in *inaddr =
        (FAR const struct sockaddr_in *)addr;

      /* Get the port number that we are binding to */

      portno = inaddr->sin_port;

      /* Bind the local IP address to the connection.  NOTE this address may
       * be INADDR_ANY meaning, essentially, that we are binding to all
       * interfaces for receiving (Sending will use the default port).
       */

      net_ipv4addr_copy(conn->u.ipv4.laddr, inaddr->sin_addr.s_addr);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      FAR const struct sockaddr_in6 *inaddr =
        (FAR const struct sockaddr_in6 *)addr;

      /* Get the port number that we are binding to */

      portno = inaddr->sin6_port;

      /* Bind the local IP address to the connection.  NOTE this address may
       * be INADDR_ANY meaning, essentially, that we are binding to all
       * interfaces for receiving (Sending will use the default port).
       */

      net_ipv6addr_copy(conn->u.ipv6.laddr, inaddr->sin6_addr.in6_u.u6_addr16);
    }
#endif /* CONFIG_NET_IPv6 */

  /* Is the user requesting to bind to any port? */

  if (portno == 0)
    {
      /* Yes.. Select any unused local port number */

      conn->lport = htons(udp_select_port(conn->domain, &conn->u));
      ret         = OK;
    }
  else
    {
      /* Interrupts must be disabled while access the UDP connection list */

      net_lock();

      /* Is any other UDP connection already bound to this address and port? */

      if (udp_find_conn(conn->domain, &conn->u, portno) == NULL)
        {
          /* No.. then bind the socket to the port */

          conn->lport = portno;
          ret         = OK;
        }
      else
        {
          ret         = -EADDRINUSE;
        }

      net_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: udp_connect
 *
 * Description:
 *   This function simply assigns a remote address to UDP "connection"
 *   structure.  This function is called as part of the implementation of:
 *
 *   - connect().  If connect() is called for a SOCK_DGRAM socket, then
 *       this logic performs the moral equivalent of connect() operation
 *       for the UDP socket.
 *   - recvfrom() and sendto().  This function is called to set the
 *       remote address of the peer.
 *
 *   The function will automatically allocate an unused local port for the
 *   new connection if the socket is not yet bound to a local address.
 *   However, another port can be chosen by using the udp_bind() call,
 *   after the udp_connect() function has been called.
 *
 * Input Parameters:
 *   conn - A reference to UDP connection structure.  A value of NULL will
 *          disconnect from any previously connected address.
 *   addr - The address of the remote host.
 *
 * Assumptions:
 *   This function is called (indirectly) from user code.  Interrupts may
 *   be enabled.
 *
 ****************************************************************************/

int udp_connect(FAR struct udp_conn_s *conn, FAR const struct sockaddr *addr)
{
  /* Has this address already been bound to a local port (lport)? */

  if (!conn->lport)
    {
      /* No.. Find an unused local port number and bind it to the
       * connection structure.
       */

      conn->lport = htons(udp_select_port(conn->domain, &conn->u));
    }

  /* Is there a remote port (rport)? */

  if (addr != NULL)
    {
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          FAR const struct sockaddr_in *inaddr =
            (FAR const struct sockaddr_in *)addr;

          conn->rport = inaddr->sin_port;
          net_ipv4addr_copy(conn->u.ipv4.raddr, inaddr->sin_addr.s_addr);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          FAR const struct sockaddr_in6 *inaddr =
            (FAR const struct sockaddr_in6 *)addr;

          conn->rport = inaddr->sin6_port;
          net_ipv6addr_copy(conn->u.ipv6.raddr, inaddr->sin6_addr.s6_addr16);
        }
#endif /* CONFIG_NET_IPv6 */
    }
  else
    {
      conn->rport = 0;

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          net_ipv4addr_copy(conn->u.ipv4.raddr, INADDR_ANY);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          net_ipv6addr_copy(conn->u.ipv6.raddr, g_ipv6_unspecaddr);
        }
#endif /* CONFIG_NET_IPv6 */
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
