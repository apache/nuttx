/****************************************************************************
 * net/sixlowpan/sixlowpan_send.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "nuttx/net/net.h"
#include "nuttx/net/netdev.h"

#include "netdev/netdev.h"
#include "socket/socket.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "sixlowpan/sixlowpan.h"

#ifdef CONFIG_NET_6LOWPAN


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_send
 *
 * Description:
 *   Process an outgoing UDP or TCP packet.  Takes an IP packet and formats
 *   it to be sent on an 802.15.4 network using 6lowpan.  Called from common
 *   UDP/TCP send logic.
 *
 *  The payload data is in the caller 'buf' and is of length 'len'.
 *  Compressed headers will be added and if necessary the packet is
 *  fragmented. The resulting packet/fragments are put in dev->d_buf and
 *  the first frame will be delivered to the 802.15.4 MAC. via ieee->i_frame.
 *
 * Input Parmeters:
 *
 * Input Parameters:
 *   dev   - The IEEE802.15.4 MAC network driver interface.
 *   raddr - The MAC address of the destination
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *   This function is expected to fail if the driver is not an IEEE802.15.4
 *   MAC network driver.  In that case, the UDP/TCP will fall back to normal
 *   IPv4/IPv6 formatting.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_send(FAR struct net_driver_s *dev, net_ipv6addr_t raddr)
{
  FAR struct ieee802154_driver_s *ieee = (FAR struct ieee802154_driver_s *)dev;

  net_lock();
  /* REVISIT: To be provided */
  net_unlock();
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_6lowpan_tcp_send
 *
 * Description:
 *   psock_6lowpan_tcp_send() call may be used only when the TCP socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Parameters:
 *   psock - An instance of the internal socket structure.
 *   buf   - Data to send
 *   len   - Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.  Returned error numbers
 *   must be consistent with definition of errors reported by send() or
 *   sendto().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
ssize_t psock_6lowpan_tcp_send(FAR struct socket *psock, FAR const void *buf,
                               size_t len)
{
  FAR struct tcp_conn_s *conn;
  FAR struct net_driver_s *dev;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_crefs > 0);
  DEBUGASSERT(psock->s_type == SOCK_STREAM);

  /* Make sure that this is a valid socket */

  if (psock != NULL || psock->s_crefs <= 0)
    {
      nerr("ERROR: Invalid socket\n");
      return (ssize_t)-EBADF;
    }

  /* Make sure that this is a connected TCP socket */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      nerr("ERROR: Not connected\n");
      return (ssize_t)-ENOTCONN;
    }

  /* Get the underlying TCP connection structure */

  conn = (FAR struct tcp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  /* Ignore if not IPv6 domain */

  if (conn->domain != PF_INET6)
    {
      nwarn("WARNING: Not IPv6\n");
      return (ssize_t)-EPROTOTYPE;
    }
#endif

  /* Route outgoing message to the correct device */

#ifdef CONFIG_NETDEV_MULTINIC
  dev = netdev_findby_ipv6addr(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
  if (dev == NULL || dev->d_lltype != NET_LL_IEEE805154)
    {
      nwarn("WARNING: Not routable or not IEEE802.15.4 MAC\n");
      return (ssize_t)-ENETUNREACH;
    }
#else
  dev = netdev_findby_ipv6addr(conn->u.ipv6.raddr);
  if (dev == NULL)
    {
      nwarn("WARNING: Not routable\n");
      return (ssize_t)-ENETUNREACH;
    }
#endif

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  /* Make sure that the IP address mapping is in the Neighbor Table */

  ret = icmpv6_neighbor(conn->u.ipv6.raddr);
  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      return (ssize_t)-ENETUNREACH;
    }
#endif

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* If routable, then call sixlowpan_send() to format and send the 6loWPAN
   * packet.
   */

  ret = sixlowpan_send(dev, conn->u.ipv6.raddr);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_send() failed: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Function: psock_6lowpan_udp_send
 *
 * Description:
 *   psock_6lowpan_udp_send() call may be used with connectionlesss UDP
 *   sockets.
 *
 * Parameters:
 *   psock - An instance of the internal socket structure.
 *   buf   - Data to send
 *   len   - Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.  Returned error numbers
 *   must be consistent with definition of errors reported by send() or
 *   sendto().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
ssize_t psock_6lowpan_udp_send(FAR struct socket *psock, FAR const void *buf,
                               size_t len)
{
  FAR struct udp_conn_s *conn;
  FAR struct net_driver_s *dev;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_crefs > 0);
  DEBUGASSERT(psock->s_type == SOCK_DGRAM);

  /* Make sure that this is a valid socket */

  if (psock != NULL || psock->s_crefs <= 0)
    {
      nerr("ERROR: Invalid socket\n");
      return (ssize_t)-EBADF;
    }

  /* Was the UDP socket connected via connect()? */

  if (psock->s_type != SOCK_DGRAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      /* No, then it is not legal to call send() with this socket. */

      return -ENOTCONN;
    }

  /* Get the underlying UDP "connection" structure */

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  /* Ignore if not IPv6 domain */

  if (conn->domain != PF_INET6)
    {
      nwarn("WARNING: Not IPv6\n");
      return (ssize_t)-EPROTOTYPE;
    }
#endif

  /* Route outgoing message to the correct device */

#ifdef CONFIG_NETDEV_MULTINIC
  dev = netdev_findby_ipv6addr(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
  if (dev == NULL || dev->d_lltype != NET_LL_IEEE805154)
    {
      nwarn("WARNING: Not routable or not IEEE802.15.4 MAC\n");
      return (ssize_t)-ENETUNREACH;
    }
#else
  dev = netdev_findby_ipv6addr(conn->u.ipv6.raddr);
  if (dev == NULL)
    {
      nwarn("WARNING: Not routable\n");
      return (ssize_t)-ENETUNREACH;
    }
#endif

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  /* Make sure that the IP address mapping is in the Neighbor Table */

  ret = icmpv6_neighbor(conn->u.ipv6.raddr);
  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      return (ssize_t)-ENETUNREACH;
    }
#endif

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* If routable, then call sixlowpan_send() to format and send the 6loWPAN
   * packet.
   */

  ret = sixlowpan_send(dev, conn->u.ipv6.raddr);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_send() failed: %d\n", ret);
    }

  return ret;
}
#endif

#endif /* CONFIG_NET_6LOWPAN */
