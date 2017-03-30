/****************************************************************************
 * net/sixlowpan/sixlowpan_udpsend.c
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

#include "nuttx/net/netdev.h"
#include "nuttx/net/udp.h"
#include "nuttx/net/sixlowpan.h"

#include "netdev/netdev.h"
#include "socket/socket.h"
#include "udp/udp.h"
#include "sixlowpan/sixlowpan_internal.h"

#if defined(CONFIG_NET_6LOWPAN) && defined(CONFIG_NET_UDP)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

ssize_t psock_6lowpan_udp_send(FAR struct socket *psock, FAR const void *buf,
                               size_t len)
{
  FAR struct udp_conn_s *conn;
  FAR struct net_driver_s *dev;
  struct ipv6udp_hdr_s ipv6udp;
  struct rimeaddr_s dest;
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

  /* Initialize the IPv6/UDP headers */
#warning Missing logic

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Get the Rime MAC address of the destination */
#warning Missing logic

  /* If routable, then call sixlowpan_send() to format and send the 6loWPAN
   * packet.
   */

  ret = sixlowpan_send(dev, (FAR const struct ipv6_hdr_s *)&ipv6udp,
                       buf, len, &dest);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_send() failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_NET_6LOWPAN && CONFIG_NET_UDP */
