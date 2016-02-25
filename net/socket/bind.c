/****************************************************************************
 * net/socket/bind.c
 *
 *   Copyright (C) 2007-2009, 2012, 2014-2016 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#ifdef CONFIG_NET_PKT
#  include <netpacket/packet.h>
#endif

#include <nuttx/net/net.h>
#include <nuttx/net/udp.h>

#include "socket/socket.h"
#include "netdev/netdev.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "local/local.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: pkt_bind
 *
 * Description:
 *   Bind a raw socket to an network device.
 *
 * Parameters:
 *   conn     AF_PACKET connection structure
 *   addr     Peer address information
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PKT
static int pkt_bind(FAR struct pkt_conn_s *conn,
                    FAR const struct sockaddr_ll *addr)
{
  int ifindex;
#if 0
  char hwaddr[6] =  /* our MAC for debugging */
  {
    0x00, 0xa1, 0xb1, 0xc1, 0xd1, 0xe1
  };
#endif
  char hwaddr[6] =  /* MAC from ifconfig */
  {
    0x00, 0xe0, 0xde, 0xad, 0xbe, 0xef
  };

  /* Look at the addr and identify network interface */

  ifindex = addr->sll_ifindex;

#if 0
  /* Get the MAC address of that interface */

  memcpy(hwaddr, g_netdevices->d_mac, 6);
#endif

  /* Put ifindex and mac address into connection */

  conn->ifindex = ifindex;
  memcpy(conn->lmac, hwaddr, 6);

  return OK;
}
#endif /* CONFIG_NET_PKT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_bind
 *
 * Description:
 *   bind() gives the socket 'psock' the local address 'addr'. 'addr' is
 *   'addrlen' bytes long. Traditionally, this is called "assigning a name
 *   to a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Parameters:
 *   psock    Socket structure of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     psock is a descriptor for a file, not a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_bind(FAR struct socket *psock, const struct sockaddr *addr,
               socklen_t addrlen)
{
#ifdef CONFIG_NET_PKT
  FAR const struct sockaddr_ll *lladdr = (const struct sockaddr_ll *)addr;
#endif
  socklen_t minlen;
  int err;
  int ret = OK;

  /* Verify that the psock corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = ENOTSOCK;
      goto errout;
    }

  /* Verify that a valid address has been provided */

  switch (addr->sa_family)
    {
#ifdef CONFIG_NET_IPv4
    case AF_INET:
      minlen = sizeof(struct sockaddr_in);
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case AF_INET6:
      minlen = sizeof(struct sockaddr_in6);
      break;
#endif

#ifdef CONFIG_NET_LOCAL
    case AF_LOCAL:
      minlen = sizeof(sa_family_t);
      break;
#endif

#ifdef CONFIG_NET_PKT
    case AF_PACKET:
      minlen = sizeof(struct sockaddr_ll);
      break;
#endif

    default:
      ndbg("ERROR: Unrecognized address family: %d\n", addr->sa_family);
      err = EAFNOSUPPORT;
      goto errout;
    }

  if (addrlen < minlen)
    {
      ndbg("ERROR: Invalid address length: %d < %d\n", addrlen, minlen);
      err = EBADF;
      goto errout;
    }

  /* Perform the binding depending on the protocol type */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_PKT
      case SOCK_RAW:
        ret = pkt_bind(psock->s_conn, lladdr);
        break;
#endif

      /* Bind a stream socket which may either be TCP/IP or a local, Unix
       * domain socket.
       */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL_STREAM)
      case SOCK_STREAM:
        {
#ifdef CONFIG_NET_LOCAL_STREAM
#ifdef CONFIG_NET_TCP
          /* Is this a Unix domain socket? */

          if (psock->s_domain == PF_LOCAL)
#endif
            {
              /* Bind the Unix domain connection structure */

              ret = psock_local_bind(psock, addr, addrlen);
            }
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_LOCAL_STREAM
          else
#endif
            {
              /* Bind the TCP/IP connection structure */

              ret = tcp_bind(psock->s_conn, addr);
            }
#endif /* CONFIG_NET_TCP */

          /* Mark the socket bound */

          if (ret >= 0)
            {
              psock->s_flags |= _SF_BOUND;
            }
        }
        break;
#endif /* CONFIG_NET_TCP || CONFIG_NET_LOCAL_STREAM */

      /* Bind a datagram socket which may either be TCP/IP or a local, Unix
       * domain socket.
       */

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL_DGRAM)
      case SOCK_DGRAM:
        {
#ifdef CONFIG_NET_LOCAL_DGRAM
#ifdef CONFIG_NET_UDP
          /* Is this a Unix domain socket? */

          if (psock->s_domain == PF_LOCAL)
#endif
            {
              /* Bind the Unix domain connection structure */

              ret = psock_local_bind(psock, addr, addrlen);
            }
#endif /* CONFIG_NET_LOCAL_DGRAM */

#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_LOCAL_DGRAM
          else
#endif
            {
              /* Bind the UDPP/IP connection structure */

              ret = udp_bind(psock->s_conn, addr);
            }
#endif /* CONFIG_NET_UDP */

          /* Mark the socket bound */

          if (ret >= 0)
            {
              psock->s_flags |= _SF_BOUND;
            }
        }
        break;
#endif /* CONFIG_NET_UDP || CONFIG_NET_LOCAL_DGRAM */

      default:
        err = EBADF;
        goto errout;
    }

  /* Was the bind successful */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  return OK;

errout:
  set_errno(err);
  return ERROR;
}

/****************************************************************************
 * Function: bind
 *
 * Description:
 *   bind() gives the socket 'sockfd' the local address 'addr'. 'addr' is
 *   'addrlen' bytes long. Traditionally, this is called "assigning a name to
 *   a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Parameters:
 *   sockfd   Socket descriptor of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EBADF
 *     sockfd is not a valid descriptor.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     sockfd is a descriptor for a file, not a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
  /* Make the socket descriptor to the underlying socket structure */

  FAR struct socket *psock = sockfd_socket(sockfd);

  /* Then let psock_bind do all of the work */

  return psock_bind(psock, addr, addrlen);
}

#endif /* CONFIG_NET */
