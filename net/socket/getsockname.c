/****************************************************************************
 * net/socket/getsockname.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/socket.h>

#include <string.h>
#include <errno.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>

#include "utils/utils.h"
#include "netdev/netdev.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: get_ipv4_sockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified PF_NET socket.
 *
 * Parameters:
 *   psock    Point to the socket structure instance [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the error.
 *   Possible errno values that may be returned include:
 *
 *   EBADF - The socket argument is not a valid file descriptor.
 *   EOPNOTSUPP - The operation is not supported for this socket's protocol.
 *   EINVAL - The socket has been shut down.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int ipv4_getsockname(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen)
{
  FAR struct net_driver_s *dev;
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
  FAR struct sockaddr_in *outaddr = (FAR struct sockaddr_in *)addr;
#endif
#ifdef CONFIG_NETDEV_MULTINIC
  in_addr_t lipaddr;
  in_addr_t ripaddr;
#endif
  net_lock_t save;

  /* Check if enough space has been provided for the full address */

  if (*addrlen < sizeof(struct sockaddr_in))
    {
      /* This function is supposed to return the partial address if
       * a smaller buffer has been provided.  This support has not
       * been implemented.
       */

      return -ENOSYS;
    }

  /* Set the port number */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
          FAR struct tcp_conn_s *tcp_conn = (FAR struct tcp_conn_s *)psock->s_conn;
          outaddr->sin_port = tcp_conn->lport; /* Already in network byte order */
#ifdef CONFIG_NETDEV_MULTINIC
          lipaddr = tcp_conn->u.ipv4.laddr;
          ripaddr = tcp_conn->u.ipv4.raddr;
#endif
        }
        break;
#endif

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
          FAR struct udp_conn_s *udp_conn = (FAR struct udp_conn_s *)psock->s_conn;
          outaddr->sin_port = udp_conn->lport; /* Already in network byte order */
#ifdef CONFIG_NETDEV_MULTINIC
          lipaddr = udp_conn->u.ipv4.laddr;
          ripaddr = udp_conn->u.ipv4.raddr;
#endif
        }
        break;
#endif

      default:
        return -EOPNOTSUPP;
    }

  /* The socket/connection does not know its IP address unless
   * CONFIG_NETDEV_MULTINIC is selected.  Otherwise the design supports only
   * a single network device and only the network device knows the IP address.
   */

  save = net_lock();

#ifdef CONFIG_NETDEV_MULTINIC
  /* Find the device matching the IPv4 address in the connection structure */

  dev = netdev_findby_ipv4addr(lipaddr, ripaddr);
#else
  /* There is only one, the first network device in the list. */

  dev = g_netdevices;
#endif

  if (!dev)
    {
      net_unlock(save);
      return -EINVAL;
    }

  /* Set the address family and the IP address */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
  outaddr->sin_family = AF_INET;
  outaddr->sin_addr.s_addr = dev->d_ipaddr;
  *addrlen = sizeof(struct sockaddr_in);
#endif
  net_unlock(save);

  /* Return success */

  return OK;
}
#endif

/****************************************************************************
 * Function: ipv6_getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified PF_NET6 socket.
 *
 * Parameters:
 *   psock    Point to the socket structure instance [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the error.
 *   Possible errno values that may be returned include:
 *
 *   EBADF - The socket argument is not a valid file descriptor.
 *   EOPNOTSUPP - The operation is not supported for this socket's protocol.
 *   EINVAL - The socket has been shut down.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int ipv6_getsockname(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen)
{
  FAR struct net_driver_s *dev;
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
  FAR struct sockaddr_in6 *outaddr = (FAR struct sockaddr_in6 *)addr;
#endif
#ifdef CONFIG_NETDEV_MULTINIC
  net_ipv6addr_t *lipaddr;
  net_ipv6addr_t *ripaddr;
#endif

  /* Check if enough space has been provided for the full address */

  if (*addrlen < sizeof(struct sockaddr_in6))
  {
    /* This function is supposed to return the partial address if
     * a smaller buffer has been provided.  This support has not
     * been implemented.
     */

    return -ENOSYS;
  }

  /* Set the port number */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
          FAR struct tcp_conn_s *tcp_conn = (FAR struct tcp_conn_s *)psock->s_conn;
          outaddr->sin6_port = tcp_conn->lport; /* Already in network byte order */
#ifdef CONFIG_NETDEV_MULTINIC
          lipaddr            = &tcp_conn->u.ipv6.laddr;
          ripaddr            = &tcp_conn->u.ipv6.raddr;
#endif
        }
        break;
#endif

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
          FAR struct udp_conn_s *udp_conn = (FAR struct udp_conn_s *)psock->s_conn;
          outaddr->sin6_port = udp_conn->lport; /* Already in network byte order */
#ifdef CONFIG_NETDEV_MULTINIC
          lipaddr            = &udp_conn->u.ipv6.laddr;
          ripaddr            = &udp_conn->u.ipv6.raddr;
#endif
        }
        break;
#endif

      default:
        return -EOPNOTSUPP;
    }

  /* The socket/connection does not know its IP address unless
   * CONFIG_NETDEV_MULTINIC is selected.  Otherwise the design supports only
   * a single network device and only the network device knows the IP address.
   */

  save = net_lock();

#ifdef CONFIG_NETDEV_MULTINIC
  /* Find the device matching the IPv6 address in the connection structure */

  dev = netdev_findby_ipv6addr(*lipaddr, *ripaddr);
#else
  /* There is only one, the first network device in the list. */

  dev = g_netdevices;
#endif

  if (!dev)
    {
      net_unlock(save);
      return -EINVAL;
    }

  /* Set the address family and the IP address */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
  outaddr->sin6_family = AF_INET6;
  memcpy(outaddr->sin6_addr.in6_u.u6_addr8, dev->d_ipv6addr, 16);
  *addrlen = sizeof(struct sockaddr_in6);
#endif
  net_unlock(save);

  /* Return success */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the error.
 *   Possible errno values that may be returned include:
 *
 *   EBADF - The socket argument is not a valid file descriptor.
 *   EOPNOTSUPP - The operation is not supported for this socket's protocol.
 *   EINVAL - The socket has been shut down.
 *
 * Assumptions:
 *
 ****************************************************************************/

int getsockname(int sockfd, FAR struct sockaddr *addr, FAR socklen_t *addrlen)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  int ret;
  int err;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Some sanity checking... Shouldn't need this on a buckled up embedded
   * system (?)
   */

#ifdef CONFIG_DEBUG
  if (!addr || !addrlen)
    {
      err = EINVAL;
      goto errout;
    }
#endif

  /* Handle by address domain */

  switch (psock->s_domain)
    {
#ifdef CONFIG_NET_IPv4
    case PF_INET:
      ret = ipv4_getsockname(psock, addr, addrlen);
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case PF_INET6:
      ret = ipv6_getsockname(psock, addr, addrlen);
      break;
#endif

    case PF_PACKET:
    default:
      err = EAFNOSUPPORT;
      goto errout;
    }

  /* Check for failure */

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

#endif /* CONFIG_NET */
