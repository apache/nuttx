/****************************************************************************
 * net/socket/net_fstat.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "tcp/tcp.h"
#include "udp/udp.h"
#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_fstat
 *
 * Description:
 *   Performs fstat operations on socket
 *
 * Input Parameters:
 *   sockfd - Socket descriptor of the socket to operate on
 *   buf    - Caller-provided location in which to return the fstat data
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int net_fstat(int sockfd, FAR struct stat *buf)
{
  FAR struct socket *psock;
  int ret = OK;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);
  if (psock == NULL)
    {
      /* sockfd does not refer to a valid, open socket */

      return -EBADF;
    }

  /* Return fstat data.  The st_mode and st_blksize fields are the only
   * fields set for socket descriptors.  The st_mode field is set to a value
   * that indicates the descriptor is a socket descriptor and the st_blksize
   * field is set to an optimal value determined by the system.  The optimal
   * packet size is the MSS.
   */

  memset(buf, 0, sizeof(struct stat));
  buf->st_mode = S_IFSOCK;

  /* The socket must be open and in a connected state in order to get the
   * MSS.  There may be multiple networks served by different network
   * devices, each supporting a different MSS.
   */

  if (psock->s_conn == NULL || !_SS_ISCONNECTED(psock->s_flags))
    {
      /* Not connected.. Return an optimal blocksize of zero (or, perhaps,
       * even an error?)
       *
       * REVISIT:  The concept of connected only applies to TCP and UDP
       * sockets.  Other sockets, such raw radio sockets, have no such
       * concept.
       */

      nwarn("WARNING:  Socket not connected\n");
      return OK;
    }

  /* We are only prepared to handle the MSS of connected TCP/IP and UDP
   * sockets here.
   */

  switch (psock->s_type)
    {
#if defined(NET_TCP_HAVE_STACK)
       case SOCK_STREAM:
         {
           FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)psock->s_conn;

           /* For TCP, the MSS is a dynamic value that maintained in the
            * connection structure.
            */

           buf->st_blksize = conn->mss;
         }
         break;
#endif

#if defined(NET_UDP_HAVE_STACK)
       case SOCK_DGRAM:
         {
           FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)psock->s_conn;
           FAR struct net_driver_s *dev;
           uint16_t iplen;

           /* For a connected UDP socket, we have do do a little more work:
            *
            *   MSS = MTU - LL_HDRLEN - UDP_HDRLEN - IP_HDRLEN
            *
            * We need to have the device that services the connection in order
            * to get the MTU and LL_HDRLEN:
            */

           dev = udp_find_raddr_device(conn);
           if (dev == NULL)
             {
               /* This should never happen except perhaps in some rare race
                * condition.  If the UDP socket is connected, then the device
                * service the network that it is connected to should always
                * exist.
                */

               nerr("ERROR:  Could not find network device\n");
               ret = -ENODEV;
             }
           else
             {
               /* We need the length of the IP header */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
               iplen = (conn->domain == PF_INET) ? IPv4_HDRLEN : IPv6_HDRLEN;
#elif defined(CONFIG_NET_IPv4)
               iplen = IPv4_HDRLEN;
#else
               iplen = IPv6_HDRLEN;
#endif
               /* Now we can calculate the MSS */

               buf->st_blksize = UDP_MSS(dev, iplen);
             }
         }
         break;
#endif
       default:
         nwarn("WARNING:  Unhandled socket type: %u\n", psock->s_type);
         break;
     }

  return ret;
}
