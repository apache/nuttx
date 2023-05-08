/****************************************************************************
 * net/socket/net_fstat.c
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
 * Name: psock_fstat
 *
 * Description:
 *   Performs fstat operations on socket
 *
 * Input Parameters:
 *   psock  - The pointer of the socket to operate on
 *   buf    - Caller-provided location in which to return the fstat data
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int psock_fstat(FAR struct socket *psock, FAR struct stat *buf)
{
  FAR struct socket_conn_s *conn;
  int ret = OK;

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

  conn = psock->s_conn;

  if (conn == NULL || !_SS_ISCONNECTED(conn->s_flags))
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
           FAR struct tcp_conn_s *tcp_conn = psock->s_conn;

           /* For TCP, the MSS is a dynamic value that maintained in the
            * connection structure.
            */

           buf->st_blksize = tcp_conn->mss;
         }
         break;
#endif

#if defined(NET_UDP_HAVE_STACK)
       case SOCK_DGRAM:
         {
           FAR struct udp_conn_s *udp_conn = psock->s_conn;
           FAR struct net_driver_s *dev;
           uint16_t iplen;

           /* For a connected UDP socket, we have do do a little more work:
            *
            *   MSS = MTU - LL_HDRLEN - UDP_HDRLEN - IP_HDRLEN
            *
            * We need to have the device that services the connection in
            * order to get the MTU and LL_HDRLEN:
            */

           dev = udp_find_raddr_device(udp_conn, NULL);
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
               iplen = (udp_conn->domain == PF_INET) ? IPv4_HDRLEN :
                                                       IPv6_HDRLEN;
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
