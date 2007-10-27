/****************************************************************************
 * net/connect.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <errno.h>

#include "net-internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Function: connection_event
 *
 * Description:
 *   Some connection related event has occurred
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   private  An instance of struct recvfrom_s cast to void*
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static void connection_event(void *private)
{
  FAR struct socket *psock = (FAR struct socket *)private;
  if (psock)
    {
      /* UIP_CLOSE: The remote host has closed the connection
       * UIP_ABORT: The remote host has aborted the connection
       * UIP_TIMEDOUT: Connection aborted due to too many retransmissions.
       */
      if ((uip_flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
        {
          /* Indicate that the socet is no longer connected */

          psock->s_flags &= ~_SF_CONNECTED;
        }

      /* UIP_CONNECTED: The socket is successfully connected */

      else if ((uip_flags & UIP_CONNECTED) != 0)
        {
          /* Indicate that the socet is no longer connected */

          psock->s_flags |= _SF_CONNECTED;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: connect
 *
 * Description:
 *   connect() connects the socket referred to by the file descriptor sockfd
 *   to the address specified by 'addr'. The addrlen argument specifies
 *   the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket sockfd.
 *
 *   If the socket sockfd is of type SOCK_DGRAM then 'addr' is the address
 *   to which datagrams are sent by default, and the only address from which
 *   datagrams are received. If the socket is of type SOCK_STREAM or
 *   SOCK_SEQPACKET, this call attempts to make a connection to the socket
 *   that is bound to the address specified by 'addr'.
 *
 *   Generally, connection-based protocol sockets may successfully connect()
 *   only once; connectionless protocol sockets may use connect() multiple
 *   times to change their association.  Connectionless sockets may dissolve
 *   the association by connecting to an address with the sa_family member of
 *   sockaddr set to AF_UNSPEC.
 *
 * Parameters:
 *   sockfd    Socket descriptor returned by socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *     EACCES, EPERM
 *       The user tried to connect to a broadcast address without having the
 *       socket broadcast flag enabled or the connection request failed
 *       because of a local firewall rule.
 *     EADDRINUSE
 *       Local address is already in use.
 *     EAFNOSUPPORT
 *       The passed address didn't have the correct address family in its
 *       sa_family field.
 *     EAGAIN
 *       No more free local ports or insufficient entries in the routing
 *       cache.
 *     EALREADY
 *       The socket is non-blocking and a previous connection attempt has
 *       not yet been completed.
 *     EBADF
 *       The file descriptor is not a valid index in the descriptor table.
 *     ECONNREFUSED
 *       No one listening on the remote address.
 *     EFAULT
 *       The socket structure address is outside the user's address space.
 *     EINPROGRESS
 *       The socket is non-blocking and the connection cannot be completed
 *       immediately.
 *     EINTR
 *       The system call was interrupted by a signal that was caught.
 *     EISCONN
 *       The socket is already connected.
 *     ENETUNREACH
 *       Network is unreachable.
 *     ENOTSOCK
 *       The file descriptor is not associated with a socket.
 *     ETIMEDOUT
 *       Timeout while attempting connection. The server may be too busy
 *       to accept new connections.
 *
 * Assumptions:
 *
 ****************************************************************************/

int connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
#ifdef CONFIG_NET_IPv6
  FAR const struct sockaddr_in6 *inaddr = (const struct sockaddr_in6 *)addr;
#else
  FAR const struct sockaddr_in *inaddr = (const struct sockaddr_in *)addr;
#endif
  int err;
  int ret;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Verify that a valid address has been provided */

#ifdef CONFIG_NET_IPv6
  if (addr->sa_family != AF_INET6 || addrlen < sizeof(struct sockaddr_in6))
#else
  if (addr->sa_family != AF_INET || addrlen < sizeof(struct sockaddr_in))
#endif
  {
      err = EBADF;
      goto errout;
  }

  /* Perform the connection depending on the protocol type */

  switch (psock->s_type)
    {
      case SOCK_STREAM:
        {
          struct uip_conn *conn;

          /* Verify that the socket is not already connected */

          if (_SS_ISCONNECTED(psock->s_flags))
            {
              err = -EISCONN;
              goto errout;
            }

          /* Get the connection reference from the socket */

          conn = psock->s_conn;
          if (conn) /* Should alwasy be non-NULL */
            {
              /* Perform the uIP connection operation */

              ret = uip_tcpconnect(psock->s_conn, inaddr);
              if (ret < 0)
                {
                  err = -ret;
                  goto errout;
                }

              /* Mark the connection bound and connected */

              psock->s_flags |= (_SF_BOUND|_SF_CONNECTED);

              /* Set up to receive callbacks on connection-related events */

              conn->connection_private = (void*)psock;
              conn->connection_event   = connection_event;
            }
        }
        break;

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
          ret = uip_udpconnect(psock->s_conn, inaddr);
          if (ret < 0)
            {
              err = -ret;
              goto errout;
            }
        }
        break;
#endif

      default:
        err = EBADF;
        goto errout;
    }

  return OK;

errout:
  *get_errno_ptr() = err;
  return ERROR;
}

#endif /* CONFIG_NET */
