/****************************************************************************
 * net/socket/connect.c
 *
 *   Copyright (C) 2007-2012, 2015 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "local/local.h"
#include "socket/socket.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
struct tcp_connect_s
{
  FAR struct tcp_conn_s  *tc_conn;    /* Reference to TCP connection structure */
  FAR struct devif_callback_s *tc_cb; /* Reference to callback instance */
  FAR struct socket *tc_psock;        /* The socket being connected */
  sem_t tc_sem;                       /* Semaphore signals recv completion */
  int tc_result;                      /* OK on success, otherwise a negated errno. */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline int psock_setup_callbacks(FAR struct socket *psock,
                                        FAR struct tcp_connect_s *pstate);
static void psock_teardown_callbacks(FAR struct tcp_connect_s *pstate,
                                     int status);
static uint16_t psock_connect_interrupt(FAR struct net_driver_s *dev,
                                        FAR void *pvconn, FAR void *pvpriv,
                                        uint16_t flags);
static inline int psock_tcp_connect(FAR struct socket *psock,
                                    FAR const struct sockaddr *addr);
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: psock_setup_callbacks
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline int psock_setup_callbacks(FAR struct socket *psock,
                                        FAR struct tcp_connect_s *pstate)
{
  FAR struct tcp_conn_s *conn = psock->s_conn;
  int ret = -EBUSY;

  /* Initialize the TCP state structure */

  (void)sem_init(&pstate->tc_sem, 0, 0); /* Doesn't really fail */
  pstate->tc_conn   = conn;
  pstate->tc_psock  = psock;
  pstate->tc_result = -EAGAIN;

  /* Set up the callbacks in the connection */

  pstate->tc_cb = tcp_callback_alloc(conn);
  if (pstate->tc_cb)
    {
      /* Set up the connection "interrupt" handler */

      pstate->tc_cb->flags   = (TCP_NEWDATA | TCP_CLOSE | TCP_ABORT |
                                TCP_TIMEDOUT | TCP_CONNECTED | NETDEV_DOWN);
      pstate->tc_cb->priv    = (void*)pstate;
      pstate->tc_cb->event   = psock_connect_interrupt;
      ret                    = OK;
    }

  return ret;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Name: psock_teardown_callbacks
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static void psock_teardown_callbacks(FAR struct tcp_connect_s *pstate,
                                     int status)
{
  FAR struct tcp_conn_s *conn = pstate->tc_conn;

  /* Make sure that no further interrupts are processed */

  tcp_callback_free(conn, pstate->tc_cb);
  pstate->tc_cb = NULL;

  /* If we successfully connected, we will continue to monitor the connection
   * state via callbacks.
   */

  if (status < 0)
    {
      /* Failed to connect. Stop the connection event monitor */

      net_stopmonitor(conn);
    }
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Name: psock_connect_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   connection operation via by the lower, device interfacing layer.
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
 *   pvconn   The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   The new flags setting
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static uint16_t psock_connect_interrupt(FAR struct net_driver_s *dev,
                                        FAR void *pvconn, FAR void *pvpriv,
                                        uint16_t flags)
{
  struct tcp_connect_s *pstate = (struct tcp_connect_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* The following errors should be detected here (someday)
       *
       *     ECONNREFUSED
       *       No one listening on the remote address.
       *     ENETUNREACH
       *       Network is unreachable.
       *     ETIMEDOUT
       *       Timeout while attempting connection. The server may be too busy
       *       to accept new connections.
       */

      /* TCP_CLOSE: The remote host has closed the connection
       * TCP_ABORT: The remote host has aborted the connection
       */

      if ((flags & (TCP_CLOSE | TCP_ABORT)) != 0)
        {
          /* Indicate that remote host refused the connection */

          pstate->tc_result = -ECONNREFUSED;
        }

      /* TCP_TIMEDOUT: Connection aborted due to too many retransmissions. */

      else if ((flags & TCP_TIMEDOUT) != 0)
        {
          /* Indicate that the connection timedout?)*/

          pstate->tc_result = -ETIMEDOUT;
        }

      else if ((flags & NETDEV_DOWN) != 0)
        {
          /* The network device went down.  Indicate that the remote host
           * is unreachable.
           */

          pstate->tc_result = -ENETUNREACH;
        }

      /* TCP_CONNECTED: The socket is successfully connected */

      else if ((flags & TCP_CONNECTED) != 0)
        {
          FAR struct socket *psock = pstate->tc_psock;
          DEBUGASSERT(psock);

          /* Mark the connection bound and connected.  NOTE this is
           * is done here (vs. later) in order to avoid any race condition
           * in the socket state.  It is known to connected here and now,
           * but not necessarily at any time later.
           */

          psock->s_flags |= (_SF_BOUND | _SF_CONNECTED);

          /* Indicate that the socket is no longer connected */

          pstate->tc_result = OK;
        }

      /* Otherwise, it is not an event of importance to us at the moment */

      else
        {
          /* Drop data received in this state */

          dev->d_len = 0;
          return flags & ~TCP_NEWDATA;
        }

      nllvdbg("Resuming: %d\n", pstate->tc_result);

      /* Stop further callbacks */

      psock_teardown_callbacks(pstate, pstate->tc_result);

#ifdef CONFIG_NET_MULTILINK
      /* When we set up the connection structure, we did not know the size
       * of the initial MSS.  Now that the connection is associated with a
       * network device, we now know the size of link layer header and can
       * determine the correct initial MSS.
       */

      DEBUGASSERT(pstate->tc_conn);

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (pstate->tc_conn->domain == PF_INET)
#endif
    {
      pstate->tc_conn->mss = TCP_IPv4_INITIAL_MSS(dev);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      pstate->tc_conn->mss = TCP_IPv4_INITIAL_MSS(dev);
    }
#endif /* CONFIG_NET_IPv6 */

#endif /* CONFIG_NET_MULTILINK */

      /* Wake up the waiting thread */

      sem_post(&pstate->tc_sem);
    }

  return flags;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Name: psock_tcp_connect
 *
 * Description:
 *   Perform a TCP connection
 *
 * Parameters:
 *   psock - A reference to the socket structure of the socket to be connected
 *   addr  - The address of the remote server to connect to
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline int psock_tcp_connect(FAR struct socket *psock,
                                    FAR const struct sockaddr *addr)
{
  struct tcp_connect_s state;
  net_lock_t           flags;
  int                  ret = OK;

  /* Interrupts must be disabled through all of the following because
   * we cannot allow the network callback to occur until we are completely
   * setup.
   */

  flags = net_lock();

  /* Get the connection reference from the socket */

  if (!psock->s_conn) /* Should always be non-NULL */
    {
      ret = -EINVAL;
    }
  else
    {
      /* Perform the TCP connection operation */

      ret = tcp_connect(psock->s_conn, addr);
    }

  if (ret >= 0)
    {
      /* Set up the callbacks in the connection */

      ret = psock_setup_callbacks(psock, &state);
      if (ret >= 0)
        {
          /* Wait for either the connect to complete or for an error/timeout
           * to occur. NOTES:  (1) net_lockedwait will also terminate if a signal
           * is received, (2) interrupts may be disabled!  They will be re-
           * enabled while the task sleeps and automatically re-disabled
           * when the task restarts.
           */

          ret = net_lockedwait(&state.tc_sem);

          /* Uninitialize the state structure */

          (void)sem_destroy(&state.tc_sem);

          /* If net_lockedwait failed, recover the negated error (probably -EINTR) */

          if (ret < 0)
            {
              ret = -errno;
            }
          else
            {
              /* If the wait succeeded, then get the new error value from
               * the state structure
               */

              ret = state.tc_result;
            }

          /* Make sure that no further interrupts are processed */

          psock_teardown_callbacks(&state, ret);
        }

      /* Check if the socket was successfully connected. */

      if (ret >= 0)
        {
          /* Yes... Now that we are connected, we need to set up to monitor
           * the state of the connection up the connection event monitor.
           */

          ret = net_startmonitor(psock);
          if (ret < 0)
            {
              /* net_startmonitor() can only fail on certain race
               * conditions where the connection was lost just before
               * this function was called.  That is not expected to
               * happen in this context, but just in case...
               */

              net_lostconnection(psock, TCP_ABORT);
            }
        }
    }

  net_unlock(flags);
  return ret;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: psock_connect
 *
 * Description:
 *   connect() connects the socket referred to by the structure 'psock'
 *   to the address specified by 'addr'. The addrlen argument specifies
 *   the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'psock'.
 *
 *   If the socket 'psock' is of type SOCK_DGRAM then 'addr' is the address
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
 *   psock     Pointer to a socket structure initialized by psock_socket()
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

int psock_connect(FAR struct socket *psock, FAR const struct sockaddr *addr,
                  socklen_t addrlen)
{
  FAR const struct sockaddr_in *inaddr = (FAR const struct sockaddr_in *)addr;
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL)
  int ret;
#endif
  int err;

  /* Verify that the psock corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Verify that a valid address has been provided */

  switch (inaddr->sin_family)
    {
#ifdef CONFIG_NET_IPv4
    case AF_INET:
      {
        if (addrlen < sizeof(struct sockaddr_in))
          {
            err = EBADF;
            goto errout;
          }
      }
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case AF_INET6:
      {
        if (addrlen < sizeof(struct sockaddr_in6))
          {
            err = EBADF;
            goto errout;
          }
      }
      break;
#endif

#ifdef CONFIG_NET_LOCAL
    case AF_LOCAL:
      {
        if (addrlen < sizeof(sa_family_t))
          {
            err = EBADF;
            goto errout;
          }
      }
      break;
#endif

    default:
      DEBUGPANIC();
      err = EAFNOSUPPORT;
      goto errout;
    }

  /* Perform the connection depending on the protocol type */

  switch (psock->s_type)
    {
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL_STREAM)
      case SOCK_STREAM:
        {
          /* Verify that the socket is not already connected */

          if (_SS_ISCONNECTED(psock->s_flags))
            {
              err = EISCONN;
              goto errout;
            }

          /* It's not ... connect it */

#ifdef CONFIG_NET_LOCAL_STREAM
#ifdef CONFIG_NET_TCP
          if (psock->s_domain == PF_LOCAL)
#endif
            {
              /* Connect to the local Unix domain server */

              ret = psock_local_connect(psock, addr);
            }
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_LOCAL_STREAM
          else
#endif
            {
              /* Connect the TCP/IP socket */

              ret = psock_tcp_connect(psock, addr);
            }
#endif /* CONFIG_NET_TCP */

          if (ret < 0)
            {
              err = -ret;
              goto errout;
            }
        }
        break;
#endif /* CONFIG_NET_TCP || CONFIG_NET_LOCAL_STREAM */

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL_DGRAM)
      case SOCK_DGRAM:
        {
#ifdef CONFIG_NET_LOCAL_DGRAM
#ifdef CONFIG_NET_UDP
          if (psock->s_domain == PF_LOCAL)
#endif
            {
              /* Perform the datagram connection logic */

              ret = psock_local_connect(psock, addr);
            }
#endif /* CONFIG_NET_LOCAL_DGRAM */

#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_LOCAL_DGRAM
          else
#endif
            {
              ret = udp_connect(psock->s_conn, addr);
            }
#endif /* CONFIG_NET_UDP */

          if (ret < 0)
            {
              err = -ret;
              goto errout;
            }
        }
        break;
#endif /* CONFIG_NET_UDP || CONFIG_NET_LOCAL_DGRAM */

      default:
        err = EBADF;
        goto errout;
    }

  return OK;

errout:
   errno = err;
  return ERROR;
}

/****************************************************************************
 * Name: connect
 *
 * Description:
 *   connect() connects the socket referred to by the file descriptor 'sockfd'
 *   to the address specified by 'addr'. The addrlen argument specifies
 *   the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'sockfd'.
 *
 *   If the socket 'sockfd' is of type SOCK_DGRAM then 'addr' is the address
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

int connect(int sockfd, FAR const struct sockaddr *addr, socklen_t addrlen)
{
  /* Get the underlying socket structure */

  FAR struct socket *psock = sockfd_socket(sockfd);

  /* Then let psock_connect() do all of the work */

  return psock_connect(psock, addr, addrlen);
}

#endif /* CONFIG_NET */
