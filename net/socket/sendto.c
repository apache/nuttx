/****************************************************************************
 * net/socket/sendto.c
 *
 *   Copyright (C) 2007-2009, 2011-2014 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "arp/arp.h"
#include "udp/udp.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If both IPv4 and IPv6 support are both enabled, then we will need to build
 * in some additional domain selection support.
 */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
#  define NEED_IPDOMAIN_SUPPORT 1
#endif

/* Timeouts on sendto() do not make sense.  Each polling cycle from the
 * driver is an opportunity to send a packet.  If the driver is not polling,
 * then the network is not up (and there are no polling cycles to drive
 * the timeout).
 *
 * There is a remote possibility that if there is a lot of other network
 * traffic that a UDP sendto could get delayed, but I would not expect this
 * generate a timeout.
 */

#undef CONFIG_NET_SENDTO_TIMEOUT

/* If supported, the sendto timeout function would depend on socket options
 * and a system clock.
 */

#ifndef CONFIG_NET_SOCKOPTS
#  undef CONFIG_NET_SENDTO_TIMEOUT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sendto_s
{
#if defined(CONFIG_NET_SENDTO_TIMEOUT) || defined(NEED_IPDOMAIN_SUPPORT)
  FAR struct socket *st_sock;         /* Points to the parent socket structure */
#endif
#ifdef CONFIG_NET_SENDTO_TIMEOUT
  uint32_t st_time;                   /* Last send time for determining timeout */
#endif
  FAR struct devif_callback_s *st_cb; /* Reference to callback instance */
  sem_t st_sem;                       /* Semaphore signals sendto completion */
  uint16_t st_buflen;                 /* Length of send buffer (error if <0) */
  const char *st_buffer;              /* Pointer to send buffer */
  int st_sndlen;                      /* Result of the send (length sent or negated errno) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: send_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Parameters:
 *   pstate - sendto state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SENDTO_TIMEOUT
static inline int send_timeout(FAR struct sendto_s *pstate)
{
  FAR struct socket *psock;

  /* Check for a timeout configured via setsockopts(SO_SNDTIMEO).
   * If none... we well let the send wait forever.
   */

  psock = pstate->st_sock;
  if (psock && psock->s_sndtimeo != 0)
    {
      /* Check if the configured timeout has elapsed */

      return net_timeo(pstate->st_time, psock->s_sndtimeo);
    }

  /* No timeout */

  return FALSE;
}
#endif /* CONFIG_NET_SENDTO_TIMEOUT */

/****************************************************************************
 * Function: sendto_ipselect
 *
 * Description:
 *   If both IPv4 and IPv6 support are enabled, then we will need to select
 *   which one to use when generating the outgoing packet.  If only one
 *   domain is selected, then the setup is already in place and we need do
 *   nothing.
 *
 * Parameters:
 *   dev    - The structure of the network driver that caused the interrupt
 *   pstate - sendto state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef NEED_IPDOMAIN_SUPPORT
static inline void sendto_ipselect(FAR struct net_driver_s *dev,
                                   FAR struct sendto_s *pstate)
{
  FAR struct socket *psock = pstate->st_sock;
  DEBUGASSERT(psock);

  /* Which domain the the socket support */

  if (psock->s_domain == PF_INET)
    {
      /* Select the IPv4 domain */

      udp_ipv4_select(dev);
    }
  else /* if (psock->s_domain == PF_INET6) */
    {
      /* Select the IPv6 domain */

      DEBUGASSERT(psock->s_domain == PF_INET6);
      udp_ipv4_select(dev);
    }
}
#endif

/****************************************************************************
 * Function: sendto_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   send operation when polled by the lower, device interfacing layer.
 *
 * Parameters:
 *   dev        The structure of the network driver that caused the interrupt
 *   conn       An instance of the UDP connection structure cast to void *
 *   pvpriv     An instance of struct sendto_s cast to void*
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static uint16_t sendto_interrupt(FAR struct net_driver_s *dev, FAR void *conn,
                                 FAR void *pvpriv, uint16_t flags)
{
  FAR struct sendto_s *pstate = (FAR struct sendto_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);
  if (pstate)
    {
      /* Check if the outgoing packet is available.  It may have been claimed
       * by a sendto interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data.  In these cases
       * we will just have to wait for the next polling cycle.
       */

      if (dev->d_sndlen > 0 || (flags & UDP_NEWDATA) != 0)
        {
           /* Another thread has beat us sending data or the buffer is busy,
            * Check for a timeout.  If not timed out, wait for the next
            * polling cycle and check again.
            */

#ifdef CONFIG_NET_SENDTO_TIMEOUT
          if (send_timeout(pstate))
            {
              /* Yes.. report the timeout */

              nlldbg("SEND timeout\n");
              pstate->st_sndlen = -ETIMEDOUT;
            }
          else
#endif /* CONFIG_NET_SENDTO_TIMEOUT */
            {
               /* No timeout.  Just wait for the next polling cycle */

               return flags;
            }
        }

      /* It looks like we are good to send the data */

      else
        {
#ifdef NEED_IPDOMAIN_SUPPORT
          /* If both IPv4 and IPv6 support are enabled, then we will need to
           * select which one to use when generating the outgoing packet.
           * If only one domain is selected, then the setup is already in
           * place and we need do nothing.
           */

          sendto_ipselect(dev, pstate);
#endif

          /* Copy the user data into d_appdata and send it */

          devif_send(dev, pstate->st_buffer, pstate->st_buflen);
          pstate->st_sndlen = pstate->st_buflen;
        }

      /* Don't allow any further call backs. */

      pstate->st_cb->flags   = 0;
      pstate->st_cb->priv    = NULL;
      pstate->st_cb->event   = NULL;

      /* Wake up the waiting thread */

      sem_post(&pstate->st_sem);
    }

  return flags;
}
#endif

/****************************************************************************
 * Function: sendto_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are have data ready to
 *   be send (UDP)
 *
 * Parameters:
 *   psock - Socket state structure
 *   conn  - The UDP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static inline void sendto_txnotify(FAR struct socket *psock,
                                   FAR struct udp_conn_s *conn)
{
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  /* If both IPv4 and IPv6 support are enabled, then we will need to select
   * the device driver using the appropriate IP domain.
   */

  if (psock->s_domain == PF_INET)
#endif
    {
      /* Notify the device driver that send data is available */

#ifdef CONFIG_NET_MULTILINK
      netdev_ipv4_txnotify(conn->u.ipv4.laddr, conn->u.ipv4.raddr);
#else
      netdev_ipv4_txnotify(conn->u.ipv4.raddr);
#endif
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else /* if (psock->s_domain == PF_INET6) */
#endif /* CONFIG_NET_IPv4 */
    {
      /* Notify the device driver that send data is available */

      DEBUGASSERT(psock->s_domain == PF_INET6);
#ifdef CONFIG_NET_MULTILINK
      netdev_ipv6_txnotify(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
#else
      netdev_ipv6_txnotify(conn->u.ipv6.raddr);
#endif
    }
#endif /* CONFIG_NET_IPv6 */
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_sendto(FAR struct socket *psock, FAR const void *buf,
                     size_t len, int flags, FAR const struct sockaddr *to,
                     socklen_t tolen)
{
#ifdef CONFIG_NET_UDP
  FAR struct udp_conn_s *conn;
#ifdef CONFIG_NET_ARP_SEND
  FAR const struct sockaddr_in *into;
#endif
  struct sendto_s state;
  net_lock_t save;
  int ret;
#endif
  int err;

  /* If to is NULL or tolen is zero, then this function is same as send (for
   * connected socket types)
   */

  if (!to || !tolen)
    {
#ifdef CONFIG_NET_TCP
      return psock_send(psock, buf, len, flags);
#else
      ndbg("ERROR: No to address\n");
      err = EINVAL;
      goto errout;
#endif
    }

  /* Verify that a valid address has been provided */

#ifdef CONFIG_NET_IPv6
  if (to->sa_family != AF_INET6 || tolen < sizeof(struct sockaddr_in6))
#else
  if (to->sa_family != AF_INET || tolen < sizeof(struct sockaddr_in))
#endif
  {
      ndbg("ERROR: Invalid address\n");
      err = EBADF;
      goto errout;
  }

  /* Verify that the psock corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      ndbg("ERROR: Invalid socket\n");
      err = EBADF;
      goto errout;
    }

  /* If this is a connected socket, then return EISCONN */

  if (psock->s_type != SOCK_DGRAM)
    {
      ndbg("ERROR: Connected socket\n");
      err = EISCONN;
      goto errout;
    }

  /* Make sure that the IP address mapping is in the ARP table */

#ifdef CONFIG_NET_ARP_SEND
  into = (FAR const struct sockaddr_in *)to;
  ret = arp_send(into->sin_addr.s_addr);
  if (ret < 0)
    {
      ndbg("ERROR: Not reachable\n");
      err = ENETUNREACH;
      goto errout;
    }
#endif

  /* Perform the UDP sendto operation */

#ifdef CONFIG_NET_UDP
  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = net_lock();
  memset(&state, 0, sizeof(struct sendto_s));
  sem_init(&state.st_sem, 0, 0);
  state.st_buflen = len;
  state.st_buffer = buf;

#if defined(CONFIG_NET_SENDTO_TIMEOUT) || defined(NEED_IPDOMAIN_SUPPORT)
  /* Save the reference to the socket structure if it will be needed for
   * asynchronous processing.
   */

  state.st_sock = psock;
#endif

#ifdef CONFIG_NET_SENDTO_TIMEOUT
  /* Set the initial time for calculating timeouts */

  state.st_time = clock_systimer();
#endif

  /* Setup the UDP socket */

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  ret = udp_connect(conn, to);
  if (ret < 0)
    {
      net_unlock(save);
      err = -ret;
      goto errout;
    }

  /* Set up the callback in the connection */

  state.st_cb = udp_callback_alloc(conn);
  if (state.st_cb)
    {
      state.st_cb->flags   = UDP_POLL;
      state.st_cb->priv    = (void*)&state;
      state.st_cb->event   = sendto_interrupt;

      /* Notify the device driver of the availability of TX data */

      sendto_txnotify(psock, conn);

      /* Wait for either the receive to complete or for an error/timeout to occur.
       * NOTES:  (1) net_lockedwait will also terminate if a signal is received, (2)
       * interrupts may be disabled!  They will be re-enabled while the task sleeps
       * and automatically re-enabled when the task restarts.
       */

      net_lockedwait(&state.st_sem);

      /* Make sure that no further interrupts are processed */

      udp_callback_free(conn, state.st_cb);
    }

  net_unlock(save);
  sem_destroy(&state.st_sem);

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for errors */

  if (state.st_sndlen < 0)
    {
      err = -state.st_sndlen;
      goto errout;
    }

  /* Success */

  return state.st_sndlen;
#else
  err = ENOSYS;
#endif /* CONFIG_NET_UDP */

errout:
  set_errno(err);
  return ERROR;
}

/****************************************************************************
 * Function: sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t sendto(int sockfd, FAR const void *buf, size_t len, int flags,
               FAR const struct sockaddr *to, socklen_t tolen)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* And let psock_sendto do all of the work */

  return psock_sendto(psock, buf, len, flags, to, tolen);
}

#endif /* CONFIG_NET */
