/****************************************************************************
 * net/udp/udp_sendto.c
 *
 *   Copyright (C) 2007-2009, 2011-2015 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET_UDP

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"
#include "socket/socket.h"
#include "udp/udp.h"

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

static uint16_t sendto_interrupt(FAR struct net_driver_s *dev, FAR void *conn,
                                 FAR void *pvpriv, uint16_t flags)
{
  FAR struct sendto_s *pstate = (FAR struct sendto_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);
  if (pstate)
    {
      /* If the network device has gone down, then we will have terminate
       * the wait now with an error.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* Terminate the transfer with an error. */

          nlldbg("ERROR: Network is down\n");
          pstate->st_sndlen = -ENETUNREACH;
        }

      /* Check if the outgoing packet is available.  It may have been claimed
       * by a sendto interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data.  In these cases
       * we will just have to wait for the next polling cycle.
       */

      else if (dev->d_sndlen > 0 || (flags & UDP_NEWDATA) != 0)
        {
           /* Another thread has beat us sending data or the buffer is busy,
            * Check for a timeout.  If not timed out, wait for the next
            * polling cycle and check again.
            */

#ifdef CONFIG_NET_SENDTO_TIMEOUT
          if (send_timeout(pstate))
            {
              /* Yes.. report the timeout */

              nlldbg("ERROR: SEND timeout\n");
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_udp_sendto
 *
 * Description:
 *   This function implements the UDP-specific logic of the standard
 *   sendto() socket operation.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 *   NOTE: All input parameters were verified by sendto() before this
 *   function was called.
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See the description in
 *   net/socket/sendto.c for the list of appropriate return value.
 *
 ****************************************************************************/

ssize_t psock_udp_sendto(FAR struct socket *psock, FAR const void *buf,
                         size_t len, int flags, FAR const struct sockaddr *to,
                         socklen_t tolen)
{
  FAR struct udp_conn_s *conn;
  FAR struct net_driver_s *dev;
  struct sendto_s state;
  net_lock_t save;
  int ret;

#if defined(CONFIG_NET_ARP_SEND) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
#ifdef CONFIG_NET_ARP_SEND
#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  if (psock->s_domain == PF_INET)
#endif
    {
      FAR const struct sockaddr_in *into;

      /* Make sure that the IP address mapping is in the ARP table */

      into = (FAR const struct sockaddr_in *)to;
      ret = arp_send(into->sin_addr.s_addr);
    }
#endif /* CONFIG_NET_ARP_SEND */

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
#ifdef CONFIG_NET_ARP_SEND
  else
#endif
    {
      FAR const struct sockaddr_in6 *into;

      /* Make sure that the IP address mapping is in the Neighbor Table */

      into = (FAR const struct sockaddr_in6 *)to;
      ret = icmpv6_neighbor(into->sin6_addr.s6_addr16);
    }
#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Did we successfully get the address mapping? */

  if (ret < 0)
    {
      ndbg("ERROR: Peer not reachable\n");
      return -ENETUNREACH;
    }
#endif /* CONFIG_NET_ARP_SEND || CONFIG_NET_ICMPv6_NEIGHBOR */

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

  /* Setup the UDP socket.  udp_connect will set the remote address in the
   * connection structure.
   */

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn);

  ret = udp_connect(conn, to);
  if (ret < 0)
    {
      ndbg("ERROR: udp_connect failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Get the device that will handle the remote packet transfers.  This
   * should never be NULL.
   */

  dev = udp_find_raddr_device(conn);
  if (dev == NULL)
    {
      ndbg("ERROR: udp_find_raddr_device failed\n");
      ret = -ENETUNREACH;
      goto errout_with_lock;
   }

  /* Set up the callback in the connection */

  state.st_cb = udp_callback_alloc(dev, conn);
  if (state.st_cb)
    {
      state.st_cb->flags   = (UDP_POLL | NETDEV_DOWN);
      state.st_cb->priv    = (void*)&state;
      state.st_cb->event   = sendto_interrupt;

      /* Notify the device driver of the availability of TX data */

      netdev_txnotify_dev(dev);

      /* Wait for either the receive to complete or for an error/timeout to occur.
       * NOTES:  (1) net_lockedwait will also terminate if a signal is received, (2)
       * interrupts may be disabled!  They will be re-enabled while the task sleeps
       * and automatically re-enabled when the task restarts.
       */

      net_lockedwait(&state.st_sem);

      /* Make sure that no further interrupts are processed */

      udp_callback_free(dev, conn, state.st_cb);
    }

  /* The result of the sendto operation is the number of bytes transferred */

  ret = state.st_sndlen;

errout_with_lock:
  /* Release the semaphore */

  sem_destroy(&state.st_sem);

  /* Set the socket state back to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Unlock the network and return the result of the sendto() operation */

  net_unlock(save);
  return ret;
}

#endif /* CONFIG_NET_UDP */
