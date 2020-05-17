/****************************************************************************
 * net/udp/udp_recvfrom.c
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "udp/udp.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF    ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF    ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define UDPIPv4BUF ((struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define UDPIPv6BUF ((struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct udp_recvfrom_s
{
  FAR struct socket       *ir_sock;      /* The parent socket structure */
  FAR struct devif_callback_s *ir_cb;    /* Reference to callback instance */
  sem_t                    ir_sem;       /* Semaphore signals recv completion */
  size_t                   ir_buflen;    /* Length of receive buffer */
  uint8_t                 *ir_buffer;    /* Pointer to receive buffer */
  FAR struct sockaddr     *ir_from;      /* Address of sender */
  FAR socklen_t           *ir_fromlen;   /* Number of bytes allocated for address of sender */
  ssize_t                  ir_recvlen;   /* The received length */
  int                      ir_result;    /* Success:OK, failure:negated errno */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_update_recvlen
 *
 * Description:
 *   Update information about space available for new data and update size
 *   of data in buffer,  This logic accounts for the case where
 *   udp_readahead() sets state.ir_recvlen == -1 .
 *
 * Input Parameters:
 *   pstate   recvfrom state structure
 *   recvlen  size of new data appended to buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void udp_update_recvlen(FAR struct udp_recvfrom_s *pstate,
                                      size_t recvlen)
{
  if (pstate->ir_recvlen < 0)
    {
      pstate->ir_recvlen = 0;
    }

  pstate->ir_recvlen += recvlen;
  pstate->ir_buffer  += recvlen;
  pstate->ir_buflen  -= recvlen;
}

/****************************************************************************
 * Name: udp_recvfrom_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event.
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   The number of bytes taken from the packet.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static size_t udp_recvfrom_newdata(FAR struct net_driver_s *dev,
                                   FAR struct udp_recvfrom_s *pstate)
{
  size_t recvlen;

  /* Get the length of the data to return */

  if (dev->d_len > pstate->ir_buflen)
    {
      recvlen = pstate->ir_buflen;
    }
  else
    {
      recvlen = dev->d_len;
    }

  /* Copy the new appdata into the user buffer */

  memcpy(pstate->ir_buffer, dev->d_appdata, recvlen);
  ninfo("Received %d bytes (of %d)\n", (int)recvlen, (int)dev->d_len);

  /* Update the accumulated size of the data read */

  udp_update_recvlen(pstate, recvlen);

  return recvlen;
}

/****************************************************************************
 * Name: udp_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static inline void udp_newdata(FAR struct net_driver_s *dev,
                               FAR struct udp_recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  udp_recvfrom_newdata(dev, pstate);

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}

static inline void udp_readahead(struct udp_recvfrom_s *pstate)
{
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)
                                pstate->ir_sock->s_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any UDP datagram already buffered in a read-ahead
   * buffer.
   */

  pstate->ir_recvlen = -1;

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL)
    {
      FAR struct iob_s *tmp;
      uint8_t src_addr_size;

      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(&src_addr_size, iob, sizeof(uint8_t), 0);
      if (recvlen != sizeof(uint8_t))
        {
          goto out;
        }

      if (0
#ifdef CONFIG_NET_IPv6
          || src_addr_size == sizeof(struct sockaddr_in6)
#endif
#ifdef CONFIG_NET_IPv4
          || src_addr_size == sizeof(struct sockaddr_in)
#endif
        )
        {
          if (pstate->ir_from)
            {
              socklen_t len = *pstate->ir_fromlen;
              len = (socklen_t)
                src_addr_size > len ? len : (socklen_t)src_addr_size;

              recvlen = iob_copyout((FAR uint8_t *)pstate->ir_from, iob,
                                    len, sizeof(uint8_t));
              if (recvlen != len)
                {
                  goto out;
                }
            }
        }

      if (pstate->ir_buflen > 0)
        {
          recvlen = iob_copyout(pstate->ir_buffer, iob, pstate->ir_buflen,
                                src_addr_size + sizeof(uint8_t));

          ninfo("Received %d bytes (of %d)\n", recvlen, iob->io_pktlen);

          /* Update the accumulated size of the data read */

          pstate->ir_recvlen  = recvlen;
          pstate->ir_buffer  += recvlen;
          pstate->ir_buflen  -= recvlen;
        }
      else
        {
          pstate->ir_recvlen = 0;
        }

out:
      /* Remove the I/O buffer chain from the head of the read-ahead
       * buffer queue.
       */

      tmp = iob_remove_queue(&conn->readahead);
      DEBUGASSERT(tmp == iob);
      UNUSED(tmp);

      /* And free the I/O buffer chain */

      iob_free_chain(iob, IOBUSER_NET_UDP_READAHEAD);
    }
}

/****************************************************************************
 * Name: udp_sender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Input Parameters:
 *   dev    - The device driver data structure
 *   pstate - the recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static inline void udp_sender(FAR struct net_driver_s *dev,
                              FAR struct udp_recvfrom_s *pstate)
{
  /* Get the family from the packet type, IP address from the IP header, and
   * the port number from the UDP header.
   */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct sockaddr_in6 *infrom =
        (FAR struct sockaddr_in6 *)pstate->ir_from;
      FAR socklen_t *fromlen = pstate->ir_fromlen;

      if (infrom)
        {
          FAR struct udp_hdr_s *udp   = UDPIPv6BUF;
          FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

          infrom->sin6_family = AF_INET6;
          infrom->sin6_port   = udp->srcport;
          *fromlen = sizeof(struct sockaddr_in6);

          net_ipv6addr_copy(infrom->sin6_addr.s6_addr, ipv6->srcipaddr);
        }
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      FAR struct sockaddr_in *infrom  =
        (FAR struct sockaddr_in *)pstate->ir_from;

      if (infrom)
        {
#ifdef CONFIG_NET_IPv6
          FAR struct udp_conn_s *conn =
            (FAR struct udp_conn_s *)pstate->ir_sock->s_conn;

          /* Hybrid dual-stack IPv6/IPv4 implementations recognize a special
           * class of addresses, the IPv4-mapped IPv6 addresses.
           */

          if (conn->domain == PF_INET6)
            {
              FAR struct sockaddr_in6 *infrom6 =
                (FAR struct sockaddr_in6 *)infrom;
              FAR socklen_t *fromlen = pstate->ir_fromlen;
              FAR struct udp_hdr_s *udp   = UDPIPv6BUF;
              FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
              in_addr_t ipv4addr;

              /* Encode the IPv4 address as an IPv4-mapped IPv6 address */

              infrom6->sin6_family = AF_INET6;
              infrom6->sin6_port = udp->srcport;
              *fromlen = sizeof(struct sockaddr_in6);

              ipv4addr = net_ip4addr_conv32(ipv6->srcipaddr);
              ip6_map_ipv4addr(ipv4addr, infrom6->sin6_addr.s6_addr16);
            }
          else
#endif
            {
              FAR struct udp_hdr_s *udp   = UDPIPv4BUF;
              FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

              infrom->sin_family = AF_INET;
              infrom->sin_port   = udp->srcport;

              net_ipv4addr_copy(infrom->sin_addr.s_addr,
                                net_ip4addr_conv32(ipv4->srcipaddr));
              memset(infrom->sin_zero, 0, sizeof(infrom->sin_zero));
            }
        }
    }
#endif /* CONFIG_NET_IPv4 */
}

/****************************************************************************
 * Name: udp_terminate
 *
 * Description:
 *   Terminate the UDP transfer.
 *
 * Input Parameters:
 *   pstate - The recvfrom state structure
 *   result - The result of the operation
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void udp_terminate(FAR struct udp_recvfrom_s *pstate, int result)
{
  /* Don't allow any further UDP call backs. */

  pstate->ir_cb->flags   = 0;
  pstate->ir_cb->priv    = NULL;
  pstate->ir_cb->event   = NULL;

  /* Save the result of the transfer */

  pstate->ir_result      = result;

  /* Wake up the waiting thread, returning the number of bytes
   * actually read.
   */

  nxsem_post(&pstate->ir_sem);
}

/****************************************************************************
 * Name: udp_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   UDP receive operation via by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event.
 *   pvconn   The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t udp_eventhandler(FAR struct net_driver_s *dev,
                                 FAR void *pvconn, FAR void *pvpriv,
                                 uint16_t flags)
{
  FAR struct udp_recvfrom_s *pstate = (FAR struct udp_recvfrom_s *)pvpriv;

  ninfo("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If the network device has gone down, then we will have terminate
       * the wait now with an error.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* Terminate the transfer with an error. */

          nerr("ERROR: Network is down\n");
          udp_terminate(pstate, -ENETUNREACH);
        }

      /* If new data is available, then complete the read action. */

      else if ((flags & UDP_NEWDATA) != 0)
        {
          /* Copy the data from the packet */

          udp_newdata(dev, pstate);

          /* We are finished. */

          ninfo("UDP done\n");

          /* Save the sender's address in the caller's 'from' location */

          udp_sender(dev, pstate);

          /* Don't allow any further UDP call backs. */

          udp_terminate(pstate, OK);

          /* Indicate that the data has been consumed */

          flags &= ~UDP_NEWDATA;
        }
    }

  return flags;
}

/****************************************************************************
 * Name: udp_recvfrom_initialize
 *
 * Description:
 *   Initialize the state structure
 *
 * Input Parameters:
 *   psock    Pointer to the socket structure for the socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void udp_recvfrom_initialize(FAR struct socket *psock, FAR void *buf,
                                    size_t len, FAR struct sockaddr *infrom,
                                    FAR socklen_t *fromlen,
                                    FAR struct udp_recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct udp_recvfrom_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&pstate->ir_sem, 0, 0); /* Doesn't really fail */
  nxsem_set_protocol(&pstate->ir_sem, SEM_PRIO_NONE);

  pstate->ir_buflen    = len;
  pstate->ir_buffer    = buf;
  pstate->ir_from      = infrom;
  pstate->ir_fromlen   = fromlen;

  /* Set up the start time for the timeout */

  pstate->ir_sock      = psock;
}

/* The only un-initialization that has to be performed is destroying the
 * semaphore.
 */

#define udp_recvfrom_uninitialize(s) nxsem_destroy(&(s)->ir_sem)

/****************************************************************************
 * Name: udp_recvfrom_result
 *
 * Description:
 *   Evaluate the result of the recv operations
 *
 * Input Parameters:
 *   result   The result of the net_timedwait operation (may indicate EINTR)
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   The result of the recv operation with errno set appropriately
 *
 * Assumptions:
 *
 ****************************************************************************/

static ssize_t udp_recvfrom_result(int result, struct udp_recvfrom_s *pstate)
{
  /* Check for a error/timeout detected by the event handler.  Errors are
   * signaled by negative errno values for the rcv length
   */

  if (pstate->ir_result < 0)
    {
      /* This might return EAGAIN on a timeout or ENOTCONN on loss of
       * connection (TCP only)
       */

      return pstate->ir_result;
    }

  /* If net_timedwait failed, then we were probably reawakened by a signal.
   * In this case, net_timedwait will have returned negated errno
   * appropriately.
   */

  if (result < 0)
    {
      return result;
    }

  return pstate->ir_recvlen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_udp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a UDP SOCK_DGRAM
 *
 * Input Parameters:
 *   psock  Pointer to the socket structure for the SOCK_DRAM socket
 *   buf    Buffer to receive data
 *   len    Length of buffer
 *   from   INET address of source (may be NULL)
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_udp_recvfrom(FAR struct socket *psock, FAR void *buf,
                           size_t len, int flags, FAR struct sockaddr *from,
                           FAR socklen_t *fromlen)
{
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)psock->s_conn;
  FAR struct net_driver_s *dev;
  struct udp_recvfrom_s state;
  int ret;

  /* Perform the UDP recvfrom() operation */

  /* Initialize the state structure.  This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  net_lock();
  udp_recvfrom_initialize(psock, buf, len, from, fromlen, &state);

  /* Copy the read-ahead data from the packet */

  udp_readahead(&state);

  /* The default return value is the number of bytes that we just copied
   * into the user buffer.  We will return this if the socket has become
   * disconnected or if the user request was completely satisfied with
   * data from the readahead buffers.
   */

  ret = state.ir_recvlen;

  /* Handle non-blocking UDP sockets */

  if (_SS_ISNONBLOCK(psock->s_flags) || (flags & MSG_DONTWAIT) != 0)
    {
      /* Return the number of bytes read from the read-ahead buffer if
       * something was received (already in 'ret'); EAGAIN if not.
       */

      if (ret < 0)
        {
          /* Nothing was received */

          ret = -EAGAIN;
        }
    }

  /* It is okay to block if we need to.  If there is space to receive
   * anything more, then we will wait to receive the data. Otherwise
   * return the number of bytes read from the read-ahead buffer
   * (already in 'ret').
   *
   * NOTE: that udp_readahead() may set state.ir_recvlen == -1.
   */

  else if (state.ir_recvlen <= 0)
    {
      /* Get the device that will handle the packet transfers.  This may be
       * NULL if the UDP socket is bound to INADDR_ANY.  In that case, no
       * NETDEV_DOWN notifications will be received.
       */

      dev = udp_find_laddr_device(conn);

      /* Set up the callback in the connection */

      state.ir_cb = udp_callback_alloc(dev, conn);
      if (state.ir_cb)
        {
          /* Set up the callback in the connection */

          state.ir_cb->flags   = (UDP_NEWDATA | NETDEV_DOWN);
          state.ir_cb->priv    = (FAR void *)&state;
          state.ir_cb->event   = udp_eventhandler;

          /* Wait for either the receive to complete or for an error/timeout
           * to occur.  net_timedwait will also terminate if a signal is
           * received.
           */

          ret = net_timedwait(&state.ir_sem, _SO_TIMEOUT(psock->s_rcvtimeo));
          if (ret == -ETIMEDOUT)
            {
              ret = -EAGAIN;
            }

          /* Make sure that no further events are processed */

          udp_callback_free(dev, conn, state.ir_cb);
          ret = udp_recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }
    }

  net_unlock();
  udp_recvfrom_uninitialize(&state);
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
