/****************************************************************************
 * net/icmpv6/icmpv6_recvmsg.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/icmpv6.h>

#include "devif/devif.h"
#include "socket/socket.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_SOCKET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv6_BUF \
  ((FAR struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPv6_BUF \
  ((FAR struct icmpv6_echo_reply_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])
#define ICMPv6_SIZE \
  ((dev)->d_len - IPv6_HDRLEN)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct icmpv6_recvfrom_s
{
  FAR struct devif_callback_s *recv_cb; /* Reference to callback instance */
  FAR struct socket *recv_sock;         /* IPPROTO_ICMP6 socket structure */
  sem_t recv_sem;                       /* Use to manage the wait for the
                                         * response */
  struct in6_addr recv_from;            /* The peer we received the request
                                         * from */
  FAR uint8_t *recv_buf;                /* Location to return the response */
  uint16_t recv_buflen;                 /* Size of the response */
  int16_t recv_result;                  /* >=0: receive size on success;
                                         * <0: negated errno on fail */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: recvfrom_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   ECHO request and/or ECHO reply actions when polled by the lower, device
 *   interfacing layer.
 *
 * Input Parameters:
 *   dev        The structure of the network driver that generated the
 *              event
 *   conn       The received packet, cast to (void *)
 *   pvpriv     An instance of struct icmpv6_recvfrom_s cast to void*
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t recvfrom_eventhandler(FAR struct net_driver_s *dev,
                                  FAR void *pvconn,
                                  FAR void *pvpriv, uint16_t flags)
{
  FAR struct icmpv6_recvfrom_s *pstate = (struct icmpv6_recvfrom_s *)pvpriv;
  FAR struct socket *psock;
  FAR struct icmpv6_conn_s *conn;
  FAR struct ipv6_hdr_s *ipv6;
  FAR struct icmpv6_echo_reply_s *icmpv6;

  ninfo("flags: %04x\n", flags);

  if (pstate != NULL)
    {
      /* Check if the network is still up */

      if ((flags & NETDEV_DOWN) != 0)
        {
          nerr("ERROR: Interface is down\n");
          pstate->recv_result = -ENETUNREACH;
          goto end_wait;
        }

      /* Is this a response on the same device that we sent the request out
       * on?
       */

      psock = pstate->recv_sock;
      DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
      conn  = psock->s_conn;
      if (dev != conn->dev)
        {
          ninfo("Wrong device\n");
          return flags;
        }

      /* Check if we have just received a ICMPv6 ECHO reply. */

      if ((flags & ICMPv6_NEWDATA) != 0)    /* No incoming data */
        {
          unsigned int recvsize;

          /* Check if it is for us.
           * REVISIT:  What if there are IPv6 extension headers present?
           */

          icmpv6 = ICMPv6_BUF;
          if (conn->id != icmpv6->id)
            {
              ninfo("Wrong ID: %u vs %u\n", icmpv6->id, conn->id);
              return flags;
            }

          ninfo("Received ICMPv6 reply\n");

          /* What should we do if the received reply is larger that the
           * buffer that the caller of sendto provided?  Truncate?  Error
           * out?
           */

          recvsize = ICMPv6_SIZE;
          if (recvsize > pstate->recv_buflen)
            {
              recvsize = pstate->recv_buflen;
            }

          /* Copy the ICMPv6 ECHO reply to the user provided buffer
           * REVISIT:  What if there are IPv6 extension headers present?
           */

          memcpy(pstate->recv_buf, ICMPv6_BUF, recvsize);

          /* Return the size of the returned data */

          DEBUGASSERT(recvsize <= INT16_MAX);
          pstate->recv_result = recvsize;

          /* Return the IPv6 address of the sender from the IPv6 header */

          ipv6 = IPv6_BUF;
          net_ipv6addr_hdrcopy(&pstate->recv_from, ipv6->srcipaddr);

          /* Decrement the count of outstanding requests.  I suppose this
           * could have already been decremented of there were multiple
           * threads calling sendto() or recvfrom().  If there finds, we
           * may have to beef up the design.
           */

          DEBUGASSERT(conn->nreqs > 0);
          conn->nreqs--;

          /* Indicate that the data has been consumed */

          flags     &= ~ICMPv6_NEWDATA;
          dev->d_len = 0;
          goto end_wait;
        }

      /* Continue waiting */
    }

  return flags;

end_wait:
  ninfo("Resuming\n");

  /* Do not allow any further callbacks */

  pstate->recv_cb->flags   = 0;
  pstate->recv_cb->priv    = NULL;
  pstate->recv_cb->event   = NULL;

  /* Wake up the waiting thread */

  nxsem_post(&pstate->recv_sem);
  return flags;
}

/****************************************************************************
 * Name: icmpv6_readahead
 *
 * Description:
 *   Copy the buffered read-ahead data to the user buffer.
 *
 * Input Parameters:
 *   conn  - IPPROTO_ICMP6 socket connection structure containing the read-
 *           ahead data.
 *   dev      The structure of the network driver that generated the event.
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   Number of bytes copied to the user buffer
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static inline ssize_t icmpv6_readahead(FAR struct icmpv6_conn_s *conn,
                                     FAR void *buf, size_t buflen,
                                     FAR struct sockaddr_in6 *from,
                                     FAR socklen_t *fromlen)
{
  FAR struct sockaddr_in6 bitbucket;
  FAR struct iob_s *iob;
  ssize_t ret = -ENODATA;
  int recvlen;

  /* Check there is any ICMPv6 replies already buffered in a read-ahead
   * buffer.
   */

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL)
    {
      FAR struct iob_s *tmp;
      uint16_t offset;
      uint8_t addrsize;

      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      /* First get the size of the address */

      recvlen = iob_copyout(&addrsize, iob, sizeof(uint8_t), 0);
      if (recvlen != sizeof(uint8_t))
        {
          ret = -EIO;
          goto out;
        }

      offset = sizeof(uint8_t);

      if (addrsize > sizeof(struct sockaddr_in6))
        {
          ret = -EINVAL;
          goto out;
        }

      /* Then get address */

      if (from == NULL)
        {
          from = &bitbucket;
        }

      recvlen = iob_copyout((FAR uint8_t *)from, iob, addrsize, offset);
      if (recvlen != addrsize)
        {
          ret = -EIO;
          goto out;
        }

      if (fromlen != NULL)
        {
          *fromlen = addrsize;
        }

      offset += addrsize;

      /* And finally, get the buffered data */

      ret = (ssize_t)iob_copyout(buf, iob, buflen, offset);

      ninfo("Received %ld bytes (of %u)\n", (long)ret, iob->io_pktlen);

out:
      /* Remove the I/O buffer chain from the head of the read-ahead
       * buffer queue.
       */

      tmp = iob_remove_queue(&conn->readahead);
      DEBUGASSERT(tmp == iob);
      UNUSED(tmp);

      /* And free the I/O buffer chain */

      iob_free_chain(iob, IOBUSER_NET_SOCK_ICMPv6);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_recvmsg
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   data gram socket with the IPPROTO_ICMP6 protocol.  icmpv6_recvmsg()
 *   receives ICMPv6 ECHO replies for the a socket.
 *
 *   If msg_name is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument 'msg_namelen' is
 *   initialized to the size of the buffer associated with msg_name, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received. If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t icmpv6_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                       int flags)
{
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR struct sockaddr *from = msg->msg_name;
  FAR socklen_t *fromlen = &msg->msg_namelen;
  FAR struct sockaddr_in6 *inaddr;
  FAR struct icmpv6_conn_s *conn;
  FAR struct net_driver_s *dev;
  struct icmpv6_recvfrom_s state;
  ssize_t ret;

  /* Some sanity checks */

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && buf != NULL);

  if (len < ICMPv6_HDRLEN)
    {
      return -EINVAL;
    }

  /* If a 'from' address has been provided, verify that it is large
   * enough to hold the AF_INET address.
   */

  if (from != NULL)
    {
      if (fromlen == NULL && *fromlen < sizeof(struct sockaddr_in6))
        {
          return -EINVAL;
        }
    }

  net_lock();

  /* We cannot receive a response from a device until a request has been
   * sent to the devivce.
   */

  conn = psock->s_conn;
  if (conn->nreqs < 1)
    {
      ret = -EPROTO;
      goto errout;
    }

  /* Get the device that was used to send the ICMPv6 request. */

  dev = conn->dev;
  DEBUGASSERT(dev != NULL);
  if (dev == NULL)
    {
      ret = -EPROTO;
      goto errout;
    }

  /* Check if there is buffered read-ahead data for this socket.  We may have
   * already received the response to previous command.
   */

  if (!IOB_QEMPTY(&conn->readahead))
    {
      ret = icmpv6_readahead(conn, buf, len,
                             (FAR struct sockaddr_in6 *)from, fromlen);
    }
  else if (_SS_ISNONBLOCK(conn->sconn.s_flags) ||
           (flags & MSG_DONTWAIT) != 0)
    {
      /* Handle non-blocking ICMP sockets */

      ret = -EAGAIN;
    }
  else
    {
      /* Initialize the state structure */

      memset(&state, 0, sizeof(struct icmpv6_recvfrom_s));

      /* This semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
       */

      nxsem_init(&state.recv_sem, 0, 0);
      nxsem_set_protocol(&state.recv_sem, SEM_PRIO_NONE);

      state.recv_sock   = psock;    /* The IPPROTO_ICMP6 socket instance */
      state.recv_result = -ENOMEM;  /* Assume allocation failure */
      state.recv_buf    = buf;      /* Location to return the response */
      state.recv_buflen = len;      /* Size of the response */

      /* Set up the callback */

      state.recv_cb = icmpv6_callback_alloc(dev, conn);
      if (state.recv_cb)
        {
          state.recv_cb->flags = (ICMPv6_NEWDATA | NETDEV_DOWN);
          state.recv_cb->priv  = (FAR void *)&state;
          state.recv_cb->event = recvfrom_eventhandler;

          /* Wait for either the response to be received or for timeout to
           * occur. (1) net_timedwait will also terminate if a signal is
           * received, (2) interrupts may be disabled!  They will be
           * re-enabled while the task sleeps and automatically re-enabled
           * when the task restarts.
           */

          ret = net_timedwait(&state.recv_sem,
                              _SO_TIMEOUT(conn->sconn.s_rcvtimeo));
          if (ret < 0)
            {
              state.recv_result = ret;
            }

          icmpv6_callback_free(dev, conn, state.recv_cb);
        }

      /* Return the negated error number in the event of a failure, or the
       * number of bytes received on success.
       */

      if (state.recv_result < 0)
        {
          nerr("ERROR: Return error=%d\n", state.recv_result);
          ret = state.recv_result;
          goto errout;
        }

      if (from != NULL)
        {
          inaddr              = (FAR struct sockaddr_in6 *)from;
          inaddr->sin6_family = AF_INET6;
          inaddr->sin6_port   = 0;

          net_ipv6addr_copy(inaddr->sin6_addr.s6_addr16,
                            state.recv_from.s6_addr16);
        }

      ret = state.recv_result;

      /* If there a no further outstanding requests,
       * make sure that the request struct is left pristine.
       */

errout:
      if (conn->nreqs < 1)
        {
          conn->id    = 0;
          conn->nreqs = 0;
          conn->dev   = NULL;

          iob_free_queue(&conn->readahead, IOBUSER_NET_SOCK_ICMPv6);
        }
    }

  net_unlock();

  return ret;
}

#endif /* CONFIG_NET_ICMPv6_SOCKET */
