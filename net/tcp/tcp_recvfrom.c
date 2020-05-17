/****************************************************************************
 * net/tcp/tcp_recvfrom.c
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
#ifdef CONFIG_NET_TCP

#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "tcp/tcp.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF    ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF    ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define TCPIPv4BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define TCPIPv6BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tcp_recvfrom_s
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
 * Name: tcp_update_recvlen
 *
 * Description:
 *   Update information about space available for new data and update size
 *   of data in buffer.
 *
 * Input Parameters:
 *   pstate   recvfrom state structure
 *   recvlen  size of new data appended to buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void tcp_update_recvlen(FAR struct tcp_recvfrom_s *pstate,
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
 * Name: tcp_recvfrom_newdata
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

static size_t tcp_recvfrom_newdata(FAR struct net_driver_s *dev,
                                   FAR struct tcp_recvfrom_s *pstate)
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

  tcp_update_recvlen(pstate, recvlen);

  return recvlen;
}

/****************************************************************************
 * Name: tcp_newdata
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

static inline void tcp_newdata(FAR struct net_driver_s *dev,
                               FAR struct tcp_recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  size_t recvlen = tcp_recvfrom_newdata(dev, pstate);

  /* If there is more data left in the packet that we could not buffer, then
   * add it to the read-ahead buffers.
   */

  if (recvlen < dev->d_len)
    {
      FAR struct tcp_conn_s *conn =
        (FAR struct tcp_conn_s *)pstate->ir_sock->s_conn;
      FAR uint8_t *buffer = (FAR uint8_t *)dev->d_appdata + recvlen;
      uint16_t buflen = dev->d_len - recvlen;
#ifdef CONFIG_DEBUG_NET
      uint16_t nsaved;

      nsaved = tcp_datahandler(conn, buffer, buflen);
#else
      tcp_datahandler(conn, buffer, buflen);
#endif

      /* There are complicated buffering issues that are not addressed fully
       * here.  For example, what if up_datahandler() cannot buffer the
       * remainder of the packet?  In that case, the data will be dropped but
       * still ACKed.  Therefore it would not be resent.
       *
       * This is probably not an issue here because we only get here if the
       * read-ahead buffers are empty and there would have to be something
       * serioulsy wrong with the configuration not to be able to buffer a
       * partial packet in this context.
       */

#ifdef CONFIG_DEBUG_NET
      if (nsaved < buflen)
        {
          nerr("ERROR: packet data not saved (%d bytes)\n", buflen - nsaved);
        }
#endif
    }

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}

/****************************************************************************
 * Name: tcp_readahead
 *
 * Description:
 *   Copy the read-ahead data from the packet
 *
 * Input Parameters:
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static inline void tcp_readahead(struct tcp_recvfrom_s *pstate)
{
  FAR struct tcp_conn_s *conn =
    (FAR struct tcp_conn_s *)pstate->ir_sock->s_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any TCP data already buffered in a read-ahead
   * buffer.
   */

  while ((iob = iob_peek_queue(&conn->readahead)) != NULL &&
          pstate->ir_buflen > 0)
    {
      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(pstate->ir_buffer, iob, pstate->ir_buflen, 0);
      ninfo("Received %d bytes (of %d)\n", recvlen, iob->io_pktlen);

      /* Update the accumulated size of the data read */

      tcp_update_recvlen(pstate, recvlen);

      /* If we took all of the data from the I/O buffer chain is empty, then
       * release it.  If there is still data available in the I/O buffer
       * chain, then just trim the data that we have taken from the
       * beginning of the I/O buffer chain.
       */

      if (recvlen >= iob->io_pktlen)
        {
          FAR struct iob_s *tmp;

          /* Remove the I/O buffer chain from the head of the read-ahead
           * buffer queue.
           */

          tmp = iob_remove_queue(&conn->readahead);
          DEBUGASSERT(tmp == iob);
          UNUSED(tmp);

          /* And free the I/O buffer chain */

          iob_free_chain(iob, IOBUSER_NET_TCP_READAHEAD);
        }
      else
        {
          /* The bytes that we have received from the head of the I/O
           * buffer chain (probably changing the head of the I/O
           * buffer queue).
           */

          iob_trimhead_queue(&conn->readahead, recvlen,
                             IOBUSER_NET_TCP_READAHEAD);
        }
    }
}

/****************************************************************************
 * Name: tcp_sender
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
 *   The network is locked
 *
 ****************************************************************************/

static inline void tcp_sender(FAR struct net_driver_s *dev,
                              FAR struct tcp_recvfrom_s *pstate)
{
  /* Get the family from the packet type, IP address from the IP header, and
   * the port number from the TCP header.
   */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct sockaddr_in6 *infrom =
        (FAR struct sockaddr_in6 *)pstate->ir_from;

      if (infrom)
        {
          FAR struct tcp_hdr_s *tcp   = TCPIPv6BUF;
          FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

          infrom->sin6_family = AF_INET6;
          infrom->sin6_port   = tcp->srcport;

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
          FAR struct tcp_hdr_s *tcp   = TCPIPv4BUF;
          FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

          infrom->sin_family = AF_INET;
          infrom->sin_port   = tcp->srcport;

          net_ipv4addr_copy(infrom->sin_addr.s_addr,
                            net_ip4addr_conv32(ipv4->srcipaddr));
          memset(infrom->sin_zero, 0, sizeof(infrom->sin_zero));
        }
    }
#endif /* CONFIG_NET_IPv4 */
}

/****************************************************************************
 * Name: tcp_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   TCP receive operation via by the lower, device interfacing layer.
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

static uint16_t tcp_eventhandler(FAR struct net_driver_s *dev,
                                 FAR void *pvconn, FAR void *pvpriv,
                                 uint16_t flags)
{
  FAR struct tcp_recvfrom_s *pstate = (struct tcp_recvfrom_s *)pvpriv;

#if 0 /* REVISIT: The assertion fires.  Why? */
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pvconn;

  /* The TCP socket is connected and, hence, should be bound to a device.
   * Make sure that the polling device is the own that we are bound to.
   */

  DEBUGASSERT(conn->dev == NULL || conn->dev == dev);
  if (conn->dev != NULL && conn->dev != dev)
    {
      return flags;
    }
#endif

  ninfo("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If new data is available, then complete the read action. */

      if ((flags & TCP_NEWDATA) != 0)
        {
          /* Copy the data from the packet (saving any unused bytes from the
           * packet in the read-ahead buffer).
           */

          tcp_newdata(dev, pstate);

          /* Save the sender's address in the caller's 'from' location */

          tcp_sender(dev, pstate);

          /* Indicate that the data has been consumed and that an ACK
           * should be sent.
           */

          flags = (flags & ~TCP_NEWDATA) | TCP_SNDACK;

          /* Check for transfer complete.  We will consider the
           * TCP/IP transfer complete as soon as any data has been received.
           * This is safe because if any additional data is received, it
           * will be retained in the TCP/IP read-ahead buffer until the
           * next receive is performed.
           */

          if (pstate->ir_recvlen > 0)
            {
              ninfo("TCP resume\n");

              /* The TCP receive buffer is non-empty.  Return now and don't
               * allow any further TCP call backs.
               */

              pstate->ir_cb->flags   = 0;
              pstate->ir_cb->priv    = NULL;
              pstate->ir_cb->event   = NULL;

              /* Wake up the waiting thread, returning the number of bytes
               * actually read.
               */

              nxsem_post(&pstate->ir_sem);
            }
        }

      /* Check for a loss of connection.
       *
       * TCP_DISCONN_EVENTS:
       *   TCP_CLOSE:    The remote host has closed the connection
       *   TCP_ABORT:    The remote host has aborted the connection
       *   TCP_TIMEDOUT: Connection aborted due to too many retransmissions.
       *   NETDEV_DOWN:  The network device went down
       */

      else if ((flags & TCP_DISCONN_EVENTS) != 0)
        {
          FAR struct socket *psock = pstate->ir_sock;

          nwarn("WARNING: Lost connection\n");

          /* We could get here recursively through the callback actions of
           * tcp_lost_connection().  So don't repeat that action if we have
           * already been disconnected.
           */

          DEBUGASSERT(psock != NULL);
          if (_SS_ISCONNECTED(psock->s_flags))
            {
              /* Handle loss-of-connection event */

              tcp_lost_connection(psock, pstate->ir_cb, flags);
            }

          /* Check if the peer gracefully closed the connection. */

          if ((flags & TCP_CLOSE) != 0)
            {
              /* This case should always return success (zero)! The value of
               * ir_recvlen, if zero, will indicate that the connection was
               * gracefully closed.
               */

              pstate->ir_result = 0;
            }
          else
            {
              pstate->ir_result = -ENOTCONN;
            }

          /* Wake up the waiting thread */

          nxsem_post(&pstate->ir_sem);
        }
    }

  return flags;
}

/****************************************************************************
 * Name: tcp_recvfrom_initialize
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

static void tcp_recvfrom_initialize(FAR struct socket *psock, FAR void *buf,
                                    size_t len, FAR struct sockaddr *infrom,
                                    FAR socklen_t *fromlen,
                                    FAR struct tcp_recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct tcp_recvfrom_s));

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

#define tcp_recvfrom_uninitialize(s) nxsem_destroy(&(s)->ir_sem)

/****************************************************************************
 * Name: tcp_recvfrom_result
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

static ssize_t tcp_recvfrom_result(int result, struct tcp_recvfrom_s *pstate)
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
 * Name: psock_tcp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a TCP/IP SOCK_STREAM
 *
 * Input Parameters:
 *   psock    Pointer to the socket structure for the SOCK_DRAM socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     INET address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_tcp_recvfrom(FAR struct socket *psock, FAR void *buf,
                           size_t len, int flags, FAR struct sockaddr *from,
                           FAR socklen_t *fromlen)
{
  struct tcp_recvfrom_s state;
  int               ret;

  /* Initialize the state structure.  This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  net_lock();
  tcp_recvfrom_initialize(psock, buf, len, from, fromlen, &state);

  /* Handle any any TCP data already buffered in a read-ahead buffer.  NOTE
   * that there may be read-ahead data to be retrieved even after the
   * socket has been disconnected.
   */

  tcp_readahead(&state);

  /* The default return value is the number of bytes that we just copied
   * into the user buffer.  We will return this if the socket has become
   * disconnected or if the user request was completely satisfied with
   * data from the readahead buffers.
   */

  ret = state.ir_recvlen;

  /* Verify that the SOCK_STREAM has been and still is connected */

  if (!_SS_ISCONNECTED(psock->s_flags))
    {
      /* Was any data transferred from the readahead buffer after we were
       * disconnected?  If so, then return the number of bytes received.  We
       * will wait to return end disconnection indications the next time that
       * recvfrom() is called.
       *
       * If no data was received (i.e.,  ret == 0  -- it will not be
       * negative) and the connection was gracefully closed by the remote
       * peer, then return success.  If ir_recvlen is zero, the caller of
       * recvfrom() will get an end-of-file indication.
       */

      if (ret <= 0 && !_SS_ISCLOSED(psock->s_flags))
        {
          /* Nothing was previously received from the read-ahead buffers.
           * The SOCK_STREAM must be (re-)connected in order to receive any
           * additional data.
           */

          ret = -ENOTCONN;
        }
    }

  /* In general, this implementation will not support non-blocking socket
   * operations... except in a few cases:  Here for TCP receive with read-
   * ahead enabled.  If this socket is configured as non-blocking then
   * return EAGAIN if no data was obtained from the read-ahead buffers.
   */

  else if (_SS_ISNONBLOCK(psock->s_flags) || (flags & MSG_DONTWAIT) != 0)
    {
      /* Return the number of bytes read from the read-ahead buffer if
       * something was received (already in 'ret'); EAGAIN if not.
       */

      if (ret <= 0)
        {
          /* Nothing was received */

          ret = -EAGAIN;
        }
    }

  /* It is okay to block if we need to.  If there is space to receive
   * anything more, then we will wait to receive the data.  Otherwise return
   * the number of bytes read from the read-ahead buffer (already in 'ret').
   */

  else

  /* We get here when we we decide that we need to setup the wait for
   * incoming TCP/IP data.  Just a few more conditions to check:
   *
   * 1) Make sure thet there is buffer space to receive additional data
   *    (state.ir_buflen > 0).  This could be zero, for example,  we filled
   *    the user buffer with data from the read-ahead buffers.  And
   * 2) then we not want to wait if we already obtained some data from the
   *    read-ahead buffer.  In that case, return now with what we have (don't
   *    want for more because there may be no timeout).
   */

  if (state.ir_recvlen == 0 && state.ir_buflen > 0)
    {
      FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)psock->s_conn;

      /* Set up the callback in the connection */

      state.ir_cb = tcp_callback_alloc(conn);
      if (state.ir_cb)
        {
          state.ir_cb->flags   = (TCP_NEWDATA | TCP_DISCONN_EVENTS);
          state.ir_cb->priv    = (FAR void *)&state;
          state.ir_cb->event   = tcp_eventhandler;

          /* Wait for either the receive to complete or for an error/timeout
           * to occur.  net_timedwait will also terminate if a signal isi
           * received.
           */

          ret = net_timedwait(&state.ir_sem, _SO_TIMEOUT(psock->s_rcvtimeo));
          if (ret == -ETIMEDOUT)
            {
              ret = -EAGAIN;
            }

          /* Make sure that no further events are processed */

          tcp_callback_free(conn, state.ir_cb);
          ret = tcp_recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }
    }

  net_unlock();
  tcp_recvfrom_uninitialize(&state);
  return (ssize_t)ret;
}

#endif /* CONFIG_NET_TCP */
