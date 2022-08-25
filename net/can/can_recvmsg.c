/****************************************************************************
 * net/can/can_recvmsg.c
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

#ifdef CONFIG_NET_CAN

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "can/can.h"
#include "socket/socket.h"
#include <netpacket/packet.h>

#ifdef CONFIG_NET_TIMESTAMP
#include <sys/time.h>
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct can_recvfrom_s
{
  FAR struct socket *pr_sock;          /* The parent socket structure */
  FAR struct devif_callback_s *pr_cb;  /* Reference to callback instance */
  sem_t        pr_sem;                 /* Semaphore signals recv completion */
  size_t       pr_buflen;              /* Length of receive buffer */
  FAR uint8_t *pr_buffer;              /* Pointer to receive buffer */
  ssize_t      pr_recvlen;             /* The received length */
  size_t       pr_msglen;              /* Length of msg buffer */
  FAR uint8_t *pr_msgbuf;              /* Pointer to msg buffer */
  int          pr_result;              /* Success:OK, failure:negated errno */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_add_recvlen
 *
 * Description:
 *   Update information about space available for new data and update size
 *   of data in buffer,  This logic accounts for the case where
 *   recvfrom_udpreadahead() sets state.pr_recvlen == -1 .
 *
 * Input Parameters:
 *   pstate   recvfrom state structure
 *   recvlen  size of new data appended to buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void can_add_recvlen(FAR struct can_recvfrom_s *pstate,
                                   size_t recvlen)
{
  if (pstate->pr_recvlen < 0)
    {
      pstate->pr_recvlen = 0;
    }

  pstate->pr_recvlen += recvlen;
  pstate->pr_buffer  += recvlen;
  pstate->pr_buflen  -= recvlen;
}

/****************************************************************************
 * Name: can_recvfrom_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event.
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static size_t can_recvfrom_newdata(FAR struct net_driver_s *dev,
                                 FAR struct can_recvfrom_s *pstate)
{
  size_t recvlen;

  if (dev->d_len > pstate->pr_buflen)
    {
      recvlen = pstate->pr_buflen;
    }
  else
    {
      recvlen = dev->d_len;
    }

  /* Copy the new packet data into the user buffer */

  memcpy(pstate->pr_buffer, dev->d_buf, recvlen);

  /* Update the accumulated size of the data read */

  can_add_recvlen(pstate, recvlen);

  return recvlen;
}

/****************************************************************************
 * Name: can_newdata
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

static inline void can_newdata(FAR struct net_driver_s *dev,
                               FAR struct can_recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  size_t recvlen = can_recvfrom_newdata(dev, pstate);

  /* If there is more data left in the packet that we could not buffer, then
   * add it to the read-ahead buffers.
   */

  if (recvlen < dev->d_len)
    {
      FAR struct can_conn_s *conn =
        (FAR struct can_conn_s *)pstate->pr_sock->s_conn;
      FAR uint8_t *buffer = (FAR uint8_t *)dev->d_appdata + recvlen;
      uint16_t buflen = dev->d_len - recvlen;
#ifdef CONFIG_DEBUG_NET
      uint16_t nsaved;

      nsaved = can_datahandler(conn, buffer, buflen);
#else
      can_datahandler(conn, buffer, buflen);
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
 * Name: can_readahead
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

static inline int can_readahead(struct can_recvfrom_s *pstate)
{
  FAR struct can_conn_s *conn =
    (FAR struct can_conn_s *) pstate->pr_sock->s_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any CAN data already buffered in a read-ahead
   * buffer.
   */

  pstate->pr_recvlen = -1;

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL &&
      pstate->pr_buflen > 0)
    {
      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(pstate->pr_buffer, iob, pstate->pr_buflen, 0);

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

          iob_free_chain(iob);
        }
      else
        {
          /* The bytes that we have received from the head of the I/O
           * buffer chain (probably changing the head of the I/O
           * buffer queue).
           */

          iob_trimhead_queue(&conn->readahead, recvlen);
        }

      /* do not pass frames with DLC > 8 to a legacy socket */
#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)
      if (!conn->fd_frames)
#endif
        {
          if (recvlen > sizeof(struct can_frame))
            {
              return 0;
            }
        }

      return recvlen;
    }

  return 0;
}

/****************************************************************************
 * Name: can_readahead
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

#ifdef CONFIG_NET_TIMESTAMP
static inline int can_readahead_timestamp(struct can_conn_s *conn,
                                          FAR uint8_t *buffer)
{
  FAR struct iob_s *iob;
  int recvlen;

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL)
    {
      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(buffer, iob, sizeof(struct timeval), 0);

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

          iob_free_chain(iob);
        }
      else
        {
          /* The bytes that we have received from the head of the I/O
           * buffer chain (probably changing the head of the I/O
           * buffer queue).
           */

          iob_trimhead_queue(&conn->readahead, recvlen);
        }

      return recvlen;
    }

  return 0;
}
#endif

#ifdef CONFIG_NET_CANPROTO_OPTIONS
static int can_recv_filter(struct can_conn_s *conn, canid_t id)
{
  uint32_t i;
  for (i = 0; i < conn->filter_count; i++)
    {
      if (conn->filters[i].can_id & CAN_INV_FILTER)
        {
          if ((id & conn->filters[i].can_mask) !=
                ((conn->filters[i].can_id & ~CAN_INV_FILTER) &
                conn->filters[i].can_mask))
            {
              return 1;
            }
        }
      else
        {
          if ((id & conn->filters[i].can_mask) ==
                (conn->filters[i].can_id & conn->filters[i].can_mask))
            {
              return 1;
            }
        }
    }

  return 0;
}
#endif

static uint16_t can_recvfrom_eventhandler(FAR struct net_driver_s *dev,
                                          FAR void *pvpriv, uint16_t flags)
{
  struct can_recvfrom_s *pstate = pvpriv;
#if defined(CONFIG_NET_CANPROTO_OPTIONS) || defined(CONFIG_NET_TIMESTAMP)
  struct can_conn_s *conn = (struct can_conn_s *)pstate->pr_sock->s_conn;
#endif

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      if ((flags & CAN_NEWDATA) != 0)
        {
          /* If a new packet is available, check receive filters
           * when is valid then complete the read action.
           */
#ifdef CONFIG_NET_CANPROTO_OPTIONS
          if (can_recv_filter(conn, (canid_t) *dev->d_appdata) == 0)
            {
              flags &= ~CAN_NEWDATA;
              return flags;
            }
#endif

          /* do not pass frames with DLC > 8 to a legacy socket */
#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)
          if (!conn->fd_frames)
#endif
            {
#ifdef CONFIG_NET_TIMESTAMP
              if ((conn->sconn.s_timestamp && (dev->d_len >
                  sizeof(struct can_frame) + sizeof(struct timeval)))
                  || (!conn->sconn.s_timestamp && (dev->d_len >
                   sizeof(struct can_frame))))
#else
              if (dev->d_len > sizeof(struct can_frame))
#endif
                {
                  /* DO WE NEED TO CLEAR FLAGS?? */

                  flags &= ~CAN_NEWDATA;
                  return flags;
                }
            }

          /* Copy the packet */

          can_newdata(dev, pstate);

#ifdef CONFIG_NET_TIMESTAMP
          if (conn->sconn.s_timestamp)
            {
              if (pstate->pr_msglen == sizeof(struct timeval))
                {
                  can_readahead_timestamp(conn, pstate->pr_msgbuf);
                }
              else
                {
                  /* We still have to consume the data
                   * otherwise IOB gets full
                   */

                  uint8_t dummy_buf[sizeof(struct timeval)];
                  can_readahead_timestamp(conn, (uint8_t *)&dummy_buf);
                }
            }
#endif

          /* We are finished. */

          /* Don't allow any further call backs. */

          pstate->pr_cb->flags   = 0;
          pstate->pr_cb->priv    = NULL;
          pstate->pr_cb->event   = NULL;

          /* indicate that the data has been consumed */

          flags &= ~CAN_NEWDATA;

          /* Wake up the waiting thread, returning the number of bytes
           * actually read.
           */

          nxsem_post(&pstate->pr_sem);
        }
    }

  return flags;
}

/****************************************************************************
 * Name: can_recvfrom_result
 *
 * Description:
 *   Evaluate the result of the recv operations
 *
 * Input Parameters:
 *   result   The result of the net_lockedwait operation (may indicate EINTR)
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   The result of the recv operation with errno set appropriately
 *
 * Assumptions:
 *
 ****************************************************************************/

static ssize_t can_recvfrom_result(int result,
                                   FAR struct can_recvfrom_s *pstate)
{
  /* Check for a error/timeout detected by the event handler.  Errors are
   * signaled by negative errno values for the rcv length
   */

  if (pstate->pr_result < 0)
    {
      /* This might return EAGAIN on a timeout */

      return pstate->pr_result;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal.
   * In this case, net_lockedwait will have returned negated errno
   * appropriately.
   */

  if (result < 0)
    {
      return result;
    }

  return pstate->pr_recvlen;
}

/****************************************************************************
 * Name: can_recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument 'msg_namelen'
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags (ignored)
 *
 ****************************************************************************/

ssize_t can_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                    int flags)
{
  FAR struct can_conn_s *conn;
  FAR struct net_driver_s *dev;
  struct can_recvfrom_s state;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = (FAR struct can_conn_s *)psock->s_conn;

  if (psock->s_type != SOCK_RAW)
    {
      nerr("ERROR: Unsupported socket type: %d\n", psock->s_type);
      ret = -ENOSYS;
    }

  net_lock();

  /* Initialize the state structure. */

  memset(&state, 0, sizeof(struct can_recvfrom_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&state.pr_sem, 0, 0); /* Doesn't really fail */
  nxsem_set_protocol(&state.pr_sem, SEM_PRIO_NONE);

  state.pr_buflen = msg->msg_iov->iov_len;
  state.pr_buffer = msg->msg_iov->iov_base;

#ifdef CONFIG_NET_TIMESTAMP
  if (conn->sconn.s_timestamp && msg->msg_controllen >=
        (sizeof(struct cmsghdr) + sizeof(struct timeval)))
    {
      struct cmsghdr *cmsg = CMSG_FIRSTHDR(msg);
      state.pr_msglen = sizeof(struct timeval);
      state.pr_msgbuf = CMSG_DATA(cmsg);
      cmsg->cmsg_level = SOL_SOCKET;
      cmsg->cmsg_type = SO_TIMESTAMP;
      cmsg->cmsg_len = state.pr_msglen;
      msg->msg_controllen = sizeof(struct cmsghdr) + sizeof(struct timeval);
    }
  else
    {
      /* Expected behavior is that the msg_controllen becomes 0,
       * otherwise CMSG_NXTHDR will go into a infinite loop
       */

      msg->msg_controllen = 0;
    }
#endif

  state.pr_sock   = psock;

  /* Handle any any CAN data already buffered in a read-ahead buffer.  NOTE
   * that there may be read-ahead data to be retrieved even after the
   * socket has been disconnected.
   */

  ret = can_readahead(&state);
  if (ret > 0)
    {
#ifdef CONFIG_NET_TIMESTAMP
      if (conn->sconn.s_timestamp)
        {
          if (state.pr_msglen == sizeof(struct timeval))
            {
              can_readahead_timestamp(conn, state.pr_msgbuf);
            }
          else
            {
              /* We still have to consume the data otherwise IOB gets full */

              uint8_t dummy_buf[sizeof(struct timeval)];
              can_readahead_timestamp(conn, (uint8_t *)&dummy_buf);
            }
        }
#endif

      goto errout_with_state;
    }

  ret = state.pr_recvlen;

  /* Handle non-blocking CAN sockets */

  if (_SS_ISNONBLOCK(conn->sconn.s_flags) || (flags & MSG_DONTWAIT) != 0)
    {
      /* Return the number of bytes read from the read-ahead buffer if
       * something was received (already in 'ret'); EAGAIN if not.
       */

      if (ret < 0)
        {
          /* Nothing was received */

          ret = -EAGAIN;
          goto errout_with_state;
        }
    }

  /* Get the device driver that will service this transfer */

  dev  = conn->dev;
  if (dev == NULL)
    {
      ret = -ENODEV;
      goto errout_with_state;
    }

  /* Set up the callback in the connection */

  state.pr_cb = can_callback_alloc(dev, conn);
  if (state.pr_cb)
    {
      state.pr_cb->flags  = (CAN_NEWDATA | CAN_POLL);
      state.pr_cb->priv   = (FAR void *)&state;
      state.pr_cb->event  = can_recvfrom_eventhandler;

      /* Wait for either the receive to complete or for an error/timeout to
       * occur. NOTES:  (1) net_lockedwait will also terminate if a signal
       * is received, (2) the network is locked!  It will be un-locked while
       * the task sleeps and automatically re-locked when the task restarts.
       */

      ret = net_lockedwait(&state.pr_sem);

      /* Make sure that no further events are processed */

      can_callback_free(dev, conn, state.pr_cb);
      ret = can_recvfrom_result(ret, &state);
    }
  else
    {
      ret = -EBUSY;
    }

errout_with_state:
  net_unlock();
  nxsem_destroy(&state.pr_sem);
  return ret;
}

#endif /* CONFIG_NET_CAN */
