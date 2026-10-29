/****************************************************************************
 * net/can/can_raw.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <utils/utils.h>

#include <arch/irq.h>

#include <netpacket/packet.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/net/can.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/poll.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#include "can/can.h"
#include "devif/devif.h"
#include "netdev/netdev.h"
#include "socket/socket.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_CAN_MAX_CONNS
#  define CONFIG_NET_CAN_MAX_CONNS 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifndef CONFIG_NET_CAN_WRITE_BUFFERS
/* This structure holds the state of the send operation until it can be
 * operated upon by the event handler.
 */

struct send_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  FAR struct can_conn_s  *snd_conn;    /* Reference to the CAN connection */
  sem_t                   snd_sem;     /* Used to wake up the waiting thread */
  FAR const uint8_t      *snd_buffer;  /* Points to the buffer of data to send */
  size_t                  snd_buflen;  /* Number of bytes in the buffer to send */
  size_t                  pr_msglen;   /* Length of msg buffer */
  FAR uint8_t            *pr_msgbuf;   /* Pointer to msg buffer */
  ssize_t                 snd_sent;    /* The number of bytes sent */
};
#endif

struct can_recvfrom_s
{
  FAR struct can_conn_s *pr_conn;      /* Connection associated with the socket */
  FAR struct devif_callback_s *pr_cb;  /* Reference to callback instance */
  FAR struct msghdr *pr_msg;           /* Pointer to receive buffer */
  sem_t        pr_sem;                 /* Semaphore signals recv completion */
  ssize_t      pr_recvlen;             /* The received length */
  int          pr_result;              /* Success:OK, failure:negated errno */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int can_raw_setup(FAR struct socket *psock);
static sockcaps_t can_raw_sockcaps(FAR struct socket *psock);
static void can_raw_addref(FAR struct socket *psock);
static int can_raw_bind(FAR struct socket *psock,
                        FAR const struct sockaddr *addr, socklen_t addrlen);
static int can_raw_getsockname(FAR struct socket *psock,
                               FAR struct sockaddr *addr,
                               FAR socklen_t *addrlen);
static int can_raw_poll(FAR struct socket *psock,
                        FAR struct pollfd_s *fds, bool setup);
static ssize_t can_raw_sendmsg(FAR struct socket *psock,
                               FAR const struct msghdr *msg, int flags);
static ssize_t can_raw_recvmsg(FAR struct socket *psock,
                               FAR struct msghdr *msg, int flags);
static int can_raw_close(FAR struct socket *psock);
#if defined(CONFIG_NET_SOCKOPTS) && defined(CONFIG_NET_CANPROTO_OPTIONS)
static int can_raw_getsockopt(FAR struct socket *psock, int level,
                              int option, FAR void *value,
                              FAR socklen_t *value_len);
static int can_raw_setsockopt(FAR struct socket *psock, int level,
                              int option, FAR const void *value,
                              socklen_t value_len);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all CAN_RAW connections. */

NET_BUFPOOL_DECLARE(g_can_raw_connections, sizeof(struct can_conn_s),
                    CONFIG_NET_CAN_PREALLOC_CONNS,
                    CONFIG_NET_CAN_ALLOC_CONNS,
                    CONFIG_NET_CAN_MAX_CONNS);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_can_raw_sockif =
{
  can_raw_setup,        /* si_setup */
  can_raw_sockcaps,     /* si_sockcaps */
  can_raw_addref,       /* si_addref */
  can_raw_bind,         /* si_bind */
  can_raw_getsockname,  /* si_getsockname */
  NULL,                 /* si_getpeername */
  NULL,                 /* si_listen */
  NULL,                 /* si_connect */
  NULL,                 /* si_accept */
  can_raw_poll,         /* si_poll */
  can_raw_sendmsg,      /* si_sendmsg */
  can_raw_recvmsg,      /* si_recvmsg */
  can_raw_close,        /* si_close */
  NULL,                 /* si_ioctl */
  NULL,                 /* si_socketpair */
  NULL                  /* si_shutdown */
#if defined(CONFIG_NET_SOCKOPTS) && defined(CONFIG_NET_CANPROTO_OPTIONS)
  , can_raw_getsockopt  /* si_getsockopt */
  , can_raw_setsockopt  /* si_setsockopt */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_raw_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized NetLink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

static FAR struct can_conn_s *can_raw_alloc(void)
{
  FAR struct can_conn_s *conn;

  /* The free list is protected by a a mutex. */

  NET_BUFPOOL_LOCK(g_can_raw_connections);

  conn = NET_BUFPOOL_TRYALLOC(g_can_raw_connections);
  if (conn != NULL)
    {
      /* FIXME SocketCAN default behavior enables loopback */

#ifdef CONFIG_NET_CANPROTO_OPTIONS
      /* By default the filter is configured to catch all,
       * this is done in commented filter code below:
       *
       * struct can_filter_t catchall_filter;
       * filter.can_id = 0;
       * filter.can_mask = 0;
       * conn->filters[0] = catchall_filter;
       *
       * However memset already sets the filter to 0
       * therefore we only have to set the filter count to 1
       */

      conn->filter_count = 1;
#endif

      /* Initialize the shared state and enqueue the connection into the
       * active list
       */

      can_conn_init(conn);
    }

  NET_BUFPOOL_UNLOCK(g_can_raw_connections);
  return conn;
}

/****************************************************************************
 * Name: can_raw_free()
 *
 * Description:
 *   Free a NetLink connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

static void can_raw_free(FAR struct can_conn_s *conn)
{
  /* The free list is protected by a mutex. */

  DEBUGASSERT(conn->crefs == 0);

  NET_BUFPOOL_LOCK(g_can_raw_connections);

  can_conn_uninit(conn);

  /* Free the connection. */

  NET_BUFPOOL_FREE(g_can_raw_connections, conn);

  NET_BUFPOOL_UNLOCK(g_can_raw_connections);
}

/****************************************************************************
 * Name: can_recvfrom_initialize
 *
 * Description:
 *   Initialize the state structure
 *
 * Input Parameters:
 *   conn     The CAN connection of interest
 *   msg      Receive info and buffer for receive data
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void can_recvfrom_initialize(FAR struct can_conn_s *conn,
                                    FAR struct msghdr *msg,
                                    FAR struct can_recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct can_recvfrom_s));
  nxsem_init(&pstate->pr_sem, 0, 0);

  pstate->pr_conn = conn;
  pstate->pr_msg  = msg;
}

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
  unsigned int offset;
  ssize_t recvlen;

#ifdef CONFIG_NET_TIMESTAMP
  cmsg_store_timestamp(pstate->pr_msg, &dev->d_iob->io_time,
                       pstate->pr_conn->sconn.s_options);
#endif

  if (dev->d_iob->io_conn == &pstate->pr_conn->sconn)
    {
      pstate->pr_msg->msg_flags |= MSG_CONFIRM;
    }

  recvlen = MIN(pstate->pr_msg->msg_iov->iov_len, dev->d_len);

  /* Copy the new packet data into the user buffer */

  offset = (dev->d_appdata - dev->d_iob->io_data) - dev->d_iob->io_offset;

  recvlen = iob_copyout(pstate->pr_msg->msg_iov->iov_base,
                        dev->d_iob, recvlen, offset);
  if (recvlen <= 0)
    {
      return 0;
    }

  /* Trim the copied buffers */

  dev->d_iob = iob_trimhead(dev->d_iob, recvlen + offset);

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
      if (can_datahandler(dev, pstate->pr_conn) < dev->d_len)
        {
          ninfo("Dropped %d bytes\n", dev->d_len);
#ifdef CONFIG_NET_STATISTICS
          g_netstats.can.drop++;
#endif
        }
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
  FAR struct can_conn_s *conn = pstate->pr_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any CAN data already buffered in a read-ahead
   * buffer.
   */

  pstate->pr_recvlen = -ENODATA;

  if (pstate->pr_msg->msg_iov->iov_len > 0 &&
      (iob = iob_remove_queue(&conn->readahead)) != NULL)
    {
      DEBUGASSERT(iob->io_pktlen > 0);

#ifdef CONFIG_NET_TIMESTAMP
      cmsg_store_timestamp(pstate->pr_msg, &iob->io_time,
                           conn->sconn.s_options);
#endif

      if (iob->io_conn == &conn->sconn)
        {
          pstate->pr_msg->msg_flags |= MSG_CONFIRM;
        }

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(pstate->pr_msg->msg_iov->iov_base,
                            iob, pstate->pr_msg->msg_iov->iov_len, 0);

      /* Update the accumulated size of the data read */

      pstate->pr_recvlen = recvlen;

      /* The rest of a frame larger than the user buffer is discarded */

      if (iob->io_pktlen > pstate->pr_msg->msg_iov->iov_len)
        {
          pstate->pr_msg->msg_flags |= MSG_TRUNC;
        }

      /* We have taken all of the data we need from the I/O buffer chain,
       * so release it. There is no trimming needed, since One CAN/CANFD
       * frame can always fit in one IOB.
       */

      static_assert(sizeof(struct can_frame) <= CONFIG_IOB_BUFSIZE);

      /* Free the I/O buffer chain */

      iob_free_chain(iob);

      return recvlen;
    }

  return 0;
}

static uint32_t can_recvfrom_eventhandler(FAR struct net_driver_s *dev,
                                          FAR void *pvpriv, uint32_t flags)
{
  struct can_recvfrom_s *pstate = pvpriv;

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      if ((flags & CAN_NEWDATA) != 0)
        {
          /* Copy the packet.  can_active() does not deliver CAN FD frames
           * to a classic CAN socket.
           */

          can_newdata(dev, pstate);

          /* We are finished. */

          /* Don't allow any further call backs. */

          pstate->pr_cb->flags = 0;
          pstate->pr_cb->priv  = NULL;
          pstate->pr_cb->event = NULL;

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
 *   result   The result of the conn_dev_sem_timedwait operation
 *            (may indicate EINTR)
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

  /* If conn_dev_sem_timedwait failed, then we were probably reawakened by
   * a signal. In this case, conn_dev_sem_timedwait will have returned
   * negated errno appropriately.
   */

  if (result < 0)
    {
      return result;
    }

  return pstate->pr_recvlen;
}

/****************************************************************************
 * Name: can_raw_recvmsg
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

static ssize_t can_raw_recvmsg(FAR struct socket *psock,
                               FAR struct msghdr *msg, int flags)
{
  FAR struct can_conn_s *conn;
  FAR struct net_driver_s *dev = NULL;
  struct can_recvfrom_s state;
  int ret;

  conn = psock->s_conn;

  if (psock->s_type != SOCK_RAW)
    {
      nerr("ERROR: Unsupported socket type: %d\n", psock->s_type);
      return -ENOSYS;
    }

  if (msg->msg_iovlen != 1)
    {
      return -ENOTSUP;
    }

  conn_lock(&conn->sconn);

  /* Initialize the state structure. */

  can_recvfrom_initialize(conn, msg, &state);

  /* Handle any any CAN data already buffered in a read-ahead buffer.  NOTE
   * that there may be read-ahead data to be retrieved even after the
   * socket has been disconnected.
   */

  ret = can_readahead(&state);
  if (ret > 0)
    {
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

  dev = conn->dev;
  if (dev == NULL && _SS_ISBOUND(conn->sconn.s_flags))
    {
      dev = netdev_default();
    }

  if (dev == NULL)
    {
      ret = -ENODEV;
      goto errout_with_state;
    }

  conn_unlock(&conn->sconn);
  conn_dev_lock(&conn->sconn, dev);

  /* Set up the callback in the connection */

  state.pr_cb = can_callback_alloc(dev, conn);
  if (state.pr_cb)
    {
      state.pr_cb->flags = (CAN_NEWDATA | CAN_POLL);
      state.pr_cb->priv  = (FAR void *)&state;
      state.pr_cb->event = can_recvfrom_eventhandler;

      /* Wait for either the receive to complete or for an error/timeout to
       * occur. NOTES:  (1) conn_dev_sem_timedwait will also terminate if a
       * signal is received, (2) the network is locked!  It will be un-locked
       * while the task sleeps and automatically re-locked when the task
       * restarts.
       */

      ret = conn_dev_sem_timedwait(&state.pr_sem, true,
                                   _SO_TIMEOUT(conn->sconn.s_rcvtimeo),
                                   &conn->sconn, dev);
      if (ret == -ETIMEDOUT)
        {
          ret = -EAGAIN;
        }

      /* Make sure that no further events are processed */

      can_callback_free(dev, conn, state.pr_cb);
      ret = can_recvfrom_result(ret, &state);
    }
  else
    {
      ret = -EBUSY;
    }

errout_with_state:
  conn_dev_unlock(&conn->sconn, dev);
  nxsem_destroy(&state.pr_sem);
  return ret;
}

#ifndef CONFIG_NET_CAN_WRITE_BUFFERS
/****************************************************************************
 * Name: psock_send_eventhandler
 ****************************************************************************/

static uint32_t psock_send_eventhandler(FAR struct net_driver_s *dev,
                                        FAR void *pvpriv, uint32_t flags)
{
  FAR struct send_s *pstate = pvpriv;

  if (pstate)
    {
      /* Check if the outgoing packet is available. It may have been claimed
       * by a send event handler serving a different thread -OR- if the
       * output buffer currently contains unprocessed incoming data. In
       * these cases we will just have to wait for the next polling cycle.
       */

      if (dev->d_sndlen > 0 || (flags & CAN_NEWDATA) != 0)
        {
          /* Another thread has beat us sending data or the buffer is busy,
           * Check for a timeout. If not timed out, wait for the next
           * polling cycle and check again.
           */

          /* No timeout. Just wait for the next polling cycle */

          return flags;
        }

      /* It looks like we are good to send the data */

      else
        {
          /* Copy the packet data into the device packet buffer and send it */

          int ret = devif_send(dev, pstate->snd_buffer,
                               pstate->snd_buflen + pstate->pr_msglen, 0);

          dev->d_len = dev->d_sndlen - pstate->pr_msglen;
          if (ret <= 0)
            {
              pstate->snd_sent = ret;
              goto end_wait;
            }

          dev->d_iob->io_conn = &pstate->snd_conn->sconn;

          pstate->snd_sent = pstate->snd_buflen;
          if (pstate->pr_msglen > 0) /* concat cmsg data after packet */
            {
              memcpy(dev->d_buf + pstate->snd_buflen, pstate->pr_msgbuf,
                     pstate->pr_msglen);
            }
        }

end_wait:

      /* Don't allow any further call backs. */

      pstate->snd_cb->flags = 0;
      pstate->snd_cb->priv  = NULL;
      pstate->snd_cb->event = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->snd_sem);
    }

  return flags;
}

/****************************************************************************
 * Name: can_raw_sendmsg
 *
 * Description:
 *   The can_raw_sendmsg() call may be used only when the packet socket is
 *   in a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      CAN frame and optional CMSG
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See sendmsg() for the complete list
 *   of return values.
 *
 ****************************************************************************/

static ssize_t can_raw_sendmsg(FAR struct socket *psock,
                               FAR const struct msghdr *msg, int flags)
{
  FAR struct net_driver_s *dev;
  FAR struct can_conn_s *conn;
  struct send_s state;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  /* Only SOCK_RAW is supported */

  if (psock->s_type != SOCK_RAW)
    {
      /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and
       * no peer address is set.
       */

      return -EDESTADDRREQ;
    }

  conn = psock->s_conn;

  /* Get the device driver that will service this transfer */

  dev = conn->dev;
  if (dev == NULL)
    {
      return -ENODEV;
    }

#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)
  if (_SO_GETOPT(conn->sconn.s_options, CAN_RAW_FD_FRAMES))
    {
      if (msg->msg_iov->iov_len != CANFD_MTU &&
          msg->msg_iov->iov_len != CAN_MTU)
        {
          return -EINVAL;
        }
    }
  else
#endif
    {
      if (msg->msg_iov->iov_len != CAN_MTU)
        {
          return -EINVAL;
        }
    }

  /* Perform the send operation */

  /* Initialize the state structure. This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  conn_dev_lock(&conn->sconn, dev);
  memset(&state, 0, sizeof(struct send_s));
  nxsem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */

  state.snd_buflen = msg->msg_iov->iov_len;  /* bytes to send */
  state.snd_buffer = msg->msg_iov->iov_base; /* Buffer to send from */
  state.snd_conn   = conn;

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  if (msg->msg_controllen > sizeof(struct cmsghdr))
    {
      FAR struct cmsghdr *cmsg = CMSG_FIRSTHDR(msg);

      if (_SO_GETOPT(conn->sconn.s_options, CAN_RAW_TX_DEADLINE) &&
          cmsg->cmsg_level == SOL_CAN_RAW &&
          cmsg->cmsg_type == CAN_RAW_TX_DEADLINE &&
          cmsg->cmsg_len == sizeof(struct timeval))
        {
          state.pr_msgbuf = CMSG_DATA(cmsg); /* Buffer to cmsg data */
          state.pr_msglen = cmsg->cmsg_len;  /* len of cmsg data */
        }
    }
#endif

  /* Allocate resource to receive a callback */

  state.snd_cb = can_callback_alloc(dev, conn);
  if (state.snd_cb)
    {
      /* Set up the callback in the connection */

      state.snd_cb->flags = CAN_POLL;
      state.snd_cb->priv  = (FAR void *)&state;
      state.snd_cb->event = psock_send_eventhandler;

      /* Notify the device driver that new TX data is available. */

      netdev_txnotify_dev(dev, CAN_POLL);

      /* Wait for the send to complete or an error to occur.
       * conn_dev_sem_timedwait will also terminate if a signal is received.
       */

      if (_SS_ISNONBLOCK(conn->sconn.s_flags) || (flags & MSG_DONTWAIT) != 0)
        {
          ret = conn_dev_sem_timedwait(&state.snd_sem, true, 0,
                                       &conn->sconn, dev);
        }
      else
        {
          ret = conn_dev_sem_timedwait(&state.snd_sem, true, UINT_MAX,
                                       &conn->sconn, dev);
        }

      /* Make sure that no further events are processed */

      can_callback_free(dev, conn, state.snd_cb);
    }

  nxsem_destroy(&state.snd_sem);
  conn_dev_unlock(&conn->sconn, dev);

  /* Check for a errors, Errors are signalled by negative errno values
   * for the send length
   */

  if (state.snd_sent < 0)
    {
      return state.snd_sent;
    }

  /* If conn_dev_sem_timedwait failed, then we were probably reawakened by
   * a signal. In this case, conn_dev_sem_timedwait will have returned
   * negated errno appropriately.
   */

  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_NET_STATISTICS
  g_netstats.can.sent++;
#endif

  /* Return the number of bytes actually sent */

  return state.snd_sent;
}

/****************************************************************************
 * Name: can_raw_cansend
 *
 * Description:
 *   can_raw_cansend() returns a value indicating if a write to the socket
 *   would block.  No space in the buffer is actually reserved, so it is
 *   possible that the write may still block if the buffer is filled by
 *   another means.
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *
 * Returned Value:
 *   OK
 *     At least one byte of data could be successfully written.
 *   -EWOULDBLOCK
 *     There is no room in the output buffer.
 *   -EBADF
 *     An invalid descriptor was specified.
 *
 ****************************************************************************/

static int can_raw_cansend(FAR struct socket *psock)
{
  /* Verify that we received a valid socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      nerr("ERROR: Invalid socket\n");
      return -EBADF;
    }

  /* TODO Query CAN driver mailboxes to see if there's mailbox available */

  return OK;
}

#else
/****************************************************************************
 * Name: psock_send_eventhandler
 ****************************************************************************/

static uint32_t psock_send_eventhandler(FAR struct net_driver_s *dev,
                                        FAR void *pvpriv, uint32_t flags)
{
  FAR struct can_conn_s *conn = pvpriv;

  DEBUGASSERT(dev != NULL && conn != NULL);

  /* Check if the outgoing packet is available. It may have been claimed
   * by a send event handler serving a different thread -OR- if the
   * output buffer currently contains unprocessed incoming data. In
   * these cases we will just have to wait for the next polling cycle.
   */

  if (dev->d_sndlen > 0 || (flags & CAN_NEWDATA) != 0)
    {
      /* Another thread has beat us sending data or the buffer is busy,
       * Check for a timeout. If not timed out, wait for the next
       * polling cycle and check again.
       */

      /* No timeout. Just wait for the next polling cycle */

      return flags;
    }

  /* It looks like we are good to send the data */

  else
    {
      uint32_t write_q_len;
      FAR struct iob_s *wrb_iob;

      /* Copy the packet data into the device packet buffer and send it */

      write_q_len = iob_get_queue_entry_count(&conn->write_q);

      if (write_q_len == 0)
        {
          return flags;
        }

      /* Peek at the head of the write queue (but don't remove anything
       * from the write queue yet).  We know from the above test that
       * the write_q is not empty.
       */

      wrb_iob = (FAR struct iob_s *)iob_remove_queue(&conn->write_q);
      DEBUGASSERT(wrb_iob != NULL);

      /* Then set-up to send that amount of data with the offset
       * corresponding to the size of the IP-dependent address structure.
       */

      netdev_iob_replace(dev, wrb_iob);

      /* Get the amount of data that we can send in the next packet.
       * We will send either the remaining data in the buffer I/O
       * buffer chain, or as much as will fit given the MSS and current
       * window size.
       */

      dev->d_sndlen = wrb_iob->io_pktlen;
      ninfo("wrb=%p sndlen=%d\n", wrb_iob, dev->d_sndlen);

      if (write_q_len > 1)
        {
          /* Set up for the next packet transfer
           * next packet now at the header of the write buffer queue.
           */

          /* Notify the device driver that new TX data is available. */

          netdev_txnotify_dev(dev, CAN_POLL);
        }
      else
        {
          /* stifle any further callbacks until more write data is
           * enqueued.
           */

          conn->sndcb->flags = 0;
          conn->sndcb->priv = NULL;
          conn->sndcb->event = NULL;
        }

      /* Only one data can be sent by low level driver at once,
       * tell the caller stop polling the other connections.
       */

      flags &= ~CAN_POLL;

#if CONFIG_NET_SEND_BUFSIZE > 0
      can_sendbuffer_notify(conn);
#endif
    }

  return flags;
}

/****************************************************************************
 * Name: can_raw_sendmsg
 *
 * Description:
 *   The can_raw_sendmsg() call may be used only when the packet socket is
 *   in a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      CAN frame and optional CMSG
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See sendmsg() for the complete list
 *   of return values.
 *
 ****************************************************************************/

static ssize_t can_raw_sendmsg(FAR struct socket *psock,
                               FAR const struct msghdr *msg, int flags)
{
  FAR struct net_driver_s *dev;
  FAR struct can_conn_s *conn;
  FAR struct iob_s *wb_iob;
  bool empty;
  bool nonblock;
  int ret = OK;

  /* Only SOCK_RAW is supported */

  if (psock->s_type != SOCK_RAW)
    {
      /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and
       * no peer address is set.
       */

      return -EDESTADDRREQ;
    }

  conn = psock->s_conn;

  /* Get the device driver that will service this transfer */

  dev = conn->dev;
  if (dev == NULL)
    {
      return -ENODEV;
    }

#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)
  if (_SO_GETOPT(conn->sconn.s_options, CAN_RAW_FD_FRAMES))
    {
      if (msg->msg_iov->iov_len != CANFD_MTU &&
          msg->msg_iov->iov_len != CAN_MTU)
        {
          return -EINVAL;
        }
    }
  else
#endif
    {
      if (msg->msg_iov->iov_len != CAN_MTU)
        {
          return -EINVAL;
        }
    }

  nonblock =
      _SS_ISNONBLOCK(conn->sconn.s_flags) || (flags & MSG_DONTWAIT) != 0;

  /* Perform the send operation */

  /* Initialize the state structure. This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  conn_dev_lock(&conn->sconn, dev);

#if CONFIG_NET_SEND_BUFSIZE > 0
  /* Wait until the frame fits.  The semaphore can be posted while there is
   * still no room, so check again after every wake-up.  A frame is always
   * accepted into an empty queue, even if it is larger than sndbufs.
   */

  while (!IOB_QEMPTY(&conn->write_q) &&
         (iob_get_queue_size(&conn->write_q) + msg->msg_iov->iov_len) >
         conn->sndbufs)
    {
      /* send buffer size exceeds the send limit */

      if (nonblock)
        {
          nerr("ERROR: Buffer overflow\n");
          ret = -EAGAIN;
          goto errout_with_lock;
        }

      ret = conn_dev_sem_timedwait(&conn->sndsem, false,
                                   _SO_TIMEOUT(conn->sconn.s_sndtimeo),
                                   &conn->sconn, dev);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }
#endif

  if (nonblock)
    {
      wb_iob = can_iob_timedalloc(0);
    }
  else
    {
      wb_iob = can_iob_timedalloc(_SO_TIMEOUT(conn->sconn.s_sndtimeo));
    }

  if (wb_iob == NULL)
    {
      /* A buffer allocation error occurred */

      nerr("ERROR: Failed to allocate write buffer\n");

      if (nonblock)
        {
          ret = -EAGAIN;
        }
      else
        {
          ret = -ENOMEM;
        }

      goto errout_with_lock;
    }

  iob_reserve(wb_iob, CONFIG_NET_LL_GUARDSIZE);
  iob_update_pktlen(wb_iob, 0, false);

  wb_iob->io_conn = &conn->sconn;

  /* Copy the user data into the write buffer.  We cannot wait for
   * buffer space if the socket was opened non-blocking.
   */

  if (nonblock)
    {
      ret = iob_trycopyin(wb_iob, (FAR uint8_t *)msg->msg_iov->iov_base,
                          msg->msg_iov->iov_len, 0, false);
    }
  else
    {
      /* iob_copyin might wait for buffers to be freed, but if
       * network is locked this might never happen, since network
       * driver is also locked, therefore we need to break the lock
       */

      conn_dev_unlock(&conn->sconn, dev);
      ret = iob_copyin(wb_iob, (FAR uint8_t *)msg->msg_iov->iov_base,
                       msg->msg_iov->iov_len, 0, false);
      conn_dev_lock(&conn->sconn, dev);
    }

  if (ret < 0)
    {
      nerr("ERROR: Failed to copy data into write buffer\n");
      goto errout_with_wrb;
    }

  /* Check if the write queue is empty */

  empty = (iob_get_queue_entry_count(&conn->write_q) == 0);

  /* Add the write buffer to the write queue */

  if (nonblock)
    {
      ret = iob_tryadd_queue(wb_iob, &conn->write_q);
    }
  else
    {
      ret = iob_add_queue(wb_iob, &conn->write_q);
    }

  if (ret < 0)
    {
      nerr("ERROR: Failed to add buffer to w_queue\n");
      goto errout_with_wrb;
    }

  /* The new write buffer lies at the head of the write queue.
   * Set up for the next packet transfer.
   */

  if (empty)
    {
      /* Allocate resource to receive a callback */

      if (conn->sndcb == NULL)
        {
          conn->sndcb = can_callback_alloc(dev, conn);
        }

      /* Test if the callback has been allocated */

      if (conn->sndcb == NULL)
        {
          /* A buffer allocation error occurred */

          nerr("ERROR: Failed to allocate callback\n");
          ret = -ENOMEM;
          goto errout_with_wq;
        }

      /* Set up the callback in the connection */

      conn->sndcb->flags = CAN_POLL;
      conn->sndcb->priv  = (FAR void *)conn;
      conn->sndcb->event = psock_send_eventhandler;

      /* Notify the device driver that new TX data is available. */

      netdev_txnotify_dev(dev, CAN_POLL);
    }

  /* unlock */

  conn_dev_unlock(&conn->sconn, dev);

  return msg->msg_iov->iov_len;

errout_with_wrb:
  iob_free_chain(wb_iob);

errout_with_wq:
  iob_free_queue(&conn->write_q);

errout_with_lock:
  conn_dev_unlock(&conn->sconn, dev);
  return ret;
}

/****************************************************************************
 * Name: can_raw_cansend
 *
 * Description:
 *   can_raw_cansend() returns a value indicating if a write to the socket
 *   would block.  No space in the buffer is actually reserved, so it is
 *   possible that the write may still block if the buffer is filled by
 *   another means.
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *
 * Returned Value:
 *   OK
 *     At least one byte of data could be successfully written.
 *   -EWOULDBLOCK
 *     There is no room in the output buffer.
 *   -EBADF
 *     An invalid descriptor was specified.
 *
 ****************************************************************************/

static int can_raw_cansend(FAR struct socket *psock)
{
#if CONFIG_NET_SEND_BUFSIZE > 0
  FAR struct can_conn_s *conn;
#endif

  /* Verify that we received a valid socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      nerr("ERROR: Invalid socket\n");
      return -EBADF;
    }

  /* In order to setup the send, we need to have at least
   * one free IOB to initialize the write buffer head.
   */

#if CONFIG_NET_SEND_BUFSIZE > 0
  conn = psock->s_conn;
#endif

  if (can_iob_navail() <= 0
#if CONFIG_NET_SEND_BUFSIZE > 0
      || iob_get_queue_size(&conn->write_q) >= conn->sndbufs
#endif
     )
    {
      return -EWOULDBLOCK;
    }

  return OK;
}

#endif

#if defined(CONFIG_NET_SOCKOPTS) && defined(CONFIG_NET_CANPROTO_OPTIONS)
/****************************************************************************
 * Name: can_raw_getsockopt
 *
 * Description:
 *   can_raw_getsockopt() retrieves the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the 'value'.
 *
 *   See <sys/socket.h> a complete list of values for the socket-level
 *   'option' argument.  Protocol-specific options are are protocol specific
 *   header files (such as nuttx/can.h for the case of the CAN protocol).
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_getsockopt() for
 *   the complete list of appropriate return error codes.
 *
 ****************************************************************************/

static int can_raw_getsockopt(FAR struct socket *psock, int level,
                              int option, FAR void *value,
                              FAR socklen_t *value_len)
{
  FAR struct can_conn_s *conn;
  int ret = OK;

  DEBUGASSERT(value != NULL && value_len != NULL);
  conn = psock->s_conn;

  /* SO_RCVBUF and SO_SNDBUF are socket-level options that are kept in
   * the CAN connection; psock_socketlevel_option() handles the others.
   */

  if (level == SOL_SOCKET)
    {
      if (option != SO_RCVBUF && option != SO_SNDBUF)
        {
          return -ENOPROTOOPT;
        }
    }
  else if (level != SOL_CAN_RAW)
    {
      return -ENOPROTOOPT;
    }

  if (psock->s_type != SOCK_RAW)
    {
      nerr("ERROR:  Not a RAW CAN socket\n");
      return -ENOTCONN;
    }

  switch (option)
    {
      case CAN_RAW_FILTER:
        if (*value_len % sizeof(struct can_filter) != 0)
          {
            ret = -EINVAL;
          }
        else if (*value_len > CONFIG_NET_CAN_RAW_FILTER_MAX *
                 sizeof(struct can_filter))
          {
            ret = -EINVAL;
          }
        else
          {
            int count = conn->filter_count;
            int i;

            if (*value_len < count * sizeof(struct can_filter))
              {
                count = *value_len / sizeof(struct can_filter);
              }
            else
              {
                *value_len = count * sizeof(struct can_filter);
              }

            for (i = 0; i < count; i++)
              {
                ((FAR struct can_filter *)value)[i] = conn->filters[i];
              }
          }
        break;

#ifdef CONFIG_NET_CAN_ERRORS
      case CAN_RAW_ERR_FILTER:
        if (*value_len < sizeof(can_err_mask_t))
          {
            return -EINVAL;
          }
        else
          {
            FAR can_err_mask_t *mask = (FAR can_err_mask_t *)value;

            *mask = conn->err_mask;
            *value_len = sizeof(can_err_mask_t);
          }
        break;
#endif

      case CAN_RAW_LOOPBACK:
      case CAN_RAW_RECV_OWN_MSGS:
#ifdef CONFIG_NET_CAN_CANFD
      case CAN_RAW_FD_FRAMES:
#endif
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
      case CAN_RAW_TX_DEADLINE:
#endif
        /* Verify that option is the size of an 'int'.  Should also check
         * that 'value' is properly aligned for an 'int'
         */

        if (*value_len < sizeof(int))
          {
            return -EINVAL;
          }

        /* Sample the current options.  This is atomic operation and so
         * should not require any special steps for thread safety. We
         * this outside of the macro because you can never be sure what
         * a macro will do.
         */

        *(FAR int *)value = _SO_GETOPT(conn->sconn.s_options, option);
        *value_len        = sizeof(int);
        break;

#if CONFIG_NET_RECV_BUFSIZE > 0
      case SO_RCVBUF:
        /* Verify that option is the size of an 'int'.  Should also check
         * that 'value' is properly aligned for an 'int'
         */

        if (*value_len != sizeof(int))
          {
            return -EINVAL;
          }

        *(FAR int *)value = conn->rcvbufs;

        break;
#endif

#if CONFIG_NET_SEND_BUFSIZE > 0
      case SO_SNDBUF:
        /* Verify that option is the size of an 'int'.  Should also check
         * that 'value' is properly aligned for an 'int'
         */

        if (*value_len < sizeof(int))
          {
            return -EINVAL;
          }

        *(FAR int *)value = conn->sndbufs;
        break;
#endif

      default:
        nerr("ERROR: Unrecognized RAW CAN socket option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: can_raw_setsockopt
 *
 * Description:
 *   can_raw_setsockopt() sets the CAN-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <nuttx/can.h> for the a complete list of values of CAN protocol
 *   options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_setcockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

static int can_raw_setsockopt(FAR struct socket *psock, int level,
                              int option, FAR const void *value,
                              socklen_t value_len)
{
  FAR struct can_conn_s *conn;
  int ret = OK;
  int count = 0;

  DEBUGASSERT(value_len == 0 || value != NULL);

  conn = psock->s_conn;

  /* SO_RCVBUF and SO_SNDBUF are socket-level options that are kept in
   * the CAN connection; psock_socketlevel_option() handles the others.
   */

  if (level == SOL_SOCKET)
    {
      if (option != SO_RCVBUF && option != SO_SNDBUF)
        {
          return -ENOPROTOOPT;
        }
    }
  else if (level != SOL_CAN_RAW)
    {
      return -ENOPROTOOPT;
    }

  if (psock->s_type != SOCK_RAW)
    {
      nerr("ERROR:  Not a RAW CAN socket\n");
      return -ENOTCONN;
    }

  switch (option)
    {
      case CAN_RAW_FILTER:
        if (value_len == 0)
          {
            conn->filter_count = 0;
            ret = OK;
          }
        else if (value_len % sizeof(struct can_filter) != 0)
          {
            ret = -EINVAL;
          }
        else if (value_len > CONFIG_NET_CAN_RAW_FILTER_MAX *
                   sizeof(struct can_filter))
          {
            ret = -EINVAL;
          }
        else
          {
            int i;

            count = value_len / sizeof(struct can_filter);

            for (i = 0; i < count; i++)
              {
                conn->filters[i] = ((struct can_filter *)value)[i];
              }

            conn->filter_count = count;

            ret = OK;
          }
        break;

#ifdef CONFIG_NET_CAN_ERRORS
      case CAN_RAW_ERR_FILTER:
        if (value_len != sizeof(can_err_mask_t))
          {
            return -EINVAL;
          }

        conn->err_mask = *(FAR can_err_mask_t *)value;
        break;
#endif

      case CAN_RAW_LOOPBACK:
      case CAN_RAW_RECV_OWN_MSGS:
#ifdef CONFIG_NET_CAN_CANFD
      case CAN_RAW_FD_FRAMES:
#endif
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
      case CAN_RAW_TX_DEADLINE:
#endif
        /* Verify that option is the size of an 'int'.  Should also check
         * that 'value' is properly aligned for an 'int'
         */

        if (value_len != sizeof(int))
          {
            return -EINVAL;
          }

        /* Lock the network so that we have exclusive access to the socket
         * options.
         */

        conn_lock(&conn->sconn);

        /* Set or clear the option bit */

        if (*(FAR int *)value)
          {
            _SO_SETOPT(conn->sconn.s_options, option);
          }
        else
          {
            _SO_CLROPT(conn->sconn.s_options, option);
          }

        conn_unlock(&conn->sconn);
        break;

#if CONFIG_NET_RECV_BUFSIZE > 0
      case SO_RCVBUF:
        {
          int buffersize;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (value_len != sizeof(int))
            {
              return -EINVAL;
            }

          /* Get the value.  Is the option being set or cleared? */

          buffersize = *(FAR int *)value;
          if (buffersize < 0)
            {
              return -EINVAL;
            }

#if CONFIG_NET_MAX_RECV_BUFSIZE > 0
          buffersize = MIN(buffersize, CONFIG_NET_MAX_RECV_BUFSIZE);
#endif
          conn->rcvbufs = buffersize;

          break;
        }
#endif

#if CONFIG_NET_SEND_BUFSIZE > 0
      case SO_SNDBUF:
        {
          int buffersize;

          /* Verify options */

          if (value_len != sizeof(int))
            {
              return -EINVAL;
            }

          buffersize = *(FAR int *)value;
          if (buffersize < 0)
            {
              return -EINVAL;
            }

#   if CONFIG_NET_MAX_SEND_BUFSIZE > 0
          /* Limit the size of the send buffer */

          buffersize = MIN(buffersize, CONFIG_NET_MAX_SEND_BUFSIZE);
#   endif

          conn->sndbufs = buffersize;
          break;
        }
#endif

      default:
        nerr("ERROR: Unrecognized CAN option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

#endif

/****************************************************************************
 * Name: can_raw_poll_eventhandler
 *
 * Description:
 *   This function is called to perform the actual CAN receive operation
 *   via the device interface layer. from can_input()
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   pvpriv   An instance of struct can_poll_s cast to void*
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

static uint32_t can_raw_poll_eventhandler(FAR struct net_driver_s *dev,
                                          FAR void *pvpriv, uint32_t flags)
{
  FAR struct can_poll_s *info = pvpriv;

  DEBUGASSERT(!info || (info->psock && info->fds));

  /* 'priv' might be null in some race conditions (?) */

  if (info)
    {
      pollevent_t eventset = 0;

      /* Check for data or connection availability events. */

      if ((flags & CAN_NEWDATA) != 0)
        {
          eventset |= POLLIN;
        }

      /* Check for loss of connection events. */

      if ((flags & NETDEV_DOWN) != 0)
        {
          eventset |= (POLLHUP | POLLERR);
        }

      /* A poll is a sign that we are free to send data. */

      else if ((flags & CAN_POLL) != 0 &&
                 can_raw_cansend(info->psock) >= 0)
        {
          eventset |= POLLOUT;
        }

      /* Awaken the caller of poll() is requested event occurred. */

      poll_notify(&info->fds, 1, eventset);
    }

  return flags;
}

/****************************************************************************
 * Name: can_raw_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    - A pointer to a user allocated socket structure to be
 *              initialized.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int can_raw_setup(FAR struct socket *psock)
{
  int domain = psock->s_domain;
  int type = psock->s_type;
  int proto = psock->s_proto;

  /* Verify that the protocol is supported */

  DEBUGASSERT((unsigned int)proto <= UINT8_MAX);

  switch (proto)
    {
      case 0:            /* INET subsystem for netlib_ifup */
      case CAN_RAW:      /* RAW sockets */
      case CAN_ISOTP:    /* ISO 15765-2 Transport Protocol */
        break;

      default:
        return -EPROTONOSUPPORT;
    }

  /* Verify the socket type (domain should always be PF_CAN here) */

  if (domain == PF_CAN && (type == SOCK_RAW || type == SOCK_DGRAM))
    {
      /* Allocate the CAN socket connection structure and save it in the
       * new socket instance.
       */

      FAR struct can_conn_s *conn = can_raw_alloc();

      if (conn == NULL)
        {
          /* Failed to reserve a connection structure */

          return -ENOMEM;
        }

#ifdef CONFIG_NET_CANPROTO_OPTIONS

      /* By default loopback is enabled */

      _SO_SETOPT(conn->sconn.s_options, CAN_RAW_LOOPBACK);
#endif

      /* Attach the connection instance to the socket */

      psock->s_conn = conn;
      return OK;
    }

  return -EPROTONOSUPPORT;
}

/****************************************************************************
 * Name: can_raw_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The non-negative set of socket capabilities is returned.
 *
 ****************************************************************************/

static sockcaps_t can_raw_sockcaps(FAR struct socket *psock)
{
  /* Permit vfcntl to set socket to non-blocking */

  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: can_raw_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_raw_addref(FAR struct socket *psock)
{
  FAR struct can_conn_s *conn;

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: can_raw_bind
 *
 * Description:
 *   can_raw_bind() gives the socket 'conn' the local address 'addr'. 'addr'
 *   is 'addrlen' bytes long. Traditionally, this is called "assigning a name
 *   to a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   conn     CAN socket connection structure
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     psock is a descriptor for a file, not a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int can_raw_bind(FAR struct socket *psock,
                        FAR const struct sockaddr *addr, socklen_t addrlen)
{
  return can_conn_bind(psock->s_conn, addr, addrlen);
}

/****************************************************************************
 * Name: can_raw_getsockname
 *
 * Description:
 *   The can_raw_getsockname() function retrieves the locally-bound name of
 *   the specified CAN socket, stores this address in the sockaddr structure
 *   pointed to by the 'addr' argument, and stores the length of this
 *   address in the object pointed to by the 'addrlen' argument.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to be queried
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int can_raw_getsockname(FAR struct socket *psock,
                               FAR struct sockaddr *addr,
                               FAR socklen_t *addrlen)
{
  FAR struct sockaddr_can *canaddr;
  FAR struct can_conn_s *conn;

  /* Check if enough space has been provided for the full address */

  if (*addrlen < sizeof(struct sockaddr_can))
    {
      /* This function is supposed to return the partial address if
       * a smaller buffer has been provided.  This support has not
       * been implemented.
       */

      return -EINVAL;
    }

  conn = psock->s_conn;

  /* Return the address information in the address structure */

  canaddr = (FAR struct sockaddr_can *)addr;
  memset(canaddr, 0, sizeof(struct sockaddr_can));

  canaddr->can_family = AF_CAN;
  canaddr->can_ifindex = can_get_ifindex(conn);

  *addrlen = sizeof(struct sockaddr_can);
  return OK;
}

/****************************************************************************
 * Name: can_raw_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 *     POLLUP:  Will never be reported
 *     POLLERR: Reported in the event of any failure.
 *     POLLOUT: Always reported if requested.
 *     POLLIN:  Reported if requested but only when pending response data is
 *              available
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored.
 *   setup - true: Setup up the poll; false: Tear down the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int can_raw_poll(FAR struct socket *psock,
                        FAR struct pollfd_s *fds, bool setup)
{
  FAR struct can_conn_s *conn;
  FAR struct can_poll_s *info = NULL;
  FAR struct devif_callback_s *cb;
  pollevent_t eventset = 0;
  int ret = OK;

  conn = psock->s_conn;

  /* FIXME add NETDEV_DOWN support */

  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      int i;

      conn_dev_lock(&conn->sconn, conn->dev);

      for (i = 0; i < CONFIG_NET_CAN_NPOLLWAITERS; i++)
        {
          if (conn->pollinfo[i].fds == NULL)
            {
              info = &conn->pollinfo[i];
              break;
            }
        }

      if (info == NULL)
        {
          ret = -EBUSY;
          goto errout_with_lock;
        }

      info->dev = conn->dev;

      cb = can_callback_alloc(info->dev, conn);
      if (cb == NULL)
        {
          ret = -EBUSY;
          goto errout_with_lock;
        }

      /* Initialize the poll info container */

      info->psock = psock;
      info->fds   = fds;
      info->cb    = cb;

      /* Initialize the callback structure.  Save the reference to the info
       * structure as callback private data so that it will be available
       * during callback processing.
       */

      cb->flags = NETDEV_DOWN;
      cb->priv  = info;
      cb->event = can_raw_poll_eventhandler;

      if ((fds->events & POLLOUT) != 0)
        {
          cb->flags |= CAN_POLL;
        }

      if ((fds->events & POLLIN) != 0)
        {
          cb->flags |= CAN_NEWDATA;
        }

      /* Save the reference in the poll info structure as fds private as well
       * for use during poll teardown as well.
       */

      fds->priv = info;

      /* Check for read data availability now */

      if (!IOB_QEMPTY(&conn->readahead))
        {
          /* Normal data may be read without blocking. */

          eventset |= POLLRDNORM;
        }

      if (can_raw_cansend(psock) >= 0)
        {
          /* A CAN frame may be sent without blocking. */

          eventset |= POLLWRNORM;
        }

      /* Check if any requested events are already in effect */

      poll_notify(&fds, 1, eventset);

errout_with_lock:
      conn_dev_unlock(&conn->sconn, conn->dev);
    }
  else
    {
      info = (FAR struct can_poll_s *)fds->priv;

      if (info != NULL)
        {
          /* Cancel any response notifications */

          conn_dev_lock(&conn->sconn, info->dev);
          can_callback_free(info->dev, conn, info->cb);

          /* Release the poll/select data slot */

          info->fds->priv = NULL;
          info->fds = NULL;

          /* Then free the poll info container */

          info->psock = NULL;
          conn_dev_unlock(&conn->sconn, info->dev);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: can_raw_close
 *
 * Description:
 *   Performs the close operation on a CAN socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int can_raw_close(FAR struct socket *psock)
{
  FAR struct can_conn_s *conn = psock->s_conn;
  int ret = OK;

  /* Perform some pre-close operations for the CAN socket type. */

  /* Is this the last reference to the connection structure (there
   * could be more if the socket was dup'ed).
   */

  if (conn->crefs <= 1)
    {
      /* Yes... inform user-space daemon of socket close. */

#ifdef CONFIG_NET_CAN_WRITE_BUFFERS
      /* Free write buffer callback. */

      if (conn->sndcb != NULL)
        {
          can_callback_free(conn->dev, conn, conn->sndcb);
          conn->sndcb = NULL;
        }
#endif

      /* Free the connection structure */

      conn->crefs = 0;
      can_raw_free(psock->s_conn);

      if (ret < 0)
        {
          /* Return with error code, but free resources. */

          nerr("ERROR: can_raw_close failed: %d\n", ret);
          return ret;
        }
    }
  else
    {
      /* No.. Just decrement the reference count */

      conn->crefs--;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_NET_CAN_WRITE_BUFFERS) && CONFIG_NET_SEND_BUFSIZE > 0
/****************************************************************************
 * Name: can_sendbuffer_notify
 *
 * Description:
 *   Notify the send buffer semaphore
 *
 * Input Parameters:
 *   conn - The CAN connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

void can_sendbuffer_notify(FAR struct can_conn_s *conn)
{
  int val = 0;

  nxsem_get_value(&conn->sndsem, &val);
  if (val < 1)
    {
      nxsem_post(&conn->sndsem);
    }
}

#endif

#endif /* CONFIG_NET_CAN */
