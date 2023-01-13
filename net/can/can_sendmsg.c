/****************************************************************************
 * net/can/can_sendmsg.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_CAN)

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "can/can.h"

#include <sys/time.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon by the event handler.
 */

struct send_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  sem_t                   snd_sem;     /* Used to wake up the waiting thread */
  FAR const uint8_t      *snd_buffer;  /* Points to the buffer of data to send */
  size_t                  snd_buflen;  /* Number of bytes in the buffer to send */
  size_t                  pr_msglen;   /* Length of msg buffer */
  FAR uint8_t            *pr_msgbuf;   /* Pointer to msg buffer */
  ssize_t                 snd_sent;    /* The number of bytes sent */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_send_eventhandler
 ****************************************************************************/

static uint16_t psock_send_eventhandler(FAR struct net_driver_s *dev,
                                        FAR void *pvpriv, uint16_t flags)
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

          devif_send(dev, pstate->snd_buffer, pstate->snd_buflen, 0);
          if (dev->d_sndlen == 0)
            {
              return flags;
            }

          pstate->snd_sent = pstate->snd_buflen;
          if (pstate->pr_msglen > 0) /* concat cmsg data after packet */
            {
              memcpy(dev->d_buf + pstate->snd_buflen, pstate->pr_msgbuf,
                      pstate->pr_msglen);
              dev->d_sndlen = pstate->snd_buflen + pstate->pr_msglen;
            }
        }

      /* Don't allow any further call backs. */

      pstate->snd_cb->flags    = 0;
      pstate->snd_cb->priv     = NULL;
      pstate->snd_cb->event    = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->snd_sem);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_sendmsg
 *
 * Description:
 *   The can_sendmsg() call may be used only when the packet socket is
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

ssize_t can_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                    int flags)
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

  conn = (FAR struct can_conn_s *)psock->s_conn;

  /* Get the device driver that will service this transfer */

  dev = conn->dev;
  if (dev == NULL)
    {
      return -ENODEV;
    }

#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)
  if (conn->fd_frames)
    {
      if (msg->msg_iov->iov_len != CANFD_MTU
              && msg->msg_iov->iov_len != CAN_MTU)
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

  net_lock();
  memset(&state, 0, sizeof(struct send_s));
  nxsem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */

  state.snd_buflen    = msg->msg_iov->iov_len;  /* bytes to send */
  state.snd_buffer    = msg->msg_iov->iov_base; /* Buffer to send from */

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  if (msg->msg_controllen > sizeof(struct cmsghdr))
    {
      struct cmsghdr *cmsg = CMSG_FIRSTHDR(msg);
      if (conn->tx_deadline && cmsg->cmsg_level == SOL_CAN_RAW
              && cmsg->cmsg_type == CAN_RAW_TX_DEADLINE
              && cmsg->cmsg_len == sizeof(struct timeval))
        {
          state.pr_msgbuf     = CMSG_DATA(cmsg); /* Buffer to cmsg data */
          state.pr_msglen     = cmsg->cmsg_len;  /* len of cmsg data */
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

      netdev_txnotify_dev(dev);

      /* Wait for the send to complete or an error to occur.
       * net_sem_timedwait will also terminate if a signal is received.
       */

      if (_SS_ISNONBLOCK(conn->sconn.s_flags) || (flags & MSG_DONTWAIT) != 0)
        {
          ret = net_sem_timedwait(&state.snd_sem, 0);
        }
      else
        {
          ret = net_sem_timedwait(&state.snd_sem, UINT_MAX);
        }

      /* Make sure that no further events are processed */

      can_callback_free(dev, conn, state.snd_cb);
    }

  nxsem_destroy(&state.snd_sem);
  net_unlock();

  /* Check for a errors, Errors are signalled by negative errno values
   * for the send length
   */

  if (state.snd_sent < 0)
    {
      return state.snd_sent;
    }

  /* If net_sem_wait failed, then we were probably reawakened by a signal.
   * In this case, net_sem_wait will have returned negated errno
   * appropriately.
   */

  if (ret < 0)
    {
      return ret;
    }

  /* Return the number of bytes actually sent */

  return state.snd_sent;
}

/****************************************************************************
 * Name: psock_can_cansend
 *
 * Description:
 *   psock_can_cansend() returns a value indicating if a write to the socket
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

int psock_can_cansend(FAR struct socket *psock)
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

#endif /* CONFIG_NET && CONFIG_NET_CAN */
