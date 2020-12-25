/****************************************************************************
 * net/can/can_send.c
 *
 *   Copyright (C) 2014, 2016-2017 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_NET_CMSG
#include <sys/time.h>
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon by the event handler.
 */

struct send_s
{
  FAR struct socket      *snd_sock;    /* Points to the parent socket structure */
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  sem_t                   snd_sem;     /* Used to wake up the waiting thread */
  FAR const uint8_t      *snd_buffer;  /* Points to the buffer of data to send */
  size_t                  snd_buflen;  /* Number of bytes in the buffer to send */
#ifdef CONFIG_NET_CMSG
  size_t                  pr_msglen;   /* Length of msg buffer */
  FAR uint8_t            *pr_msgbuf;   /* Pointer to msg buffer */
#endif
  ssize_t                 snd_sent;    /* The number of bytes sent */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_send_eventhandler
 ****************************************************************************/

static uint16_t psock_send_eventhandler(FAR struct net_driver_s *dev,
                                        FAR void *pvconn,
                                        FAR void *pvpriv, uint16_t flags)
{
  FAR struct send_s *pstate = (FAR struct send_s *)pvpriv;

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

          devif_can_send(dev, pstate->snd_buffer, pstate->snd_buflen);
          pstate->snd_sent = pstate->snd_buflen;
#ifdef CONFIG_NET_CMSG
          if (pstate->pr_msglen > 0) /* concat cmsg data after packet */
            {
              memcpy(dev->d_buf + pstate->snd_buflen, pstate->pr_msgbuf,
                      pstate->pr_msglen);
              dev->d_sndlen = pstate->snd_buflen + pstate->pr_msglen;
            }
#endif
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
 * Name: psock_can_send
 *
 * Description:
 *   The psock_can_send() call may be used only when the packet socket is in
 *   a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See send() for the complete list
 *   of return values.
 *
 ****************************************************************************/

ssize_t psock_can_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len)
{
  FAR struct net_driver_s *dev;
  FAR struct can_conn_s *conn;
  struct send_s state;
  int ret = OK;

  conn = (FAR struct can_conn_s *)psock->s_conn;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Get the device driver that will service this transfer */

  dev = conn->dev;
  if (dev == NULL)
    {
      return -ENODEV;
    }
#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)

  if (conn->fd_frames)
    {
      if (len != CANFD_MTU && len != CAN_MTU)
        {
          return -EINVAL;
        }
    }
#endif
  else
    {
      if (len != CAN_MTU)
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

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */
  nxsem_set_protocol(&state.snd_sem, SEM_PRIO_NONE);

  state.snd_sock      = psock;          /* Socket descriptor to use */
  state.snd_buflen    = len;            /* Number of bytes to send */
  state.snd_buffer    = buf;            /* Buffer to send from */

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
       * net_lockedwait will also terminate if a signal is received.
       */

      ret = net_lockedwait(&state.snd_sem);

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

  /* If net_lockedwait failed, then we were probably reawakened by a signal.
   * In this case, net_lockedwait will have returned negated errno
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
 * Name: psock_can_sendmsg
 *
 * Description:
 *   The psock_can_sendmsg() call may be used only when the packet socket is
 *   in a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      msg to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See send() for the complete list
 *   of return values.
 *
 ****************************************************************************/

ssize_t psock_can_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg)
{
  FAR struct net_driver_s *dev;
  FAR struct can_conn_s *conn;
  struct send_s state;
  int ret = OK;

  conn = (FAR struct can_conn_s *)psock->s_conn;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

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

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */
  nxsem_set_protocol(&state.snd_sem, SEM_PRIO_NONE);

  state.snd_sock      = psock;                  /* Socket descriptor */
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
       * net_lockedwait will also terminate if a signal is received.
       */

      ret = net_lockedwait(&state.snd_sem);

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

  /* If net_lockedwait failed, then we were probably reawakened by a signal.
   * In this case, net_lockedwait will have returned negated errno
   * appropriately.
   */

  if (ret < 0)
    {
      return ret;
    }

  /* Return the number of bytes actually sent */

  return state.snd_sent;
}

#endif /* CONFIG_NET && CONFIG_NET_CAN */
