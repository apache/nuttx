/****************************************************************************
 * net/bluetooth/bluetooth_recvmsg.c
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <netpacket/bluetooth.h>
#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bluetooth_recvfrom_s
{
  FAR struct socket *ir_sock;          /* Points to the parent socket structure */
  FAR struct devif_callback_s *ir_cb;  /* Reference to callback instance */
  FAR struct sockaddr *ir_from;        /* Location to return the from address */
  FAR uint8_t *ir_buffer;              /* Pointer to receive buffer */
  size_t ir_buflen;                    /* Length of receive buffer */
  sem_t ir_sem;                        /* Semaphore signals recv completion */
  ssize_t ir_result;                   /* Success:size, failure:negated errno */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_count_frames
 *
 * Description:
 *   Return the number of frames in the RX queue.
 *
 * Input Parameters:
 *   conn   - The socket connection structure.
 *
 * Returned Value:
 *   The number of frames in the queue.
 *
 ****************************************************************************/

#if CONFIG_NET_BLUETOOTH_BACKLOG > 0
static int bluetooth_count_frames(FAR struct bluetooth_conn_s *conn)
{
  FAR struct bluetooth_container_s *container;
  int count;

  for (count = 0, container = conn->bc_rxhead;
       container != NULL;
       count++, container = container->bn_flink)
    {
    }

  return count;
}
#endif

/****************************************************************************
 * Name: bluetooth_recvfrom_sender
 *
 * Description:
 *   Perform the reception operation if there are any queued frames in the
 *   RX frame queue.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static ssize_t
  bluetooth_recvfrom_rxqueue(FAR struct radio_driver_s *radio,
                             FAR struct bluetooth_recvfrom_s *pstate)
{
  FAR struct bluetooth_container_s *container;
  FAR struct sockaddr_l2 *iaddr;
  FAR struct bluetooth_conn_s *conn;
  FAR struct iob_s *iob;
  size_t copylen;
  int ret = -EAGAIN;

  /* Check if there is anything in in the RX input queue */

  DEBUGASSERT(pstate != NULL && pstate->ir_sock != NULL);
  conn = (FAR struct bluetooth_conn_s *)pstate->ir_sock->s_conn;
  DEBUGASSERT(conn != NULL);

  if (conn->bc_rxhead != NULL)
    {
      /* Remove the container from the RX input queue. */

      container           = conn->bc_rxhead;
      DEBUGASSERT(container != NULL);
      conn->bc_rxhead     = container->bn_flink;
      container->bn_flink = NULL;

      /* Did the RX queue become empty? */

      if (conn->bc_rxhead == NULL)
        {
          conn->bc_rxtail = NULL;
        }

#if CONFIG_NET_BLUETOOTH_BACKLOG > 0
      /* Decrement the count of frames in the queue. */

      DEBUGASSERT(conn->bc_backlog > 0);
      conn->bc_backlog--;
      DEBUGASSERT((int)conn->bc_backlog == bluetooth_count_frames(conn));
#endif

      /* Extract the IOB containing the frame from the container */

      iob               = container->bn_iob;
      container->bn_iob = NULL;
      DEBUGASSERT(iob != NULL);

      /* Copy the new packet data into the user buffer */

      copylen = iob->io_len - iob->io_offset;
      memcpy(pstate->ir_buffer, &iob->io_data[iob->io_offset], copylen);

      ninfo("Received %d bytes\n", (int)copylen);
      ret = copylen;

      /* If a 'from' address pointer was supplied, copy the source address
       * in the container there.
       */

      if (pstate->ir_from != NULL)
        {
          iaddr             = (FAR struct sockaddr_l2 *)pstate->ir_from;
          iaddr->l2_family  = AF_BLUETOOTH;
          BLUETOOTH_ADDRCOPY(&iaddr->l2_bdaddr, &container->bn_raddr);
          iaddr->l2_cid = container->bn_channel;
        }

      /* Free both the IOB and the container */

      iob_free(iob, IOBUSER_NET_SOCK_BLUETOOTH);
      bluetooth_container_free(container);
    }

  return ret;
}

/****************************************************************************
 * Name: bluetooth_recvfrom_eventhandler
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t bluetooth_recvfrom_eventhandler(FAR struct net_driver_s *dev,
                                                 FAR void *pvconn,
                                                 FAR void *pvpriv,
                                                 uint16_t flags)
{
  FAR struct bluetooth_recvfrom_s *pstate;
  FAR struct radio_driver_s *radio;
  ssize_t ret;

  ninfo("flags: %04x\n", flags);

  DEBUGASSERT(pvpriv != NULL && dev != NULL && pvconn != NULL);

  /* Ignore polls from non Bluetooth network drivers */

  if (dev->d_lltype != NET_LL_BLUETOOTH)
    {
      return flags;
    }

  /* Make sure that this is the driver to which the socket is bound. */

#warning Missing logic

  pstate = (FAR struct bluetooth_recvfrom_s *)pvpriv;
  radio  = (FAR struct radio_driver_s *)dev;

  /* 'pstate' might be null in some race conditions (?) */

  if (pstate != NULL)
    {
      /* If a new packet is available, then complete the read action. */

      if ((flags & BLUETOOTH_NEWDATA) != 0)
        {
          /* Attempt to receive the frame */

          ret = bluetooth_recvfrom_rxqueue(radio, pstate);
          if (ret > 0)
            {
              /* Don't allow any further call backs. */

              pstate->ir_cb->flags   = 0;
              pstate->ir_cb->priv    = NULL;
              pstate->ir_cb->event   = NULL;
              pstate->ir_result      = ret;

              /* indicate that the data has been consumed */

              flags &= ~BLUETOOTH_NEWDATA;

              /* Wake up the waiting thread, returning the number of bytes
               * actually read.
               */

              nxsem_post(&pstate->ir_sem);
            }
        }
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_recvmsg
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   and AF_INET6 address families.  bluetooth_recvmsg() receives messages
 *   from a socket, and may be used to receive data on a socket whether or
 *   not it is connection-oriented.
 *
 *   If msg_name is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument 'msg_namelen' is
 *   initialized to the size of the buffer associated with msg_name, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive data
 *   flags    Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received. If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0. Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

ssize_t bluetooth_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                          int flags)
{
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR struct sockaddr *from = msg->msg_name;
  FAR socklen_t *fromlen = &msg->msg_namelen;
  FAR struct bluetooth_conn_s *conn =
    (FAR struct bluetooth_conn_s *)psock->s_conn;
  FAR struct radio_driver_s *radio;
  struct bluetooth_recvfrom_s state;
  ssize_t ret;

  /* If a 'from' address has been provided, verify that it is large
   * enough to hold this address family.
   */

  if (from != NULL && *fromlen < sizeof(struct sockaddr_l2))
    {
      return -EINVAL;
    }

  if (psock->s_type != SOCK_RAW)
    {
      nerr("ERROR: Unsupported socket type: %d\n", psock->s_type);
      return -EPROTONOSUPPORT;
    }

  /* Perform the packet recvmsg() operation */

  /* Initialize the state structure.  This is done with the network
   * locked because we don't want anything to happen until we are ready.
   */

  net_lock();
  memset(&state, 0, sizeof(struct bluetooth_recvfrom_s));

  state.ir_buflen = len;
  state.ir_buffer = buf;
  state.ir_sock   = psock;
  state.ir_from   = from;

  /* Get the device driver that will service this transfer */

  radio = bluetooth_find_device(conn, &conn->bc_laddr);
  if (radio == NULL)
    {
      ret = -ENODEV;
      goto errout_with_lock;
    }

  /* Before we wait for data, let's check if there are already frame(s)
   * waiting in the RX queue.
   */

  ret = bluetooth_recvfrom_rxqueue(radio, &state);
  if (ret > 0)
    {
      /* Good newe!  We have a frame and we are done. */

      net_unlock();
      return ret;
    }

  /* We will have to wait.  This semaphore is used for signaling and,
   * hence, should not have priority inheritance enabled.
   */

  nxsem_init(&state.ir_sem, 0, 0); /* Doesn't really fail */
  nxsem_set_protocol(&state.ir_sem, SEM_PRIO_NONE);

  /* Set up the callback in the connection */

  state.ir_cb = bluetooth_callback_alloc(&radio->r_dev, conn);
  if (state.ir_cb)
    {
      state.ir_cb->flags  = (BLUETOOTH_NEWDATA | BLUETOOTH_POLL);
      state.ir_cb->priv   = (FAR void *)&state;
      state.ir_cb->event  = bluetooth_recvfrom_eventhandler;

      /* Wait for either the receive to complete or for an error/timeout to
       * occur. NOTES:  (1) net_lockedwait will also terminate if a signal
       * is received, (2) the network is locked!  It will be un-locked while
       * the task sleeps and automatically re-locked when the task restarts.
       */

      net_lockedwait(&state.ir_sem);

      /* Make sure that no further events are processed */

      bluetooth_callback_free(&radio->r_dev, conn, state.ir_cb);
      ret = state.ir_result;
    }
  else
    {
      ret = -EBUSY;
    }

  nxsem_destroy(&state.ir_sem);

errout_with_lock:
  net_unlock();
  return ret;
}

#endif /* CONFIG_NET_BLUETOOTH */
