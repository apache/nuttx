/****************************************************************************
 * net/bluetooth/bluetooth_sendto.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <netpacket/bluetooth.h>
#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

#include "utils/utils.h"
#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the event handler.
 */

struct bluetooth_sendto_s
{
  FAR struct socket *is_sock;           /* Points to the parent socket structure */
  FAR struct devif_callback_s *is_cb;   /* Reference to callback instance */
  bt_addr_t is_destaddr;                /* Frame destination address */
  uint8_t is_channel;                   /* Frame destination channel */
  sem_t is_sem;                         /* Used to wake up the waiting thread */
  FAR const uint8_t *is_buffer;         /* User buffer of data to send */
  size_t is_buflen;                     /* Number of bytes in the is_buffer */
  ssize_t is_sent;                      /* The number of bytes sent (or error) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_sendto_eventhandler
 ****************************************************************************/

static uint16_t bluetooth_sendto_eventhandler(FAR struct net_driver_s *dev,
                                               FAR void *pvconn,
                                               FAR void *pvpriv,
                                               uint16_t flags)
{
  FAR struct radio_driver_s *radio;
  FAR struct bluetooth_sendto_s *pstate;
  struct bluetooth_frame_meta_s meta;
  FAR struct iob_s *iob;
  int hdrlen;
  int ret;

  DEBUGASSERT(pvpriv != NULL && dev != NULL && pvconn != NULL);

  /* Ignore polls from non Bluetooth network drivers */

  if (dev->d_lltype != NET_LL_BLUETOOTH)
    {
      return flags;
    }

  /* Make sure that this is the driver to which the socket is connected. */
#warning Missing logic

  pstate = (FAR struct bluetooth_sendto_s *)pvpriv;
  radio  = (FAR struct radio_driver_s *)dev;

  ninfo("flags: %04x sent: %d\n", flags, pstate->is_sent);

  if (pstate != NULL && (flags & BLUETOOTH_POLL) != 0)
    {
      /* Initialize the meta data */

      BLUETOOTH_ADDRCOPY(&meta.bm_raddr, &pstate->is_destaddr);
      meta.bm_channel = pstate->is_channel;

      /* Get the Bluetooth MAC header length */

      hdrlen = radio->r_get_mhrlen(radio, &meta);
      if (hdrlen < 0)
        {
          nerr("ERROR: Failed to get header length: %d\n", hdrlen);
          ret = hdrlen;
          goto errout;
        }

      /* Verify that the user buffer can fit within the frame with this
       * MAC header.
       */

      DEBUGASSERT(BLUETOOTH_MAX_FRAMELEN <= CONFIG_IOB_BUFSIZE);
      if (pstate->is_buflen + hdrlen > BLUETOOTH_MAX_FRAMELEN)
        {
          nerr("ERROR: User buffer will not fit into the frame: %u > %u\n",
               (unsigned int)(pstate->is_buflen + hdrlen),
               (unsigned int)CONFIG_IOB_BUFSIZE);
          ret = -E2BIG;
          goto errout;
        }

      /* Allocate an IOB to hold the frame data */

      iob = net_ioballoc(false);
      if (iob == NULL)
        {
          nwarn("WARNING: Failed to allocate IOB\n");
          return flags;
        }

      /* Initialize the IOB */

      iob->io_offset = hdrlen;
      iob->io_len    = pstate->is_buflen + hdrlen;
      iob->io_pktlen = pstate->is_buflen + hdrlen;

      /* Copy the user data into the IOB */

      memcpy(&iob->io_data[hdrlen], pstate->is_buffer, pstate->is_buflen);

      /* And submit the IOB to the network driver */

      ret = radio->r_req_data(radio, &meta, iob);
      if (ret < 0)
        {
          nerr("ERROR: r_req_data() failed: %d\n", ret);
          goto errout;
        }

      /* Save the successful result */

      pstate->is_sent = pstate->is_buflen;

      /* Don't allow any further call backs. */

      pstate->is_cb->flags    = 0;
      pstate->is_cb->priv     = NULL;
      pstate->is_cb->event    = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->is_sem);
    }

  return flags;

errout:
  /* Don't allow any further call backs. */

  pstate->is_cb->flags    = 0;
  pstate->is_cb->priv     = NULL;
  pstate->is_cb->event    = NULL;
  pstate->is_sent         = ret;

  /* Wake up the waiting thread */

  nxsem_post(&pstate->is_sem);
  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_bluetooth_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See sendto() for the complete list
 *   of return values.
 *
 ****************************************************************************/

ssize_t psock_bluetooth_sendto(FAR struct socket *psock, FAR const void *buf,
                                size_t len, int flags,
                                FAR const struct sockaddr *to, socklen_t tolen)
{
  FAR struct sockaddr_bt_s *destaddr;
  FAR struct radio_driver_s *radio;
  FAR struct bluetooth_conn_s *conn;
  struct bluetooth_sendto_s state;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  conn = (FAR struct bluetooth_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Verify that the address is large enough to be a valid PF_BLUETOOTH
   * address.
   */

  if (tolen < sizeof(bt_addr_t))
    {
      return -EDESTADDRREQ;
    }

  /* Get the device driver that will service this transfer */

  radio = bluetooth_find_device(conn, &conn->bc_laddr);
  if (radio == NULL)
    {
      return -ENODEV;
    }

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Perform the send operation */

  /* Initialize the state structure. This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  net_lock();
  memset(&state, 0, sizeof(struct bluetooth_sendto_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  (void)nxsem_init(&state.is_sem, 0, 0); /* Doesn't really fail */
  (void)nxsem_setprotocol(&state.is_sem, SEM_PRIO_NONE);

  state.is_sock   = psock;          /* Socket descriptor to use */
  state.is_buflen = len;            /* Number of bytes to send */
  state.is_buffer = buf;            /* Buffer to send from */

  /* Copy the destination address */

  destaddr = (FAR struct sockaddr_bt_s *)to;
  memcpy(&state.is_destaddr, &destaddr->bt_bdaddr,
         sizeof(bt_addr_t));

  if (len > 0)
    {
      /* Allocate resource to receive a callback */

      state.is_cb = bluetooth_callback_alloc(&radio->r_dev, conn);
      if (state.is_cb)
        {
          /* Set up the callback in the connection */

          state.is_cb->flags = PKT_POLL;
          state.is_cb->priv  = (FAR void *)&state;
          state.is_cb->event = bluetooth_sendto_eventhandler;

          /* Notify the device driver that new TX data is available. */

          netdev_txnotify_dev(&radio->r_dev);

          /* Wait for the send to complete or an error to occur.
           * net_lockedwait will also terminate if a signal is received.
           */

          ret = net_lockedwait(&state.is_sem);

          /* Make sure that no further events are processed */

          bluetooth_callback_free(&radio->r_dev, conn, state.is_cb);
        }
    }

  nxsem_destroy(&state.is_sem);
  net_unlock();

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for a errors, Errors are signaled by negative errno values
   * for the send length
   */

  if (state.is_sent < 0)
    {
      return state.is_sent;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal. In
   * this case, net_lockedwait will have returned negated errno appropriately.
   */

  if (ret < 0)
    {
      return ret;
    }

  /* Return the number of bytes actually sent */

  return state.is_sent;
}

#endif /* CONFIG_NET_BLUETOOTH */
