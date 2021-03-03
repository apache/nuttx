/****************************************************************************
 * net/pkt/pkt_send.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

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
#include "pkt/pkt.h"

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

  ninfo("flags: %04x sent: %d\n", flags, pstate->snd_sent);

  if (pstate)
    {
      /* Check if the outgoing packet is available. It may have been claimed
       * by a send event handler serving a different thread -OR- if the
       * output buffer currently contains unprocessed incoming data. In
       * these cases we will just have to wait for the next polling cycle.
       */

      if (dev->d_sndlen > 0 || (flags & PKT_NEWDATA) != 0)
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

          devif_pkt_send(dev, pstate->snd_buffer, pstate->snd_buflen);
          pstate->snd_sent = pstate->snd_buflen;

          /* Make sure no ARP request overwrites this ARP request.  This
           * flag will be cleared in arp_out().
           */

          IFF_SET_NOARP(dev->d_flags);
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
 * Name: psock_pkt_send
 *
 * Description:
 *   The psock_pkt_send() call may be used only when the packet socket is in
 *   a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See send() for the complete list
 *   of return values.
 *
 ****************************************************************************/

ssize_t psock_pkt_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len)
{
  FAR struct net_driver_s *dev;
  struct send_s state;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  /* Get the device driver that will service this transfer */

  dev = pkt_find_device((FAR struct pkt_conn_s *)psock->s_conn);
  if (dev == NULL)
    {
      return -ENODEV;
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

  if (len > 0)
    {
      FAR struct pkt_conn_s *conn = (FAR struct pkt_conn_s *)psock->s_conn;

      /* Allocate resource to receive a callback */

      state.snd_cb = pkt_callback_alloc(dev, conn);
      if (state.snd_cb)
        {
          /* Set up the callback in the connection */

          state.snd_cb->flags = PKT_POLL;
          state.snd_cb->priv  = (FAR void *)&state;
          state.snd_cb->event = psock_send_eventhandler;

          /* Notify the device driver that new TX data is available. */

          netdev_txnotify_dev(dev);

          /* Wait for the send to complete or an error to occur.
           * net_lockedwait will also terminate if a signal is received.
           */

          ret = net_lockedwait(&state.snd_sem);

          /* Make sure that no further events are processed */

          pkt_callback_free(dev, conn, state.snd_cb);
        }
    }

  nxsem_destroy(&state.snd_sem);
  net_unlock();

  /* Check for errors.  Errors are signalled by negative errno values
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

#endif /* CONFIG_NET && CONFIG_NET_PKT */
