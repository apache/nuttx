/****************************************************************************
 * net/ieee802154/ieee802154_sendto.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "ieee802154/ieee802154.h"

#ifdef CONFIG_NET_IEEE802154

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the interrupt level.
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
 * Name: psock_send_interrupt
 ****************************************************************************/

static uint16_t psock_send_interrupt(FAR struct net_driver_s *dev,
                                     FAR void *pvconn,
                                     FAR void *pvpriv, uint16_t flags)
{
  FAR struct send_s *pstate;
  FAR struct radio_driver_s *radio;

  DEBUGASSERT(pvpriv != NULL && dev != NULL && pvconn != NULL);

  /* Ignore polls from non IEEE 802.15.4 network drivers */

  if (dev->d_lltype != NET_LL_IEEE802154)
    {
      return flags;
    }

  /* Make sure that this is the driver to which the socket is connected. */
#warning Missing logic

  pstate = (FAR struct send_s *)pvpriv;
  radio  = (FAR struct radio_driver_s *)dev;

  ninfo("flags: %04x sent: %d\n", flags, pstate->snd_sent);

  if (pstate)
    {
      /* Check if the outgoing packet is available. It may have been claimed
       * by a send interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data. In these cases
       * we will just have to wait for the next polling cycle.
       */

      if (radio->r_dev.d_sndlen > 0 || (flags & PKT_NEWDATA) != 0)
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
#warning Missing logic
          //devif_ieee802154_send(radio, pstate->snd_buffer, pstate->snd_buflen);
          pstate->snd_sent = pstate->snd_buflen;

          /* Make sure no ARP request overwrites this ARP request.  This
           * flag will be cleared in arp_out().
           */

          IFF_SET_NOARP(radio->r_dev.d_flags);
        }

      /* Don't allow any further call backs. */

      pstate->snd_cb->flags    = 0;
      pstate->snd_cb->priv     = NULL;
      pstate->snd_cb->event    = NULL;

      /* Wake up the waiting thread */

      sem_post(&pstate->snd_sem);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_ieee802154_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.  See sendto()
 *   for the complete list of return values.
 *
 ****************************************************************************/

ssize_t psock_ieee802154_sendto(FAR struct socket *psock, FAR const void *buf,
                                size_t len, int flags,
                                FAR const struct sockaddr *to, socklen_t tolen)
{
  FAR struct radio_driver_s *radio;
  FAR struct ieee802154_conn_s *conn;
  struct send_s state;
  int errcode;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      errcode = EBADF;
      goto errout;
    }

  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Get the device driver that will service this transfer */

  radio = ieee802154_find_device(conn, &conn->laddr);
  if (radio == NULL)
    {
      errcode = ENODEV;
      goto errout;
    }

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Perform the send operation */

  /* Initialize the state structure. This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  net_lock();
  memset(&state, 0, sizeof(struct send_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  (void)sem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */
  (void)sem_setprotocol(&state.snd_sem, SEM_PRIO_NONE);

  state.snd_sock      = psock;          /* Socket descriptor to use */
  state.snd_buflen    = len;            /* Number of bytes to send */
  state.snd_buffer    = buf;            /* Buffer to send from */

  if (len > 0)
    {
      /* Allocate resource to receive a callback */

      state.snd_cb = ieee802154_callback_alloc(&radio->r_dev, conn);
      if (state.snd_cb)
        {
          /* Set up the callback in the connection */

          state.snd_cb->flags = PKT_POLL;
          state.snd_cb->priv  = (FAR void *)&state;
          state.snd_cb->event = psock_send_interrupt;

          /* Notify the device driver that new TX data is available. */

          netdev_txnotify_dev(&radio->r_dev);

          /* Wait for the send to complete or an error to occur: NOTES: (1)
           * net_lockedwait will also terminate if a signal is received, (2)
           * interrupts may be disabled! They will be re-enabled while the
           * task sleeps and automatically re-enabled when the task restarts.
           */

          ret = net_lockedwait(&state.snd_sem);

          /* Make sure that no further interrupts are processed */

          ieee802154_callback_free(&radio->r_dev, conn, state.snd_cb);
        }
    }

  sem_destroy(&state.snd_sem);
  net_unlock();

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for a errors, Errors are signalled by negative errno values
   * for the send length
   */

  if (state.snd_sent < 0)
    {
      errcode = state.snd_sent;
      goto errout;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal. In
   * this case, net_lockedwait will have set errno appropriately.
   */

  if (ret < 0)
    {
      errcode = -ret;
      goto errout;
    }

  /* Return the number of bytes actually sent */

  return state.snd_sent;

errout:
  set_errno(errcode);
  return ERROR;
}

#endif /* CONFIG_NET_IEEE802154 */
