/****************************************************************************
 * net/pkt/pkt_send.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Function: psock_send_interrupt
 ****************************************************************************/

static uint16_t psock_send_interrupt(FAR struct net_driver_s *dev,
                                     FAR void *pvconn,
                                     FAR void *pvpriv, uint16_t flags)
{
  FAR struct send_s *pstate = (FAR struct send_s *)pvpriv;

  nllvdbg("flags: %04x sent: %d\n", flags, pstate->snd_sent);

  if (pstate)
    {
      /* Check if the outgoing packet is available. It may have been claimed
       * by a send interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data. In these cases
       * we will just have to wait for the next polling cycle.
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

      sem_post(&pstate->snd_sem);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_pkt_send
 *
 * Description:
 *   The psock_pkt_send() call may be used only when the packet socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_pkt_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len)
{
  FAR struct net_driver_s *dev;
  struct send_s state;
  net_lock_t save;
  int err;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Get the device driver that will service this transfer */

  dev = pkt_find_device((FAR struct pkt_conn_s *)psock->s_conn);
  if (dev == NULL)
    {
      err = ENODEV;
      goto errout;
    }

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Perform the send operation */

  /* Initialize the state structure. This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save                = net_lock();
  memset(&state, 0, sizeof(struct send_s));
  (void)sem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */
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
          state.snd_cb->priv  = (void*)&state;
          state.snd_cb->event = psock_send_interrupt;

          /* Notify the device driver that new TX data is available. */

          netdev_txnotify_dev(dev);

          /* Wait for the send to complete or an error to occur: NOTES: (1)
           * net_lockedwait will also terminate if a signal is received, (2)
           * interrupts may be disabled! They will be re-enabled while the
           * task sleeps and automatically re-enabled when the task restarts.
           */

          ret = net_lockedwait(&state.snd_sem);

          /* Make sure that no further interrupts are processed */

          pkt_callback_free(dev, conn, state.snd_cb);
        }
    }

  sem_destroy(&state.snd_sem);
  net_unlock(save);

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for a errors, Errors are signalled by negative errno values
   * for the send length
   */

  if (state.snd_sent < 0)
    {
      err = state.snd_sent;
      goto errout;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal. In
   * this case, net_lockedwait will have set errno appropriately.
   */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  /* Return the number of bytes actually sent */

  return state.snd_sent;

errout:
  set_errno(err);
  return ERROR;
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
