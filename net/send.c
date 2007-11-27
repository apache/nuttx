/****************************************************************************
 * net/send.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <sys/types.h>
#include <sys/socket.h>

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "net-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define STATE_POLLWAIT   1
#define STATE_DATA_SENT  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the interrupt level.
 */

struct send_s
{
  FAR struct socket *snd_sock;    /* Points to the parent socket structure */
  sem_t                snd_sem;     /* Used to wake up the waiting thread */
  FAR const uint8     *snd_buffer;  /* Points to the buffer of data to send */
  size_t               snd_buflen;  /* Number of bytes in the buffer to send */
  ssize_t              snd_sent;    /* The number of bytes sent */
  uint8                snd_state;   /* The state of the send operation. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: send_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   send operation when polled by the uIP layer.
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static uint8 send_interrupt(struct uip_driver_s *dev, struct uip_conn *conn, uint8 flags)
{
  struct send_s *pstate = (struct send_s *)conn->data_private;

  nvdbg("flags: %02x state: %d\n", flags, pstate->snd_state);

  /* If the data has not been sent OR if it needs to be retransmitted,
   * then send it now.
   */

  if (pstate->snd_state != STATE_DATA_SENT || (flags & UIP_REXMIT) != 0)
    {
      if (pstate->snd_buflen > uip_mss(conn))
        {
          uip_send(dev, pstate->snd_buffer, uip_mss(conn));
        }
      else
        {
          uip_send(dev, pstate->snd_buffer, pstate->snd_buflen);
        }

      pstate->snd_state = STATE_DATA_SENT;
    }

  /* Check if all data has been sent and acknowledged */

  else if (pstate->snd_state == STATE_DATA_SENT && (flags & UIP_ACKDATA) != 0)
    {
      /* Yes.. the data has been sent AND acknowledged */

      if (pstate->snd_buflen > uip_mss(conn))
        {
          /* Not all data has been sent */

          pstate->snd_sent   += uip_mss(conn);
          pstate->snd_buflen -= uip_mss(conn);
          pstate->snd_buffer += uip_mss(conn);

          /* Send again on the next poll */

          pstate->snd_state = STATE_POLLWAIT;
        }
      else
        {
          /* All data has been sent */

          pstate->snd_sent   += pstate->snd_buflen;
          pstate->snd_buffer += pstate->snd_buflen;
          pstate->snd_buflen  = 0;

          /* Don't allow any further call backs. */

          conn->data_flags   = 0;
          conn->data_private = NULL;
          conn->data_event   = NULL;

          /* Wake up the waiting thread, returning the number of bytes
           * actually sent.
           */

          sem_post(&pstate->snd_sem);
        }
    }

 /* Check for a loss of connection */

  else if ((flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
    {
      /* Stop further callbacks */

      conn->data_flags   = 0;
      conn->data_private = NULL;
      conn->data_event   = NULL;

      /* Report not connected */

      pstate->snd_sent = -ENOTCONN;

      /* Wake up the waiting thread */

      sem_post(&pstate->snd_sem);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: send
 *
 * Description:
 *   The send() call may be used only when the socket is in a connected state
 *   (so that the intended recipient is known). The only difference between
 *   send() and write() is the presence of flags. With zero flags parameter,
 *   send() is equivalent to write(). Also, send(sockfd,buf,len,flags) is
 *   equivalent to sendto(sockfd,buf,len,flags,NULL,0).
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
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
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t send(int sockfd, const void *buf, size_t len, int flags)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  struct uip_conn *conn;
  struct send_s state;
  irqstate_t save;
  int err;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* If this is a connected socket, then return ENOTCONN */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      err = ENOTCONN;
      goto errout;
    }

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Perform the TCP send operation */

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save                = irqsave();
  memset(&state, 0, sizeof(struct send_s));
  (void)sem_init(&state. snd_sem, 0, 0); /* Doesn't really fail */
  state.snd_sock      = psock;
  state.snd_buflen    = len;
  state.snd_buffer    = buf;
  state.snd_state     = STATE_POLLWAIT;

  if (len > 0)
    {
      /* Set up the callback in the connection */

      conn               = (struct uip_conn *)psock->s_conn;
      conn->data_flags   = UIP_REXMIT|UIP_ACKDATA|UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT;
      conn->data_private = (void*)&state;
      conn->data_event   = send_interrupt;

      /* Notify the device driver of the availaibilty of TX data */

      netdev_txnotify(&conn->ripaddr);

      /* Wait for the send to complete or an error to occur:  NOTES: (1)
       * sem_wait will also terminate if a signal is received, (2) interrupts
       * are disabled!  They will be re-enabled while the task sleeps and
       * automatically re-enabled when the task restarts.
       */

      ret = sem_wait(&state. snd_sem);

      /* Make sure that no further interrupts are processed */

      conn->data_flags   = 0;
      conn->data_private = NULL;
      conn->data_event   = NULL;
    }

  sem_destroy(&state. snd_sem);
  irqrestore(save);

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for a errors.  Errors are signaled by negative errno values
   * for the send length
   */

  if (state.snd_sent < 0)
    {
      err = state.snd_sent;
      goto errout;
    }

  /* If sem_wait failed, then we were probably reawakened by a signal. In
   * this case, sem_wait will have set errno appropriately.
   */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  /* Return the number of bytes actually sent */

  return state.snd_sent;

errout:
  *get_errno_ptr() = err;
  return ERROR;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
