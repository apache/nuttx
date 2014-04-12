/****************************************************************************
 * net/net_sendfile.c
 *
 *   Copyright (C) 2013 UVC Ingenieure. All rights reserved.
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Max Holtzberg <mh@uvc.de>
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

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <fcntl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#include "net_internal.h"
#include "uip/uip_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#if defined(CONFIG_NET_TCP_SPLIT) && !defined(CONFIG_NET_TCP_SPLIT_SIZE)
#  define CONFIG_NET_TCP_SPLIT_SIZE 40
#endif

#define TCPBUF ((struct uip_tcpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the interrupt level.
 */

struct sendfile_s
{
  FAR struct socket         *snd_sock;    /* Points to the parent socket structure */
  FAR struct uip_callback_s *snd_datacb;  /* Data callback */
  FAR struct uip_callback_s *snd_ackcb;   /* ACK callback */
  FAR struct file           *snd_file;    /* File structure of the input file */
  sem_t                      snd_sem;     /* Used to wake up the waiting thread */
  off_t                      snd_foffset; /* Input file offset */
  size_t                     snd_flen;    /* File length */
  ssize_t                    snd_sent;    /* The number of bytes sent */
  uint32_t                   snd_isn;     /* Initial sequence number */
  uint32_t                   snd_acked;   /* The number of bytes acked */
#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
  uint32_t                   snd_time;    /* Last send time for determining timeout */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sendfile_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Parameters:
 *   pstate   send state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
static inline int sendfile_timeout(FAR struct sendfile_s *pstate)
{
  FAR struct socket *psock = 0;

  /* Check for a timeout configured via setsockopts(SO_SNDTIMEO).
   * If none... we well let the send wait forever.
   */

  psock = pstate->snd_sock;
  if (psock && psock->s_sndtimeo != 0)
    {
      /* Check if the configured timeout has elapsed */

      return net_timeo(pstate->snd_time, psock->s_sndtimeo);
    }

  /* No timeout */

  return FALSE;
}
#endif /* CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */

static uint16_t ack_interrupt(FAR struct uip_driver_s *dev, FAR void *pvconn,
                              FAR void *pvpriv, uint16_t flags)
{
  FAR struct sendfile_s *pstate = (FAR struct sendfile_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  if ((flags & UIP_ACKDATA) != 0)
    {
      /* Update the timeout */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
      pstate->snd_time = clock_systimer();
#endif

      /* The current acknowledgement number number is the (relative) offset
       * of the of the next byte needed by the receiver.  The snd_isn is the
       * offset of the first byte to send to the receiver.  The difference
       * is the number of bytes to be acknowledged.
       */

      pstate->snd_acked = uip_tcpgetsequence(TCPBUF->ackno) - pstate->snd_isn;
      nllvdbg("ACK: acked=%d sent=%d flen=%d\n",
             pstate->snd_acked, pstate->snd_sent, pstate->snd_flen);

      dev->d_sndlen = 0;

      flags &= ~UIP_ACKDATA;
    }
  else if ((flags & UIP_REXMIT) != 0)
    {
      nlldbg("REXMIT\n");

      /* Yes.. in this case, reset the number of bytes that have been sent
       * to the number of bytes that have been ACKed.
       */

      pstate->snd_sent = pstate->snd_acked;
    }

  /* Check for a loss of connection */

  else if ((flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
    {
      /* Report not connected */

      nlldbg("Lost connection\n");

      net_lostconnection(pstate->snd_sock, flags);
      pstate->snd_sent = -ENOTCONN;
    }

  /* Wake up the waiting thread */

  sem_post(&pstate->snd_sem);

  return flags;
}

/****************************************************************************
 * Function: sendfile_interrupt
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

static uint16_t sendfile_interrupt(FAR struct uip_driver_s *dev, FAR void *pvconn,
                                   FAR void *pvpriv, uint16_t flags)
{
  FAR struct uip_conn *conn = (FAR struct uip_conn*)pvconn;
  FAR struct sendfile_s *pstate = (FAR struct sendfile_s *)pvpriv;
  int ret;

  nllvdbg("flags: %04x acked: %d sent: %d\n",
          flags, pstate->snd_acked, pstate->snd_sent);

  /* Check for a loss of connection */

  if ((flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
    {
      /* Report not connected */

      nlldbg("Lost connection\n");

      net_lostconnection(pstate->snd_sock, flags);
      pstate->snd_sent = -ENOTCONN;
      goto end_wait;
    }

  /* We get here if (1) not all of the data has been ACKed, (2) we have been
   * asked to retransmit data, (3) the connection is still healthy, and (4)
   * the outgoing packet is available for our use.  In this case, we are
   * now free to send more data to receiver -- UNLESS the buffer contains
   * unprocessing incoming data.  In that event, we will have to wait for the
   * next polling cycle.
   */

  if ((flags & UIP_NEWDATA) == 0 && pstate->snd_sent < pstate->snd_flen)
    {
      /* Get the amount of data that we can send in the next packet */

      uint32_t sndlen = pstate->snd_flen - pstate->snd_sent;

      if (sndlen > uip_mss(conn))
        {
          sndlen = uip_mss(conn);
        }

      /* Check if we have "space" in the window */

      if ((pstate->snd_sent - pstate->snd_acked + sndlen) < conn->winsize)
        {
          uint32_t seqno;

          /* Then set-up to send that amount of data. (this won't actually
           * happen until the polling cycle completes).
           */

          ret = file_seek(pstate->snd_file,
                          pstate->snd_foffset + pstate->snd_sent, SEEK_SET);
          if (ret < 0)
            {
              int errcode = errno;
              nlldbg("failed to lseek: %d\n", errcode);
              pstate->snd_sent = -errcode;
              goto end_wait;
            }

          ret = file_read(pstate->snd_file, dev->d_snddata, sndlen);
          if (ret < 0)
            {
              int errcode = errno;
              nlldbg("failed to read from input file: %d\n", errcode);
              pstate->snd_sent = -errcode;
              goto end_wait;
            }

          dev->d_sndlen = sndlen;

          /* Set the sequence number for this packet.  NOTE:  uIP updates
           * sndseq on recept of ACK *before* this function is called.  In that
           * case sndseq will point to the next unacknowledge byte (which might
           * have already been sent).  We will overwrite the value of sndseq
           * here before the packet is sent.
           */

          seqno = pstate->snd_sent + pstate->snd_isn;
          nllvdbg("SEND: sndseq %08x->%08x len: %d\n", conn->sndseq, seqno, ret);

          uip_tcpsetsequence(conn->sndseq, seqno);

          /* Check if the destination IP address is in the ARP table.  If not,
           * then the send won't actually make it out... it will be replaced with
           * an ARP request.
           *
           * NOTE 1: This could be an expensive check if there are a lot of entries
           * in the ARP table.  Hence, we only check on the first packet -- when
           * snd_sent is zero.
           *
           * NOTE 2: If we are actually harvesting IP addresses on incoming IP
           * packets, then this check should not be necessary; the MAC mapping
           * should already be in the ARP table.
           */

#if defined(CONFIG_NET_ETHERNET) && !defined (CONFIG_NET_ARP_IPIN)
          if (pstate->snd_sent != 0 || uip_arp_find(conn->ripaddr) != NULL)
#endif
            {
              /* Update the amount of data sent (but not necessarily ACKed) */

              pstate->snd_sent += sndlen;
              nllvdbg("pid: %d SEND: acked=%d sent=%d flen=%d\n", getpid(),
                     pstate->snd_acked, pstate->snd_sent, pstate->snd_flen);
            }
        }
      else
        {
          nlldbg("Window full, wait for ack\n");
          goto wait;
        }
    }

  /* All data has been send and we are just waiting for ACK or re-transmit
   * indications to complete the send.  Check for a timeout.
   */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
  if (sendfile_timeout(pstate))
    {
      /* Yes.. report the timeout */

      nlldbg("SEND timeout\n");
      pstate->snd_sent = -ETIMEDOUT;
      goto end_wait;
    }
#endif /* CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */

  if (pstate->snd_sent >= pstate->snd_flen
      && pstate->snd_acked < pstate->snd_flen)
    {
      /* All data has been sent, but there are outstanding ACK's */

      goto wait;
    }

end_wait:

  /* Do not allow any further callbacks */

  pstate->snd_datacb->flags   = 0;
  pstate->snd_datacb->priv    = NULL;
  pstate->snd_datacb->event   = NULL;

  /* Wake up the waiting thread */

  sem_post(&pstate->snd_sem);

wait:
  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_sendfile
 *
 * Description:
 *   The send() call may be used only when the socket is in a connected state
 *   (so that the intended recipient is known). The only difference between
 *   send() and write() is the presence of flags. With zero flags parameter,
 *   send() is equivalent to write(). Also, send(sockfd,buf,len,flags) is
 *   equivalent to sendto(sockfd,buf,len,flags,NULL,0).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
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

ssize_t net_sendfile(int outfd, struct file *infile, off_t *offset,
                     size_t count)
{
  FAR struct socket *psock = sockfd_socket(outfd);
  FAR struct uip_conn *conn = (FAR struct uip_conn*)psock->s_conn;
  struct sendfile_s state;
  uip_lock_t save;
  int err;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* If this is an un-connected socket, then return ENOTCONN */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      err = ENOTCONN;
      goto errout;
    }

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save  = uip_lock();

  memset(&state, 0, sizeof(struct sendfile_s));
  sem_init(&state. snd_sem, 0, 0);          /* Doesn't really fail */
  state.snd_sock    = psock;                /* Socket descriptor to use */
  state.snd_foffset = offset ? *offset : 0; /* Input file offset */
  state.snd_flen    = count;                /* Number of bytes to send */
  state.snd_file    = infile;               /* File to read from */

  /* Allocate resources to receive a callback */

  state.snd_datacb = uip_tcpcallbackalloc(conn);

  if (state.snd_datacb == NULL)
    {
      nlldbg("Failed to allocate data callback\n");
      err = ENOMEM;
      goto errout_locked;
    }

  state.snd_ackcb = uip_tcpcallbackalloc(conn);

  if (state.snd_ackcb == NULL)
    {
      nlldbg("Failed to allocate ack callback\n");
      err = ENOMEM;
      goto errout_datacb;
    }

  /* Get the initial sequence number that will be used */

  state.snd_isn          = uip_tcpgetsequence(conn->sndseq);

  /* There is no outstanding, unacknowledged data after this
   * initial sequence number.
   */

  conn->unacked          = 0;

  /* Set the initial time for calculating timeouts */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
  state.snd_time         = clock_systimer();
#endif

  /* Set up the ACK callback in the connection */

  state.snd_ackcb->flags = UIP_ACKDATA|UIP_REXMIT|UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT;
  state.snd_ackcb->priv  = (void*)&state;
  state.snd_ackcb->event = ack_interrupt;

  /* Perform the TCP send operation */

  do
    {
      state.snd_datacb->flags = UIP_POLL;
      state.snd_datacb->priv  = (void*)&state;
      state.snd_datacb->event = sendfile_interrupt;

      /* Notify the device driver of the availaibilty of TX data */

      netdev_txnotify(conn->ripaddr);

      uip_lockedwait(&state.snd_sem);
    }
  while (state.snd_sent >= 0 && state.snd_acked < state.snd_flen);

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  uip_tcpcallbackfree(conn, state.snd_ackcb);

 errout_datacb:
  uip_tcpcallbackfree(conn, state.snd_datacb);

 errout_locked:

  sem_destroy(&state. snd_sem);
  uip_unlock(save);

 errout:

  if (err)
    {
      set_errno(err);
      return ERROR;
    }
  else if (state.snd_sent < 0)
    {
      set_errno(-state.snd_sent);
      return ERROR;
    }
  else
    {
      return state.snd_sent;
    }
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
