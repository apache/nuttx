/****************************************************************************
 * net/net_send_buffered.c
 *
 *   Copyright (C) 2007-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Jason Jiang  <jasonj@live.cn>
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

#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP) && \
    defined(CONFIG_NET_TCP_WRITE_BUFFERS)

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/clock.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#ifdef CONFIG_NET_ARP_IPIN
#  include <nuttx/net/uip/uip-arp.h>
#endif

#include "net_internal.h"
#include "uip/uip_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TCPBUF ((struct uip_tcpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: send_insert_seqment
 *
 * Description:
 *   Insert a new segment in a write buffer queue, keep the segment queue in
 *   ascending order of sequence number.
 *
 * Parameters:
 *   segment   The segment to be inserted
 *   q         The write buffer queue in which to insert the segment
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static void send_insert_seqment(FAR struct uip_wrbuffer_s *segment,
                                FAR sq_queue_t *q)
{
  sq_entry_t *entry = (sq_entry_t*)segment;
  sq_entry_t *insert = NULL;

  sq_entry_t *itr;
  for (itr = sq_peek(q); itr; itr = sq_next(itr))
    {
      FAR struct uip_wrbuffer_s *segment0 = (FAR struct uip_wrbuffer_s*)itr;
      if (segment0->wb_seqno < segment->wb_seqno)
        {
          insert = itr;
        }
      else
        {
          break;
        }
    }

  if (insert)
    {
      sq_addafter(insert, entry, q);
    }
  else
    {
      sq_addfirst(entry, q);
    }
}

/****************************************************************************
 * Function: send_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   send operation when polled by the uIP layer.
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
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

static uint16_t send_interrupt(FAR struct uip_driver_s *dev, FAR void *pvconn,
                               FAR void *pvpriv, uint16_t flags)
{
  FAR struct uip_conn *conn = (FAR struct uip_conn*)pvconn;
  FAR struct socket *psock = (FAR struct socket *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  /* If this packet contains an acknowledgement, then update the count of
   * acknowledged bytes.
   */

  if ((flags & UIP_ACKDATA) != 0)
    {
      FAR sq_entry_t *entry, *next;
      FAR struct uip_wrbuffer_s *segment;
      uint32_t ackno;

      ackno = uip_tcpgetsequence(TCPBUF->ackno);
      for (entry = sq_peek(&conn->unacked_q); entry; entry = next)
        {
          next    = sq_next(entry);
          segment = (FAR struct uip_wrbuffer_s*)entry;

          if (segment->wb_seqno < ackno)
            {
              nllvdbg("ACK: acked=%d buflen=%d ackno=%d\n",
                      segment->wb_seqno, segment->wb_nbytes, ackno);

              /* Segment was ACKed. Remove from ACK waiting queue */

              sq_rem(entry, &conn->unacked_q);

              /* Return the write buffer to the pool of free buffers */

              uip_tcpwrbuffer_release(segment);
            }
        }
    }

  /* Check for a loss of connection */

  else if ((flags & (UIP_CLOSE | UIP_ABORT | UIP_TIMEDOUT)) != 0)
    {
      /* Report not connected */

       nllvdbg("Lost connection\n");
       net_lostconnection(psock, flags);
       goto end_wait;
     }

   /* Check if we are being asked to retransmit data */

   else if ((flags & UIP_REXMIT) != 0)
    {
      sq_entry_t *entry;

      /* Put all segments that have been sent but not ACKed to write queue
       * again note, the un-ACKed segment is put at the first of the write_q,
       * so it can be sent as soon as possible.
       */

      while ((entry = sq_remlast(&conn->unacked_q)))
        {
          struct uip_wrbuffer_s *segment = (struct uip_wrbuffer_s*)entry;

          if (segment->wb_nrtx >= UIP_MAXRTX)
            {
              //conn->unacked -= segment->wb_nbytes;

              /* Return the write buffer */

              uip_tcpwrbuffer_release(segment);

              /* NOTE expired is different from un-ACKed, it is designed to
               * represent the number of segments that have been sent,
               * retransmitted, and un-ACKed, if expired is not zero, the
               * connection will be closed.
               *
               * field expired can only be updated at UIP_ESTABLISHED state
               */

              conn->expired++;
              continue;
            }

          send_insert_seqment(segment, &conn->write_q);
        }
    }

  /* Check if the outgoing packet is available (it may have been claimed
   * by a sendto interrupt serving a different thread).
   */

  if (dev->d_sndlen > 0)
    {
      /* Another thread has beat us sending data, wait for the next poll */

      return flags;
    }

  /* We get here if (1) not all of the data has been ACKed, (2) we have been
   * asked to retransmit data, (3) the connection is still healthy, and (4)
   * the outgoing packet is available for our use.  In this case, we are
   * now free to send more data to receiver -- UNLESS the buffer contains
   * unprocesed incoming data.  In that event, we will have to wait for the
   * next polling cycle.
   */

  if ((conn->tcpstateflags & UIP_ESTABLISHED) &&
      (flags & (UIP_POLL | UIP_REXMIT)) &&
      !(sq_empty(&conn->write_q)))
    {
      /* Check if the destination IP address is in the ARP table.  If not,
       * then the send won't actually make it out... it will be replaced with
       * an ARP request.
       *
       * NOTE 1: This could be an expensive check if there are a lot of
       * entries in the ARP table.
       *
       * NOTE 2: If we are actually harvesting IP addresses on incoming IP
       * packets, then this check should not be necessary; the MAC mapping
       * should already be in the ARP table.
       */

#if defined(CONFIG_NET_ETHERNET) && !defined(CONFIG_NET_ARP_IPIN)
      if (uip_arp_find(conn->ripaddr) != NULL)
#endif
        {
          FAR struct uip_wrbuffer_s *segment;
          FAR void *sndbuf;
          size_t sndlen;

          /* Get the amount of data that we can send in the next packet */

          segment = (FAR struct uip_wrbuffer_s *)sq_remfirst(&conn->write_q);
          if (segment)
            {
              sndbuf = segment->wb_buffer;
              sndlen = segment->wb_nbytes;

              DEBUGASSERT(sndlen <= uip_mss(conn));

              /* REVISIT:  There should be a check here to assure that we do
               * not excced the window (conn->winsize).
               */

              /* Set the sequence number for this segment.  NOTE: uIP
               * updates sndseq on receipt of ACK *before* this function
               * is called. In that case sndseq will point to the next
               * unacknowledged byte (which might have already been
               * sent). We will overwrite the value of sndseq here
               * before the packet is sent.
               */

              if (segment->wb_nrtx == 0 && segment->wb_seqno == (unsigned)-1)
                {
                  segment->wb_seqno = conn->isn + conn->sent;
                }

              uip_tcpsetsequence(conn->sndseq, segment->wb_seqno);

              /* Then set-up to send that amount of data. (this won't
               * actually happen until the polling cycle completes).
               */

              uip_send(dev, sndbuf, sndlen);

              /* Remember how much data we send out now so that we know
               * when everything has been acknowledged.  Just increment
               * the amount of data sent. This will be needed in
               * sequence* number calculations and we know that this is
               * not a re-transmission. Re-transmissions do not go through
               * this path.
               */

              if (segment->wb_nrtx == 0)
                {
                  conn->unacked += sndlen;
                  conn->sent    += sndlen;
                }

              /* Increment the retransmission counter before expiration.
               * NOTE we will not calculate the retransmission timer
               * (RTT) to save cpu cycles, each send_insert_seqment
               * segment will be retransmitted UIP_MAXRTX times in halt-
               * second interval before expiration.
               */

              segment->wb_nrtx++;

              /* The segment is waiting for ACK again */

              send_insert_seqment(segment, &conn->unacked_q);

              /* Only one data can be sent by low level driver at once,
               * tell the caller stop polling the other connection.
               */

              flags &= ~UIP_POLL;
            }
        }
    }

  /* Continue waiting */

  return flags;

end_wait:

  /* Do not allow any further callbacks */

  psock->s_sndcb->flags = 0;
  psock->s_sndcb->event = NULL;

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_send
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

ssize_t psock_send(FAR struct socket *psock, FAR const void *buf, size_t len,
                   int flags)
{
  uip_lock_t save;
  size_t     completed = 0;
  int        err;
  int        ret = OK;

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      err = ENOTCONN;
      goto errout;
    }

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  save = uip_lock();

  if (len > 0)
    {
      struct uip_conn *conn = (struct uip_conn*)psock->s_conn;

      if (!psock->s_sndcb)
        {
          psock->s_sndcb = uip_tcpcallbackalloc(conn);

          /* Set up the callback in the connection */

          psock->s_sndcb->flags = (UIP_ACKDATA | UIP_REXMIT |UIP_POLL | \
                                  UIP_CLOSE | UIP_ABORT | UIP_TIMEDOUT);
          psock->s_sndcb->priv  = (void*)psock;
          psock->s_sndcb->event = send_interrupt;
        }

      /* Allocate resources to receive a callback */

      while (completed < len)
        {
          FAR struct uip_wrbuffer_s *segment = uip_tcpwrbuffer_alloc(NULL);
          if (segment)
            {
              size_t cnt;

              segment->wb_seqno = (unsigned)-1;
              segment->wb_nrtx  = 0;

              if (len - completed > CONFIG_NET_TCP_WRITE_BUFSIZE)
                {
                  cnt = CONFIG_NET_TCP_WRITE_BUFSIZE;
                }
              else
                {
                  cnt = len - completed;
                }

              segment->wb_nbytes = cnt;
              memcpy(segment->wb_buffer, (char*)buf + completed, cnt);
              completed += cnt;

              /* send_interrupt() will refer to all the write buffer by
               * conn->writebuff
               */

              sq_addlast(&segment->wb_node, &conn->write_q);

              /* Notify the device driver of the availability of TX data */

              netdev_txnotify(conn->ripaddr);
            }
        }
    }

  uip_unlock(save);

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for a errors.  Errors are signaled by negative errno values
   * for the send length
   */

  if (completed < 0)
    {
      err = completed;
      goto errout;
    }

  /* If uip_lockedwait failed, then we were probably reawakened by a signal.
   * In this case, uip_lockedwait will have set errno appropriately.
   */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  /* Return the number of bytes actually sent */

  return completed;

errout:
  set_errno(err);
  return ERROR;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP && CONFIG_NET_TCP_WRITE_BUFFERS */
