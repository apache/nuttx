/****************************************************************************
 * net/tcp/tcp_sendfile.c
 *
 *   Copyright (C) 2013 UVC Ingenieure. All rights reserved.
 *   Copyright (C) 2007-2017 Gregory Nutt. All rights reserved.
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
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"
#include "neighbor/neighbor.h"
#include "socket/socket.h"
#include "tcp/tcp.h"

#if defined(CONFIG_NET_SENDFILE) && defined(CONFIG_NET_TCP) && \
    defined(NET_TCP_HAVE_STACK)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_NET_TCP_SPLIT) && !defined(CONFIG_NET_TCP_SPLIT_SIZE)
#  define CONFIG_NET_TCP_SPLIT_SIZE 40
#endif

#define TCPIPv4BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define TCPIPv6BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the driver poll event.
 */

struct sendfile_s
{
  FAR struct socket *snd_sock;             /* Points to the parent socket structure */
  FAR struct devif_callback_s *snd_datacb; /* Data callback */
  FAR struct devif_callback_s *snd_ackcb;  /* ACK callback */
  FAR struct file   *snd_file;             /* File structure of the input file */
  sem_t              snd_sem;              /* Used to wake up the waiting thread */
  off_t              snd_foffset;          /* Input file offset */
  size_t             snd_flen;             /* File length */
  ssize_t            snd_sent;             /* The number of bytes sent */
  uint32_t           snd_isn;              /* Initial sequence number */
  uint32_t           snd_acked;            /* The number of bytes acked */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t ack_eventhandler(FAR struct net_driver_s *dev,
                                 FAR void *pvconn,
                                 FAR void *pvpriv, uint16_t flags)
{
  FAR struct sendfile_s *pstate = (FAR struct sendfile_s *)pvpriv;

  ninfo("flags: %04x\n", flags);

  if ((flags & TCP_ACKDATA) != 0)
    {
      FAR struct tcp_hdr_s *tcp;

      /* Get the offset address of the TCP header */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      if (IFF_IS_IPv6(dev->d_flags))
#endif
        {
          DEBUGASSERT(pstate->snd_sock->s_domain == PF_INET6);
          tcp = TCPIPv6BUF;
        }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      else
#endif
        {
          DEBUGASSERT(pstate->snd_sock->s_domain == PF_INET);
          tcp = TCPIPv4BUF;
        }
#endif /* CONFIG_NET_IPv4 */

      /* The current acknowledgement number number is the (relative) offset
       * of the of the next byte needed by the receiver.  The snd_isn is the
       * offset of the first byte to send to the receiver.  The difference
       * is the number of bytes to be acknowledged.
       */

      pstate->snd_acked = tcp_getsequence(tcp->ackno) - pstate->snd_isn;
      ninfo("ACK: acked=%d sent=%d flen=%d\n",
            pstate->snd_acked, pstate->snd_sent, pstate->snd_flen);

      dev->d_sndlen = 0;

      flags &= ~TCP_ACKDATA;
    }
  else if ((flags & TCP_REXMIT) != 0)
    {
      nwarn("WARNING: TCP_REXMIT\n");

      /* Yes.. in this case, reset the number of bytes that have been sent
       * to the number of bytes that have been ACKed.
       */

      pstate->snd_sent = pstate->snd_acked;
    }

  /* Check for a loss of connection */

  else if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      FAR struct socket *psock = pstate->snd_sock;

      nwarn("WARNING: Lost connection\n");

      /* We could get here recursively through the callback actions of
       * tcp_lost_connection().  So don't repeat that action if we have
       * already been disconnected.
       */

      DEBUGASSERT(psock != NULL);
      if (_SS_ISCONNECTED(psock->s_flags))
        {
          /* Report not connected */

          tcp_lost_connection(psock, pstate->snd_ackcb, flags);
        }

      /* Report not connected */

      pstate->snd_sent = -ENOTCONN;
    }

  /* Prohibit further callbacks */

  pstate->snd_ackcb->flags = 0;
  pstate->snd_ackcb->priv  = NULL;
  pstate->snd_ackcb->event = NULL;

  /* Wake up the waiting thread */

  nxsem_post(&pstate->snd_sem);

  return flags;
}

/****************************************************************************
 * Name: sendfile_eventhandler
 *
 * Description:
 *   This function is called to perform the actual send operation when
 *   polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static uint16_t sendfile_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *pvconn, FAR void *pvpriv,
                                      uint16_t flags)
{
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pvconn;
  FAR struct sendfile_s *pstate = (FAR struct sendfile_s *)pvpriv;
  int ret;

  /* The TCP socket is connected and, hence, should be bound to a device.
   * Make sure that the polling device is the own that we are bound to.
   */

  DEBUGASSERT(conn->dev != NULL);
  if (dev != conn->dev)
    {
      return flags;
    }

  ninfo("flags: %04x acked: %d sent: %d\n",
        flags, pstate->snd_acked, pstate->snd_sent);

  /* Check for a loss of connection */

  if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      FAR struct socket *psock = pstate->snd_sock;

      nwarn("WARNING: Lost connection\n");

      /* We could get here recursively through the callback actions of
       * tcp_lost_connection().  So don't repeat that action if we have
       * already been disconnected.
       */

      DEBUGASSERT(psock != NULL);
      if (_SS_ISCONNECTED(psock->s_flags))
        {
          /* Report not connected */

          tcp_lost_connection(psock, pstate->snd_datacb, flags);
        }

      /* Report not connected */

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

  if ((flags & TCP_NEWDATA) == 0 && pstate->snd_sent < pstate->snd_flen)
    {
      /* Get the amount of data that we can send in the next packet */

      uint32_t sndlen = pstate->snd_flen - pstate->snd_sent;

      if (sndlen > conn->mss)
        {
          sndlen = conn->mss;
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
              nerr("ERROR: Failed to lseek: %d\n", ret);
              pstate->snd_sent = ret;
              goto end_wait;
            }

          ret = file_read(pstate->snd_file, dev->d_appdata, sndlen);
          if (ret < 0)
            {
              nerr("ERROR: Failed to read from input file: %d\n", (int)ret);
              pstate->snd_sent = ret;
              goto end_wait;
            }

          dev->d_sndlen = sndlen;

          /* Set the sequence number for this packet.  NOTE:  The network
           * updates sndseq on recept of ACK *before* this function is
           * called.  In that case sndseq will point to the next
           * unacknowledge byte (which might have already been sent).  We
           * will overwrite the value of sndseq here before the packet is
           * sent.
           */

          seqno = pstate->snd_sent + pstate->snd_isn;
          ninfo("SEND: sndseq %08x->%08x len: %d\n",
                conn->sndseq, seqno, ret);

          tcp_setsequence(conn->sndseq, seqno);

          /* Update the amount of data sent (but not necessarily ACKed) */

          pstate->snd_sent += sndlen;
          ninfo("pid: %d SEND: acked=%d sent=%d flen=%d\n", getpid(),
                pstate->snd_acked, pstate->snd_sent, pstate->snd_flen);
        }
      else
        {
          nwarn("WARNING: Window full, wait for ack\n");
          goto wait;
        }
    }

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

  nxsem_post(&pstate->snd_sem);

wait:
  return flags;
}

/****************************************************************************
 * Name: sendfile_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are have data ready to
 *   be send (TCP)
 *
 * Input Parameters:
 *   psock - Socket state structure
 *   conn  - The TCP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sendfile_txnotify(FAR struct socket *psock,
                                     FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  /* If both IPv4 and IPv6 support are enabled, then we will need to select
   * the device driver using the appropriate IP domain.
   */

  if (psock->s_domain == PF_INET)
#endif
    {
      /* Notify the device driver that send data is available */

      netdev_ipv4_txnotify(conn->u.ipv4.laddr, conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else /* if (psock->s_domain == PF_INET6) */
#endif /* CONFIG_NET_IPv4 */
    {
      /* Notify the device driver that send data is available */

      DEBUGASSERT(psock->s_domain == PF_INET6);
      netdev_ipv6_txnotify(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_IPv6 */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_sendfile
 *
 * Description:
 *   The tcp_sendfile() call may be used only when the INET socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See sendfile() for a list
 *   appropriate error return values.
 *
 ****************************************************************************/

ssize_t tcp_sendfile(FAR struct socket *psock, FAR struct file *infile,
                      FAR off_t *offset, size_t count)
{
  FAR struct tcp_conn_s *conn;
  struct sendfile_s state;
  int ret;

  /* If this is an un-connected socket, then return ENOTCONN */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      nerr("ERROR: Not connected\n");
      return -ENOTCONN;
    }

  /* Make sure that we have the IP address mapping */

  conn = (FAR struct tcp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

#if defined(CONFIG_NET_ARP_SEND) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
#ifdef CONFIG_NET_ARP_SEND
#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  if (psock->s_domain == PF_INET)
#endif
    {
      /* Make sure that the IP address mapping is in the ARP table */

      ret = arp_send(conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_ARP_SEND */
#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
#ifdef CONFIG_NET_ARP_SEND
  else
#endif
    {
      /* Make sure that the IP address mapping is in the Neighbor Table */

      ret = icmpv6_neighbor(conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Did we successfully get the address mapping? */

  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      return -ENETUNREACH;
    }
#endif /* CONFIG_NET_ARP_SEND || CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Initialize the state structure.  This is done with the network
   * locked because we don't want anything to happen until we are
   * ready.
   */

  net_lock();
  memset(&state, 0, sizeof(struct sendfile_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&state.snd_sem, 0, 0);           /* Doesn't really fail */
  nxsem_set_protocol(&state.snd_sem, SEM_PRIO_NONE);

  state.snd_sock    = psock;                /* Socket descriptor to use */
  state.snd_foffset = offset ? *offset : 0; /* Input file offset */
  state.snd_flen    = count;                /* Number of bytes to send */
  state.snd_file    = infile;               /* File to read from */

  /* Allocate resources to receive a callback */

  state.snd_datacb = tcp_callback_alloc(conn);

  if (state.snd_datacb == NULL)
    {
      nerr("ERROR: Failed to allocate data callback\n");
      ret = -ENOMEM;
      goto errout_locked;
    }

  state.snd_ackcb = tcp_callback_alloc(conn);

  if (state.snd_ackcb == NULL)
    {
      nerr("ERROR: Failed to allocate ack callback\n");
      ret = -ENOMEM;
      goto errout_datacb;
    }

  /* Get the initial sequence number that will be used */

  state.snd_isn          = tcp_getsequence(conn->sndseq);

  /* There is no outstanding, unacknowledged data after this
   * initial sequence number.
   */

  conn->tx_unacked       = 0;

  /* Set up the ACK callback in the connection */

  state.snd_ackcb->flags = (TCP_ACKDATA | TCP_REXMIT | TCP_DISCONN_EVENTS);
  state.snd_ackcb->priv  = (FAR void *)&state;
  state.snd_ackcb->event = ack_eventhandler;

  /* Perform the TCP send operation */

  state.snd_datacb->flags = TCP_POLL;
  state.snd_datacb->priv  = (FAR void *)&state;
  state.snd_datacb->event = sendfile_eventhandler;

  /* Notify the device driver of the availability of TX data */

  sendfile_txnotify(psock, conn);

  for (; ; )
    {
      uint32_t acked = state.snd_acked;

      ret = net_timedwait_uninterruptible(&state.snd_sem,
                                          _SO_TIMEOUT(psock->s_sndtimeo));
      if (ret != -ETIMEDOUT || acked == state.snd_acked)
        {
          break; /* Timeout without any progress */
        }
    }

  tcp_callback_free(conn, state.snd_ackcb);

errout_datacb:
  tcp_callback_free(conn, state.snd_datacb);

errout_locked:

  nxsem_destroy(&state.snd_sem);
  net_unlock();

  if (ret < 0)
    {
      return ret;
    }
  else
    {
      return state.snd_sent;
    }
}

#endif /* CONFIG_NET_SENDFILE && CONFIG_NET_TCP && NET_TCP_HAVE_STACK */
