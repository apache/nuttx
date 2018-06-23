/****************************************************************************
 * net/icmp/icmp_recvfrom.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/icmp.h>

#include "devif/devif.h"
#include "socket/socket.h"
#include "icmp/icmp.h"

#ifdef CONFIG_NET_ICMP_SOCKET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF  ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPBUF  ((struct icmp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define ICMPSIZE ((dev)->d_len - IPv4_HDRLEN)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct icmp_recvfrom_s
{
  FAR struct devif_callback_s *recv_cb; /* Reference to callback instance */
  FAR struct socket *recv_sock; /* IPPROTO_ICMP socket structure */
  sem_t recv_sem;               /* Use to manage the wait for the response */
  clock_t recv_time;            /* Start time for determining timeouts */
  in_addr_t recv_from;          /* The peer we received the request from */
  FAR uint8_t *recv_buf;        /* Location to return the response */
  uint16_t recv_buflen;         /* Size of the response */
  int16_t recv_result;          /* >=0: receive size on success;
                                 * <0:negated errno on fail */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: recvfrom_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Input Parameters:
 *   pstate - Reference to instance ot recvfrom state structure
 *
 * Returned Value:
 *   true: timeout false: no timeout
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SOCKOPTS
static inline int recvfrom_timeout(FAR struct icmp_recvfrom_s *pstate)
{
  FAR struct socket *psock;

  /* Check for a timeout configured via setsockopts(SO_SNDTIMEO).
   * If none... we will let the send wait forever.
   */

  psock = pstate->recv_sock;
  if (psock != NULL && psock->s_rcvtimeo != 0)
    {
      /* Check if the configured timeout has elapsed */

      return net_timeo(pstate->recv_time, psock->s_rcvtimeo);
    }

  /* No timeout */

  return false;
}
#endif /* CONFIG_NET_SOCKOPTS */

/****************************************************************************
 * Name: recvfrom_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   ECHO request and/or ECHO reply actions when polled by the lower, device
 *   interfacing layer.
 *
 * Input Parameters:
 *   dev        The structure of the network driver that generated the
 *              event.
 *   conn       The received packet, cast to (void *)
 *   pvpriv     An instance of struct icmp_recvfrom_s cast to void*
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t recvfrom_eventhandler(FAR struct net_driver_s *dev,
                                  FAR void *pvconn,
                                  FAR void *pvpriv, uint16_t flags)
{
  FAR struct icmp_recvfrom_s *pstate = (struct icmp_recvfrom_s *)pvpriv;
  FAR struct socket *psock;
  FAR struct icmp_conn_s *conn;
  FAR struct ipv4_hdr_s *ipv4;
  FAR struct icmp_hdr_s *icmp;

  ninfo("flags: %04x\n", flags);

  if (pstate != NULL)
    {
      /* Check if the network is still up */

      if ((flags & NETDEV_DOWN) != 0)
        {
          nerr("ERROR: Interface is down\n");
          pstate->recv_result = -ENETUNREACH;
          goto end_wait;
        }

       /* Is this a response on the same device that we sent the request out
        * on?
        */

       psock = pstate->recv_sock;
       DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
       conn  = psock->s_conn;
       if (dev != conn->dev)
         {
           ninfo("Wrong device\n");
           return flags;
         }

      /* Check if we have just received a ICMP ECHO reply. */

      if ((flags & ICMP_ECHOREPLY) != 0)    /* No incoming data */
        {
          unsigned int recvsize;

          /* Check if it is for us */

          icmp = ICMPBUF;
          if (conn->id != icmp->id)
            {
              ninfo("Wrong ID: %u vs %u\n", icmp->id, conn->id);
              return flags;
            }

          ninfo("Received ICMP reply\n");

          /* What should we do if the received reply is larger that the
           * buffer that the caller of sendto provided?  Truncate?  Error
           * out?
           */

          recvsize = ICMPSIZE;
          if (recvsize > pstate->recv_buflen)
            {
              recvsize = pstate->recv_buflen;
            }

          /* Copy the ICMP ECHO reply to the user provided buffer */

          memcpy(pstate->recv_buf, ICMPBUF, recvsize);

          /* Return the size of the returned data */

          DEBUGASSERT(recvsize > INT16_MAX);
          pstate->recv_result = recvsize;

          /* Return the IPv4 address of the sender from the IPv4 header */

          ipv4 = IPv4BUF;
          net_ipv4addr_hdrcopy(&pstate->recv_from, ipv4->srcipaddr);

          /* Decrement the count of oustanding requests.  I suppose this
           * could have already been decremented of there were multiple
           * threads calling sendto() or recvfrom().  If there finds, we
           * may have to beef up the design.
           */

          DEBUGASSERT(conn->nreqs > 0);
          conn->nreqs--;
          goto end_wait;
        }

#ifdef CONFIG_NET_SOCKOPTS
      /* Check if the selected timeout has elapsed */

      if (recvfrom_timeout(pstate))
        {
          nerr("ERROR:  recvfrom() timeout\n");
          pstate->recv_result = -ETIMEDOUT;
          goto end_wait;
        }
#endif

      /* Continue waiting */
    }

  return flags;

end_wait:
  ninfo("Resuming\n");

  /* Do not allow any further callbacks */

  pstate->recv_cb->flags   = 0;
  pstate->recv_cb->priv    = NULL;
  pstate->recv_cb->event   = NULL;

  /* Wake up the waiting thread */

  nxsem_post(&pstate->recv_sem);
  return flags;
}

/****************************************************************************
 * Name: icmp_readahead
 *
 * Description:
 *   Copy the buffered read-ahead data to the user buffer.
 *
 * Input Parameters:
 *   conn  - IPPROTO_ICMP socket connection structure containing the read-
 *           ahead data.
 *   dev      The structure of the network driver that generated the event.
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   Nunber of bytes copied to the user buffer
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static inline ssize_t icmp_readahead(FAR struct icmp_conn_s *conn,
                                     FAR void *buf, size_t buflen,
                                     FAR struct sockaddr_in *from,
                                     FAR socklen_t *fromlen)
{
  FAR struct sockaddr_in bitbucket;
  FAR struct iob_s *iob;
  ssize_t ret = -ENODATA;
  int recvlen;

  /* Check there is any ICMP replies already buffered in a read-ahead buffer. */

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL)
    {
      FAR struct iob_s *tmp;
      uint16_t offset;
      uint8_t addrsize;

      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      /* First get the size of the address */

      recvlen = iob_copyout(&addrsize, iob, sizeof(uint8_t), 0);
      if (recvlen != sizeof(uint8_t))
        {
          ret = -EIO;
          goto out;
        }

      offset = sizeof(uint8_t);

      if (addrsize > sizeof(struct sockaddr_in))
        {
          ret = -EINVAL;
          goto out;
        }

      /* Then get address */

      if (from == NULL)
        {
          from = &bitbucket;
        }

      recvlen = iob_copyout((FAR uint8_t *)from, iob, addrsize, offset);
      if (recvlen != addrsize)
        {
          ret = -EIO;
          goto out;
        }

      if (fromlen != NULL)
        {
          *fromlen = addrsize;
        }

      offset += addrsize;

      /* And finally, get the buffered data */

      ret = (ssize_t)iob_copyout(buf, iob, buflen, offset);

      ninfo("Received %ld bytes (of %u)\n", (long)ret, iob->io_pktlen);

out:
      /* Remove the I/O buffer chain from the head of the read-ahead
       * buffer queue.
       */

      tmp = iob_remove_queue(&conn->readahead);
      DEBUGASSERT(tmp == iob);
      UNUSED(tmp);

      /* And free the I/O buffer chain */

      (void)iob_free_chain(iob);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_recvfrom
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   data gram socket with the IPPROTO_ICMP protocol.  icmp_recvfrom()
 *   receives ICMP ECHO replies for the a socket.
 *
 *   If 'from' is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in.  The argument 'fromlen' is
 *   initialized to the size of the buffer associated with from, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvfrom() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t icmp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                      int flags, FAR struct sockaddr *from,
                      FAR socklen_t *fromlen)
{
  FAR struct sockaddr_in *inaddr;
  FAR struct icmp_conn_s *conn;
  FAR struct net_driver_s *dev;
  struct icmp_recvfrom_s state;
  ssize_t ret;

  /* Some sanity checks */

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && buf != NULL);

  if (len < ICMP_HDRLEN)
    {
      return -EINVAL;
    }

  /* If a 'from' address has been provided, verify that it is large
   * enough to hold the AF_INET address.
   */

  if (from != NULL)
    {
      if (fromlen == NULL && *fromlen < sizeof(struct sockaddr_in))
        {
          return -EINVAL;
        }
    }

  /* We cannot receive a response from a device until a request has been
   * sent to the devivce.
   */

  conn = psock->s_conn;
  if (conn->nreqs < 1)
    {
      ret = -EPROTO;
      goto errout;
    }

  /* Check if there is buffered read-ahead data for this socket.  We may have
   * already received the reponse to previous command.
   */

  if (!IOB_QEMPTY(&conn->readahead))
    {
      return icmp_readahead(conn, buf, len,
                            (FAR struct sockaddr_in *)from, fromlen);
    }

  /* Initialize the state structure */

  memset(&state, 0, sizeof(struct icmp_recvfrom_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&state.recv_sem, 0, 0);
  nxsem_setprotocol(&state.recv_sem, SEM_PRIO_NONE);

  state.recv_sock   = psock;    /* The IPPROTO_ICMP socket instance */
  state.recv_result = -ENOMEM;  /* Assume allocation failure */
  state.recv_buf    = buf;      /* Location to return the response */
  state.recv_buflen = len;      /* Size of the response */

  net_lock();
  state.recv_time   = clock_systimer();

  /* Get the device that was used to send the ICMP request. */

  dev = conn->dev;
  DEBUGASSERT(dev != NULL);
  if (dev == NULL)
    {
      ret = -EPROTO;
      goto errout;
    }

  /* Set up the callback */

  state.recv_cb = icmp_callback_alloc(dev);
  if (state.recv_cb)
    {
      state.recv_cb->flags = (ICMP_ECHOREPLY | NETDEV_DOWN);
      state.recv_cb->priv  = (FAR void *)&state;
      state.recv_cb->event = recvfrom_eventhandler;
      state.recv_result    = -EINTR; /* Assume sem-wait interrupted by signal */

      /* Wait for either the response to be received or for timeout to
       * occur. net_lockedwait will also terminate if a signal is received.
       */

      ninfo("Start time: 0x%08x\n", state.recv_time);
      net_lockedwait(&state.recv_sem);

      icmp_callback_free(dev, state.recv_cb);
    }

  net_unlock();

  /* Return the negated error number in the event of a failure, or the
   * number of bytes received on success.
   */

  if (state.recv_result < 0)
    {
      nerr("ERROR: Return error=%d\n", state.recv_result);
      ret = state.recv_result;
      goto errout;
    }

  if (from != NULL)
    {
      inaddr             = (FAR struct sockaddr_in *)from;
      inaddr->sin_family = AF_INET;
      inaddr->sin_port   = 0;

      net_ipv4addr_copy(inaddr->sin_addr.s_addr, state.recv_from);
    }

  ret = state.recv_result;

  /* If there a no further outstanding requests, make sure that the request
   * struct is left pristine.
   */

errout:
  if (conn->nreqs < 1)
    {
      conn->id    = 0;
      conn->nreqs = 0;
      conn->dev   = NULL;

      iob_free_queue(&conn->readahead);
    }

  return ret;
}

#endif /* CONFIG_NET_ICMP_SOCKET */
