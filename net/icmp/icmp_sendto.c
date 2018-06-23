/****************************************************************************
 * net/icmp/icmp_sendto.c
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <debug.h>

#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <nuttx/semaphore.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmp.h>

#include "utils/utils.h"
#include "socket/socket.h"
#include "netdev/netdev.h"
#include "devif/devif.h"
#include "inet/inet.h"
#include "arp/arp.h"
#include "icmp/icmp.h"

#ifdef CONFIG_NET_ICMP_SOCKET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF \
  ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPBUF \
  ((struct icmp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct icmp_sendto_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  FAR struct socket *snd_sock; /* IPPROTO_ICMP socket structure */
  sem_t snd_sem;               /* Use to manage the wait for send complete */
  clock_t snd_time;            /* Start time for determining timeouts */
  in_addr_t snd_toaddr;        /* The peer to send the request to */
  FAR const uint8_t *snd_buf;  /* ICMP header + data payload */
  uint16_t snd_buflen;         /* Size of the ICMP header + data payload */
  int16_t snd_result;          /* 0: success; <0:negated errno on fail */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sendto_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Input Parameters:
 *   pstate - Reference to instance ot sendto state structure
 *
 * Returned Value:
 *   true: timeout false: no timeout
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SOCKOPTS
static inline int sendto_timeout(FAR struct icmp_sendto_s *pstate)
{
  FAR struct socket *psock;

  /* Check for a timeout configured via setsockopts(SO_SNDTIMEO).
   * If none... we will let the send wait forever.
   */

  psock = pstate->snd_sock;
  if (psock != NULL && psock->s_sndtimeo != 0)
    {
      /* Check if the configured timeout has elapsed */

      return net_timeo(pstate->snd_time, psock->s_sndtimeo);
    }

  /* No timeout */

  return false;
}
#endif /* CONFIG_NET_SOCKOPTS */

/****************************************************************************
 * Name: sendto_request
 *
 * Description:
 *   Setup to send an ICMP request packet
 *
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   pstate - Reference to an instance of the ICMP sendto state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void sendto_request(FAR struct net_driver_s *dev,
                           FAR struct icmp_sendto_s *pstate)
{
  FAR struct ipv4_hdr_s *ipv4;
  FAR struct icmp_hdr_s *icmp;

  IFF_SET_IPv4(dev->d_flags);

  /* The total length to send is the size of the application data plus the
   * IP and ICMP headers (and, eventually, the Ethernet header)
   */

  dev->d_len = IPv4_HDRLEN + pstate->snd_buflen;

  /* The total size of the data (including the size of the ICMP header) */

  dev->d_sndlen += pstate->snd_buflen;

  /* Initialize the IP header. */

  ipv4              = IPv4BUF;
  ipv4->vhl         = 0x45;
  ipv4->tos         = 0;
  ipv4->len[0]      = (dev->d_len >> 8);
  ipv4->len[1]      = (dev->d_len & 0xff);
  ++g_ipid;
  ipv4->ipid[0]     = g_ipid >> 8;
  ipv4->ipid[1]     = g_ipid & 0xff;
  ipv4->ipoffset[0] = IP_FLAG_DONTFRAG >> 8;
  ipv4->ipoffset[1] = IP_FLAG_DONTFRAG & 0xff;
  ipv4->ttl         = IP_TTL;
  ipv4->proto       = IP_PROTO_ICMP;

  net_ipv4addr_hdrcopy(ipv4->srcipaddr, &dev->d_ipaddr);
  net_ipv4addr_hdrcopy(ipv4->destipaddr, &pstate->snd_toaddr);

  /* Copy the ICMP header and payload into place after the IPv4 header */

  icmp              = ICMPBUF;
  memcpy(icmp, pstate->snd_buf, pstate->snd_buflen);

  /* Calculate IP checksum. */

  ipv4->ipchksum    = 0;
  ipv4->ipchksum    = ~(ipv4_chksum(dev));

  /* Calculate the ICMP checksum. */

  icmp->icmpchksum  = 0;
  icmp->icmpchksum  = ~(icmp_chksum(dev, pstate->snd_buflen));
  if (icmp->icmpchksum == 0)
    {
      icmp->icmpchksum = 0xffff;
    }

  ninfo("Outgoing ICMP packet length: %d (%d)\n",
        dev->d_len, (ipv4->len[0] << 8) | ipv4->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmp.sent++;
  g_netstats.ipv4.sent++;
#endif
}

/****************************************************************************
 * Name: sendto_eventhandler
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
 *   pvpriv     An instance of struct icmp_sendto_s cast to (void *)
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t sendto_eventhandler(FAR struct net_driver_s *dev,
                                  FAR void *conn,
                                  FAR void *pvpriv, uint16_t flags)
{
  FAR struct icmp_sendto_s *pstate = (struct icmp_sendto_s *)pvpriv;

  ninfo("flags: %04x\n", flags);

  if (pstate != NULL)
    {
      /* Check if the network is still up */

      if ((flags & NETDEV_DOWN) != 0)
        {
          nerr("ERROR: Interface is down\n");
          pstate->snd_result = -ENETUNREACH;
          goto end_wait;
        }

      /* Check:
       *   If the outgoing packet is available (it may have been claimed
       *   by a sendto event handler serving a different thread)
       * -OR-
       *   If the output buffer currently contains unprocessed incoming
       *   data.
       * -OR-
       *   If we have already sent the ECHO request
       *
       * In the first two cases, we will just have to wait for the next
       * polling cycle.
       */

      if (dev->d_sndlen <= 0 &&           /* Packet available */
          (flags & ICMP_NEWDATA) == 0)    /* No incoming data */
        {
          /* Send the ICMP echo request.  */

          ninfo("Send ICMP request\n");

          sendto_request(dev, pstate);
          pstate->snd_result = OK;
          goto end_wait;
        }

#ifdef CONFIG_NET_SOCKOPTS
      /* Check if the selected timeout has elapsed */

      if (sendto_timeout(pstate))
        {
          int failcode;

          /* Check if this device is on the same network as the destination
           * device.
           */

          if (!net_ipv4addr_maskcmp(pstate->snd_toaddr, dev->d_ipaddr, dev->d_netmask))
            {
              /* Destination address was not on the local network served by this
               * device.  If a timeout occurs, then the most likely reason is
               * that the destination address is not reachable.
               */

              nerr("ERROR:  Not reachable\n");
              failcode = -ENETUNREACH;
            }
          else
            {
              nerr("ERROR:  sendto() timeout\n");
              failcode = -ETIMEDOUT;
            }

          /* Report the failure */

          pstate->snd_result = failcode;
          goto end_wait;
        }
#endif

      /* Continue waiting */
    }

  return flags;

end_wait:
  ninfo("Resuming\n");

  /* Do not allow any further callbacks */

  pstate->snd_cb->flags   = 0;
  pstate->snd_cb->priv    = NULL;
  pstate->snd_cb->event   = NULL;

  /* Wake up the waiting thread */

  nxsem_post(&pstate->snd_sem);
  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_sendto
 *
 * Description:
 *   Implements the sendto() operation for the case of the IPPROTO_ICMP
 *   socket.  The 'buf' parameter points to a block of memory that includes
 *   an ICMP request header, followed by any payload that accompanies the
 *   request.  The 'len' parameter includes both the size of the ICMP header
 *   and the following payload.
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
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send_to() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

ssize_t icmp_sendto(FAR struct socket *psock, FAR const void *buf, size_t len,
                    int flags, FAR const struct sockaddr *to, socklen_t tolen)
{
  FAR const struct sockaddr_in *inaddr;
  FAR struct net_driver_s *dev;
  FAR struct icmp_conn_s *conn;
  FAR struct icmp_hdr_s *icmp;
  struct icmp_sendto_s state;
  int ret;

  /* Some sanity checks */

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL &&
              buf != NULL && to != NULL);

  if (len < ICMP_HDRLEN || tolen < sizeof(struct sockaddr_in))
    {
      return -EINVAL;
    }

  conn   = psock->s_conn;
  inaddr = (FAR const struct sockaddr_in *)to;

  /* Get the device that will be used to route this ICMP ECHO request */

  dev = netdev_findby_ipv4addr(INADDR_ANY, inaddr->sin_addr.s_addr);
  if (dev == NULL)
    {
      nerr("ERROR: Not reachable\n");
      ret = -ENETUNREACH;
      goto errout;
    }

  /* If we are no longer processing the same ping ID, then flush any pending
   * packets from the read-ahead buffer.
   *
   * REVISIT:  How to we free up any lingering reponses if there are no
   * futher pings?
   */

  icmp = (FAR struct icmp_hdr_s *)buf;
  if (icmp->type != ICMP_ECHO_REQUEST || icmp->id != conn->id ||
      dev != conn->dev)
    {
      conn->id    = 0;
      conn->nreqs = 0;
      conn->dev   = NULL;

      iob_free_queue(&conn->readahead);
    }

#ifdef CONFIG_NET_ARP_SEND
  /* Make sure that the IP address mapping is in the ARP table */

  ret = arp_send(inaddr->sin_addr.s_addr);
  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      ret = -ENETUNREACH;
      goto errout;
    }
#endif

  /* Initialize the state structure */

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&state.snd_sem, 0, 0);
  nxsem_setprotocol(&state.snd_sem, SEM_PRIO_NONE);

  state.snd_sock   = psock;            /* The IPPROTO_ICMP socket instance */
  state.snd_result = -ENOMEM;          /* Assume allocation failure */
  state.snd_toaddr = inaddr->sin_addr.s_addr; /* Address of the peer to send the request */
  state.snd_buf    = buf;              /* ICMP header + data payload */
  state.snd_buflen = len;              /* Size of the ICMP header + data payload */

  net_lock();
  state.snd_time   = clock_systimer();

  /* Set up the callback */

  state.snd_cb = icmp_callback_alloc(dev);
  if (state.snd_cb)
    {
      state.snd_cb->flags   = (ICMP_POLL | NETDEV_DOWN);
      state.snd_cb->priv    = (FAR void *)&state;
      state.snd_cb->event   = sendto_eventhandler;
      state.snd_result      = -EINTR; /* Assume sem-wait interrupted by signal */

      /* Setup to receive ICMP ECHO replies */

      if (icmp->type == ICMP_ECHO_REQUEST)
        {
          conn->id    = icmp->id;
          conn->nreqs = 1;
        }

        conn->dev     = dev;

      /* Notify the device driver of the availability of TX data */

      netdev_txnotify_dev(dev);

      /* Wait for either the send to complete or for timeout to occur.
       * net_lockedwait will also terminate if a signal is received.
       */

      ninfo("Start time: 0x%08x\n", state.snd_time);
      net_lockedwait(&state.snd_sem);

      icmp_callback_free(dev, state.snd_cb);
    }

  net_unlock();

  /* Return the negated error number in the event of a failure, or the
   * number of bytes sent on success.
   */

  if (state.snd_result < 0)
    {
      nerr("ERROR: Return error=%d\n", state.snd_result);
      ret = state.snd_result;
      goto errout;
    }

  return len;

errout:
  conn->id    = 0;
  conn->nreqs = 0;
  conn->dev   = NULL;

  iob_free_queue(&conn->readahead);
  return ret;
}

#endif /* CONFIG_NET_ICMP_SOCKET */
