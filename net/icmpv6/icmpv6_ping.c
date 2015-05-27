/****************************************************************************
 * net/icmpv6/icmpv6_ping.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET_ICMPv6_PING

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <debug.h>

#include <net/if.h>

#include <nuttx/clock.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmpv6.h>
#include <nuttx/net/netstats.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHBUF    ((struct eth_hdr_s *)&dev->d_buf[0])
#define ICMPv6BUF ((struct icmpv6_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPv6ECHOREQ \
  ((struct icmpv6_echo_request_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])
#define ICMPv6ECHOREPLY \
  ((struct icmpv6_echo_reply_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct icmpv6_ping_s
{
  FAR struct devif_callback_s *png_cb; /* Reference to callback instance */

  sem_t          png_sem;     /* Use to manage the wait for the response */
  uint32_t       png_time;    /* Start time for determining timeouts */
  uint32_t       png_ticks;   /* System clock ticks to wait */
  int            png_result;  /* 0: success; <0:negated errno on fail */
  net_ipv6addr_t png_addr;    /* The peer to be ping'ed */
  uint16_t       png_id;      /* Used to match requests with replies */
  uint16_t       png_seqno;   /* IN: seqno to send; OUT: seqno received */
  uint16_t       png_datlen;  /* The length of data to send in the ECHO request */
  bool           png_sent;    /* true... the PING request has been sent */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: ping_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Parameters:
 *   pstate - Ping state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static inline int ping_timeout(FAR struct icmpv6_ping_s *pstate)
{
  uint32_t elapsed =  clock_systimer() - pstate->png_time;
  if (elapsed >= pstate->png_ticks)
    {
      return TRUE;
    }

  return FALSE;
}

/****************************************************************************
 * Name: icmpv6_echo_request
 *
 * Description:
 *   Format an ICMPv6 Echo Request message.. This version
 *   is for a standalone solicitation.  If formats:
 *
 *   - The Ethernet header
 *   - The IPv6 header
 *   - The ICMPv6 Echo Request Message
 *
 * Parameters:
 *   dev    - Reference to an Ethernet device driver structure
 *   pstate - Ping state structure
 *
 * Return:
 *   None
 *
 ****************************************************************************/

static void icmpv6_echo_request(FAR struct net_driver_s *dev,
                                FAR struct icmpv6_ping_s *pstate)
{
  FAR struct icmpv6_iphdr_s *icmp;
  FAR struct icmpv6_echo_request_s *req;
  uint16_t reqlen;
  int i;

  nllvdbg("Send ECHO request: seqno=%d\n", pstate->png_seqno);

  /* Set up the IPv6 header (most is probably already in place) */

  icmp          = ICMPv6BUF;
  icmp->vtc     = 0x60;                    /* Version/traffic class (MS) */
  icmp->tcf     = 0;                       /* Traffic class (LS)/Flow label (MS) */
  icmp->flow    = 0;                       /* Flow label (LS) */

  /* Length excludes the IPv6 header */

  reqlen        = SIZEOF_ICMPV6_ECHO_REQUEST_S(pstate->png_datlen);
  icmp->len[0]  = (reqlen >> 8);
  icmp->len[1]  = (reqlen & 0xff);

  icmp->proto   = IP_PROTO_ICMP6;          /* Next header */
  icmp->ttl     = IP_TTL;                  /* Hop limit */

  /* Set the multicast destination IP address */

  net_ipv6addr_copy(icmp->destipaddr,  pstate->png_addr);

  /* Add out IPv6 address as the source address */

  net_ipv6addr_copy(icmp->srcipaddr, dev->d_ipv6addr);

  /* Set up the ICMPv6 Echo Request message */

  req           = ICMPv6ECHOREQ;
  req->type     = ICMPv6_ECHO_REQUEST; /* Message type */
  req->code     = 0;                   /* Message qualifier */
  req->id       = htons(pstate->png_id);
  req->seqno    = htons(pstate->png_seqno);

  /* Add some easily verifiable data */

  for (i = 0; i < pstate->png_datlen; i++)
    {
      req->data[i] = i;
    }

  /* Calculate the checksum over both the ICMP header and payload */

  icmp->chksum  = 0;
  icmp->chksum  = ~icmpv6_chksum(dev);

  /* Set the size to the size of the IPv6 header and the payload size */

  IFF_SET_IPv6(dev->d_flags);

  dev->d_sndlen = reqlen;
  dev->d_len    = reqlen + IPv6_HDRLEN;

  nllvdbg("Outgoing ICMPv6 Echo Request length: %d (%d)\n",
          dev->d_len, (icmp->len[0] << 8) | icmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

/****************************************************************************
 * Function: ping_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   ECHO request and/or ECHO reply actions when polled by the lower, device
 *   interfacing layer.
 *
 * Parameters:
 *   dev        The structure of the network driver that caused the interrupt
 *   conn       The received packet, cast to void *
 *   pvpriv     An instance of struct icmpv6_ping_s cast to void*
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static uint16_t ping_interrupt(FAR struct net_driver_s *dev, FAR void *conn,
                               FAR void *pvpriv, uint16_t flags)
{
  FAR struct icmpv6_ping_s *pstate = (struct icmpv6_ping_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  if (pstate)
    {
      /* Check if the network is still up */

      if ((flags & NETDEV_DOWN) != 0)
        {
          nlldbg("ERROR: Interface is down\n");
          pstate->png_result = -ENETUNREACH;
          goto end_wait;
        }

      /* Check if this is a ICMPv6 ECHO reply.  If so, return the sequence
       * number to the caller.  NOTE: We may not even have sent the
       * requested ECHO request; this could have been the delayed ECHO
       * response from a previous ping.
       */

      else if ((flags & ICMPv6_ECHOREPLY) != 0 && conn != NULL)
        {
          FAR struct icmpv6_echo_reply_s *reply = ICMPv6ECHOREPLY;

          nllvdbg("ECHO reply: id=%d seqno=%d\n",
                  ntohs(reply->id), ntohs(reply->seqno));

          if (ntohs(reply->id) == pstate->png_id)
            {
              /* Consume the ECHOREPLY */

              flags     &= ~ICMPv6_ECHOREPLY;
              dev->d_len = 0;

              /* Return the result to the caller */

              pstate->png_result = OK;
              pstate->png_seqno  = ntohs(reply->seqno);
              goto end_wait;
            }
        }

      /* Check:
       *   If the outgoing packet is available (it may have been claimed
       *   by a sendto interrupt serving a different thread)
       * -OR-
       *   If the output buffer currently contains unprocessed incoming
       *   data.
       * -OR-
       *   If we have already sent the ECHO request
       *
       * In the first two cases, we will just have to wait for the next
       * polling cycle.
       */

      if (dev->d_sndlen <= 0 &&             /* Packet available */
          (flags & ICMPv6_NEWDATA) == 0 &&  /* No incoming data */
          !pstate->png_sent)                /* Request not sent */
        {
          /* Send the ECHO request now. */

          icmpv6_echo_request(dev, pstate);
          pstate->png_sent = true;
          return flags;
        }

      /* Check if the selected timeout has elapsed */

      if (ping_timeout(pstate))
        {
          int failcode;

          /* Check if this device is on the same network as the destination
           * device.
           */

          if (!net_ipv6addr_maskcmp(pstate->png_addr, dev->d_ipv6addr,
                                    dev->d_ipv6netmask))
            {
              /* Destination address was not on the local network served by
               * this device.  If a timeout occurs, then the most likely
               * reason is that the destination address is not reachable.
               */

              nlldbg("Not reachable\n");
              failcode = -ENETUNREACH;
            }
          else
            {
              nlldbg("Ping timeout\n");
              failcode = -ETIMEDOUT;
            }

          /* Report the failure */

          pstate->png_result = failcode;
          goto end_wait;
        }

      /* Continue waiting */
    }

  return flags;

end_wait:
  nllvdbg("Resuming\n");

  /* Do not allow any further callbacks */

  pstate->png_cb->flags   = 0;
  pstate->png_cb->priv    = NULL;
  pstate->png_cb->event   = NULL;

  /* Wake up the waiting thread */

  sem_post(&pstate->png_sem);
  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imcpv6_ping
 *
 * Description:
 *   Send a ECHO request and wait for the ECHO response
 *
 * Parameters:
 *   addr  - The IP address of the peer to send the ICMPv6 ECHO request to
 *           in network order.
 *   id    - The ID to use in the ICMPv6 ECHO request.  This number should be
 *           unique; only ECHO responses with this matching ID will be
 *           processed (host order)
 *   seqno - The sequence number used in the ICMPv6 ECHO request.  NOT used
 *           to match responses (host order)
 *   dsecs - Wait up to this many deci-seconds for the ECHO response to be
 *           returned (host order).
 *
 * Return:
 *   seqno of received ICMPv6 ECHO with matching ID (may be different
 *   from the seqno argument (may be a delayed response from an earlier
 *   ping with the same ID). Or a negated errno on any failure.
 *
 * Assumptions:
 *   Called from the user level with interrupts enabled.
 *
 ****************************************************************************/

int icmpv6_ping(net_ipv6addr_t addr, uint16_t id, uint16_t seqno,
                uint16_t datalen, int dsecs)
{
  FAR struct net_driver_s *dev;
  struct icmpv6_ping_s state;
  net_lock_t save;

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  int ret;

  /* Make sure that the IP address mapping is in the Neighbor Table */

  ret = icmpv6_neighbor(addr);
  if (ret < 0)
    {
      ndbg("ERROR: Not reachable\n");
      return -ENETUNREACH;
    }
#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Get the device that will be used to route this ICMP ECHO request */

#ifdef CONFIG_NETDEV_MULTINIC
  dev = netdev_findby_ipv6addr(g_ipv6_allzeroaddr, addr);
#else
  dev = netdev_findby_ipv6addr(addr);
#endif
  if (dev == 0)
    {
      ndbg("ERROR: Not reachable\n");
      return -ENETUNREACH;
    }

  /* Initialize the state structure */

  sem_init(&state.png_sem, 0, 0);
  state.png_ticks  = DSEC2TICK(dsecs);     /* System ticks to wait */
  state.png_result = -ENOMEM;              /* Assume allocation failure */
  state.png_id     = id;                   /* The ID to use in the ECHO request */
  state.png_seqno  = seqno;                /* The seqno to use in the ECHO request */
  state.png_datlen = datalen;              /* The length of data to send in the ECHO request */
  state.png_sent   = false;                /* ECHO request not yet sent */

  net_ipv6addr_copy(state.png_addr, addr); /* Address of the peer to be ping'ed */

  save             = net_lock();
  state.png_time   = clock_systimer();

  /* Set up the callback */

  state.png_cb = icmpv6_callback_alloc(dev);
  if (state.png_cb)
    {
      state.png_cb->flags   = (ICMPv6_POLL | ICMPv6_ECHOREPLY);
      state.png_cb->priv    = (void*)&state;
      state.png_cb->event   = ping_interrupt;
      state.png_result      = -EINTR; /* Assume sem-wait interrupted by signal */

      /* Notify the device driver of the availability of TX data */

      netdev_txnotify_dev(dev);

      /* Wait for either the full round trip transfer to complete or
       * for timeout to occur. (1) net_lockedwait will also terminate if a
       * signal is received, (2) interrupts may be disabled!  They will
       * be re-enabled while the task sleeps and automatically
       * re-enabled when the task restarts.
       */

      nllvdbg("Start time: 0x%08x seqno: %d\n", state.png_time, seqno);
      net_lockedwait(&state.png_sem);

      icmpv6_callback_free(dev, state.png_cb);
    }

  net_unlock(save);

  /* Return the negated error number in the event of a failure, or the
   * sequence number of the ECHO reply on success.
   */

  if (!state.png_result)
    {
      nllvdbg("Return seqno=%d\n", state.png_seqno);
      return (int)state.png_seqno;
    }
  else
    {
      nlldbg("Return error=%d\n", -state.png_result);
      return state.png_result;
    }
}

#endif /* CONFIG_NET_ICMPv6_PING */
