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
#if defined(CONFIG_NET) && defined(CONFIG_NET_ICMPv6) && \
    defined(CONFIG_NET_ICMPv6v6_PING)

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <debug.h>

#include <net/if.h>

#include <nuttx/clock.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmpv6.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICMPv6BUF ((struct icmpv6_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPv6DAT (&dev->d_buf[NET_LL_HDRLEN(dev) + sizeof(struct icmpv6_iphdr_s)])

/* Allocate a new ICMPv6 data callback */

#define icmpv6_callback_alloc()   devif_callback_alloc(&g_icmpv6_echocallback)
#define icmpv6_callback_free(cb)  devif_callback_free(cb, &g_icmpv6_echocallback)

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
  uint16_t       png_seqno;   /* IN: seqno to send; OUT: seqno recieved */
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
  FAR uint8_t *ptr;
  int i;

  nllvdbg("flags: %04x\n", flags);
  if (pstate)
    {
      /* Check if this is a ICMPv6 ECHO reply.  If so, return the sequence
       * number to the caller.  NOTE: We may not even have sent the
       * requested ECHO request; this could have been the delayed ECHO
       * response from a previous ping.
       */

      if ((flags & ICMPv6_ECHOREPLY) != 0 && conn != NULL)
        {
          FAR struct icmpv6_iphdr_s *icmpv6 = (FAR struct icmpv6_iphdr_s *)conn;

          nlldbg("ECHO reply: id=%d seqno=%d\n",
                 ntohs(icmpv6->id), ntohs(icmpv6->seqno));

          if (ntohs(icmpv6->id) == pstate->png_id)
            {
              /* Consume the ECHOREPLY */

              flags     &= ~ICMPv6_ECHOREPLY;
              dev->d_len = 0;

              /* Return the result to the caller */

              pstate->png_result = OK;
              pstate->png_seqno  = ntohs(icmpv6->seqno);
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
          FAR struct icmpv6_iphdr_s *picmpv6 = ICMPv6BUF;

          /* We can send the ECHO request now.
           *
           * Format the ICMPv6 ECHO request packet
           */

          picmpv6->type  = ICMPv6_ECHO_REQUEST;
          picmpv6->icode = 0;
# error "IPv6 ECHO Request not implemented"

          /* Add some easily verifiable data */

          for (i = 0, ptr = ICMPv6DAT; i < pstate->png_datlen; i++)
            {
              *ptr++ = i;
            }

          /* Send the ICMPv6 echo request.  Note that d_sndlen is set to
           * the size of the ICMPv6 payload and does not include the size
           * of the ICMPv6 header.
           */

          nlldbg("Send ECHO request: seqno=%d\n", pstate->png_seqno);

          dev->d_sndlen = pstate->png_datlen + 4;
          icmpv6_send(dev, &pstate->png_addr);
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

          if (!net_ipv6addr_maskcmp(pstate->png_addr, dev->d_ipaddr, dev->d_netmask))
            {
              /* Destination address was not on the local network served by this
               * device.  If a timeout occurs, then the most likely reason is
               * that the destination address is not reachable.
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
  struct icmpv6_ping_s state;
  net_lock_t save;
#ifdef CONFIG_NET_ARP_SEND
  int ret;

  /* Make sure that the IP address mapping is in the ARP table */

  ret = arp_send(addr);
  if (ret < 0)
    {
      ndbg("ERROR: Not reachable\n");
      return -ENETUNREACH;
    }
#endif

  /* Initialize the state structure */

  sem_init(&state.png_sem, 0, 0);
  state.png_ticks  = DSEC2TICK(dsecs); /* System ticks to wait */
  state.png_result = -ENOMEM;          /* Assume allocation failure */
  state.png_addr   = addr;             /* Address of the peer to be ping'ed */
  state.png_id     = id;               /* The ID to use in the ECHO request */
  state.png_seqno  = seqno;            /* The seqno to use int the ECHO request */
  state.png_datlen = datalen;          /* The length of data to send in the ECHO request */
  state.png_sent   = false;            /* ECHO request not yet sent */

  save             = net_lock();
  state.png_time   = clock_systimer();

  /* Set up the callback */

  state.png_cb = icmpv6_callback_alloc();
  if (state.png_cb)
    {
      state.png_cb->flags   = (ICMPv6_POLL | ICMPv6_ECHOREPLY);
      state.png_cb->priv    = (void*)&state;
      state.png_cb->event   = ping_interrupt;
      state.png_result      = -EINTR; /* Assume sem-wait interrupted by signal */

      /* Notify the device driver of the availability of TX data */

#ifdef CONFIG_NET_MULTILINK
      netdev_ipv6_txnotify(g_ipv6_allzeroaddr, state.png_addr);
#else
      netdev_ipv6_txnotify(state.png_addr);
#endif

      /* Wait for either the full round trip transfer to complete or
       * for timeout to occur. (1) net_lockedwait will also terminate if a
       * signal is received, (2) interrupts may be disabled!  They will
       * be re-enabled while the task sleeps and automatically
       * re-enabled when the task restarts.
       */

      nlldbg("Start time: 0x%08x seqno: %d\n", state.png_time, seqno);
      net_lockedwait(&state.png_sem);

      icmpv6_callback_free(state.png_cb);
    }

  net_unlock(save);

  /* Return the negated error number in the event of a failure, or the
   * sequence number of the ECHO reply on success.
   */

  if (!state.png_result)
    {
      nlldbg("Return seqno=%d\n", state.png_seqno);
      return (int)state.png_seqno;
    }
  else
    {
      nlldbg("Return error=%d\n", -state.png_result);
      return state.png_result;
    }
}

#endif /* CONFIG_NET_ICMPv6 && CONFIG_NET_ICMPv6v6_PING  ... */
