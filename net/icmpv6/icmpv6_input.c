/****************************************************************************
 * net/icmpv6/icmpv6_input.c
 * Handling incoming ICMPv6 input
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/icmpv6.h>

#include "devif/devif.h"
#include "neighbor/neighbor.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHBUF \
  ((struct eth_hdr_s *)&dev->d_buf[0])
#define IPv6BUF \
  ((FAR struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPICMPv6 \
  ((struct icmpv6_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPv6REPLY \
  ((FAR struct icmpv6_echo_reply_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])
#define ICMPv6SIZE \
 ((dev)->d_len - IPv6_HDRLEN)

#define ICMPv6SOLICIT \
  ((struct icmpv6_neighbor_solicit_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])
#define ICMPv6ADVERTISE \
  ((struct icmpv6_neighbor_advertise_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])
#define ICMPv6RADVERTISE \
  ((struct icmpv6_router_advertise_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_datahandler
 *
 * Description:
 *   Handle ICMPv6 echo replies that are not accepted by the application.
 *
 * Input Parameters:
 *   dev    - Device instance only the input packet in d_buf, length = d_len;
 *   conn   - A pointer to the ICMPv6 connection structure
 *   buffer - A pointer to the buffer to be copied to the read-ahead
 *     buffers
 *   buflen - The number of bytes to copy to the read-ahead buffer.
 *
 * Returned Value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to buflen; partial packets are not buffered.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
static uint16_t icmpv6_datahandler(FAR struct net_driver_s *dev,
                                   FAR struct icmpv6_conn_s *conn)
{
  FAR struct ipv6_hdr_s *ipv6;
  struct sockaddr_in6 inaddr;
  FAR struct iob_s *iob;
  uint16_t offset;
  uint16_t buflen;
  uint8_t addrsize;
  int ret;

  /* Try to allocate on I/O buffer to start the chain without waiting (and
   * throttling as necessary).  If we would have to wait, then drop the
   * packet.
   */

  iob = iob_tryalloc(true);
  if (iob == NULL)
    {
      nerr("ERROR: Failed to create new I/O buffer chain\n");
      goto drop;
    }

  /* Put the IPv6 address at the beginning of the read-ahead buffer */

  ipv6               = IPv6BUF;
  inaddr.sin6_family = AF_INET6;
  inaddr.sin6_port   = 0;
  net_ipv6addr_copy(inaddr.sin6_addr.s6_addr16, ipv6->srcipaddr);

  /* Copy the src address info into the I/O buffer chain.  We will not wait
   * for an I/O buffer to become available in this context.  It there is
   * any failure to allocated, the entire I/O buffer chain will be discarded.
   */

  addrsize = sizeof(struct sockaddr_in6);
  ret      = iob_trycopyin(iob, &addrsize, sizeof(uint8_t), 0, true);
  if (ret < 0)
    {
      /* On a failure, iob_trycopyin return a negated error value but does
       * not free any I/O buffers.
       */

      nerr("ERROR: Failed to length to the I/O buffer chain: %d\n", ret);
      goto drop_with_chain;
    }

  offset = sizeof(uint8_t);

  ret = iob_trycopyin(iob, (FAR const uint8_t *)&inaddr,
                      sizeof(struct sockaddr_in6), offset, true);
  if (ret < 0)
    {
      /* On a failure, iob_trycopyin return a negated error value but does
       * not free any I/O buffers.
       */

      nerr("ERROR: Failed to source address to the I/O buffer chain: %d\n", ret);
      goto drop_with_chain;
    }

  offset += sizeof(struct sockaddr_in6);

  /* Copy the new ICMPv6 reply into the I/O buffer chain (without waiting) */

  buflen = ICMPv6SIZE;
  ret = iob_trycopyin(iob, (FAR uint8_t *)ICMPv6REPLY, buflen, offset, true);
  if (ret < 0)
    {
      /* On a failure, iob_copyin return a negated error value but does
       * not free any I/O buffers.
       */

      nerr("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      goto drop_with_chain;
    }

  /* Add the new I/O buffer chain to the tail of the read-ahead queue (again
   * without waiting).
   */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      goto drop_with_chain;
    }

  ninfo("Buffered %d bytes\n", buflen + addrsize + 1);
  dev->d_len = 0;
  return buflen;

drop_with_chain:
  (void)iob_free_chain(iob);

drop:
  dev->d_len = 0;
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_input
 *
 * Description:
 *   Handle incoming ICMPv6 input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received ICMPv6
 *         packet
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmpv6_input(FAR struct net_driver_s *dev)
{
  FAR struct icmpv6_iphdr_s *ipicmp = IPICMPv6;

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.recv++;
#endif

  /* Handle the ICMPv6 message by its type */

  switch (ipicmp->type)
    {
    /* If we get a neighbor solicitation for our address we should send
     * a neighbor advertisement message back.
     */

    case ICMPv6_NEIGHBOR_SOLICIT:
      {
        FAR struct icmpv6_neighbor_solicit_s *sol;

        /* Check if we are the target of the solicitation */

        sol = ICMPv6SOLICIT;
        if (net_ipv6addr_cmp(sol->tgtaddr, dev->d_ipv6addr))
          {
            /* Yes..  Send a neighbor advertisement back to where the neighbor
             * solicitation came from.
             */

            icmpv6_advertise(dev, ipicmp->srcipaddr);

            /* All statistics have been updated.  Nothing to do but exit. */

            return;
          }
        else
          {
            goto icmpv6_drop_packet;
          }
      }
      break;

    /* Check if we received a Neighbor Advertisement */

    case ICMPv6_NEIGHBOR_ADVERTISE:
      {
        FAR struct icmpv6_neighbor_advertise_s *adv;

        /* If the IPv6 destination address matches our address, and if so,
         * add the neighbor address mapping to the list of neighbors.
         *
         * Missing checks:
         *   optlen = 1 (8 octets)
         *   Should only update Neighbor Table if [O]verride bit is set in flags
         */

        adv = ICMPv6ADVERTISE;
        if (net_ipv6addr_cmp(ipicmp->destipaddr, dev->d_ipv6addr))
          {
            /* This message is required to support the Target link-layer
             * address option.
             */

            if (adv->opttype == ICMPv6_OPT_TGTLLADDR)
              {
                /* Save the sender's address mapping in our Neighbor Table. */

                neighbor_add(dev, ipicmp->srcipaddr, adv->tgtlladdr);

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
                /* Then notify any logic waiting for the Neighbor Advertisement */

                icmpv6_notify(ipicmp->srcipaddr);
#endif

                /* We consumed the packet but we don't send anything in
                 * response.
                 */

                goto icmpv6_send_nothing;
              }
          }

        goto icmpv6_drop_packet;
      }
      break;

#ifdef CONFIG_NET_ICMPv6_ROUTER
    /* Check if we received a Router Solicitation */

    case ICMPV6_ROUTER_SOLICIT:
      {
        /* Just give a knee-jerk Router Advertisement in respond with no
         * further examination of the Router Solicitation.
         */

        icmpv6_radvertise(dev);

        /* All statistics have been updated.  Nothing to do but exit. */

        return;
      }
#endif

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
    /* Check if we received a Router Advertisement */

    case ICMPV6_ROUTER_ADVERTISE:
      {
        FAR struct icmpv6_router_advertise_s *adv;
        FAR uint8_t *options;
        uint16_t pktlen;
        uint16_t optlen;
        int ndx;

        /* Get the length of the option data */

        pktlen = (uint16_t)ipicmp->len[0] << 8 | ipicmp->len[1];
        if (pktlen <= ICMPv6_RADV_MINLEN)
          {
            /* Too small to contain any options */

            goto icmpv6_drop_packet;
          }

        optlen = ICMPv6_RADV_OPTLEN(pktlen);

        /* We need to have a valid router advertisement with a Prefix and
         * with the "A" bit set in the flags.  Options immediately follow
         * the ICMPv6 router advertisement.
         */

        adv     = ICMPv6RADVERTISE;
        options = (FAR uint8_t *)adv + sizeof(struct icmpv6_router_advertise_s);

        for (ndx = 0; ndx + sizeof(struct icmpv6_prefixinfo_s) <= optlen; )
          {
            FAR struct icmpv6_srclladdr_s *sllopt =
              (FAR struct icmpv6_srclladdr_s *)&options[ndx];

            if (sllopt->opttype == 1 && sllopt->optlen == 1)
              {
                neighbor_add(dev, ipicmp->srcipaddr, sllopt->srclladdr);
              }

            FAR struct icmpv6_prefixinfo_s *opt =
              (FAR struct icmpv6_prefixinfo_s *)&options[ndx];

            /* Is this the sought for prefix? Is it the correct size? Is
             * the "A" flag set?
             */

            if (opt->opttype &&
                opt->optlen == 4 &&
               (opt->flags & ICMPv6_PRFX_FLAG_A) != 0)
              {
                /* Yes.. Notify any waiting threads */

                icmpv6_rnotify(dev, ipicmp->srcipaddr, opt->prefix, opt->preflen);
                goto icmpv6_send_nothing;
              }

            /* Skip to the next option (units of octets) */

            ndx += (opt->optlen << 3);
          }

        goto icmpv6_drop_packet;
      }
      break;
#endif

    /* Handle the ICMPv6 Echo Request */

    case ICMPv6_ECHO_REQUEST:
      {
        /* ICMPv6 echo (i.e., ping) processing. This is simple, we only
         * change the ICMPv6 type from ECHO to ECHO_REPLY and update the
         * ICMPv6 checksum before we return the packet.
         */

        ipicmp->type = ICMPv6_ECHO_REPLY;

        net_ipv6addr_copy(ipicmp->destipaddr, ipicmp->srcipaddr);
        net_ipv6addr_copy(ipicmp->srcipaddr, dev->d_ipv6addr);

        ipicmp->chksum = 0;
        ipicmp->chksum = ~icmpv6_chksum(dev);
      }
      break;

#ifdef CONFIG_NET_ICMPv6_SOCKET
    /* If an ICMPv6 echo reply is received then there should also be
     * a thread waiting to received the echo response.
     */

    case ICMPv6_ECHO_REPLY:
      {
        uint16_t flags = ICMPv6_ECHOREPLY;

        /* Dispatch the ECHO reply to the waiting thread */

        flags = devif_conn_event(dev, NULL, flags, dev->d_conncb);

        /* Was the ECHO reply consumed by any waiting thread? */

        if ((flags & ICMPv6_ECHOREPLY) != 0)
          {
            FAR struct icmpv6_echo_reply_s *reply;
            FAR struct icmpv6_conn_s *conn;
            uint16_t nbuffered;

            /* Nothing consumed the ICMP reply.  That might because this is
             * an old, invalid reply or simply because the ping application
             * has not yet put its poll or recv in place.
             */

            /* Is there any connection that might expect this reply? */

            reply = ICMPv6REPLY;
            conn = icmpv6_findconn(dev, reply->id);
            if (conn == NULL)
              {
                /* No.. drop the packet */

                goto icmpv6_drop_packet;
              }

            /* Yes.. Add the ICMP echo reply to the IPPROTO_ICMP socket read
             * ahead buffer.
             */

            nbuffered = icmpv6_datahandler(dev, conn);
            if (nbuffered == 0)
              {
                /* Could not buffer the data.. drop the packet */

                goto icmpv6_drop_packet;
              }
          }

          goto icmpv6_send_nothing;
      }
      break;
#endif

    default:
      {
        nwarn("WARNING: Unknown ICMPv6 type: %d\n", ipicmp->type);
        goto icmpv6_type_error;
      }
    }

  ninfo("Outgoing ICMPv6 packet length: %d (%d)\n",
          dev->d_len, (ipicmp->len[0] << 8) | ipicmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
  return;

icmpv6_type_error:
#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.typeerr++;
#endif

icmpv6_drop_packet:
#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.drop++;
#endif

icmpv6_send_nothing:
  dev->d_len = 0;
}

#endif /* CONFIG_NET_ICMPv6 */
