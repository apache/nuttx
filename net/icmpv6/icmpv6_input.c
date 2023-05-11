/****************************************************************************
 * net/icmpv6/icmpv6_input.c
 * Handling incoming ICMPv6 input
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#include <nuttx/net/dns.h>

#include "devif/devif.h"
#include "neighbor/neighbor.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"
#include "mld/mld.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICMPv6REPLY      ((FAR struct icmpv6_echo_reply_s *)icmpv6)
#define ICMPv6SIZE       ((dev)->d_len - iplen)

#define ICMPv6SOLICIT    ((FAR struct icmpv6_neighbor_solicit_s *)icmpv6)
#define ICMPv6ADVERTISE  ((FAR struct icmpv6_neighbor_advertise_s *)icmpv6)
#define ICMPv6RADVERTISE ((FAR struct icmpv6_router_advertise_s *)icmpv6)

#define MLDQUERY         ((FAR struct mld_mcast_listen_query_s *)icmpv6)
#define MLDREPORT_V1     ((FAR struct mld_mcast_listen_report_v1_s *)icmpv6)
#define MLDREPORT_V2     ((FAR struct mld_mcast_listen_report_v2_s *)icmpv6)
#define MLDDONE          ((FAR struct mld_mcast_listen_done_s *)icmpv6)

#ifdef CONFIG_NET_ICMPv6_SOCKET

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct icmpv6_deliver_s
{
  FAR struct net_driver_s *dev; /* Current network device */
  unsigned int iplen;           /* The size of the IPv6 header */
  bool delivered;               /* Whether the message is delivered */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool icmpv6_filter(FAR const uint32_t *data, uint8_t type)
{
  /* We require only the four bytes of the ICMPv6 header. */

  DEBUGASSERT(data != NULL);

  return (data[type >> 5] & (1u << (type & 31))) != 0;
}

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

static uint16_t icmpv6_datahandler(FAR struct net_driver_s *dev,
                                   FAR struct icmpv6_conn_s *conn,
                                   unsigned int iplen)
{
  FAR struct ipv6_hdr_s *ipv6;
  struct sockaddr_in6 inaddr;
  FAR struct iob_s *iob;
  unsigned int offset;
  uint16_t buflen;
  int ret;

  iob = iob_tryalloc(false);
  if (iob == NULL)
    {
      return -ENOMEM;
    }

  /* Put the IPv6 address at the beginning of the read-ahead buffer */

  ipv6               = IPv6BUF;
  inaddr.sin6_family = AF_INET6;
  inaddr.sin6_port   = 0;
  net_ipv6addr_copy(inaddr.sin6_addr.s6_addr16, ipv6->srcipaddr);

  /* Copy the src address info into the front of I/O buffer chain which
   * overwrites the contents of the packet header field.
   */

  memcpy(iob->io_data, &inaddr, sizeof(struct sockaddr_in6));
  offset = sizeof(struct sockaddr_in6);

  iob->io_data[offset++] = ipv6->ttl;

  iob_reserve(iob, offset);

  /* Copy the ICMPv6 message into the I/O buffer chain (without waiting) */

  ret = iob_clone_partial(dev->d_iob, dev->d_iob->io_pktlen,
                          iplen, iob, 0, true, false);
  if (ret < 0)
    {
      iob_free_chain(iob);
      return ret;
    }

  buflen = ICMPv6SIZE;

  /* Add the new I/O buffer chain to the tail of the read-ahead queue (again
   * without waiting).
   */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob);
    }
  else
    {
      ninfo("Buffered %d bytes\n", buflen);
    }

  return buflen;
}

/****************************************************************************
 * Name: icmpv6_delivery_callback
 *
 * Description:
 *   Copy the icmpv6 package to the application according to the filter
 *   conditions, but ICMPv6_ECHO_REPLY is a special message type, if there
 *   is an application waiting, it will also copy.
 *
 * Input Parameters:
 *   conn - A pointer to the ICMPv6 connection structure.
 *   arg - The context information
 *
 ****************************************************************************/

static int icmpv6_delivery_callback(FAR struct icmpv6_conn_s *conn,
                                    FAR void *arg)
{
  FAR struct icmpv6_deliver_s *info   = arg;
  FAR struct net_driver_s     *dev    = info->dev;
  FAR struct icmpv6_hdr_s     *icmpv6 = IPBUF(info->iplen);

  if (icmpv6_filter(conn->filter.icmp6_filt, icmpv6->type) &&
      (icmpv6->type != ICMPv6_ECHO_REPLY || conn->id != ICMPv6REPLY->id ||
       conn->dev != dev))
    {
      return 0;
    }

  info->delivered = true;
  if (devif_conn_event(dev, ICMPv6_NEWDATA, conn->sconn.list) !=
      ICMPv6_NEWDATA)
    {
      dev->d_len = dev->d_iob->io_pktlen;
    }
  else
    {
      icmpv6_datahandler(dev, conn, info->iplen);
    }

  return 0;
}

/****************************************************************************
 * Name: icmpv6_deliver
 *
 * Description:
 *   Copy the icmpv6 package to the application according to the filter
 *   conditions, but ICMPv6_ECHO_REPLY is a special message type, if there
 *   is an application waiting, it will also copy.
 *
 * Input Parameters:
 *   dev - Reference to a device driver structure.
 *   iplen - The size of the IPv6 header.  This may be larger than
 *           IPv6_HDRLEN the IPv6 header if IPv6 extension headers are
 *           present.
 *
 ****************************************************************************/

static bool icmpv6_deliver(FAR struct net_driver_s *dev, unsigned int iplen)
{
  struct icmpv6_deliver_s info;

  info.dev       = dev;
  info.iplen     = iplen;
  info.delivered = false;

  icmpv6_foreach(icmpv6_delivery_callback, &info);

  return info.delivered;
}

#endif /* CONFIG_NET_ICMPv6_SOCKET */

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
 *   dev   - The device driver structure containing the received ICMPv6
 *           packet
 *   iplen - The size of the IPv6 header.  This may be larger than
 *           IPv6_HDRLEN the IPv6 header if IPv6 extension headers are
 *           present.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmpv6_input(FAR struct net_driver_s *dev, unsigned int iplen)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct icmpv6_hdr_s *icmpv6 = IPBUF(iplen);
#ifdef CONFIG_NET_ICMPv6_SOCKET
  bool delivered = icmpv6_deliver(dev, iplen);
#endif

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.recv++;
#endif

  /* REVISIT:
   * - Verify that the message length is valid.
   * - Verify the ICMPv6 checksum
   */

  /* Handle the ICMPv6 message by its type */

  switch (icmpv6->type)
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
            if (sol->opttype == ICMPv6_OPT_SRCLLADDR)
              {
                /* Save the sender's address mapping in our Neighbor Table. */

                neighbor_add(dev, ipv6->srcipaddr, sol->srclladdr);
              }

            /* Yes.. Send a neighbor advertisement back to where the neighbor
             * solicitation came from.
             */

            icmpv6_advertise(dev, ipv6->srcipaddr);

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
         *   Should only update Neighbor Table if
         *     [O]verride bit is set in flags
         */

        adv = ICMPv6ADVERTISE;
        if (net_ipv6addr_cmp(ipv6->destipaddr, dev->d_ipv6addr))
          {
            /* This message is required to support the Target link-layer
             * address option.
             */

            if (adv->opttype == ICMPv6_OPT_TGTLLADDR)
              {
                /* Save the sender's address mapping in our Neighbor Table. */

                neighbor_add(dev, ipv6->srcipaddr, adv->tgtlladdr);
              }

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
            /* Then notify any logic waiting for the Neighbor Advertisement */

            icmpv6_notify(ipv6->srcipaddr);
#endif

            /* We consumed the packet but we don't send anything in
             * response.
             */

            goto icmpv6_send_nothing;
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
        bool prefix = false;
        uint16_t pktlen;
        uint16_t optlen;
        int ndx;

        /* Get the length of the option data */

        pktlen = (uint16_t)ipv6->len[0] << 8 | ipv6->len[1];
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
        options = (FAR uint8_t *)adv +
                   sizeof(struct icmpv6_router_advertise_s);

        for (ndx = 0; ndx < optlen; )
          {
           FAR struct icmpv6_generic_s *opt =
                                (FAR struct icmpv6_generic_s *)&options[ndx];

            switch (opt->opttype)
              {
                case ICMPv6_OPT_SRCLLADDR:
                  {
                    FAR struct icmpv6_srclladdr_s *sllopt =
                                      (FAR struct icmpv6_srclladdr_s *)opt;
                    neighbor_add(dev, ipv6->srcipaddr, sllopt->srclladdr);
                  }
                  break;

                case ICMPv6_OPT_PREFIX:
                  {
                    FAR struct icmpv6_prefixinfo_s *prefixopt =
                                      (FAR struct icmpv6_prefixinfo_s *)opt;

                    /* Is the "A" flag set? */

                    if ((prefixopt->flags & ICMPv6_PRFX_FLAG_A) != 0)
                      {
                        /* Yes.. Set the new network addresses. */

                        icmpv6_setaddresses(dev, ipv6->srcipaddr,
                                    prefixopt->prefix, prefixopt->preflen);

                        /* Notify any waiting threads */

                        icmpv6_rnotify(dev);
                        prefix = true;
                      }
                  }
                  break;

                case ICMPv6_OPT_MTU:
                  {
                    FAR struct icmpv6_mtu_s *mtuopt =
                                        (FAR struct icmpv6_mtu_s *)opt;
                    dev->d_pktsize = NTOHS(mtuopt->mtu[1]) + dev->d_llhdrlen;
                  }
                  break;

#ifdef CONFIG_ICMPv6_AUTOCONF_RDNSS
                case ICMPv6_OPT_RDNSS:
                  {
                    FAR struct icmpv6_rdnss_s *rdnss =
                                          (FAR struct icmpv6_rdnss_s *)opt;
                    FAR struct in6_addr *servers;
                    struct sockaddr_in6 addr;
                    int nservers;
                    int ret;
                    int i;

                    if (rdnss->optlen < 3)
                      {
                        nerr("rdnss error length %d\n", rdnss->optlen);
                        break;
                      }

                    /* optlen is in units of 8 bytes. The header is 1 unit
                     * (8 bytes) and each address is another 2 units
                     * (16 bytes). So the number of addresses is equal to
                     * (optlen - 1) / 2.
                     */

                    servers  = (FAR struct in6_addr *)rdnss->servers;
                    nservers = (rdnss->optlen - 1) / 2;

                    /* Set the IPv6 DNS server address */

                    memset(&addr, 0, sizeof(addr));
                    addr.sin6_family = AF_INET6;

                    for (i = 0; i < CONFIG_NETDB_DNSSERVER_NAMESERVERS &&
                         i < nservers; i++)
                      {
                        net_ipv6addr_copy(&addr.sin6_addr, servers + i);
                        ret = dns_add_nameserver(
                                          (FAR const struct sockaddr *)&addr,
                                          sizeof(struct sockaddr_in6));
                        if (ret < 0 && ret != -EEXIST)
                          {
                            nerr("dns add nameserver failed %d", ret);
                          }
                      }
                  }
                  break;
#endif

                default:
                  break;
              }

            /* Skip to the next option (units of octets) */

            ndx += (opt->optlen << 3);
          }

        if (prefix)
          {
            goto icmpv6_send_nothing;
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

        icmpv6->type = ICMPv6_ECHO_REPLY;

        net_ipv6addr_copy(ipv6->destipaddr, ipv6->srcipaddr);
        net_ipv6addr_copy(ipv6->srcipaddr, dev->d_ipv6addr);

        icmpv6->chksum = 0;
        icmpv6->chksum = ~icmpv6_chksum(dev, iplen);
      }
      break;

#if (CONFIG_NET_ICMPv6_PMTU_ENTRIES > 0)
    case ICMPv6_PACKET_TOO_BIG:
      {
        FAR struct icmpv6_pmtu_entry *entry;
        FAR struct ipv6_hdr_s *inner;
        int mtu;

        mtu = (ntohs(icmpv6->data[0]) << 16) | (ntohs(icmpv6->data[1]));
        if (mtu <= 0)
          {
            goto icmpv6_type_error;
          }

        inner = (FAR struct ipv6_hdr_s *)(icmpv6 + 1);
        entry = icmpv6_find_pmtu_entry(inner->destipaddr);
        if (entry == NULL)
          {
            icmpv6_add_pmtu_entry(inner->destipaddr, mtu);
          }
        else
          {
            entry->pmtu = mtu;
          }

        goto icmpv6_send_nothing;
      }
#endif

#ifdef CONFIG_NET_MLD
    /* Dispatch received Multicast Listener Discovery (MLD) packets. */

    case ICMPV6_MCAST_LISTEN_QUERY:      /* Multicast Listener Query, RFC 2710 and RFC 3810 */
      {
        FAR struct mld_mcast_listen_query_s *query = MLDQUERY;
        int ret;

        ret = mld_query(dev, query);
        if (ret < 0)
          {
            goto icmpv6_drop_packet;
          }
      }
      break;

    case ICMPV6_MCAST_LISTEN_REPORT_V1:  /* Version 1 Multicast Listener Report, RFC 2710 */
      {
        FAR struct mld_mcast_listen_report_v1_s *report = MLDREPORT_V1;
        int ret;

        ret = mld_report_v1(dev, report);
        if (ret < 0)
          {
            goto icmpv6_drop_packet;
          }
      }
      break;

    case ICMPV6_MCAST_LISTEN_REPORT_V2:  /* Version 2 Multicast Listener Report, RFC 3810 */
      {
        FAR struct mld_mcast_listen_report_v2_s *report = MLDREPORT_V2;
        int ret;

        ret = mld_report_v2(dev, report);
        if (ret < 0)
          {
            goto icmpv6_drop_packet;
          }
      }
      break;

    case ICMPV6_MCAST_LISTEN_DONE:       /* Multicast Listener Done, RFC 2710 */
      {
        FAR struct mld_mcast_listen_done_s *done = MLDDONE;
        int ret;

        ret = mld_done(dev, done);
        if (ret < 0)
          {
            goto icmpv6_drop_packet;
          }
      }
      break;
#endif

    default:
      {
#ifdef CONFIG_NET_ICMPv6_SOCKET
        if (delivered)
          {
            goto icmpv6_send_nothing;
          }
#endif

        nwarn("WARNING: Unknown ICMPv6 type: %d\n", icmpv6->type);
        goto icmpv6_type_error;
      }
    }

#ifdef CONFIG_NET_STATISTICS
  if (dev->d_len > 0)
    {
      ninfo("Outgoing ICMPv6 packet length: %d (%d)\n",
            dev->d_len, (ipv6->len[0] << 8) | ipv6->len[1]);

      g_netstats.icmpv6.sent++;
      g_netstats.ipv6.sent++;
    }
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
