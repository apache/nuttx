/****************************************************************************
 * net/mld/mld_send.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>
#include <arpa/inet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/ipv6ext.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "inet/inet.h"
#include "utils/utils.h"
#include "mld/mld.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug */

#ifdef CONFIG_NET_MLD_TXDUMP
#  define mld_dumppkt(b,n) lib_dumpbuffer("MLD", (FAR const uint8_t*)(b), (n))
#else
#  define mld_dumppkt(b,n)
#endif

/* IPv6 header size with extensions */

#define RASIZE      sizeof(struct ipv6_router_alert_s)
#define MLD_HDRLEN  (IPv6_HDRLEN + RASIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_send
 *
 * Description:
 *   Sends an MLD IP packet on a network interface. This function constructs
 *   the IP header and calculates the IP header checksum.
 *
 * Input Parameters:
 *   dev     - The device driver structure to use in the send operation.
 *   group   - Describes the multicast group member and identifies the
 *             message to be sent.
 *   msgtype - The type of the message to be sent (see enum mld_msgtype_e)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void mld_send(FAR struct net_driver_s *dev, FAR struct mld_group_s *group,
              uint8_t msgtype)
{
  FAR struct ipv6_router_alert_s *ra;
  FAR const uint16_t *destipaddr;
  unsigned int mldsize;

  /* Only a general query message can have a NULL group */

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(msgtype == MLD_SEND_GENQUERY || group != NULL);

  /* Select IPv6 */

  IFF_SET_IPv6(dev->d_flags);

  /* What is the size of the ICMPv6 payload? */

  switch (msgtype)
    {
      case MLD_SEND_GENQUERY:           /* Send General Query */
      case MLD_SEND_MASQUERY:           /* Send Multicast Address Specific (MAS) Query */
        {
          mldinfo("Send General/MAS Query, flags=%02x\n",
                  group != NULL ? group->flags : dev->d_mld.flags);
          mldsize = SIZEOF_MLD_MCAST_LISTEN_QUERY_S(0);
        }
        break;

      case MLD_SEND_V1REPORT:           /* Send MLDv1 Report message */
        {
          mldinfo("Send MLDv1 Report, flags=%02x\n", group->flags);
          mldsize = sizeof(struct mld_mcast_listen_report_v1_s);
        }
        break;

      case MLD_SEND_V2REPORT:           /* Send MLDv2 Report message */
        {
          unsigned int addreclen;

          mldinfo("Send MLDv2 Report, flags=%02x\n", group->flags);
          addreclen = SIZEOF_MLD_MCAST_ADDREC_V2_S(0, 0);
          mldsize   = SIZEOF_MLD_MCAST_LISTEN_REPORT_V2_S(addreclen);
        }
        break;

      case MLD_SEND_DONE:               /* Send Done message */
        {
          mldinfo("Send Done message, flags=%02x\n", group->flags);
          mldsize = sizeof(struct mld_mcast_listen_done_s);
        }
        break;

      default:
        {
          mlderr("Bad msgtype: %02x\n", msgtype);
          DEBUGPANIC();
        }

        return;
    }

  /* The total length to send is the size of the IPv6 header, 4 bytes for the
   * ROUTER ALERT, and the MLD ICMPv6 payload (and, eventually, the Ethernet
   * header length)
   */

  dev->d_len     = MLD_HDRLEN + mldsize;

  /* The total size of the data is the size of the ICMPv6 payload PLUS the
   * size of the IPv6 extension headers.
   */

  dev->d_sndlen  = RASIZE + mldsize;

  /* Update device buffer length */

  iob_update_pktlen(dev->d_iob, dev->d_len);

  /* Select the IPv6 destination address.
   * This varies with the type of message being sent:
   *
   *   MESSAGE                 DESTINATION ADDRESS
   *   General Query Message:  The link-local, all nodes multicast address.
   *   MAS Query Messages:     The group multicast address.
   *   V1 Report Message:      The group multicast address.
   *   V2 Report Message:      The link-local, all MLDv2 router multicast
   *                           address.
   *   Done Message:           The link-local, all routers multicast address.
   */

  switch (msgtype)
    {
      case MLD_SEND_GENQUERY:           /* Send General Query */
        destipaddr = g_ipv6_allnodes;
        break;

      case MLD_SEND_MASQUERY:           /* Send Multicast Address Specific (MAS) Query */
      case MLD_SEND_V1REPORT:           /* Send MLDv1 Report message */
        destipaddr = group->grpaddr;
        break;

      case MLD_SEND_V2REPORT:           /* Send MLDv2 Report message */
        destipaddr = g_ipv6_allmldv2routers;
        break;

      case MLD_SEND_DONE:               /* Send Done message */
        destipaddr = g_ipv6_allrouters;
        break;

      default:                          /* Can't happen, but eliminates a warning */
        return;
    }

  mldinfo("destipaddr: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          destipaddr[0], destipaddr[1], destipaddr[2], destipaddr[3],
          destipaddr[4], destipaddr[5], destipaddr[6], destipaddr[7]);

  ipv6_build_header(IPv6BUF, dev->d_sndlen, NEXT_HOPBYBOT_EH,
                    dev->d_ipv6addr, destipaddr, MLD_TTL);

  /* Add the router alert IP header option.
   *
   * The IPv6 router alert option (type 5) is defined in RFC 2711.
   */

  ra              = IPBUF(IPv6_HDRLEN);
  memset(ra, 0, RASIZE);

  ra->hbyh.nxthdr = IP_PROTO_ICMP6;          /* ICMPv6 payload follows extension header */
  ra->hbyh.len    = 1;                       /* One 8-octet option follows */
  ra->type        = HOPBYHOP_ROUTER_ALERT;   /* Router alert */

  /* Format the MLD ICMPv6 payload into place after the IPv6 header (with
   * router alert)
   */

  switch (msgtype)
    {
      case MLD_SEND_GENQUERY:           /* Send General Query */
      case MLD_SEND_MASQUERY:           /* Send Multicast Address Specific (MAS) Query */
        {
          FAR struct mld_mcast_listen_query_s *query = IPBUF(MLD_HDRLEN);

          /* Initialize the Query payload.  In a General Query, both the
           * Multicast Address field and the Number of Sources (N)
           * field are zero.
           *
           * Careful here.  In MLDv1 compatibility mode, the MRC is not
           * encoded and must follow the rules for MLDv1.
           */

          memset(query, 0, sizeof(struct mld_mcast_listen_query_s));
          query->type   = ICMPV6_MCAST_LISTEN_QUERY;
          query->mrc    = MLD_QRESP_MSEC;

          /* The General Query and the MAS Query differ only in that the
           * setting of the group multicast address field.  This field
           * is the unspecified address for General Query, but the group
           * multicast address for the MAS query.
           */

          if (msgtype == MLD_SEND_GENQUERY)
            {
              net_ipv6addr_hdrcopy(query->grpaddr, g_ipv6_unspecaddr);
            }
          else
            {
              net_ipv6addr_hdrcopy(query->grpaddr, group->grpaddr);
            }

          /* Fields unique to the extended MLDv2 query */

          if (!IS_MLD_V1COMPAT(dev->d_mld.flags))
            {
              query->flags  = MLD_ROBUSTNESS;
              query->qqic   = MLD_QRESP_SEC;
            }

          /* Calculate the ICMPv6 checksum. */

          query->chksum = 0;
          query->chksum = ~icmpv6_chksum(dev, MLD_HDRLEN);

          MLD_STATINCR(g_netstats.mld.query_sent);

#ifdef CONFIG_NET_MLD_ROUTER
          /* Save the number of members that reported in the previous query
           * cycle;  reset the number of members that have reported in the
           * new query cycle.
           */

          if (msgtype == MLD_SEND_GENQUERY)
            {
              /* Update accumulated membership for all groups. */

              mld_new_pollcycle(dev)
            }
          else
            {
              /* Updated accumulated membership only for this group */

              group->lstmbrs = group->members;
              group->members = 0;
            }
#endif
        }
        break;

      case MLD_SEND_V1REPORT:           /* Send MLDv1 Report message */
        {
          FAR struct mld_mcast_listen_report_v1_s *report =
                                                   IPBUF(MLD_HDRLEN);

          /* Initialize the Report payload */

          memset(report, 0, sizeof(struct mld_mcast_listen_report_v1_s));
          net_ipv6addr_hdrcopy(report->mcastaddr, &group->grpaddr);
          report->type    = ICMPV6_MCAST_LISTEN_REPORT_V1;

          /* Calculate the ICMPv6 checksum. */

          report->chksum  = 0;
          report->chksum  = ~icmpv6_chksum(dev, MLD_HDRLEN);

          SET_MLD_LASTREPORT(group->flags); /* Remember we were the last to report */
          MLD_STATINCR(g_netstats.mld.v1report_sent);
        }
        break;

      case MLD_SEND_V2REPORT:           /* Send MLDv2 Report message */
        {
          FAR struct mld_mcast_listen_report_v2_s *report =
                                                   IPBUF(MLD_HDRLEN);
          FAR struct mld_mcast_addrec_v2_s *addrec;

          /* Initialize the Report payload */

          memset(report, 0, mldsize);
          report->type    = ICMPV6_MCAST_LISTEN_REPORT_V2;
          report->naddrec = HTONS(1);

          addrec          = report->addrec;
          addrec->rectype = MODE_IS_INCLUDE;
          net_ipv6addr_hdrcopy(addrec->mcast, &group->grpaddr);

          /* Calculate the ICMPv6 checksum. */

          report->chksum  = 0;
          report->chksum  = ~icmpv6_chksum(dev, MLD_HDRLEN);

          SET_MLD_LASTREPORT(group->flags); /* Remember we were the last to report */
          MLD_STATINCR(g_netstats.mld.v2report_sent);
        }
        break;

      case MLD_SEND_DONE:               /* Send Done message */
        {
          FAR struct mld_mcast_listen_done_s *done = IPBUF(MLD_HDRLEN);

          /* Initialize the Done payload */

          memset(done, 0, sizeof(struct mld_mcast_listen_done_s));
          done->type      = ICMPV6_MCAST_LISTEN_DONE;
          net_ipv6addr_hdrcopy(done->mcastaddr, &group->grpaddr);

          /* Calculate the ICMPv6 checksum. */

          done->chksum    = 0;
          done->chksum    = ~icmpv6_chksum(dev, MLD_HDRLEN);

          MLD_STATINCR(g_netstats.mld.done_sent);
        }
        break;

      default:                          /* Can't happen, but eliminates a warning */
        return;
    }

  MLD_STATINCR(g_netstats.icmpv6.sent);
  MLD_STATINCR(g_netstats.ipv6.sent);

  mldinfo("Outgoing ICMPv6 MLD packet length: %d\n", dev->d_len);

  mld_dumppkt((FAR const uint8_t *)IPv6BUF, MLD_HDRLEN + mldsize);
}

/****************************************************************************
 * Name: mld_report_msgtype
 *
 * Description:
 *   Determine which type of Report to send, MLDv1 or MLDv2, depending on
 *   current state of compatibility mode flag.
 *
 ****************************************************************************/

uint8_t mld_report_msgtype(FAR struct net_driver_s *dev)
{
  if (IS_MLD_V1COMPAT(dev->d_mld.flags))
    {
      return MLD_SEND_V1REPORT;
    }
  else
    {
      return MLD_SEND_V2REPORT;
    }
}
