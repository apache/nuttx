/****************************************************************************
 * net/mld/mld_send.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <debug.h>
#include <arpa/inet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/ipopt.h>
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

#undef MLD_DUMPPKT       /* Define to enable packet dump */

#ifndef CONFIG_DEBUG_NET
#  undef MLD_DUMPPKT
#endif

#ifdef MLD_DUMPPKT
#  define mld_dumppkt(b,n) lib_dumpbuffer("MLD", (FAR const uint8_t*)(b), (n))
#else
#  define mld_dumppkt(b,n)
#endif

/* Buffer layout */

#define IPv6BUF    ((FAR struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define RABUF      ((FAR uint8_t *)&dev->d_buf[NET_LL_HDRLEN(dev)] + \
                    IPv6_HDRLEN)
#define RASIZE     (4 * sizeof(uint16_t))
#define REPORTBUF  ((FAR struct mld_mcast_listen_report_v1_s *) \
                    &dev->d_buf[NET_LL_HDRLEN(dev)] + IPv6_HDRLEN + RASIZE)
#define DONEBUF    ((FAR struct mld_mcast_listen_done_v1_s *) \
                    &dev->d_buf[NET_LL_HDRLEN(dev)] + IPv6_HDRLEN + RASIZE)

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
 *   dev        - The device driver structure to use in the send operation.
 *   group      - Describes the multicast group member and identifies the
 *                message to be sent.
 *   destipaddr - The IP address of the recipient of the message
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void mld_send(FAR struct net_driver_s *dev, FAR struct mld_group_s *group,
              FAR const net_ipv6addr_t destipaddr)
{
  FAR struct ipv6_hdr_s *ipv6;
  FAR uint8_t *ra;
  unsigned int mldsize;

  ninfo("msgid: %02x \n", group->msgid);
  ninfo("destipaddr: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        destipaddr[0], destipaddr[1], destipaddr[2], destipaddr[3],
        destipaddr[4], destipaddr[5], destipaddr[6], destipaddr[7]);

  /* Select IPv6 */

  IFF_SET_IPv6(dev->d_flags);

  /* What is the size of the ICMPv6 payload?  Currently only Version 1
   * REPORT and DONE packets are sent (these are actually the same size).
   * This will change.
   */

  switch (group->msgid)
    {
      case ICMPV6_MCAST_LISTEN_REPORT_V1:
        mldsize = sizeof(struct mld_mcast_listen_report_v1_s);
        break;

      case ICMPV6_MCAST_LISTEN_DONE_V1:
        mldsize = sizeof(struct mld_mcast_listen_done_v1_s);
        break;

      /* Not yet supported */

      case ICMPV6_MCAST_LISTEN_QUERY:
      case ICMPV6_MCAST_LISTEN_REPORT_V2:
      default:
        DEBUGPANIC();
        return;
    }

  /* The total length to send is the size of the IPv6 header, 4 bytes for the
   * ROUTER ALERT, and the MLD ICMPv6 payload (and, eventually, the Ethernet
   * header length)
   */

  dev->d_len     = IPv6_HDRLEN + RASIZE + mldsize;

  /* The total size of the data is the size of the ICMPv6 payload */

  dev->d_sndlen  = mldsize;

  /* Set up the IPv6 header */

  ipv6           = IPv6BUF;
  ipv6->vtc      = 0x60;                     /* Version/traffic class (MS) */
  ipv6->tcf      = 0;                        /* Traffic class(LS)/Flow label(MS) */
  ipv6->flow     = 0;                        /* Flow label (LS) */
  ipv6->len[0]   = (dev->d_sndlen >> 8);     /* Length excludes the IPv6 header */
  ipv6->len[1]   = (dev->d_sndlen & 0xff);
  ipv6->proto    = IP_PROTO_ICMP6;           /* ICMPv6 payload */
  ipv6->ttl      = MLD_TTL;                  /* MLD Time-to-live */

  net_ipv6addr_hdrcopy(ipv6->srcipaddr, dev->d_ipv6addr);
  net_ipv6addr_hdrcopy(ipv6->destipaddr, destipaddr);

  /* Add the router alert IP header option.
   *
   * The IPv6 router alert option (type 5) is defined in RFC 2711.
   * REVISIT:  This should go into a new header file ipv6opt.h
   */

  ra             = RABUF;
  ra[0]          = 5;                        /* Option type */
  ra[1]          = 2;                        /* Option length */
  ra[2]          = 0;                        /* Router alert value */
  ra[3]          = 0;

  /* Format the MLD ICMPv6 payload into place after the IPv6 header (with
   * router alert)
   */

  switch (group->msgid)
    {
      case ICMPV6_MCAST_LISTEN_REPORT_V1:
        {
          FAR struct mld_mcast_listen_report_v1_s *report = REPORTBUF;

          /* Initializer the Report payload */

          memset(report, 0, sizeof(struct mld_mcast_listen_report_v1_s));
          report->type    = ICMPV6_MCAST_LISTEN_REPORT_V1;
          net_ipv6addr_hdrcopy(report->mcastaddr, &group->grpaddr);

          /* Calculate the ICMPv6 checksum. */

          report->chksum  = 0;
          report->chksum  = ~icmpv6_chksum(dev);

          MLD_STATINCR(g_netstats.mld.report_sent);
        }
        break;

      case ICMPV6_MCAST_LISTEN_DONE_V1:
        {
          FAR struct mld_mcast_listen_done_v1_s *done = DONEBUF;

          /* Initializer the Done payload */

          memset(done, 0, sizeof(struct mld_mcast_listen_done_v1_s));
          done->type      = ICMPV6_MCAST_LISTEN_DONE_V1;
          net_ipv6addr_hdrcopy(done->mcastaddr, &group->grpaddr);

          /* Calculate the ICMPv6 checksum. */

          done->chksum    = 0;
          done->chksum    = ~icmpv6_chksum(dev);

          MLD_STATINCR(g_netstats.mld.done_sent);
        }
        break;

      /* Not yet supported */

      case ICMPV6_MCAST_LISTEN_QUERY:
      case ICMPV6_MCAST_LISTEN_REPORT_V2:
      default:
        DEBUGPANIC();
        return;
    }

  MLD_STATINCR(g_netstats.icmpv6.sent);
  MLD_STATINCR(g_netstats.ipv6.sent);

  ninfo("Outgoing ICMPv6 MLD packet length: %d (%d)\n",
        dev->d_len, (ipv6->len[0] << 8) | ipv6->len[1]);

  mld_dumppkt(RA, IPMLD_HDRLEN + RASIZE);
}
