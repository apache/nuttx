/****************************************************************************
 * include/net/if_arp.h
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

#ifndef __INCLUDE_NET_IF_ARP_H
#define __INCLUDE_NET_IF_ARP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARP protocol HARDWARE identifiers. */

#define ARPHRD_NETROM     0             /* from KA9Q: NET/ROM pseudo    */
#define ARPHRD_ETHER      1             /* Ethernet                     */
#define ARPHRD_EETHER     2             /* Experimental Ethernet        */
#define ARPHRD_AX25       3             /* AX.25 Level 2                */
#define ARPHRD_PRONET     4             /* PROnet token ring            */
#define ARPHRD_CHAOS      5             /* Chaosnet                     */
#define ARPHRD_IEEE802    6             /* IEEE 802.2 Ethernet/TR/TB    */
#define ARPHRD_ARCNET     7             /* ARCnet                       */
#define ARPHRD_APPLETLK   8             /* APPLEtalk                    */
#define ARPHRD_DLCI       15            /* Frame Relay DLCI             */
#define ARPHRD_ATM        19            /* ATM                          */
#define ARPHRD_METRICOM   23            /* Metricom STRIP (new IANA id) */
#define ARPHRD_IEEE1394   24            /* IEEE 1394 IPv4 - RFC 2734    */
#define ARPHRD_EUI64      27            /* EUI-64                       */
#define ARPHRD_INFINIBAND 32            /* InfiniBand                   */

/* Dummy types for non ARP hardware */
#define ARPHRD_SLIP     256
#define ARPHRD_CSLIP    257
#define ARPHRD_SLIP6    258
#define ARPHRD_CSLIP6   259
#define ARPHRD_RSRVD    260             /* Notional KISS type           */
#define ARPHRD_ADAPT    264
#define ARPHRD_ROSE     270
#define ARPHRD_X25      271             /* CCITT X.25                   */
#define ARPHRD_HWX25    272             /* Boards with X.25 in firmware */
#define ARPHRD_CAN      280             /* Controller Area Network      */
#define ARPHRD_PPP      512
#define ARPHRD_CISCO    513             /* Cisco HDLC                   */
#define ARPHRD_HDLC     ARPHRD_CISCO
#define ARPHRD_LAPB     516             /* LAPB                         */
#define ARPHRD_DDCMP    517             /* Digital's DDCMP protocol     */
#define ARPHRD_RAWHDLC  518             /* Raw HDLC                     */
#define ARPHRD_RAWIP    519             /* Raw IP                       */
#define ARPHRD_TUNNEL   768             /* IPIP tunnel                  */
#define ARPHRD_TUNNEL6  769             /* IP6IP6 tunnel                */
#define ARPHRD_FRAD     770             /* Frame Relay Access Device    */
#define ARPHRD_SKIP     771             /* SKIP vif                     */
#define ARPHRD_LOOPBACK 772             /* Loopback device              */
#define ARPHRD_LOCALTLK 773             /* Localtalk device             */
#define ARPHRD_FDDI     774             /* Fiber Distributed Data Interface */
#define ARPHRD_BIF      775             /* AP1000 BIF                   */
#define ARPHRD_SIT      776             /* sit0 device - IPv6-in-IPv4   */
#define ARPHRD_IPDDP    777             /* IP over DDP tunneller        */
#define ARPHRD_IPGRE    778             /* GRE over IP                  */
#define ARPHRD_PIMREG   779             /* PIMSM register interface     */
#define ARPHRD_HIPPI    780             /* High Performance Parallel Interface */
#define ARPHRD_ASH      781             /* Nexus 64Mbps Ash             */
#define ARPHRD_ECONET   782             /* Acorn Econet                 */
#define ARPHRD_IRDA     783             /* Linux-IrDA                   */

/* ARP works differently on different FC media .. so  */

#define ARPHRD_FCPP     784             /* Point to point fibrechannel  */
#define ARPHRD_FCAL     785             /* Fibrechannel arbitrated loop */
#define ARPHRD_FCPL     786             /* Fibrechannel public loop     */
#define ARPHRD_FCFABRIC 787             /* Fibrechannel fabric          */

/* 787->799 reserved for fibrechannel media types */

#define ARPHRD_IEEE802_TR         800   /* Magic type ident for TR      */
#define ARPHRD_IEEE80211          801   /* IEEE 802.11                  */
#define ARPHRD_IEEE80211_PRISM    802   /* IEEE 802.11 + Prism2 header  */
#define ARPHRD_IEEE80211_RADIOTAP 803   /* IEEE 802.11 + radiotap header */
#define ARPHRD_IEEE802154         804   /* IEEE 802.15.4 */
#define ARPHRD_IEEE802154_MONITOR 805   /* IEEE 802.15.4 network monitor */

#define ARPHRD_PHONET      820          /* PhoNet media type            */
#define ARPHRD_PHONET_PIPE 821          /* PhoNet pipe header           */
#define ARPHRD_CAIF        822          /* CAIF media type              */
#define ARPHRD_IP6GRE      823          /* GRE over IPv6                */
#define ARPHRD_NETLINK     824          /* Netlink header               */
#define ARPHRD_6LOWPAN     825          /* IPv6 over LoWPAN             */
#define ARPHRD_VSOCKMON    826          /* Vsock monitor header         */

#define ARPHRD_VOID        0xFFFF       /* Void type, nothing is known  */
#define ARPHRD_NONE        0xFFFE       /* zero header length           */

/* ARP protocol opcodes. */

#define ARPOP_REQUEST   1               /* ARP request                  */
#define ARPOP_REPLY     2               /* ARP reply                    */
#define ARPOP_RREQUEST  3               /* RARP request                 */
#define ARPOP_RREPLY    4               /* RARP reply                   */
#define ARPOP_InREQUEST 8               /* InARP request                */
#define ARPOP_InREPLY   9               /* InARP reply                  */
#define ARPOP_NAK       10              /* (ATM)ARP NAK                 */

/* See RFC 826 for protocol description.  ARP packets are variable
 * in size; the arphdr structure defines the fixed-length portion.
 * Protocol type values are the same as those for 10 Mb/s Ethernet.
 * It is followed by the variable-sized fields ar_sha, arp_spa,
 * arp_tha and arp_tpa in that order, according to the lengths
 * specified.  Field names used correspond to RFC 826.
 */

struct arphdr
{
  unsigned short int ar_hrd;          /* Format of hardware address.  */
  unsigned short int ar_pro;          /* Format of protocol address.  */
  unsigned char ar_hln;               /* Length of hardware address.  */
  unsigned char ar_pln;               /* Length of protocol address.  */
  unsigned short int ar_op;           /* ARP opcode (command).  */
};

#endif /* __INCLUDE_NET_IF_ARP_H */
