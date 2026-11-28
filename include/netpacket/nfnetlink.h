/****************************************************************************
 * include/netpacket/nfnetlink.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NETPACKET_NFNETLINK_H
#define __INCLUDE_NETPACKET_NFNETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <netpacket/netlink.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Attribute definitions for struct nfattr **********************************/

#define NFNL_NFA_NEST                    0x8000
#define NFA_TYPE(attr)                   ((attr)->nfa_type & 0x7fff)

#define NFA_MASK                         (sizeof(uint32_t) - 1)
#define NFA_ALIGN(n)                     (((n) + NFA_MASK) & ~NFA_MASK)
#define NFA_OK(nfa, n)                   \
  ((n) >= (int)sizeof(struct nfattr) &&  \
   (nfa)->nfa_len >= sizeof(struct nfattr) && (nfa)->nfa_len <= (n))
#define NFA_NEXT(nfa, attrlen)           \
  ((attrlen) -= NFA_ALIGN((nfa)->nfa_len), \
   (FAR struct nfattr *)(((FAR char *)(nfa)) + \
                        NFA_ALIGN((nfa)->nfa_len)))
#define NFA_LENGTH(len)                  (sizeof(struct nfattr) + (len))
#define NFA_SPACE(len)                   NFA_ALIGN(NFA_LENGTH(len))
#define NFA_DATA(nfa)                    \
  ((FAR void *)(((FAR char *)(nfa)) + NFA_LENGTH(0)))
#define NFA_PAYLOAD(nfa)                 \
  ((int)((nfa)->nfa_len) - NFA_LENGTH(0))

/* Definitions for struct nfgenmsg ******************************************/

#define NFM_NFA(n)                       \
  ((FAR struct nfattr *)(((FAR char *)(n)) + \
                        NLMSG_ALIGN(sizeof(struct nfgenmsg))))
#define NFM_PAYLOAD(n)                   \
  NLMSG_PAYLOAD(n, sizeof(struct nfgenmsg))

#define NFNETLINK_V0                     0

#define NFNL_SUBSYS_ID(x)                (((x) & 0xff00) >> 8)
#define NFNL_MSG_TYPE(x)                 ((x) & 0x00ff)

#define NFNL_SUBSYS_NONE                 0
#define NFNL_SUBSYS_CTNETLINK            1
#define NFNL_SUBSYS_CTNETLINK_EXP        2
#define NFNL_SUBSYS_QUEUE                3
#define NFNL_SUBSYS_ULOG                 4
#define NFNL_SUBSYS_OSF                  5
#define NFNL_SUBSYS_IPSET                6
#define NFNL_SUBSYS_ACCT                 7
#define NFNL_SUBSYS_CTNETLINK_TIMEOUT    8
#define NFNL_SUBSYS_CTHELPER             9
#define NFNL_SUBSYS_NFTABLES             10
#define NFNL_SUBSYS_NFT_COMPAT           11
#define NFNL_SUBSYS_HOOK                 12
#define NFNL_SUBSYS_COUNT                13

#define IPCTNL_MSG_CT_NEW                0
#define IPCTNL_MSG_CT_GET                1
#define IPCTNL_MSG_CT_DELETE             2
#define IPCTNL_MSG_CT_GET_CTRZERO        3
#define IPCTNL_MSG_CT_GET_STATS_CPU      4
#define IPCTNL_MSG_CT_GET_STATS          5
#define IPCTNL_MSG_CT_GET_DYING          6
#define IPCTNL_MSG_CT_GET_UNCONFIRMED    7
#define IPCTNL_MSG_MAX                   8

#define CTA_UNSPEC                       0
#define CTA_TUPLE_ORIG                   1
#define CTA_TUPLE_REPLY                  2
#define CTA_STATUS                       3
#define CTA_PROTOINFO                    4
#define CTA_HELP                         5
#define CTA_NAT_SRC                      6
#define CTA_TIMEOUT                      7
#define CTA_MARK                         8
#define CTA_COUNTERS_ORIG                9
#define CTA_COUNTERS_REPLY               10
#define CTA_USE                          11
#define CTA_ID                           12
#define CTA_NAT_DST                      13
#define CTA_TUPLE_MASTER                 14
#define CTA_SEQ_ADJ_ORIG                 15
#define CTA_NAT_SEQ_ADJ_ORIG             CTA_SEQ_ADJ_ORIG
#define CTA_SEQ_ADJ_REPLY                16
#define CTA_NAT_SEQ_ADJ_REPLY            CTA_SEQ_ADJ_REPLY
#define CTA_ZONE                         18
#define CTA_SECCTX                       19
#define CTA_TIMESTAMP                    20
#define CTA_MARK_MASK                    21
#define CTA_LABELS                       22
#define CTA_LABELS_MASK                  23
#define CTA_SYNPROXY                     24
#define CTA_FILTER                       25
#define CTA_STATUS_MASK                  26
#define CTA_MAX                          26

#define CTA_TUPLE_UNSPEC                 0
#define CTA_TUPLE_IP                     1
#define CTA_TUPLE_PROTO                  2
#define CTA_TUPLE_ZONE                   3
#define CTA_TUPLE_MAX                    3

#define CTA_IP_UNSPEC                    0
#define CTA_IP_V4_SRC                    1
#define CTA_IP_V4_DST                    2
#define CTA_IP_V6_SRC                    3
#define CTA_IP_V6_DST                    4
#define CTA_IP_MAX                       4

#define CTA_PROTO_UNSPEC                 0
#define CTA_PROTO_NUM                    1
#define CTA_PROTO_SRC_PORT               2
#define CTA_PROTO_DST_PORT               3
#define CTA_PROTO_ICMP_ID                4
#define CTA_PROTO_ICMP_TYPE              5
#define CTA_PROTO_ICMP_CODE              6
#define CTA_PROTO_ICMPV6_ID              7
#define CTA_PROTO_ICMPV6_TYPE            8
#define CTA_PROTO_ICMPV6_CODE            9
#define CTA_PROTO_MAX                    9

#define NF_NETLINK_CONNTRACK_NEW         0x00000001
#define NF_NETLINK_CONNTRACK_UPDATE      0x00000002
#define NF_NETLINK_CONNTRACK_DESTROY     0x00000004
#define NF_NETLINK_CONNTRACK_EXP_NEW     0x00000008
#define NF_NETLINK_CONNTRACK_EXP_UPDATE  0x00000010
#define NF_NETLINK_CONNTRACK_EXP_DESTROY 0x00000020

#define NFNLGRP_NONE                     0
#define NFNLGRP_CONNTRACK_NEW            1
#define NFNLGRP_CONNTRACK_UPDATE         2
#define NFNLGRP_CONNTRACK_DESTROY        3
#define NFNLGRP_CONNTRACK_EXP_NEW        4
#define NFNLGRP_CONNTRACK_EXP_UPDATE     5
#define NFNLGRP_CONNTRACK_EXP_DESTROY    6
#define NFNLGRP_NFTABLES                 7
#define NFNLGRP_ACCT_QUOTA               8
#define NFNLGRP_NFTRACE                  9
#define NFNLGRP_MAX                      9

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct nfattr
{
  uint16_t nfa_len;
  uint16_t nfa_type;
};

struct nfgenmsg
{
  uint8_t  nfgen_family;
  uint8_t  version;
  uint16_t res_id;
};

#endif /* __INCLUDE_NETPACKET_NFNETLINK_H */
