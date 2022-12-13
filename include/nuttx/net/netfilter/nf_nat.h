/****************************************************************************
 * include/nuttx/net/netfilter/nf_nat.h
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

#ifndef __INCLUDE_NUTTX_NET_NETFILTER_NF_NAT_H
#define __INCLUDE_NUTTX_NET_NETFILTER_NF_NAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TABLE_NAME_NAT "nat"

/****************************************************************************
 * Public Types
 ****************************************************************************/

union nf_conntrack_man_proto
{
  /* Add other protocols here. */

  uint16_t all;

  struct
    {
      uint16_t port;
    } tcp;
  struct
    {
      uint16_t port;
    } udp;
  struct
    {
      uint16_t id;
    } icmp;
  struct
    {
      uint16_t port;
    } dccp;
  struct
    {
      uint16_t port;
    } sctp;
  struct
    {
      uint16_t key; /* GRE key is 32bit, PPtP only uses 16bit */
    } gre;
};

struct nf_nat_ipv4_range
{
  unsigned int                 flags;
  uint32_t                     min_ip;
  uint32_t                     max_ip;
  union nf_conntrack_man_proto min;
  union nf_conntrack_man_proto max;
};

struct nf_nat_ipv4_multi_range_compat
{
  unsigned int             rangesize;
  struct nf_nat_ipv4_range range[1];
};

#endif /* __INCLUDE_NUTTX_NET_NETFILTER_NF_NAT_H */
