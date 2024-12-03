/****************************************************************************
 * include/netinet/if_ether.h
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

#ifndef __INCLUDE_NETINET_IF_ETHER_H
#define __INCLUDE_NETINET_IF_ETHER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <net/if_arp.h>
#include <net/ethernet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETH_ALEN  6         /* Octets in one ethernet addr   */
#define ETH_TLEN  2         /* Octets in ethernet type field */
#define ETH_HLEN  14        /* Total octets in header.   */
#define ETH_ZLEN  60        /* Min. octets in frame sans FCS */
#define ETH_DATA_LEN  1500  /* Max. octets in payload  */
#define ETH_FRAME_LEN 1514  /* Max. octets in frame sans FCS */
#define ETH_FCS_LEN 4       /* Octets in the FCS     */

#define ETH_MIN_MTU 68      /* Min IPv4 MTU per RFC791  */
#define ETH_MAX_MTU 0xFFFFU /* 65535, same as IP_MAX_MTU  */

#define ETH_P_IP  ETHERTYPE_IP
#define ETH_P_ARP ETHERTYPE_ARP

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct ethhdr
{
  uint8_t  h_dest[ETH_ALEN];      /* destination eth addr    */
  uint8_t  h_source[ETH_ALEN];    /* source ether addr    */
  uint16_t h_proto;               /* packet type ID field    */
};

/* Ethernet Address Resolution Protocol.
 *
 * See RFC 826 for protocol description.  Structure below is adapted
 * to resolving internet addresses.  Field names used correspond to
 * RFC 826.
 */

struct ether_arp
{
  struct  arphdr ea_hdr;          /* fixed-size header */
  uint8_t arp_sha[ETH_ALEN];      /* sender hardware address */
  uint8_t arp_spa[4];             /* sender protocol address */
  uint8_t arp_tha[ETH_ALEN];      /* target hardware address */
  uint8_t arp_tpa[4];             /* target protocol address */
};

#define arp_hrd ea_hdr.ar_hrd
#define arp_pro ea_hdr.ar_pro
#define arp_hln ea_hdr.ar_hln
#define arp_pln ea_hdr.ar_pln
#define arp_op  ea_hdr.ar_op

#endif /* __INCLUDE_NETINET_IF_ETHER_H */
