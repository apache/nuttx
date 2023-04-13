/****************************************************************************
 * include/net/ethernet.h
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

#ifndef __INCLUDE_NET_ETHERNET_H
#define __INCLUDE_NET_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHER_ADDR_LEN  6

/* Ethernet protocol ID's */
#define ETHERTYPE_PUP      0x0200    /* Xerox PUP */
#define ETHERTYPE_SPRITE   0x0500    /* Sprite */
#define ETHERTYPE_IP       0x0800    /* IP */
#define ETHERTYPE_ARP      0x0806    /* Address resolution */
#define ETHERTYPE_REVARP   0x8035    /* Reverse ARP */
#define ETHERTYPE_AT       0x809B    /* AppleTalk protocol */
#define ETHERTYPE_AARP     0x80F3    /* AppleTalk ARP */
#define ETHERTYPE_VLAN     0x8100    /* IEEE 802.1Q VLAN tagging */
#define ETHERTYPE_IPX      0x8137    /* IPX */
#define ETHERTYPE_IPV6     0x86dd    /* IP protocol version 6 */
#define ETHERTYPE_LOOPBACK 0x9000    /* used to test interfaces */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct ether_addr
{
  uint8_t ether_addr_octet[6];            /* 48-bit Ethernet address */
};

struct ether_header
{
  uint8_t  ether_dhost[ETHER_ADDR_LEN];   /* Destination Ethernet address */
  uint8_t  ether_shost[ETHER_ADDR_LEN];   /* Source Ethernet address */
  uint16_t ether_type;                    /* Ethernet packet type */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NET_ETHERNET_H */
