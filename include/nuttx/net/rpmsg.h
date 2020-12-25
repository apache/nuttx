/****************************************************************************
 * include/nuttx/net/rpmsg.h
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

#ifndef __INCLUDE_NUTTX_NET_RPMSG_H
#define __INCLUDE_NUTTX_NET_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NET_RPMSG_EPT_NAME              "rpmsg-%s"

#define NET_RPMSG_IFUP                  0 /* IP-->LINK */
#define NET_RPMSG_IFDOWN                1 /* IP-->LINK */
#define NET_RPMSG_ADDMCAST              2 /* IP-->LINK */
#define NET_RPMSG_RMMCAST               3 /* IP-->LINK */
#define NET_RPMSG_DEVIOCTL              4 /* IP-->LINK */
#define NET_RPMSG_SOCKIOCTL             5 /* IP<--LINK */
#define NET_RPMSG_TRANSFER              6 /* IP<->LINK */

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct net_rpmsg_lnkaddr_s
{
  uint32_t length;
  uint8_t  addr[16];
} end_packed_struct;

begin_packed_struct struct net_rpmsg_header_s
{
  uint32_t command;
  int32_t  result;
  uint64_t cookie;
} end_packed_struct;

begin_packed_struct struct net_rpmsg_ifup_s
{
  struct net_rpmsg_header_s  header;
  struct net_rpmsg_lnkaddr_s lnkaddr;

  /* All addresses in the network order */

  uint32_t                   ipaddr;
  uint32_t                   draddr;
  uint32_t                   netmask;
  uint32_t                   dnsaddr;
  uint16_t                   ipv6addr[8];
  uint16_t                   ipv6draddr[8];
  uint16_t                   ipv6netmask[8];
  uint16_t                   ipv6dnsaddr[8];
} end_packed_struct;

#define net_rpmsg_ifdown_s net_rpmsg_header_s

begin_packed_struct struct net_rpmsg_mcast_s
{
  struct net_rpmsg_header_s  header;
  struct net_rpmsg_lnkaddr_s lnkaddr;
} end_packed_struct;

begin_packed_struct struct net_rpmsg_ioctl_s
{
  struct net_rpmsg_header_s header;
  uint32_t                  code;
  uint32_t                  length;
  uint8_t                   arg[0];
} end_packed_struct;

begin_packed_struct struct net_rpmsg_transfer_s
{
  struct net_rpmsg_header_s header;
  uint32_t                  length;
  uint8_t                   data[0];
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_NET_RPMSG_H */
