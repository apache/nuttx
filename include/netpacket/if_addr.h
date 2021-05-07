/****************************************************************************
 * include/netpacket/if_addr.h
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

#ifndef __INCLUDE_NETPACKET_IF_ADDR_H
#define __INCLUDE_NETPACKET_IF_ADDR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <netpacket/netlink.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IFA_MAX (__IFA_MAX - 1)

/* ifa_flags */

#define IFA_F_SECONDARY       0x01
#define IFA_F_TEMPORARY       IFA_F_SECONDARY

#define IFA_F_NODAD           0x02
#define IFA_F_OPTIMISTIC      0x04
#define IFA_F_DADFAILED       0x08
#define IFA_F_HOMEADDRESS     0x10
#define IFA_F_DEPRECATED      0x20
#define IFA_F_TENTATIVE       0x40
#define IFA_F_PERMANENT       0x80
#define IFA_F_MANAGETEMPADDR 0x100
#define IFA_F_NOPREFIXROUTE  0x200
#define IFA_F_MCAUTOJOIN     0x400
#define IFA_F_STABLE_PRIVACY 0x800

/* backwards compatibility for userspace */

#define IFA_RTA(r)  ((FAR struct rtattr *)(((FAR char *)(r)) + \
                      NLMSG_ALIGN(sizeof(struct ifaddrmsg))))
#define IFA_PAYLOAD(n) NLMSG_PAYLOAD(n, sizeof(struct ifaddrmsg))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* RTM_NEWADDR, RTM_DELADDR, RTM_GETADDR
 *
 * Add, remove or receive information about an IP address associated with
 * an interface.  These messages contain an ifaddrmsg structure, optionally
 * followed by rtattr routing attributes.
 */

struct ifaddrmsg
{
  uint8_t  ifa_family;    /* Address type:  AF_INET or AF_INET6 */
  uint8_t  ifa_prefixlen; /* Prefix length of address */
  uint8_t  ifa_flags;     /* Address flags.  See IFA_F_* definitions */
  uint8_t  ifa_scope;     /* Address scope */
  int32_t  ifa_index;     /* Unique interface index */
};

/* Important comment:
 * IFA_ADDRESS is prefix address, rather than local interface address.
 * It makes no difference for normally configured broadcast interfaces,
 * but for point-to-point IFA_ADDRESS is DESTINATION address,
 * local address is supplied in IFA_LOCAL attribute.
 *
 * IFA_FLAGS is a u32 attribute that extends the u8 field ifa_flags.
 * If present, the value from struct ifaddrmsg will be ignored.
 */

enum
{
  IFA_UNSPEC,
  IFA_ADDRESS,
  IFA_LOCAL,
  IFA_LABEL,
  IFA_BROADCAST,
  IFA_ANYCAST,
  IFA_CACHEINFO,
  IFA_MULTICAST,
  IFA_FLAGS,
  __IFA_MAX,
};

struct ifa_cacheinfo
{
  uint32_t ifa_prefered;
  uint32_t ifa_valid;
  uint32_t cstamp; /* created timestamp, hundredths of seconds */
  uint32_t tstamp; /* updated timestamp, hundredths of seconds */
};

#endif /* __INCLUDE_NETPACKET_IF_ADDR_H */
