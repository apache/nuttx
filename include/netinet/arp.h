/****************************************************************************
 * include/netinet/arp.h
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

#ifndef __INCLUDE_NETINET_ARP_H
#define __INCLUDE_NETINET_ARP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <net/if.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Three ioctls are available on all PF_INET sockets. Each ioctl takes a
 * pointer to a 'struct arpreq' as its parameter.
 */

#define SIOCSARP        _ARPIOC(1) /* Set a ARP mapping */
#define SIOCDARP        _ARPIOC(2) /* Delete an ARP mapping */
#define SIOCGARP        _ARPIOC(3) /* Get an ARP mapping */

/* Definitions for bits in field arp_flags of struct arpreq.  If the
 * ATF_NETMASK flag is set, then arp_netmask should be valid.  This should
 * be set to 0xffffffff, or 0 to remove an existing arp entry.
 */

#define ATF_COM         (1 << 0)   /* Lookup complete */
#define ATF_PERM        (1 << 1)   /* Permanent entry */
#define ATF_PUBL        (1 << 2)   /* Publish entry */
#define ATF_USETRAILERS (1 << 3)   /* Trailers requested (obsolete) */
#define ATF_NETMASK     (1 << 4)   /* Use a netmask */
#define ATF_DONTPUB     (1 << 5)   /* Don't answer */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* All ARP ioctls take a pointer to a struct arpreq as their parameter: */

struct arpreq
{
  struct sockaddr arp_pa;                /* Protocol address */
  struct sockaddr arp_ha;                /* Hardware address */
  struct sockaddr arp_netmask;           /* Netmask of protocol address */
  uint8_t         arp_flags;             /* Flags */
  uint8_t         arp_dev[IFNAMSIZ + 1]; /* Device name (zero terminated) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NETINET_ARP_H */
