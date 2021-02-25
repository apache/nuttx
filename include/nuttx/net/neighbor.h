/****************************************************************************
 * include/nuttx/net/neighbor.h
 * Definitions for use with IPv6 Neighbor Table
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

#ifndef __INCLUDE_NUTTX_NET_NEIGHBOR_H
#define __INCLUDE_NUTTX_NET_NEIGHBOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <net/ethernet.h>

#include <nuttx/clock.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_IPv6_NCONF_ENTRIES
#  define CONFIG_NET_IPv6_NCONF_ENTRIES 8
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Describes the link layer address */

struct neighbor_addr_s
{
  uint8_t                    na_lltype;  /* See enum net_lltype_e */
  uint8_t                    na_llsize;  /* Link layer header size */

  union
  {
#ifdef CONFIG_NET_ETHERNET
    struct ether_addr        na_ethernet;
#endif
#ifdef CONFIG_NET_6LOWPAN
    struct netdev_maxaddr_s  na_sixlowpan;
#endif
    uint8_t                  na_addr[1];
  } u;
};

/* This structure describes on entry in the neighbor table.  This is intended
 * for internal use within the Neighbor implementation.
 *
 * An unused entry can be detected by all zero entries, particularly the
 * ne_ipaddr (the IPv6 unspecified address) which should never be zero for a
 * real neighbor.
 */

struct neighbor_entry_s
{
  net_ipv6addr_t         ne_ipaddr;  /* IPv6 address of the Neighbor */
  struct neighbor_addr_s ne_addr;    /* Link layer address of the Neighbor */
  clock_t                ne_time;    /* For aging, units of tick */
};

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

#endif /* CONFIG_NET_LOOPBACK */
#endif /* __INCLUDE_NUTTX_NET_NEIGHBOR_H */
