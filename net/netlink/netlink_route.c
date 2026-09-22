/****************************************************************************
 * net/netlink/netlink_route.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <arpa/inet.h>

#include <net/route.h>
#include <netpacket/netlink.h>
#include <netinet/if_ether.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/neighbor.h>
#include <nuttx/net/netlink.h>
#ifdef CONFIG_NET_CANFORWARD
#include <nuttx/net/cangw.h>
#endif
#include "netdev/netdev.h"
#include "arp/arp.h"
#include "inet/inet.h"
#include "net/if_arp.h"
#include "neighbor/neighbor.h"
#include "route/route.h"
#include "netlink/netlink.h"
#include "utils/utils.h"

#ifdef CONFIG_NETLINK_ROUTE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_NET_ARP) && !defined(CONFIG_NET_IPv6)
#  undef CONFIG_NETLINK_DISABLE_GETNEIGH
#  define CONFIG_NETLINK_DISABLE_GETNEIGH 1
#endif

#if !defined(CONFIG_NET_ROUTE) || (!defined(CONFIG_NET_IPv4) && \
    !defined(CONFIG_NET_IPv6))
#  undef CONFIG_NETLINK_DISABLE_GETROUTE
#  define CONFIG_NETLINK_DISABLE_GETROUTE 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* RTM_GETLINK:  Enumerate network devices */

struct getlink_recvfrom_response_s
{
  struct nlmsghdr  hdr;
  struct ifinfomsg iface;
  struct rtattr    attrmtu;
  uint32_t         mtu;                         /* IFLA_MTU attribute */
#if defined(CONFIG_NET_ETHERNET) || defined(CONFIG_NET_TUN)
  struct rtattr    attraddr;
  uint8_t          mac[NLMSG_ALIGN(ETH_ALEN)];  /* IFLA_ADDRESS attribute */
#endif
  struct rtattr    attrname;
  uint8_t          data[IFNAMSIZ];              /* IFLA_IFNAME attribute */
};

struct getlink_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct getlink_recvfrom_response_s payload;
};

/* RTM_GETNEIGH:  Get neighbor table entry */

struct getneigh_recvfrom_response_s
{
  struct nlmsghdr hdr;
  struct ndmsg    msg;
  struct rtattr   attr;
  uint8_t         data[1];
};

#define SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(n) \
  (sizeof(struct getneigh_recvfrom_response_s) + (n) - 1)

struct getneigh_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct getneigh_recvfrom_response_s payload;
};

#define SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(n) \
  (sizeof(struct getneigh_recvfrom_rsplist_s) + (n) - 1)

/* RTM_GETROUTE.  Get routing tables */

struct getroute_recvfrom_ipv4addr_s
{
  struct rtattr attr;
  in_addr_t     addr;
};

struct getroute_recvfrom_ipv4response_s
{
  struct nlmsghdr hdr;
  struct rtmsg    rte;
  struct getroute_recvfrom_ipv4addr_s dst;
  struct getroute_recvfrom_ipv4addr_s genmask;
  struct getroute_recvfrom_ipv4addr_s gateway;
};

struct getroute_recvfrom_ipv4resplist_s
{
  sq_entry_t flink;
  struct getroute_recvfrom_ipv4response_s payload;
};

struct getroute_recvfrom_ipv6addr_s
{
  struct rtattr  attr;
  net_ipv6addr_t addr;
};

struct getroute_recvfrom_ipv6response_s
{
  struct nlmsghdr hdr;
  struct rtmsg    rte;
  struct getroute_recvfrom_ipv6addr_s dst;
  struct getroute_recvfrom_ipv6addr_s genmask;
  struct getroute_recvfrom_ipv6addr_s gateway;
};

struct getroute_recvfrom_ipv6resplist_s
{
  sq_entry_t flink;
  struct getroute_recvfrom_ipv6response_s payload;
};

/* RTM_GETADDR:  Get the specified network device address info */

struct getaddr_recvfrom_response_s
{
  struct nlmsghdr  hdr;
  struct ifaddrmsg ifaddr;
  struct rtattr    attr;
#ifndef CONFIG_NET_IPv6
  struct in_addr   local_addr;  /* IFA_LOCAL is the only attribute supported */
#else
  struct in6_addr  local_addr;  /* IFA_LOCAL is the only attribute supported */
#endif
};

struct getaddr_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct getaddr_recvfrom_response_s payload;
};

struct getprefix_recvfrom_addr_s
{
  struct rtattr  attr;
  net_ipv6addr_t addr;
};

struct getprefix_recvfrom_cache_s
{
  struct rtattr           attr;
  struct prefix_cacheinfo pci;
};

struct getprefix_recvfrom_response_s
{
  struct nlmsghdr  hdr;
  struct prefixmsg pmsg;
  struct getprefix_recvfrom_addr_s  prefix;
  struct getprefix_recvfrom_cache_s pci;
};

struct getprefix_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct getprefix_recvfrom_response_s payload;
};

/* RTM_NEWROUTE:  response the ack of adding a new route */

struct new_ccgw_rule_response_s
{
  struct nlmsghdr hdr;
  struct nlmsgerr err;
};

struct new_ccgw_rule_resplist_s
{
  sq_entry_t flink;
  struct new_ccgw_rule_response_s payload;
};

/* netdev_foreach() callback */

struct nlroute_sendto_request_s
{
  struct nlmsghdr hdr;
  struct rtgenmsg gen;
};

struct nlroute_info_s
{
  NETLINK_HANDLE handle;
  FAR const struct nlroute_sendto_request_s *req;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NETLINK_VALIDATE_POLICY
#  ifdef CONFIG_NET_IPv4
static const struct nla_policy g_ifa_ipv4_policy[] =
{
  {0},                                              /* IFA_UNSPEC */
  {NLA_U32, 0, NULL},                               /* IFA_ADDRESS */
  {NLA_U32, 0, NULL},                               /* IFA_LOCAL */
  {NLA_STRING, IFNAMSIZ - 1, NULL},                 /* IFA_LABEL */
  {NLA_U32, 0, NULL},                               /* IFA_BROADCAST */
  {0},                                              /* IFA_ANYCAST */
  {NLA_UNSPEC, sizeof(struct ifa_cacheinfo), NULL}, /* IFA_CACHEINFO */
  {0},                                              /* IFA_MULTICAST */
  {NLA_U32, 0, NULL},                               /* IFA_FLAGS */
  {NLA_U32, 0, NULL},                               /* IFA_RT_PRIORITY */
};

static_assert(nitems(g_ifa_ipv4_policy) ==
              IFA_MAX + 1, "The policy definition has changed,"
              " please check it");
#  endif
#  ifdef CONFIG_NET_IPv6
static const struct nla_policy g_ifa_ipv6_policy[] =
{
  {0},                                              /* IFA_UNSPEC */
  {0, sizeof(struct in6_addr), NULL},               /* IFA_ADDRESS */
  {0, sizeof(struct in6_addr), NULL},               /* IFA_LOCAL */
  {0},                                              /* IFA_LABEL */
  {0},                                              /* IFA_BROADCAST */
  {0},                                              /* IFA_ANYCAST */
  {NLA_UNSPEC, sizeof(struct ifa_cacheinfo), NULL}, /* IFA_CACHEINFO */
  {0},                                              /* IFA_MULTICAST */
  {0, sizeof(uint32_t), NULL},                      /* IFA_FLAGS */
  {0, sizeof(uint32_t), NULL},                      /* IFA_RT_PRIORITY */
};

static_assert(nitems(g_ifa_ipv6_policy) ==
              IFA_MAX + 1, "The policy definition has changed,"
              " please check it");
#  endif
#else
#  define g_ifa_ipv4_policy NULL
#  define g_ifa_ipv6_policy NULL
#endif

#if (!defined(CONFIG_NETLINK_DISABLE_NEWROUTE) || \
     !defined(CONFIG_NETLINK_DISABLE_DELROUTE)) && \
    defined(CONFIG_NET_CANFORWARD)
const struct nla_policy g_cgw_policy[CGW_MAX + 1] =
{
  [CGW_MOD_AND] =
    {
      .len = sizeof(struct cgw_frame_mod)
    },

  [CGW_MOD_OR] =
    {
      .len = sizeof(struct cgw_frame_mod)
    },

  [CGW_MOD_XOR] =
    {
      .len = sizeof(struct cgw_frame_mod)
    },

  [CGW_MOD_SET] =
    {
      .len = sizeof(struct cgw_frame_mod)
    },

  [CGW_CS_XOR] =
    {
      .len = sizeof(struct cgw_csum_xor)
    },

  [CGW_CS_CRC8] =
    {
      .len = sizeof(struct cgw_csum_crc8)
    },

  [CGW_SRC_IF] =
    {
      .type = NLA_U32
    },

  [CGW_DST_IF] =
    {
      .type = NLA_U32
    },

  [CGW_FILTER] =
    {
      .len = sizeof(struct can_filter)
    },

  [CGW_LIM_HOPS] =
    {
      .type = NLA_U8
    },

  [CGW_MOD_UID] =
    {
      .type = NLA_U32
    },

  [CGW_FDMOD_AND] =
    {
      .len = sizeof(struct cgw_fdframe_mod)
    },

  [CGW_FDMOD_OR] =
    {
      .len = sizeof(struct cgw_fdframe_mod)
    },

  [CGW_FDMOD_XOR] =
    {
      .len = sizeof(struct cgw_fdframe_mod)
    },

  [CGW_FDMOD_SET] =
    {
      .len = sizeof(struct cgw_fdframe_mod)
    },
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_get_device
 *
 * Description:
 *   Generate one device response.
 *
 * Input Parameters:
 *   dev  - The network device to report on.
 *   type - The netlink message type to report, either RTM_NEWLINK or
 *          RTM_DELLINK.  The caller decides this because the meaning
 *          differs per usage:  a RTM_GETLINK dump always reports
 *          RTM_NEWLINK (link state is carried by ifi_flags), while an
 *          RTNLGRP_LINK broadcast reports the actual link event.
 *   req  - The originating request, or NULL for an unsolicited broadcast.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK

static uint16_t netlink_convert_device_type(uint8_t lltype)
{
  switch (lltype)
    {
      case NET_LL_ETHERNET:
        return ARPHRD_ETHER;

      case NET_LL_IEEE80211:
        return ARPHRD_IEEE80211;

      case NET_LL_LOOPBACK:
        return ARPHRD_LOOPBACK;

      case NET_LL_SLIP:
        return ARPHRD_SLIP;

      case NET_LL_TUN:
      case NET_LL_BLUETOOTH:
      case NET_LL_PKTRADIO:
      case NET_LL_MBIM:
        return ARPHRD_NONE;

      case NET_LL_IEEE802154:
        return ARPHRD_IEEE802154;

      case NET_LL_CAN:
        return ARPHRD_CAN;

      case NET_LL_CELL:
        return ARPHRD_PHONET_PIPE;

      default:
        nerr("ERROR: invalid lltype %d\n", lltype);
        return ARPHRD_VOID;
    }
}

static FAR struct netlink_response_s *
netlink_get_device(FAR struct net_driver_s *dev, int type,
                   FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getlink_recvfrom_rsplist_s *alloc;
  FAR struct getlink_recvfrom_response_s *resp;

  /* Allocate the response buffer */

  alloc = (FAR struct getlink_recvfrom_rsplist_s *)
    kmm_zalloc(sizeof(struct getlink_recvfrom_rsplist_s));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return NULL;
    }

  /* Initialize the response buffer */

  resp                   = &alloc->payload;

  resp->hdr.nlmsg_len    = sizeof(struct getlink_recvfrom_response_s);
  resp->hdr.nlmsg_type   = type;
  resp->hdr.nlmsg_flags  = req ? req->hdr.nlmsg_flags : 0;
  resp->hdr.nlmsg_seq    = req ? req->hdr.nlmsg_seq : 0;
  resp->hdr.nlmsg_pid    = req ? req->hdr.nlmsg_pid : 0;

  resp->iface.ifi_family = req ? req->gen.rtgen_family : AF_PACKET;
  resp->iface.ifi_type   = netlink_convert_device_type(dev->d_lltype);
#ifdef CONFIG_NETDEV_IFINDEX
  resp->iface.ifi_index  = dev->d_ifindex;
#endif
  resp->iface.ifi_flags  = dev->d_flags;
  resp->iface.ifi_change = 0xffffffff;

  resp->attrmtu.rta_len  = RTA_LENGTH(sizeof(uint32_t));
  resp->attrmtu.rta_type = IFLA_MTU;
  resp->mtu              = NETDEV_PKTSIZE(dev) - NET_LL_HDRLEN(dev);

#if defined(CONFIG_NET_ETHERNET) || defined(CONFIG_NET_TUN)
  resp->attraddr.rta_len  = RTA_LENGTH(ETH_ALEN);
  resp->attraddr.rta_type = IFLA_ADDRESS;
  if (dev->d_lltype == NET_LL_ETHERNET ||
      dev->d_lltype == NET_LL_IEEE80211 ||
      dev->d_lltype == NET_LL_LOOPBACK ||
      dev->d_lltype == NET_LL_TUN)
    {
      memcpy(&resp->mac, &dev->d_mac.ether, ETH_ALEN);
    }
  else
    {
      memset(&resp->mac, 0, ETH_ALEN);
    }
#endif

  resp->attrname.rta_len  = RTA_LENGTH(strnlen(dev->d_ifname, IFNAMSIZ));
  resp->attrname.rta_type = IFLA_IFNAME;

  strlcpy((FAR char *)resp->data, dev->d_ifname, IFNAMSIZ);

  /* Finally, return the response */

  return (FAR struct netlink_response_s *)alloc;
}
#endif

#ifdef CONFIG_NET_IPv4
static uint32_t make_mask(int prefixlen)
{
  if (prefixlen)
    {
      return HTONL(UINT32_MAX << (32 - prefixlen));
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: netlink_get_ifaddr
 *
 * Description:
 *   Generate one interface address response.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK
static FAR struct netlink_response_s *
netlink_get_ifaddr(FAR struct net_driver_s *dev, int domain, int type,
                   FAR const void *local_addr, uint8_t prefixlen,
                   FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getaddr_recvfrom_rsplist_s *alloc;
  FAR struct getaddr_recvfrom_response_s *resp;

  /* Allocate the response buffer */

  alloc = (FAR struct getaddr_recvfrom_rsplist_s *)
    kmm_zalloc(sizeof(struct getaddr_recvfrom_rsplist_s));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return NULL;
    }

  /* Initialize the response buffer */

  resp                    = &alloc->payload;

  resp->hdr.nlmsg_len     = sizeof(struct getaddr_recvfrom_response_s);
  resp->hdr.nlmsg_type    = type;
  resp->hdr.nlmsg_flags   = req ? req->hdr.nlmsg_flags : 0;
  resp->hdr.nlmsg_seq     = req ? req->hdr.nlmsg_seq : 0;
  resp->hdr.nlmsg_pid     = req ? req->hdr.nlmsg_pid : 0;

  resp->ifaddr.ifa_family = domain;
#ifdef CONFIG_NETDEV_IFINDEX
  resp->ifaddr.ifa_index  = dev->d_ifindex;
#endif
  resp->ifaddr.ifa_flags  = IFA_F_PERMANENT;
  resp->ifaddr.ifa_scope  = RT_SCOPE_UNIVERSE;

  resp->attr.rta_type     = IFA_LOCAL;
#ifdef CONFIG_NET_IPv4
  if (domain == AF_INET)
    {
      resp->attr.rta_len = RTA_LENGTH(sizeof(struct in_addr));
      memcpy(&resp->local_addr, local_addr, sizeof(struct in_addr));
      resp->ifaddr.ifa_prefixlen = prefixlen;
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (domain == AF_INET6)
    {
      resp->attr.rta_len = RTA_LENGTH(sizeof(struct in6_addr));
      memcpy(&resp->local_addr, local_addr, sizeof(struct in6_addr));
      resp->ifaddr.ifa_prefixlen = prefixlen;
    }
#endif

  /* Finally, return the response */

  return (FAR struct netlink_response_s *)alloc;
}
#endif

/****************************************************************************
 * Name: netlink_get_devlist
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK
static int netlink_device_callback(FAR struct net_driver_s *dev,
                                   FAR void *arg)
{
  FAR struct nlroute_info_s *info = arg;
  FAR struct netlink_response_s * resp;

  /* A RTM_GETLINK dump always reports RTM_NEWLINK */

  resp = netlink_get_device(dev, RTM_NEWLINK, info->req);
  if (resp == NULL)
    {
      return -ENOMEM;
    }

  netlink_add_response(info->handle, resp);
  return OK;
}

static int netlink_get_devlist(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  struct nlroute_info_s info;
  int ret;

  /* Visit each device */

  info.handle = handle;
  info.req    = req;

  ret = netdev_foreach(netlink_device_callback, &info);
  if (ret < 0)
    {
      return ret;
    }

  return netlink_add_terminator(handle, &req->hdr, 0);
}
#endif

/****************************************************************************
 * Name: netlink_fill_arptable()
 *
 * Description:
 *   Return the entire ARP table.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ARP) && !defined(CONFIG_NETLINK_DISABLE_GETNEIGH)
static size_t netlink_fill_arptable(
                              FAR struct getneigh_recvfrom_rsplist_s **entry)
{
  unsigned int ncopied;
  size_t allocsize;
  size_t tabsize;
  size_t rspsize;

  /* Lock the network so that the ARP table will be stable, then copy
   * the ARP table into the allocated memory.
   */

  ncopied = arp_snapshot((FAR struct arpreq *)(*entry)->payload.data,
                         CONFIG_NET_ARPTAB_SIZE);

  /* Now we have the real number of valid entries in the ARP table and
   * we can trim the allocation.
   */

  if (ncopied > 0)
    {
      FAR struct getneigh_recvfrom_rsplist_s *newentry;

      tabsize = ncopied * sizeof(struct arpreq);
      rspsize = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
      allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

      newentry = kmm_realloc(*entry, allocsize);

      if (newentry != NULL)
        {
          *entry = newentry;
        }

      (*entry)->payload.hdr.nlmsg_len = rspsize;
      (*entry)->payload.attr.rta_len  = RTA_LENGTH(tabsize);
    }

  return ncopied;
}
#endif

/****************************************************************************
 * Name: netlink_fill_nbtable()
 *
 * Description:
 *   Return the entire IPv6 neighbor table.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETNEIGH)
static size_t netlink_fill_nbtable(
                              FAR struct getneigh_recvfrom_rsplist_s **entry)
{
  unsigned int ncopied;
  size_t allocsize;
  size_t tabsize;
  size_t rspsize;

  /* Lock the network so that the Neighbor table will be stable, then
   * copy the Neighbor table into the allocated memory.
   */

  ncopied = neighbor_snapshot(
                      (FAR struct neighbor_entry_s *)(*entry)->payload.data,
                      CONFIG_NET_IPv6_NCONF_ENTRIES);

  /* Now we have the real number of valid entries in the Neighbor table
   * and we can trim the allocation.
   */

  if (ncopied > 0)
    {
      FAR struct getneigh_recvfrom_rsplist_s *newentry;

      tabsize   = ncopied * sizeof(struct neighbor_entry_s);
      rspsize   = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
      allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

      newentry = kmm_realloc(*entry, allocsize);

      if (newentry != NULL)
        {
          *entry = newentry;
        }

      (*entry)->payload.hdr.nlmsg_len = rspsize;
      (*entry)->payload.attr.rta_len  = RTA_LENGTH(tabsize);
    }

  return ncopied;
}
#endif

/****************************************************************************
 * Name: netlink_fill_nbtable()
 *
 * Description:
 *   Return the entire IPv6 neighbor table.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETLINK_DISABLE_GETNEIGH)
static FAR struct netlink_response_s *
netlink_get_neighbor(FAR const void *neigh, int domain, int type,
                     FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getneigh_recvfrom_rsplist_s *alloc;
  FAR struct getneigh_recvfrom_response_s *resp;
  size_t allocsize;
  size_t tabsize;
  size_t tabnum;
  size_t rspsize;

  /* Preallocate memory to hold the maximum sized ARP table
   * REVISIT:  This is probably excessively large and could cause false
   * memory out conditions.  A better approach would be to actually count
   * the number of valid entries in the ARP table.
   */

#if defined(CONFIG_NET_ARP)
  if (domain == AF_INET)
    {
      tabnum  = req ? CONFIG_NET_ARPTAB_SIZE : 1;
      tabsize = tabnum * sizeof(struct arpreq);
    }
  else
#endif
#if defined(CONFIG_NET_IPv6)
  if (domain == AF_INET6)
    {
      tabnum  = req ? CONFIG_NET_IPv6_NCONF_ENTRIES : 1;
      tabsize = tabnum * sizeof(struct neighbor_entry_s);
    }
  else
#endif
    {
      return NULL;
    }

  rspsize   = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
  allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

  /* Allocate the response buffer */

  alloc = kmm_zalloc(allocsize);
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return NULL;
    }

  /* Initialize the response buffer */

  resp                  = &alloc->payload;
  resp->hdr.nlmsg_len   = rspsize;
  resp->hdr.nlmsg_type  = type;
  resp->hdr.nlmsg_flags = req ? req->hdr.nlmsg_flags : 0;
  resp->hdr.nlmsg_seq   = req ? req->hdr.nlmsg_seq : 0;
  resp->hdr.nlmsg_pid   = req ? req->hdr.nlmsg_pid : 0;

  resp->msg.ndm_family = domain;
  resp->attr.rta_len   = RTA_LENGTH(tabsize);

  /* Copy neigh or arp entries into resp data */

  if (req == NULL)
    {
      if (neigh == NULL)
        {
          kmm_free(alloc);
          return NULL;
        }

      /* Only one entry need to notify */

      memcpy(resp->data, neigh, tabsize);
    }
#if defined(CONFIG_NET_ARP)
  else if (domain == AF_INET)
    {
      tabnum = netlink_fill_arptable(&alloc);
    }
#endif
#if defined(CONFIG_NET_IPv6)
  else if (domain == AF_INET6)
    {
      tabnum = netlink_fill_nbtable(&alloc);
    }
#endif

  /* If no entry in table, just free alloc */

  if (tabnum <= 0)
    {
      kmm_free(alloc);
      nwarn("WARNING: Failed to get entry in %s table.\n",
            domain == AF_INET ? "ARP" : "neighbor");
      return NULL;
    }

  return (FAR struct netlink_response_s *)alloc;
}

static int netlink_get_neighborlist(NETLINK_HANDLE handle,
                                    FAR const struct nlroute_sendto_request_s
                                    *req)
{
  FAR struct netlink_response_s *resp;
  int family = req->gen.rtgen_family;
  bool match = false;
  bool found = false;

  /* As in Linux, AF_UNSPEC dumps the tables of all families */

#ifdef CONFIG_NET_ARP
  if (family == AF_INET || family == AF_UNSPEC)
    {
      match = true;
      resp  = netlink_get_neighbor(NULL, AF_INET, RTM_GETNEIGH, req);
      if (resp != NULL)
        {
          netlink_add_response(handle, resp);
          found = true;
        }
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (family == AF_INET6 || family == AF_UNSPEC)
    {
      match = true;
      resp  = netlink_get_neighbor(NULL, AF_INET6, RTM_GETNEIGH, req);
      if (resp != NULL)
        {
          netlink_add_response(handle, resp);
          found = true;
        }
    }
#endif

  if (!match)
    {
      return -EAFNOSUPPORT;
    }

  /* The empty table of one family is an error.  The dump of all families
   * ends with NLMSG_DONE also when all tables are empty, as in Linux.
   */

  if (!found && family != AF_UNSPEC)
    {
      return -ENOENT;
    }

  return netlink_add_terminator(handle, &req->hdr, 0);
}
#endif /* CONFIG_NETLINK_DISABLE_GETNEIGH */

/****************************************************************************
 * Name: netlink_ipv4_route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static FAR struct netlink_response_s *
netlink_get_ipv4_route(FAR const struct net_route_ipv4_s *route, int type,
                       FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getroute_recvfrom_ipv4resplist_s *alloc;
  FAR struct getroute_recvfrom_ipv4response_s *resp;

  DEBUGASSERT(route != NULL);

  /* Allocate the response */

  alloc = (FAR struct getroute_recvfrom_ipv4resplist_s *)
    kmm_zalloc(sizeof(struct getroute_recvfrom_ipv4resplist_s));
  if (alloc == NULL)
    {
      return NULL;
    }

  /* Format the response */

  resp                  = &alloc->payload;
  resp->hdr.nlmsg_len   = sizeof(struct getroute_recvfrom_ipv4response_s);
  resp->hdr.nlmsg_type  = type;
  resp->hdr.nlmsg_flags = req ? req->hdr.nlmsg_flags : 0;
  resp->hdr.nlmsg_seq   = req ? req->hdr.nlmsg_seq : 0;
  resp->hdr.nlmsg_pid   = req ? req->hdr.nlmsg_pid : 0;

  resp->rte.rtm_family   = AF_INET;
  resp->rte.rtm_table    = RT_TABLE_MAIN;
  resp->rte.rtm_protocol = RTPROT_STATIC;
  resp->rte.rtm_scope    = RT_SCOPE_SITE;

  resp->dst.attr.rta_len  = RTA_LENGTH(sizeof(in_addr_t));
  resp->dst.attr.rta_type = RTA_DST;
  resp->dst.addr          = route->target;

  resp->genmask.attr.rta_len  = RTA_LENGTH(sizeof(in_addr_t));
  resp->genmask.attr.rta_type = RTA_GENMASK;
  resp->genmask.addr          = route->netmask;

  resp->gateway.attr.rta_len  = RTA_LENGTH(sizeof(in_addr_t));
  resp->gateway.attr.rta_type = RTA_GATEWAY;
  resp->gateway.addr          = route->router;

  return (FAR struct netlink_response_s *)alloc;
}

/****************************************************************************
 * Name: netlink_ipv4route_callback
 *
 * Input Parameters:
 *   route - The entry of IPV4 routing table.
 *   arg   - The netlink info of request.
 *
 ****************************************************************************/

static int netlink_ipv4route_callback(FAR struct net_route_ipv4_s *route,
                                      FAR void *arg)
{
  FAR struct nlroute_info_s *info = arg;
  FAR struct netlink_response_s *resp;

  resp = netlink_get_ipv4_route(route, RTM_NEWROUTE, info->req);
  if (resp == NULL)
    {
      return -ENOENT;
    }

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(info->handle, resp);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_list_ipv4_route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_list_ipv4_route(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  struct nlroute_info_s info;

  /* Visit each routing table entry */

  info.handle = handle;
  info.req    = req;

  return net_foreachroute_ipv4(netlink_ipv4route_callback, &info);
}
#endif

/****************************************************************************
 * Name: netlink_get_ipv6_route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static FAR struct netlink_response_s *
netlink_get_ipv6_route(FAR const struct net_route_ipv6_s *route, int type,
                       FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getroute_recvfrom_ipv6resplist_s *alloc;
  FAR struct getroute_recvfrom_ipv6response_s *resp;

  DEBUGASSERT(route != NULL);

  /* Allocate the response */

  alloc = (FAR struct getroute_recvfrom_ipv6resplist_s *)
    kmm_zalloc(sizeof(struct getroute_recvfrom_ipv6resplist_s));
  if (alloc == NULL)
    {
      return NULL;
    }

  /* Format the response */

  resp                  = &alloc->payload;
  resp->hdr.nlmsg_len   = sizeof(struct getroute_recvfrom_ipv6response_s);
  resp->hdr.nlmsg_type  = type;
  resp->hdr.nlmsg_flags = req ? req->hdr.nlmsg_flags : 0;
  resp->hdr.nlmsg_seq   = req ? req->hdr.nlmsg_seq : 0;
  resp->hdr.nlmsg_pid   = req ? req->hdr.nlmsg_pid : 0;

  resp->rte.rtm_family   = AF_INET6;
  resp->rte.rtm_table    = RT_TABLE_MAIN;
  resp->rte.rtm_protocol = RTPROT_STATIC;
  resp->rte.rtm_scope    = RT_SCOPE_SITE;

  resp->dst.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->dst.attr.rta_type = RTA_DST;
  net_ipv6addr_copy(resp->dst.addr, route->target);

  resp->genmask.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->genmask.attr.rta_type = RTA_GENMASK;
  net_ipv6addr_copy(resp->genmask.addr, route->netmask);

  resp->gateway.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->gateway.attr.rta_type = RTA_GATEWAY;
  net_ipv6addr_copy(resp->gateway.addr, route->router);

  return (FAR struct netlink_response_s *)alloc;
}

/****************************************************************************
 * Name: netlink_ipv6route_callback
 *
 * Description:
 *   Response netlink message from ipv6 route list.
 *
 * Input Parameters:
 *   route - The entry of IPV6 routing table.
 *   arg   - The netlink info of request.
 *
 ****************************************************************************/

static int netlink_ipv6route_callback(FAR struct net_route_ipv6_s *route,
                                      FAR void *arg)
{
  FAR struct nlroute_info_s *info = arg;
  FAR struct netlink_response_s *resp;

  resp = netlink_get_ipv6_route(route, RTM_NEWROUTE, info->req);
  if (resp == NULL)
    {
      return -ENOENT;
    }

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(info->handle, resp);

  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_ipv6route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_list_ipv6_route(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  struct nlroute_info_s info;

  /* Visit each routing table entry */

  info.handle = handle;
  info.req    = req;

  return net_foreachroute_ipv6(netlink_ipv6route_callback, &info);
}
#endif

/****************************************************************************
 * Name: netlink_list_route
 *
 * Description:
 *   Dump the routing table of the family of the request.  As in Linux,
 *   AF_UNSPEC dumps the tables of all families.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETROUTE
static int netlink_list_route(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  int family = req->gen.rtgen_family;
  int ret = -EAFNOSUPPORT;

#ifdef CONFIG_NET_IPv4
  if (family == AF_INET || family == AF_UNSPEC)
    {
      ret = netlink_list_ipv4_route(handle, req);
      if (ret < 0)
        {
          return ret;
        }
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (family == AF_INET6 || family == AF_UNSPEC)
    {
      ret = netlink_list_ipv6_route(handle, req);
      if (ret < 0)
        {
          return ret;
        }
    }
#endif

  if (ret < 0)
    {
      return ret;
    }

  /* Terminate the routing table */

  return netlink_add_terminator(handle, &req->hdr, 0);
}
#endif

/****************************************************************************
 * Name: netlink_new_ipv4addr
 *
 * Description:
 *   Set the ipv4 address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_NEWADDR)
static int netlink_new_ipv4addr(NETLINK_HANDLE handle,
                                FAR const struct nlmsghdr *nlmsg)
{
  FAR struct net_driver_s *dev;
  FAR struct ifaddrmsg *ifm = NLMSG_DATA(nlmsg);
  FAR struct nlattr *tb[IFA_MAX + 1];
  struct netlink_ext_ack extack;
  int ret;

  ret = nlmsg_parse(nlmsg, sizeof(*ifm), tb, IFA_MAX, g_ifa_ipv4_policy,
                    &extack);
  if (ret < 0)
    {
      return ret;
    }

  if (ifm->ifa_prefixlen > 32 || !tb[IFA_LOCAL])
    {
      return -EINVAL;
    }

  dev = netdev_findbyindex(ifm->ifa_index);

  if (dev == NULL)
    {
      return -ENODEV;
    }

  netdev_lock(dev);
  dev->d_ipaddr  = nla_get_in_addr(tb[IFA_LOCAL]);
  dev->d_netmask = make_mask(ifm->ifa_prefixlen);

  netlink_device_notify_ipaddr(dev, RTM_NEWADDR, AF_INET, &dev->d_ipaddr,
                               ifm->ifa_prefixlen);
  netdev_unlock(dev);

  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_new_ipv6addr
 *
 * Description:
 *   Set the ipv6 address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_NEWADDR)
static int netlink_new_ipv6addr(NETLINK_HANDLE handle,
                                FAR const struct nlmsghdr *nlmsg)
{
  FAR struct net_driver_s *dev;
  FAR struct ifaddrmsg *ifm = NLMSG_DATA(nlmsg);
  FAR struct nlattr *tb[IFA_MAX + 1];
  struct netlink_ext_ack extack;
  int ret;

  ret = nlmsg_parse(nlmsg, sizeof(*ifm), tb, IFA_MAX, g_ifa_ipv6_policy,
                    &extack);
  if (ret < 0)
    {
      return ret;
    }

  if (ifm->ifa_prefixlen > 128 || !tb[IFA_LOCAL])
    {
      return -EINVAL;
    }

  dev = netdev_findbyindex(ifm->ifa_index);

  if (dev == NULL)
    {
      return -ENODEV;
    }

  netdev_lock(dev);
  ret = netdev_ipv6_add(dev, nla_data(tb[IFA_LOCAL]), ifm->ifa_prefixlen);
  if (ret == OK)
    {
      netlink_device_notify_ipaddr(dev, RTM_NEWADDR, AF_INET6,
                                nla_data(tb[IFA_LOCAL]), ifm->ifa_prefixlen);
    }

  netdev_unlock(dev);

  return ret;
}
#endif

/****************************************************************************
 * Name: netlink_del_ipv4addr
 *
 * Description:
 *   Clear the ipv4 address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_DELADDR)
static int netlink_del_ipv4addr(NETLINK_HANDLE handle,
                                FAR const struct nlmsghdr *nlmsg)
{
  FAR struct net_driver_s *dev;
  FAR struct ifaddrmsg *ifm = NLMSG_DATA(nlmsg);
  FAR struct nlattr *tb[IFA_MAX + 1];
  struct netlink_ext_ack extack;
  int ret;

  ret = nlmsg_parse(nlmsg, sizeof(*ifm), tb, IFA_MAX, g_ifa_ipv4_policy,
                    &extack);
  if (ret < 0)
    {
      return ret;
    }

  dev = netdev_findbyindex(ifm->ifa_index);

  if (dev == NULL)
    {
      return -ENODEV;
    }

  netdev_lock(dev);
  if (tb[IFA_LOCAL] && dev->d_ipaddr != nla_get_in_addr(tb[IFA_LOCAL]))
    {
      netdev_unlock(dev);
      return -EADDRNOTAVAIL;
    }

  netlink_device_notify_ipaddr(dev, RTM_DELADDR, AF_INET, &dev->d_ipaddr,
                               net_ipv4_mask2pref(dev->d_netmask));
  dev->d_ipaddr  = 0;

  netdev_unlock(dev);

  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_del_ipv6addr
 *
 * Description:
 *   Clear the ipv6 address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_DELADDR)
static int netlink_del_ipv6addr(NETLINK_HANDLE handle,
                                FAR const struct nlmsghdr *nlmsg)
{
  FAR struct net_driver_s *dev;
  FAR struct ifaddrmsg *ifm = NLMSG_DATA(nlmsg);
  FAR struct nlattr *tb[IFA_MAX + 1];
  struct netlink_ext_ack extack;
  int ret;

  ret = nlmsg_parse(nlmsg, sizeof(*ifm), tb, IFA_MAX, g_ifa_ipv6_policy,
                    &extack);
  if (ret < 0)
    {
      return ret;
    }

  if (!tb[IFA_LOCAL] || ifm->ifa_prefixlen > 128)
    {
      return -EINVAL;
    }

  dev = netdev_findbyindex(ifm->ifa_index);

  if (dev == NULL)
    {
      return -ENODEV;
    }

  netdev_lock(dev);
  if (!NETDEV_IS_MY_V6ADDR(dev, nla_data(tb[IFA_LOCAL])))
    {
      netdev_unlock(dev);
      return -EADDRNOTAVAIL;
    }

  ret = netdev_ipv6_del(dev, nla_data(tb[IFA_LOCAL]), ifm->ifa_prefixlen);
  if (ret == OK)
    {
      netlink_device_notify_ipaddr(dev, RTM_DELADDR, AF_INET6,
                                nla_data(tb[IFA_LOCAL]), ifm->ifa_prefixlen);
    }

  netdev_unlock(dev);

  return ret;
}
#endif

/****************************************************************************
 * Name: netlink_get_addr
 *
 * Description:
 *   Clear the ipv4/ipv6 address.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETADDR
#ifdef CONFIG_NET_IPv6
static int netlink_ipv6_addr_callback(FAR struct net_driver_s *dev,
                                      FAR struct netdev_ifaddr6_s *addr,
                                      FAR void *arg)
{
  FAR struct nlroute_info_s *info = arg;
  FAR struct netlink_response_s *resp;

  resp = netlink_get_ifaddr(dev, AF_INET6, RTM_NEWADDR, addr->addr,
                            net_ipv6_mask2pref(addr->mask), info->req);
  if (resp == NULL)
    {
      return -ENOMEM;
    }

  netlink_add_response(info->handle, resp);
  return OK;
}
#endif

static int netlink_addr_callback(FAR struct net_driver_s *dev, FAR void *arg)
{
  FAR struct nlroute_info_s *info = arg;

#ifdef CONFIG_NET_IPv4
  if (info->req->gen.rtgen_family == AF_INET ||
      info->req->gen.rtgen_family == AF_UNSPEC)
    {
      FAR struct netlink_response_s *resp;

      resp = netlink_get_ifaddr(dev, AF_INET, RTM_NEWADDR, &dev->d_ipaddr,
                                net_ipv4_mask2pref(dev->d_netmask),
                                info->req);
      if (resp == NULL)
        {
          return -ENOMEM;
        }

      netlink_add_response(info->handle, resp);
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (info->req->gen.rtgen_family == AF_INET6 ||
      info->req->gen.rtgen_family == AF_UNSPEC)
    {
      return netdev_ipv6_foreach(dev, netlink_ipv6_addr_callback, arg);
    }
#endif

  return OK;
}

static int netlink_get_addr(NETLINK_HANDLE handle,
                            FAR const struct nlroute_sendto_request_s *req)
{
  struct nlroute_info_s info;
  int ret;

  /* As in Linux, AF_UNSPEC dumps the addresses of all families */

  switch (req->gen.rtgen_family)
    {
      case AF_UNSPEC:
#ifdef CONFIG_NET_IPv4
      case AF_INET:
#endif
#ifdef CONFIG_NET_IPv6
      case AF_INET6:
#endif
        break;

      default:
        return -EAFNOSUPPORT;
    }

  /* Visit each device */

  info.handle = handle;
  info.req    = req;

  ret = netdev_foreach(netlink_addr_callback, &info);
  if (ret < 0)
    {
      return ret;
    }

  return netlink_add_terminator(handle, &req->hdr, 0);
}
#endif

#if !defined(CONFIG_NETLINK_DISABLE_NEWADDR) && defined(CONFIG_NET_IPv6)
static FAR struct netlink_response_s *
netlink_fill_ipv6prefix(FAR struct net_driver_s *dev, int type,
                        FAR const struct icmpv6_prefixinfo_s *pinfo)
{
  FAR struct getprefix_recvfrom_rsplist_s *alloc;
  FAR struct getprefix_recvfrom_response_s *resp;

  DEBUGASSERT(dev != NULL && pinfo != NULL);

  alloc = kmm_zalloc(sizeof(struct getprefix_recvfrom_rsplist_s));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return NULL;
    }

  /* Initialize the response buffer */

  resp                  = &alloc->payload;

  resp->hdr.nlmsg_len   = sizeof(struct getprefix_recvfrom_response_s);
  resp->hdr.nlmsg_type  = type;
  resp->hdr.nlmsg_flags = 0;
  resp->hdr.nlmsg_seq   = 0;
  resp->hdr.nlmsg_pid   = 0;

  resp->pmsg.prefix_family = AF_INET6;
#ifdef CONFIG_NETDEV_IFINDEX
  resp->pmsg.prefix_ifindex = dev->d_ifindex;
#endif
  resp->pmsg.prefix_len  = pinfo->optlen;
  resp->pmsg.prefix_type = pinfo->opttype;

  resp->prefix.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->prefix.attr.rta_type = PREFIX_ADDRESS;
  net_ipv6addr_copy(resp->prefix.addr, pinfo->prefix);

  resp->pci.attr.rta_len  = RTA_LENGTH(sizeof(struct prefix_cacheinfo));
  resp->pci.attr.rta_type = PREFIX_CACHEINFO;

  resp->pci.pci.preferred_time  = NTOHS(pinfo->plifetime[0]) << 16;
  resp->pci.pci.preferred_time |= NTOHS(pinfo->plifetime[1]);
  resp->pci.pci.valid_time      = NTOHS(pinfo->vlifetime[0]) << 16;
  resp->pci.pci.valid_time     |= NTOHS(pinfo->vlifetime[1]);

  /* Finally, return the response */

  return (FAR struct netlink_response_s *)alloc;
}
#endif

#if !defined(CONFIG_NETLINK_DISABLE_NEWROUTE) && \
    defined(CONFIG_NET_CANFORWARD)

static int netlink_add_cangw_rule(FAR struct rtcanmsg *rt_msg,
                                  FAR struct nlattr *tb[])
{
  FAR struct cangw_rule_s *cgw_rule;
  FAR struct net_driver_s *src_dev;
  uint32_t src_if;
  uint32_t dst_if;
  int ret;

  /* Allocate and zero-initialize cangw_rule structure */

  cgw_rule = kmm_zalloc(sizeof(struct cangw_rule_s));
  if (cgw_rule == NULL)
    {
      nwarn("Failed to allocate cangw_rule structure\n");
      ret = -ENOMEM;
      goto errout;
    }

  cgw_rule->gwtype = rt_msg->gwtype;
  cgw_rule->flags  = rt_msg->flags;

  /* check for can_filter in attributes */

  if (tb[CGW_FILTER])
    {
      nla_memcpy(&cgw_rule->gw.ccgw.filter, tb[CGW_FILTER],
                  sizeof(struct can_filter));
    }

  ret = -ENODEV;

  /* specifying two interfaces is mandatory */

  if (!tb[CGW_SRC_IF] || !tb[CGW_DST_IF])
    {
      goto errout_with_rule;
    }

  src_if = *(FAR uint32_t *)nla_data(tb[CGW_SRC_IF]);
  dst_if = *(FAR uint32_t *)nla_data(tb[CGW_DST_IF]);

  src_dev = netdev_findbyindex(src_if);
  cgw_rule->dest_dev = netdev_findbyindex(dst_if);

  if (src_dev == NULL || cgw_rule->dest_dev == NULL)
    {
      goto errout_with_rule;
    }

  add_cangw_rule(src_dev, cgw_rule);
  return OK;

errout_with_rule:
  if (cgw_rule != NULL)
    {
      kmm_free(cgw_rule);
    }

errout:
  return ret;
}

static int netlink_new_can_route(FAR const struct nlmsghdr *nlmsg)
{
  FAR struct nlattr *tb[CGW_MAX + 1];
  FAR struct rtcanmsg *rt_msg = nlmsg_data(nlmsg);
  int ret;

  /* nlmsg boundary check */

  if (nlmsg->nlmsg_len < nlmsg_msg_size(sizeof(struct rtcanmsg)))
    {
      return -EINVAL;
    }

  /* parse nlmsg and acquire tb. */

  ret = nlmsg_parse(nlmsg, sizeof(struct rtcanmsg), tb, CGW_MAX,
                    g_cgw_policy, NULL);
  if (ret < 0)
    {
      nwarn("nlmsg_parse failed: %d\n", ret);
      return ret;
    }

  /* get r->gwtype and switch r->gwtype */

  switch (rt_msg->gwtype)
    {
      case CGW_TYPE_CAN_CAN:
        return netlink_add_cangw_rule(rt_msg, tb);

      default:
        return -EAFNOSUPPORT;
    }
}

#endif /* CONFIG_NET_CANFORWARD && !CONFIG_NETLINK_DISABLE_NEWROUTE */

#if !defined(CONFIG_NETLINK_DISABLE_DELROUTE) && \
    defined(CONFIG_NET_CANFORWARD)

static int netlink_remove_cangw_rule(FAR struct rtcanmsg *rt_msg,
                                     FAR struct nlattr *tb[])
{
  FAR struct net_driver_s *src_dev;
  uint32_t src_if;
  int count;

  /* CGW_SRC_IF is mandatory — we need to know which device to operate on */

  if (!tb[CGW_SRC_IF])
    {
      return -EINVAL;
    }

  src_if = *(FAR uint32_t *)nla_data(tb[CGW_SRC_IF]);
  src_dev = netdev_findbyindex(src_if);

  if (src_dev == NULL)
    {
      nerr("src_dev is not found\n");
      return -ENODEV;
    }

  /* If filter or destination attributes are present, build a match rule
   * and remove only matching entries. Otherwise pass NULL to remove all
   * rules on the source device.
   */

  if (tb[CGW_FILTER] || tb[CGW_DST_IF])
    {
      struct cangw_rule_s rule;

      memset(&rule, 0, sizeof(rule));

      /* Populate filter criteria if provided */

      if (tb[CGW_FILTER])
        {
          nla_memcpy(&rule.gw.ccgw.filter, tb[CGW_FILTER],
                     sizeof(struct can_filter));
        }

      /* Resolve destination device if specified; dst_if == 0 means
       * match any destination.
       */

      if (tb[CGW_DST_IF])
        {
          uint32_t dst_if = *(FAR uint32_t *)nla_data(tb[CGW_DST_IF]);

          if (dst_if != 0)
            {
              rule.dest_dev = netdev_findbyindex(dst_if);
              if (rule.dest_dev == NULL)
                {
                  return -ENODEV;
                }
            }
        }

      /* Pass NULL for removed — let remove_cangw_rule free heap nodes
       * directly instead of collecting them.
       */

      count = remove_cangw_rule(src_dev, &rule, NULL);
      if (count == 0)
        {
          nwarn("rule not found\n");
          return -ENOENT;
        }
    }
  else
    {
      /* No filter/dst specified — flush all rules on this device */

      count = remove_cangw_rule(src_dev, NULL, NULL);
    }

  return OK;
}

static int netlink_del_can_route(FAR const struct nlmsghdr *nlmsg)
{
  FAR struct nlattr *tb[CGW_MAX + 1];
  FAR struct rtcanmsg *rt_msg = nlmsg_data(nlmsg);
  int ret;

  /* nlmsg boundary check */

  if (nlmsg->nlmsg_len < nlmsg_msg_size(sizeof(struct rtcanmsg)))
    {
      return -EINVAL;
    }

  /* parse nlmsg and acquire tb. */

  ret = nlmsg_parse(nlmsg, sizeof(struct rtcanmsg), tb,
                    CGW_MAX, g_cgw_policy, NULL);
  if (ret < 0)
    {
      nwarn("nlmsg_parse failed: %d\n", ret);
      return ret;
    }

  /* get r->gwtype and switch r->gwtype */

  switch (rt_msg->gwtype)
    {
      case CGW_TYPE_CAN_CAN:
        return netlink_remove_cangw_rule(rt_msg, tb);

      default:
        return -EAFNOSUPPORT;
    }
}

#endif /* CONFIG_NET_CANFORWARD && !CONFIG_NETLINK_DISABLE_DELROUTE */

#if (!defined(CONFIG_NETLINK_DISABLE_NEWROUTE) || \
     !defined(CONFIG_NETLINK_DISABLE_DELROUTE)) && \
    defined(CONFIG_NET_CANFORWARD)

static void netlink_can_ack(NETLINK_HANDLE handle,
                            FAR const struct nlmsghdr *nlmsg,
                            int err)
{
  FAR struct new_ccgw_rule_response_s *resp;
  FAR struct new_ccgw_rule_resplist_s *alloc =
             kmm_zalloc(sizeof(struct new_ccgw_rule_resplist_s));

  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer "
           "about new ccgw rule.\n");
      return;
    }

  resp = &alloc->payload;
  resp->hdr.nlmsg_len   = sizeof(struct new_ccgw_rule_response_s);
  resp->hdr.nlmsg_type  = NLMSG_ERROR;
  resp->hdr.nlmsg_flags = nlmsg->nlmsg_flags;
  resp->hdr.nlmsg_seq   = nlmsg->nlmsg_seq;
  resp->hdr.nlmsg_pid   = nlmsg->nlmsg_pid;
  resp->err.error       = err;
  resp->err.msg         = *nlmsg;

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(handle,
                       (FAR struct netlink_response_s *)alloc);
}

#endif /* CONFIG_NET_CANFORWARD && !CONFIG_NETLINK_DISABLE_NEWROUTE/DELROUTE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_new_ipv4route
 *
 * Description:
 *   Process IPv4 route addition request
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_NEWROUTE)
static int netlink_new_ipv4route(FAR const struct nlmsghdr *nlmsg)
{
  FAR struct nlattr *tb[RTA_MAX + 1];
  FAR const struct rtmsg *rtm;
  in_addr_t gateway = 0;
  int oif_index = 0;
  FAR struct net_driver_s *dev = NULL;
  int ret;

  memset(tb, 0, sizeof(tb));

  if (nlmsg->nlmsg_len < NLMSG_LENGTH(sizeof(struct rtmsg)))
    {
      return -EINVAL;
    }

  ret = nlmsg_parse((FAR struct nlmsghdr *)nlmsg,
                    sizeof(struct rtmsg),
                    tb, RTA_MAX, NULL, NULL);
  if (ret < 0)
    {
      nwarn("nlmsg_parse failed: %d\n", ret);
      return ret;
    }

  rtm = NLMSG_DATA(nlmsg);

  if (tb[RTA_GATEWAY])
    {
      /* Validate gateway attribute length */

      if (nla_len(tb[RTA_GATEWAY]) >= sizeof(in_addr_t))
        {
          gateway = *(FAR in_addr_t *)nla_data(tb[RTA_GATEWAY]);
        }
      else
        {
          nwarn("netlink_new_ipv4route: Gateway address too short\n");
          return -EINVAL;
        }
    }

  if (tb[RTA_OIF])
    {
      /* Validate output interface index length */

      if (nla_len(tb[RTA_OIF]) >= sizeof(uint32_t))
        {
          oif_index = *(FAR uint32_t *)nla_data(tb[RTA_OIF]);
        }
      else
        {
          nwarn("netlink_new_ipv4route: OIF index too short\n");
          return -EINVAL;
        }
    }

  dev = netdev_findbyindex(oif_index);
  if (dev != NULL)
    {
#ifdef CONFIG_NET_ROUTE
      struct net_route_ipv4_s route;
      in_addr_t dst = 0;

      /* Parse destination if present */

      if (tb[RTA_DST])
        {
          if (nla_len(tb[RTA_DST]) >= sizeof(in_addr_t))
            {
              dst = *(FAR in_addr_t *)nla_data(tb[RTA_DST]);
            }
          else
            {
              nwarn("netlink_new_ipv4route: "
                    "Destination address too short\n");
              return -EINVAL;
            }
        }

      memset(&route, 0, sizeof(route));
      route.target  = dst;                         /* Destination network */
      route.netmask = make_mask(rtm->rtm_dst_len); /* Netmask from prefix */
      route.router  = gateway;                     /* Gateway */

      /* Broadcast route change notification */

      netlink_route_notify(&route, RTM_NEWROUTE, AF_INET);
#endif

      /* Update default router only when gateway is specified AND this
       * route is a default route (prefix length == 0).
       */

      if (gateway != 0 && rtm->rtm_dst_len == 0)
        {
          netdev_lock(dev);
          dev->d_draddr = gateway;
          netdev_unlock(dev);
        }

      return OK;
    }

  return -ENODEV;
}
#endif /* CONFIG_NET_IPv4 && !CONFIG_NETLINK_DISABLE_NEWROUTE */

/****************************************************************************
 * Name: netlink_new_ipv6route
 *
 * Description:
 *   Process IPv6 route addition request
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_NEWROUTE)
static int netlink_new_ipv6route(FAR const struct nlmsghdr *nlmsg)
{
  FAR struct nlattr *tb[RTA_MAX + 1];
  FAR const struct rtmsg *rtm;
  net_ipv6addr_t gateway;
  net_ipv6addr_t dst;
  int oif_index = 0;
  FAR struct net_driver_s *dev = NULL;
  int ret;

  memset(tb, 0, sizeof(tb));

  if (nlmsg->nlmsg_len < NLMSG_LENGTH(sizeof(struct rtmsg)))
    {
      return -EINVAL;
    }

  ret = nlmsg_parse((FAR struct nlmsghdr *)nlmsg,
                    sizeof(struct rtmsg),
                    tb, RTA_MAX, NULL, NULL);
  if (ret < 0)
    {
      nwarn("nlmsg_parse failed: %d\n", ret);
      return ret;
    }

  rtm = NLMSG_DATA(nlmsg);

  if (tb[RTA_GATEWAY])
    {
      if (nla_len(tb[RTA_GATEWAY]) >= sizeof(net_ipv6addr_t))
        {
          memcpy(&gateway, nla_data(tb[RTA_GATEWAY]),
                 sizeof(net_ipv6addr_t));
        }
      else
        {
          nwarn("netlink_new_ipv6route: Gateway address too short\n");
          return -EINVAL;
        }
    }

  /* Parse destination if present; default to :: (unspecified) */

  memset(&dst, 0, sizeof(dst));
  if (tb[RTA_DST])
    {
      if (nla_len(tb[RTA_DST]) >= sizeof(net_ipv6addr_t))
        {
          memcpy(&dst, nla_data(tb[RTA_DST]), sizeof(net_ipv6addr_t));
        }
      else
        {
          nwarn("netlink_new_ipv6route: Destination address too short\n");
          return -EINVAL;
        }
    }

  if (tb[RTA_OIF])
    {
      /* Validate output interface index length */

      if (nla_len(tb[RTA_OIF]) >= sizeof(uint32_t))
        {
          oif_index = *(FAR uint32_t *)nla_data(tb[RTA_OIF]);
        }
      else
        {
          nwarn("netlink_new_ipv6route: OIF index too short\n");
          return -EINVAL;
        }
    }

  if (oif_index > 0)
    {
      dev = netdev_findbyindex(oif_index);
    }

  if (dev)
    {
#ifdef CONFIG_NET_ROUTE
      struct net_route_ipv6_s route;

      memset(&route, 0, sizeof(struct net_route_ipv6_s));
      net_ipv6addr_copy(route.target, &dst);
      net_ipv6_pref2mask(route.netmask, rtm->rtm_dst_len);
      net_ipv6addr_copy(route.router, &gateway);
      netlink_route_notify(&route, RTM_NEWROUTE, AF_INET6);
#endif

      /* Update default router only when gateway is specified AND this
       * route is a default route (prefix length == 0).
       */

      if (!net_ipv6addr_cmp(&gateway, &g_ipv6_unspecaddr) &&
          rtm->rtm_dst_len == 0)
        {
          netdev_lock(dev);
          memcpy(&dev->d_ipv6draddr, &gateway, sizeof(net_ipv6addr_t));
          netdev_unlock(dev);
        }

      return OK;
    }

  return -ENODEV;
}
#endif /* CONFIG_NET_IPv6 && !CONFIG_NETLINK_DISABLE_NEWROUTE */

/****************************************************************************
 * Name: netlink_route_sendto()
 *
 * Description:
 *   Perform the sendto() operation for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

ssize_t netlink_route_sendto(NETLINK_HANDLE handle,
                             FAR const struct nlmsghdr *nlmsg,
                             size_t len, int flags,
                             FAR const struct sockaddr_nl *to,
                             socklen_t tolen)
{
  FAR const struct nlroute_sendto_request_s *req =
    (FAR const struct nlroute_sendto_request_s *)nlmsg;
  int ret = 0;

  DEBUGASSERT(handle != NULL && nlmsg != NULL &&
              nlmsg->nlmsg_len >= sizeof(struct nlmsghdr) &&
              len >= sizeof(struct nlmsghdr) &&
              len >= nlmsg->nlmsg_len && to != NULL &&
              tolen >= sizeof(struct sockaddr_nl));

  /* Handle according to the message type */

  switch (nlmsg->nlmsg_type)
    {
#ifndef CONFIG_NETLINK_DISABLE_GETLINK
      /* Dump a list of all devices */

      case RTM_GETLINK:

        /* Generate the response */

        ret = netlink_get_devlist(handle, req);
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_GETNEIGH
      /* Retrieve ARP/Neighbor Tables */

      case RTM_GETNEIGH:
        ret = netlink_get_neighborlist(handle, req);
        break;
#endif /* !CONFIG_NETLINK_DISABLE_GETNEIGH */

#ifndef CONFIG_NETLINK_DISABLE_GETROUTE
      /* Retrieve the IPv4 or IPv6 routing table */

      case RTM_GETROUTE:
        ret = netlink_list_route(handle, req);
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_NEWADDR
      /* Set the IPv4 or IPv6 address */

      case RTM_NEWADDR:
#ifdef CONFIG_NET_IPv4
        if (req->gen.rtgen_family == AF_INET)
          {
            ret = netlink_new_ipv4addr(handle, nlmsg);
          }
        else
#endif
#ifdef CONFIG_NET_IPv6
        if (req->gen.rtgen_family == AF_INET6)
          {
            ret = netlink_new_ipv6addr(handle, nlmsg);
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
          }
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_DELADDR
      /* Clear the IPv4 or IPv6 address */

      case RTM_DELADDR:
#ifdef CONFIG_NET_IPv4
        if (req->gen.rtgen_family == AF_INET)
          {
            ret = netlink_del_ipv4addr(handle, nlmsg);
          }
        else
#endif
#ifdef CONFIG_NET_IPv6
        if (req->gen.rtgen_family == AF_INET6)
          {
            ret = netlink_del_ipv6addr(handle, nlmsg);
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
          }
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_GETADDR
      /* Get the IPv4 or IPv6 address */

      case RTM_GETADDR:
        ret = netlink_get_addr(handle, req);
        break;
#endif
#ifndef CONFIG_NETLINK_DISABLE_NEWROUTE
      /* Add a route or a CAN gateway rule */

      case RTM_NEWROUTE:
#ifdef CONFIG_NET_IPv4
        if (req->gen.rtgen_family == AF_INET)
          {
            ret = netlink_new_ipv4route(nlmsg);
          }
        else
#endif
#ifdef CONFIG_NET_IPv6
        if (req->gen.rtgen_family == AF_INET6)
          {
            ret = netlink_new_ipv6route(nlmsg);
          }
        else
#endif
#ifdef CONFIG_NET_CANFORWARD
        if (req->gen.rtgen_family == AF_CAN)
          {
            ret = netlink_new_can_route(nlmsg);
            if (nlmsg->nlmsg_flags & NLM_F_ACK)
              {
                netlink_can_ack(handle, nlmsg, ret);
              }
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
            nwarn("Unsupported address family %d\n",
                  req->gen.rtgen_family);
          }
        break;
#endif /* !CONFIG_NETLINK_DISABLE_NEWROUTE */

#ifndef CONFIG_NETLINK_DISABLE_DELROUTE
      case RTM_DELROUTE:
#ifdef CONFIG_NET_CANFORWARD
        if (req->gen.rtgen_family == AF_CAN)
          {
            ret = netlink_del_can_route(nlmsg);
            if (nlmsg->nlmsg_flags & NLM_F_ACK)
              {
                netlink_can_ack(handle, nlmsg, ret);
              }
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
          }
        break;
#endif /* !CONFIG_NETLINK_DISABLE_DELROUTE */

      default:
        ret = -ENOSYS;
        break;
    }

  /* On success, return the size of the request that was processed */

  if (ret >= 0)
    {
      ret = len;
    }

  return ret;
}

/****************************************************************************
 * Name: netlink_device_notify()
 *
 * Description:
 *   Perform the route broadcast for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK
void netlink_device_notify(FAR struct net_driver_s *dev)
{
  FAR struct netlink_response_s *resp;

  DEBUGASSERT(dev != NULL);

  /* report the link event itself, RTM_NEWLINK when the device came up and
   * RTM_DELLINK when it went down.
   */

  resp = netlink_get_device(dev, IFF_IS_UP(dev->d_flags) ? RTM_NEWLINK :
                                                           RTM_DELLINK,
                            NULL);
  if (resp != NULL)
    {
      netlink_add_broadcast(RTNLGRP_LINK, resp);
      netlink_add_terminator(NULL, NULL, RTNLGRP_LINK);
    }
}
#endif

/****************************************************************************
 * Name: netlink_device_notify_ipaddr()
 *
 * Description:
 *   Perform the route broadcast for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETLINK_DISABLE_NEWADDR) || \
    !defined(CONFIG_NETLINK_DISABLE_DELADDR) || \
    !defined(CONFIG_NETLINK_DISABLE_GETADDR)
void netlink_device_notify_ipaddr(FAR struct net_driver_s *dev,
                                  int type, int domain,
                                  FAR const void *addr, uint8_t preflen)
{
  FAR struct netlink_response_s *resp;
  int group;

  DEBUGASSERT(dev != NULL);

  resp = netlink_get_ifaddr(dev, domain, type, addr, preflen, NULL);
  if (resp != NULL)
    {
#ifdef CONFIG_NET_IPv4
      if (domain == AF_INET)
        {
          group = RTNLGRP_IPV4_IFADDR;
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (domain == AF_INET6)
        {
          group = RTNLGRP_IPV6_IFADDR;
        }
      else
#endif
        {
          nwarn("netlink_device_notify_ipaddr unknown type %d domain %d\n",
                type, domain);
          return;
        }

      netlink_add_broadcast(group, resp);
      netlink_add_terminator(NULL, NULL, group);
    }
}
#endif

/****************************************************************************
 * Name: netlink_route_notify
 *
 * Description:
 *   Perform the route broadcast for the NETLINK_NETFILTER protocol.
 *
 * Input Parameters:
 *   route  - The route entry
 *   type   - The type of the message, RTM_*ROUTE
 *   domain - The domain of the message
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETROUTE
void netlink_route_notify(FAR const void *route, int type, int domain)
{
  FAR struct netlink_response_s *resp;
  int group;

  DEBUGASSERT(route != NULL);

#ifdef CONFIG_NET_IPv4
  if (domain == AF_INET)
    {
      resp = netlink_get_ipv4_route((FAR struct net_route_ipv4_s *)route,
                                    type, NULL);
      group = RTNLGRP_IPV4_ROUTE;
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (domain == AF_INET6)
    {
      resp = netlink_get_ipv6_route((FAR struct net_route_ipv6_s *)route,
                                    type, NULL);
      group = RTNLGRP_IPV6_ROUTE;
    }
  else
#endif
    {
      nwarn("netlink_route_notify unknown type %d domain %d\n",
            type, domain);
      return;
    }

  if (resp != NULL)
    {
      netlink_add_broadcast(group, resp);
      netlink_add_terminator(NULL, NULL, group);
    }
}
#endif

/****************************************************************************
 * Name: netlink_neigh_notify()
 *
 * Description:
 *   Perform the neigh broadcast for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETNEIGH
void netlink_neigh_notify(FAR const void *neigh, int type, int domain)
{
  FAR struct netlink_response_s *resp;

  resp = netlink_get_neighbor(neigh, domain, type, NULL);
  if (resp == NULL)
    {
      return;
    }

  netlink_add_broadcast(RTNLGRP_NEIGH, resp);
  netlink_add_terminator(NULL, NULL, RTNLGRP_NEIGH);
}
#endif

/****************************************************************************
 * Name: netlink_ipv6_prefix_notify()
 *
 * Description:
 *   Perform the RA prefix for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETLINK_DISABLE_NEWADDR) && defined(CONFIG_NET_IPv6)
void netlink_ipv6_prefix_notify(FAR struct net_driver_s *dev, int type,
                                FAR const struct icmpv6_prefixinfo_s *pinfo)
{
  FAR struct netlink_response_s *resp;

  resp = netlink_fill_ipv6prefix(dev, type, pinfo);
  if (resp == NULL)
    {
      return;
    }

  netlink_add_broadcast(RTNLGRP_IPV6_PREFIX, resp);
  netlink_add_terminator(NULL, NULL, RTNLGRP_IPV6_PREFIX);
}
#endif

#endif /* CONFIG_NETLINK_ROUTE */
