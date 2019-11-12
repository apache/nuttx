/****************************************************************************
 * net/netlink/netlink_route.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <net/route.h>
#include <netpacket/netlink.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/neighbor.h>

#include "netdev/netdev.h"
#include "arp/arp.h"
#include "neighbor/neighbor.h"
#include "route/route.h"
#include "netlink/netlink.h"

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

#undef NETLINK_DISABLE_NLMSGDONE
#if defined(CONFIG_NETLINK_DISABLE_GETLINK) && \
    defined(CONFIG_NETLINK_DISABLE_GETROUTE)
#  define NETLINK_DISABLE_NLMSGDONE 1
#endif

/* Helpers ******************************************************************/

#define IFA_RTA(r)  \
  ((FAR struct rtattr *)(((FAR char *)(r)) + \
   NLMSG_ALIGN(sizeof(struct ifaddrmsg))))
#define IFA_PAYLOAD(n) \
  NLMSG_PAYLOAD(n, sizeof(struct ifaddrmsg))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Used to send message done.  gen is tacked on to provide the address
 * family.  It is discarded before returning the struct nlmsghdr payload.
 */

struct nlroute_msgdone_response_s
{
  struct nlmsghdr  hdr;
  struct rtgenmsg  gen;
};

struct nlroute_msgdone_rsplist_s
{
  sq_entry_t flink;
  struct nlroute_msgdone_response_s payload;
};

/* RTM_GETLINK:  Enumerate network devices .  The message contains the
 * 'rtgenmsg' structure.
 */

struct getlink_sendto_request_s
{
  struct nlmsghdr  hdr;
  struct rtgenmsg  gen;
};

struct getlink_recvfrom_response_s
{
  struct nlmsghdr  hdr;
  struct ifinfomsg iface;
  struct rtattr    attr;
  uint8_t          data[IFNAMSIZ];  /* IFLA_IFNAME is the only attribute supported */
};

struct getlink_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct getlink_recvfrom_response_s payload;
};

/* RTM_GETNEIGH:  Get neighbor table entry.  The message contains an 'ndmsg'
 * structure.
 */

struct getneigh_sendto_request_s
{
  struct nlmsghdr hdr;
  struct ndmsg    msg;
};

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

struct getroute_sendto_request_s
{
  struct nlmsghdr  hdr;
  struct rtgenmsg  gen;
};

struct getroute_recvfrom_response_s
{
  struct nlmsghdr  hdr;
  struct rtmsg     rte;

  /* Addresses follow */
};

struct getroute_recvfrom_resplist_s
{
  sq_entry_t flink;
  struct getroute_recvfrom_response_s payload;
};

struct getroute_recvfrom_ipv4addr_s
{
  struct rtattr    attr;
  in_addr_t        addr;
};

struct getroute_recvfrom_ipv4response_s
{
  struct nlmsghdr  hdr;
  struct rtmsg     rte;
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
  struct rtattr    attr;
  net_ipv6addr_t   addr;
};

struct getroute_recvfrom_ipv6response_s
{
  struct nlmsghdr  hdr;
  struct rtmsg     rte;
  struct getroute_recvfrom_ipv6addr_s dst;
  struct getroute_recvfrom_ipv6addr_s genmask;
  struct getroute_recvfrom_ipv6addr_s gateway;
};

struct getroute_recvfrom_ipv6resplist_s
{
  sq_entry_t flink;
  struct getroute_recvfrom_ipv6response_s payload;
};

/* netdev_foreach() callabck */

struct nlroute_devinfo_s
{
  FAR struct socket *psock;
  FAR const struct getlink_sendto_request_s *req;
};

struct nlroute_routeinfo_s
{
  FAR struct socket *psock;
  FAR const struct getroute_sendto_request_s *req;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_device_callback
 *
 * Description:
 *   Handle one device.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK
static int netlink_device_callback(FAR struct net_driver_s *dev,
                                   FAR void *arg)
{
  FAR struct nlroute_devinfo_s *devinfo;
  FAR struct getlink_recvfrom_rsplist_s *alloc;
  FAR struct getlink_recvfrom_response_s *resp;

  DEBUGASSERT(dev != NULL && arg != NULL);

  devinfo = (FAR struct nlroute_devinfo_s *)arg;
  DEBUGASSERT(devinfo->psock != NULL && devinfo->req != NULL);

  /* Check if the link is in the UP state */

  if ((dev->d_flags & IFF_UP) == 0)
    {
      /* No.. skip this device */

      return 0;
    }

  /* Filter only the requested address families */

  switch (devinfo->req->gen.rtgen_family)
    {
#ifdef CONFIG_NET_LOCAL
      case AF_LOCAL:
        /* Should have devinfo->psock->s_domain == PF_LOCAL and d_lltype ==
         * NET_LL_LOOPBACK.
         */

        if (devinfo->psock->s_domain == PF_LOCAL)
          {
            DEBUGASSERT(dev->d_lltype == NET_LL_LOOPBACK);
            break;
          }
        else
          {
            return 0;
          }
#endif

#if CONFIG_NET_IPv4
        /* Should have devinfo->psock->s_domain == PF_INET but d_lltype could be
         * several things.
         */

      case AF_INET:

        if (devinfo->psock->s_domain == PF_INET)
          {
            break;
          }
        else
          {
            return 0;
          }
#endif

#ifdef CONFIG_NET_IPv6
        /* Should have devinfo->psock->s_domain == PF_INET6 but d_lltype could be
         * several things.
         */

      case AF_INET6:

        if (devinfo->psock->s_domain == PF_INET6)
          {
            break;
          }
        else
          {
            return 0;
          }
#endif

#ifdef CONFIG_NET_BLUETOOTH
        /* Should have devinfo->psock->s_domain == PF_PACKET and d_lltype should be
         * NET_LL_BLUETOOTH.
         */

      case AF_BLUETOOTH:
        if (devinfo->psock->s_domain == PF_PACKET)
          {
            DEBUGASSERT(dev->d_lltype == NET_LL_BLUETOOTH);
            break;
          }
        else
          {
            return 0;
          }
#endif

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_IEEE802154)
      /* psock_domain could be PF_PACKET or PF_INET6 but d_lltype should
       * be AF_IEEE802154.
       */

      case AF_IEEE802154:
        if (dev->d_lltype == NET_LL_IEEE802154)
          {
            DEBUGASSERT(devinfo->psock->s_domain == PF_PACKET ||
                        devinfo->psock->s_domain == PF_INET6);
            break;
          }
        else
          {
            return 0;
          }
#endif

#ifdef CONFIG_NET_6LOWPAN
      /* psock_domain should be PF_INET6 and d_lltype should be
       * NET_LL_PKTRADIO.
       */

        if (dev->d_lltype == NET_LL_PKTRADIO)
          {
            DEBUGASSERT(devinfo->psock->s_domain == PF_INET6);
            break;
          }
        else
          {
            return 0;
          }
#endif

      case AF_PACKET:     /* Take all address families */
        break;

      case AF_UNSPEC:
      case AF_PKTRADIO:
      default:
        nerr("ERROR: Unsupported address family: %u\n", devinfo->req->gen);
        return 0;
    }

  /* Allocate the response buffer */

  alloc = (FAR struct getlink_recvfrom_rsplist_s *)
    kmm_malloc(sizeof(struct getlink_recvfrom_rsplist_s));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return -ENOMEM;
    }

  /* Initialize the response buffer */

  resp                   = &alloc->payload;

  resp->hdr.nlmsg_len    = sizeof(struct getlink_recvfrom_response_s);
  resp->hdr.nlmsg_type   = RTM_NEWLINK;
  resp->hdr.nlmsg_flags  = devinfo->req->hdr.nlmsg_flags;
  resp->hdr.nlmsg_seq    = devinfo->req->hdr.nlmsg_seq;
  resp->hdr.nlmsg_pid    = devinfo->req->hdr.nlmsg_pid;

  resp->iface.ifi_family = devinfo->req->gen.rtgen_family;
  resp->iface.ifi_pid    = devinfo->req->hdr.nlmsg_pid;
  resp->iface.ifi_type   = devinfo->req->hdr.nlmsg_type;
#ifdef CONFIG_NETDEV_IFINDEX
  resp->iface.ifi_index  = dev->dd_ifindex;
#else
  resp->iface.ifi_index  = 0;
#endif
  resp->iface.ifi_flags  = devinfo->req->hdr.nlmsg_flags;
  resp->iface.ifi_change = 0xffffffff;

  resp->attr.rta_len     = RTA_LENGTH(strnlen(dev->d_ifname, IFNAMSIZ));
  resp->attr.rta_type    = IFLA_IFNAME;

  strncpy((FAR char *)resp->data, dev->d_ifname, IFNAMSIZ);

  /* REVISIT:  Another response should be provided with nlmsg_type =
   * RTM_NEWROUTE.  That response should include struct rtmsg followed by a
   * number of attributes.  This response provides routing information for
   * the device including address (RTA_DST) and gateway (RTA_GATEWAY)
   * attributes.
   */

  /* Finally, add the data to the list of pending responses */

  netlink_add_response(devinfo->psock, (FAR struct netlink_response_s *)alloc);
  return 0;
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
static int netlink_get_devlist(FAR struct socket *psock,
                               FAR const struct getlink_sendto_request_s *req)
{
  struct nlroute_devinfo_s devinfo;
  FAR struct nlroute_msgdone_rsplist_s *alloc;
  FAR struct nlroute_msgdone_response_s *resp;
  int ret;

  /* Pre-allocate the list terminator */

  alloc = (FAR struct nlroute_msgdone_rsplist_s *)
    kmm_malloc(sizeof(struct nlroute_msgdone_rsplist_s));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response terminator.\n");
      return -ENOMEM;
    }

  /* Visit each device */

  devinfo.psock = psock;
  devinfo.req   = req;

  net_lock();
  ret = netdev_foreach(netlink_device_callback, &devinfo);
  if (ret < 0)
    {
      net_unlock();
      return ret;
    }

  /* Initialize and send the list terminator */

  resp                   = &alloc->payload;
  resp->hdr.nlmsg_len    = sizeof(struct nlroute_msgdone_response_s);
  resp->hdr.nlmsg_type   = NLMSG_DONE;
  resp->hdr.nlmsg_flags  = req->hdr.nlmsg_flags;
  resp->hdr.nlmsg_seq    = req->hdr.nlmsg_seq;
  resp->hdr.nlmsg_pid    = req->hdr.nlmsg_pid;
  resp->gen.rtgen_family = req->gen.rtgen_family;

  /* Finally, add the data to the list of pending responses */

  netlink_add_response(psock, (FAR struct netlink_response_s *)alloc);
  net_unlock();
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_arptable()
 *
 * Description:
 *   Return the entire ARP table.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ARP) && !defined(CONFIG_NETLINK_DISABLE_GETNEIGH)
static int netlink_get_arptable(FAR struct socket *psock,
                                FAR const struct getneigh_sendto_request_s *req)
{
  FAR struct getneigh_recvfrom_rsplist_s *entry;
  unsigned int ncopied;
  size_t tabsize;
  size_t rspsize;
  size_t allocsize;

  /* Preallocate memory to hold the maximum sized ARP table
   * REVISIT:  This is probably excessively large and could cause false
   * memory out conditions.  A better approach would be to actually count
   * the number of valid entries in the ARP table.
   */

  tabsize   = CONFIG_NET_ARPTAB_SIZE * sizeof(struct arp_entry_s);
  rspsize   = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
  allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

  entry     = (FAR struct getneigh_recvfrom_rsplist_s *)kmm_malloc(allocsize);
  if (entry == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return -ENOMEM;
    }

  /* Populate the entry */

  memcpy(&entry->payload.hdr, &req->hdr, sizeof(struct nlmsghdr));
  entry->payload.hdr.nlmsg_len = rspsize;
  memcpy(&entry->payload.msg, &req->msg, sizeof(struct ndmsg));
  entry->payload.attr.rta_len  = RTA_LENGTH(tabsize);
  entry->payload.attr.rta_type = 0;

  /* Lock the network so that the ARP table will be stable, then copy
   * the ARP table into the allocated memory.
   */

  net_lock();
  ncopied = arp_snapshot((FAR struct arp_entry_s *)entry->payload.data,
                         CONFIG_NET_ARPTAB_SIZE);
  net_unlock();

  /* Now we have the real number of valid entries in the ARP table and
   * we can trim the allocation.
   */

  if (ncopied < CONFIG_NET_ARPTAB_SIZE)
    {
      FAR struct getneigh_recvfrom_rsplist_s *newentry;

      tabsize = ncopied * sizeof(struct arp_entry_s);
      rspsize = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
      allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

      newentry = (FAR struct getneigh_recvfrom_rsplist_s *)
        kmm_realloc(entry, allocsize);

      if (newentry != NULL)
        {
           entry = newentry;
        }

      entry->payload.hdr.nlmsg_len = rspsize;
      entry->payload.attr.rta_len  = RTA_LENGTH(tabsize);
    }

  /* Finally, add the data to the list of pending responses */

  netlink_add_response(psock, (FAR struct netlink_response_s *)entry);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_nbtable()
 *
 * Description:
 *   Return the entire IPv6 neighbor table.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETNEIGH)
static int netlink_get_nbtable(FAR struct socket *psock,
                               FAR const struct getneigh_sendto_request_s *req)
{
  FAR struct getneigh_recvfrom_rsplist_s *entry;
  unsigned int ncopied;
  size_t tabsize;
  size_t rspsize;
  size_t allocsize;

  /* Preallocate memory to hold the maximum sized Neighbor table
   * REVISIT:  This is probably excessively large and could cause false
   * memory out conditions.  A better approach would be to actually count
   * the number of valid entries in the Neighbor table.
   */

  tabsize   = CONFIG_NET_IPv6_NCONF_ENTRIES *
              sizeof(struct neighbor_entry_s);
  rspsize   = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
  allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

  entry     = (FAR struct getneigh_recvfrom_rsplist_s *)kmm_malloc(allocsize);
  if (entry == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return -ENOMEM;
    }

  /* Populate the entry */

  memcpy(&entry->payload.hdr, &req->hdr, sizeof(struct nlmsghdr));
  entry->payload.hdr.nlmsg_len = rspsize;
  memcpy(&entry->payload.msg, &req->msg, sizeof(struct ndmsg));
  entry->payload.attr.rta_len  = RTA_LENGTH(tabsize);
  entry->payload.attr.rta_type = 0;

  /* Lock the network so that the Neighbor table will be stable, then
   * copy the Neighbor table into the allocated memory.
   */

  net_lock();
  ncopied = neighbor_snapshot((FAR struct neighbor_entry_s *)entry->payload.data,
                              CONFIG_NET_IPv6_NCONF_ENTRIES);
  net_unlock();

  /* Now we have the real number of valid entries in the Neighbor table
   * and we can trim the allocation.
   */

  if (ncopied < CONFIG_NET_IPv6_NCONF_ENTRIES)
    {
      FAR struct getneigh_recvfrom_rsplist_s *newentry;

      tabsize   = ncopied * sizeof(struct neighbor_entry_s);
      rspsize   = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
      allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

      newentry = (FAR struct getneigh_recvfrom_rsplist_s *)
        kmm_realloc(entry, allocsize);

      if (newentry != NULL)
        {
           entry = newentry;
        }

      entry->payload.hdr.nlmsg_len = rspsize;
      entry->payload.attr.rta_len  = RTA_LENGTH(tabsize);
    }

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(psock, (FAR struct netlink_response_s *)entry);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_route_terminator
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETROUTE
static int netlink_route_terminator(FAR struct socket *psock,
                                    FAR const struct getroute_sendto_request_s *req)
{
  FAR struct nlroute_msgdone_rsplist_s *alloc;
  FAR struct nlroute_msgdone_response_s *resp;

  /* Allocate the list terminator */

  alloc = (FAR struct nlroute_msgdone_rsplist_s *)
    kmm_malloc(sizeof(struct nlroute_msgdone_rsplist_s));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response terminator.\n");
      return -ENOMEM;
    }

  /* Initialize and send the list terminator */

  resp                   = &alloc->payload;
  resp->hdr.nlmsg_len    = sizeof(struct nlroute_msgdone_response_s);
  resp->hdr.nlmsg_type   = NLMSG_DONE;
  resp->hdr.nlmsg_flags  = req->hdr.nlmsg_flags;
  resp->hdr.nlmsg_seq    = req->hdr.nlmsg_seq;
  resp->hdr.nlmsg_pid    = req->hdr.nlmsg_pid;
  resp->gen.rtgen_family = req->gen.rtgen_family;

  /* Finally, add the response to the list of pending responses */

  net_lock();
  netlink_add_response(psock, (FAR struct netlink_response_s *)alloc);
  net_unlock();

  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_ipv4_route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_ipv4_route(FAR struct net_route_ipv4_s *route,
                              FAR void *arg)
{
  FAR struct nlroute_routeinfo_s *routeinfo;
  FAR struct getroute_recvfrom_ipv4resplist_s *alloc;
  FAR struct getroute_recvfrom_ipv4response_s *resp;

  DEBUGASSERT(route != NULL && arg != NULL);
  routeinfo = (FAR struct nlroute_routeinfo_s *)arg;

  /* Allocate the response */

  alloc = (FAR struct getroute_recvfrom_ipv4resplist_s *)
    kmm_malloc(sizeof(struct getroute_recvfrom_ipv4resplist_s));
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  /* Format the response */

  resp                        = &alloc->payload;
  resp->hdr.nlmsg_len         = sizeof(struct getroute_recvfrom_ipv4response_s);
  resp->hdr.nlmsg_type        = RTM_NEWROUTE;
  resp->hdr.nlmsg_flags       = routeinfo->req->hdr.nlmsg_flags;
  resp->hdr.nlmsg_seq         = routeinfo->req->hdr.nlmsg_seq;
  resp->hdr.nlmsg_pid         = routeinfo->req->hdr.nlmsg_pid;

  memset(&resp->rte, 0, sizeof(struct rtmsg));  /* REVISIT:  Uninitialize fields */
  resp->rte.rtm_family        = routeinfo->req->gen.rtgen_family;
  resp->rte.rtm_table         = RT_TABLE_MAIN;
  resp->rte.rtm_protocol      = RTPROT_STATIC;
  resp->rte.rtm_scope         = RT_SCOPE_SITE;
  
  resp->dst.attr.rta_len      = RTA_LENGTH(sizeof(in_addr_t));
  resp->dst.attr.rta_type     = RTA_DST;
  resp->dst.addr              = route->target;

  resp->genmask.attr.rta_len  = RTA_LENGTH(sizeof(in_addr_t));
  resp->genmask.attr.rta_type = RTA_GENMASK;
  resp->genmask.addr          = route->netmask;

  resp->gateway.attr.rta_len  = RTA_LENGTH(sizeof(in_addr_t));
  resp->gateway.attr.rta_type = RTA_GATEWAY;
  resp->gateway.addr          = route->router;

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(routeinfo->psock, (FAR struct netlink_response_s *)alloc);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_ipv4route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_get_ipv4route(FAR struct socket *psock,
                                 FAR const struct getroute_sendto_request_s *req)
{
  struct nlroute_routeinfo_s routeinfo;
  int ret;

  /* Visit each routing table entry */

  routeinfo.psock = psock;
  routeinfo.req   = req;

  net_lock();
  ret = net_foreachroute_ipv4(netlink_ipv4_route, &routeinfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Terminate the routing table */

  ret = netlink_route_terminator(psock, req);
  net_unlock();
  return ret;
}
#endif

/****************************************************************************
 * Name: netlink_ipv6_route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_ipv6_route(FAR struct net_route_ipv6_s *route,
                              FAR void *arg)
{
  FAR struct nlroute_routeinfo_s *routeinfo;
  FAR struct getroute_recvfrom_ipv6resplist_s *alloc;
  FAR struct getroute_recvfrom_ipv6response_s *resp;

  DEBUGASSERT(route != NULL && arg != NULL);
  routeinfo = (FAR struct nlroute_routeinfo_s *)arg;

  /* Allocate the response */

  alloc = (FAR struct getroute_recvfrom_ipv6resplist_s *)
    kmm_malloc(sizeof(struct getroute_recvfrom_ipv6resplist_s));
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  /* Format the response */

  resp                        = &alloc->payload;
  resp->hdr.nlmsg_len         = sizeof(struct getroute_recvfrom_ipv6response_s);
  resp->hdr.nlmsg_type        = RTM_NEWROUTE;
  resp->hdr.nlmsg_flags       = routeinfo->req->hdr.nlmsg_flags;
  resp->hdr.nlmsg_seq         = routeinfo->req->hdr.nlmsg_seq;
  resp->hdr.nlmsg_pid         = routeinfo->req->hdr.nlmsg_pid;

  memset(&resp->rte, 0, sizeof(struct rtmsg));  /* REVISIT:  Uninitialize fields */
  resp->rte.rtm_family        = routeinfo->req->gen.rtgen_family;
  resp->rte.rtm_table         = RT_TABLE_MAIN;
  resp->rte.rtm_protocol      = RTPROT_STATIC;
  resp->rte.rtm_scope         = RT_SCOPE_SITE;
  
  resp->dst.attr.rta_len      = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->dst.attr.rta_type     = RTA_DST;
  net_ipv6addr_copy(resp->dst.addr, route->target);

  resp->genmask.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->genmask.attr.rta_type = RTA_GENMASK;
  net_ipv6addr_copy(resp->genmask.addr, route->netmask);

  resp->gateway.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->gateway.attr.rta_type = RTA_GATEWAY;
  net_ipv6addr_copy(resp->gateway.addr, route->router);

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(routeinfo->psock, (FAR struct netlink_response_s *)alloc);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_ip6vroute
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_get_ip6vroute(FAR struct socket *psock,
                                 FAR const struct getroute_sendto_request_s *req)
{
  struct nlroute_routeinfo_s routeinfo;
  int ret;

  /* Visit each routing table entry */

  routeinfo.psock = psock;
  routeinfo.req   = req;

  net_lock();
  ret = net_foreachroute_ipv6(netlink_ipv6_route, &routeinfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Terminate the routing table */

  ret = netlink_route_terminator(psock, req);
  net_unlock();
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_route_sendto()
 *
 * Description:
 *   Perform the sendto() operation for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

ssize_t netlink_route_sendto(FAR struct socket *psock,
                             FAR const struct nlmsghdr *nlmsg,
                             size_t len, int flags,
                             FAR const struct sockaddr_nl *to,
                             socklen_t tolen)
{
  FAR const struct getneigh_sendto_request_s *gnreq =
    (FAR const struct getneigh_sendto_request_s *)nlmsg;
  int ret;

  DEBUGASSERT(psock != NULL && nlmsg != NULL &&
              nlmsg->nlmsg_len >= sizeof(struct nlmsghdr) &&
              len >= sizeof(struct nlmsghdr) &&
              len >= nlmsg->nlmsg_len && to != NULL &&
              tolen >= sizeof(struct sockaddr_nl));

  /* Handle according to the message type */

  switch (gnreq->hdr.nlmsg_type)
    {
#ifndef CONFIG_NETLINK_DISABLE_GETLINK
      /* Dump a list of all devices */

      case RTM_GETLINK:
        {
          FAR const struct getlink_sendto_request_s *glreq =
            (FAR const struct getlink_sendto_request_s *)nlmsg;

          /* Generate the response */

          ret = netlink_get_devlist(psock, glreq);
        }
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_GETNEIGH
      /* Retrieve ARP/Neighbor Tables */

      case RTM_GETNEIGH:
        {
#ifdef CONFIG_NET_ARP
          /* Retrieve the ARP table in its entirety. */

          if (gnreq->msg.ndm_family == AF_INET)
            {
              ret = netlink_get_arptable(psock, gnreq);
            }
          else
#endif

#ifdef CONFIG_NET_IPv6
          /* Retrieve the IPv6 neighbor table in its entirety. */

          if (gnreq->msg.ndm_family == AF_INET6)
            {
               ret = netlink_get_nbtable(psock, gnreq);
            }
          else
#endif
            {
              ret = -EAFNOSUPPORT;
            }
        }
        break;
#endif /* !CONFIG_NETLINK_DISABLE_GETNEIGH */

#ifndef CONFIG_NETLINK_DISABLE_GETROUTE
      /* Retrieve the IPv4 or IPv6 routing table */

      case RTM_GETROUTE:
        {
          FAR const struct getroute_sendto_request_s *grreq =
            (FAR const struct getroute_sendto_request_s *)nlmsg;

#ifdef CONFIG_NET_IPv4
          if (grreq->gen.rtgen_family == AF_INET)
            {
              ret = netlink_get_ipv4route(psock, grreq);
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
          if (grreq->gen.rtgen_family == AF_INET6)
            {
              ret = netlink_get_ip6vroute(psock, grreq);
            }
          else
#endif
            {
              ret = -EAFNOSUPPORT;
            }
        }
        break;
#endif

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
 * Name: netlink_route_recvfrom()
 *
 * Description:
 *   Perform the recvfrom() operation for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

ssize_t netlink_route_recvfrom(FAR struct socket *psock,
                               FAR struct nlmsghdr *nlmsg,
                               size_t len, int flags,
                               FAR struct sockaddr_nl *from)
{
  FAR struct netlink_response_s *entry;
  ssize_t ret;

  DEBUGASSERT(psock != NULL && nlmsg != NULL &&
              len >= sizeof(struct nlmsghdr));

  /* Find the response to this message */

  net_lock();
  entry = (FAR struct netlink_response_s *)netlink_get_response(psock);
  net_unlock();

  if (entry == NULL)
    {
      /* REVISIT:  The correct behavior here for a blocking socket would be
       * to wait until the data becomes available.  This is not an issue for
       * the currently supported operations since they are fully synchronous
       * but will become an issue in the future.
       */

      return -ENOENT;
    }

  if (len < entry->msg.nlmsg_len)
    {
      kmm_free(entry);
      return -EMSGSIZE;
    }

  /* Handle the response according to the message type */

  switch (entry->msg.nlmsg_type)
    {
#ifndef CONFIG_NETLINK_DISABLE_GETLINK
      case RTM_NEWLINK:
        {
          FAR struct getlink_recvfrom_rsplist_s *resp =
            (FAR struct getlink_recvfrom_rsplist_s *)entry;

          /* Copy the payload to the user buffer */

          memcpy(nlmsg, &resp->payload, resp->payload.hdr.nlmsg_len);

          /* Return address.  REVISIT... this is just a guess. */

          if (from != NULL)
            {
              from->nl_family = resp->payload.iface.ifi_family;
              from->nl_pad    = 0;
              from->nl_pid    = resp->payload.hdr.nlmsg_pid;
              from->nl_groups = resp->payload.hdr.nlmsg_type;
            }

          /* The return value is the payload size */

          ret = resp->payload.hdr.nlmsg_len;
        }
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_GETNEIGH
      case RTM_GETNEIGH:
        {
          FAR struct getneigh_recvfrom_rsplist_s *resp =
            (FAR struct getneigh_recvfrom_rsplist_s *)entry;

          /* Copy the payload to the user buffer */

          memcpy(nlmsg, &resp->payload, resp->payload.hdr.nlmsg_len);

          /* Return address.  REVISIT... this is just a guess. */

          if (from != NULL)
            {
              from->nl_family = resp->payload.msg.ndm_family;
              from->nl_pad    = 0;
              from->nl_pid    = resp->payload.hdr.nlmsg_pid;
              from->nl_groups = resp->payload.hdr.nlmsg_type;
            }

          /* The return value is the payload size */

          ret = resp->payload.hdr.nlmsg_len;
        }
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_GETROUTE
      case RTM_NEWROUTE:
        {
          FAR struct getroute_recvfrom_resplist_s *resp =
            (FAR struct getroute_recvfrom_resplist_s *)entry;

          /* Copy the payload to the user buffer */

          memcpy(nlmsg, &resp->payload, resp->payload.hdr.nlmsg_len);

          /* Return address.  REVISIT... this is just a guess. */

          if (from != NULL)
            {
              from->nl_family = resp->payload.rte.rtm_family;
              from->nl_pad    = 0;
              from->nl_pid    = resp->payload.hdr.nlmsg_pid;
              from->nl_groups = resp->payload.hdr.nlmsg_type;
            }

          /* The return value is the payload size */

          ret = resp->payload.hdr.nlmsg_len;
        }
        break;
#endif

#ifndef NETLINK_DISABLE_NLMSGDONE
      case NLMSG_DONE:
        {
          FAR struct nlroute_msgdone_rsplist_s *resp =
            (FAR struct nlroute_msgdone_rsplist_s *)entry;

          /* Copy the payload to the user buffer */

          resp->payload.hdr.nlmsg_len = sizeof(struct nlmsghdr);
          memcpy(nlmsg, &resp->payload, sizeof(struct nlmsghdr));

          /* Return address.  REVISIT... this is just a guess. */

          if (from != NULL)
            {
              from->nl_family = resp->payload.gen.rtgen_family;
              from->nl_pad    = 0;
              from->nl_pid    = resp->payload.hdr.nlmsg_pid;
              from->nl_groups = 0;
            }

          /* The return value is the payload size */

          ret = sizeof(struct nlmsghdr);
        }
        break;
#endif

      default:
        nerr("ERROR: Unrecognized message type: %u\n",
             entry->msg.nlmsg_type);
        ret = -EIO;
        break;
    }

  kmm_free(entry);
  return ret;
}

#endif /* CONFIG_NETLINK_ROUTE */
