/****************************************************************************
 * net/netlink/netlink_netfilter.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <netpacket/netlink.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/net/icmp.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netlink.h>

#include "inet/inet.h"
#include "nat/nat.h"
#include "netlink/netlink.h"
#include "utils/utils.h"

#ifdef CONFIG_NETLINK_NETFILTER

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nfnl_sendto_request_s
{
  struct nlmsghdr hdr;
  struct nfgenmsg msg;
};

struct nfnl_info_s
{
  NETLINK_HANDLE handle;
  FAR const struct nfnl_sendto_request_s *req;
};

struct nfnl_ipv4addr_s
{
  struct nfattr attr;
  in_addr_t     addr;
};

struct nfnl_ipv6addr_s
{
  struct nfattr  attr;
  net_ipv6addr_t addr;
};

struct nfnl_attr_u8_s
{
  struct nfattr attr;
  uint8_t       value;
  uint8_t       pad[3];
};

struct nfnl_attr_u16_s
{
  struct nfattr attr;
  uint16_t      value;
  uint16_t      pad[1];
};

/* Struct of a conntrack tuple
 * +------+--------------+-----------------+
 * | attr | CTA_TUPLE_IP | CTA_TUPLE_PROTO |
 * +------+--------------+-----------------+
 */

/* CTA_TUPLE_IP definitions */

struct conntrack_tuple_ipv4_s
{
  struct nfattr attr;
  struct nfnl_ipv4addr_s src;
  struct nfnl_ipv4addr_s dst;
};

struct conntrack_tuple_ipv6_s
{
  struct nfattr attr;
  struct nfnl_ipv6addr_s src;
  struct nfnl_ipv6addr_s dst;
};

/* CTA_TUPLE_PROTO definitions */

struct conntrack_tuple_tcpudp_s
{
  struct nfattr attr;
  struct nfnl_attr_u8_s  proto;
  struct nfnl_attr_u16_s sport;
  struct nfnl_attr_u16_s dport;
};

struct conntrack_tuple_icmp_s
{
  struct nfattr attr;
  struct nfnl_attr_u8_s  proto;
  struct nfnl_attr_u16_s id;
  struct nfnl_attr_u8_s  type;
  struct nfnl_attr_u8_s  code;
};

/* Struct of a conntrack response
 * +-----+-----+-----------------+----------------+
 * | hdr | msg | tuple of origin | tuple of reply |
 * +-----+-----+-----------------+----------------+
 */

struct conntrack_recvfrom_response_s
{
  struct nlmsghdr hdr;
  struct nfgenmsg msg;
  uint8_t         data[1];
};

#define SIZEOF_CTNL_RECVFROM_RESPONSE_S(n) \
  (sizeof(struct conntrack_recvfrom_response_s) + (n) - 1)

struct conntrack_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct conntrack_recvfrom_response_s payload;
};

#define SIZEOF_CTNL_RECVFROM_RSPLIST_S(n) \
  (sizeof(struct conntrack_recvfrom_rsplist_s) + (n) - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_conntrack_tuple_size
 *
 * Description:
 *   Get the size of a CTA_TUPLE. Struct of a conntrack tuple:
 *     +------+--------------+-----------------+
 *     | attr | CTA_TUPLE_IP | CTA_TUPLE_PROTO |
 *     +------+--------------+-----------------+
 *
 * Input Parameters:
 *   domain - The domain of the tuple
 *   proto  - The protocol of the tuple
 *
 * Returned Value:
 *   The size of the tuple.
 *
 ****************************************************************************/

static ssize_t netlink_conntrack_tuple_size(uint8_t domain, uint8_t proto)
{
  size_t size = sizeof(struct nfattr);

  switch (domain)
    {
      case PF_INET:
        size += sizeof(struct conntrack_tuple_ipv4_s);
        break;

      case PF_INET6:
        size += sizeof(struct conntrack_tuple_ipv6_s);
        break;

      default:
        return -EINVAL;
    }

  switch (proto)
    {
      case IPPROTO_TCP:
      case IPPROTO_UDP:
        size += sizeof(struct conntrack_tuple_tcpudp_s);
        break;

      case IPPROTO_ICMP:
      case IPPROTO_ICMP6:
        size += sizeof(struct conntrack_tuple_icmp_s);
        break;

      default:
        return -EINVAL;
    }

  return size;
}

/****************************************************************************
 * Name: netlink_conntrack_fill_ip
 *
 * Description:
 *   Fill the data of a CTA_TUPLE_IP.
 *
 * Input Parameters:
 *   buf    - The buffer to fill
 *   domain - The domain of the addresses
 *   src    - The source address
 *   dst    - The destination address
 *
 * Returned Value:
 *   The size of the filled data.
 *
 ****************************************************************************/

static size_t netlink_conntrack_fill_ip(FAR void *buf, uint8_t domain,
                                        FAR const void *src,
                                        FAR const void *dst)
{
#ifdef CONFIG_NET_NAT44
  if (domain == PF_INET)
    {
      FAR struct conntrack_tuple_ipv4_s *tuple_ipv4 = buf;

      tuple_ipv4->attr.nfa_len  = sizeof(struct conntrack_tuple_ipv4_s);
      tuple_ipv4->attr.nfa_type = CTA_TUPLE_IP | NFNL_NFA_NEST;

      tuple_ipv4->src.attr.nfa_len  = NFA_LENGTH(sizeof(in_addr_t));
      tuple_ipv4->src.attr.nfa_type = CTA_IP_V4_SRC;
      net_ipv4addr_hdrcopy(&tuple_ipv4->src.addr, src);

      tuple_ipv4->dst.attr.nfa_len  = NFA_LENGTH(sizeof(in_addr_t));
      tuple_ipv4->dst.attr.nfa_type = CTA_IP_V4_DST;
      net_ipv4addr_hdrcopy(&tuple_ipv4->dst.addr, dst);

      return tuple_ipv4->attr.nfa_len;
    }
#endif

#ifdef CONFIG_NET_NAT66
  if (domain == PF_INET6)
    {
      FAR struct conntrack_tuple_ipv6_s *tuple_ipv6 = buf;

      tuple_ipv6->attr.nfa_len  = sizeof(struct conntrack_tuple_ipv6_s);
      tuple_ipv6->attr.nfa_type = CTA_TUPLE_IP | NFNL_NFA_NEST;

      tuple_ipv6->src.attr.nfa_len  = NFA_LENGTH(sizeof(net_ipv6addr_t));
      tuple_ipv6->src.attr.nfa_type = CTA_IP_V6_SRC;
      net_ipv6addr_hdrcopy(tuple_ipv6->src.addr, src);

      tuple_ipv6->dst.attr.nfa_len  = NFA_LENGTH(sizeof(net_ipv6addr_t));
      tuple_ipv6->dst.attr.nfa_type = CTA_IP_V6_DST;
      net_ipv6addr_hdrcopy(tuple_ipv6->dst.addr, dst);

      return tuple_ipv6->attr.nfa_len;
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: netlink_conntrack_fill_proto
 *
 * Description:
 *   Fill the data of a CTA_TUPLE_PROTO.
 *
 * Input Parameters:
 *   buf   - The buffer to fill
 *   proto - The protocol of the tuple
 *   sport - The source port
 *   dport - The destination port
 *   reply - True if the tuple is a reply
 *
 * Returned Value:
 *   The size of the filled data.
 *
 ****************************************************************************/

static size_t netlink_conntrack_fill_proto(FAR void *buf, uint8_t proto,
                                           uint16_t sport, uint16_t dport,
                                           bool reply)
{
  switch (proto)
    {
#ifdef CONFIG_NET_TCP
      case IPPROTO_TCP:
#endif
#ifdef CONFIG_NET_UDP
      case IPPROTO_UDP:
#endif
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
        {
          FAR struct conntrack_tuple_tcpudp_s *tuple_tcpudp = buf;

          tuple_tcpudp->attr.nfa_len  =
                                     sizeof(struct conntrack_tuple_tcpudp_s);
          tuple_tcpudp->attr.nfa_type = CTA_TUPLE_PROTO | NFNL_NFA_NEST;

          tuple_tcpudp->proto.attr.nfa_len  = NFA_LENGTH(sizeof(uint8_t));
          tuple_tcpudp->proto.attr.nfa_type = CTA_PROTO_NUM;
          tuple_tcpudp->proto.value         = proto;

          tuple_tcpudp->sport.attr.nfa_len  = NFA_LENGTH(sizeof(uint16_t));
          tuple_tcpudp->sport.attr.nfa_type = CTA_PROTO_SRC_PORT;
          tuple_tcpudp->sport.value         = sport;

          tuple_tcpudp->dport.attr.nfa_len  = NFA_LENGTH(sizeof(uint16_t));
          tuple_tcpudp->dport.attr.nfa_type = CTA_PROTO_DST_PORT;
          tuple_tcpudp->dport.value         = dport;

          return tuple_tcpudp->attr.nfa_len;
        }
#endif

#ifdef CONFIG_NET_ICMP
      case IPPROTO_ICMP:
#endif
#ifdef CONFIG_NET_ICMPv6
      case IPPROTO_ICMP6:
#endif
#if defined(CONFIG_NET_ICMP) || defined(CONFIG_NET_ICMPv6)
        {
          FAR struct conntrack_tuple_icmp_s *tuple_icmp = buf;

          tuple_icmp->attr.nfa_len  = sizeof(struct conntrack_tuple_icmp_s);
          tuple_icmp->attr.nfa_type = CTA_TUPLE_PROTO | NFNL_NFA_NEST;

          tuple_icmp->proto.attr.nfa_len  = NFA_LENGTH(sizeof(uint8_t));
          tuple_icmp->proto.attr.nfa_type = CTA_PROTO_NUM;
          tuple_icmp->proto.value         = proto;

          tuple_icmp->id.attr.nfa_len     = NFA_LENGTH(sizeof(uint16_t));
          tuple_icmp->id.value            = reply ? dport : sport;

          tuple_icmp->type.attr.nfa_len   = NFA_LENGTH(sizeof(uint8_t));

          tuple_icmp->code.attr.nfa_len   = NFA_LENGTH(sizeof(uint8_t));
          tuple_icmp->code.value          = 0;

#ifdef CONFIG_NET_ICMP
          if (proto == IPPROTO_ICMP)
            {
              tuple_icmp->id.attr.nfa_type   = CTA_PROTO_ICMP_ID;
              tuple_icmp->type.attr.nfa_type = CTA_PROTO_ICMP_TYPE;
              tuple_icmp->type.value         = reply ? ICMP_ECHO_REPLY :
                                                       ICMP_ECHO_REQUEST;
              tuple_icmp->code.attr.nfa_type = CTA_PROTO_ICMP_CODE;
            }
#endif

#ifdef CONFIG_NET_ICMPv6
          if (proto == IPPROTO_ICMP6)
            {
              tuple_icmp->id.attr.nfa_type   = CTA_PROTO_ICMPV6_ID;
              tuple_icmp->type.attr.nfa_type = CTA_PROTO_ICMPV6_TYPE;
              tuple_icmp->type.value         = reply ? ICMPv6_ECHO_REPLY :
                                                       ICMPv6_ECHO_REQUEST;
              tuple_icmp->code.attr.nfa_type = CTA_PROTO_ICMPV6_CODE;
            }
#endif

          return tuple_icmp->attr.nfa_len;
        }
#endif
    }

  return 0;
}

/****************************************************************************
 * Name: netlink_get_ipv4/ipv6_conntrack
 *
 * Description:
 *   Get the conntrack response corresponding to an NAT entry.
 *
 ****************************************************************************/

static FAR struct netlink_response_s *
netlink_get_conntrack(FAR const struct nlmsghdr *req, uint16_t flags,
                      uint8_t type, uint8_t domain, uint8_t proto,
                      FAR const void *lipaddr, uint16_t lport,
                      FAR const void *eipaddr, uint16_t eport,
                      FAR const void *ripaddr, uint16_t rport)
{
  FAR struct conntrack_recvfrom_rsplist_s *entry;
  FAR struct nfattr *tuple;
  ssize_t tuple_size = netlink_conntrack_tuple_size(domain, proto);
  size_t  offset     = 0;
  size_t  allocsize;
  size_t  rspsize;

  if (tuple_size < 0)
    {
      nerr("ERROR: Failed to get tuple size in response.\n");
      return NULL;
    }

  rspsize   = SIZEOF_CTNL_RECVFROM_RESPONSE_S(tuple_size * 2);
  allocsize = SIZEOF_CTNL_RECVFROM_RSPLIST_S(tuple_size * 2);

  entry = kmm_malloc(allocsize);
  if (entry == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return NULL;
    }

  entry->payload.hdr.nlmsg_len   = rspsize;
  entry->payload.hdr.nlmsg_type  = type | (NFNL_SUBSYS_CTNETLINK << 8);
  entry->payload.hdr.nlmsg_flags = flags;
  entry->payload.hdr.nlmsg_seq   = req ? req->nlmsg_seq : 0;
  entry->payload.hdr.nlmsg_pid   = req ? req->nlmsg_pid : 0;

  entry->payload.msg.nfgen_family = domain;
  entry->payload.msg.version      = NFNETLINK_V0;
  entry->payload.msg.res_id       = 0;

  /* CTA_TUPLE_ORIG */

  tuple = (FAR struct nfattr *)&entry->payload.data[offset];
  tuple->nfa_len  = tuple_size;
  tuple->nfa_type = CTA_TUPLE_ORIG | NFNL_NFA_NEST;
  offset += sizeof(struct nfattr);

  offset += netlink_conntrack_fill_ip(&entry->payload.data[offset], domain,
                                      lipaddr, ripaddr);
  offset += netlink_conntrack_fill_proto(&entry->payload.data[offset], proto,
                                         lport, rport, false);

  DEBUGASSERT(offset == tuple_size);

  /* CTA_TUPLE_REPLY */

  tuple = (FAR struct nfattr *)&entry->payload.data[offset];
  tuple->nfa_len  = tuple_size;
  tuple->nfa_type = CTA_TUPLE_REPLY | NFNL_NFA_NEST;
  offset += sizeof(struct nfattr);

  offset += netlink_conntrack_fill_ip(&entry->payload.data[offset], domain,
                                      ripaddr, eipaddr);
  offset += netlink_conntrack_fill_proto(&entry->payload.data[offset], proto,
                                         rport, eport, true);

  DEBUGASSERT(offset == tuple_size * 2);

  return (FAR struct netlink_response_s *)entry;
}

#ifdef CONFIG_NET_NAT44
static FAR struct netlink_response_s *
netlink_get_ipv4_conntrack(FAR const struct nlmsghdr *req,
                           FAR const ipv4_nat_entry_t *entry,
                           uint16_t flags, uint8_t type)
{
#ifndef CONFIG_NET_NAT44_SYMMETRIC
  const in_addr_t any = INADDR_ANY;
#endif
  return netlink_get_conntrack(req, flags, type, PF_INET, entry->protocol,
                               &entry->local_ip, entry->local_port,
                               &entry->external_ip, entry->external_port,
#ifdef CONFIG_NET_NAT44_SYMMETRIC
                               &entry->peer_ip, entry->peer_port
#else
                               &any, 0 /* Zero-address */
#endif
                              );
}
#endif

#ifdef CONFIG_NET_NAT66
static FAR struct netlink_response_s *
netlink_get_ipv6_conntrack(FAR const struct nlmsghdr *req,
                           FAR const ipv6_nat_entry_t *entry,
                           uint16_t flags, uint8_t type)
{
  return netlink_get_conntrack(req, flags, type, PF_INET6, entry->protocol,
                               entry->local_ip, entry->local_port,
                               entry->external_ip, entry->external_port,
#ifdef CONFIG_NET_NAT66_SYMMETRIC
                               entry->peer_ip, entry->peer_port
#else
                               g_ipv6_unspecaddr, 0 /* Zero-address */
#endif
      );
}
#endif

/****************************************************************************
 * Name: netlink_add_ipv4/ipv6_conntrack
 *
 * Description:
 *   Add the conntrack response of an IPv4/IPv6 NAT entry.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT44
static void netlink_add_ipv4_conntrack(FAR ipv4_nat_entry_t *entry,
                                       FAR void *arg)
{
  FAR struct nfnl_info_s *info = arg;
  FAR struct netlink_response_s *resp;
  uint16_t flags = NLM_F_MULTI | NLM_F_DUMP_FILTERED;

  resp = netlink_get_ipv4_conntrack(&info->req->hdr, entry, flags,
                                    IPCTNL_MSG_CT_NEW);
  if (resp != NULL)
    {
      netlink_add_response(info->handle, resp);
    }
}
#endif

#ifdef CONFIG_NET_NAT66
static void netlink_add_ipv6_conntrack(FAR ipv6_nat_entry_t *entry,
                                       FAR void *arg)
{
  FAR struct nfnl_info_s *info = arg;
  FAR struct netlink_response_s *resp;
  uint16_t flags = NLM_F_MULTI | NLM_F_DUMP_FILTERED;

  resp = netlink_get_ipv6_conntrack(&info->req->hdr, entry, flags,
                                    IPCTNL_MSG_CT_NEW);

  if (resp != NULL)
    {
      netlink_add_response(info->handle, resp);
    }
}
#endif

/****************************************************************************
 * Name: netlink_list_conntrack
 *
 * Description:
 *   Return the entire NAT table.
 *
 ****************************************************************************/

static int netlink_list_conntrack(NETLINK_HANDLE handle,
                                 FAR const struct nfnl_sendto_request_s *req)
{
  struct nfnl_info_s info;
  uint8_t type = NFNL_MSG_TYPE(req->hdr.nlmsg_type);
  if (type != IPCTNL_MSG_CT_GET)
    {
      return -ENOSYS;
    }

  info.handle = handle;
  info.req    = req;

  switch (req->msg.nfgen_family)
    {
#ifdef CONFIG_NET_NAT44
      case AF_INET:
        ipv4_nat_entry_foreach(netlink_add_ipv4_conntrack, &info);
        break;
#endif

#ifdef CONFIG_NET_NAT66
      case AF_INET6:
        ipv6_nat_entry_foreach(netlink_add_ipv6_conntrack, &info);
        break;
#endif

      default:
        return -ENOSYS;
    }

  return netlink_add_terminator(handle, &req->hdr, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_netfilter_sendto
 *
 * Description:
 *   Perform the sendto() operation for the NETLINK_NETFILTER protocol.
 *
 ****************************************************************************/

ssize_t netlink_netfilter_sendto(NETLINK_HANDLE handle,
                                 FAR const struct nlmsghdr *nlmsg,
                                 size_t len, int flags,
                                 FAR const struct sockaddr_nl *to,
                                 socklen_t tolen)
{
  FAR const struct nfnl_sendto_request_s *req =
    (FAR const struct nfnl_sendto_request_s *)nlmsg;
  ssize_t ret = -ENOSYS;
  uint8_t subsys;

  DEBUGASSERT(handle != NULL && nlmsg != NULL &&
              nlmsg->nlmsg_len >= sizeof(struct nlmsghdr) &&
              len >= sizeof(struct nlmsghdr) &&
              len >= nlmsg->nlmsg_len && to != NULL &&
              tolen >= sizeof(struct sockaddr_nl));

  /* Split subsys and type from nlmsg->nlmsg_type */

  subsys = NFNL_SUBSYS_ID(nlmsg->nlmsg_type);

  /* Handle according to the subsystem */

  switch (subsys)
    {
      case NFNL_SUBSYS_CTNETLINK:
        ret = netlink_list_conntrack(handle, req);
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
 * Name: netlink_conntrack_notify
 *
 * Description:
 *   Perform the conntrack broadcast for the NETLINK_NETFILTER protocol.
 *
 * Input Parameters:
 *   type      - The type of the message, IPCTNL_MSG_CT_*
 *   domain    - The domain of the message
 *   nat_entry - The NAT entry
 *
 ****************************************************************************/

void netlink_conntrack_notify(uint8_t type, uint8_t domain,
                              FAR const void *nat_entry)
{
  FAR struct netlink_response_s *resp;
  uint16_t flags;
  int group;

  switch (type)
    {
      case IPCTNL_MSG_CT_NEW:
        group = NFNLGRP_CONNTRACK_NEW;
        flags = NLM_F_EXCL | NLM_F_CREATE;
        break;

      case IPCTNL_MSG_CT_DELETE:
        group = NFNLGRP_CONNTRACK_DESTROY;
        flags = 0;
        break;

      default:
        return;
    }

  switch (domain)
    {
#ifdef CONFIG_NET_NAT44
      case PF_INET:
        resp = netlink_get_ipv4_conntrack(NULL, nat_entry, flags, type);
        break;
#endif

#ifdef CONFIG_NET_NAT66
      case PF_INET6:
        resp = netlink_get_ipv6_conntrack(NULL, nat_entry, flags, type);
        break;
#endif

      default:
        return;
    }

  if (resp != NULL)
    {
      netlink_add_broadcast(group, resp);
    }
}

#endif /* CONFIG_NETLINK_NETFILTER */
