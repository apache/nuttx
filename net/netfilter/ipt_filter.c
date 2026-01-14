/****************************************************************************
 * net/netfilter/ipt_filter.c
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

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/netfilter/ip_tables.h>
#include <nuttx/net/netfilter/x_tables.h>

#include "ipfilter/ipfilter.h"
#include "netdev/netdev.h"
#include "netfilter/iptables.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FILTER_VALID_HOOKS ((1 << NF_INET_LOCAL_IN) | \
                            (1 << NF_INET_FORWARD)  | \
                            (1 << NF_INET_LOCAL_OUT))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: convert_chain
 *
 * Description:
 *   Convert iptables chain to ipfilter chain.
 *
 ****************************************************************************/

static enum ipfilter_chain_e convert_chain(enum nf_inet_hooks hook)
{
  switch (hook)
    {
      case NF_INET_LOCAL_IN:
        return IPFILTER_CHAIN_INPUT;

      case NF_INET_FORWARD:
        return IPFILTER_CHAIN_FORWARD;

      case NF_INET_LOCAL_OUT:
      default:
        return IPFILTER_CHAIN_OUTPUT;
    }
}

/****************************************************************************
 * Name: convert_invflags
 *
 * Description:
 *   Convert iptables invflags to ipfilter invflags.
 *
 * Input Parameters:
 *   entry    - The ipfilter entry to be filled.
 *   invflags - The iptables invflags to be converted.
 *
 ****************************************************************************/

static void convert_invflags(FAR struct ipfilter_entry_s *entry,
                             uint8_t invflags)
{
  entry->inv_indev  = !!(invflags & IPT_INV_VIA_IN);
  entry->inv_outdev = !!(invflags & IPT_INV_VIA_OUT);
  entry->inv_proto  = !!(invflags & IPT_INV_PROTO);
  entry->inv_srcip  = !!(invflags & IPT_INV_SRCIP);
  entry->inv_dstip  = !!(invflags & IPT_INV_DSTIP);
}

/****************************************************************************
 * Name: convert_tcpudp
 *
 * Description:
 *   Convert iptables tcp/udp match to ipfilter entry.
 *
 * Input Parameters:
 *   entry    - The ipfilter entry to be filled.
 *   spts     - The source ports to be converted.
 *   dpts     - The destination ports to be converted.
 *   invflags - The iptables tcp/udp invflags to be converted.
 *
 ****************************************************************************/

static void convert_tcpudp(FAR struct ipfilter_entry_s *entry,
                           uint16_t spts[2], uint16_t dpts[2],
                           uint8_t invflags)
{
  entry->match.tcpudp.sports[0] = spts[0];
  entry->match.tcpudp.sports[1] = spts[1];
  entry->match.tcpudp.dports[0] = dpts[0];
  entry->match.tcpudp.dports[1] = dpts[1];

  entry->inv_sport = !!(invflags & XT_TCP_INV_SRCPT);
  entry->inv_dport = !!(invflags & XT_TCP_INV_DSTPT);

  entry->match_tcpudp = 1;
}

/****************************************************************************
 * Name: convert_icmp
 *
 * Description:
 *   Convert iptables icmp match to ipfilter entry.
 *
 * Input Parameters:
 *   entry    - The ipfilter entry to be filled.
 *   type     - The icmp type to be converted.
 *   invflags - The iptables icmp invflags to be converted.
 *
 ****************************************************************************/

static void convert_icmp(FAR struct ipfilter_entry_s *entry, uint8_t type,
                         uint8_t invflags)
{
  entry->match.icmp.type = type;
  entry->inv_icmp = !!(invflags & IPT_ICMP_INV);
  entry->match_icmp = 1;
}

/****************************************************************************
 * Name: convert_target
 *
 * Description:
 *   Convert iptables target to ipfilter target.
 *
 * Input Parameters:
 *   target - The iptables target to be converted.
 *
 * Returned Value:
 *   The converted ipfilter target.
 *
 ****************************************************************************/

static uint8_t convert_target(FAR const struct xt_entry_target *target)
{
  if (strcmp(target->u.user.name, XT_REJECT_TARGET) == 0)
    {
      return IPFILTER_TARGET_REJECT;
    }

  if (strcmp(target->u.user.name, XT_STANDARD_TARGET) == 0)
    {
      int verdict = ((FAR const struct xt_standard_target *)target)->verdict;
      verdict = -verdict - 1;

      if (verdict == NF_ACCEPT)
        {
          return IPFILTER_TARGET_ACCEPT;
        }
      else if (verdict == NF_DROP)
        {
          return IPFILTER_TARGET_DROP;
        }
    }

  nwarn("WARNING: Unsupported target %s\n", target->u.user.name);
  return IPFILTER_TARGET_DROP;
}

/****************************************************************************
 * Name: convert_entry
 *
 * Description:
 *   Convert iptables entry to ipfilter entry.
 *
 * Input Parameters:
 *   entry - The iptables entry to be converted.
 *
 * Returned Value:
 *   The converted ipfilter entry.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static FAR struct ipv4_filter_entry_s *
convert_ipv4entry(FAR const struct ipt_entry *entry)
{
  FAR const struct xt_entry_match *match;
  FAR const struct xt_entry_target *target;
  FAR struct ipv4_filter_entry_s *filter =
              (FAR struct ipv4_filter_entry_s *)ipfilter_cfg_alloc(PF_INET);
  if (filter == NULL)
    {
      return NULL;
    }

  match  = IPT_MATCH(entry);
  target = IPT_TARGET(entry);

  /* Convert common fields */

  filter->sip  = entry->ip.src.s_addr;
  filter->dip  = entry->ip.dst.s_addr;
  filter->smsk = entry->ip.smsk.s_addr;
  filter->dmsk = entry->ip.dmsk.s_addr;

  filter->common.indev  = netdev_findbyname(entry->ip.iniface);
  filter->common.outdev = netdev_findbyname(entry->ip.outiface);
  filter->common.proto  = entry->ip.proto;
  filter->common.target = convert_target(target);

  convert_invflags(&filter->common, entry->ip.invflags);

  /* Convert match fields */

  if (entry->target_offset < sizeof(struct xt_entry_match))
    {
      ninfo("No match inside entry, skip match conversion.\n");
      goto skip_match;
    }

  switch (entry->ip.proto)
    {
      case IPPROTO_TCP:
        if (strcmp(match->u.user.name, XT_MATCH_NAME_TCP) == 0)
          {
            FAR struct xt_tcp *tcp = (FAR struct xt_tcp *)(match + 1);
            convert_tcpudp(&filter->common, tcp->spts, tcp->dpts,
                           tcp->invflags);
          }
        break;

      case IPPROTO_UDP:
        if (strcmp(match->u.user.name, XT_MATCH_NAME_TCP) == 0)
          {
            FAR struct xt_udp *udp = (FAR struct xt_udp *)(match + 1);
            convert_tcpudp(&filter->common, udp->spts, udp->dpts,
                           udp->invflags);
          }
        break;

      case IPPROTO_ICMP:
        if (strcmp(match->u.user.name, XT_MATCH_NAME_ICMP) == 0)
          {
            FAR struct ipt_icmp *icmp = (FAR struct ipt_icmp *)(match + 1);
            convert_icmp(&filter->common, icmp->type, icmp->invflags);
          }
        break;

      default:
        break;
    }

skip_match:
  return filter;
}
#endif

#ifdef CONFIG_NET_IPv6
static FAR struct ipv6_filter_entry_s *
convert_ipv6entry(FAR const struct ip6t_entry *entry)
{
  FAR const struct xt_entry_match *match;
  FAR const struct xt_entry_target *target;
  FAR struct ipv6_filter_entry_s *filter =
              (FAR struct ipv6_filter_entry_s *)ipfilter_cfg_alloc(PF_INET6);
  if (filter == NULL)
    {
      return NULL;
    }

  match  = IP6T_MATCH(entry);
  target = IP6T_TARGET(entry);

  /* Convert common fields */

  net_ipv6addr_copy(filter->sip, entry->ipv6.src.s6_addr16);
  net_ipv6addr_copy(filter->dip, entry->ipv6.dst.s6_addr16);
  net_ipv6addr_copy(filter->smsk, entry->ipv6.smsk.s6_addr16);
  net_ipv6addr_copy(filter->dmsk, entry->ipv6.dmsk.s6_addr16);

  filter->common.indev  = netdev_findbyname(entry->ipv6.iniface);
  filter->common.outdev = netdev_findbyname(entry->ipv6.outiface);
  filter->common.proto  = entry->ipv6.proto;
  filter->common.target = convert_target(target);

  convert_invflags(&filter->common, entry->ipv6.invflags);

  /* Convert match fields */

  if (entry->target_offset < sizeof(struct xt_entry_match))
    {
      ninfo("No match inside entry, skip match conversion.\n");
      goto skip_match;
    }

  switch (entry->ipv6.proto)
    {
      case IPPROTO_TCP:
        if (strcmp(match->u.user.name, XT_MATCH_NAME_TCP) == 0)
          {
            FAR struct xt_tcp *tcp = (FAR struct xt_tcp *)(match + 1);
            convert_tcpudp(&filter->common, tcp->spts, tcp->dpts,
                           tcp->invflags);
          }
        break;

      case IPPROTO_UDP:
        if (strcmp(match->u.user.name, XT_MATCH_NAME_TCP) == 0)
          {
            FAR struct xt_udp *udp = (FAR struct xt_udp *)(match + 1);
            convert_tcpudp(&filter->common, udp->spts, udp->dpts,
                           udp->invflags);
          }
        break;

      case IPPROTO_ICMP6:
        if (strcmp(match->u.user.name, XT_MATCH_NAME_ICMP6) == 0)
          {
            FAR struct ip6t_icmp *icmp6 =
                                        (FAR struct ip6t_icmp *)(match + 1);
            convert_icmp(&filter->common, icmp6->type, icmp6->invflags);
          }
        break;

      default:
        break;
    }

skip_match:
  return filter;
}
#endif

/****************************************************************************
 * Name: adjust_filter
 *
 * Description:
 *   Adjust filter config according to the iptables config.
 *
 * Input Parameters:
 *   repl - The config got from user space to control filter table.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static void adjust_ipv4filter(FAR const struct ipt_replace *repl)
{
  FAR const struct ipt_entry *entry;
  FAR const uint8_t *head;
  enum ipfilter_chain_e chain;
  enum nf_inet_hooks hook;
  size_t size;

  for (hook = NF_INET_LOCAL_IN; hook <= NF_INET_LOCAL_OUT; hook++)
    {
      /* Clear all filter config first. */

      chain = convert_chain(hook);
      ipfilter_cfg_clear(PF_INET, chain);

      /* Set filter config according to iptables config. */

      head = (FAR const uint8_t *)repl->entries + repl->hook_entry[hook];
      size = repl->underflow[hook] - repl->hook_entry[hook];

      /* We need the underflow entry as the default of the chain. */

      size++;

      ipt_entry_for_every(entry, head, size)
        {
          FAR struct ipv4_filter_entry_s *filter = convert_ipv4entry(entry);
          if (filter != NULL)
            {
              ipfilter_cfg_add(&filter->common, PF_INET, chain);
            }
          else
            {
              nwarn("WARNING: Failed to convert entry!\n");
            }
        }
    }
}
#endif

#ifdef CONFIG_NET_IPv6
static void adjust_ipv6filter(FAR const struct ip6t_replace *repl)
{
  FAR const struct ip6t_entry *entry;
  FAR const uint8_t *head;
  enum ipfilter_chain_e chain;
  enum nf_inet_hooks hook;
  size_t size;

  for (hook = NF_INET_LOCAL_IN; hook <= NF_INET_LOCAL_OUT; hook++)
    {
      /* Clear all filter config first. */

      chain = convert_chain(hook);
      ipfilter_cfg_clear(PF_INET6, chain);

      /* Set filter config according to iptables config. */

      head = (FAR const uint8_t *)repl->entries + repl->hook_entry[hook];
      size = repl->underflow[hook] - repl->hook_entry[hook];

      /* We need the underflow entry as the default of the chain. */

      size++;

      ip6t_entry_for_every(entry, head, size)
        {
          FAR struct ipv6_filter_entry_s *filter = convert_ipv6entry(entry);
          if (filter != NULL)
            {
              ipfilter_cfg_add(&filter->common, PF_INET6, chain);
            }
          else
            {
              nwarn("WARNING: Failed to convert entry!\n");
            }
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipt_filter_init
 *
 * Description:
 *   Init filter table data.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct ipt_replace *ipt_filter_init(void)
{
  return ipt_alloc_table(XT_TABLE_NAME_FILTER, FILTER_VALID_HOOKS);
}
#endif

#ifdef CONFIG_NET_IPv6
FAR struct ip6t_replace *ip6t_filter_init(void)
{
  return ip6t_alloc_table(XT_TABLE_NAME_FILTER, FILTER_VALID_HOOKS);
}
#endif

/****************************************************************************
 * Name: ipt_filter_apply
 *
 * Description:
 *   Try to apply filter rules, will do nothing if failed.
 *
 * Input Parameters:
 *   repl - The config got from user space to control filter table.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int ipt_filter_apply(FAR const struct ipt_replace *repl)
{
  FAR const struct ipt_entry *entry;
  FAR const struct xt_entry_match *match;
  FAR const struct xt_entry_target *target;

  /* Check config first. */

  ipt_entry_for_every(entry, repl->entries, repl->size)
    {
      match  = IPT_MATCH(entry);
      target = IPT_TARGET(entry);

      /* Check match type matches the protocol */

      if (entry->target_offset >= sizeof(struct xt_entry_match))
        {
          if (strcmp(match->u.user.name, XT_MATCH_NAME_TCP) == 0 &&
              entry->ip.proto != IPPROTO_TCP)
            {
              nwarn("WARNING: TCP match for non-TCP protocol\n");
              return -EINVAL;
            }

          if (strcmp(match->u.user.name, XT_MATCH_NAME_UDP) == 0 &&
              entry->ip.proto != IPPROTO_UDP)
            {
              nwarn("WARNING: UDP match for non-UDP protocol\n");
              return -EINVAL;
            }

          if (strcmp(match->u.user.name, XT_MATCH_NAME_ICMP) == 0 &&
              entry->ip.proto != IPPROTO_ICMP)
            {
              nwarn("WARNING: ICMP match for non-ICMP protocol\n");
              return -EINVAL;
            }
        }

      /* Check target type */

      if (strcmp(target->u.user.name, XT_REJECT_TARGET) != 0 &&
          strcmp(target->u.user.name, XT_STANDARD_TARGET) != 0 &&
          strcmp(target->u.user.name, XT_ERROR_TARGET) != 0)
        {
          nwarn("WARNING: Unsupported target %s\n", target->u.user.name);
          return -EINVAL;
        }
    }

  /* Set config table into ip filter. */

  adjust_ipv4filter(repl);

  return OK;
}
#endif

#ifdef CONFIG_NET_IPv6
int ip6t_filter_apply(FAR const struct ip6t_replace *repl)
{
  FAR const struct ip6t_entry *entry;
  FAR const struct xt_entry_match *match;
  FAR const struct xt_entry_target *target;

  /* Check config first. */

  ip6t_entry_for_every(entry, repl->entries, repl->size)
    {
      match  = IP6T_MATCH(entry);
      target = IP6T_TARGET(entry);

      /* Check match type matches the protocol */

      if (entry->target_offset >= sizeof(struct xt_entry_match))
        {
          if (strcmp(match->u.user.name, XT_MATCH_NAME_TCP) == 0 &&
              entry->ipv6.proto != IPPROTO_TCP)
            {
              nwarn("WARNING: TCP match for non-TCP protocol\n");
              return -EINVAL;
            }

          if (strcmp(match->u.user.name, XT_MATCH_NAME_UDP) == 0 &&
              entry->ipv6.proto != IPPROTO_UDP)
            {
              nwarn("WARNING: UDP match for non-UDP protocol\n");
              return -EINVAL;
            }

          if (strcmp(match->u.user.name, XT_MATCH_NAME_ICMP6) == 0 &&
              entry->ipv6.proto != IPPROTO_ICMP6)
            {
              nwarn("WARNING: ICMP6 match for non-ICMP6 protocol\n");
              return -EINVAL;
            }
        }

      /* Check target type */

      if (strcmp(target->u.user.name, XT_REJECT_TARGET) != 0 &&
          strcmp(target->u.user.name, XT_STANDARD_TARGET) != 0 &&
          strcmp(target->u.user.name, XT_ERROR_TARGET) != 0)
        {
          nwarn("WARNING: Unsupported target %s\n", target->u.user.name);
          return -EINVAL;
        }
    }

  /* Set config table into ip filter. */

  adjust_ipv6filter(repl);

  return OK;
}
#endif
