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

#include <nuttx/debug.h>
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/clock.h>
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

/* convert_limit() uses the avg of struct xt_rateinfo as the cost in
 * 1 / IPFILTER_LIMIT_SCALE clock tick per 1 / TICK_PER_SEC second.
 */

static_assert(IPFILTER_LIMIT_SCALE == XT_LIMIT_SCALE,
              "convert_limit() needs the same scale as XT_LIMIT_SCALE");

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A match that the filter knows: its name, the size of its data, and the
 * protocol that it needs (0 = any protocol).
 */

struct filter_match_s
{
  FAR const char *name;
  size_t          size;
  uint8_t         proto;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static const struct filter_match_s g_ipv4_matches[] =
{
  {XT_MATCH_NAME_TCP,   sizeof(struct xt_tcp),      IPPROTO_TCP},
  {XT_MATCH_NAME_UDP,   sizeof(struct xt_udp),      IPPROTO_UDP},
  {XT_MATCH_NAME_ICMP,  sizeof(struct ipt_icmp),    IPPROTO_ICMP},
  {XT_MATCH_NAME_LIMIT, sizeof(struct xt_rateinfo), 0}
};
#endif

#ifdef CONFIG_NET_IPv6
static const struct filter_match_s g_ipv6_matches[] =
{
  {XT_MATCH_NAME_TCP,   sizeof(struct xt_tcp),      IPPROTO_TCP},
  {XT_MATCH_NAME_UDP,   sizeof(struct xt_udp),      IPPROTO_UDP},
  {XT_MATCH_NAME_ICMP6, sizeof(struct ip6t_icmp),   IPPROTO_ICMP6},
  {XT_MATCH_NAME_LIMIT, sizeof(struct xt_rateinfo), 0}
};
#endif

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
 * Name: convert_tcpflags
 *
 * Description:
 *   Convert iptables tcp flags match (--tcp-flags, --syn) to ipfilter entry.
 *
 * Input Parameters:
 *   entry    - The ipfilter entry to be filled.
 *   tcp      - The iptables tcp match to be converted.
 *
 ****************************************************************************/

static void convert_tcpflags(FAR struct ipfilter_entry_s *entry,
                             FAR const struct xt_tcp *tcp)
{
  /* Without flags to examine, the flags match all packets, or no packet
   * when it is inverted ("! --tcp-flags NONE NONE").
   */

  if (tcp->flg_mask == 0 && tcp->flg_cmp == 0 &&
      (tcp->invflags & XT_TCP_INV_FLAGS) == 0)
    {
      return;
    }

  entry->flg_mask    = tcp->flg_mask;
  entry->flg_cmp     = tcp->flg_cmp;
  entry->inv_flags   = !!(tcp->invflags & XT_TCP_INV_FLAGS);
  entry->match_flags = 1;
}

/****************************************************************************
 * Name: convert_limit
 *
 * Description:
 *   Convert iptables limit match to the token bucket of ipfilter entry.
 *   One packet costs the time between two packets, in units of
 *   1 / IPFILTER_LIMIT_SCALE clock tick.  avg is in units of
 *   1 / XT_LIMIT_SCALE second, and the two scales are the same, so the
 *   cost is avg * TICK_PER_SEC.  The product fits in 64 bits.
 *
 * Input Parameters:
 *   entry    - The ipfilter entry to be filled.
 *   rate     - The iptables limit match to be converted.
 *
 ****************************************************************************/

static void convert_limit(FAR struct ipfilter_entry_s *entry,
                          FAR const struct xt_rateinfo *rate)
{
  uint64_t cost;

  if (rate->avg == 0 || rate->burst == 0)
    {
      return;
    }

  cost = (uint64_t)rate->avg * TICK_PER_SEC;

  entry->cost        = cost;
  entry->credit_cap  = rate->burst > UINT64_MAX / cost ?
                       UINT64_MAX : cost * rate->burst;
  entry->credit      = entry->credit_cap;
  entry->prev        = clock_systime_ticks();
  entry->match_limit = 1;
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
                           FAR const uint16_t *spts,
                           FAR const uint16_t *dpts, uint8_t invflags)
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
 * Name: convert_match
 *
 * Description:
 *   Convert one iptables match to ipfilter entry.
 *
 * Input Parameters:
 *   entry    - The ipfilter entry to be filled.
 *   match    - The iptables match to be converted.
 *   proto    - The protocol of the iptables entry.
 *
 ****************************************************************************/

static void convert_match(FAR struct ipfilter_entry_s *entry,
                          FAR const struct xt_entry_match *match,
                          uint8_t proto)
{
  FAR const void *data = match + 1;

  if (proto == IPPROTO_TCP &&
      strcmp(match->u.user.name, XT_MATCH_NAME_TCP) == 0)
    {
      FAR const struct xt_tcp *tcp = data;

      convert_tcpudp(entry, tcp->spts, tcp->dpts, tcp->invflags);
      convert_tcpflags(entry, tcp);
    }
  else if (proto == IPPROTO_UDP &&
           strcmp(match->u.user.name, XT_MATCH_NAME_UDP) == 0)
    {
      FAR const struct xt_udp *udp = data;

      convert_tcpudp(entry, udp->spts, udp->dpts, udp->invflags);
    }
  else if (proto == IPPROTO_ICMP &&
           strcmp(match->u.user.name, XT_MATCH_NAME_ICMP) == 0)
    {
      FAR const struct ipt_icmp *icmp = data;

      convert_icmp(entry, icmp->type, icmp->invflags);
    }
  else if (proto == IPPROTO_ICMP6 &&
           strcmp(match->u.user.name, XT_MATCH_NAME_ICMP6) == 0)
    {
      FAR const struct ip6t_icmp *icmp6 = data;

      convert_icmp(entry, icmp6->type, icmp6->invflags);
    }
  else if (strcmp(match->u.user.name, XT_MATCH_NAME_LIMIT) == 0)
    {
      convert_limit(entry, data);
    }
}

/****************************************************************************
 * Name: convert_matches
 *
 * Description:
 *   Convert all iptables matches of an entry to ipfilter entry.  The
 *   matches are between the entry header and the target, and each match
 *   has its own size.  "-p tcp --syn -m limit", for example, has two.
 *
 * Input Parameters:
 *   entry         - The ipfilter entry to be filled.
 *   first         - The first iptables match of the entry.
 *   target_offset - The offset of the target in the iptables entry.
 *   entry_size    - The size of the iptables entry header.
 *   proto         - The protocol of the iptables entry.
 *
 ****************************************************************************/

static void convert_matches(FAR struct ipfilter_entry_s *entry,
                            FAR const struct xt_entry_match *first,
                            size_t target_offset, size_t entry_size,
                            uint8_t proto)
{
  FAR const uint8_t *base = (FAR const uint8_t *)first;
  FAR const struct xt_entry_match *match;
  size_t off = 0;
  size_t len;

  if (target_offset < entry_size + sizeof(struct xt_entry_match))
    {
      ninfo("No match inside entry, skip match conversion.\n");
      return;
    }

  len = target_offset - entry_size;

  while (off + sizeof(struct xt_entry_match) <= len)
    {
      match = (FAR const struct xt_entry_match *)(base + off);
      if (match->u.match_size < sizeof(struct xt_entry_match) ||
          off + match->u.match_size > len)
        {
          nwarn("WARNING: Bad match size %u\n", match->u.match_size);
          return;
        }

      convert_match(entry, match, proto);
      off += match->u.match_size;
    }
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
 * Name: check_matches
 *
 * Description:
 *   Check all iptables matches of an entry, as Linux xt_check_match() does.
 *   Each match must be inside the entry, be large enough for its data,
 *   have a size that is a multiple of XT_ALIGN() (so the next match is
 *   aligned), have a name that the filter knows, and fit the protocol of
 *   the entry.  The conversion skips a match that it
 *   does not know, so an unknown match would make a broader rule.
 *
 * Input Parameters:
 *   head          - The iptables entry.
 *   entry_size    - The size of the iptables entry header.
 *   target_offset - The offset of the target in the iptables entry.
 *   next_offset   - The size of the iptables entry with all its parts.
 *   proto         - The protocol of the iptables entry.
 *   matches       - The matches that the filter knows for the family.
 *   nmatches      - The number of known matches.
 *
 * Returned Value:
 *   OK if all matches are valid, otherwise -EINVAL.
 *
 ****************************************************************************/

static int check_matches(FAR const void *head, size_t entry_size,
                         size_t target_offset, size_t next_offset,
                         uint8_t proto,
                         FAR const struct filter_match_s *matches,
                         size_t nmatches)
{
  FAR const uint8_t *base = (FAR const uint8_t *)head + entry_size;
  FAR const struct xt_entry_match *match;
  size_t off = 0;
  size_t len;
  size_t i;

  if (target_offset < entry_size || target_offset > next_offset)
    {
      nwarn("WARNING: Bad target offset %zu\n", target_offset);
      return -EINVAL;
    }

  len = target_offset - entry_size;

  while (off < len)
    {
      match = (FAR const struct xt_entry_match *)(base + off);
      if (len - off < sizeof(struct xt_entry_match) ||
          match->u.match_size < sizeof(struct xt_entry_match) ||
          match->u.match_size > len - off ||
          XT_ALIGN(match->u.match_size) != match->u.match_size)
        {
          nwarn("WARNING: Bad match size %zu\n",
                len - off < sizeof(struct xt_entry_match) ?
                len - off : (size_t)match->u.match_size);
          return -EINVAL;
        }

      for (i = 0; i < nmatches; i++)
        {
          if (strcmp(match->u.user.name, matches[i].name) == 0)
            {
              break;
            }
        }

      if (i == nmatches)
        {
          nwarn("WARNING: Unsupported match %.*s\n",
                (int)sizeof(match->u.user.name), match->u.user.name);
          return -EINVAL;
        }

      if (match->u.match_size < sizeof(struct xt_entry_match) +
                                matches[i].size)
        {
          nwarn("WARNING: Short %s match %u\n", matches[i].name,
                match->u.match_size);
          return -EINVAL;
        }

      if (matches[i].proto != 0 && matches[i].proto != proto)
        {
          nwarn("WARNING: %s match for protocol %u\n", matches[i].name,
                proto);
          return -EINVAL;
        }

      off += match->u.match_size;
    }

  return OK;
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

  convert_matches(&filter->common, match, entry->target_offset,
                  sizeof(struct ipt_entry), entry->ip.proto);

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

  convert_matches(&filter->common, match, entry->target_offset,
                  sizeof(struct ip6t_entry), entry->ipv6.proto);

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
  FAR const struct xt_entry_target *target;
  int ret;

  /* Check config first. */

  ipt_entry_for_every(entry, repl->entries, repl->size)
    {
      target = IPT_TARGET(entry);

      /* Check all matches and their protocol */

      ret = check_matches(entry, sizeof(struct ipt_entry),
                          entry->target_offset, entry->next_offset,
                          entry->ip.proto, g_ipv4_matches,
                          nitems(g_ipv4_matches));
      if (ret < 0)
        {
          return ret;
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
  FAR const struct xt_entry_target *target;
  int ret;

  /* Check config first. */

  ip6t_entry_for_every(entry, repl->entries, repl->size)
    {
      target = IP6T_TARGET(entry);

      /* Check all matches and their protocol */

      ret = check_matches(entry, sizeof(struct ip6t_entry),
                          entry->target_offset, entry->next_offset,
                          entry->ipv6.proto, g_ipv6_matches,
                          nitems(g_ipv6_matches));
      if (ret < 0)
        {
          return ret;
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
