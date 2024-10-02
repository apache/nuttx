/****************************************************************************
 * include/nuttx/net/netfilter/ip6_tables.h
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

#ifndef __INCLUDE_NUTTX_NET_NETFILTER_IP6_TABLES_H
#define __INCLUDE_NUTTX_NET_NETFILTER_IP6_TABLES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <net/if.h>
#include <netinet/in.h>

#include <nuttx/net/netfilter/netfilter.h>
#include <nuttx/net/netfilter/x_tables.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IP6T_BASE_CTL                   64

#define IP6T_SO_SET_REPLACE             (IP6T_BASE_CTL)
#define IP6T_SO_SET_ADD_COUNTERS        (IP6T_BASE_CTL + 1)
#define IP6T_SO_SET_MAX                 IP6T_SO_SET_ADD_COUNTERS

#define IP6T_SO_GET_INFO                (IP6T_BASE_CTL)
#define IP6T_SO_GET_ENTRIES             (IP6T_BASE_CTL + 1)
#define IP6T_SO_GET_REVISION_MATCH      (IP6T_BASE_CTL + 4)
#define IP6T_SO_GET_REVISION_TARGET     (IP6T_BASE_CTL + 5)
#define IP6T_SO_GET_MAX                 IP6T_SO_GET_REVISION_TARGET

/* Values for "flag" field in struct ip6t_ip6 (general ip6 structure). */

#define IP6T_F_PROTO               0x01    /* Set if rule cares about upper protocols */
#define IP6T_F_TOS                 0x02    /* Match the TOS. */
#define IP6T_F_GOTO                0x04    /* Set if jump is a goto */
#define IP6T_F_MASK                0x07    /* All possible flag bits mask. */

/* Values for "inv" field in struct ip6t_ip6. */

#define IP6T_INV_VIA_IN                 0x01    /* Invert the sense of IN IFACE. */
#define IP6T_INV_VIA_OUT                0x02    /* Invert the sense of OUT IFACE */
#define IP6T_INV_TOS                    0x04    /* Invert the sense of TOS. */
#define IP6T_INV_SRCIP                  0x08    /* Invert the sense of SRC IP. */
#define IP6T_INV_DSTIP                  0x10    /* Invert the sense of DST OP. */
#define IP6T_INV_FRAG                   0x20    /* Invert the sense of FRAG. */
#define IP6T_INV_PROTO                  XT_INV_PROTO
#define IP6T_INV_MASK                   0x7F    /* All possible flag bits mask. */

/* Values for "inv" field for struct ip6t_icmp. */

#define IP6T_ICMP_INV                   0x01    /* Invert the sense of type/code test */

/* Standard return verdict, or do jump. */

#define IP6T_STANDARD_TARGET            XT_STANDARD_TARGET

/* Error verdict. */

#define IP6T_ERROR_TARGET               XT_ERROR_TARGET

#define ip6t_entry_target               xt_entry_target
#define ip6t_entry_match                xt_entry_match

/* Foreach macro for entries. */

#define ip6t_entry_for_every(entry, head, size) \
  for ((entry) = (FAR struct ip6t_entry *)(head); \
       (entry) < (FAR struct ip6t_entry *)((FAR uint8_t *)(head) + (size)); \
       (entry) = (FAR struct ip6t_entry *) \
                     ((FAR uint8_t *)(entry) + (entry)->next_offset))

/* Get pointer to match from an entry pointer. */

#define IP6T_MATCH(e) \
  ((FAR struct xt_entry_match *)((FAR struct ip6t_entry *)(e) + 1))
#define IP6T_TARGET(e) \
  ((FAR struct xt_entry_target *)((FAR uint8_t *)(e) + (e)->target_offset))

/* Auto fill common fields of entry and target. */

#define IP6T_FILL_ENTRY(e, target_name) \
  do \
    { \
      (e)->entry.target_offset = offsetof(typeof(*(e)), target); \
      (e)->entry.next_offset = sizeof(*(e)); \
      (e)->target.target.u.target_size = sizeof(*(e)) - \
                                         (e)->entry.target_offset; \
      strlcpy((e)->target.target.u.user.name, (target_name), \
              sizeof((e)->target.target.u.user.name)); \
    } \
  while(0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ip6t_ip6
{
  /* Source and destination IP6 addr */

  struct in6_addr src;
  struct in6_addr dst;

  /* Mask for src and dest IP6 addr */

  struct in6_addr smsk;
  struct in6_addr dmsk;
  char iniface[IFNAMSIZ];
  char outiface[IFNAMSIZ];
  unsigned char iniface_mask[IFNAMSIZ];
  unsigned char outiface_mask[IFNAMSIZ];

  /* Upper protocol number
   * - The allowed value is 0 (any) or protocol number of last parsable
   *   header, which is 50 (ESP), 59 (No Next Header), 135 (MH), or
   *   the non IPv6 extension headers.
   * - The protocol numbers of IPv6 extension headers except of ESP and
   *   MH do not match any packets.
   * - You also need to set IP6T_FLAGS_PROTO to "flags" to check protocol.
   */

  uint16_t proto;

  /* TOS to match if flags & IP6T_F_TOS */

  uint8_t tos;

  /* Flags word */

  uint8_t flags;

  /* Inverse flags */

  uint8_t invflags;
};

/* This structure defines each of the firewall rules.  Consists of 3
 * parts which are 1) general IP header stuff 2) match specific
 * stuff 3) the target to perform if the rule matches
 */

struct ip6t_entry
{
  struct ip6t_ip6 ipv6;

  /* Mark with fields that we care about. */

  unsigned int nfcache;

  /* Size of ipt_entry + matches */

  uint16_t target_offset;

  /* Size of ipt_entry + matches + target */

  uint16_t next_offset;

  /* Back pointer */

  unsigned int comefrom;

  /* Packet and byte counters. */

  struct xt_counters counters;

  /* The matches (if any), then the target. */

  unsigned char elems[1];
};

/* The argument to IP6T_SO_GET_INFO */

struct ip6t_getinfo
{
  /* Which table: caller fills this in. */

  char name[XT_TABLE_MAXNAMELEN];

  /* Kernel fills these in. */

  /* Which hook entry points are valid: bitmask */

  unsigned int valid_hooks;

  /* Hook entry points: one per netfilter hook. */

  unsigned int hook_entry[NF_INET_NUMHOOKS];

  /* Underflow points. */

  unsigned int underflow[NF_INET_NUMHOOKS];

  /* Number of entries */

  unsigned int num_entries;

  /* Size of entries. */

  unsigned int size;
};

/* The argument to IP6T_SO_GET_ENTRIES. */

struct ip6t_get_entries
{
  /* Which table: user fills this in. */

  char name[XT_TABLE_MAXNAMELEN];

  /* User fills this in: total entry size. */

  unsigned int size;

  /* The entries. */

  struct ip6t_entry entrytable[1];
};

/* The argument to IP6T_SO_SET_REPLACE. */

struct ip6t_replace
{
  /* Which table. */

  char name[XT_TABLE_MAXNAMELEN];

  /* Which hook entry points are valid: bitmask.  You can't change this. */

  unsigned int valid_hooks;

  /* Number of entries */

  unsigned int num_entries;

  /* Total size of new entries */

  unsigned int size;

  /* Hook entry points. */

  unsigned int hook_entry[NF_INET_NUMHOOKS];

  /* Underflow points. */

  unsigned int underflow[NF_INET_NUMHOOKS];

  /* Information about old entries: */

  /* Number of counters (must be equal to current number of entries). */

  unsigned int num_counters;

  /* The old entries' counters. */

  FAR struct xt_counters *counters;

  /* The entries (hang off end: not really an array). */

  struct ip6t_entry entries[1];
};

/* ICMPv6 matching stuff */

struct ip6t_icmp
{
  uint8_t type;     /* type to match */
  uint8_t code[2];  /* range of code */
  uint8_t invflags; /* Inverse flags */
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Helper functions */

static inline FAR struct xt_entry_target *
ip6t_get_target(FAR struct ip6t_entry *e)
{
  return (FAR struct xt_entry_target *)((FAR char *)e + e->target_offset);
}

#endif /* __INCLUDE_NUTTX_NET_NETFILTER_IP6_TABLES_H */
