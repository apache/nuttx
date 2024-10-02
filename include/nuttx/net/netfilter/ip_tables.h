/****************************************************************************
 * include/nuttx/net/netfilter/ip_tables.h
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

#ifndef __INCLUDE_NUTTX_NET_NETFILTER_IP_TABLES_H
#define __INCLUDE_NUTTX_NET_NETFILTER_IP_TABLES_H

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

#define IPT_BASE_CTL               64

#define IPT_SO_SET_REPLACE         (IPT_BASE_CTL)
#define IPT_SO_SET_ADD_COUNTERS    (IPT_BASE_CTL + 1)
#define IPT_SO_SET_MAX             IPT_SO_SET_ADD_COUNTERS

#define IPT_SO_GET_INFO            (IPT_BASE_CTL)
#define IPT_SO_GET_ENTRIES         (IPT_BASE_CTL + 1)
#define IPT_SO_GET_REVISION_MATCH  (IPT_BASE_CTL + 2)
#define IPT_SO_GET_REVISION_TARGET (IPT_BASE_CTL + 3)
#define IPT_SO_GET_MAX             IPT_SO_GET_REVISION_TARGET

/* Values for "inv" field in struct ipt_ip. */

#define IPT_INV_VIA_IN             0x01    /* Invert the sense of IN IFACE. */
#define IPT_INV_VIA_OUT            0x02    /* Invert the sense of OUT IFACE */
#define IPT_INV_TOS                0x04    /* Invert the sense of TOS. */
#define IPT_INV_SRCIP              0x08    /* Invert the sense of SRC IP. */
#define IPT_INV_DSTIP              0x10    /* Invert the sense of DST OP. */
#define IPT_INV_FRAG               0x20    /* Invert the sense of FRAG. */
#define IPT_INV_PROTO              XT_INV_PROTO
#define IPT_INV_MASK               0x7F    /* All possible flag bits mask. */

/* Values for "inv" field for struct ipt_icmp. */

#define IPT_ICMP_INV               0x01    /* Invert the sense of type/code test */

/* Standard return verdict, or do jump. */

#define IPT_STANDARD_TARGET        XT_STANDARD_TARGET

/* Error verdict. */

#define IPT_ERROR_TARGET           XT_ERROR_TARGET

#define ipt_standard_target        xt_standard_target
#define ipt_entry_target           xt_entry_target
#define ipt_entry_match            xt_entry_match

/* Foreach macro for entries. */

#define ipt_entry_for_every(entry, head, size) \
  for ((entry) = (FAR struct ipt_entry *)(head); \
       (entry) < (FAR struct ipt_entry *)((FAR uint8_t *)(head) + (size)); \
       (entry) = (FAR struct ipt_entry *) \
                     ((FAR uint8_t *)(entry) + (entry)->next_offset))

/* Get pointer to match / target from an entry pointer. */

#define IPT_MATCH(e) \
  ((FAR struct xt_entry_match *)((FAR struct ipt_entry *)(e) + 1))
#define IPT_TARGET(e) \
  ((FAR struct xt_entry_target *)((FAR uint8_t *)(e) + (e)->target_offset))

/* Auto fill common fields of entry and target. */

#define IPT_FILL_ENTRY(e, target_name) \
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

struct ipt_ip
{
  /* Source and destination IP addr and mask */

  struct in_addr src;
  struct in_addr dst;
  struct in_addr smsk;
  struct in_addr dmsk;

  /* IN/OUT interface name and mask */

  char    iniface[IFNAMSIZ];
  char    outiface[IFNAMSIZ];
  uint8_t iniface_mask[IFNAMSIZ];
  uint8_t outiface_mask[IFNAMSIZ];

  uint16_t proto;    /* Protocol, 0 = ANY */
  uint8_t  flags;    /* Flags word */
  uint8_t  invflags; /* Inverse flags */
};

/* This structure defines each of the firewall rules.
 * Consists of 3 parts which are
 * 1) general IP header stuff
 * 2) match specific stuff
 * 3) the target to perform if the rule matches
 */

struct ipt_entry
{
  struct ipt_ip ip;

  unsigned int nfcache;        /* Mark with fields that we care about. */

  uint16_t target_offset;      /* Size of ipt_entry + matches */
  uint16_t next_offset;        /* Size of ipt_entry + matches + target */

  unsigned int comefrom;       /* Back pointer */

  struct xt_counters counters; /* Packet and byte counters. */

  unsigned char elems[1];      /* The matches (if any), then the target. */
};

/* Note 1: How entries are organized in following interface arguments.
 *           ----------------------------------------------------------
 * Metadata  |       num_entries = n, size = sizeof(n entries)        |
 *           |--------------------------------------------------------|
 * Entry 0   |   entry   |   matches  |             target            |
 *           | ipt_entry | <optional> | xt_entry_target | target data |
 *           |--------------------------------------------------------|
 * Entry 1   |   entry   |   matches  |             target            |
 *           | ipt_entry | <optional> | xt_entry_target | target data |
 *           |--------------------------------------------------------|
 *   ...     |                        ...                             |
 *           |--------------------------------------------------------|
 * Entry n-2 |   entry   |   matches  |             target            |
 *           | ipt_entry | <optional> | xt_entry_target | target data |
 *           |--------------------------------------------------------|
 * Entry n-1 |   entry   |   matches  |        xt_error_target        |
 *           | ipt_entry |   0 Byte   | xt_entry_target |  errorname  |
 *           |--------------------------------------------------------|
 *
 * Note 2: How hooks are organized in following interface arguments.
 *   -----------------------------
 *   |          Metadata         | example: valid_hooks = 0,1,2,4
 *   |---------------------------|
 *   |--------- <hook 0> --------|
 *   | Entry 0 [user set target] | <-- hook_entry[hook 0]
 *   | Entry 1 [standard target] | <-- underflow[hook 0]
 *   |--------- <hook 1> --------|
 *   | Entry 2 [standard target] | <-- hook_entry[hook 1], underflow[hook 1]
 *   |--------- <hook 2> --------|
 *   | Entry 3 [user set target] | <-- hook_entry[hook 2]
 *   | Entry 4 [user set target] |
 *   | Entry 5 [standard target] | <-- underflow[hook 2]
 *   |--------- <hook 4> --------|
 *   | Entry 6 [standard target] | <-- hook_entry[hook 4], underflow[hook 4]
 *   |------ <last target> ------|
 *   | Entry 7 [error target]    |
 *   -----------------------------
 *
 * Note 3:
 *   It can be any value in invalid hook's hook_entry & underflow, we default
 * set them to 0, but should never read invalid hook's pointer.
 *
 * Note 4:
 *   Last entry of a hook should be an entry with standard target (without
 * any matches). It's used to define default behavior of the hook.
 *
 * Note 5:
 *   Last entry of the whole entry table should be an entry with error target
 * without any matches.
 */

/* The argument to IPT_SO_GET_INFO */

struct ipt_getinfo
{
  /* Which table: caller fills this in. */

  char name[XT_TABLE_MAXNAMELEN];

  /* Kernel fills these in. */

  /* Which hook entry points are valid: bitmask */

  unsigned int valid_hooks;

  /* Hook entry and underflow points:
   * one per netfilter hook, points to the first/last entry of the hook.
   */

  unsigned int hook_entry[NF_INET_NUMHOOKS];
  unsigned int underflow[NF_INET_NUMHOOKS];

  unsigned int num_entries; /* Number of entries */
  unsigned int size;        /* Size of entries. */
};

/* The argument to IPT_SO_SET_REPLACE. */

struct ipt_replace
{
  /* Which table. */

  char name[XT_TABLE_MAXNAMELEN];

  /* Which hook entry points are valid: bitmask.  You can't change this. */

  unsigned int valid_hooks;

  unsigned int num_entries; /* Number of entries */
  unsigned int size;        /* Total size of new entries */

  /* Hook entry and underflow points:
   * one per netfilter hook, points to the first/last entry of the hook.
   */

  unsigned int hook_entry[NF_INET_NUMHOOKS];
  unsigned int underflow[NF_INET_NUMHOOKS];

  /* The old entries' counters and the number of counters (we ignore them) */

  unsigned int num_counters;
  FAR struct xt_counters *counters;

  /* The entries (hang off end: not really an array). */

  struct ipt_entry entries[0];
};

/* The argument to IPT_SO_GET_ENTRIES. */

struct ipt_get_entries
{
  /* Which table: user fills this in. */

  char name[XT_TABLE_MAXNAMELEN];

  /* User fills this in: total entry size. */

  unsigned int size;

  /* The entries. */

  struct ipt_entry entrytable[0];
};

/* ICMP matching stuff */

struct ipt_icmp
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
ipt_get_target(FAR struct ipt_entry *e)
{
  return (FAR struct xt_entry_target *)((FAR char *)e + e->target_offset);
}

#endif /* __INCLUDE_NUTTX_NET_NETFILTER_IP_TABLES_H */
