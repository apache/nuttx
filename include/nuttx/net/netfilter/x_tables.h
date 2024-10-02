/****************************************************************************
 * include/nuttx/net/netfilter/x_tables.h
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

#ifndef __INCLUDE_NUTTX_NET_NETFILTER_X_TABLES_H
#define __INCLUDE_NUTTX_NET_NETFILTER_X_TABLES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#include <nuttx/net/netfilter/netfilter.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XT_FUNCTION_MAXNAMELEN  30
#define XT_EXTENSION_MAXNAMELEN 29
#define XT_TABLE_MAXNAMELEN     32

#define XT_INV_PROTO            0x40 /* Invert the sense of PROTO. */

/* Values for "inv" field in struct xt_tcp. */

#define XT_TCP_INV_SRCPT        0x01 /* Invert the sense of source ports. */
#define XT_TCP_INV_DSTPT        0x02 /* Invert the sense of dest ports. */
#define XT_TCP_INV_FLAGS        0x04 /* Invert the sense of TCP flags. */
#define XT_TCP_INV_OPTION       0x08 /* Invert the sense of option test. */
#define XT_TCP_INV_MASK         0x0F /* All possible flags. */

/* Values for "invflags" field in struct xt_udp. */

#define XT_UDP_INV_SRCPT        0x01 /* Invert the sense of source ports. */
#define XT_UDP_INV_DSTPT        0x02 /* Invert the sense of dest ports. */
#define XT_UDP_INV_MASK         0x03 /* All possible flags. */

/* Target names */

#define XT_STANDARD_TARGET      ""   /* Standard return verdict, or do jump. */
#define XT_ERROR_TARGET         "ERROR"
#define XT_MASQUERADE_TARGET    "MASQUERADE"
#define XT_REJECT_TARGET        "REJECT"

/* Match name to simplify our code */

#define XT_MATCH_NAME_TCP       "tcp"
#define XT_MATCH_NAME_UDP       "udp"
#define XT_MATCH_NAME_ICMP      "icmp"
#define XT_MATCH_NAME_ICMP6     "icmp6"

/* Table name to simplify our code */

#define XT_TABLE_NAME_FILTER    "filter"

/* For standard target */

#define XT_RETURN               (-NF_REPEAT - 1)

#define XT_ALIGN(s)                                           \
  (((s) + ((typeof(s))(__alignof__(struct _xt_align)) - 1)) & \
   ~((typeof(s))(__alignof__(struct _xt_align)) - 1))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* this is a dummy structure to find out the alignment requirement for a
 * struct containing all the fundamental data types that are used in
 * ipt_entry, ip6t_entry and arpt_entry.  This sucks, and it is a hack.
 * It will be my personal pleasure to remove it -HW
 */

struct _xt_align
{
  uint8_t  u8;
  uint16_t u16;
  uint32_t u32;
  uint64_t u64;
};

struct xt_entry_target
{
  union
    {
      struct
        {
          uint16_t target_size;

          char name[XT_EXTENSION_MAXNAMELEN];
          uint8_t revision;
        } user;
      struct
        {
          uint16_t target_size;
        } kernel;

      uint16_t target_size; /* Total length */
    } u;

  unsigned char data[1];
};

struct xt_standard_target
{
  struct xt_entry_target target;
  int verdict;
};

struct xt_error_target
{
  struct xt_entry_target target;
  char errorname[XT_FUNCTION_MAXNAMELEN];
};

struct xt_counters
{
  /* Packet and byte counters */

  uint64_t pcnt;
  uint64_t bcnt;
};

/* The argument to IPT_SO_ADD_COUNTERS. */

struct xt_counters_info
{
  /* Which table. */

  char name[XT_TABLE_MAXNAMELEN];
  unsigned int num_counters;

  /* The counters (actually `number' of these). */

  struct xt_counters counters[1];
};

struct xt_match; /* reserved */

struct xt_entry_match
{
  union
  {
    struct
    {
      uint16_t match_size;

      /* Used by userspace */

      char name[XT_EXTENSION_MAXNAMELEN];
      uint8_t revision;
    } user;
    struct
    {
      uint16_t match_size;

      /* Used inside the kernel */

      FAR struct xt_match *match;
    } kernel;

    /* Total length */

    uint16_t match_size;
  } u;
  unsigned char data[1];
};

/* TCP matching stuff */

struct xt_tcp
{
  uint16_t spts[2]; /* Source port range. */
  uint16_t dpts[2]; /* Destination port range. */
  uint8_t option;   /* TCP Option iff non-zero */
  uint8_t flg_mask; /* TCP flags mask byte */
  uint8_t flg_cmp;  /* TCP flags compare byte */
  uint8_t invflags; /* Inverse flags */
};

/* UDP matching stuff */

struct xt_udp
{
  uint16_t spts[2]; /* Source port range. */
  uint16_t dpts[2]; /* Destination port range. */
  uint8_t invflags; /* Inverse flags */
};

#endif /* __INCLUDE_NUTTX_NET_NETFILTER_X_TABLES_H */
