/****************************************************************************
 * include/netpacket/genetlink.h
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

#ifndef __INCLUDE_NETPACKET_GENETLINK_H
#define __INCLUDE_NETPACKET_GENETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <netpacket/netlink.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GENL_NAMSIZ             16      /* Length of family name */

#define GENL_MIN_ID             NLMSG_MIN_TYPE
#define GENL_MAX_ID             1023

#define GENL_HDRLEN             NLMSG_ALIGN(sizeof(struct genlmsghdr))

#define GENL_ADMIN_PERM         0x01
#define GENL_CMD_CAP_DO         0x02
#define GENL_CMD_CAP_DUMP       0x04
#define GENL_CMD_CAP_HASPOL     0x08
#define GENL_UNS_ADMIN_PERM     0x10

/* List of reserved static generic netlink identifiers: */

#define GENL_ID_CTRL            NLMSG_MIN_TYPE
#define GENL_ID_VFS_DQUOT       (NLMSG_MIN_TYPE + 1)
#define GENL_ID_PMCRAID         (NLMSG_MIN_TYPE + 2)

/* Must be last reserved + 1 */

#define GENL_START_ALLOC        (NLMSG_MIN_TYPE + 3)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct genlmsghdr
{
  uint8_t  cmd;
  uint8_t  version;
  uint16_t reserved;
};

/****************************************************************************
 * Controller
 ****************************************************************************/

enum
{
  CTRL_CMD_UNSPEC,
  CTRL_CMD_NEWFAMILY,
  CTRL_CMD_DELFAMILY,
  CTRL_CMD_GETFAMILY,
  CTRL_CMD_NEWOPS,
  CTRL_CMD_DELOPS,
  CTRL_CMD_GETOPS,
  CTRL_CMD_NEWMCAST_GRP,
  CTRL_CMD_DELMCAST_GRP,
  CTRL_CMD_GETMCAST_GRP, /* unused */
  CTRL_CMD_GETPOLICY,
  __CTRL_CMD_MAX,
};

#define CTRL_CMD_MAX            (__CTRL_CMD_MAX - 1)

enum
{
  CTRL_ATTR_UNSPEC,
  CTRL_ATTR_FAMILY_ID,
  CTRL_ATTR_FAMILY_NAME,
  CTRL_ATTR_VERSION,
  CTRL_ATTR_HDRSIZE,
  CTRL_ATTR_MAXATTR,
  CTRL_ATTR_OPS,
  CTRL_ATTR_MCAST_GROUPS,
  CTRL_ATTR_POLICY,
  CTRL_ATTR_OP_POLICY,
  CTRL_ATTR_OP,
  __CTRL_ATTR_MAX,
};

#define CTRL_ATTR_MAX           (__CTRL_ATTR_MAX - 1)

enum
{
  CTRL_ATTR_OP_UNSPEC,
  CTRL_ATTR_OP_ID,
  CTRL_ATTR_OP_FLAGS,
  __CTRL_ATTR_OP_MAX,
};

#define CTRL_ATTR_OP_MAX        (__CTRL_ATTR_OP_MAX - 1)

enum
{
  CTRL_ATTR_MCAST_GRP_UNSPEC,
  CTRL_ATTR_MCAST_GRP_NAME,
  CTRL_ATTR_MCAST_GRP_ID,
  __CTRL_ATTR_MCAST_GRP_MAX,
};

#define CTRL_ATTR_MCAST_GRP_MAX (__CTRL_ATTR_MCAST_GRP_MAX - 1)

enum
{
  CTRL_ATTR_POLICY_UNSPEC,
  CTRL_ATTR_POLICY_DO,
  CTRL_ATTR_POLICY_DUMP,

  __CTRL_ATTR_POLICY_DUMP_MAX,
  CTRL_ATTR_POLICY_DUMP_MAX = __CTRL_ATTR_POLICY_DUMP_MAX - 1
};

#define CTRL_ATTR_POLICY_MAX    (__CTRL_ATTR_POLICY_DUMP_MAX - 1)

#endif /* __INCLUDE_NETPACKET_GENETLINK_H */
