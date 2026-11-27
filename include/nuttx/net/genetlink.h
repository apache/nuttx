/****************************************************************************
 * include/nuttx/net/genetlink.h
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

#ifndef __INCLUDE_NUTTX_NET_GENETLINK_H
#define __INCLUDE_NUTTX_NET_GENETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <netpacket/genetlink.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GENL_MAX_FAMILIES                8
#define GENL_MCAST_GRPS_MAX              8

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct netlink_conn_s;

/* Multicast group definition - matches Linux struct genl_multicast_group */

struct genl_multicast_group
{
  char name[GENL_NAMSIZ];
};

/* Generic netlink family - simplified version of Linux struct genl_family.
 *
 * Each family provides:
 *   - name:        Family name (e.g. "nl80211"), used by CTRL_CMD_GETFAMILY
 *   - version:     Protocol version
 *   - maxattr:     Maximum attribute number understood by this family
 *   - hdrsize:     Size of user-specific header in messages
 *   - cmd_handler: Callback to handle incoming messages for this family
 *   - mcgrps/n_mcgrps: Multicast group definitions
 *
 * The 'id' and 'mcgrp_offset' fields are assigned dynamically by
 * genl_register_family().
 */

struct genl_family
{
  uint16_t id;                /* Assigned dynamically; do not set */
  uint16_t mcgrp_offset;      /* Assigned dynamically; do not set */
  char     name[GENL_NAMSIZ];
  uint8_t  version;
  uint8_t  hdrsize;
  uint16_t maxattr;

  /* Command handler - called for every message sent to this family. */

  int (*cmd_handler)(FAR struct netlink_conn_s *conn,
                     FAR const struct nlmsghdr *nlmsg);

  /* Multicast groups */

  FAR const struct genl_multicast_group *mcgrps;
  uint8_t n_mcgrps;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: genl_register_family
 *
 * Description:
 *   Register a generic netlink family.  A unique family ID is allocated
 *   from the GENL_START_ALLOC range and stored in family->id.
 *   Multicast group IDs are also allocated and stored via
 *   family->mcgrp_offset.
 *
 *   This mirrors the Linux kernel genl_register_family() API.
 *
 * Input Parameters:
 *   family - Pointer to the family structure to register.  The 'name',
 *            'version', 'maxattr', 'cmd_handler', 'mcgrps', and
 *            'n_mcgrps' fields must be filled in by the caller.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int genl_register_family(FAR struct genl_family *family);

/****************************************************************************
 * Name: genl_unregister_family
 *
 * Description:
 *   Unregister a previously registered generic netlink family.
 *
 * Input Parameters:
 *   family - Pointer to the family structure to unregister.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int genl_unregister_family(FAR const struct genl_family *family);

/****************************************************************************
 * Name: genl_family_find_byname
 *
 * Description:
 *   Look up a registered family by name.
 *
 * Input Parameters:
 *   name - The family name to search for.
 *
 * Returned Value:
 *   Pointer to the family on success; NULL if not found.
 *
 ****************************************************************************/

FAR const struct genl_family *genl_family_find_byname(FAR const char *name);

/****************************************************************************
 * Name: genl_family_find_byid
 *
 * Description:
 *   Look up a registered family by ID.
 *
 * Input Parameters:
 *   id - The family ID to search for.
 *
 * Returned Value:
 *   Pointer to the family on success; NULL if not found.
 *
 ****************************************************************************/

FAR const struct genl_family *genl_family_find_byid(uint16_t id);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_GENETLINK_H */
