/****************************************************************************
 * net/netlink/netlink_generic.c
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

#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/netlink.h>
#include <nuttx/net/genetlink.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "netlink/netlink.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Family registry - stores pointers to registered families */

static FAR struct genl_family *g_genl_families[GENL_MAX_FAMILIES];
static mutex_t g_genl_lock = NXMUTEX_INITIALIZER;
static int g_genl_family_count;
static uint16_t g_genl_next_id = GENL_START_ALLOC;
static uint16_t g_genl_next_mcgrp = 1;  /* Multicast group IDs start at 1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: add_mcast_grp_attr
 *
 * Description:
 *   Helper to serialize one multicast group as nested rtattr.
 *
 * Input Parameters:
 *   buf    - Buffer to write the attribute data.
 *   offset - Current offset in the buffer.
 *   index  - Index of the multicast group.
 *   gname  - Name of the multicast group.
 *   gid    - ID of the multicast group.
 *
 * Returned Value:
 *   The aligned length of the added attribute.
 *
 ****************************************************************************/

static int add_mcast_grp_attr(FAR uint8_t *buf, int offset,
                              int index, FAR const char *gname, int gid)
{
  FAR struct rtattr *grp = (FAR struct rtattr *)(buf + offset);
  FAR uint8_t *ptr = (FAR uint8_t *)RTA_DATA(grp);
  FAR struct rtattr *attr;
  int len = 0;

  grp->rta_type = index | NLA_F_NESTED;

  /* Group Name */

  attr = (FAR struct rtattr *)ptr;
  attr->rta_type = CTRL_ATTR_MCAST_GRP_NAME;
  attr->rta_len = RTA_LENGTH(strlen(gname) + 1);
  strlcpy((FAR char *)RTA_DATA(attr), gname, GENL_NAMSIZ);
  len += RTA_ALIGN(attr->rta_len);

  /* Group ID */

  attr = (FAR struct rtattr *)(ptr + len);
  attr->rta_type = CTRL_ATTR_MCAST_GRP_ID;
  attr->rta_len = RTA_LENGTH(4);
  *(FAR uint32_t *)RTA_DATA(attr) = gid;
  len += RTA_ALIGN(attr->rta_len);

  grp->rta_len = RTA_LENGTH(len);
  return RTA_ALIGN(grp->rta_len);
}

/****************************************************************************
 * Name: send_family_info
 *
 * Description:
 *   Build and queue a CTRL_CMD_NEWFAMILY response for one family.
 *   This is used both for CTRL_CMD_GETFAMILY replies and DUMP mode.
 *
 * Input Parameters:
 *   conn    - The netlink connection to send the response to.
 *   req     - The request message header.
 *   family  - The generic netlink family to report.
 *   dump    - True if this is part of a dump operation (NLM_F_MULTI).
 *
 * Returned Value:
 *   The length of the netlink message on success.
 *   A negated error value is returned if an unexpected error occurred.
 *
 ****************************************************************************/

static int send_family_info(FAR struct netlink_conn_s *conn,
                            FAR const struct nlmsghdr *req,
                            FAR const struct genl_family *family,
                            bool dump)
{
  FAR struct netlink_response_s *response;
  FAR struct genlmsghdr *genl;
  FAR struct nlmsghdr *nlh;
  FAR struct rtattr *attr;
  FAR uint8_t *attr_ptr;
  FAR uint8_t *buf;
  int attr_len;
  int buf_len;
  int offset;

  /* Calculate required buffer size:
   * base headers + fixed attrs + family name + mcast groups
   */

  buf_len = NLMSG_HDRLEN + sizeof(struct genlmsghdr)
         + RTA_SPACE(2)                          /* FAMILY_ID */
         + RTA_SPACE(GENL_NAMSIZ)                /* FAMILY_NAME */
         + RTA_SPACE(4) * 3;                     /* VERSION+HDRSIZE+MAXATTR */

  if (family->mcgrps && family->n_mcgrps > 0)
    {
      int i;

      for (i = 0; i < family->n_mcgrps; i++)
        {
          buf_len += RTA_SPACE(RTA_SPACE(GENL_NAMSIZ) + RTA_SPACE(4));
        }

      buf_len += RTA_SPACE(0); /* mcast root header */
    }

  response = (FAR struct netlink_response_s *)
    kmm_zalloc(sizeof(struct netlink_response_s) + NLMSG_ALIGN(buf_len));
  if (response == NULL)
    {
      return -ENOMEM;
    }

  buf = (FAR uint8_t *)&response->msg;
  nlh = (FAR struct nlmsghdr *)buf;

  nlh->nlmsg_type = GENL_ID_CTRL;
  nlh->nlmsg_flags = dump ? NLM_F_MULTI : 0;
  nlh->nlmsg_seq = req->nlmsg_seq;
  nlh->nlmsg_pid = req->nlmsg_pid;

  genl = (FAR struct genlmsghdr *)NLMSG_DATA(nlh);
  genl->cmd = CTRL_CMD_NEWFAMILY;
  genl->version = 1;

  offset = NLMSG_ALIGN(sizeof(struct genlmsghdr));
  attr_ptr = (FAR uint8_t *)genl + offset;

  /* CTRL_ATTR_FAMILY_ID */

  attr = (FAR struct rtattr *)attr_ptr;
  attr->rta_type = CTRL_ATTR_FAMILY_ID;
  attr->rta_len = RTA_LENGTH(2);
  *(FAR uint16_t *)RTA_DATA(attr) = family->id;
  attr_len = RTA_ALIGN(attr->rta_len);
  attr_ptr += attr_len;
  offset += attr_len;

  /* CTRL_ATTR_FAMILY_NAME */

  attr = (FAR struct rtattr *)attr_ptr;
  attr->rta_type = CTRL_ATTR_FAMILY_NAME;
  attr->rta_len = RTA_LENGTH(strlen(family->name) + 1);
  strlcpy((FAR char *)RTA_DATA(attr), family->name, GENL_NAMSIZ);
  attr_len = RTA_ALIGN(attr->rta_len);
  attr_ptr += attr_len;
  offset += attr_len;

  /* CTRL_ATTR_VERSION */

  attr = (FAR struct rtattr *)attr_ptr;
  attr->rta_type = CTRL_ATTR_VERSION;
  attr->rta_len = RTA_LENGTH(4);
  *(FAR uint32_t *)RTA_DATA(attr) = family->version;
  attr_len = RTA_ALIGN(attr->rta_len);
  attr_ptr += attr_len;
  offset += attr_len;

  /* CTRL_ATTR_HDRSIZE */

  attr = (FAR struct rtattr *)attr_ptr;
  attr->rta_type = CTRL_ATTR_HDRSIZE;
  attr->rta_len = RTA_LENGTH(4);
  *(FAR uint32_t *)RTA_DATA(attr) = family->hdrsize;
  attr_len = RTA_ALIGN(attr->rta_len);
  attr_ptr += attr_len;
  offset += attr_len;

  /* CTRL_ATTR_MAXATTR */

  attr = (FAR struct rtattr *)attr_ptr;
  attr->rta_type = CTRL_ATTR_MAXATTR;
  attr->rta_len = RTA_LENGTH(4);
  *(FAR uint32_t *)RTA_DATA(attr) = family->maxattr;
  attr_len = RTA_ALIGN(attr->rta_len);
  attr_ptr += attr_len;
  offset += attr_len;

  /* CTRL_ATTR_MCAST_GROUPS — only if the family has multicast groups */

  if (family->mcgrps && family->n_mcgrps > 0)
    {
      FAR struct rtattr *mcast = (FAR struct rtattr *)attr_ptr;
      FAR uint8_t *grp;
      int grp_len = 0;
      int i;

      mcast->rta_type = CTRL_ATTR_MCAST_GROUPS | NLA_F_NESTED;
      grp = (FAR uint8_t *)RTA_DATA(mcast);

      for (i = 0; i < family->n_mcgrps; i++)
        {
          grp_len += add_mcast_grp_attr(grp, grp_len,
                                        i + 1,
                                        family->mcgrps[i].name,
                                        family->mcgrp_offset + i);
        }

      mcast->rta_len = RTA_LENGTH(grp_len);
      attr_len = RTA_ALIGN(mcast->rta_len);
      attr_ptr += attr_len;
      offset += attr_len;
    }

  nlh->nlmsg_len = NLMSG_LENGTH(offset);

  netlink_add_response(conn, response);
  return nlh->nlmsg_len;
}

/****************************************************************************
 * Name: send_ack_response
 *
 * Description:
 *   Send an ACK (NLMSG_ERROR with error=0) response.
 *
 * Input Parameters:
 *   conn - The netlink connection to send the ACK to.
 *   req  - The request message header.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *   A negated error value is returned if an unexpected error occurred.
 *
 ****************************************************************************/

static int send_ack_response(FAR struct netlink_conn_s *conn,
                             FAR const struct nlmsghdr *req)
{
  FAR struct netlink_response_s *response;
  FAR struct nlmsgerr *err;
  int payload_len = sizeof(struct nlmsgerr);

  response = (FAR struct netlink_response_s *)
    kmm_zalloc(sizeof(struct netlink_response_s) + NLMSG_ALIGN(payload_len));
  if (response == NULL)
    {
      return -ENOMEM;
    }

  response->msg.nlmsg_len = NLMSG_LENGTH(payload_len);
  response->msg.nlmsg_type = NLMSG_ERROR;
  response->msg.nlmsg_flags = 0;
  response->msg.nlmsg_seq = req->nlmsg_seq;
  response->msg.nlmsg_pid = req->nlmsg_pid;

  err = (FAR struct nlmsgerr *)NLMSG_DATA(&response->msg);
  err->error = 0;
  memcpy(&err->msg, req, sizeof(struct nlmsghdr));

  netlink_add_response(conn, response);
  return 0;
}

/****************************************************************************
 * Name: netlink_ctrl_getfamily
 *
 * Description:
 *   Handle a CTRL_CMD_GETFAMILY request.  Supports both single-family
 *   lookup by name and NLM_F_DUMP to enumerate all families.
 *
 * Input Parameters:
 *   conn - The netlink connection handling the request.
 *   req  - The request message header.
 *
 * Returned Value:
 *   The length of the request message on success.
 *   A negated error value is returned if an unexpected error occurred.
 *
 ****************************************************************************/

static int netlink_ctrl_getfamily(FAR struct netlink_conn_s *conn,
                                  FAR const struct nlmsghdr *req)
{
  FAR const struct genl_family *fam;
  FAR struct genlmsghdr *ghdr;
  FAR struct nlattr *na;
  FAR char *family_name = NULL;
  int rem;
  int ret;
  int i;

  ghdr = (FAR struct genlmsghdr *)NLMSG_DATA(req);
  na = (FAR struct nlattr *)((FAR char *)ghdr + GENL_HDRLEN);
  rem = req->nlmsg_len - NLMSG_HDRLEN - GENL_HDRLEN;

  while (sizeof(struct nlattr) <= rem)
    {
      if (na->nla_len < sizeof(struct nlattr) || na->nla_len > rem)
        {
          break;
        }

      if (nla_type(na) == CTRL_ATTR_FAMILY_NAME)
        {
          /* The name is used with strcmp() and printed with %s, so take it
           * only when the payload really holds a string that fits.
           */

          FAR char *data = nla_data(na);
          int len = nla_len(na);

          if (len > 0 && len <= GENL_NAMSIZ &&
              memchr(data, '\0', len) != NULL)
            {
              family_name = data;
            }
        }

      na = nla_next(na, &rem);
    }

  if (req->nlmsg_flags & NLM_F_DUMP)
    {
      FAR const struct genl_family *snapshot[GENL_MAX_FAMILIES];
      FAR struct netlink_response_s *done_resp;
      FAR struct nlmsghdr *done_hdr;
      struct genl_family ctrl_fam;
      int nfam = 0;

      /* Dump the built-in nlctrl family */

      memset(&ctrl_fam, 0, sizeof(ctrl_fam));
      ctrl_fam.id = GENL_ID_CTRL;
      strlcpy(ctrl_fam.name, "nlctrl", GENL_NAMSIZ);
      ctrl_fam.version = 1;
      ctrl_fam.maxattr = CTRL_ATTR_MAX;
      send_family_info(conn, req, &ctrl_fam, true);

      /* Dump all registered families.  Take a snapshot of the table under
       * the lock, then send outside it: send_family_info() ends in
       * netlink_add_response(), which takes the netlink lock, and nesting
       * that inside g_genl_lock would fix an order that nothing else
       * needs.
       *
       * A family unregistered after the snapshot still appears in this
       * one dump.  The pointer stays valid, because the family is owned
       * by its caller and genl_unregister_family() does not free it.
       */

      nxmutex_lock(&g_genl_lock);
      for (i = 0; i < g_genl_family_count; i++)
        {
          if (g_genl_families[i])
            {
              snapshot[nfam++] = g_genl_families[i];
            }
        }

      nxmutex_unlock(&g_genl_lock);

      for (i = 0; i < nfam; i++)
        {
          send_family_info(conn, req, snapshot[i], true);
        }

      /* Send NLMSG_DONE terminator */

      done_resp = kmm_zalloc(sizeof(*done_resp));
      if (done_resp == NULL)
        {
          return -ENOMEM;
        }

      done_hdr = &done_resp->msg;
      done_hdr->nlmsg_len = NLMSG_LENGTH(0);
      done_hdr->nlmsg_type = NLMSG_DONE;
      done_hdr->nlmsg_flags = NLM_F_MULTI;
      done_hdr->nlmsg_seq = req->nlmsg_seq;
      done_hdr->nlmsg_pid = req->nlmsg_pid;

      netlink_add_response(conn, done_resp);
      return req->nlmsg_len;
    }

  /* Single family lookup by name */

  if (family_name)
    {
      /* Check built-in nlctrl */

      if (strcmp(family_name, "nlctrl") == 0)
        {
          struct genl_family ctrl_fam;

          memset(&ctrl_fam, 0, sizeof(ctrl_fam));
          ctrl_fam.id = GENL_ID_CTRL;
          strlcpy(ctrl_fam.name, "nlctrl", GENL_NAMSIZ);
          ctrl_fam.version = 1;
          ctrl_fam.maxattr = CTRL_ATTR_MAX;
          send_family_info(conn, req, &ctrl_fam, false);

          if (req->nlmsg_flags & NLM_F_ACK)
            {
              send_ack_response(conn, req);
            }

          return req->nlmsg_len;
        }

      /* Search registered families */

      fam = genl_family_find_byname(family_name);
      if (fam)
        {
          ret = send_family_info(conn, req, fam, false);
          if (ret < 0)
            {
              return ret;
            }

          if (req->nlmsg_flags & NLM_F_ACK)
            {
              send_ack_response(conn, req);
            }

          return req->nlmsg_len;
        }
    }

  nerr("netlink_ctrl_getfamily: family '%s' not found\n",
       family_name ? family_name : "(null)");
  return -ENOENT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: genl_register_family
 *
 * Description:
 *   Register a generic netlink family with the netlink subsystem.
 *   Allocates a dynamic family ID and multicast group IDs if needed.
 *
 * Input Parameters:
 *   family - Pointer to the generic netlink family structure to register.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *   -EEXIST is returned if a family with the same name already exists.
 *   -ENOMEM is returned if the maximum number of families is reached.
 *
 ****************************************************************************/

int genl_register_family(FAR struct genl_family *family)
{
  int ret = 0;
  int i;

  DEBUGASSERT(family != NULL && family->name[0] != '\0');

  nxmutex_lock(&g_genl_lock);

  /* Check for duplicate */

  for (i = 0; i < g_genl_family_count; i++)
    {
      if (g_genl_families[i] &&
          strcmp(g_genl_families[i]->name, family->name) == 0)
        {
          ret = -EEXIST;
          goto out;
        }
    }

  if (g_genl_family_count >= GENL_MAX_FAMILIES)
    {
      ret = -ENOMEM;
      goto out;
    }

  /* The ids only ever increase, so refuse the registration once either
   * space is used up rather than hand out an id that cannot be addressed.
   * A multicast group lives in the 32-bit mask of a connection, so group
   * 32 is the last one that NETLINK_ADD_MEMBERSHIP can subscribe to and
   * netlink_add_broadcast() can deliver to.
   */

  if (g_genl_next_id > GENL_MAX_ID)
    {
      ret = -ENOSPC;
      goto out;
    }

  if (family->mcgrps && family->n_mcgrps > 0 &&
      g_genl_next_mcgrp + family->n_mcgrps - 1 > 32)
    {
      ret = -ENOSPC;
      goto out;
    }

  /* Allocate a dynamic family ID */

  family->id = g_genl_next_id++;

  /* Allocate multicast group IDs */

  if (family->mcgrps && family->n_mcgrps > 0)
    {
      family->mcgrp_offset = g_genl_next_mcgrp;
      g_genl_next_mcgrp += family->n_mcgrps;
    }
  else
    {
      family->mcgrp_offset = 0;
    }

  /* Store in registry */

  g_genl_families[g_genl_family_count++] = family;

  ninfo("genl: registered family '%s' id=%d mcgrp_offset=%d\n",
        family->name, family->id, family->mcgrp_offset);

out:
  nxmutex_unlock(&g_genl_lock);
  return ret;
}

/****************************************************************************
 * Name: genl_unregister_family
 *
 * Description:
 *   Unregister a generic netlink family from the netlink subsystem.
 *
 * Input Parameters:
 *   family - Pointer to the generic netlink family structure to unregister.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *   -ENOENT is returned if the family was not found in the registry.
 *
 ****************************************************************************/

int genl_unregister_family(FAR const struct genl_family *family)
{
  int ret = -ENOENT;
  int i;

  DEBUGASSERT(family != NULL);

  nxmutex_lock(&g_genl_lock);

  for (i = 0; i < g_genl_family_count; i++)
    {
      if (g_genl_families[i] == family)
        {
          /* Shift remaining entries down */

          g_genl_family_count--;
          g_genl_families[i] = g_genl_families[g_genl_family_count];
          g_genl_families[g_genl_family_count] = NULL;
          ret = 0;
          break;
        }
    }

  nxmutex_unlock(&g_genl_lock);
  return ret;
}

/****************************************************************************
 * Name: genl_family_find_byname
 *
 * Description:
 *   Find a registered generic netlink family by name.
 *
 * Input Parameters:
 *   name - The name of the family to find.
 *
 * Returned Value:
 *   Pointer to the generic netlink family structure if found.
 *   NULL is returned if the family was not found.
 *
 ****************************************************************************/

FAR const struct genl_family *genl_family_find_byname(FAR const char *name)
{
  FAR const struct genl_family *found = NULL;
  int i;

  DEBUGASSERT(name != NULL);

  nxmutex_lock(&g_genl_lock);

  for (i = 0; i < g_genl_family_count; i++)
    {
      if (g_genl_families[i] &&
          strcmp(g_genl_families[i]->name, name) == 0)
        {
          found = g_genl_families[i];
          break;
        }
    }

  nxmutex_unlock(&g_genl_lock);
  return found;
}

/****************************************************************************
 * Name: genl_family_find_byid
 *
 * Description:
 *   Find a registered generic netlink family by ID.
 *
 * Input Parameters:
 *   id - The family ID to search for.
 *
 * Returned Value:
 *   Pointer to the generic netlink family structure if found.
 *   NULL is returned if the family was not found.
 *
 ****************************************************************************/

FAR const struct genl_family *genl_family_find_byid(uint16_t id)
{
  FAR const struct genl_family *found = NULL;
  int i;

  nxmutex_lock(&g_genl_lock);

  for (i = 0; i < g_genl_family_count; i++)
    {
      if (g_genl_families[i] && g_genl_families[i]->id == id)
        {
          found = g_genl_families[i];
          break;
        }
    }

  nxmutex_unlock(&g_genl_lock);
  return found;
}

/****************************************************************************
 * Name: netlink_generic_sendto
 *
 * Description:
 *   Perform the sendto() operation for the NETLINK_GENERIC protocol.
 *   Dispatches to the controller (GENL_ID_CTRL) or to a registered
 *   family's cmd_handler based on nlmsg_type.
 *
 * Input Parameters:
 *   conn  - The netlink connection.
 *   msg   - The netlink message to send.
 *   len   - Length of the message.
 *   flags - Send flags.
 *   to    - Destination address (sockaddr_nl).
 *   tolen - Length of the destination address.
 *
 * Returned Value:
 *   The length of the request message on success.
 *   A negated error value is returned if an unexpected error occurred.
 *
 ****************************************************************************/

ssize_t netlink_generic_sendto(FAR struct netlink_conn_s *conn,
                               FAR const struct nlmsghdr *msg,
                               size_t len, int flags,
                               FAR const struct sockaddr_nl *to,
                               socklen_t tolen)
{
  FAR const struct genl_family *family;

  DEBUGASSERT(conn != NULL && msg != NULL);

  /* netlink_sendmsg() only guarantees a complete nlmsghdr.  Every generic
   * netlink message carries a genlmsghdr after it, which the dispatch below
   * reads, so refuse a request that is too short for one.
   */

  if (msg->nlmsg_len < NLMSG_LENGTH(GENL_HDRLEN))
    {
      nerr("genl: message too short: %u\n", (unsigned int)msg->nlmsg_len);
      return -EINVAL;
    }

  /* Handle controller messages (CTRL_CMD_GETFAMILY etc.) */

  if (msg->nlmsg_type == GENL_ID_CTRL)
    {
      FAR struct genlmsghdr *ghdr;

      ghdr = (FAR struct genlmsghdr *)NLMSG_DATA(msg);
      if (ghdr->cmd == CTRL_CMD_GETFAMILY)
        {
          return netlink_ctrl_getfamily(conn, msg);
        }

      nerr("genl: unsupported ctrl cmd %d\n", ghdr->cmd);
      return -EOPNOTSUPP;
    }

  /* Dispatch to registered family by nlmsg_type (== family ID) */

  family = genl_family_find_byid(msg->nlmsg_type);
  if (family && family->cmd_handler)
    {
      return family->cmd_handler(conn, msg);
    }

  nerr("genl: no family for nlmsg_type=%d\n", msg->nlmsg_type);
  return -ENOENT;
}
