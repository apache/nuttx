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

#include <netpacket/netlink.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/net/arp.h>

#include "arp/arp.h"
#include "netlink/netlink.h"

#ifdef CONFIG_NETLINK_ROUTE

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define IFA_RTA(r)  \
  ((FAR struct rtattr *)(((FAR char *)(r)) + \
   NLMSG_ALIGN(sizeof(struct ifaddrmsg))))
#define IFA_PAYLOAD(n) \
  NLMSG_PAYLOAD(n, sizeof(struct ifaddrmsg))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* RTM_NEWNEIGH, RTM_DELNEIGH, RTM_GETNEIGH
 *   Add, remove or receive information about a neighbor table entry (e.g.,
 *   an ARP entry).  The message contains an ndmsg structure.
 */

struct nlroute_sendto_request_s
{
  struct nlmsghdr hdr;
  struct ndmsg msg;
  uint8_t data[1];
};

#define SIZEOF_NLROUTE_SENDTO_REQUEST_S(n) \
  (sizeof(struct nlroute_sendto_request_s) + (n) - 1)

struct nlroute_recvfrom_response_s
{
  FAR struct netlink_reqdata_s *flink;
  struct nlmsghdr hdr;
  struct ndmsg msg;
  struct rtattr attr;
  uint8_t data[1];
};

#define SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(n) \
  (sizeof(struct nlroute_recvfrom_response_s) + (n) - 1)

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
  FAR const struct nlroute_sendto_request_s *req;

  DEBUGASSERT(psock != NULL && nlmsg != NULL &&
              nlmsg->nlmsg_len >= sizeof(struct nlmsghdr) &&
              len >= sizeof(struct nlmsghdr) &&
              len >= nlmsg->nlmsg_len && to != NULL &&
              tolen >= sizeof(struct sockaddr_nl));

  req = (FAR const struct nlroute_sendto_request_s *)nlmsg;

#ifdef CONFIG_NET_ARP
  /* REVISIT:  Currently, the only operation supported is retrieving the
   * ARP table in its entirety.
   */

  if (req->hdr.nlmsg_type == RTM_GETNEIGH && req->msg.ndm_family == AF_INET)
    {
      FAR struct nlroute_recvfrom_response_s *entry;
      unsigned int ncopied;
      size_t tabsize;
      size_t paysize;
      size_t entsize;

      /* Preallocate memory to hold the maximum sized ARP table
       * REVISIT:  This is probably excessively large and could cause false
       * memory out conditions.  A better approach would be to actually count
       * the number of valid entries in the ARP table.
       */

      tabsize = CONFIG_NET_ARPTAB_SIZE * sizeof( struct arp_entry_s);
      paysize = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
      entsize = SIZEOF_NETLINK_RESPONSE_S(paysize);

      entry   = (FAR struct nlroute_recvfrom_response_s *)kmm_malloc(entsize);
      if (entry == NULL)
        {
          return -ENOMEM;
        }

      /* Populate the entry */

      memcpy(&entry->hdr, &req->hdr, sizeof(struct nlmsghdr));
      entry->hdr.nlmsg_len = paysize;
      memcpy(&entry->msg, &req->msg, sizeof(struct ndmsg));
      entry->attr.rta_len  = tabsize;
      entry->attr.rta_type = 0;

      /* Lock the network so that the ARP table will be stable, then copy
       * the ARP table into the allocated memory.
       */

      net_lock();
      ncopied = arp_snapshot((FAR struct arp_entry_s *)entry->data,
                             CONFIG_NET_ARPTAB_SIZE);
      net_unlock();

      /* Now we have the real number of valid entries in the ARP table and
       * we can trim the allocation.
       */

      if (ncopied < CONFIG_NET_ARPTAB_SIZE)
        {
          FAR struct nlroute_recvfrom_response_s *newentry;

          tabsize = ncopied * sizeof( struct arp_entry_s);
          paysize = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
          entsize = SIZEOF_NETLINK_RESPONSE_S(paysize);

          newentry = (FAR struct nlroute_recvfrom_response_s *)
            kmm_realloc(entry, entsize);

          if (newentry != NULL)
            {
               entry = newentry;
            }

          entry->hdr.nlmsg_len = paysize;
          entry->attr.rta_len  = tabsize;
        }

      /* Finally, add the data to the list of pending responses */

      netlink_add_response(psock, (FAR struct netlink_response_s *)entry);
      return OK;
    }

#else
  UNUSED(req);
#endif

  /* REVISIT:  Not implemented */

  return -ENOSYS;
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
  FAR struct nlroute_recvfrom_response_s *resp;

  DEBUGASSERT(psock != NULL && nlmsg != NULL &&
              nlmsg->nlmsg_len >= sizeof(struct nlmsghdr) &&
              len >= sizeof(struct nlmsghdr) &&
              from != NULL);

  /* Find the response to this message */

  net_lock();
  resp = (FAR struct nlroute_recvfrom_response_s *)
    netlink_get_response(psock, nlmsg);
  net_unlock();

  if (resp == NULL)
    {
      return -ENOENT;
    }

  if (len < resp->hdr.nlmsg_len)
    {
      kmm_free(resp);
      return -EMSGSIZE;
    }

  memcpy(nlmsg, resp, resp->hdr.nlmsg_len);

  /* Return address.  REVISIT... this is just a guess. */

  from->nl_family = resp->msg.ndm_family;
  from->nl_pad    = 0;
  from->nl_pid    = resp->hdr.nlmsg_pid;
  from->nl_groups = resp->hdr.nlmsg_type;

  return resp->hdr.nlmsg_len;
}

#endif /* CONFIG_NETLINK_ROUTE */
