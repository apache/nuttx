/****************************************************************************
 * net/procfs/net_udp.c
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
#include <stdio.h>
#include <string.h>
#include <nuttx/debug.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <nuttx/net/netstats.h>

#include "procfs/procfs.h"
#include "udp/udp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK

#ifdef CONFIG_NET_IPv6
#  define UDP_LINELEN 180
#else
#  define UDP_LINELEN 120
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int netprocfs_udpline(FAR struct netprocfs_file_s *priv,
                             FAR char *buffer, size_t buflen,
                             FAR struct udp_conn_s *conn)
{
  uint8_t domain = net_ip_domain_select(conn->domain, PF_INET, PF_INET6);
  int addrlen = (domain == PF_INET) ?
                INET_ADDRSTRLEN : INET6_ADDRSTRLEN;
  char remote[INET6_ADDRSTRLEN];
  char local[INET6_ADDRSTRLEN];
  FAR void *laddr;
  FAR void *raddr;
  int len;

  laddr = net_ip_binding_laddr(&conn->u, domain);
  raddr = net_ip_binding_raddr(&conn->u, domain);

  len = snprintf(buffer, buflen,
                 "    %2u"
                 ": %3" PRIx8
#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
                 " %6" PRIu32
#endif
                 " %6u",
                 priv->lines,
                 conn->sconn.s_flags,
#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
                 udp_wrbuffer_inqueue_size(conn),
#endif
                 (conn->readahead) ? conn->readahead->io_pktlen : 0);

  len += snprintf(buffer + len, buflen - len,
                  " %*s:%-6" PRIu16 " %*s:%-6" PRIu16 "\n",
                  (domain == PF_INET6) ? addrlen / 2 : addrlen,
                  inet_ntop(domain, laddr, local, addrlen),
                  ntohs(conn->lport),
                  (domain == PF_INET6) ? addrlen / 2 : addrlen,
                  inet_ntop(domain, raddr, remote, addrlen),
                  ntohs(conn->rport));

  return len;
}

/****************************************************************************
 * Name: netprocfs_udpstats
 *
 * Description:
 *   Write the lines of the connections after the last one written.  The
 *   walk finds its place by the list id of the last connection written, as
 *   netprocfs_tcpstats() (net_tcp.c) does: a read of the file to its end
 *   lists once each connection that is in the list from the first read to
 *   the end, and it ends.
 *
 * Assumptions:
 *   The UDP connection list is locked.
 *
 ****************************************************************************/

static ssize_t netprocfs_udpstats(FAR struct netprocfs_file_s *priv,
                                  FAR char *buffer, size_t buflen)
{
  FAR struct udp_conn_s *conn = NULL;
  int len = 0;

  while ((conn = udp_nextconn(conn)) != NULL)
    {
      if (!UDP_LISTID_AFTER(conn->listid, priv->last) ||
          UDP_LISTID_AFTER(conn->listid, priv->limit))
        {
          continue;
        }

      if (buflen - len < UDP_LINELEN)
        {
          break;
        }

      len += netprocfs_udpline(priv, buffer + len, buflen - len, conn);
      priv->last = conn->listid;
      priv->lines++;
    }

  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_read_udpstats
 *
 * Description:
 *   Read and format UDP statistics.
 *
 * Input Parameters:
 *   priv - A reference to the network procfs file structure
 *   buffer - The user-provided buffer into which network status will be
 *            returned.
 *   bulen  - The size in bytes of the user provided buffer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

ssize_t netprocfs_read_udpstats(FAR struct netprocfs_file_s *priv,
                                FAR char *buffer, size_t buflen)
{
  FAR struct udp_conn_s *conn;
  int len = 0;

  udp_conn_list_lock();
  conn = udp_nextconn(NULL);
  if (priv->lines == 0 && conn != NULL)
    {
      /* The first read: the header, and the bounds of this read, the
       * connections from the head of the list to the last one now.
       */

      priv->last  = conn->listid - 1;
      priv->limit = udp_conn_lastid();

      len = snprintf(buffer, buflen, "UDP sl "
                                     " flg   "
#if CONFIG_NET_SEND_BUFSIZE > 0
                                     "txsz   "
#endif
                                     "rxsz "
                                      "%-*s "
                                      "%-*s\n"
                                      ,
                                      INET6_ADDRSTRLEN / 2,
                                      "local_address",
                                      INET6_ADDRSTRLEN / 2,
                                      "remote_address"
                                      );
      priv->lines = 1;
    }

  if (priv->lines > 0)
    {
      len += netprocfs_udpstats(priv, buffer + len, buflen - len);
    }

  udp_conn_list_unlock();
  return len;
}

#endif /* NET_UDP_HAVE_STACK */
