/****************************************************************************
 * net/procfs/net_tcp.c
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
#include <debug.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <nuttx/net/netstats.h>

#include "procfs/procfs.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)

#ifdef CONFIG_NET_IPv6
#  define TCP_LINELEN 180
#else
#  define TCP_LINELEN 120
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t netprocfs_tcpstats(FAR struct netprocfs_file_s *priv,
                                  FAR char *buffer, size_t buflen,
                                  uint8_t domain, int *skip)
{
  int addrlen = (domain == PF_INET) ?
                INET_ADDRSTRLEN : INET6_ADDRSTRLEN;
  FAR struct tcp_conn_s *conn = NULL;
  char remote[addrlen + 1];
  char local[addrlen + 1];
  int len = 0;
  void *laddr;
  void *raddr;

  net_lock();

  local[addrlen] = '\0';
  remote[addrlen] = '\0';

  while ((conn = tcp_nextconn(conn)) != NULL)
    {
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
      if (conn->domain != domain)
        {
          continue;
        }
#endif /* CONFIG_NET_IPv4 && CONFIG_NET_IPv6 */

      if (++(*skip) <= priv->offset)
        {
          continue;
        }

      if (buflen - len < TCP_LINELEN)
        {
          break;
        }

#ifdef CONFIG_NET_IPv4
#  ifdef CONFIG_NET_IPv6
      if (domain == PF_INET)
#  endif /* CONFIG_NET_IPv6 */
        {
          laddr = &conn->u.ipv4.laddr;
          raddr = &conn->u.ipv4.raddr;
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#  ifdef CONFIG_NET_IPv4
      else
#  endif /* CONFIG_NET_IPv4 */
        {
          laddr = &conn->u.ipv6.laddr;
          raddr = &conn->u.ipv6.raddr;
        }
#endif /* CONFIG_NET_IPv6 */

      len += snprintf(buffer + len, buflen - len,
                      "    %2" PRIu8
                      ": %02" PRIx8
                      " %3" PRIx8 " %3" PRIu8
                      " %3" PRIu8
                      " %4" PRIu32
                      " %3" PRIu8
#if CONFIG_NET_SEND_BUFSIZE > 0
                      " %6" PRIu32
#endif
                      " %6u",
                      priv->offset++,
                      conn->tcpstateflags,
                      conn->sconn.s_flags,
                      conn->crefs,
                      conn->timer,
                      (uint32_t)conn->tx_unacked,
                      conn->nrtx,
#if CONFIG_NET_SEND_BUFSIZE > 0
                      tcp_wrbuffer_inqueue_size(conn),
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
    }

  net_unlock();

  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_read_tcpstats
 *
 * Description:
 *   Read and format TCP statistics.
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

ssize_t netprocfs_read_tcpstats(FAR struct netprocfs_file_s *priv,
                                FAR char *buffer, size_t buflen)
{
  int skip = 1;
  int len = 0;

  net_lock();

  if (tcp_nextconn(NULL) != NULL)
    {
      if (priv->offset == 0)
        {
          len = snprintf(buffer, buflen, "TCP sl  "
                                         "st flg ref tmr uack nrt   "
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
          priv->offset = 1;
        }

#ifdef CONFIG_NET_IPv4
      len += netprocfs_tcpstats(priv, buffer + len,
                                buflen - len, PF_INET, &skip);
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
      len += netprocfs_tcpstats(priv, buffer + len,
                                buflen - len, PF_INET6, &skip);
#endif /* CONFIG_NET_IPv6 */
    }

  net_unlock();

  return len;
}

#endif /* CONFIG_NET_TCP && !CONFIG_NET_TCP_NO_STACK */
