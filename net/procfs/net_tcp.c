/****************************************************************************
 * net/procfs/net_tcp.c
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

#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "procfs/procfs.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void netprocfs_tcpline(FAR struct netprocfs_file_s *priv,
                              FAR struct tcp_conn_s *conn)
{
  FAR char *buffer = priv->line;
  size_t buflen = NET_LINELEN;
  uint8_t domain = net_ip_domain_select(conn->domain, PF_INET, PF_INET6);
  int addrlen = (domain == PF_INET) ?
                INET_ADDRSTRLEN : INET6_ADDRSTRLEN;
  char remote[INET6_ADDRSTRLEN];
  char local[INET6_ADDRSTRLEN];
  FAR void *laddr;
  FAR void *raddr;
  size_t len;

  laddr = net_ip_binding_laddr(&conn->u, domain);
  raddr = net_ip_binding_raddr(&conn->u, domain);

  len = snprintf(buffer, buflen,
                 "    %2u"
                 ": %02" PRIx8
                 " %3" PRIx8 " %3" PRIu8
                 " %3" PRIu8
                 " %4" PRIu32
                 " %3" PRIu8
#if CONFIG_NET_SEND_BUFSIZE > 0 && defined(CONFIG_NET_TCP_WRITE_BUFFERS)
                 " %6" PRIu32
#endif
                 " %6u",
                 priv->lines,
                 conn->tcpstateflags,
                 conn->sconn.s_flags,
                 conn->crefs,
                 conn->timer,
                 (uint32_t)conn->tx_unacked,
                 conn->nrtx,
#if CONFIG_NET_SEND_BUFSIZE > 0 && defined(CONFIG_NET_TCP_WRITE_BUFFERS)
                 tcp_wrbuffer_inqueue_size(conn),
#endif
                 (conn->readahead) ? conn->readahead->io_pktlen : 0);

  if (len < buflen)
    {
      len += snprintf(buffer + len, buflen - len,
                      " %*s:%-6" PRIu16 " %*s:%-6" PRIu16 "\n",
                      (domain == PF_INET6) ? addrlen / 2 : addrlen,
                      inet_ntop(domain, laddr, local, addrlen),
                      ntohs(conn->lport),
                      (domain == PF_INET6) ? addrlen / 2 : addrlen,
                      inet_ntop(domain, raddr, remote, addrlen),
                      ntohs(conn->rport));
    }

  /* snprintf() returns the length of the whole line, also when it does
   * not fit.
   */

  priv->linesize = len < buflen ? len : buflen - 1;
  priv->offset   = 0;
}

/****************************************************************************
 * Name: netprocfs_tcpnext
 *
 * Description:
 *   Return the first connection after conn (or from the head of the list
 *   if conn is NULL) that this read lists: its list id is after the one of
 *   the last connection written and not after priv->limit.  NULL if there
 *   is none.
 *
 * Assumptions:
 *   The TCP connection list is locked.
 *
 ****************************************************************************/

static FAR struct tcp_conn_s *
netprocfs_tcpnext(FAR struct netprocfs_file_s *priv,
                  FAR struct tcp_conn_s *conn)
{
  while ((conn = tcp_nextconn(conn)) != NULL)
    {
      if (TCP_LISTID_AFTER(conn->listid, priv->last) &&
          !TCP_LISTID_AFTER(conn->listid, priv->limit))
        {
          break;
        }
    }

  return conn;
}

/****************************************************************************
 * Name: netprocfs_tcpstats
 *
 * Description:
 *   Write the lines of the connections after the last one written.
 *
 *   A read of the file to its end lists once each connection that is in
 *   the list from the first read to the end.  The walk finds its place by
 *   the list id of the last connection written (priv->last), not by a
 *   count and never through a pointer kept while the list was unlocked:
 *   the list ids grow from the head to the tail, so a connection that goes
 *   away does not move the place.  A connection that comes after the first
 *   read has a list id after priv->limit and is not listed, so the read
 *   ends.
 *
 * Assumptions:
 *   The TCP connection list is locked; it may be unlocked and locked again
 *   while a device lock is taken.
 *
 ****************************************************************************/

static ssize_t netprocfs_tcpstats(FAR struct netprocfs_file_s *priv,
                                  FAR char *buffer, size_t buflen)
{
  FAR struct net_driver_s *held = NULL;
  FAR struct net_driver_s *dev;
  FAR struct tcp_conn_s *conn;
  size_t len = 0;

  /* A line is written in priv->line[] and as much of it as fits is
   * transferred.  If a part of it is left, the buffer is full: the next
   * read takes that part first.
   */

  conn = netprocfs_tcpnext(priv, NULL);
  while (conn != NULL && len < buflen)
    {
      /* The input path changes the read-ahead and the write queues of a
       * connection with only its device locked, and it locks the
       * connection list under the device lock.  So a device lock is only
       * tried under the list lock, and it is kept for the next lines of
       * the same device.  If another thread holds it, the list is
       * unlocked, the device locked and the list locked again, and the
       * walk finds its place again by the list id.  That ends: the device
       * of a connection in the list does not change, so after the wait the
       * line is written, or the connection has gone.
       */

      dev = conn->dev;
      if (dev != NULL && dev != held)
        {
          if (held != NULL)
            {
              netdev_unlock(held);
              held = NULL;
            }

          if (netdev_trylock(dev) != OK)
            {
              tcp_conn_list_unlock();
              netdev_lock(dev);
              tcp_conn_list_lock();
              held = dev;

              conn = netprocfs_tcpnext(priv, NULL);
              continue;
            }

          held = dev;
        }

      netprocfs_tcpline(priv, conn);
      len += netprocfs_read_line(priv, buffer + len, buflen - len);
      priv->last = conn->listid;
      priv->lines++;
      conn = netprocfs_tcpnext(priv, conn);
    }

  if (held != NULL)
    {
      netdev_unlock(held);
    }

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
  FAR struct tcp_conn_s *conn;
  size_t len;

  /* First the rest of a line that did not fit in the buffer of the last
   * read.
   */

  len = netprocfs_read_line(priv, buffer, buflen);

  tcp_conn_list_lock();

  conn = tcp_nextconn(NULL);
  if (priv->lines == 0 && conn != NULL)
    {
      /* The first read: the header, and the bounds of this read, the
       * connections from the head of the list to the last one now.
       */

      priv->last  = conn->listid - 1;
      priv->limit = tcp_conn_lastid();

      len = snprintf(priv->line, NET_LINELEN, "TCP sl  "
                                     "st flg ref tmr uack nrt   "
#if CONFIG_NET_SEND_BUFSIZE > 0 && defined(CONFIG_NET_TCP_WRITE_BUFFERS)
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
      priv->linesize = len < NET_LINELEN ? len : NET_LINELEN - 1;
      priv->offset   = 0;
      priv->lines    = 1;

      len = netprocfs_read_line(priv, buffer, buflen);
    }

  if (priv->lines > 0 && priv->linesize == 0)
    {
      len += netprocfs_tcpstats(priv, buffer + len, buflen - len);
    }

  tcp_conn_list_unlock();

  return len;
}

#endif /* NET_TCP_HAVE_STACK */
