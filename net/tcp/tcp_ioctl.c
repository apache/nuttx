/****************************************************************************
 * net/tcp/tcp_ioctl.c
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

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <net/if.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>

#include "tcp/tcp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_path
 *
 * Description:
 *   This function generates tcp status as path.
 *
 * Parameters:
 *   conn     The TCP connection of interest
 *   buf      The buffer to get the path
 *   len      The length of the buffer
 *
 ****************************************************************************/

static void tcp_path(FAR struct tcp_conn_s *conn, FAR char *buf, size_t len)
{
  uint8_t domain = net_ip_domain_select(conn->domain, PF_INET, PF_INET6);
  char remote[INET6_ADDRSTRLEN];
  char local[INET6_ADDRSTRLEN];
  FAR void *laddr = net_ip_binding_laddr(&conn->u, domain);
  FAR void *raddr = net_ip_binding_raddr(&conn->u, domain);

  snprintf(buf, len, "tcp:["
           "%s:%" PRIu16 "<->%s:%" PRIu16
#if CONFIG_NET_SEND_BUFSIZE > 0
           ", tx %" PRIu32 "/%" PRId32
#endif
#if CONFIG_NET_RECV_BUFSIZE > 0
           ", rx %u/%" PRId32
#  ifdef CONFIG_NET_TCP_OUT_OF_ORDER
           " + ofo %d"
#  endif
#endif
           ", st %02" PRIx8
           ", flg %" PRIx8
           "]",
           inet_ntop(domain, laddr, local, sizeof(local)),
           ntohs(conn->lport),
           inet_ntop(domain, raddr, remote, sizeof(remote)),
           ntohs(conn->rport),
#if CONFIG_NET_SEND_BUFSIZE > 0
           tcp_wrbuffer_inqueue_size(conn),
           conn->snd_bufs,
#endif
#if CONFIG_NET_RECV_BUFSIZE > 0
           (conn->readahead) ? conn->readahead->io_pktlen : 0,
           conn->rcv_bufs,
#  ifdef CONFIG_NET_TCP_OUT_OF_ORDER
           tcp_ofoseg_bufsize(conn),
#  endif
#endif
           conn->tcpstateflags,
           conn->sconn.s_flags
           );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_ioctl
 *
 * Description:
 *   This function performs tcp specific ioctl() operations.
 *
 * Parameters:
 *   conn     The TCP connection of interest
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *   arglen   The length of 'arg'
 *
 ****************************************************************************/

int tcp_ioctl(FAR struct tcp_conn_s *conn, int cmd, unsigned long arg)
{
  int ret = OK;

  net_lock();

  switch (cmd)
    {
      case FIONREAD:
        if (conn->readahead != NULL)
          {
            *(FAR int *)((uintptr_t)arg) = conn->readahead->io_pktlen;
          }
        else
          {
            *(FAR int *)((uintptr_t)arg) = 0;
          }
        break;
      case FIONSPACE:
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
#  if CONFIG_NET_SEND_BUFSIZE == 0
        *(FAR int *)((uintptr_t)arg) =
                                iob_navail(true) * CONFIG_IOB_BUFSIZE;
#  else
        *(FAR int *)((uintptr_t)arg) =
                        conn->snd_bufs - tcp_wrbuffer_inqueue_size(conn);
#  endif
#else
        *(FAR int *)((uintptr_t)arg) = MIN_TCP_MSS;
#endif
        break;
      case FIOC_FILEPATH:
        tcp_path(conn, (FAR char *)(uintptr_t)arg, PATH_MAX);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  net_unlock();

  return ret;
}
