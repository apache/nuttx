/****************************************************************************
 * net/udp/udp_ioctl.c
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

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <net/if.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>

#include "udp/udp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_path
 *
 * Description:
 *   This function generates udp status as path.
 *
 * Parameters:
 *   conn     The UDP connection of interest
 *   buf      The buffer to get the path
 *   len      The length of the buffer
 *
 ****************************************************************************/

static void udp_path(FAR struct udp_conn_s *conn, FAR char *buf, size_t len)
{
  uint8_t domain = net_ip_domain_select(conn->domain, PF_INET, PF_INET6);
  char remote[INET6_ADDRSTRLEN];
  char local[INET6_ADDRSTRLEN];
  FAR void *laddr = net_ip_binding_laddr(&conn->u, domain);
  FAR void *raddr = net_ip_binding_raddr(&conn->u, domain);

  snprintf(buf, len, "udp:["
           "%s:%" PRIu16 "<->%s:%" PRIu16
#if CONFIG_NET_SEND_BUFSIZE > 0
           ", tx %" PRIu32 "/%" PRId32
#endif
#if CONFIG_NET_RECV_BUFSIZE > 0
           ", rx %u/%" PRId32
#endif
           ", flg %" PRIx8
           "]",
           inet_ntop(domain, laddr, local, sizeof(local)),
           ntohs(conn->lport),
           inet_ntop(domain, raddr, remote, sizeof(remote)),
           ntohs(conn->rport),
#if CONFIG_NET_SEND_BUFSIZE > 0
           udp_wrbuffer_inqueue_size(conn),
           conn->sndbufs,
#endif
#if CONFIG_NET_RECV_BUFSIZE > 0
           (conn->readahead) ? conn->readahead->io_pktlen : 0,
           conn->rcvbufs,
#endif
           conn->sconn.s_flags
           );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_ioctl
 *
 * Description:
 *   This function performs udp specific ioctl() operations.
 *
 * Parameters:
 *   conn     The UDP connection of interest
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 ****************************************************************************/

int udp_ioctl(FAR struct udp_conn_s *conn, int cmd, unsigned long arg)
{
  FAR struct iob_s *iob;
  int ret = OK;

  net_lock();

  switch (cmd)
    {
      case FIONREAD:
        iob = conn->readahead;
        if (iob)
          {
            uint16_t datalen;
            iob_copyout((FAR uint8_t *)&datalen, iob, sizeof(datalen), 0);
            *(FAR int *)((uintptr_t)arg) = datalen;
          }
        else
          {
            *(FAR int *)((uintptr_t)arg) = 0;
          }
        break;
      case FIONSPACE:
#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
#  if CONFIG_NET_SEND_BUFSIZE == 0
        *(FAR int *)((uintptr_t)arg) =
                                iob_navail(true) * CONFIG_IOB_BUFSIZE;
#  else
        *(FAR int *)((uintptr_t)arg) =
                        conn->sndbufs - udp_wrbuffer_inqueue_size(conn);
#  endif
#else
        *(FAR int *)((uintptr_t)arg) = MIN_UDP_MSS;
#endif
        break;
      case FIOC_FILEPATH:
        udp_path(conn, (FAR char *)(uintptr_t)arg, PATH_MAX);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  net_unlock();

  return ret;
}
