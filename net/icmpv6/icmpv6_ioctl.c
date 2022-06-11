/****************************************************************************
 * net/icmpv6/icmpv6_ioctl.c
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
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <net/if.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>

#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_ioctl
 *
 * Description:
 *   This function performs icmpv6 specific ioctl() operations.
 *
 * Parameters:
 *   conn     The icmpv6 connection of interest
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *   arglen   The length of 'arg'
 *
 ****************************************************************************/

int icmpv6_ioctl(FAR struct socket *psock,
               int cmd, FAR void *arg, size_t arglen)
{
  FAR struct icmpv6_conn_s *conn = psock->s_conn;
  int ret = OK;

  net_lock();

  switch (cmd)
    {
      case FIONREAD:
        if (iob_peek_queue(&conn->readahead) != NULL)
          {
            *(FAR int *)((uintptr_t)arg) =
                          iob_peek_queue(&conn->readahead)->io_pktlen;
          }
        else
          {
            *(FAR int *)((uintptr_t)arg) = 0;
          }
        break;
      case FIONSPACE:
        *(FAR int *)((uintptr_t)arg) = MIN_UDP_MSS;
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  net_unlock();

  return ret;
}
