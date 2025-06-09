/****************************************************************************
 * net/pkt/pkt_setsockopt.c
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

#include <sys/time.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <netpacket/packet.h>
#include <nuttx/net/net.h>
#include <nuttx/net/pkt.h>

#include "netdev/netdev.h"
#include "socket/socket.h"
#include "utils/utils.h"
#include "pkt/pkt.h"

#ifdef CONFIG_NET_PKTPROTO_OPTIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_setsockopt
 *
 * Description:
 *   pkt_setsockopt() sets the PKT-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <nuttx/pkt.h> for the a complete list of values of PKT protocol
 *   options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_setcockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

int pkt_setsockopt(FAR struct socket *psock, int level, int option,
                   FAR const void *value, socklen_t value_len)
{
  int ret = OK;

  DEBUGASSERT(value_len == 0 || value != NULL);

  if (level != SOL_PACKET)
    {
      return -ENOPROTOOPT;
    }

  if (psock->s_type != SOCK_DGRAM && psock->s_type != SOCK_RAW)
    {
      nerr("ERROR:  Not a valid PKT socket\n");
      return -ENOTCONN;
    }

  switch (option)
    {
#if defined(CONFIG_NET_PKT_WRITE_BUFFERS) && CONFIG_NET_SEND_BUFSIZE > 0
      case SO_SNDBUF:
        {
          FAR struct pkt_conn_s *conn;
          conn = psock->s_conn;
          int buffersize;

          /* Verify options */

          if (value_len != sizeof(int))
            {
              return -EINVAL;
            }

          buffersize = *(FAR int *)value;
          if (buffersize < 0)
            {
              return -EINVAL;
            }

#   if CONFIG_NET_MAX_SEND_BUFSIZE > 0
          /* Limit the size of the send buffer */

          buffersize = MIN(buffersize, CONFIG_NET_MAX_SEND_BUFSIZE);
#   endif

          conn->sndbufs = buffersize;
          break;
        }
#endif

#ifdef CONFIG_NET_MCASTGROUP
      case PACKET_ADD_MEMBERSHIP:
      case PACKET_DROP_MEMBERSHIP:
        {
          FAR const struct packet_mreq *mreq;
          FAR struct net_driver_s *dev;

          if (value == NULL || value_len < sizeof(struct packet_mreq))
            {
              return -EINVAL;
            }

          mreq = (FAR const struct packet_mreq *)value;
          dev = netdev_findbyindex(mreq->mr_ifindex);
          if (dev == NULL)
            {
              return -ENODEV;
            }

          if (mreq->mr_type == PACKET_MR_MULTICAST)
            {
              if (option == PACKET_ADD_MEMBERSHIP && dev->d_addmac != NULL)
                {
                  /* Add the multicast MAC address to the device */

                  ret = dev->d_addmac(dev, mreq->mr_address);
                }
              else if (option == PACKET_DROP_MEMBERSHIP &&
                       dev->d_rmmac != NULL)
                {
                  /* Drop the multicast MAC address from the device */

                  ret = dev->d_rmmac(dev, mreq->mr_address);
                }
              else
                {
                  nerr("ERROR: Device does not support add MAC address\n");
                  ret = -ENOSYS;
                }
            }
          else
            {
              nerr("ERROR: Invalid mr_type: %d\n", mreq->mr_type);
              return -ENOSYS;
            }
        }
        break;
#endif

      default:
        nerr("ERROR: Unrecognized PKT option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET_PKTPROTO_OPTIONS */
