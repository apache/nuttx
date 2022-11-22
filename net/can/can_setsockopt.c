/****************************************************************************
 * net/can/can_setsockopt.c
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

#include <netpacket/can.h>

#include <nuttx/net/net.h>
#include <nuttx/net/can.h>

#include "socket/socket.h"
#include "utils/utils.h"
#include "can/can.h"

#ifdef CONFIG_NET_CANPROTO_OPTIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_setsockopt
 *
 * Description:
 *   can_setsockopt() sets the CAN-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/can.h> for the a complete list of values of CAN protocol
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

int can_setsockopt(FAR struct socket *psock, int level, int option,
                   FAR const void *value, socklen_t value_len)
{
  FAR struct can_conn_s *conn;
  int ret = OK;
  int count = 0;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  DEBUGASSERT(value_len == 0 || value != NULL);

  conn = (FAR struct can_conn_s *)psock->s_conn;

#ifdef CONFIG_NET_TIMESTAMP

  /* Generates a timestamp for each incoming packet */

  if (level == SOL_SOCKET && option == SO_TIMESTAMP)
    {
      /* Verify that option is at least the size of an integer. */

      if (value_len < sizeof(int32_t))
        {
          return -EINVAL;
        }

      /* Lock the network so that we have exclusive access to the socket
       * options.
       */

      net_lock();

      conn->timestamp = *((FAR int32_t *)value);

      net_unlock();
      return OK;
    }
#endif

  if (level != SOL_CAN_RAW)
    {
      return -ENOPROTOOPT;
    }

  if (psock->s_type != SOCK_RAW)
    {
      nerr("ERROR:  Not a RAW CAN socket\n");
      return -ENOTCONN;
    }

  switch (option)
    {
      case CAN_RAW_FILTER:
        if (value_len == 0)
          {
            conn->filter_count = 0;
            ret = OK;
          }
        else if (value_len % sizeof(struct can_filter) != 0)
          {
            ret = -EINVAL;
          }
        else if (value_len > CONFIG_NET_CAN_RAW_FILTER_MAX *
                   sizeof(struct can_filter))
          {
            ret = -EINVAL;
          }
        else
          {
        count = value_len / sizeof(struct can_filter);

        for (int i = 0; i < count; i++)
          {
        conn->filters[i] = ((struct can_filter *)value)[i];
          }

        conn->filter_count = count;

            ret = OK;
          }
        break;

      case CAN_RAW_ERR_FILTER:
        break;

      case CAN_RAW_LOOPBACK:
        if (value_len != sizeof(conn->loopback))
          {
            return -EINVAL;
          }

        conn->loopback = *(FAR int32_t *)value;

        break;

      case CAN_RAW_RECV_OWN_MSGS:
        if (value_len != sizeof(conn->recv_own_msgs))
          {
            return -EINVAL;
          }

        conn->recv_own_msgs = *(FAR int32_t *)value;

        break;

#ifdef CONFIG_NET_CAN_CANFD
      case CAN_RAW_FD_FRAMES:
        if (value_len != sizeof(conn->fd_frames))
          {
            return -EINVAL;
          }

        conn->fd_frames = *(FAR int32_t *)value;

        break;
#endif

      case CAN_RAW_JOIN_FILTERS:
        break;

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
      case CAN_RAW_TX_DEADLINE:
        if (value_len != sizeof(conn->tx_deadline))
          {
            return -EINVAL;
          }

        conn->tx_deadline = *(FAR int32_t *)value;

        break;
#endif

      default:
        nerr("ERROR: Unrecognized CAN option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET_CANPROTO_OPTIONS */
