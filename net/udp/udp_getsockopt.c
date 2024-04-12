/****************************************************************************
 * net/udp/udp_getsockopt.c
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

#include <netinet/udp.h>

#include <nuttx/net/net.h>
#include <nuttx/net/udp.h>

#include "socket/socket.h"
#include "utils/utils.h"
#include "udp/udp.h"

#ifdef CONFIG_NET_UDPPROTO_OPTIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_getsockopt
 *
 * Description:
 *   udp_getsockopt() retrieves the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the 'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET; to retrieve options at the UDP-protocol level, the level
 *   argument is SOL_UDP.
 *
 *   See <sys/socket.h> a complete list of values for the socket-level
 *   'option' argument.  Protocol-specific options are are protocol specific
 *   header files (such as netinet/udp.h for the case of the UDP protocol).
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_getsockopt() for
 *   the complete list of appropriate return error codes.
 *
 ****************************************************************************/

int udp_getsockopt(FAR struct socket *psock, int option,
                   FAR void *value, FAR socklen_t *value_len)
{
  FAR struct udp_conn_s *conn;
  int ret;

  DEBUGASSERT(value != NULL && value_len != NULL);
  conn = psock->s_conn;

  /* All of the UDP protocol options apply only UDP sockets. */

  if (psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR:  Not a UDP socket\n");
      return -ENOTCONN;
    }

  /* Handle the UDP option */

  switch (option)
    {
#ifdef CONFIG_NET_UDP_OFFLOAD
      case UDP_SEGMENT:
        if (*value_len < sizeof(uint16_t))
          {
            return -EINVAL;
          }
        else
          {
            FAR uint16_t *gso_size = (FAR uint16_t *)value;
            *gso_size              = conn->gso_size;
            *value_len             = sizeof(uint16_t);
            ret                    = OK;
          }
        break;
      case UDP_GRO:
        if (*value_len < sizeof(int))
          {
            return -EINVAL;
          }
        else
          {
            FAR int *gro_enable = (FAR int *)value;
            *gro_enable         = conn->gro_enable;
            *value_len          = sizeof(int);
            ret                 = OK;
          }
        break;
#endif
      default:
        nerr("ERROR: Unrecognized UDP option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET_UDPPROTO_OPTIONS */
