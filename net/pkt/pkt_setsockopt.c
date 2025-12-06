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

#include <nuttx/net/net.h>
#include <nuttx/net/pkt.h>

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

  if (psock->s_type != SOCK_RAW)
    {
      nerr("ERROR:  Not a RAW PKT socket\n");
      return -ENOTCONN;
    }

  switch (option)
    {
#if CONFIG_NET_SEND_BUFSIZE > 0
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

      default:
        nerr("ERROR: Unrecognized PKT option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET_PKTPROTO_OPTIONS */
